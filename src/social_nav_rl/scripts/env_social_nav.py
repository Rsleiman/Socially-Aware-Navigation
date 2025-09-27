#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

import gymnasium as gym
from gymnasium import spaces
import numpy as np

import time
import json
from typing import Tuple, Dict, Any, Optional

from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Float32MultiArray, String
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from people_msgs.msg import People
from gazebo_msgs.srv import SetEntityState
from gazebo_msgs.msg import EntityState
from geometry_msgs.msg import Point, Quaternion

from social_reward_calculator import SocialRewardCalculator


class SocialNavEnvironment(gym.Env, Node):
    """
    A2C Social Navigation Environment for Unitree Go2
    #TODO: Create holonomic vs non-holonomic action space
    Observation Space: Fused laser-like data from multiple sensors
    Action Space: Continuous control commands for robot movement
    """
    
    def __init__(self, node_name='social_nav_env', robot_name='go2'):
        gym.Env.__init__(self)
        Node.__init__(self, node_name)
        
        # Environment parameters
        self.num_rays = 240  
        self.num_observation_types = 3  # Agent distances, obstacle distances, group IDs
        self.fused_object_array_size = self.num_rays * self.num_observation_types  # 720
        self.max_episode_steps = 1000
        self.current_step = 0
        self.robot_name = robot_name
        self.xy_limit = 5.0
        self.goal_feature_dim = 4  # relative_x, relative_y, distance, heading
        self.position_dim = 2  # global x, y (normalised)
        self.additional_feature_dim = self.goal_feature_dim + self.position_dim
        environment_span = 2 * self.xy_limit
        environment_diagonal = np.sqrt(2) * environment_span
        self.max_goal_distance = max(environment_diagonal, 1e-3)
        self.goal_relative_limit = max(environment_span, 1e-3)

        # Action repeat so actions can have more significance
        self.action_repeat = 3
        
        # Action space: [linear_x, linear_y, angular_z] (holonomic)
        # from gait.yaml: x [-0.5, 0.8], y [-0.6, 0.6], z [-0.8, 0.8]
        self.action_space = spaces.Box(
            low=np.array([-0.5, -0.6, -0.8]),   
            high=np.array([0.8, 0.6, 0.8]),  
            dtype=np.float32
        )
        
        # Observation space: normalised fused data + goal-relative features + global position
        fused_low = np.full(self.fused_object_array_size, -1.0, dtype=np.float32)
        fused_high = np.ones(self.fused_object_array_size, dtype=np.float32)
        goal_low = np.array([-1.0, -1.0, 0.0, -1.0], dtype=np.float32)
        goal_high = np.ones(self.goal_feature_dim, dtype=np.float32)
        position_low = np.array([-1.0, -1.0], dtype=np.float32)
        position_high = np.array([1.0, 1.0], dtype=np.float32)

        self.observation_space = spaces.Box(
            low=np.concatenate([fused_low, goal_low, position_low]),
            high=np.concatenate([fused_high, goal_high, position_high]),
            shape=(self.fused_object_array_size + self.additional_feature_dim,),
            dtype=np.float32
        )
        
        # State variables
        self.latest_observation = np.zeros(self.fused_object_array_size, dtype=np.float32)  
        self.latest_goal_features = np.zeros(self.goal_feature_dim, dtype=np.float32)
        self.robot_pose = None
        self.robot_orientation = None  
        
        # Goal tracking
        self.current_goal_position = None 
        self.goal_bounds = [-self.xy_limit, self.xy_limit]

        # New data for social reward calculation
        self.latest_people_data = None
        self.agent_name_mapping = {}
        self.latest_scan_data = None
        
        # Initialise social reward calculator
        self.reward_calculator = SocialRewardCalculator(parent_node=self)

        # qos profiles
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # profile for agent mapping
        agent_mapping_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Publishers #
        # Smoothened velocity pub
        self.rl_vel_pub = self.create_publisher(
            Twist, 
            '/rl_vel',
            10
        )
        # Goal pose for reset
        self.goal_pose_pub = self.create_publisher(
            PoseStamped,
            '/goal_pose',
            10
        )
        
        # Subscribers #
        # Fused array for laser-like data
        self.observation_sub = self.create_subscription(
            Float32MultiArray,
            '/social_observation/fused_object_array',
            self.observation_callback,
            qos_profile
        )
        # GT Odometry
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom/ground_truth',
            self.odom_callback,
            qos_profile
        )
        # People data for social reward
        self.people_sub = self.create_subscription(
            People,
            '/people',
            self.people_callback,
            qos_profile
        )
        # Agent name mapping for group identification
        self.agent_mapping_sub = self.create_subscription(
            String,
            '/social_observation/agent_name_to_data',
            self.agent_mapping_callback,
            agent_mapping_qos
        )
        # Laser scan for obstacle detection
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile
        )
        
        # Service client for robot teleportation
        self.teleport_client = self.create_client(SetEntityState, '/gazebo/set_entity_state')
        
        # Initialise
        self.get_logger().info("Social Navigation Environment initialised")
        self.get_logger().info(f"Action space: {self.action_space}")
        self.get_logger().info(f"Observation space (low, high, shape, dtype): {self.observation_space}")
        self.get_logger().info(f"Fused object array size: {self.fused_object_array_size} (3 * {self.num_rays} rays)")
        self.get_logger().info(
            f"Total observation dimensions: {self.fused_object_array_size + self.additional_feature_dim} "
            f"(fused array + goal features + position)"
        )
        self.get_logger().info(f"Social reward calculator initialised")
        
    # CALLBACKS #
    def observation_callback(self, msg):
        """Process incoming fused object array data"""
        try:
            data = np.array(msg.data, dtype=np.float32)
            
            if data is not None and len(data) >= self.fused_object_array_size:
                raw_fused_data = data[:self.fused_object_array_size]
                
                # Process each section separately for proper normalization
                agent_distances = raw_fused_data[:self.num_rays]
                obstacle_distances = raw_fused_data[self.num_rays:2*self.num_rays]
                group_ids = raw_fused_data[2*self.num_rays:3*self.num_rays]
                
                # Normalised: -1 (no detection) -> -1, otherwise normalise by max range
                normalised_agents = np.where(
                    agent_distances < 0,
                    -1.0,
                    np.clip(agent_distances / self.xy_limit, 0.0, 1.0)
                )
                
                normalised_obstacles = np.where(
                    obstacle_distances < 0,
                    -1.0,
                    np.clip(obstacle_distances / self.xy_limit, 0.0, 1.0)
                )
                
                max_groups = 10  # TODO: Make this a parameter
                normalised_group_ids = np.where(
                    group_ids < 0,
                    -1.0,
                    np.clip(group_ids / max_groups, 0.0, 1.0)
                )
                
                self.latest_observation = np.concatenate([
                    normalised_agents,
                    normalised_obstacles,
                    normalised_group_ids
                ])
                
            else:
                self.get_logger().warn(f"Fused array size mismatch: expected {self.fused_object_array_size}, got {len(data) if data is not None else 0}")
                
        except Exception as e:
            self.get_logger().warn(f"Error processing observation: {e}")
            
    def odom_callback(self, msg):
        self.robot_pose = msg.pose.pose
        self.robot_orientation = msg.pose.pose.orientation
        
    def people_callback(self, msg):
        self.latest_people_data = msg
        
    def agent_mapping_callback(self, msg):
        try:
            self.agent_name_mapping = json.loads(msg.data)
        except Exception as e:
            self.get_logger().warn(f"Error parsing agent mapping: {e}")
            
    def scan_callback(self, msg):
        self.latest_scan_data = msg


    # ENVIRONMENT METHODS #  
    def step(self, action: np.ndarray) -> Tuple[np.ndarray, float, bool, bool, Dict[str, Any]]:
        # Ensure action is within bounds
        action = np.clip(action, self.action_space.low, self.action_space.high)

        cmd = Twist()
        cmd.linear.x = float(action[0])
        cmd.linear.y = float(action[1])
        cmd.angular.z = float(action[2])

        # Action repeat: execute same action multiple times and accumulate rewards
        total_reward = 0.0
        terminated = False
        truncated = False
        final_termination_flags = {}

        for repeat_step in range(self.action_repeat):
            self.rl_vel_pub.publish(cmd)

            rclpy.spin_once(self, timeout_sec=0.2) 
            time.sleep(0.1) 

            step_reward, reward_components, termination_flags = self.calculate_reward_and_termination_conditions()
            total_reward += step_reward
            final_termination_flags = termination_flags

            # Check if any termination condition is met
            terminated = any(termination_flags.values())

            if terminated:
                break

        self.current_step += 1
        truncated = self.current_step >= self.max_episode_steps

        # Get final observation after all action repeats
        full_observation = self.get_full_observation()

        info = {
            'step': self.current_step,
            'reward_components': reward_components,
            'termination_flags': final_termination_flags,
            'action': action.tolist(),
            'action_repeats_executed': repeat_step + 1  # How many repeats were actually executed
        }

        return full_observation, total_reward, terminated, truncated, info

    def reset(self, seed=None, options=None) -> Tuple[np.ndarray, Dict[str, Any]]:
        super().reset(seed=seed)

        cmd = Twist()
        self.rl_vel_pub.publish(cmd)

        # Reset
        self.current_step = 0
        self.reward_calculator.reset()
        self.latest_observation = np.zeros(self.fused_object_array_size, dtype=np.float32)
        self.latest_goal_features = np.zeros(self.goal_feature_dim, dtype=np.float32)
        self.robot_pose = None
        self.robot_orientation = None
        self.current_goal_position = None
        self.latest_people_data = None
        self.latest_scan_data = None

        time.sleep(0.5)  # Wait for robot to stop

        # Teleport robot to origin at random yaw
        yaw = np.random.uniform(-np.pi, np.pi)
        self.teleport_robot_to_origin(x=0.0, y=0.0, z=0.25, yaw=yaw)

        time.sleep(1.5)

        # wait for odom and laserscan to update before publishing goal
        while self.robot_pose is None or self.latest_scan_data is None:
            rclpy.spin_once(self, timeout_sec=0.1)

        self.publish_random_goal()

        
        # Ensure fresh sensor data
        for _ in range(5):
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.1)

        observation = self.get_full_observation()
        info = {'step': 0}

        self.get_logger().info(f"Environment reset, episode starting at origin")

        return observation, info


    # HELPERS #
    def get_full_observation(self) -> np.ndarray:
        goal_features = self.compute_goal_features()

        if self.robot_pose is not None:
            x_norm = np.clip(self.robot_pose.position.x / self.xy_limit, -1.0, 1.0)
            y_norm = np.clip(self.robot_pose.position.y / self.xy_limit, -1.0, 1.0)
            robot_position = np.array([x_norm, y_norm], dtype=np.float32)
        else:
            robot_position = np.array([0.0, 0.0], dtype=np.float32)

        full_observation = np.concatenate(
            [self.latest_observation, goal_features, robot_position]
        )
        return full_observation

    def compute_goal_features(self) -> np.ndarray:
        if self.current_goal_position is None or self.robot_pose is None:
            self.latest_goal_features = np.zeros(self.goal_feature_dim, dtype=np.float32)
            return self.latest_goal_features

        yaw = self.get_robot_yaw()
        if yaw is None:
            self.latest_goal_features = np.zeros(self.goal_feature_dim, dtype=np.float32)
            return self.latest_goal_features

        dx_map = float(self.current_goal_position[0]) - self.robot_pose.position.x
        dy_map = float(self.current_goal_position[1]) - self.robot_pose.position.y

        cos_yaw = np.cos(yaw)
        sin_yaw = np.sin(yaw)

        rel_x_robot = cos_yaw * dx_map + sin_yaw * dy_map
        rel_y_robot = -sin_yaw * dx_map + cos_yaw * dy_map

        rel_x_norm = np.clip(rel_x_robot / self.goal_relative_limit, -1.0, 1.0)
        rel_y_norm = np.clip(rel_y_robot / self.goal_relative_limit, -1.0, 1.0)

        distance = np.sqrt(rel_x_robot**2 + rel_y_robot**2)
        distance_norm = np.clip(distance / self.max_goal_distance, 0.0, 1.0)

        heading = np.arctan2(rel_y_robot, rel_x_robot)
        heading_norm = np.clip(heading / np.pi, -1.0, 1.0)

        self.latest_goal_features = np.array(
            [rel_x_norm, rel_y_norm, distance_norm, heading_norm],
            dtype=np.float32
        )
        return self.latest_goal_features

    def get_robot_yaw(self) -> Optional[float]:
        if self.robot_orientation is None:
            return None

        q = self.robot_orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return np.arctan2(siny_cosp, cosy_cosp)
    

    def publish_random_goal(self):
        """
        Goal must:
        - Be certain distance from origin
        - Be certain distance from any agent
        - Be certain distance from environment bounds (xy_limit)
        - Be certain distance from obstacle
        """
        min_distance_from_origin = 1.0 
        min_distance_from_agents = 0.2
        min_distance_from_obstacles = 0.2

        max_attempts = 100
        
        obstacle_positions = self._compute_obstacle_positions_from_scan()

        for attempt in range(max_attempts):
            goal_x = np.random.uniform(self.goal_bounds[0], self.goal_bounds[1])
            goal_y = np.random.uniform(self.goal_bounds[0], self.goal_bounds[1])

            distance_from_origin = np.sqrt(goal_x**2 + goal_y**2)
            
            # Check distance from origin
            if distance_from_origin < min_distance_from_origin:
                continue
                
            # check distance to wall
            if (abs(goal_x) > self.xy_limit - 0.4) or (abs(goal_y) > self.xy_limit - 0.4):
                continue

            # check distance to obstacles using laser scan data
            if obstacle_positions is not None and obstacle_positions.size > 0:
                goal_vector = np.array([goal_x, goal_y], dtype=np.float32)
                deltas = obstacle_positions - goal_vector
                obstacle_distances = np.linalg.norm(deltas, axis=1)

                if np.any(obstacle_distances < min_distance_from_obstacles):
                    continue
            

            # Check distance from agents using existing people data
            valid_goal = True
            if self.latest_people_data is not None:
                for person in self.latest_people_data.people:
                    agent_distance = np.sqrt(
                        (goal_x - person.position.x)**2 + 
                        (goal_y - person.position.y)**2
                    )
                    if agent_distance < min_distance_from_agents:
                        valid_goal = False
                        break
            
            if valid_goal:
                break
        else:
            # Fallback: if no valid goal found, use a safe default
            self.get_logger().warn("Could not find valid goal after maximum attempts, using fallback")
            goal_x, goal_y = 3.0, 3.0

        self.current_goal_position = np.array([goal_x, goal_y], dtype=np.float32)
        
        # Publish goal message
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = "map"
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        
        goal_pose.pose.position.x = goal_x
        goal_pose.pose.position.y = goal_y
        goal_pose.pose.position.z = 0.0
        
        goal_pose.pose.orientation.x = 0.0
        goal_pose.pose.orientation.y = 0.0
        goal_pose.pose.orientation.z = 0.0
        goal_pose.pose.orientation.w = 1.0
        
        self.goal_pose_pub.publish(goal_pose)
        
        # Enhanced logging
        if self.latest_people_data is not None:
            num_agents = len(self.latest_people_data.people)
            if num_agents > 0:
                self.get_logger().info(f"Published goal at ({goal_x:.2f}, {goal_y:.2f}) - avoided {num_agents} agents")
            else:
                self.get_logger().info(f"Published goal at ({goal_x:.2f}, {goal_y:.2f}) - no agents present")
        else:
            self.get_logger().info(f"Published goal at ({goal_x:.2f}, {goal_y:.2f}) - no people data available")
            
        return self.current_goal_position

    def _compute_obstacle_positions_from_scan(self) -> Optional[np.ndarray]:
        """Convert the latest laser scan into obstacle points in the map frame."""
        if self.latest_scan_data is None or self.robot_pose is None:
            return None

        yaw = self.get_robot_yaw()
        if yaw is None:
            return None

        scan = self.latest_scan_data

        ranges = np.array(scan.ranges, dtype=np.float32)
        valid_mask = np.isfinite(ranges)
        valid_mask &= ranges >= scan.range_min

        if np.isfinite(scan.range_max) and scan.range_max > 0:
            valid_mask &= ranges <= scan.range_max

        if not np.any(valid_mask):
            return None

        ranges = ranges[valid_mask]

        angles = scan.angle_min + np.arange(len(scan.ranges), dtype=np.float32) * scan.angle_increment
        angles = angles[valid_mask]

        x_local = ranges * np.cos(angles)
        y_local = ranges * np.sin(angles)

        cos_yaw = np.cos(yaw)
        sin_yaw = np.sin(yaw)
        robot_x = self.robot_pose.position.x
        robot_y = self.robot_pose.position.y

        x_map = robot_x + cos_yaw * x_local - sin_yaw * y_local
        y_map = robot_y + sin_yaw * x_local + cos_yaw * y_local

        return np.stack((x_map, y_map), axis=-1)
    

    def teleport_robot_to_origin(self, x=0.0, y=0.0, z=0.25, yaw=0.0):
        if not self.teleport_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn("Teleportation service not available")
            return False
        
        try:
            # Create entity state message
            entity_state = EntityState()
            entity_state.name = self.robot_name
            
            # Set position
            entity_state.pose.position = Point(x=x, y=y, z=z)
            
            entity_state.pose.orientation = Quaternion(
                x=0.0, y=0.0, 
                z=np.sin(yaw/2), w=np.cos(yaw/2)
            )
            
            # Zero velocities
            entity_state.twist.linear.x = 0.0
            entity_state.twist.linear.y = 0.0
            entity_state.twist.linear.z = 0.0
            entity_state.twist.angular.x = 0.0
            entity_state.twist.angular.y = 0.0
            entity_state.twist.angular.z = 0.0
            
            # Create request
            request = SetEntityState.Request()
            request.state = entity_state
            
            # Call service
            future = self.teleport_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
            
            if future.result() is not None and future.result().success:
                self.get_logger().info(f"Robot teleported to ({x:.2f}, {y:.2f}, {z:.2f})")
                return True
            else:
                self.get_logger().warn("Failed to teleport robot")
                return False
                
        except Exception as e:
            self.get_logger().error(f"Error during teleportation: {e}")
            return False



    # REWARD AND TERMINATION CONDITIONS #
    def calculate_reward_and_termination_conditions(self) -> Tuple[float, Dict[str, float], Dict[str, bool]]:
        """
        Calculate reward and detect termination conditions using ground truth data
        Returns:
            Tuple of (total_reward, component_rewards_dict, termination_flags_dict)
        """        
        # Prepare goal position (convert from [x,y] to Point)
        goal_point = None
        if self.current_goal_position is not None:
            goal_point = Point()
            goal_point.x = float(self.current_goal_position[0])
            goal_point.y = float(self.current_goal_position[1])
            goal_point.z = 0.0
        
        # Calculate social reward using ground truth data
        total_reward, reward_components, termination_flags = self.reward_calculator.calculate_total_reward_and_termination_conditions(
            robot_pose=self.robot_pose,
            people_data=self.latest_people_data,
            goal_position=goal_point,
            scan_data=self.latest_scan_data,
            agent_name_mapping=self.agent_name_mapping,
        )

        # Log termination conditions (only when they occur)
        if termination_flags.get('collision_obstacle', False):
            self.get_logger().info("Termination: Obstacle collision")
        if termination_flags.get('collision_human', False):
            self.get_logger().info("Termination: Human collision")
        if termination_flags.get('goal_reached', False):
            self.get_logger().info("Termination: Goal reached!")
        if termination_flags.get('robot_flipped', False):
            self.get_logger().info("Termination: Robot flipped")

        if hasattr(self, '_reward_log_counter'):
            self._reward_log_counter += 1
        else:
            self._reward_log_counter = 0
            
        # Log reward components every 100 steps or when something notable happens
        if (self._reward_log_counter % 100 == 0 or 
            any(termination_flags.values()) or
            abs(total_reward) > 2):
            
            self.get_logger().info(
                f"Rewards: "
                f"r_ro={reward_components.get('r_ro', 0):.3f}, "
                f"r_rh={reward_components.get('r_rh', 0):.3f}, "
                f"r_rs={reward_components.get('r_rs', 0):.3f}, "
                f"r_rg={reward_components.get('r_rg', 0):.3f}, "
                f"r_rf={reward_components.get('r_rf', 0):.3f}, "
                f"total={total_reward:.3f}"
            )
        
        return total_reward, reward_components, termination_flags



if __name__ == '__main__':
    #debug
    rclpy.init()
    
    env = SocialNavEnvironment()
    
    try:
        obs, info = env.reset()
        print(f"Initial observation shape: {obs.shape}")
        
        for i in range(10):
            action = env.action_space.sample()
            obs, reward, terminated, truncated, info = env.step(action)
            print(f"Step {i}: reward={reward:.3f}, terminated={terminated}")
            
            if terminated or truncated:
                obs, info = env.reset()
                
    except KeyboardInterrupt:
        print("Interrupted by user")
    finally:
        rclpy.shutdown()
