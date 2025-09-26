#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

import gymnasium as gym
from gymnasium import spaces
import numpy as np

import time
import json
from typing import Tuple, Dict, Any

from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Float32MultiArray, String
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from people_msgs.msg import People
from gazebo_msgs.srv import SetEntityState
from gazebo_msgs.msg import EntityState
from geometry_msgs.msg import Point, Quaternion
from typing import Optional

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
        self.num_observation_types = 4  # Agent distances, obstacle distances, group IDs, goal distances
        self.fused_object_array_size = self.num_rays * self.num_observation_types # 960
        self.max_episode_steps = 1000
        self.current_step = 0
        self.robot_name = robot_name
        self.xy_limit = 5.0

        # Action repeat so actions can have more significance
        self.action_repeat = 3
        
        # Action space: [linear_x, linear_y, angular_z] (holonomic)
        # from gait.yaml: x [-0.5, 0.8], y [-0.6, 0.6], z [-0.8, 0.8]
        self.action_space = spaces.Box(
            low=np.array([-0.5, -0.6, -0.8]),   
            high=np.array([0.8, 0.6, 0.8]),  
            dtype=np.float32
        )
        
        # Observation space: normalised data from 4*240 rays + x,y coordinates
        # Fused array values: -1 (no detection) or [0,1] (normalised distances/group_ids)
        self.observation_space = spaces.Box(
            low=np.concatenate([np.full(self.fused_object_array_size, -1.0), np.array([-self.xy_limit, -self.xy_limit])]),  # fused array + position bounds
            high=np.concatenate([np.ones(self.fused_object_array_size), np.array([self.xy_limit, self.xy_limit])]),    # fused array + position bounds
            shape=(self.fused_object_array_size + 2,),
            dtype=np.float32
        )
        
        # State variables
        self.latest_observation = np.zeros(self.fused_object_array_size, dtype=np.float32)  
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
        self.get_logger().info(f"Fused object array size: {self.fused_object_array_size} (4 * {self.num_rays} rays)")
        self.get_logger().info(f"Total observation dimensions: {self.fused_object_array_size + 2} (fused array + position)")
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
                goal_distances = raw_fused_data[3*self.num_rays:4*self.num_rays]
                
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
                
                normalised_goals = np.where(
                    goal_distances < 0,
                    -1.0,
                    np.clip(goal_distances / self.xy_limit, 0.0, 1.0)
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
                    normalised_group_ids,
                    normalised_goals
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

        time.sleep(0.5)  # Wait for robot to stop

        # Teleport robot to origin at random yaw
        yaw = np.random.uniform(-np.pi, np.pi)
        self.teleport_robot_to_origin(x=0.0, y=0.0, z=0.25, yaw=yaw)
        self.publish_random_goal()

        time.sleep(1.5)
        
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
        # Get normalised robot position
        if self.robot_pose is not None:
            # Normalise position to [-1, 1] range 
            x_norm = np.clip(self.robot_pose.position.x / self.xy_limit, -1.0, 1.0)
            y_norm = np.clip(self.robot_pose.position.y / self.xy_limit, -1.0, 1.0)
            position = np.array([x_norm, y_norm], dtype=np.float32)
        else:
            position = np.array([0.0, 0.0], dtype=np.float32)
        
        # Combine fused array data (960 values) with position (2 values)
        full_observation = np.concatenate([self.latest_observation, position])
        return full_observation
    

    def publish_random_goal(self):
        min_distance = 2.0 
        distance_from_origin = 0.0
        
        # Keep generating until we find a valid goal
        while distance_from_origin < min_distance:
            goal_x = np.random.uniform(self.goal_bounds[0], self.goal_bounds[1])
            goal_y = np.random.uniform(self.goal_bounds[0], self.goal_bounds[1])
        
            distance_from_origin = np.sqrt(goal_x**2 + goal_y**2)
        
        self.current_goal_position = np.array([goal_x, goal_y])
        
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
        self.get_logger().info(f"Published random goal at ({goal_x:.2f}, {goal_y:.2f})")
        return self.current_goal_position
    

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
