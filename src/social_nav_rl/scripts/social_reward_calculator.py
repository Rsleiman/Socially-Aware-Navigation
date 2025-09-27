#!/usr/bin/env python3

import numpy as np
import math
from typing import Dict, List, Tuple, Optional
from geometry_msgs.msg import Point, Pose, Quaternion
from people_msgs.msg import People, Person
from sensor_msgs.msg import LaserScan

# Import Shapely for convex hull-based social interaction calculations
from shapely.geometry import Point as ShapelyPoint, MultiPoint, Polygon



class SocialRewardCalculator:
    """
    Advanced social navigation reward calculator implementing four reward components:
    1. r_ro: Obstacle collision avoidance
    2. r_rh: Human distance maintenance  
    3. r_rs: Social group interaction avoidance
    4. r_rg: Goal progress incentive
    5. r_rf: Robot orientation stability (Go2-specific)
    6. Termination conditions for collisions, goal arrival, and robot flipping
    """
    
    def __init__(self, parent_node=None):
        """
        Initialise reward calculator with parameters
        """
        self.node = parent_node
        
        # Robot parameters
        self.robot_footprint_radius = 0.4  # meters - conservative estimate for Go2
        self.human_radius = 0.3  # meters - from people_decoder_node
        
        # Obstacle collision filtering to prevent false positives from tilting lidar
        self.obstacle_collision_step_threshold = 10  
        self.consecutive_collision_count = 0
        
        # Obstacle proximity filtering
        self.obstacle_proximity_step_threshold = 3  
        self.consecutive_proximity_count = 0

        # Human collision filtering to prevent false positives after reset
        self.human_collision_step_threshold = 3  
        self.consecutive_human_collision_count = 0
        
        # Robot orientation filtering to prevent false positives after reset  
        self.flipped_step_threshold = 5  
        self.consecutive_flipped_count = 0
        
        # Post-reset grace period to allow sensor data to stabilize
        self.reset_grace_period = 5
        self.reset_grace_period_step = 0

        # Proximity thresholds
        self.personal_space_radius = 1.2  
        self.obstacle_proximity_radius = 0.7  
        self.goal_arrival_threshold = 0.5 
        
        # Reward weights and penalties
        self.human_collision_penalty = -200.0  
        self.obstacle_collision_penalty = -100.0 
        self.personal_space_penalty = -0.5  
        self.obstacle_space_penalty = -0.5  
        self.robot_flipped_penalty = -200.0 
        self.arrival_bonus = 200.0       # Large bonus for reaching goal
        self.progress_reward_scale = 40  # For goal progress reward
        self.core_group_penalty = -3.0  # Penalty for entering core group area
        
        # State tracking for goal progress
        self.previous_goal_distance = None
        self.goal_reached = False
        self.progress_tick_total = 5
        self.progress_ticks_remaining = self.progress_tick_total
        self.progress_reward_per_tick = 0.0
        self.max_realistic_movement = 2.0  # Max 2 meters per step

        self.log_info("SocialRewardCalculator initialised with parameters:")
        self.log_info(f"  Robot footprint radius: {self.robot_footprint_radius}m")
        self.log_info(f"  Personal space radius: {self.personal_space_radius}m")
        self.log_info(f"  Goal arrival threshold: {self.goal_arrival_threshold}m")
    
    def calculate_total_reward_and_termination_conditions(
        self, 
        robot_pose: Pose,
        people_data: Optional[People],
        goal_position: Optional[Point], 
        scan_data: Optional[LaserScan],
        agent_name_mapping: Optional[Dict],
    ) -> Tuple[float, Dict[str, float], Dict[str, bool]]:
        
        components = {}
        termination_flags = {
            'collision_obstacle': False,
            'collision_human': False,
            'goal_reached': False,
            'robot_flipped': False
        }

        # Update grace period counter
        if self.reset_grace_period_step < self.reset_grace_period:
            self.reset_grace_period_step += 1

        # Extract robot position and orientation
        robot_pos = robot_pose.position
        robot_orientation = robot_pose.orientation
        
        # 1. Obstacle collision reward (r_ro)
        components['r_ro'], termination_flags['collision_obstacle'] = self.calculate_obstacle_reward_and_collision(scan_data)
        # 2. Human distance reward (r_rh)
        components['r_rh'], termination_flags['collision_human'] = self.calculate_human_distance_reward_and_collision(robot_pos, people_data)
        # 3. Social interaction reward (r_rs)
        components['r_rs'] = self.calculate_social_interaction_reward(robot_pos, people_data, agent_name_mapping)
        # 4. Goal progress reward (r_rg)
        components['r_rg'], termination_flags['goal_reached'] = self.calculate_goal_progress_reward_and_arrival(robot_pos, goal_position)
        # 5. Robot orientation reward (r_rf)
        components['r_rf'], termination_flags['robot_flipped'] = self.calculate_robot_orientation_reward_and_flipped(robot_orientation)

        # Apply termination flags only after grace period (except goal reached)
        if self.reset_grace_period_step < self.reset_grace_period:
            termination_flags['collision_obstacle'] = False
            termination_flags['collision_human'] = False
            termination_flags['robot_flipped'] = False
        
        # Total reward
        total_reward = sum(components.values())

        return total_reward, components, termination_flags

    def calculate_obstacle_reward_and_collision(self, scan_data: Optional[LaserScan]) -> Tuple[float, bool]:
        obstacle_reward = 0.0
        collision_detected = False

        if scan_data is None:
            return obstacle_reward, collision_detected

        try:
            # Find minimum distance to obstacles from laser scan
            valid_ranges = [r for r in scan_data.ranges if scan_data.range_min <= r <= scan_data.range_max]
            if not valid_ranges:
                return obstacle_reward, collision_detected

            min_obstacle_distance = min(valid_ranges)
            
            # Create proximity penalty increasing as robot gets closer to obstacles
            if min_obstacle_distance < self.obstacle_proximity_radius:
                self.consecutive_proximity_count += 1
                if self.consecutive_proximity_count >= self.obstacle_proximity_step_threshold:
                    penalty_range = self.obstacle_proximity_radius - self.robot_footprint_radius
                    proximity_factor = (self.obstacle_proximity_radius - min_obstacle_distance) / penalty_range
                    obstacle_reward += self.obstacle_space_penalty * proximity_factor
            else:
                self.consecutive_proximity_count = 0

            # Check if robot footprint intersects with obstacles
            if min_obstacle_distance < self.robot_footprint_radius:
                self.consecutive_collision_count += 1
                # self.log_info(f"collision count: {self.consecutive_collision_count}")
                if self.consecutive_collision_count >= self.obstacle_collision_step_threshold:
                    obstacle_reward = self.obstacle_collision_penalty
                    collision_detected = True
            else:
                self.consecutive_collision_count = 0

            return obstacle_reward, collision_detected

        except Exception as e:
            self.log_warn(f"Error calculating obstacle reward: {e}")
            return 0.0, False




    def calculate_human_distance_reward_and_collision(self, robot_pos: Point, people_data: Optional[People]) -> Tuple[float, bool]:
        if people_data is None or not people_data.people:
            self.consecutive_human_collision_count = 0
            return 0.0, False

        human_reward = 0.0
        collision_detected = False
        any_collision_this_step = False
        
        try:
            for person in people_data.people:
                # Calculate distance to each person
                human_pos = person.position
                distance = self.calculate_distance_2d(robot_pos, human_pos)
                
                # Check for collision or personal space violation
                collision_threshold = self.robot_footprint_radius + self.human_radius
                if distance < collision_threshold:
                    any_collision_this_step = True
                elif distance < self.personal_space_radius:
                    proximity_factor = (self.personal_space_radius - distance) / (self.personal_space_radius - collision_threshold)
                    human_reward += self.personal_space_penalty * proximity_factor

            if any_collision_this_step:
                self.consecutive_human_collision_count += 1
                if self.consecutive_human_collision_count >= self.human_collision_step_threshold:
                    human_reward = self.human_collision_penalty
                    collision_detected = True
            else:
                self.consecutive_human_collision_count = 0

            return human_reward, collision_detected
        
        except Exception as e:
            self.log_warn(f"Error calculating human distance reward: {e}")
            return 0.0, False
    



    def calculate_social_interaction_reward(
        self, 
        robot_pos: Point, 
        people_data: Optional[People], 
        agent_name_mapping: Optional[Dict]
    ) -> float:
        if (people_data is None or not people_data.people or 
            agent_name_mapping is None):
            return 0.0
            
        try:
            # Group people by group_id (excluding single agents with group_id = -1)
            groups = self.group_people_by_group_id(people_data, agent_name_mapping)
            
            total_penalty = 0.0
            robot_point = ShapelyPoint(robot_pos.x, robot_pos.y)
            
            for group_id, group_members in groups.items():
                # Skip single agents (group_id = -1) or groups with < 2 people
                if group_id == -1 or len(group_members) < 2:
                    continue
                    
                # Get group member positions as list of (x,y) tuples
                group_points = [(person.position.x, person.position.y) for person in group_members]
                
                # Calculate intrusion penalty for this group
                penalty = self.calculate_group_intrusion_penalty(
                    group_points, robot_point, group_id
                )
                total_penalty += penalty
                    
        except Exception as e:
            self.log_warn(f"Error calculating social interaction reward: {e}")
            
        return total_penalty
    
    def calculate_group_intrusion_penalty(
        self, 
        group_points: List[Tuple[float, float]], 
        robot_point: ShapelyPoint,
        group_id: int
    ) -> float:
        if len(group_points) < 2:
            return 0.0
            
        try:
            # Create MultiPoint and compute convex hull
            multi_point = MultiPoint(group_points)
            hull = multi_point.convex_hull
            
            # Extend hull by buffer distance to increase group p-space footprint
            buffer_distance = 0.3
            outer_hull = hull.buffer(buffer_distance)
            
            # Check robot position relative to group boundaries
            robot_in_outer = outer_hull.contains(robot_point)
            robot_in_inner = hull.contains(robot_point)
                        
            if not robot_in_outer:
                # Robot is outside the personal space - no penalty
                return 0.0
            elif robot_in_inner:
                # Robot is inside the core group area - maximum penalty
                self.log_info(f"Robot inside group {group_id} core area: penalty={self.core_group_penalty:.3f}")
                return self.core_group_penalty
            else: # Robot is in the buffer zone - scaled penalty based on intrusion depth
                # Calculate distance from robot to the group hull
                distance_to_hull = hull.distance(robot_point)
                
                # Calculate normalised intrusion depth 
                intrusion_depth = (buffer_distance - distance_to_hull) / buffer_distance
                intrusion_depth = np.clip(intrusion_depth, 0.0, 1.0)
                
                # Scale penalty based on intrusion depth
                penalty = self.core_group_penalty * intrusion_depth
                
                self.log_info(f"Robot in group {group_id} buffer zone: "
                             f"dist_to_hull={distance_to_hull:.3f}, depth={intrusion_depth:.3f}, penalty={penalty:.3f}")
                return penalty
                
        except Exception as e:
            self.log_warn(f"Error calculating group {group_id} intrusion penalty: {e}")
            return 0.0

    def calculate_goal_progress_reward_and_arrival(
        self,
        robot_pos: Point,
        goal_position: Optional[Point],
    ) -> Tuple[float, bool]:
        if goal_position is None:
            return 0.0, False
            
        try:
            # Calculate current distance to goal
            current_goal_distance = self.calculate_distance_2d(robot_pos, goal_position)
            
            goal_reward = 0.0
            goal_reached_flag = False
            
            # r_ag: Arrival bonus
            if current_goal_distance < self.goal_arrival_threshold:
                if not self.goal_reached:  # Only give bonus once
                    goal_reward += self.arrival_bonus
                    self.goal_reached = True
                    goal_reached_flag = True
                    self.log_info(f"Goal reached! Distance: {current_goal_distance:.3f}m")
            else:
                self.goal_reached = False
            
            # r_rtg: Progress reward
            
            # Calculate and apply progress reward if new goal distance
            if self.previous_goal_distance is not None and self.previous_goal_distance != current_goal_distance:
                distance_change = self.previous_goal_distance - current_goal_distance

                if abs(distance_change) > self.max_realistic_movement:
                    self.log_warn(f"Ignoring unrealistic distance change: {distance_change:.3f}m (likely teleportation)")
                else:
                    # Positive reward for getting closer, negative for getting further
                    progress_scale = self.progress_reward_scale
                    
                    # Scale up rewards when close to goal (within 1.5m)
                    if current_goal_distance <= 1.5:
                        progress_scale *= 2.0
                    
                    total_progress_reward = distance_change * progress_scale
                    # Spread over set number of future ticks
                    self.progress_reward_per_tick = total_progress_reward / self.progress_tick_total
                    self.progress_ticks_remaining = self.progress_tick_total

                    # Apply progress reward if ticks remain
                    goal_reward += self.progress_reward_per_tick
                    self.progress_ticks_remaining -= 1

            # Apply old progress reward if ticks remain        
            elif self.progress_ticks_remaining > 0:
                goal_reward += self.progress_reward_per_tick
                self.progress_ticks_remaining -= 1

            self.previous_goal_distance = current_goal_distance

            return goal_reward, goal_reached_flag
            
        except Exception as e:
            self.log_warn(f"Error : {e}")
            return 0.0, False
        



    def calculate_robot_orientation_reward_and_flipped(self, robot_orientation: Quaternion) -> Tuple[float, bool]:
        orientation_reward = 0.0
        robot_flipped = False
        
        if robot_orientation is not None:
            roll, pitch, _ = self.quaternion_to_euler(robot_orientation)
            
            # Roll and Pitch penalty: linear increase with angle magnitude
            roll_threshold = 0.175  # 10 degrees
            if abs(roll) > roll_threshold:
                roll_penalty = -2.0 * (abs(roll) - roll_threshold)
            else:
                roll_penalty = 0.0

            pitch_threshold = 0.175  # 10 degrees
            if abs(pitch) > pitch_threshold:
                pitch_penalty = -2.0 * (abs(pitch) - pitch_threshold)
            else:
                pitch_penalty = 0.0
            
            orientation_reward = roll_penalty + pitch_penalty
            
            # Check if robot is flipped (>80 degrees)
            if abs(roll) > 1.4 or abs(pitch) > 1.4:  # 80 degrees
                self.consecutive_flipped_count += 1
                if self.consecutive_flipped_count >= self.flipped_step_threshold:
                    robot_flipped = True
                    orientation_reward += self.robot_flipped_penalty
            else:
                self.consecutive_flipped_count = 0
        
        return orientation_reward, robot_flipped


    # Helper methods
    def group_people_by_group_id(
        self, 
        people_data: People, 
        agent_name_mapping: Dict
    ) -> Dict[int, List[Person]]:
        groups = {}
        
        for person in people_data.people:
            if person.name in agent_name_mapping:
                _, group_id = agent_name_mapping[person.name]
                
                if group_id not in groups:
                    groups[group_id] = []
                groups[group_id].append(person)
            else:
                self.log_warn(f"  Person {person.name} not found in agent_name_mapping!")
        
        return groups
    
    def reset(self):
        # Reset variables for new episode
        self.previous_goal_distance = None
        self.progress_reward_per_tick = 0.0
        self.progress_ticks_remaining = 0
        self.goal_reached = False
        self.consecutive_collision_count = 0
        self.consecutive_human_collision_count = 0
        self.consecutive_flipped_count = 0
        self.reset_grace_period_step = 0
        self.log_info(f"SocialRewardCalculator state reset - grace period: {self.reset_grace_period} steps")
    


    # Helpers #

    def quaternion_to_euler(self, quat: Quaternion) -> Tuple[float, float, float]:
        x, y, z, w = quat.x, quat.y, quat.z, quat.w
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)
        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = np.copysign(np.pi / 2, sinp)
        else:
            pitch = np.arcsin(sinp)
        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)
        return roll, pitch, yaw

    def calculate_distance_2d(self, point1: Point, point2: Point) -> float:
        dx = point1.x - point2.x
        dy = point1.y - point2.y
        return math.sqrt(dx * dx + dy * dy)
    
    #loggins
    def log_info(self, message: str):
        if self.node:
            self.node.get_logger().info(message)
    
    def log_warn(self, message: str):
        if self.node:
            self.node.get_logger().warn(message)
