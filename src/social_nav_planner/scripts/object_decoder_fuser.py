#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.time import Time

import numpy as np
import time
from typing import Optional

from std_msgs.msg import Float32MultiArray
from rclpy.time import Time

#TODO: Add functionality to handle group_id and goal

class TimestampedArray:
    def __init__(self, data: np.ndarray, timestamp: Time):
        self.data = data
        self.timestamp = timestamp


class ObjectDecoderFuser(Node):
    def __init__(self):
        super().__init__('object_decoder_fuser')
        
        # Parameters
        self.declare_parameter('num_rays', 240)
        self.declare_parameter('update_rate', 5.0)  # Hz - Consistent with other nodes for RL
        self.declare_parameter('max_time_diff', 0.1)  # Max time difference for synchronisation (seconds)
        
        self.num_rays = self.get_parameter('num_rays').get_parameter_value().integer_value
        self.update_rate = self.get_parameter('update_rate').get_parameter_value().double_value
        self.max_time_diff = self.get_parameter('max_time_diff').get_parameter_value().double_value
        
        # Initialise arrays
        self.latest_people_distances: Optional[TimestampedArray] = None
        self.latest_obstacle_distances: Optional[TimestampedArray] = None
        
        self.fused_people_distances = np.full(self.num_rays, -1.0, dtype=np.float32)
        self.fused_obstacle_distances = np.full(self.num_rays, -1.0, dtype=np.float32)
        
        # Initialise message objects
        self.people_msg = Float32MultiArray()
        self.obstacles_msg = Float32MultiArray()
        self.combined_msg = Float32MultiArray()
        
        
        # QoS Profile
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Subscribers
        self.people_sub = self.create_subscription(
            Float32MultiArray, 
            '/social_observation/distances', 
            self.people_callback, 
            qos
        )
        self.obstacles_sub = self.create_subscription(
            Float32MultiArray, 
            '/social_observation/obstacle_distances', 
            self.obstacles_callback, 
            qos
        )
        
        # Publishers
        #TODO: Should publish occlusion filtered individual arrays separately? Leaning towards no
        # self.fused_people_pub = self.create_publisher(
        #     Float32MultiArray, 
        #     '/social_observation/fused_people_distances', 
        #     10
        # )
        # self.fused_obstacles_pub = self.create_publisher(
        #     Float32MultiArray, 
        #     '/social_observation/fused_obstacle_distances', 
        #     10
        # )
        self.fused_object_pub = self.create_publisher(
            Float32MultiArray,
            '/social_observation/fused_object_array',
            10
        )
        
        # Timer for processing and publishing
        self.timer = self.create_timer(
            1.0 / self.update_rate,
            self.process_and_publish
        )
        
        self.get_logger().info('Social Observation Fuser initialised (Performance Optimised)')
        self.get_logger().info(f'  - Number of rays: {self.num_rays}')
        self.get_logger().info(f'  - Update rate: {self.update_rate}Hz')
        self.get_logger().info(f'  - Max time diff: {self.max_time_diff}s (for synchronisation)')
        self.get_logger().info('  - Frame consistency: Both inputs in front_laser frame')
        self.get_logger().info('  - Output topics:')
        self.get_logger().info('    * /social_observation/fused_people_distances')
        self.get_logger().info('    * /social_observation/fused_obstacle_distances') 
        self.get_logger().info('    * /social_observation/fused_object_array (2x240 array)')


    def people_callback(self, msg):
        if len(msg.data) == self.num_rays:
            # Store data with current timestamp for synchronisation
            current_time = self.get_clock().now()
            self.latest_people_distances = TimestampedArray(
                np.array(msg.data, dtype=np.float32), current_time
            )
        else:
            self.get_logger().warn(f'People distances array size mismatch: expected {self.num_rays}, got {len(msg.data)}')

    def obstacles_callback(self, msg):
        if len(msg.data) == self.num_rays:
            # Store data with current timestamp for synchronisation
            current_time = self.get_clock().now()
            self.latest_obstacle_distances = TimestampedArray(
                np.array(msg.data, dtype=np.float32), current_time
            )
        else:
            self.get_logger().warn(f'Obstacle distances array size mismatch: expected {self.num_rays}, got {len(msg.data)}')

    # Check if input arrays are time synchronised
    def arrays_synchronised(self) -> bool:
        if self.latest_people_distances is None or self.latest_obstacle_distances is None:
            return False
            
        time_diff = abs(
            (self.latest_people_distances.timestamp - self.latest_obstacle_distances.timestamp).nanoseconds * 1e-9
        )
        return time_diff <= self.max_time_diff

    def process_and_publish(self):
        if not self.arrays_synchronised():
            # No synchronised data yet, publish empty arrays
            self.publish_arrays()
            return
            
        # Get synchronised input arrays
        people_distances = self.latest_people_distances.data
        obstacle_distances = self.latest_obstacle_distances.data
        
        # Reset fused arrays using efficient fill operation
        self.fused_people_distances.fill(-1.0)
        self.fused_obstacle_distances.fill(-1.0)
        
        # Vectorised occlusion logic using NumPy boolean indexing
        # Create masks for valid detections (>= 0)
        people_valid = people_distances >= 0
        obstacles_valid = obstacle_distances >= 0
        
        # Case 1: Both people and obstacles present
        both_present = people_valid & obstacles_valid
        if np.any(both_present):
            # People are closer - people visible, obstacles occluded
            people_closer = both_present & (people_distances <= obstacle_distances)
            self.fused_people_distances[people_closer] = people_distances[people_closer]
            
            # Obstacles are closer - obstacles visible, people occluded  
            obstacles_closer = both_present & (obstacle_distances < people_distances)
            self.fused_obstacle_distances[obstacles_closer] = obstacle_distances[obstacles_closer]

        # Case 2: Only people present
        only_people = people_valid & ~obstacles_valid
        if np.any(only_people):
            self.fused_people_distances[only_people] = people_distances[only_people]
    
        # Case 3: Only obstacles present
        only_obstacles = obstacles_valid & ~people_valid
        if np.any(only_obstacles):
            self.fused_obstacle_distances[only_obstacles] = obstacle_distances[only_obstacles]

        # Publish the fused arrays
        self.publish_arrays()


    def publish_arrays(self):       
        
        #TODO: Should publish occlusion filtered individual arrays separately? Leaning towards no
        # # Publish fused people distances using pre-allocated message
        # self.people_msg.data = self.fused_people_distances.tolist()
        # self.fused_people_pub.publish(self.people_msg)
        
        # # Publish fused obstacle distances using pre-allocated message
        # self.obstacles_msg.data = self.fused_obstacle_distances.tolist()
        # self.fused_obstacles_pub.publish(self.obstacles_msg)
        
        # TODO: Do the same with goal and agent_group_id columns
        # Publish combined 2x240 observation array using pre-allocated message
        # Flattened format: [people_0, ..., people_239, obstacle_0, ..., obstacle_239]
        combined_data = np.concatenate([self.fused_people_distances, self.fused_obstacle_distances])
        self.combined_msg.data = combined_data.tolist()

        self.fused_object_pub.publish(self.combined_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ObjectDecoderFuser()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
