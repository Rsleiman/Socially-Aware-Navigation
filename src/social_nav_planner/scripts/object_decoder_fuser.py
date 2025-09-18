#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.time import Time

import numpy as np
import time
from typing import Optional

from std_msgs.msg import Float32MultiArray, Int16MultiArray
from geometry_msgs.msg import PoseStamped
from rclpy.time import Time

#TODO: Add functionality to handle goal

## CLI debugger:
"""
echo "Agent Distances:" &&
ros2 topic echo /social_observation/agent_distances --once --field data &&
echo "Obstacle Distances:" &&
ros2 topic echo /social_observation/obstacle_distances --once --field data &&
echo "Agent Group IDs:" &&
ros2 topic echo /social_observation/agent_group_ids --once --field data &&
echo "Goal Distances:" &&
ros2 topic echo /social_observation/goal_distances --once --field data &&
echo "Fused Object Array:" &&
ros2 topic echo /social_observation/fused_object_array --once --field data
"""


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
        self.latest_agent_distances: Optional[TimestampedArray] = None
        self.latest_obstacle_distances: Optional[TimestampedArray] = None
        self.latest_agent_group_ids: Optional[TimestampedArray] = None
        self.latest_goal_distances: Optional[TimestampedArray] = None

        self.occluded_agent_distances = np.full(self.num_rays, -1.0, dtype=np.float32)
        self.occluded_obstacle_distances = np.full(self.num_rays, -1.0, dtype=np.float32)
        self.occluded_agent_group_ids = np.full(self.num_rays, -1, dtype=np.int32)
        self.goal_distances = np.full(self.num_rays, -1.0, dtype=np.float32) #TODO: Determine whether to occlude goals behind obstacles. Leaning towards no.
    
        # Initialise combined message object
        # Flattened format: [agent_0, ..., agent_239, obstacle_0, ..., obstacle_239, group_id_0, ..., group_id_239, goal_distance_0, ..., goal_distance_239]
        self.combined_msg = Float32MultiArray()
        
        
        # QoS Profile
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Subscribers
        self.agents_sub = self.create_subscription(
            Float32MultiArray, 
            '/social_observation/agent_distances', 
            self.agents_callback, 
            qos
        )
        self.obstacles_sub = self.create_subscription(
            Float32MultiArray, 
            '/social_observation/obstacle_distances', 
            self.obstacles_callback, 
            qos
        )
        self.agent_group_ids_sub = self.create_subscription(
            Int16MultiArray,
            '/social_observation/agent_group_ids',
            self.agent_group_ids_callback,
            qos
        )

        # Currently not used, but could be useful for future extensions
        # self.agent_mapping_sub = self.create_subscription(
        #     Float32MultiArray,
        #     '/social_observation/agent_name_to_data',
        #     self.agent_mapping_callback,
        #     qos
        # )

        self.goal_distances_sub = self.create_subscription(
            Float32MultiArray,
            '/social_observation/goal_distances',
            self.goal_distances_callback,
            qos
        )

        # Publishers
        #TODO: Should publish occlusion filtered individual arrays separately? Leaning towards no
        # self.fused_agents_pub = self.create_publisher(
        #     Float32MultiArray, 
        #     '/social_observation/occluded_agent_distances', 
        #     10
        # )
        # self.fused_obstacles_pub = self.create_publisher(
        #     Float32MultiArray, 
        #     '/social_observation/occluded_obstacle_distances', 
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
        self.get_logger().info('    * /social_observation/occluded_agent_distances')
        self.get_logger().info('    * /social_observation/occluded_obstacle_distances') 
        self.get_logger().info('    * /social_observation/fused_object_array (2x240 array)')


    def agents_callback(self, msg):
        if len(msg.data) == self.num_rays:
            self.latest_agent_distances = TimestampedArray(
                np.array(msg.data, dtype=np.float32), self.get_clock().now()
            )
        else:
            self.get_logger().warn(f'Agent distances array size mismatch: expected {self.num_rays}, got {len(msg.data)}')

    def obstacles_callback(self, msg):
        if len(msg.data) == self.num_rays:
            self.latest_obstacle_distances = TimestampedArray(
                np.array(msg.data, dtype=np.float32), self.get_clock().now()
            )
        else:
            self.get_logger().warn(f'Obstacle distances array size mismatch: expected {self.num_rays}, got {len(msg.data)}')

    def agent_group_ids_callback(self, msg):
        if len(msg.data) == self.num_rays:
            self.latest_agent_group_ids = TimestampedArray(
                np.array(msg.data, dtype=np.int32), self.get_clock().now()
            )
        else:
            self.get_logger().warn(f'Agent Group IDs array size mismatch: expected {self.num_rays}, got {len(msg.data)}')

    def goal_distances_callback(self, msg):
        if len(msg.data) == self.num_rays:
            self.latest_goal_distances = TimestampedArray(
                np.array(msg.data, dtype=np.float32), self.get_clock().now()
            )
        else:
            self.get_logger().warn(f'Goal distances array size mismatch: expected {self.num_rays}, got {len(msg.data)}')

    # Check if input arrays are time synchronised
    def arrays_synchronised(self) -> bool:
        # Ensure all 4 data sources are available
        if (self.latest_agent_distances is None or
            self.latest_obstacle_distances is None or
            self.latest_agent_group_ids is None or
            self.latest_goal_distances is None):
            self.get_logger().warn("Waiting for all input arrays to be available...")
            return False
        
        # Check the maximum time difference between any of the input arrays is within the allowed threshold
        time_diffs = [
            abs((self.latest_agent_distances.timestamp - self.latest_obstacle_distances.timestamp).nanoseconds * 1e-9),
            abs((self.latest_agent_distances.timestamp - self.latest_agent_group_ids.timestamp).nanoseconds * 1e-9),
            abs((self.latest_obstacle_distances.timestamp - self.latest_agent_group_ids.timestamp).nanoseconds * 1e-9),
            abs((self.latest_goal_distances.timestamp - self.latest_agent_distances.timestamp).nanoseconds * 1e-9),
            abs((self.latest_goal_distances.timestamp - self.latest_obstacle_distances.timestamp).nanoseconds * 1e-9),
            abs((self.latest_goal_distances.timestamp - self.latest_agent_group_ids.timestamp).nanoseconds * 1e-9)
        ]

        time_diff_labels = [
            "agent vs obstacle",
            "agent vs group_id",
            "obstacle vs group_id",
            "goal vs agent",
            "goal vs obstacle",
            "goal vs group_id"
        ]
        highest_time_diff = max(time_diffs)
        if highest_time_diff <= self.max_time_diff:
            self.get_logger().info(f"Input arrays synchronised: {highest_time_diff}s <= threshold: {self.max_time_diff}s")
            return True
        else:
            self.get_logger().warn(f"Input arrays not synchronised: {highest_time_diff:.4f}s > threshold: {self.max_time_diff:.4f}s. \
                                   Culprit: {time_diff_labels[np.argmax(time_diffs)]}")
            return False


    def process_and_publish(self):
        # Reset fused arrays
        self.occluded_agent_distances.fill(-1.0)
        self.occluded_obstacle_distances.fill(-1.0)
        self.occluded_agent_group_ids.fill(-1) # Occlude the same way as agent distances
        self.goal_distances.fill(-1.0) 

        if not self.arrays_synchronised():
            #TODO: Determine what to do if not synchronised, leaning towards publishing nothing (or last valid?)
            return
            
        # Get synchronised input arrays
        agent_distances = self.latest_agent_distances.data
        obstacle_distances = self.latest_obstacle_distances.data
        agent_group_ids = self.latest_agent_group_ids.data
        goal_distances = self.latest_goal_distances.data

        
        # Create masks for valid detections (>= 0)
        agents_valid = agent_distances >= 0
        obstacles_valid = obstacle_distances >= 0

        # Case 1: Both agents and obstacles present, occlude futher object
        both_present = agents_valid & obstacles_valid
        if np.any(both_present):
            self.get_logger().info("Both agents and obstacles present")
            # When agents are closer:
            agents_closer = both_present & (agent_distances <= obstacle_distances)
            self.occluded_agent_distances[agents_closer] = agent_distances[agents_closer]
            self.occluded_agent_group_ids[agents_closer] = agent_group_ids[agents_closer]

            # When obstacles are closer:
            obstacles_closer = both_present & (obstacle_distances < agent_distances)
            self.occluded_obstacle_distances[obstacles_closer] = obstacle_distances[obstacles_closer]

        # Case 2: Only agents present
        only_agents = agents_valid & ~obstacles_valid
        if np.any(only_agents):
            self.get_logger().info("Only agents present")
            self.occluded_agent_distances[only_agents] = agent_distances[only_agents]
            self.occluded_agent_group_ids[only_agents] = agent_group_ids[only_agents]

        # Case 3: Only obstacles present
        only_obstacles = obstacles_valid & ~agents_valid
        if np.any(only_obstacles):
            self.get_logger().info("Only obstacles present")
            self.occluded_obstacle_distances[only_obstacles] = obstacle_distances[only_obstacles]

        # Process goal distances
        self.goal_distances = goal_distances

        # Publish the fused arrays (no occlusion needed)
        self.publish_arrays()


    def publish_arrays(self):  
        self.get_logger().info("Publishing fused object array...")     
        
        #TODO: Should publish occlusion filtered individual arrays separately? Leaning towards no
        # # Publish fused agent distances using pre-allocated message
        # self.agent_msg.data = self.occluded_agent_distances.tolist()
        # self.fused_agents_pub.publish(self.agent_msg)

        # # Publish fused obstacle distances using pre-allocated message
        # self.obstacle_msg.data = self.occluded_obstacle_distances.tolist()
        # self.fused_obstacles_pub.publish(self.obstacle_msg)

        # TODO: Do the same with goal and agent_group_id columns
        # Publish combined 2x240 observation array using pre-allocated message
        # Flattened format: [agent_0, ..., agent_239, obstacle_0, ..., obstacle_239]
        combined_data = np.concatenate(
            [
                self.occluded_agent_distances,
                self.occluded_obstacle_distances,
                self.occluded_agent_group_ids,
                self.goal_distances
            ]
        )
        self.combined_msg.data = combined_data.tolist()
        self.fused_object_pub.publish(self.combined_msg)
        self.get_logger().info('Published fused object array')

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
