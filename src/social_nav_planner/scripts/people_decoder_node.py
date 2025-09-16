#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

import numpy as np
import math
from typing import List, Tuple, Optional
import json

from people_msgs.msg import People #type: ignore
from std_msgs.msg import Float32MultiArray, MultiArrayDimension, String, Int16MultiArray
from geometry_msgs.msg import Point
import tf2_ros
import tf2_geometry_msgs


class PeopleDecoderNode(Node):    
    def __init__(self):
        super().__init__('people_decoder_node')
        # self.count = 0  # For debug logging

        # Declare Parameters
        self.declare_parameter('num_rays', 240)
        self.declare_parameter('fov_degrees', 180.0)  
        self.declare_parameter('max_range', 10.0)     
        self.declare_parameter('agent_radius', 0.3)   # Agent footprint radius in meters
        self.declare_parameter('update_rate', 10.0)   # Hz #TODO: Adjust based on speed of device
        self.declare_parameter('robot_frame', 'base_link')
        self.declare_parameter('world_frame', 'map')
        
        # Get parameters
        self.num_rays = self.get_parameter('num_rays').get_parameter_value().integer_value
        self.fov_degrees = self.get_parameter('fov_degrees').get_parameter_value().double_value
        self.max_range = self.get_parameter('max_range').get_parameter_value().double_value
        self.agent_radius = self.get_parameter('agent_radius').get_parameter_value().double_value
        self.update_rate = self.get_parameter('update_rate').get_parameter_value().double_value
        self.robot_frame = self.get_parameter('robot_frame').get_parameter_value().string_value
        self.world_frame = self.get_parameter('world_frame').get_parameter_value().string_value
        
        self.fov_radians = math.radians(self.fov_degrees)
        
        # Calculate angular resolution
        self.angular_resolution = self.fov_radians / self.num_rays
        
        # Initialize arrays
        self.distances = np.full(self.num_rays, -1.0, dtype=np.float32)
        self.agent_ids = np.full(self.num_rays, -1, dtype=np.int16)

        # Map from agent name to index (for consistent IDs)
        self.agent_name_to_index = {} 
        self.next_agent_index = 0
        self.last_mapping_json = None

        self.latest_people = None

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # QoS Profile for subscription - ensures we get the latest data if it falls behind
        people_sub_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Mapping publisher QoS profile - allows for single published message when subscriber connects
        mapping_pub_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Subscribers
        self.people_subscriber = self.create_subscription(
            People,
            '/people',
            self.people_callback,
            people_sub_qos
        )
        
        # Publishers
        self.distances_publisher = self.create_publisher(
            Float32MultiArray,
            '/social_observation/distances',
            10
        )
        self.agent_ids_publisher = self.create_publisher(
            Int16MultiArray,
            '/social_observation/agent_ids',
            10
        )
        self.agent_name_to_index_publisher = self.create_publisher(
            String,
            '/social_observation/agent_name_to_index',
            mapping_pub_qos
        )
        
        # Timer for processing and publishing observations
        self.timer = self.create_timer(
            1.0 / self.update_rate,
            self.process_and_publish
        )
        
        # self.get_logger().info(f'People Decoder Node initialized:')
        # self.get_logger().info(f'  - Number of rays: {self.num_rays}')
        # self.get_logger().info(f'  - FOV: {self.fov_degrees}°')
        # self.get_logger().info(f'  - Max range: {self.max_range}m')
        # self.get_logger().info(f'  - Agent radius: {self.agent_radius}m')
        # self.get_logger().info(f'  - Update rate: {self.update_rate}Hz')
        # self.get_logger().info(f'  - Angular resolution: {math.degrees(self.angular_resolution):.2f}°')


    def people_callback(self, msg: People):
        self.latest_people = msg

    # Process the latest people data and publish people arrays.
    def process_and_publish(self):
        if self.latest_people is None:
            # No people data yet, publish empty observations
            self.publish_arrays()
            return
            
        # Reset arrays
        self.distances.fill(-1.0)
        self.agent_ids = [-1] * self.num_rays
        mapping_changed = False
        
        # Process each person
        for person in self.latest_people.people:
            if person.name not in self.agent_name_to_index:
                self.agent_name_to_index[person.name] = self.next_agent_index
                self.next_agent_index += 1
                mapping_changed = True
            
            # get agent index
            agent_index = self.agent_name_to_index[person.name]

            # Transform person position to robot frame
            robot_point = self.transform_point_to_robot_frame(
                person.position, 
                self.latest_people.header.frame_id
            )
            if robot_point is None:
                continue
                
            # Compute bearing and distance
            bearing, distance = self.compute_bearing_and_distance(robot_point)
            
            # Get angular bins for the agent
            bins = self.get_angular_bins_for_agent(bearing, distance)
            
            # Update observation arrays
            for bin_idx in bins:
                # If bin is empty or current agent is closer, update it
                if self.distances[bin_idx] < 0 or distance < self.distances[bin_idx]:
                    self.distances[bin_idx] = distance
                    self.agent_ids[bin_idx] = agent_index

        # Publish observations
        self.publish_arrays(mapping_changed)



    def transform_point_to_robot_frame(self, point: Point, source_frame: str):
        try:
            # Get transform from source_frame to robot_frame
            transform = self.tf_buffer.lookup_transform(
                self.robot_frame,
                source_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
            
            # Create a PointStamped message
            point_stamped = tf2_geometry_msgs.PointStamped()
            point_stamped.header.frame_id = source_frame
            point_stamped.point = point
            
            # Transform the point
            transformed_point = tf2_geometry_msgs.do_transform_point(point_stamped, transform)
            
            return transformed_point.point
            
        except Exception as e:
            self.get_logger().debug(f'TF2 transformation failed: {e}')
            return None

    def compute_bearing_and_distance(self, point: Point) -> Tuple[float, float]:
        distance = math.sqrt(point.x**2 + point.y**2)
        bearing = math.atan2(point.y, point.x)
        return bearing, distance

    def get_angular_bins_for_agent(self, bearing: float, distance: float) -> List[int]:
        bins = []
        
        # Skip if agent is too far or behind robot
        if distance > self.max_range:
            return bins
            
        # Calculate angular span based on agent radius and distance
        if distance > 0:
            angular_span = math.atan2(self.agent_radius, distance)
        else:
            angular_span = self.angular_resolution
            
        # Calculate start and end angles for FOV (-90° to +90° in front)
        fov_start = -self.fov_radians / 2
        fov_end = self.fov_radians / 2
        
        # Check if agent is within FOV
        if bearing < fov_start - angular_span or bearing > fov_end + angular_span:
            return bins
            
        # Calculate bin range
        start_angle = max(bearing - angular_span, fov_start)
        end_angle = min(bearing + angular_span, fov_end)
        
        # Convert angles to bin indices
        start_bin = int((start_angle - fov_start) / self.angular_resolution)
        end_bin = int((end_angle - fov_start) / self.angular_resolution)
        
        # Clamp to valid range
        start_bin = max(0, start_bin)
        end_bin = min(self.num_rays - 1, end_bin)
        
        # Add all bins in range
        for bin_idx in range(start_bin, end_bin + 1):
            bins.append(bin_idx)
            
        return bins


    # Publish the distance and agent ID arrays.
    def publish_arrays(self, mapping_changed=False):
        # Publish distances
        distances_msg = Float32MultiArray()
        distances_msg.layout.dim.append(MultiArrayDimension())
        distances_msg.layout.dim[0].label = "rays"
        distances_msg.layout.dim[0].size = self.num_rays
        distances_msg.layout.dim[0].stride = 1
        distances_msg.layout.data_offset = 0
        distances_msg.data = self.distances.tolist()
        
        self.distances_publisher.publish(distances_msg)

        # Publish agent IDs
        agent_ids_msg = Int16MultiArray()
        agent_ids_msg.layout.dim.append(MultiArrayDimension())
        agent_ids_msg.layout.dim[0].label = "rays"
        agent_ids_msg.layout.dim[0].size = self.num_rays
        agent_ids_msg.layout.dim[0].stride = 1
        agent_ids_msg.layout.data_offset = 0
        agent_ids_msg.data = [int(idx) for idx in self.agent_ids]

        self.agent_ids_publisher.publish(agent_ids_msg)

        # Publish the mapping as a JSON string only if changed
        mapping_json = json.dumps(self.agent_name_to_index)
        if mapping_changed or self.last_mapping_json != mapping_json:
            mapping_msg = String()
            mapping_msg.data = mapping_json
            self.agent_name_to_index_publisher.publish(mapping_msg)
            self.last_mapping_json = mapping_json

        # # Debug logging
        # self.count += 1
        # if self.count % 10 == 0:  # Log every 10th publish
        #     self.get_logger().info(f'Published mapping: {mapping_json}\n')
        #     self.get_logger().info(f'Published distances: {[dist for dist in self.distances.tolist()]}\n\n')
        #     self.get_logger().info(f'Published agent IDs: {[int(idx) for idx in self.agent_ids]}\n\n')


def main(args=None):
    rclpy.init(args=args)

    node = PeopleDecoderNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
