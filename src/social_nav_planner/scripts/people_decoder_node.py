#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rcl_interfaces.srv import GetParameters

import numpy as np
import math
from typing import List, Tuple, Optional
import json

from people_msgs.msg import People #type: ignore
from std_msgs.msg import Float32MultiArray, String, Int16MultiArray
from geometry_msgs.msg import Point
import tf2_ros
import tf2_geometry_msgs

#TODO: Add functionality to handle group_id (if any)


class PeopleDecoderNode(Node):    
    def __init__(self):
        super().__init__('people_decoder_node')
        # self.count = 0  # For debug logging

        # Declare Parameters
        self.declare_parameter('num_rays', 240)
        self.declare_parameter('fov_degrees', 180.0)  
        self.declare_parameter('max_range', 10.0)     
        self.declare_parameter('agent_radius', 0.3)   # Agent footprint radius in meters
        self.declare_parameter('update_rate', 5.0)    # TODO: Tune rate depending on performance vs functionality
        self.declare_parameter('robot_frame', 'front_laser') # Supposed to simulate laser data
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

        # Map from agent name to [id, group_id] from /hunav_loader parameters
        self.agent_name_to_data = {}
        self.last_mapping_json = None
        
        # Create client to hunav_loader's parameter service and load agent data
        self.param_client = self.create_client(GetParameters, '/hunav_loader/get_parameters')
        self.load_agent_data_from_parameters()

        self.latest_people = None
        self.latest_transform = None

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
            '/social_observation/agent_distances',
            10
        )
        #TODO: Currently, the object_decoder_fuser does not alter or process the agent_ids data. Do not use data from this topic until addressed.
        self.agent_ids_publisher = self.create_publisher(
            Int16MultiArray,
            '/social_observation/agent_ids',
            10
        )
        # For ease of mapping agent names to id and group_id
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

    # Load agent data from hunav_loader parameters
    def load_agent_data_from_parameters(self):
        try:
            self.get_logger().info('Waiting for hunav_loader parameter service...')
            while not self.param_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('Still waiting for hunav_loader parameter service...')
            
            # Get list of agents
            req = GetParameters.Request()
            req.names = ['agents']
            future = self.param_client.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            response = future.result()
            
            if response is None or len(response.values) == 0:
                self.get_logger().warn('Failed to get agents parameter from hunav_loader')
                return
                
            agent_names = response.values[0].string_array_value
            self.get_logger().info(f'Got agents from hunav_loader: {agent_names}')
            
            # Get id and group_id for each agent
            for agent_name in agent_names:
                try:
                    # Request id and group_id for this agent
                    req = GetParameters.Request()
                    req.names = [f'{agent_name}.id', f'{agent_name}.group_id']
                    future = self.param_client.call_async(req)
                    rclpy.spin_until_future_complete(self, future)
                    response = future.result()
                    
                    if response is None or len(response.values) < 2:
                        self.get_logger().warn(f'Failed to get id/group_id for agent {agent_name}')
                        continue
                    
                    agent_id = response.values[0].integer_value
                    group_id = response.values[1].integer_value
                    
                    self.agent_name_to_data[agent_name] = [agent_id, group_id]
                    self.get_logger().info(f'Loaded agent {agent_name}: id={agent_id}, group_id={group_id}')
                    
                except Exception as e:
                    self.get_logger().warn(f'Error getting parameters for agent {agent_name}: {e}')
                    
        except Exception as e:
            self.get_logger().warn(f'Failed to load agent data from hunav_loader: {e}')

    def people_callback(self, msg: People):
        self.latest_people = msg

    # Process the latest people data and publish people arrays.
    def process_and_publish(self):
        if self.latest_people is None:
            # No people data yet, publish empty observations
            self.publish_arrays()
            return
        
        # Get transform once per timer tick and cache it
        try:
            self.latest_transform = self.tf_buffer.lookup_transform(
                self.robot_frame,
                self.latest_people.header.frame_id,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
        except Exception as e:
            self.get_logger().debug(f'TF2 transformation failed: {e}')
            self.publish_arrays()
            return
            
        # Reset arrays
        self.distances = np.full(self.num_rays, -1.0, dtype=np.float32)
        self.agent_ids = np.full(self.num_rays, -1, dtype=np.int16)
        
        # Process each person
        for person in self.latest_people.people:
            # Get agent data from loaded configuration
            if person.name in self.agent_name_to_data:
                # Use loaded configuration data
                agent_id, group_id = self.agent_name_to_data[person.name]
            else:
                self.get_logger().warn(f'Agent name {person.name} not found in loaded configuration. Skipping.')
                continue
            # Transform person position
            robot_point = self.transform_point(person.position)
            if robot_point is None:
                continue
                
            # Compute bearing and distance
            bearing, distance = self.compute_bearing_and_distance(robot_point)
            
            # Get angular bins for the agent
            bins = self.get_angular_bins_for_agent(bearing, distance)
            
            # Update observation arrays
            for bin_idx in bins:
                # Update iff empty or current agent is closer
                if self.distances[bin_idx] < 0 or distance < self.distances[bin_idx]:
                    self.distances[bin_idx] = distance
                    self.agent_ids[bin_idx] = agent_id

        # Publish observations
        self.publish_arrays()



    def transform_point(self, point: Point):
        try:
            # Create a PointStamped message
            point_stamped = tf2_geometry_msgs.PointStamped()
            point_stamped.header.frame_id = self.latest_people.header.frame_id
            point_stamped.point = point
            
            # Transform the point using cached transform
            transformed_point = tf2_geometry_msgs.do_transform_point(point_stamped, self.latest_transform)
            
            return transformed_point.point
            
        except Exception as e:
            self.get_logger().debug(f'TF2 point transformation failed: {e}')
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
    def publish_arrays(self):
        # Publish distances
        distances_msg = Float32MultiArray()
        distances_msg.data = self.distances.tolist()
        
        self.distances_publisher.publish(distances_msg)

        # Publish agent IDs
        agent_ids_msg = Int16MultiArray()
        agent_ids_msg.data = self.agent_ids.tolist()

        self.agent_ids_publisher.publish(agent_ids_msg)

        # Publish the mapping as a JSON string
        mapping_json = json.dumps(self.agent_name_to_data)
        if self.last_mapping_json != mapping_json:
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
