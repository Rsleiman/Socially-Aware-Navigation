#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import numpy as np
import math

from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray, MultiArrayDimension


class ObstacleDecoderNode(Node):    
    def __init__(self):
        super().__init__('obstacle_decoder_node')

        # Declare Parameters
        self.declare_parameter('num_rays', 240)
        self.declare_parameter('fov_degrees', 180.0)  
        self.declare_parameter('max_range', 10.0)     
        self.declare_parameter('update_rate', 5.0)
        
        # Get parameters
        self.num_rays = self.get_parameter('num_rays').get_parameter_value().integer_value
        self.fov_degrees = self.get_parameter('fov_degrees').get_parameter_value().double_value
        self.max_range = self.get_parameter('max_range').get_parameter_value().double_value
        self.update_rate = self.get_parameter('update_rate').get_parameter_value().double_value
        
        self.fov_radians = math.radians(self.fov_degrees)
        
        # Initialize distance array
        self.distances = np.full(self.num_rays, -1.0, dtype=np.float32)
        
        self.latest_scan = None
        
        self.distances_layout = [MultiArrayDimension()]
        self.distances_layout[0].label = "rays" 
        self.distances_layout[0].size = self.num_rays
        self.distances_layout[0].stride = 1

        # QoS Profile for subscription - ensures we get the latest data if it falls behind
        scan_sub_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Subscribers
        self.scan_subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            scan_sub_qos
        )
        
        # Publishers
        self.distances_publisher = self.create_publisher(
            Float32MultiArray,
            '/social_observation/obstacle_distances',
            10
        )
        
        # Process and publish data at 10Hz #TODO: Tune rate depending on performance vs functionalaity
        self.timer = self.create_timer(
            1.0 / self.update_rate,
            self.process_and_publish
        )
        
        self.get_logger().info(f'Obstacle Decoder Node initialized:')
        self.get_logger().info(f'  - Number of rays: {self.num_rays}')
        self.get_logger().info(f'  - FOV: {self.fov_degrees}°')
        self.get_logger().info(f'  - Max range: {self.max_range}m')
        self.get_logger().info(f'  - Update rate: {self.update_rate}Hz')

    def scan_callback(self, msg):
        self.latest_scan = msg

    def process_and_publish(self):
        if self.latest_scan is None:
            # No scan data yet, Publish empty observations
            self.publish_array()
            return
            
        self.distances.fill(-1.0)
        
        self.process_laser_scan(self.latest_scan)
        self.publish_array()

    def process_laser_scan(self, scan):
        ranges = np.array(scan.ranges)
        
        # simple error checking
        if len(ranges) != self.num_rays:
            self.get_logger().warn(f'Expected {self.num_rays} rays but got {len(ranges)}')
            return
        
        # Create validity mask for ranges
        valid_mask = (
            np.isfinite(ranges) & 
            (ranges >= scan.range_min) & 
            (ranges <= min(self.max_range, scan.range_max))
        )
        
        self.distances[:] = np.where(valid_mask, ranges, -1.0)

    def publish_array(self):    
        distances_msg = Float32MultiArray()
        distances_msg.layout.dim = self.distances_layout
        distances_msg.layout.data_offset = 0
        distances_msg.data = self.distances.tolist()
        
        self.distances_publisher.publish(distances_msg)


def main(args=None):
    rclpy.init(args=args)

    node = ObstacleDecoderNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
