#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, PointStamped
import tf2_ros
from tf2_geometry_msgs import do_transform_point
import math

# TODO: Are incorporating poses necessary? Could just use clickedpoint.

class GoalPoseBridge(Node):
    """ROS2 node that bridges RViz clicked points and/or initialpose to goal_pose topic
    """
    def __init__(self):
        super().__init__('goal_pose_bridge')
        
        # Goal pose publisher
        self.goal_pose_publisher = self.create_publisher(
            PoseStamped,
            '/goal_pose',
            10
        )
        
        # TF2 components for coordinate transforms
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Subscribe to clicked_point from RViz (Publish Point tool)
        self.clicked_point_subscriber = self.create_subscription(
            PointStamped,
            '/clicked_point',
            self.clicked_point_callback,
            10
        )
        
        # Subscribe to initialpose topic (we can repurpose this as goal input)
        # self.initialpose_subscriber = self.create_subscription(
        #     PoseWithCovarianceStamped,
        #     '/initialpose',
        #     self.initialpose_callback,
        #     10
        # )
        
        self.get_logger().info('Goal Pose Bridge initialized')
        self.get_logger().info('  - Listens to: /clicked_point (from RViz Publish Point tool)')
        self.get_logger().info('  - Listens to: /initialpose (repurposed as goal input)')
        self.get_logger().info('  - Publishes: /goal_pose (for bt_navigator & HuNav)')
        self.get_logger().info('')
        self.get_logger().info('Usage:')
        self.get_logger().info('  1. Use RViz "Publish Point" tool to set goals via clicked points')
        # self.get_logger().info('  2. Or use "2D Pose Estimate" tool (initialpose) as goal input')

    def clicked_point_callback(self, msg):
        """Handle clicked point from RViz Publish Point tool and convert to goal pose"""
        self.get_logger().info(f'Received clicked point: x={msg.point.x:.2f}, y={msg.point.y:.2f}')
        
        # Convert PointStamped to PoseStamped (with zero orientation)
        goal_pose = PoseStamped()
        goal_pose.header = msg.header
        goal_pose.pose.position = msg.point
        goal_pose.pose.orientation.w = 1.0  # Identity quaternion (no rotation)
        
        # Republish to /goal_pose topic
        self.goal_pose_publisher.publish(goal_pose)
        self.get_logger().info('Published clicked point as goal to /goal_pose topic')

    # def initialpose_callback(self, msg):
    #     """Handle initial pose - repurposed as goal input"""
    #     self.get_logger().info(f'Received initialpose (treating as goal): x={msg.pose.pose.position.x:.2f}, y={msg.pose.pose.position.y:.2f}')
        
    #     # Convert PoseWithCovarianceStamped to PoseStamped
    #     goal_pose = PoseStamped()
    #     goal_pose.header = msg.header
    #     goal_pose.pose = msg.pose.pose
        
    #     # Republish to /goal_pose topic
    #     self.goal_pose_publisher.publish(goal_pose)
    #     self.get_logger().info('Published initialpose as goal to /goal_pose topic')

def main(args=None):
    rclpy.init(args=args)
    
    node = GoalPoseBridge()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
