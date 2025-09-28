#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import math


class GroundTruthAMCL(Node):
    def __init__(self):
        super().__init__("groundtruth_amcl")

        self.latest_pose = None
        
        # Subscriber to Gazebo's ground truth odom
        self.odom_sub = self.create_subscription(
            Odometry,
            "/odom/ground_truth",
            self.odom_callback,
            10
        )
        
        # Create AMCL publisher
        self.amcl_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            "/amcl_pose",
            10
        )
        
        # map to odom transform
        self.tf_broadcaster = TransformBroadcaster(self)
        
        self.get_logger().info("GroundTruth AMCL node started")

    # Publish GT odom as AMCL pose to bypass localization
    def odom_callback(self, msg):
        self.latest_pose = msg.pose.pose
        self.latest_timestamp = msg.header.stamp
        
        amcl_pose = PoseWithCovarianceStamped()
        amcl_pose.header.stamp = msg.header.stamp
        amcl_pose.header.frame_id = "map"
        
        amcl_pose.pose.pose = msg.pose.pose
         
        amcl_pose.pose.covariance = [
            0.01, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.01, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.01, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.01, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.01, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.01
        ]
        
        self.amcl_pub.publish(amcl_pose)
        # self.get_logger().info(f"Published AMCL pose")

        transform = TransformStamped()
        transform.header.stamp = msg.header.stamp
        transform.header.frame_id = "map"
        transform.child_frame_id = "odom"
        transform.transform.translation.x = msg.pose.pose.position.x
        transform.transform.translation.y = msg.pose.pose.position.y
        transform.transform.translation.z = msg.pose.pose.position.z
        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = 0.0
        transform.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(transform)
        # self.get_logger().info(f"Published map->odom transform")

        # # Log occasionally for debugging
        # if hasattr(self, "_log_counter"):
        #     self._log_counter += 1
        # else:
        #     self._log_counter = 0
            
        # if self._log_counter % 100 == 0:  # Log every 100 messages
        #     # Simple quaternion to yaw conversion
        #     q = msg.pose.pose.orientation
        #     yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        #     self.get_logger().info(f"Published ground truth pose: x={msg.pose.pose.position.x:.2f}, "
        #                          f"y={msg.pose.pose.position.y:.2f}, "
        #                          f"yaw={yaw:.2f}")



def main(args=None):
    rclpy.init(args=args)
    
    groundtruth_amcl = GroundTruthAMCL()
    
    try:
        rclpy.spin(groundtruth_amcl)
    except KeyboardInterrupt:
        pass
    
    groundtruth_amcl.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
