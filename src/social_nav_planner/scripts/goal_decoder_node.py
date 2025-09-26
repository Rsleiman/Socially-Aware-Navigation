#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import math
import numpy as np
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PoseStamped, Point
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Float32MultiArray


#TODO: Determine if we should occlude goals behind obstacles.
#   Leaning towards no because if we assumed localisation, the robot should be aware of where the goal is even if it is occluded.
#TODO: Determine if we should only consider goals within a certain distance.
#   Leaning towards no because the RL policy needs goal information at all distances.
#TODO: Determine if we should consider the goal as a point or a pose with orientation.
#   Leaning towards point because the RL policy only needs a direction and distance to the goal.

class GoalDecoder(Node):
    """
    ROS2 node that
    - Subcribes to the goal_pose topic and extracts the pose (or position)
    - Callback: assigns the pose to a self.goal_pose variable
    - Every 5Hz, transforms the pose to the robot's front_laser frame
    - Determines if the goal is within the laser's field of view
    - If so, determines which laser ray it corresponds to (index 0-239)
    - Publishes the full goal array (240,)
        - If goal in FOV: -1's except for the index corresponding to the goal ray which becomes the distance to the goal  
        - If goal not in FOV: all -1's
    - Publishes to /social_observation/goal_distances as Float32MultiArray
    """

    def __init__(self):
        super().__init__('goal_decoder')

        # Declare Parameters #
        # Laser parameters
        self.declare_parameter('num_rays', 240)
        self.declare_parameter('fov_degrees', 180.0)
        self.declare_parameter('max_range', 10.0)
        self.declare_parameter('update_rate', 5.0)  # Hz

        # Transform parameters
        self.declare_parameter('robot_frame', 'front_laser')
        self.declare_parameter('world_frame', 'map')

        # Get parameters
        self.num_rays = self.get_parameter('num_rays').get_parameter_value().integer_value
        self.fov_degrees = self.get_parameter('fov_degrees').get_parameter_value().double_value
        self.max_range = self.get_parameter('max_range').get_parameter_value().double_value
        self.update_rate = self.get_parameter('update_rate').get_parameter_value().double_value
        self.robot_frame = self.get_parameter('robot_frame').get_parameter_value().string_value
        self.world_frame = self.get_parameter('world_frame').get_parameter_value().string_value

        self.fov_radians = math.radians(self.fov_degrees)

        # Calculate angle resolution
        self.angular_resolution = self.fov_radians / self.num_rays

        # Initialize goal distances array
        self.goal_distances = np.full(self.num_rays, -1.0, dtype=np.float32)

        # Latest received goal pose
        self.latest_goal_pose = None
        self.latest_transform = None

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # QoS Profile for subscription - ensures we get the latest data if it falls behind
        people_sub_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscribers
        self.goal_subscriber = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_callback,
            people_sub_qos
        )

        # Publishers
        self.goal_distances_publisher = self.create_publisher(
            Float32MultiArray,
            '/social_observation/goal_distances',
            10
        )

        # Timer to process and publish data at specified update rate
        self.timer = self.create_timer(
            1.0 / self.update_rate,
            self.process_and_publish
        )

    def goal_callback(self, msg):
        self.latest_goal_pose = msg
        # self.get_logger().info(f"Received new goal pose: {msg}")  # For debugging

    def process_and_publish(self):
        if self.latest_goal_pose is None:
            # self.get_logger().warn("No goal pose received yet.")
            return

        try:
            # Get the latest transform from world frame to robot frame
            self.latest_transform = self.tf_buffer.lookup_transform(
                self.robot_frame,
                self.world_frame,
                rclpy.time.Time()
            )
        except Exception as e:
            self.get_logger().error(f"Transform lookup failed: {e}")
            self.publish_array()  # Publish empty array if transform fails
            return

        # Transform the goal pose to the robot's frame
        transformed_goal_point = self.transform_point(self.latest_goal_pose.pose.position)
        if transformed_goal_point is None:
            self.get_logger().warn("Goal point transformation failed. Skipping publish.")
            return

        # Extract x, y coordinates of the goal in the robot's frame
        transformed_goal_x = transformed_goal_point.x
        transformed_goal_y = transformed_goal_point.y

        # Calculate distance and angle to the goal
        distance_to_goal = math.sqrt(transformed_goal_x**2 + transformed_goal_y**2)
        angle_to_goal = math.atan2(transformed_goal_y, transformed_goal_x)  # Angle in radians


        # Reset all distances to -1
        self.goal_distances.fill(-1.0)

        # Assign goal distance in closest ray if the goal is within laser's FOV
        if ((-self.fov_radians/2) <= angle_to_goal <= (self.fov_radians/2)):

            # Determine which ray corresponds to this angle
            ray_index = int((angle_to_goal + (self.fov_radians/2)) / self.angular_resolution)
            ray_index = min(max(ray_index, 0), self.num_rays - 1)  # Clamp index to valid range

            # Update the goal distances array
            self.goal_distances[ray_index] = distance_to_goal

            self.publish_array()

            # For debugging
            # self.get_logger().info(f"Published goal distances: {self.goal_distances}")

        else:
            # self.get_logger().info("Goal is outside the laser's field of view.")
            self.publish_array()  # Publish array with all -1s if goal is out of FOV
            

    def transform_point(self, point: Point):
        try:
            # Create a PointStamped message
            point_stamped = tf2_geometry_msgs.PointStamped()
            point_stamped.header.frame_id = self.world_frame
            point_stamped.header.stamp = rclpy.time.Time().to_msg() 
            point_stamped.point = point

            # self.get_logger().info(f"Transforming point: {point_stamped}")

            # Transform the point using cached transform
            transformed_point = tf2_geometry_msgs.do_transform_point(point_stamped, self.latest_transform)
            
            return transformed_point.point
            
        except Exception as e:
            self.get_logger().debug(f'TF2 point transformation failed: {e}')
            return None
        

    def publish_array(self):
        goal_distances_msg = Float32MultiArray()
        goal_distances_msg.data = self.goal_distances.tolist()
        self.goal_distances_publisher.publish(goal_distances_msg)
    

def main(args=None):
    rclpy.init(args=args)

    node = GoalDecoder()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()