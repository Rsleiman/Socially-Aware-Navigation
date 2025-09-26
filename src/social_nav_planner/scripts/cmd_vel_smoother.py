#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
import numpy as np
from collections import deque
import time


class CmdVelSmoother(Node):    
    def __init__(self):
        super().__init__('cmd_vel_smoother')
        
        # Declare parameters
        self.declare_parameter('publish_rate', 20.0)  # Hz - Higher rate for smoother control
        self.declare_parameter('timeout', 2.0)  # seconds - Stop if no commands for this long
        self.declare_parameter('max_linear_accel', 2.0)
        self.declare_parameter('max_angular_accel', 4.0)
        
        # Velocity limits - based on gait.yaml for Go2
        self.declare_parameter('max_linear_x', 0.8)
        self.declare_parameter('max_linear_y', 0.6)
        self.declare_parameter('max_angular_z', 0.8)  

        # (0.0 = no smoothing, 1.0 = max smoothing)
        self.declare_parameter('smoothing_factor', 0.15)
        
        # Get parameters
        self.publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        self.timeout = self.get_parameter('timeout').get_parameter_value().double_value
        self.max_linear_accel = self.get_parameter('max_linear_accel').get_parameter_value().double_value
        self.max_angular_accel = self.get_parameter('max_angular_accel').get_parameter_value().double_value
        self.max_linear_x = self.get_parameter('max_linear_x').get_parameter_value().double_value
        self.max_linear_y = self.get_parameter('max_linear_y').get_parameter_value().double_value
        self.max_angular_z = self.get_parameter('max_angular_z').get_parameter_value().double_value
        self.smoothing_factor = self.get_parameter('smoothing_factor').get_parameter_value().double_value
        
        # profile for real-time control
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Subscriber to raw RL commands
        self.rl_vel_sub = self.create_subscription(
            Twist,
            '/rl_vel',
            self.rl_vel_callback,
            qos_profile
        )
        
        # Publisher for smoothed commands  
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
        
        # State variables
        self.current_vel = Twist()  
        self.target_vel = Twist()   
        self.last_command_time = time.time()
        self.last_update_time = time.time()
        
        # timer for publishing smoothed commands
        self.timer = self.create_timer(
            1.0 / self.publish_rate,
            self.update_and_publish
        )
        
    def rl_vel_callback(self, msg: Twist):
        self.target_vel.linear.x = np.clip(msg.linear.x, -self.max_linear_x, self.max_linear_x)
        self.target_vel.linear.y = np.clip(msg.linear.y, -self.max_linear_y, self.max_linear_y)
        self.target_vel.linear.z = 0.0  
        
        self.target_vel.angular.x = 0.0 
        self.target_vel.angular.y = 0.0
        self.target_vel.angular.z = np.clip(msg.angular.z, -self.max_angular_z, self.max_angular_z)
        
        self.last_command_time = time.time()
        
    def update_and_publish(self):
        current_time = time.time()
        dt = current_time - self.last_update_time
        self.last_update_time = current_time
        
        # Stop after certain with no commands
        if current_time - self.last_command_time > self.timeout:
            self.target_vel = Twist()
        
        smoothed_vel = Twist()
        
        # Smooth linear velocities
        smoothed_vel.linear.x = self.smooth_and_limit(
            self.current_vel.linear.x, 
            self.target_vel.linear.x, 
            self.max_linear_accel, 
            dt
        )
        
        smoothed_vel.linear.y = self.smooth_and_limit(
            self.current_vel.linear.y, 
            self.target_vel.linear.y, 
            self.max_linear_accel, 
            dt
        )
        
        # Smooth angular velocity
        smoothed_vel.angular.z = self.smooth_and_limit(
            self.current_vel.angular.z, 
            self.target_vel.angular.z, 
            self.max_angular_accel, 
            dt
        )
        
        self.current_vel = smoothed_vel
        self.cmd_vel_pub.publish(smoothed_vel)
        
    def smooth_and_limit(self, current, target, max_accel, dt):
        # apply filter for smoothness
        alpha = self.smoothing_factor
        filtered = alpha * current + (1.0 - alpha) * target
        
        # apply acceleration limiting
        vel_diff = filtered - current
        max_vel_change = max_accel * dt
        
        if abs(vel_diff) > max_vel_change:
            vel_diff = np.sign(vel_diff) * max_vel_change
            
        return current + vel_diff


def main(args=None):
    rclpy.init(args=args)
    
    smoother = CmdVelSmoother()
    
    try:
        rclpy.spin(smoother)
    except KeyboardInterrupt:
        pass
    finally:
        smoother.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
