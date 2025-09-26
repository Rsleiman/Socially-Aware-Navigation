#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty
from std_msgs.msg import Float32MultiArray
import time


class GazeboMonitor(Node):
    def __init__(self):
        super().__init__('gazebo_monitor')
        
        # Parameters
        self.declare_parameter('data_timeout', 2.0)    
        self.declare_parameter('check_frequency', 1.0) 
        
        self.data_timeout = self.get_parameter('data_timeout').value
        self.check_frequency = self.get_parameter('check_frequency').value
        
        self.last_data_time = None
        self.pause_detected = False
        
        #  this stops when Gazebo pauses
        self.data_sub = self.create_subscription(
            Float32MultiArray,
            '/social_observation/fused_object_array',
            self.data_callback,
            10
        )
        
        self.unpause_client = self.create_client(Empty, '/unpause_physics')
        
        self.timer = self.create_timer(
            1.0 / self.check_frequency, 
            self.check_data_timeout
        )
        
        self.get_logger().info("Waiting for Gazebo unpause service...")
        self.wait_for_services()
        
    def data_callback(self, msg):
        self.last_data_time = time.time()
        if self.pause_detected:
            self.get_logger().info("Data flow resumed. Gazebo is running")
            self.pause_detected = False
        
    def wait_for_services(self):
        while not self.unpause_client.wait_for_service(timeout_sec=1.0):
            if not rclpy.ok():
                return
            self.get_logger().warn("Waiting for /unpause_physics service...")
            
        self.get_logger().info("Gazebo unpause service is available")
    
    def check_data_timeout(self):
        if self.last_data_time is None:
            return
            
        current_time = time.time()
        time_since_data = current_time - self.last_data_time
        
        if time_since_data > self.data_timeout:
            # self.get_logger().info("NOTE: Time since last fused array is larger than timeout")
            if not self.pause_detected:
                self.get_logger().warn(f"No data received for {time_since_data:.1f}s - Gazebo likely paused")
                self.pause_detected = True
                self.attempt_unpause()            
        # else:
        #     self.get_logger().info(f"Current time: {current_time:.2f}, Last data time: {self.last_data_time:.2f}, Time since data: {time_since_data:.1f}s")

    def attempt_unpause(self):     
        request = Empty.Request()
        future = self.unpause_client.call_async(request)        
        future.add_done_callback(self.unpause_callback)
        
    def unpause_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info("Successfully called /unpause_physics service")
            
        except Exception as e:
            self.get_logger().error(f"Failed to call /unpause_physics service: {str(e)}")


def main(args=None):
    rclpy.init(args=args)
    
    try:
        monitor = GazeboMonitor()
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error in gazebo monitor: {e}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
