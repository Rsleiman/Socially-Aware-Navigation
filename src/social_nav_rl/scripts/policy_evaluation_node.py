#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
import torch
import os
import glob
from typing import Optional

from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from ament_index_python.packages import get_package_share_directory

from policy_network import A2CAgent
from env_social_nav import SocialNavEnvironment


class PolicyEvaluationNode(Node):
    def __init__(self):
        super().__init__('policy_evaluation')
        
        # Parameters
        self.model_path = self.declare_parameter('model_path', 'latest').value
        self.action_mode = self.declare_parameter('action_mode', 'holonomic').value
        self.deterministic = self.declare_parameter('deterministic', True).value
        
        # State
        self.agent: Optional[A2CAgent] = None
        self.current_goal: Optional[PoseStamped] = None
        self.episode_active = False
        self.step_count = 0
        
        # Create simplified environment for observation processing
        self.env = SocialNavEnvironment(
            node_name='policy_eval_env',
            action_mode=self.action_mode
        )
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/rl_vel', 10)
        self.status_pub = self.create_publisher(String, '/evaluation/episode_status', 10)
        
        # Subscribers
        self.command_sub = self.create_subscription(
            String, '/evaluation/episode_command', self.command_callback, 10)
        self.goal_sub = self.create_subscription(
            PoseStamped, '/evaluation/current_goal', self.goal_callback, 10)
        
        # Load model
        self.load_model()
        
        # Allow some time for topics to connect
        self.get_logger().info("Waiting 5 seconds for topics to connect...")
        self.startup_timer = self.create_timer(5.0, self.delayed_start)
        
    def delayed_start(self):
        """Start the control timer after a delay"""
        print("DEBUG Timer: delayed_start called")
        self.control_timer = self.create_timer(0.1, self.control_callback)  # 10Hz
        print("DEBUG Timer: Control timer created successfully at 10Hz")
        print(f"DEBUG Timer: Timer object: {self.control_timer}")
        self.get_logger().info("Policy evaluation node ready - control timer started at 10Hz")
        
    def load_model(self):
        try:
            if self.model_path == 'latest':
                model_path = self.find_latest_model()
            else:
                model_path = self.model_path
                
            if not os.path.exists(model_path):
                raise FileNotFoundError(f"Model file not found: {model_path}")
                
            # Initialize agent matching training configuration
            self.agent = A2CAgent(
                obs_dim=726,  # Match training config
                action_dim=3 if self.action_mode == 'holonomic' else 2,
                device='cuda' if torch.cuda.is_available() else 'cpu'
            )
            
            self.agent.load(model_path)
            self.get_logger().info(f"Loaded model: {model_path}")
            
        except Exception as e:
            self.get_logger().error(f"Failed to load model: {e}")
            raise
            
    def find_latest_model(self) -> str:
        pkg_dir = get_package_share_directory('social_nav_rl')
        model_dir = os.path.join(pkg_dir, 'models')
        
        # Look for final models first, then latest step models
        final_models = glob.glob(os.path.join(model_dir, '*_final.pt'))
        if final_models:
            latest_final = max(final_models, key=os.path.getmtime)
            return latest_final
            
        step_models = glob.glob(os.path.join(model_dir, '*_step_*.pt'))
        if step_models:
            # Sort by step number in filename
            def extract_step(path):
                try:
                    return int(os.path.basename(path).split('_step_')[-1].split('.')[0])
                except:
                    return 0
            latest_step = max(step_models, key=extract_step)
            return latest_step
            
        raise FileNotFoundError("No model files found in models directory")
        
    def command_callback(self, msg: String):
        if msg.data == "start":
            self.episode_active = True
            self.episode_step = 0
            self.get_logger().info("Episode started")
        elif msg.data == "stop":
            self.episode_active = False
            self.get_logger().info("Episode stopped")
        elif msg.data.startswith("reset:"):
            # Parse reset coordinates (format: "reset:x,y,z,yaw")
            try:
                coords = msg.data.split(":")[1].split(",")
                x, y, z, yaw = map(float, coords)
                self.reset_robot(x, y, z, yaw)
                # Reset episode state
                self.episode_active = False
                self.episode_step = 0
            except (IndexError, ValueError) as e:
                self.get_logger().error(f"Error parsing reset command: {e}")
        else:
            self.get_logger().warn(f"Unknown command: {msg.data}")
            
    def goal_callback(self, msg):
        self.current_goal = msg
        # Update environment's goal
        if hasattr(self.env, 'current_goal_position'):
            self.env.current_goal_position = [msg.pose.position.x, msg.pose.position.y]
            
    def reset_robot(self, x: float, y: float, z: float, yaw: float):
        self.get_logger().info(f"Resetting robot to ({x:.2f}, {y:.2f}, {z:.2f}, {yaw:.2f})")
        
        # Use environment's teleport functionality
        self.env.teleport_robot_to_origin(x, y, z, yaw)
        
        # Stop any movement
        self.publish_zero_velocity()
        self.episode_active = False
        
    def control_callback(self):
        if not self.episode_active:
            return
            
        if self.agent is None:
            self.get_logger().warn("Agent not loaded, skipping control")
            return
            
        try:
            # Get observation from environment
            obs = self.env.get_full_observation()
            
            # Check if observation is valid
            if obs is None:
                self.get_logger().warn("Observation is None, cannot proceed")
                return
                
            # Get action from policy
            action, _ = self.agent.predict(
                obs, 
                deterministic=self.deterministic,
                action_space=self.env.action_space
            )
            
            # Publish action as cmd_vel
            cmd = Twist()
            cmd.linear.x = float(action[0])
            if self.action_mode == 'holonomic':
                cmd.linear.y = float(action[1])
                cmd.angular.z = float(action[2])
            else:
                cmd.linear.y = 0.0
                cmd.angular.z = float(action[1])
                
            self.cmd_vel_pub.publish(cmd)
            self.step_count += 1
            
            # Check termination conditions using environment's reward calculator
            if self.step_count % 5 == 0:  # Check every 5 steps to avoid overload
                try:
                    _, _, termination_flags = self.env.calculate_reward_and_termination_conditions()
                    
                    if any(termination_flags.values()):
                        self.episode_active = False
                        self.publish_zero_velocity()
                        
                        status_msg = String()
                        if termination_flags.get('goal_reached', False):
                            status_msg.data = "completed"
                        else:
                            status_msg.data = "terminated"
                        self.status_pub.publish(status_msg)
                        
                        self.get_logger().info(f"Episode terminated: {termination_flags}")
                except Exception as e:
                    self.get_logger().debug(f"Termination check failed (environment not ready): {e}")

            # Timeout condition      
            if self.step_count > 1200:
                self.episode_active = False
                self.publish_zero_velocity()
                
                status_msg = String()
                status_msg.data = "truncated"
                self.status_pub.publish(status_msg)
                
                self.get_logger().info("Episode truncated due to timeout")
                
        except Exception as e:
            self.get_logger().error(f"Control callback error: {e}")
            
    def publish_zero_velocity(self):
        cmd = Twist()
        self.cmd_vel_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = PolicyEvaluationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()