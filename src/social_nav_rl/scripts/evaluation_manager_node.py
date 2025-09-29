#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import yaml
import os
import time
from typing import Dict, List, Optional
from dataclasses import dataclass

from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import String, Bool
from hunav_msgs.srv import StartEvaluation
from std_srvs.srv import Empty
from ament_index_python.packages import get_package_share_directory

@dataclass
class SpawnConfig:
    x: float
    y: float  
    z: float
    yaw: float

@dataclass
class GoalConfig:
    x: float
    y: float

@dataclass  
class ScenarioConfig:
    name: str
    description: str
    environment_name: str
    robot_spawn: SpawnConfig
    goals: List[GoalConfig]
    agent_config: str


class EvaluationManagerNode(Node):
    def __init__(self):
        super().__init__('evaluation_manager')
        
        # Parameters
        self.experiment_tag = self.declare_parameter('experiment_tag', 'eval_test').value
        self.num_episodes = self.declare_parameter('num_episodes', 10).value
        self.scenario_config_file = self.declare_parameter('scenario_config', 'default_scenarios.yaml').value
        
        # State tracking
        self.current_episode = 0
        self.current_scenario_idx = 0
        self.current_goal_idx = 0
        self.recording_active = False
        self.episode_start_time = None
        self.scenarios: List[ScenarioConfig] = []
        
        # Service clients for hunav_evaluator
        self.start_recording_client = self.create_client(StartEvaluation, 'hunav_start_recording')
        self.stop_recording_client = self.create_client(Empty, 'hunav_stop_recording')
        
        # Publishers
        self.episode_command_pub = self.create_publisher(String, '/evaluation/episode_command', 10)
        self.goal_pub = self.create_publisher(PoseStamped, '/evaluation/current_goal', 10)
        
        # Subscribers  
        self.episode_status_sub = self.create_subscription(
            String, '/evaluation/episode_status', self.episode_status_callback, 10)
        
        # Load scenario configuration
        self.load_scenarios()
        
        # Wait for services
        self.get_logger().info("Waiting for hunav_evaluator services...")
        self.start_recording_client.wait_for_service(timeout_sec=30.0)
        self.stop_recording_client.wait_for_service(timeout_sec=30.0)
        self.get_logger().info("Services connected")
        
        # Start evaluation after brief delay
        self.startup_timer = self.create_timer(3.0, self.start_evaluation_sequence)
        
    def load_scenarios(self):
        try:
            # Find scenario config file
            if not os.path.isabs(self.scenario_config_file):
                pkg_dir = get_package_share_directory('social_nav_rl')
                config_path = os.path.join(pkg_dir, 'config', self.scenario_config_file)
            else:
                config_path = self.scenario_config_file
                
            with open(config_path, 'r') as f:
                config_data = yaml.safe_load(f)
                
            for scenario_data in config_data['scenarios']:
                spawn_data = scenario_data['robot_spawn']
                spawn_config = SpawnConfig(
                    x=spawn_data['x'], y=spawn_data['y'], 
                    z=spawn_data['z'], yaw=spawn_data['yaw']
                )
                
                goals = [GoalConfig(x=g['x'], y=g['y']) for g in scenario_data['goals']]
                
                scenario = ScenarioConfig(
                    name=scenario_data['name'],
                    description=scenario_data['description'],
                    environment_name=scenario_data['environment_name'],
                    robot_spawn=spawn_config,
                    goals=goals,
                    agent_config=scenario_data['agent_config']
                )
                self.scenarios.append(scenario)
                
            self.get_logger().info(f"Loaded {len(self.scenarios)} scenarios")
            
        except Exception as e:
            self.get_logger().error(f"Failed to load scenarios: {e}")
            # Fallback to single scenario
            self.scenarios = [ScenarioConfig(
                name="fallback",
                description="Single fallback scenario",
                environment_name="default",
                robot_spawn=SpawnConfig(0.0, 0.0, 0.25, 0.0),
                goals=[GoalConfig(3.0, 0.0)],
                agent_config="agents_experimenting.yaml"
            )]
            
    def start_evaluation_sequence(self):
        self.startup_timer.cancel()  # Cancel the startup timer
        self.evaluation_timer = self.create_timer(2.0, self.evaluation_timer_callback)
        self.get_logger().info(f"Starting evaluation: {self.num_episodes} episodes")
        
    def evaluation_timer_callback(self):
        if self.current_episode >= self.num_episodes:
            self.get_logger().info("Evaluation complete!")
            self.evaluation_timer.cancel()
            return
            
        # Check for episode timeout
        if self.recording_active and self.episode_start_time:
            episode_duration = time.time() - self.episode_start_time
            if episode_duration > 60.0:  # 60 second timeout
                self.get_logger().info(f"Episode {self.current_episode + 1} timed out after {episode_duration:.1f}s")
                self.episode_status_callback(String(data="truncated"))
                return
            
        if not self.recording_active:
            self.start_next_episode()
            
    def start_next_episode(self):
        # Select scenario and goal
        scenario = self.scenarios[self.current_scenario_idx % len(self.scenarios)]
        goal = scenario.goals[self.current_goal_idx % len(scenario.goals)]
        
        self.get_logger().info(
            f"Episode {self.current_episode + 1}/{self.num_episodes}: "
            f"Scenario '{scenario.name}', Goal ({goal.x}, {goal.y})"
        )
        
        # Send reset command to policy evaluation node
        reset_cmd = String()
        reset_cmd.data = f"reset:{scenario.robot_spawn.x},{scenario.robot_spawn.y},{scenario.robot_spawn.z},{scenario.robot_spawn.yaw}"
        self.episode_command_pub.publish(reset_cmd)
        
        # Publish goal
        goal_msg = PoseStamped()
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.header.frame_id = "map"
        goal_msg.pose.position.x = goal.x
        goal_msg.pose.position.y = goal.y
        goal_msg.pose.position.z = 0.0
        goal_msg.pose.orientation.w = 1.0
        self.goal_pub.publish(goal_msg)
        
        # Since hunav_evaluator auto-starts recording, skip explicit start
        # Just assume recording is active and proceed
        time.sleep(1.0)  # Wait for reset to complete
        self.recording_active = True
        self.episode_start_time = time.time()
        self.get_logger().info(f"Episode {self.current_episode + 1} started (assuming hunav_evaluator recording)")
        
        # Send start command
        start_cmd = String()
        start_cmd.data = "start"
        self.episode_command_pub.publish(start_cmd)
        
    def start_metric_recording(self, goal_msg: PoseStamped, scenario_name: str):
        req = StartEvaluation.Request()
        req.robot_goal = goal_msg
        req.experiment_tag = f"{self.experiment_tag}_{scenario_name}"
        req.run_id = self.current_episode
        
        future = self.start_recording_client.call_async(req)
        future.add_done_callback(self.start_recording_callback)
        
    def start_recording_callback(self, future):
        try:
            response = future.result()
            # If recording is already active, that's fine - just proceed
            if response.success:
                self.get_logger().info(f"Successfully started recording for episode {self.current_episode + 1}")
                self.recording_active = True
                self.episode_start_time = time.time()
            else:
                # Recording might already be active, proceed anyway
                self.get_logger().info(f"Recording setup returned: {response.success}. Proceeding with evaluation.")
                self.recording_active = True
                self.episode_start_time = time.time()
        except Exception as e:
            self.get_logger().error(f"Start recording service failed: {e}")
            # Assume recording is working and continue anyway
            self.get_logger().info("Assuming hunav_evaluator is already recording. Proceeding.")
            self.recording_active = True
            self.episode_start_time = time.time()
            
    def episode_status_callback(self, msg):
        if msg.data in ["terminated", "truncated", "completed"]:
            if self.recording_active:
                self.stop_metric_recording()
                
    def stop_metric_recording(self):
        req = Empty.Request()
        future = self.stop_recording_client.call_async(req)
        future.add_done_callback(self.stop_recording_callback)
        
    def stop_recording_callback(self, future):
        try:
            future.result()  # Wait for completion
            self.recording_active = False
            episode_duration = time.time() - self.episode_start_time if self.episode_start_time else 0
            
            self.get_logger().info(
                f"Stopped recording for episode {self.current_episode}, "
                f"duration: {episode_duration:.1f}s"
            )
            
            # Move to next episode
            self.current_episode += 1
            self.current_goal_idx += 1
            
            # Cycle through scenarios if needed
            total_goals = sum(len(s.goals) for s in self.scenarios)
            if self.current_goal_idx >= len(self.scenarios[self.current_scenario_idx].goals):
                self.current_scenario_idx += 1
                self.current_goal_idx = 0
                
        except Exception as e:
            self.get_logger().error(f"Stop recording service failed: {e}")
            self.recording_active = False


def main(args=None):
    rclpy.init(args=args)
    node = EvaluationManagerNode()
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