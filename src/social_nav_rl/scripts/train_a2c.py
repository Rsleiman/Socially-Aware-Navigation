#!/usr/bin/env python3

import rclpy
import numpy as np
import torch
import time
import os
from datetime import datetime
from collections import deque


from env_social_nav import SocialNavEnvironment
from policy_network import A2CAgent

from ament_index_python.packages import get_package_share_directory
from training_monitor import TrainingMonitor

class A2CTrainer:   
    def __init__(self, config):
        self.config = config
        
        # Training parameters
        self.total_timesteps = config.get('total_timesteps', 100000)
        self.rollout_steps = config.get('rollout_steps', 128)  # Steps per rollout
        self.eval_episodes = config.get('eval_episodes', 5)
        self.eval_freq = config.get('eval_freq', 1000) 
        self.save_freq = config.get('save_freq', 5000)  
        
        # Setup directories
        package_dir = get_package_share_directory('social_nav_rl')

        self.log_dir = os.path.join(package_dir, 'logs')
        self.model_dir = os.path.join(package_dir, 'models')

        os.makedirs(self.log_dir, exist_ok=True)
        os.makedirs(self.model_dir, exist_ok=True)
        
        # Device
        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        print(f"Using device: {self.device}")
        
        # Initialise environment and agent
        print("Initialising environment...")
        self.env = SocialNavEnvironment()
        
        print("Initialising agent...")
        self.agent = A2CAgent(
            obs_dim=962,  
            action_dim=3, 
            learning_rate=config.get('learning_rate', 3e-4),
            device=self.device,
        )
        
        # Training tracking
        self.global_step = 0
        self.episode_count = 0
        self.episode_rewards = deque(maxlen=100)
        self.episode_lengths = deque(maxlen=100)
        
        # Track if we need to initialize first episode
        self.begin = False
        
        # Monitor for plotting
        self.monitor = None
        try:
            self.monitor = TrainingMonitor()
            print("Training monitor enabled - plots will be saved")
        except Exception as e:
            print(f"Could not initialise monitor: {e}")
            print("Training will continue without plotting")
            self.monitor = None
        
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.run_name = f"a2c_holo_{timestamp}"
        
        print(f"Trainer initialised: {self.run_name}")
        print(f"Total timesteps: {self.total_timesteps}")
        print(f"Rollout steps: {self.rollout_steps}")

        # Evaluation and logging files
        self.eval_log_path = os.path.join(self.log_dir, f'{self.run_name}_eval.csv')
        self.episode_log_path = os.path.join(self.log_dir, f'{self.run_name}_episodes.csv')
        
        with open(self.eval_log_path, 'w') as f:
            f.write("step,episode_count,mean_reward,std_reward,mean_length,collision_rate,goal_success_rate,mean_components\n")
        
        with open(self.episode_log_path, 'w') as f:
            f.write("episode,step,reward,length,termination,goal_reached,collision,r_ro,r_rh,r_rs,r_rg,r_rf\n")

    def collect_rollout(self):      
        observations = []
        actions = []
        rewards = []
        dones = []
        values = []
        
        # Get current observation from environment
        obs = self.env.get_full_observation()
        
        # TODO: resets at end of rollout even if episode continues:
        episode_accumulated_reward = 0.0
        episode_reward_components = {'r_ro': 0, 'r_rh': 0, 'r_rs': 0, 'r_rg': 0, 'r_rf': 0}
        
        for step in range(self.rollout_steps):
            
            # Get action and value
            action = self.agent.predict(obs, deterministic=False, action_space=self.env.action_space)
            value = self.agent.network.get_value(
                torch.FloatTensor(obs).unsqueeze(0).to(self.device)
            ).item()
            
            # Store current experience
            observations.append(obs.copy())
            actions.append(action.copy())
            values.append(value)
            
            # Execute action
            next_obs, reward, terminated, truncated, info = self.env.step(action)
            done = terminated or truncated
            
            rewards.append(reward)
            dones.append(done)
            episode_accumulated_reward += reward
            
            # for episode summary:
            if 'reward_components' in info:
                for component, value in info['reward_components'].items():
                    if component in episode_reward_components:
                        episode_reward_components[component] += value
            
            # Update global step
            self.global_step += 1
            
            # Simplified step logging - only every 50 steps during rollout
            if step % 50 == 0:
                self.env.get_logger().info(
                    f"Rollout {step}/{self.rollout_steps}: reward={reward:.2f}, "
                    f"episode_total={episode_accumulated_reward:.1f}, "
                    f"action=[{action[0]:.2f}, {action[1]:.2f}, {action[2]:.2f}]"
                )
            
            # Handle episode end
            if done:
                episode_length = info.get('step', 1)
                
                self.episode_rewards.append(episode_accumulated_reward)
                self.episode_lengths.append(episode_length)
                self.episode_count += 1
                
                # Log to monitor if available  
                if self.monitor:
                    reward_components = info.get('reward_components', {})
                    self.monitor.log_episode(episode_accumulated_reward, episode_length, self.global_step, reward_components)
                
                # Determine termination reason
                termination_reason = "max_steps"
                goal_reached = False
                collision = False
                
                if 'termination_flags' in info:
                    flags = info['termination_flags']
                    if flags.get('goal_reached', False):
                        termination_reason = "goal_reached"  
                        goal_reached = True
                    elif flags.get('collision_human', False):
                        termination_reason = "collision_human"
                        collision = True
                    elif flags.get('collision_obstacle', False):
                        termination_reason = "collision_obstacle"
                        collision = True
                    elif flags.get('robot_flipped', False):
                        termination_reason = "robot_flipped"
                        collision = True
                
                # Log episode summary to file
                reward_components = info.get('reward_components', {})
                with open(self.episode_log_path, 'a') as f:
                    f.write(f"{self.episode_count},{self.global_step},{episode_accumulated_reward:.3f},"
                           f"{episode_length},{termination_reason},{goal_reached},{collision},"
                           f"{reward_components.get('r_ro', 0):.3f},{reward_components.get('r_rh', 0):.3f},"
                           f"{reward_components.get('r_rs', 0):.3f},{reward_components.get('r_rg', 0):.3f},"
                           f"{reward_components.get('r_rf', 0):.3f}\n")
                
                # Enhanced console episode logging - less verbose
                self.env.get_logger().info(
                    f"Episode {self.episode_count}: reward={episode_accumulated_reward:.1f}, "
                    f"length={episode_length}, reason={termination_reason}"
                )
                
                # Reset environment and tracking
                obs, _ = self.env.reset()
                # Reset LSTM states for new episode
                self.agent.reset_lstm_states()
                episode_accumulated_reward = 0.0
                episode_reward_components = {'r_ro': 0, 'r_rh': 0, 'r_rs': 0, 'r_rg': 0, 'r_rf': 0}
            else:
                obs = next_obs
        
        return {
            'observations': np.array(observations),
            'actions': np.array(actions),
            'rewards': np.array(rewards),
            'dones': np.array(dones),
            'values': np.array(values)
        }
    
    def evaluate_agent(self, num_episodes=5):       
        total_rewards = []
        total_lengths = []
        collision_count = 0
        goal_success_count = 0
        component_totals = {'r_ro': 0, 'r_rh': 0, 'r_rs': 0, 'r_rg': 0, 'r_rf': 0}
        
        self.env.get_logger().info(f"Starting evaluation for {num_episodes} episodes...")
        
        for _ in range(num_episodes):
            obs, _ = self.env.reset()
            # Reset LSTM states for evaluation episode
            self.agent.reset_lstm_states()
            episode_reward = 0
            episode_length = 0
            episode_components = {'r_ro': 0, 'r_rh': 0, 'r_rs': 0, 'r_rg': 0, 'r_rf': 0}
            
            while episode_length < 500:
                # Get deterministic actions for evaluation
                action = self.agent.predict(obs, deterministic=True, action_space=self.env.action_space)
                obs, reward, terminated, truncated, info = self.env.step(action)
                
                episode_reward += reward
                episode_length += 1
                
                if 'reward_components' in info:
                    for component, value in info['reward_components'].items():
                        if component in episode_components:
                            episode_components[component] += value
                
                if terminated or truncated:
                    if 'termination_flags' in info:
                        flags = info['termination_flags']
                        if flags.get('goal_reached', False):
                            goal_success_count += 1
                        if flags.get('collision_human', False) or flags.get('collision_obstacle', False):
                            collision_count += 1
                    break
                
            
            total_rewards.append(episode_reward)
            total_lengths.append(episode_length)
            
            # Add to component totals
            for component in component_totals:
                component_totals[component] += episode_components[component]
        
        # Calculate average component values
        avg_components = {comp: total/num_episodes for comp, total in component_totals.items()}
        
        eval_stats = {
            'mean_reward': np.mean(total_rewards),
            'std_reward': np.std(total_rewards),
            'mean_length': np.mean(total_lengths),
            'collision_rate': collision_count / num_episodes,
            'goal_success_rate': goal_success_count / num_episodes,
            'mean_components': avg_components
        }
        
        self.env.get_logger().info(
            f"Evaluation: reward={eval_stats['mean_reward']:.1f}±{eval_stats['std_reward']:.1f}, "
            f"length={eval_stats['mean_length']:.0f}, collision_rate={eval_stats['collision_rate']:.2f}, "
            f"success_rate={eval_stats['goal_success_rate']:.2f}"
        )
        
        return eval_stats
    
    def train(self):
        """Main training loop"""
        
        self.env.get_logger().info("Starting A2C training...")
        
        # Initialize the first episode
        if not self.begin:
            self.env.get_logger().info("Initializing first episode...")
            self.env.reset()
            self.agent.reset_lstm_states()
            self.begin = True
        
        start_time = time.time()
        last_eval_step = 0
        last_save_step = 0
        
        while self.global_step < self.total_timesteps:
            
            # Collect rollout
            rollout_start_time = time.time()
            rollout_data = self.collect_rollout()
            collect_time = time.time() - rollout_start_time
            
            # Update agent
            update_start_time = time.time()
            losses = self.agent.update(
                rollout_data['observations'],
                rollout_data['actions'],
                rollout_data['rewards'],
                rollout_data['dones'],
                rollout_data['values'],
                action_space=self.env.action_space
            )
            update_time = time.time() - update_start_time
            
            # Log training loss to monitor
            if self.monitor:
                self.monitor.log_training_loss(losses.get('total_loss', 0.0))
                
                # Update plots every 10 updates
                if hasattr(self, '_plot_update_counter'):
                    self._plot_update_counter += 1
                else:
                    self._plot_update_counter = 0
                    
                if self._plot_update_counter % 10 == 0:
                    self.monitor.update_plots()
            
            # Enhanced training progress logging
            if len(self.episode_rewards) > 0:
                mean_reward = np.mean(self.episode_rewards)
                mean_length = np.mean(self.episode_lengths)
                
                # Calculate success rate from recent episodes
                recent_episodes = min(20, len(self.episode_rewards))
                
                self.env.get_logger().info(
                    f"Progress: Step {self.global_step}/{self.total_timesteps} | "
                    f"Episodes: {self.episode_count} | "
                    f"Recent Avg Reward: {mean_reward:.1f} | "
                    f"Avg Length: {mean_length:.0f} | "
                    f"Loss: P={losses['policy_loss']:.3f} V={losses['value_loss']:.3f} | "
                    f"Time: {collect_time+update_time:.1f}s"
                )
            
            # Evaluation
            if self.global_step - last_eval_step >= self.eval_freq:
                eval_stats = self.evaluate_agent(self.eval_episodes)
                last_eval_step = self.global_step
                
                # Save enhanced evaluation results
                components_str = ','.join([f"{v:.3f}" for v in eval_stats['mean_components'].values()])
                with open(self.eval_log_path, 'a') as f:
                    f.write(f"{self.global_step},{self.episode_count},"
                           f"{eval_stats['mean_reward']:.3f},{eval_stats['std_reward']:.3f},"
                           f"{eval_stats['mean_length']:.1f},{eval_stats['collision_rate']:.3f},"
                           f"{eval_stats['goal_success_rate']:.3f},{components_str}\n")
            
            # Save model
            if self.global_step - last_save_step >= self.save_freq:
                model_path = os.path.join(self.model_dir, f'{self.run_name}_step_{self.global_step}.pt')
                self.agent.save(model_path)
                self.env.get_logger().info(f"Model saved to {model_path}")
                last_save_step = self.global_step
        
        # Final save and evaluation
        final_model_path = os.path.join(self.model_dir, f'{self.run_name}_final.pt')
        self.agent.save(final_model_path)
        
        final_eval = self.evaluate_agent(10)
        
        total_time = time.time() - start_time
        self.env.get_logger().info(
            f"Training complete! Total time: {total_time:.2f}s | "
            f"Final evaluation: {final_eval['mean_reward']:.3f}±{final_eval['std_reward']:.3f}"
        )
        
        return final_eval


def main():  
    # Initialise ROS2
    rclpy.init()
    
    # Training configuration
    config = {
    'total_timesteps': 500000,
    'rollout_steps': 128,
    'learning_rate': 1e-4,
    'eval_episodes': 3,
    'eval_freq': 2500,
    'save_freq': 5000,
    }
    
    try:
        # Create and run trainer
        trainer = A2CTrainer(config)
        final_results = trainer.train()
        
        print(f"\nTraining Results:")
        print(f"Final Mean Reward: {final_results['mean_reward']:.3f} ± {final_results['std_reward']:.3f}")
        print(f"Final Mean Length: {final_results['mean_length']:.1f}")
        print(f"Final Collision Rate: {final_results['collision_rate']:.2f}")
        print(f"Final Success Rate: {final_results['goal_success_rate']:.2f}")
        
    except Exception as e:
        print(f"\nTraining failed with error: {e}")
    finally:
        print("Shutting down...")
        # Clean up monitor
        if 'trainer' in locals() and trainer.monitor:
            trainer.monitor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
