#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import time
import os

import matplotlib.pyplot as plt

from collections import deque
import numpy as np
from ament_index_python.packages import get_package_share_directory


class TrainingMonitor(Node):   
    def __init__(self):
        super().__init__('training_monitor')
        
        # Tracking variables
        self.episode_rewards = deque(maxlen=1000)
        self.episode_lengths = deque(maxlen=1000)
        self.training_losses = deque(maxlen=1000)
        self.timestamps = deque(maxlen=1000)
        self.reward_components = deque(maxlen=1000)
        
        # Setup directories
        package_dir = get_package_share_directory('social_nav_rl')
        
        self.log_dir = os.path.join(package_dir, 'logs')
        self.plot_dir = os.path.join(package_dir, 'plots')
        
        os.makedirs(self.log_dir, exist_ok=True)
        os.makedirs(self.plot_dir, exist_ok=True)
                
        self.get_logger().info(f"Training Monitor initialised")
        self.get_logger().info(f"Plots will be saved to: {self.plot_dir}")
        
    def log_episode(self, reward, length, step, reward_components=None):
        self.episode_rewards.append(reward)
        self.episode_lengths.append(length)
        self.timestamps.append(time.time())
        
        if reward_components:
            self.reward_components.append(reward_components)
        
        self.get_logger().info(f"Episode logged: reward={reward:.1f}, length={length}, step={step}")
        
    def log_training_loss(self, loss):
        self.training_losses.append(loss)
        
    def update_plots(self):
        if len(self.episode_rewards) < 5:
            self.get_logger().info(f"Not enough episodes yet ({len(self.episode_rewards)}/5) - skipping plot update")
            return
            
        try:
            self.get_logger().info(f"Updating plots with {len(self.episode_rewards)} episodes...")
            
            # Create figure with subplots
            fig, axes = plt.subplots(2, 3, figsize=(15, 10))
            fig.suptitle('A2C Social Navigation Training Progress')
            
            rewards_array = np.array(list(self.episode_rewards))
            lengths_array = np.array(list(self.episode_lengths))
            
            # Episode rewards
            axes[0, 0].plot(rewards_array, alpha=0.7, color='blue', label='Raw Rewards')
            if len(rewards_array) > 20:
                # Add moving average
                window = min(50, len(rewards_array) // 4)
                moving_avg = np.convolve(rewards_array, np.ones(window)/window, mode='valid')
                axes[0, 0].plot(range(window-1, len(rewards_array)), moving_avg, 'r-', alpha=0.9, linewidth=2, label=f'MA({window})')
            axes[0, 0].set_title('Episode Rewards')
            axes[0, 0].set_xlabel('Episode')
            axes[0, 0].set_ylabel('Reward')
            axes[0, 0].grid(True, alpha=0.3)
            axes[0, 0].legend()
            
            # Episode lengths
            axes[0, 1].plot(lengths_array, alpha=0.7, color='green', label='Episode Length')
            if len(lengths_array) > 20:
                window = min(50, len(lengths_array) // 4)
                moving_avg = np.convolve(lengths_array, np.ones(window)/window, mode='valid')
                axes[0, 1].plot(range(window-1, len(lengths_array)), moving_avg, 'r-', alpha=0.9, linewidth=2, label=f'MA({window})')
            axes[0, 1].set_title('Episode Lengths')
            axes[0, 1].set_xlabel('Episode')
            axes[0, 1].set_ylabel('Steps')
            axes[0, 1].grid(True, alpha=0.3)
            axes[0, 1].legend()
            
            # Reward distribution
            axes[0, 2].hist(rewards_array, bins=20, alpha=0.7, color='skyblue', edgecolor='black')
            mean_reward = np.mean(rewards_array)
            axes[0, 2].axvline(mean_reward, color='r', linestyle='--', linewidth=2,
                             label=f'Mean: {mean_reward:.1f}')
            axes[0, 2].set_title('Reward Distribution')
            axes[0, 2].set_xlabel('Reward')
            axes[0, 2].set_ylabel('Frequency')
            axes[0, 2].grid(True, alpha=0.3)
            axes[0, 2].legend()
            
            # Training losses (if available)
            if len(self.training_losses) > 0:
                axes[1, 0].plot(list(self.training_losses), alpha=0.7, color='orange')
                axes[1, 0].set_title('Training Loss')
                axes[1, 0].set_xlabel('Update Step')
                axes[1, 0].set_ylabel('Loss')
                axes[1, 0].grid(True, alpha=0.3)
            else:
                axes[1, 0].text(0.5, 0.5, 'No loss data available', ha='center', va='center', transform=axes[1, 0].transAxes)
                axes[1, 0].set_title('Training Loss')
            
            # Performance trends (recent vs overall)
            if len(rewards_array) >= 20:
                recent_count = min(100, len(rewards_array) // 4)
                recent_rewards = rewards_array[-recent_count:]
                overall_mean = np.mean(rewards_array)
                recent_mean = np.mean(recent_rewards)
                
                axes[1, 1].bar(['Overall', 'Recent'], [overall_mean, recent_mean], 
                              color=['lightblue', 'darkblue'], alpha=0.7)
                axes[1, 1].set_title(f'Performance Comparison\n(Recent = last {recent_count} episodes)')
                axes[1, 1].set_ylabel('Average Reward')
                axes[1, 1].grid(True, alpha=0.3, axis='y')
                
                # Add value labels on bars
                for i, v in enumerate([overall_mean, recent_mean]):
                    axes[1, 1].text(i, v + abs(v)*0.05, f'{v:.1f}', ha='center', va='bottom')
            else:
                axes[1, 1].text(0.5, 0.5, 'Need more episodes\nfor comparison', ha='center', va='center', transform=axes[1, 1].transAxes)
                axes[1, 1].set_title('Performance Comparison')
            
            # Training statistics
            recent_rewards = rewards_array[-100:] if len(rewards_array) >= 100 else rewards_array
            recent_lengths = lengths_array[-100:] if len(lengths_array) >= 100 else lengths_array
            
            stats_text = f"""Training Statistics
                    Recent ({len(recent_rewards)} episodes):
                    Mean Reward: {np.mean(recent_rewards):.1f} ± {np.std(recent_rewards):.1f}
                    Mean Length: {np.mean(recent_lengths):.0f} ± {np.std(recent_lengths):.0f}

                    Overall ({len(rewards_array)} episodes):
                    Episodes: {len(rewards_array)}
                    Best Reward: {np.max(rewards_array):.1f}
                    Worst Reward: {np.min(rewards_array):.1f}
                    Avg Length: {np.mean(lengths_array):.0f}"""
            
            axes[1, 2].text(0.05, 0.95, stats_text, transform=axes[1, 2].transAxes, 
                           verticalalignment='top', fontsize=10, fontfamily='monospace',
                           bbox=dict(boxstyle="round,pad=0.3", facecolor="lightgray", alpha=0.8))
            axes[1, 2].set_title('Training Statistics')
            axes[1, 2].axis('off')
            
            plt.tight_layout()
            
            # Save plot with timestamp
            timestamp = time.strftime("%Y%m%d_%H%M%S")
            plot_path = os.path.join(self.plot_dir, f'a2c_holo_progress_{timestamp}.png')
            plt.savefig(plot_path, dpi=150, bbox_inches='tight')
            
            # Also save as latest for easy viewing
            latest_plot_path = os.path.join(self.plot_dir, 'a2c_holo_progress_latest.png')
            plt.savefig(latest_plot_path, dpi=150, bbox_inches='tight')
            
            plt.close()
            
            self.get_logger().info(f"Training plot updated: {plot_path}")
            
        except Exception as e:
            self.get_logger().error(f"Error updating plots: {e}")
            import traceback
            self.get_logger().error(traceback.format_exc())

def main():
    rclpy.init()
    
    monitor = TrainingMonitor()
    
    try:
        # Example usage - in practice this would be called by the trainer
        print("Training Monitor running...")
        print("Use Ctrl+C to stop and generate final report")
        
        rclpy.spin(monitor)

    except Exception as e:
        print(f"Error in training monitor: {e}")
        
    finally:
        monitor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
