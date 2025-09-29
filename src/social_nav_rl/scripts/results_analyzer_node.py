#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import pandas as pd
import matplotlib
import matplotlib.pyplot as plt
import os
import glob
from datetime import datetime
from typing import Dict, List, Optional

import seaborn as sns


from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory


class ResultsAnalyzerNode(Node):
    def __init__(self):
        super().__init__('results_analyzer')
        
        # Parameters
        self.experiment_tag = self.declare_parameter('experiment_tag', 'eval_test').value
        self.output_dir = self.declare_parameter('output_dir', '').value
        
        if not self.output_dir:
            pkg_dir = get_package_share_directory('social_nav_rl')
            self.output_dir = os.path.join(pkg_dir, 'results')
            
        os.makedirs(self.output_dir, exist_ok=True)
        
        # Subscriber for analysis trigger
        self.trigger_sub = self.create_subscription(
            String, '/evaluation/analyze_results', self.analyze_callback, 10)
            
        self.get_logger().info(f"Results analyzer ready, output dir: {self.output_dir}")
        
    def analyze_callback(self, msg):
        if msg.data == 'analyze':
            self.analyze_results()
            
    def analyze_results(self):
        try:
            # Find evaluator result files
            hunav_pkg_dir = get_package_share_directory('hunav_evaluator')
            results_dir = os.path.join(hunav_pkg_dir, 'results')
            
            # Look for files matching our experiment tag
            csv_files = glob.glob(os.path.join(results_dir, f'*{self.experiment_tag}*.csv'))
            
            if not csv_files:
                self.get_logger().warn(f"No CSV files found for experiment: {self.experiment_tag}")
                return
                
            main_csv = None
            steps_csvs = []
            
            for csv_file in csv_files:
                if '_steps_' in os.path.basename(csv_file):
                    steps_csvs.append(csv_file)
                else:
                    main_csv = csv_file
                    
            if main_csv is None:
                self.get_logger().warn("No main metrics CSV found")
                return
                
            self.get_logger().info(f"Analyzing results from {main_csv}")
            
            # Load and analyze main metrics
            df_main = pd.read_csv(main_csv)
            self.generate_summary_report(df_main)
            self.generate_plots(df_main, steps_csvs)
            
            self.get_logger().info("Analysis complete")
            
        except Exception as e:
            self.get_logger().error(f"Analysis failed: {e}")
            
    def generate_summary_report(self, df: pd.DataFrame):
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        report_path = os.path.join(self.output_dir, f'summary_{self.experiment_tag}_{timestamp}.txt')
        
        with open(report_path, 'w') as f:
            f.write(f"Social Navigation Evaluation Summary\n")
            f.write(f"Experiment Tag: {self.experiment_tag}\n")
            f.write(f"Generated: {datetime.now()}\n")
            f.write("="*50 + "\n\n")
            
            # Basic statistics
            f.write("BASIC STATISTICS:\n")
            f.write(f"Number of episodes: {len(df)}\n\n")
            
            # Key metrics
            key_metrics = [
                'completed', 'time_to_reach_goal', 'path_length',
                'robot_on_person_collision', 'person_on_robot_collision',
                'avg_distance_to_closest_person', 'minimum_distance_to_people',
                'intimate_space_intrusions', 'personal_space_intrusions', 'social_space_intrusions',
                'group_intimate_space_intrusions', 'group_personal_space_intrusions', 'group_social_space_intrusions'
            ]
            
            f.write("KEY METRICS:\n")
            for metric in key_metrics:
                if metric in df.columns:
                    values = df[metric].dropna()
                    if len(values) > 0:
                        f.write(f"{metric}:\n")
                        f.write(f"  Mean: {values.mean():.4f}\n")
                        f.write(f"  Std:  {values.std():.4f}\n")
                        f.write(f"  Min:  {values.min():.4f}\n")
                        f.write(f"  Max:  {values.max():.4f}\n")
                        f.write("\n")
                        
            # Success rate
            if 'completed' in df.columns:
                success_rate = df['completed'].mean()
                f.write(f"SUCCESS RATE: {success_rate:.2%}\n\n")
                
            # Collision rate
            collision_metrics = ['robot_on_person_collision', 'person_on_robot_collision']
            collision_rate = 0.0
            for col in collision_metrics:
                if col in df.columns:
                    collision_rate += df[col].sum()
            collision_rate = collision_rate / len(df) if len(df) > 0 else 0.0
            f.write(f"COLLISION RATE: {collision_rate:.2%}\n\n")
            
        self.get_logger().info(f"Summary report saved: {report_path}")
        
    def generate_plots(self, df_main: pd.DataFrame, steps_csvs: List[str]):
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        
        # Set up plotting style
        sns.set_style("whitegrid")
        plt.rcParams['figure.figsize'] = (12, 8)
        
        # 1. Success metrics plot
        fig, axes = plt.subplots(2, 2, figsize=(15, 10))
        fig.suptitle(f'Evaluation Results: {self.experiment_tag}', fontsize=16)
        
        # Success rate pie chart
        if 'completed' in df_main.columns:
            success_counts = df_main['completed'].value_counts()
            axes[0,0].pie(success_counts.values, labels=['Failed', 'Success'], autopct='%1.1f%%')
            axes[0,0].set_title('Task Completion Rate')
            
        # Time to goal histogram  
        if 'time_to_reach_goal' in df_main.columns:
            successful_episodes = df_main[df_main['completed'] == True]
            if len(successful_episodes) > 0:
                axes[0,1].hist(successful_episodes['time_to_reach_goal'], bins=10, alpha=0.7)
                axes[0,1].set_xlabel('Time to Goal (s)')
                axes[0,1].set_ylabel('Frequency')
                axes[0,1].set_title('Time to Reach Goal Distribution')
                
        # Space intrusion metrics
        intrusion_cols = [col for col in df_main.columns if 'space_intrusions' in col]
        if intrusion_cols:
            intrusion_data = df_main[intrusion_cols].mean()
            axes[1,0].bar(range(len(intrusion_data)), intrusion_data.values)
            axes[1,0].set_xticks(range(len(intrusion_data)))
            axes[1,0].set_xticklabels([col.replace('_space_intrusions', '') for col in intrusion_cols], 
                                     rotation=45, ha='right')
            axes[1,0].set_ylabel('Average Intrusion %')
            axes[1,0].set_title('Space Intrusion Percentages')
            
        # Distance to people
        if 'avg_distance_to_closest_person' in df_main.columns:
            axes[1,1].hist(df_main['avg_distance_to_closest_person'], bins=10, alpha=0.7)
            axes[1,1].set_xlabel('Distance (m)')
            axes[1,1].set_ylabel('Frequency') 
            axes[1,1].set_title('Average Distance to Closest Person')
            
        plt.tight_layout()
        plot_path = os.path.join(self.output_dir, f'evaluation_plots_{self.experiment_tag}_{timestamp}.png')
        plt.savefig(plot_path, dpi=300, bbox_inches='tight')
        plt.close()
        
        self.get_logger().info(f"Plots saved: {plot_path}")


def main(args=None):
    rclpy.init(args=args)
    node = ResultsAnalyzerNode()
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