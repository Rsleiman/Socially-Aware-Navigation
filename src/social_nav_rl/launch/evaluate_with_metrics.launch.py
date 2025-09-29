#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    
    # Evaluation-specific launch arguments
    model_path_arg = DeclareLaunchArgument(
        'model_path', 
        default_value='latest',
        description='Path to trained model (.pt file) or "latest" for most recent'
    )
    
    experiment_tag_arg = DeclareLaunchArgument(
        'experiment_tag',
        default_value='eval_test',
        description='Tag for experiment identification in metrics'
    )
    
    num_episodes_arg = DeclareLaunchArgument(
        'num_episodes',
        default_value='10',
        description='Number of evaluation episodes to run'
    )
    
    scenario_config_arg = DeclareLaunchArgument(
        'scenario_config',
        default_value='default_scenarios.yaml',
        description='YAML file containing scenario configurations'
    )
    
    action_mode_arg = DeclareLaunchArgument(
        'action_mode',
        default_value='holonomic',
        description='Action mode: holonomic or nonholonomic'
    )
    
    # Package directories
    social_nav_rl_share = FindPackageShare('social_nav_rl')
    social_nav_planner_share = FindPackageShare('social_nav_planner')
    
    # Include the complete simulation launch file
    simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([social_nav_planner_share, 'launch', 'simulation.launch.py'])
        ]),
        launch_arguments={
            'configuration_file': 'agents_experimenting.yaml',
            'environment_name': 'default',
            'use_rviz': 'false',
            'spawn_go2': 'true',
            'use_sim_time': 'true',
        }.items()
    )
    
    # HuNav Evaluator with custom metrics config
    hunav_evaluator_node = Node(
        package='hunav_evaluator',
        executable='hunav_evaluator_node',
        name='hunav_evaluator_evaluation',
        parameters=[
            PathJoinSubstitution([social_nav_rl_share, 'config', 'eval_metrics.yaml'])
        ],
        output='screen'
    )
    
    # Evaluation Manager Node
    evaluation_manager_node = Node(
        package='social_nav_rl',
        executable='evaluation_manager_node.py',
        name='evaluation_manager',
        parameters=[{
            'experiment_tag': LaunchConfiguration('experiment_tag'),
            'num_episodes': LaunchConfiguration('num_episodes'),
            'scenario_config': LaunchConfiguration('scenario_config'),
            'use_sim_time': True,
        }],
        output='screen'
    )
    
    # Policy Evaluation Node
    policy_evaluation_node = Node(
        package='social_nav_rl',
        executable='policy_evaluation_node.py',
        name='policy_evaluation',
        parameters=[{
            'model_path': LaunchConfiguration('model_path'),
            'action_mode': LaunchConfiguration('action_mode'),
            'use_sim_time': True,
            'deterministic': True,
        }],
        output='screen'
    )
    
    # Results Analysis Node
    results_analyzer_node = Node(
        package='social_nav_rl',
        executable='results_analyzer_node.py',
        name='results_analyzer',
        parameters=[{
            'experiment_tag': LaunchConfiguration('experiment_tag'),
            'output_dir': PathJoinSubstitution([social_nav_rl_share, 'results']),
        }],
        output='screen'
    )
    
    return LaunchDescription([
        # Arguments
        model_path_arg,
        experiment_tag_arg, 
        num_episodes_arg,
        scenario_config_arg,
        action_mode_arg,
        
        # Include simulation environment
        simulation_launch,
        
        # Evaluation nodes
        hunav_evaluator_node,
        evaluation_manager_node,
        policy_evaluation_node,
        results_analyzer_node,
    ])
