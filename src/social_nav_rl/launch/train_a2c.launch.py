#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    
    # Launch arguments
    declare_environment_name = DeclareLaunchArgument(
        "environment_name", 
        default_value="train",
        description="Name of the environment to load"
    )
    
    declare_agent_config_file = DeclareLaunchArgument(
        "configuration_file", 
        default_value="train_env_1.yaml",
        description="Agent configuration file name"
    )
    
    declare_total_timesteps = DeclareLaunchArgument(
        "total_timesteps",
        default_value="1000000", 
        description="Total training timesteps"
    )
    
    declare_learning_rate = DeclareLaunchArgument(
        "learning_rate",
        default_value="3e-4",
        description="Learning rate for training"
    )
    
    # Get launch configurations
    environment_name = LaunchConfiguration("environment_name")
    agent_config_file = LaunchConfiguration("configuration_file")
    total_timesteps = LaunchConfiguration("total_timesteps")
    learning_rate = LaunchConfiguration("learning_rate")

    # Map file for navigation components
    map_yaml_file = PathJoinSubstitution([
            FindPackageShare("hunav_gazebo_wrapper"),
            "maps",
            PythonExpression(["'", environment_name, ".yaml'"])
        ])
    
    # simulation environment (without nav2)
    simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("social_nav_planner"),
                "launch",
                "simulation.launch.py"
            ])
        ]),
        launch_arguments={
            "use_rviz": "false",  # Wwe launch RViz separately with nav2 config
            "navigation": "true",  # prevents static transform publisher from launching
            "environment_name": environment_name,
            "configuration_file": agent_config_file,
            "use_sim_time": "true",
        }.items()
    )

    # Minimal Nav2 components
    nav2_minimal_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("social_nav_rl"),
                "launch", 
                "nav2_minimal_for_training.launch.py"
            ])
        ]),
        launch_arguments={
            "map": map_yaml_file,
            "use_sim_time": "true",
            "params_file": PathJoinSubstitution([
                FindPackageShare("social_nav_planner"),
                "config",
                "navigation_groundtruth.yaml"
            ]),
            "autostart": "true",
            "use_lifecycle_mgr": "true",
        }.items()
    )

    # Ground truth AMCL node (for localization transforms)
    groundtruth_amcl_node = Node(
        package="social_nav_planner",
        executable="groundtruth_amcl.py",
        name="groundtruth_amcl",
        parameters=[PathJoinSubstitution([
            FindPackageShare("social_nav_planner"),
            "config", 
            "navigation_groundtruth.yaml"
        ])],
        output="screen",
    )

    # RViz2 with Nav2 configuration for visualization
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare("nav2_bringup"),
        "rviz",
        "nav2_default_view.rviz"
    ])
    
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=['-d', rviz_config_file],
        parameters=[{"use_sim_time": True}],
        output="screen"
    )
    
    # A2C Training node
    training_node = Node(
        package="social_nav_rl",
        executable="train_a2c.py",
        name="a2c_trainer",
        output="screen",
        parameters=[{
            "total_timesteps": total_timesteps,
            "learning_rate": learning_rate,
            "use_sim_time": True
        }],
        arguments=["--ros-args", "--log-level", "INFO"]
    )
    
    # Gazebo Monitor node. Automatically unpauses Gazebo if it gets stuck
    gazebo_monitor_node = Node(
        package="social_nav_rl",
        executable="gazebo_monitor.py",
        name="gazebo_monitor",
        output="screen",
        parameters=[{
            "use_sim_time": False,      # Must be false to detect Gazebo pause!!
            "data_timeout": 1.0,        # Attempt Unpause if no data for 1 second
            "check_frequency": 1.0      
        }],
        arguments=["--ros-args", "--log-level", "INFO"]
    )
    
    # Start training after simulation is ready
    training_timer = TimerAction(
        period=20.0,  # Wait for simulation to stabilize
        actions=[training_node]
    )
    
    return LaunchDescription([
        declare_environment_name,
        declare_agent_config_file,
        declare_total_timesteps,
        declare_learning_rate,
        
        simulation_launch,
        nav2_minimal_launch,
        groundtruth_amcl_node,
        rviz_node,
        gazebo_monitor_node,
        training_timer,
    ])
