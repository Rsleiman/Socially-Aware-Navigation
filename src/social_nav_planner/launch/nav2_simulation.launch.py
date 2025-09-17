#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import (IncludeLaunchDescription, DeclareLaunchArgument, SetEnvironmentVariable,
                           TimerAction, RegisterEventHandler, LogInfo, OpaqueFunction, ExecuteProcess)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart


def generate_launch_description():
    
    # Launch arguments
    rviz = LaunchConfiguration("rviz") # Must be different name than use_rviz used for simulation ( I think)
    use_composition = LaunchConfiguration("use_composition")
    autostart = LaunchConfiguration("autostart")
    params_file = LaunchConfiguration("params_file")
    environment_name = LaunchConfiguration("environment_name")
    agent_config_file = LaunchConfiguration("configuration_file")

    map_yaml_file = PathJoinSubstitution([
            FindPackageShare("hunav_gazebo_wrapper"),
            "maps",
            PythonExpression(["'", environment_name, ".yaml'"])
        ])
    
    # Default parameters
    declare_rviz = DeclareLaunchArgument(
        "rviz", default_value="true",
        description="Whether to start RVIZ")
    
    declare_use_composition = DeclareLaunchArgument(
        "use_composition", default_value="False", 
        description="Whether to use composed bringup")
        
    declare_autostart = DeclareLaunchArgument(
        "autostart", default_value="true",
        description="Automatically startup the nav2 stack")
        
    declare_params_file = DeclareLaunchArgument(
        "params_file",
        default_value=PathJoinSubstitution([
            FindPackageShare("social_nav_planner"),
            "config",
            "navigation_groundtruth.yaml"
        ]),
        description="Full path to the ROS2 parameters file to use for all launched nodes")

    declare_environment_name = DeclareLaunchArgument(
        "environment_name", default_value="cafe", # TODO: default does not have a yaml or pgm yet. Will lead to map_server errors.
        description="Name of the environment to load")
    
    declare_agent_config_file = DeclareLaunchArgument(
        "configuration_file", default_value="agents_experimenting.yaml",
        description="Specify agent configuration file name in the hunav_gazebo_wrapper/scenarios directory"
        )

    # ----------------------------------------------------------
    # Nodes and launch files
    # ----------------------------------------------------------

    # Sim launch
    simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("social_nav_planner"),
                "launch",
                "simulation.launch.py"
            ])
        ]),
        launch_arguments={
            "use_rviz": "false",  # Launch our own RViz with Nav2 config
            "navigation": "true",  # Disable static transform publisher
            "use_sim_time": "true",
            "environment_name": environment_name,
            "configuration_file": agent_config_file,
        }.items()
    )

    # Nav2 bringup (custom launch without AMCL)
    nav2_bringup_no_amcl_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("social_nav_planner"),
                "launch",
                "nav2_no_amcl.launch.py"
            ])
        ]),
        launch_arguments={
            "map": map_yaml_file,
            "use_sim_time": "true",
            "params_file": params_file,
            "autostart": autostart,
            "use_lifecycle_mgr": "true",
        }.items()
    )

    # Ground truth AMCL node (replaces real AMCL)
    groundtruth_amcl_node = Node(
        package="social_nav_planner",
        executable="groundtruth_amcl.py",
        name="groundtruth_amcl",
        parameters=[params_file],
        output="screen",
    )

    nav2_bringup = TimerAction(
        period=10.0,  # TODO: Tune delay
        actions=[nav2_bringup_no_amcl_launch]
    )

    groundtruth_amcl = TimerAction(
        period=10.0,  # TODO: Tune delay
        actions=[groundtruth_amcl_node]
    )

    # RViz2 node with Nav2 config
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
        output="screen",
        condition=IfCondition(rviz)
    )

    # rviz_log = LogInfo(msg="Attempting to launch RViz2 node...")

    # def print_rviz_config(context):
        # print("RViz config file:", rviz_config_file.perform(context))
        # return []

    rviz_event = TimerAction(
        period=5.0, #TODO: Does not work if wait is too long
        actions=[
            # rviz_log,
            # OpaqueFunction(function=print_rviz_config),
            rviz_node
        ]
    )

    # Clear costmaps after Nav2 
    clear_costmaps = RegisterEventHandler(
        OnProcessStart(
            target_action=groundtruth_amcl_node,
            on_start=[
                TimerAction(
                    period=10.0,  # Wait 10 seconds after Nav2 starts
                    actions=[
                        LogInfo(msg="Clearing costmaps..."),
                        ExecuteProcess(
                            cmd=[
                                'ros2', 'service', 'call',
                                '/global_costmap/clear_entirely_global_costmap',
                                'nav2_msgs/srv/ClearEntireCostmap',
                                '{}'
                            ],
                            output='screen'
                        ),
                        ExecuteProcess(
                            cmd=[
                                'ros2', 'service', 'call',
                                '/local_costmap/clear_entirely_local_costmap',
                                'nav2_msgs/srv/ClearEntireCostmap',
                                '{}'
                            ],
                            output='screen'
                        )
                    ]
                )
            ]
        )
    )
    # ----------------------------------------------------------
    # Launch Description
    # ----------------------------------------------------------
    return LaunchDescription([
        declare_rviz,
        declare_use_composition,
        declare_autostart,
        declare_params_file,
        declare_environment_name,
        declare_agent_config_file,

        simulation_launch,
        nav2_bringup,
        groundtruth_amcl,
        rviz_event,
        clear_costmaps,
    ])
