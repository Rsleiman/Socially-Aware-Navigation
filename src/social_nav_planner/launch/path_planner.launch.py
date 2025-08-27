from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, OpaqueFunction, TimerAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessStart
import yaml

#TODO:
# - Add launch description for the go2 robot simulation with rviz
# - Change origin values to exactly match the map origin in the YAML file





# Static transform publisher Opaque Function (map -> odom) REMOVE ONCE INTEGRATED WITH SLAM LOCALISATION OR EQUIVALENT
def load_origin(context, *args, **kwargs):
    # Resolve the map_yaml LaunchConfiguration into a real string path
    yaml_file = LaunchConfiguration("map_yaml").perform(context)

    try:
        with open(yaml_file, "r") as f:
            data = yaml.safe_load(f)
        origin = data.get("origin", [0.0, 0.0, 0.0])
        origin_x, origin_y, origin_yaw = map(float, origin)
    except Exception as e:
        print(f"[WARN] Could not load origin from {yaml_file}: {e}")
        origin_x, origin_y, origin_yaw = 0.0, 0.0, 0.0

    return [
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="map_to_odom_tf",
            arguments=[
                str(origin_x), str(origin_y), "0.0",   # x, y, z
                "0.0", "0.0", str(origin_yaw),       # roll, pitch, yaw
                "map", "odom"
            ],
            output="screen"
        )
    ]


def generate_launch_description():
    # Launch configs #
    map_yaml = LaunchConfiguration("map_yaml")
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_gazebo = LaunchConfiguration("use_gazebo", default="true")

    # Include the go2 robot simulation launch file #
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("go2_config"),
                "launch",
                "gazebo.launch.py"
            ])
        ),
        launch_arguments={
            "rviz": "true",
            "use_sim_time": use_sim_time,
            # "world": ... # If we want to use a specific world file
        }.items(),
        condition=IfCondition(use_gazebo)
    )

    # Nodes #
    # Path planner node
    path_planner_node = Node(
        package="social_nav_planner",
        executable="path_planner.py",
        name="path_planner",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time
        }]
    )

    # Map server node
    map_server_node = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[{
            "yaml_filename": map_yaml,
            "use_sim_time": use_sim_time
        }]
    )
    
    # Lifecycle manager for map server
    lifecycle_manager_node = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_map",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "autostart": True,
            "node_names": ["map_server"]
        }]
    )
    
    # Return the launch description #
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            "map_yaml",
            default_value=PathJoinSubstitution([
                FindPackageShare("go2_config"),
                "maps",
                "playground.yaml"
            ]),
            description="Full path to map yaml file"
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulation (Gazebo) clock if true"
        ),
        DeclareLaunchArgument(
            "use_gazebo",
            default_value="true",
            description="Use Gazebo simulation if true"),

        sim_launch,

        path_planner_node,

        RegisterEventHandler(
            OnProcessStart(
                target_action=path_planner_node,
                on_start=[TimerAction(
                    period=1.0,
                    actions=[map_server_node]
                )]
            )
        ),

        RegisterEventHandler(
            OnProcessStart(
                target_action=map_server_node,
                on_start=[lifecycle_manager_node]
            )
        ),

        # Static transform publisher (map -> odom)
        OpaqueFunction(function=load_origin),

    ])
