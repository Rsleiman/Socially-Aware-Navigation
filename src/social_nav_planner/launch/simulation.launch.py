# Make sure to run 'source /usr/share/gazebo/setup.sh' in bash before launching this file

from os import path
from os import environ
from os import pathsep
import sys
sys.path.append('/opt/ros/humble/lib/gazebo_ros')
from gazebo_ros_paths import GazeboRosPaths # type: ignore
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (IncludeLaunchDescription, SetEnvironmentVariable, 
                            DeclareLaunchArgument, ExecuteProcess, Shutdown, 
                            RegisterEventHandler, TimerAction, LogInfo)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (PathJoinSubstitution, TextSubstitution,
                            LaunchConfiguration, PythonExpression, EnvironmentVariable, Command)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import (OnExecutionComplete, OnProcessExit,
                                OnProcessIO, OnProcessStart, OnShutdown)

def generate_launch_description():

    if 'GAZEBO_MODEL_PATH' in environ:
        paths = environ['GAZEBO_MODEL_PATH'].split(':')
        cleaned = [p for p in paths if not p.endswith('hunav_gazebo_wrapper')]
        environ['GAZEBO_MODEL_PATH'] = ':'.join(cleaned)

    # ----------------------------------------------------------
    # World generation parameters
    # ----------------------------------------------------------
    gz_obs = LaunchConfiguration('use_gazebo_obs')
    rate = LaunchConfiguration('update_rate')
    robot_name = LaunchConfiguration('robot_name')
    global_frame = LaunchConfiguration('global_frame_to_publish')
    use_navgoal = LaunchConfiguration('use_navgoal_to_start')
    navgoal_topic = LaunchConfiguration('navgoal_topic')
    ignore_models = LaunchConfiguration('ignore_models')
    navigation = LaunchConfiguration('navigation')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration("rviz")

    # ----------------------------------------------------------
    # Robot parameters
    # ----------------------------------------------------------
    namespace = LaunchConfiguration('robot_namespace')
    # scan_model = LaunchConfiguration('laser_model')
    # use_rgbd = LaunchConfiguration('rgbd_sensors')
    gz_x = LaunchConfiguration('gzpose_x')
    gz_y = LaunchConfiguration('gzpose_y')
    gz_z = LaunchConfiguration('gzpose_z')
    gz_R = LaunchConfiguration('gzpose_R')
    gz_P = LaunchConfiguration('gzpose_P')
    gz_Y = LaunchConfiguration('gzpose_Y')


    # HuNav agent configuration file
    agent_conf_file = PathJoinSubstitution([
        FindPackageShare('hunav_gazebo_wrapper'),
        'scenarios',
        LaunchConfiguration('configuration_file')
    ])

    # Read the yaml file and load the parameters
    hunav_loader_node = Node(
        package='hunav_agent_manager',
        executable='hunav_loader',
        output='screen',
        parameters=[agent_conf_file]
        #arguments=['--ros-args', '--params-file', conf_file]
    )

    # Base world file name 
    world_file = PathJoinSubstitution([
        FindPackageShare('hunav_gazebo_wrapper'),
        'worlds',
        PythonExpression(["'", LaunchConfiguration('environment_name'), ".world'"])
    ])

    # the node looks for the base_world file in the directory 'worlds'
    # of the package hunav_gazebo_plugin direclty. It then adds the agents and robots
    # to that world and saves it as 'generatedWorld.world' in the install/share/worlds directory

    # HuNav -> generate the world with the agents
    hunav_gazebo_worldgen_node = Node(
        package='hunav_gazebo_wrapper',
        executable='hunav_gazebo_world_generator',
        output='screen',
        parameters=[{'base_world': world_file},
        {'use_gazebo_obs': gz_obs},
        {'update_rate': rate},
        {'robot_name': robot_name},
        {'global_frame_to_publish': global_frame},
        {'use_navgoal_to_start': use_navgoal},
        {'navgoal_topic': navgoal_topic},
        {'ignore_models': ignore_models}]
        #arguments=['--ros-args', '--params-file', conf_file]
    )

    ordered_launch_event = RegisterEventHandler(
        OnProcessStart(
            target_action=hunav_loader_node,
            on_start=[
                LogInfo(msg='HunNavLoader started, launching HuNav_Gazebo_world_generator after 2 seconds...'),
                TimerAction(
                    period=2.0,
                    actions=[hunav_gazebo_worldgen_node],
                )
            ]
        )
    )

    # ----------------------------------------------------------
    # Launch Gazebo Classic (server and client)
    # ----------------------------------------------------------
    config_file_name = 'params.yaml' 
    pkg_dir = get_package_share_directory('hunav_gazebo_wrapper') 
    config_file = path.join(pkg_dir, 'launch', config_file_name) 
    # Alternatively:
    # path.join(
    #   launch_ros.substitutions.FindPackageShare(package="champ_gazebo").find("champ_gazebo"),
    #   "config/gazebo.yaml")
    
    world_path = PathJoinSubstitution([
        FindPackageShare('hunav_gazebo_wrapper'),
        'worlds',
        'generatedWorld.world' #TODO: Create a name based on the base world (from 'environment_name') to allow for multiple generated worlds
    ])

    gzserver_cmd = [
        'gzserver ',
         world_path, 
        _boolean_command('verbose'), '',
        # _boolean_command('pause'), '',
        '-s ', 'libgazebo_ros_init.so',
        '-s ', 'libgazebo_ros_factory.so',
        #'-s ', #'libgazebo_ros_state.so',
        '--ros-args',
        '--params-file', config_file,
    ]

    gzclient_cmd = [
        'gzclient',
        _boolean_command('verbose'), ' ',
    ]

    gzserver_process = ExecuteProcess(
        cmd=gzserver_cmd,
        output='screen',
        #additional_env=env,
        shell=True,
        on_exit=Shutdown(),
    )

    gzclient_process = ExecuteProcess(
        cmd=gzclient_cmd,
        output='screen',
        #additional_env=env,
        shell=True,
        on_exit=Shutdown(),  
    )

    

    # ----------------------------------------------------------
    # GO2 ROBOT
    # ----------------------------------------------------------
    config_pkg_share = FindPackageShare(package="go2_config").find("go2_config")
    descr_pkg_share = FindPackageShare(package="go2_description").find("go2_description")

    joints_config = path.join(config_pkg_share, "config/joints/joints.yaml")
    gait_config = path.join(config_pkg_share, "config/gait/gait.yaml")
    links_config = path.join(config_pkg_share, "config/links/links.yaml")
    default_model_path = path.join(descr_pkg_share, "xacro/robot.xacro")

    go2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            path.join(
                get_package_share_directory("champ_bringup"),
                "launch",
                "bringup.launch.py"
            )
        ),
        launch_arguments={
            "description_path": default_model_path,
            "joints_map_path": joints_config,
            "links_map_path": links_config,
            "gait_config_path": gait_config,
            "use_sim_time": use_sim_time,
            "robot_name": robot_name,
            "gazebo": "true",
            "lite": "false",
            "rviz": use_rviz,
            "joint_controller_topic": "joint_group_effort_controller/joint_trajectory",
            "hardware_connected": "false",
            "publish_foot_contacts": "false",
            "close_loop_odom": "true",
        }.items(),
        condition=IfCondition(LaunchConfiguration('spawn_go2'))
    )

    spawn_go2 = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        output="screen",
        arguments=[
            "-entity", robot_name,
            "-topic", "/robot_description",
            "-robot_namespace", namespace,
            "-x", gz_x,
            "-y", gz_y,
            "-z", gz_z,
            "-R", gz_R, "-P", gz_P,
            "-Y", gz_Y,
        ],
        condition=IfCondition(LaunchConfiguration('spawn_go2'))
    )

    # TODO: May not be needed
    # contact_sensor = Node(
    #     package="champ_gazebo",
    #     executable="contact_sensor",
    #     output="screen",
    #     parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}, links_config],
    #     condition=IfCondition(LaunchConfiguration('spawn_go2'))
    # )

    # Controllers
    load_joint_state_controller = ExecuteProcess(
        cmd=["ros2", "control", "load_controller", "--set-state", "active", "joint_states_controller"],
        output="screen",
        condition=IfCondition(LaunchConfiguration('spawn_go2'))
    )

    load_joint_trajectory_effort_controller = ExecuteProcess(
        cmd=["ros2", "control", "load_controller", "--set-state", "active", "joint_group_effort_controller"],
        output="screen",
        condition=IfCondition(LaunchConfiguration('spawn_go2'))
    )

    # TODO: Optional if you want position controller instead:
    # load_joint_trajectory_position_controller = ExecuteProcess(
    #     cmd=["ros2", "control", "load_controller", "--set-state", "active", "joint_group_position_controller"],
    #     output="screen",
    #     condition=IfCondition(LaunchConfiguration('spawn_go2'))
    # )



    gz_launch_event = RegisterEventHandler(
        OnProcessStart(
            target_action=hunav_gazebo_worldgen_node,
            on_start=[
                LogInfo(msg='GenerateWorld started, launching Gazebo after 2 seconds...'),
                TimerAction(
                    period=2.0,
                    actions=[gzserver_process, gzclient_process],
                )
            ]
        )
    )

    go2_spawn_event = RegisterEventHandler(
        OnProcessStart(
            target_action=gzserver_process,
            on_start=[
                LogInfo(msg='Gazebo server started, spawning Go2 after 2 seconds...'),
                TimerAction(
                    period=2.0,
                    actions=[
                        go2_bringup,
                        spawn_go2,
                        load_joint_state_controller,
                        load_joint_trajectory_effort_controller,
                        # load_joint_trajectory_position_controller,
                        # contact_sensor,
                    ],
                )
            ]
        )
    )

    # ------------------------------------------------------------------
    # Other HuNav nodes: behavior manager and evaluator
    # ------------------------------------------------------------------

    hunav_manager_node = Node(
        package='hunav_agent_manager',
        executable='hunav_agent_manager',
        name='hunav_agent_manager',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    metrics_file = PathJoinSubstitution([
        FindPackageShare('hunav_evaluator'),
        'config',
        LaunchConfiguration('metrics_file')
    ])

    hunav_evaluator_node = Node(
        package='hunav_evaluator',
        executable='hunav_evaluator_node',
        output='screen',
        parameters=[metrics_file]
    )

    
    # DO NOT Launch this if any robot localization is launched
    static_tf_node = Node(
        package = "tf2_ros", 
        executable = "static_transform_publisher",
        output='screen',
        arguments = ['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        condition=UnlessCondition(navigation)
    )

    
    # ----------------------------------------------------------
    # Declare the launch arguments
    # ----------------------------------------------------------
    declare_agents_conf_file = DeclareLaunchArgument(
        'configuration_file', default_value='agents_cafe.yaml',
        description='Specify agent configuration file name in the hunav_gazebo_wrapper/scenarios directory'
    )
    declare_metrics_conf_file = DeclareLaunchArgument(
        'metrics_file', default_value='metrics.yaml',
        description='Specify the name of the evaluation metrics configuration file in the hunav_evaluator/config directory'
    )
    # declare_arg_world = DeclareLaunchArgument(
    #     'base_world', default_value='no_roof_small_warehouse.world',
    #     description='Specify world file name'
    # ) #TODO: Remove?
    declare_arg_environment = DeclareLaunchArgument(
        'environment_name', default_value='cafe',
        description='Specify the name of the environment. This is used to load the Gazebo world file and map file.' #TODO: Improve description (its the stem part of the base world file)
    )
    declare_gz_obs = DeclareLaunchArgument(
        'use_gazebo_obs', default_value='True',
        description='Whether to fill the agents obstacles with closest Gazebo obstacle or not'
    )
    declare_update_rate = DeclareLaunchArgument(
        'update_rate', default_value='100.0',
        description='Update rate of the plugin'
    )
    declare_robot_name = DeclareLaunchArgument(
        'robot_name', default_value='go2',
        description='Specify the name of the robot Gazebo model'
    )
    declare_frame_to_publish = DeclareLaunchArgument(
        'global_frame_to_publish', default_value='map',
        description='Name of the global frame in which the position of the agents are provided'
    )
    declare_use_navgoal = DeclareLaunchArgument(
        'use_navgoal_to_start', default_value='False',
        description='Whether to start the agents movements when a navigation goal is received or not'
    )
    declare_navgoal_topic = DeclareLaunchArgument(
        'navgoal_topic', default_value='goal_pose',
        description='Name of the topic in which navigation goal for the robot will be published'
    )
    declare_navigation = DeclareLaunchArgument(
        'navigation', default_value='False',
        description='If launch the pmb2 navigation system'
    )
    declare_ignore_models = DeclareLaunchArgument(
        'ignore_models', default_value='aws_robomaker_warehouse_GroundB_01_001 ground_plane cafe',
        description='list of Gazebo models that the agents should ignore as obstacles as the ground_plane. Indicate the models with a blank space between them'
    )
    declare_arg_verbose = DeclareLaunchArgument(
        'verbose', default_value='true',
        description='Set "true" to increase messages written to terminal.'
    )
    # declare_arg_pause = DeclareLaunchArgument(
    #     'pause', default_value='true',
    #     description='Set "true" to launch Gazebo paused.'
    # )
    declare_arg_namespace = DeclareLaunchArgument(
        'robot_namespace', default_value='',
        description='The type of robot')
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    declare_use_rviz = DeclareLaunchArgument(
        "rviz", default_value="false", description="Launch rviz"
    )
    
    # Initial robot pose 
    declare_arg_px = DeclareLaunchArgument('gzpose_x', default_value='0.0',
            description='The robot initial position in the X axis of the world')
    declare_arg_py = DeclareLaunchArgument('gzpose_y', default_value='0.0',
            description='The robot initial position in the Y axis of the world')
    declare_arg_pz = DeclareLaunchArgument('gzpose_z', default_value='0.225',
            description='The robot initial position in the Z axis of the world')
    declare_arg_pR = DeclareLaunchArgument('gzpose_R', default_value='0.0',
            description='The robot initial roll angle in the world')
    declare_arg_pP = DeclareLaunchArgument('gzpose_P', default_value='0.0',
            description='The robot initial pitch angle in the world')
    declare_arg_pY = DeclareLaunchArgument('gzpose_Y', default_value='0.0',
            description='The robot initial yaw angle in the world')
    
    # Sensors toggles (left for completeness. TODO: Go2 stack typically ignores these here)
    # declare_arg_laser = DeclareLaunchArgument('laser_model', default_value='sick-571-gpu',
    #         description='the laser model to be used')
    # declare_arg_rgbd = DeclareLaunchArgument('rgbd_sensors', default_value='false',
    #         description='whether to use rgbd cameras or not')

    # Optional flag to spawn Go2 via go2_config
    declare_spawn_go2 = DeclareLaunchArgument(
        'spawn_go2', default_value='True',
        description='If True, include go2_config/launch/gazebo.launch.py to spawn the robot'
    )


    # ----------------------------------------------------------
    # Launch Description Assembly
    # ----------------------------------------------------------
    ld = LaunchDescription()

    # Declare the launch arguments
    ld.add_action(declare_agents_conf_file)
    ld.add_action(declare_metrics_conf_file)
    ld.add_action(declare_arg_environment)
    ld.add_action(declare_gz_obs)
    ld.add_action(declare_update_rate)
    ld.add_action(declare_robot_name)
    ld.add_action(declare_frame_to_publish)
    ld.add_action(declare_use_navgoal)
    ld.add_action(declare_navgoal_topic)
    ld.add_action(declare_navigation)
    ld.add_action(declare_ignore_models)
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_use_rviz)
    ld.add_action(declare_arg_verbose)
    # ld.add_action(declare_arg_pause) #TODO: Seems to cause robot issues
    ld.add_action(declare_arg_namespace)
    # ld.add_action(declare_arg_laser)
    # ld.add_action(declare_arg_rgbd)
    ld.add_action(declare_arg_px)
    ld.add_action(declare_arg_py)
    ld.add_action(declare_arg_pz)
    ld.add_action(declare_arg_pR)
    ld.add_action(declare_arg_pP)
    ld.add_action(declare_arg_pY)
    ld.add_action(declare_spawn_go2)

    # Generate the world with the agents
    ld.add_action(hunav_loader_node)
    ld.add_action(ordered_launch_event)

    # hunav behavior manager + evaluator
    ld.add_action(hunav_manager_node)
    ld.add_action(hunav_evaluator_node)

    # launch Gazebo after worldGenerator 
    ld.add_action(gz_launch_event)

    # spawn Go2 after Gazebo
    ld.add_action(go2_spawn_event)

    # Static TF if no nav stack
    ld.add_action(static_tf_node)
    return ld

# Add boolean commands if true
def _boolean_command(arg):
    cmd = ['"--', arg, '" if "true" == "', LaunchConfiguration(arg), '" else ""']
    py_cmd = PythonExpression(cmd)
    return py_cmd
    # py_cmd = PythonExpression(cmd)  # Commented out duplicate line
    # return py_cmd  # Commented out duplicate return