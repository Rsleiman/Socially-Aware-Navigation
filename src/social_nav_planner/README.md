# Social Navigation Planner

This package provides a social navigation system for the Go2 robot with ground truth localization, integrated with Hunavsim.

## Summary
The system integrates:
- Gazebo simulation with HuNav human agents
- Go2 quadruped robot
- Nav2 navigation stack with ground truth localization
- Custom path planner capability (to be implemented)
- Hunav Evaluator to measure navigation wrt. social metrics

## Components

### 1. Groundtruth AMCL Node (`groundtruth_amcl.py`)
- Subscribes to `/odom/ground_truth` from Gazebo
- Publishes pose on `/amcl_pose` for Nav2
- Broadcasts `map->base_link` transform
- Bypasses Nav2 Stack's requirement for localisation.

### 2. Custom Navigation Configuration (`navigation_groundtruth.yaml`)
- Nav2 parameters optimized for ground truth localization
- AMCL sections commented out
- Costmap configurations for laser data 

### 3. Launch Files
- `simulation.launch.py`: Original simulation with Gazebo + HuNav + Go2 (No Nav stack)
- `nav2_no_amcl.launch.py`: Nav2 stack bringup without AMCL (replaced by our GT AMCL node)
- `nav2_simulation.launch.py`: Complete system with Nav Stack

## Usage

### 1. Launch the complete system:
```bash
cd /path/to/go2_social_nav_ws
source /opt/ros/humble/setup.bash
source /usr/share/gazebo/setup.sh # To load gazebo default models and env variables
source install/setup.bash
ros2 launch social_nav_planner nav2_simulation.launch.py # TODO: Params (path planner, config file, etc.)
```

### 2. Available launch arguments:
- `use_rviz`: Launch RViz
- `map`: Map file to use
- `environment_name`: Gazebo world
- `configuration_file`: HuNav agent configuration yaml
- ...

If Missing dependencies: Run `rosdep install --from-paths . -r -y`

### Useful debug comnands:
```bash
# node status
ros2 node list | grep -E "(amcl|nav2|controller)"
# check transforms
ros2 run tf2_ros tf2_echo map base_link
# Check nav status  
ros2 service call /bt_navigator/get_state lifecycle_msgs/srv/GetState
# look at costmaps
ros2 run nav2_costmap_2d nav2_costmap_2d_markers voxel_grid:=/local_costmap/voxel_grid
```

## TODO: 
1. Add laserscan data and costmap updating
2. Implement custom path planner plugins
3. Tune costmap params