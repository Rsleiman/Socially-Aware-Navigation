# Socially-Aware Navigation

A ROS 2 framework for training and evaluating socially-aware navigation policies for the Unitree Go2 quadruped robot using reinforcement learning in simulated human environments.

## Overview

This project implements socially-aware navigation using Deep Reinforcement Learning (A2C) in Gazebo simulation with realistic human agent interactions powered by the Social Force Model. The system integrates the Unitree Go2 robot with the HuNavSim framework and provides both holonomic and non-holonomic control modes.

### Key Features

- **Deep RL Training**: A2C (Advantage Actor-Critic) policy training for socially-aware navigation
- **Realistic Human Simulation**: Integration with HuNavSim for realistic pedestrian behaviors using Social Force Models
- **Dual Action Modes**: Support for both holonomic and non-holonomic robot control
- **Social Metrics**: Comprehensive evaluation using the HuNav Evaluator for social navigation metrics
- **Nav2 Integration**: Custom ground-truth localization with ROS 2 Nav2 stack
- **Simulation Environment**: Gazebo-based simulation with configurable human groups and scenarios

## Architecture

### Custom Packages

This repository contains two main custom packages that you created:

#### 1. `social_nav_planner`
Navigation stack integration and sensor processing for the Go2 robot.

**Key Components:**
- Ground truth AMCL node (`groundtruth_amcl.py`) - bypasses traditional localization with perfect positioning
- People decoder node - converts human agents to laser-like observations
- Obstacle decoder node - processes laser scan data for RL policy
- Goal decoder node - converts goal poses to relative features
- Object decoder fuser - combines all sensor data with occlusion handling
- Command velocity smoother - smooths RL actions for robot control
- Metrics recorder - interfaces with HuNav Evaluator for social metrics

**Launch Files:**
- `simulation.launch.py` - Complete simulation environment with Gazebo + HuNav + Go2
- `nav2_simulation.launch.py` - Full navigation stack with Nav2 integration
- `nav2_no_amcl.launch.py` - Nav2 components without AMCL (using ground truth localization)

#### 2. `social_nav_rl`
Reinforcement learning training, evaluation, and policy deployment.

**Key Components:**
- `env_social_nav.py` - Gymnasium-compatible RL environment for social navigation
- `policy_network.py` - A2C agent implementation with actor-critic networks
- `train_a2c.py` - Complete A2C training pipeline
- `social_reward_calculator.py` - Multi-component reward function:
  - Obstacle collision avoidance (r_ro)
  - Human distance maintenance (r_rh)
  - Social group interaction avoidance (r_rs)
  - Goal progress incentive (r_rg)
  - Robot orientation stability (r_rf)
- `training_monitor.py` - Real-time training visualization and logging
- `gazebo_monitor.py` - Automatic Gazebo unpause for stable training
- `evaluation_manager_node.py` - Orchestrates policy evaluation across scenarios
- `policy_evaluation_node.py` - Deploys trained policies for evaluation
- `results_analyzer_node.py` - Analyzes and visualizes evaluation results

**Launch Files:**
- `train_a2c.launch.py` - Complete A2C training pipeline
- `evaluate_with_metrics.launch.py` - Policy evaluation with social metrics
- `nav2_minimal_for_training.launch.py` - Minimal Nav2 components for training

### Imported Dependencies

The following packages are imported dependencies from external sources:

- **BehaviorTree.CPP** & **BehaviorTree.ROS2** - Behavior tree framework
- **hunav_gazebo_wrapper** - Gazebo world and scenario management
- **hunav_sim** - Human navigation simulation with Social Force Models
- **lightsfm** - Lightweight Social Force Model implementation
- **people** - ROS people message definitions
- **unitree-go2-ros2** - Unitree Go2 robot description and control

## Installation

### Prerequisites

- **ROS 2 Humble** (recommended)
- **Gazebo Classic** (11.x)
- **Python 3.8+**
- **PyTorch** (for RL training)
- **Gymnasium** (OpenAI Gym successor)

### Dependencies

```bash
# Install ROS 2 dependencies
sudo apt update
sudo apt install ros-humble-nav2-bringup \
                 ros-humble-gazebo-ros-pkgs \
                 ros-humble-people-msgs

# Install Python dependencies
pip install torch gymnasium numpy matplotlib pandas seaborn shapely
```

### Build Instructions

```bash
# Create workspace
mkdir -p ~/go2_social_nav_ws/src
cd ~/go2_social_nav_ws/src

# Clone this repository
git clone https://github.com/Rsleiman/Socially-Aware-Navigation.git .

# Install ROS dependencies
cd ~/go2_social_nav_ws
rosdep install --from-paths src --ignore-src -r -y

# Build workspace
source /opt/ros/humble/setup.bash
source /usr/share/gazebo/setup.sh
colcon build --symlink-install

# Source workspace
source install/setup.bash
```

## Usage

### Training an A2C Policy

Launch the complete training pipeline:

```bash
cd ~/go2_social_nav_ws
source install/setup.bash

ros2 launch social_nav_rl train_a2c.launch.py \
    environment_name:=cafe \
    configuration_file:=agents_experimenting.yaml \
    total_timesteps:=1000000 \
    learning_rate:=3e-4 \
    action_mode:=holonomic
```

**Training Arguments:**
- `environment_name`: Gazebo world to use (default: `cafe`)
- `configuration_file`: HuNav agent configuration YAML
- `total_timesteps`: Total training steps (default: 1,000,000)
- `learning_rate`: Learning rate for A2C (default: 3e-4)
- `action_mode`: `holonomic` or `nonholonomic` (default: `holonomic`)

Training outputs:
- Models saved to: `install/social_nav_rl/share/social_nav_rl/models/`
- Logs saved to: `install/social_nav_rl/share/social_nav_rl/logs/`
- Plots saved to: `install/social_nav_rl/share/social_nav_rl/plots/`

### Evaluating a Trained Policy

Evaluate a trained model with social navigation metrics:

```bash
ros2 launch social_nav_rl evaluate_with_metrics.launch.py \
    model_path:=latest \
    experiment_tag:=my_evaluation \
    num_episodes:=10 \
    action_mode:=holonomic
```

**Evaluation Arguments:**
- `model_path`: Path to `.pt` model file or `latest` for most recent
- `experiment_tag`: Tag for identifying this evaluation run
- `num_episodes`: Number of episodes to evaluate (default: 10)
- `action_mode`: Must match the trained model's action mode

Results saved to: `install/social_nav_rl/share/social_nav_rl/results/`

### Running Simulation Only

Launch the Gazebo simulation environment with HuNav agents:

```bash
ros2 launch social_nav_planner simulation.launch.py \
    environment_name:=cafe \
    configuration_file:=agents_experimenting.yaml \
    use_rviz:=true
```

### Running with Nav2 Stack

Launch the complete navigation system with Nav2:

```bash
ros2 launch social_nav_planner nav2_simulation.launch.py \
    environment_name:=cafe \
    configuration_file:=agents_experimenting.yaml
```

Then send navigation goals via RViz2 using the "2D Goal Pose" tool or publish to `/goal_pose` topic.

## Observation Space

The RL policy receives a 726-dimensional observation vector:

- **Fused Object Array** (720 dims): 240 rays × 3 types
  - Agent distances (0-10m, normalized)
  - Obstacle distances (0-10m, normalized)
  - Group IDs (categorical)
- **Goal Features** (4 dims): relative_x, relative_y, distance, heading
- **Robot Position** (2 dims): global x, y (normalized)

## Action Space

### Holonomic Mode (3D)
- `linear_x`: [-0.5, 0.8] m/s
- `linear_y`: [-0.6, 0.6] m/s
- `angular_z`: [-0.8, 0.8] rad/s

### Non-holonomic Mode (2D)
- `linear_x`: [-0.5, 0.8] m/s
- `angular_z`: [-0.8, 0.8] rad/s

## Reward Function

Multi-component reward system balancing goal progress and social constraints:

1. **Goal Progress** (r_rg): Positive reward for moving toward goal
2. **Human Distance** (r_rh): Penalty for violating personal space (<1.2m)
3. **Obstacle Proximity** (r_ro): Penalty for proximity to obstacles
4. **Social Group Avoidance** (r_rs): Penalty for entering group core areas
5. **Orientation Stability** (r_rf): Penalty for extreme robot orientations

**Terminal Conditions:**
- ✅ Goal reached (distance < 0.5m): +1000 reward
- ❌ Robot-human collision: -200 reward
- ❌ Obstacle collision: -100 reward
- ❌ Robot flipped: -200 reward
- ⏱️ Max episode steps: 1000 steps

## Project Structure

```
Socially-Aware-Navigation/
├── src/
│   ├── social_nav_planner/          # Custom: Navigation & sensor processing
│   │   ├── scripts/                 # Python nodes
│   │   ├── launch/                  # Launch files
│   │   ├── config/                  # Navigation parameters
│   │   └── README.md
│   │
│   ├── social_nav_rl/               # Custom: RL training & evaluation
│   │   ├── scripts/                 # RL environment, training, evaluation
│   │   ├── launch/                  # Training/evaluation launch files
│   │   └── config/                  # RL & metrics configuration
│   │
│   ├── hunav_sim/                   # Dependency: Human navigation simulation
│   ├── hunav_gazebo_wrapper/        # Dependency: Gazebo integration
│   ├── unitree-go2-ros2/            # Dependency: Go2 robot description
│   ├── lightsfm/                    # Dependency: Social Force Model
│   ├── people/                      # Dependency: People messages
│   ├── BehaviorTree.CPP/            # Dependency: Behavior trees
│   └── BehaviorTree.ROS2/           # Dependency: BT ROS2 bindings
│
└── resources/                       # Additional resources
```

## Debugging

### Check Node Status
```bash
# List active nodes
ros2 node list

# Check transforms
ros2 run tf2_ros tf2_echo map base_link

# Monitor topics
ros2 topic echo /social_observation/fused_object_array
ros2 topic echo /cmd_vel_smoothed
```

### Common Issues

**Gazebo freezes during training:**
- The `gazebo_monitor` node automatically detects and unpauses Gazebo

**No laser scan data:**
- Verify laser sensor is publishing: `ros2 topic echo /front_laser_scan`
- Check sensor parameters in Go2 URDF

**Nav2 errors:**
- Ensure ground truth AMCL is running: `ros2 node list | grep amcl`
- Check map server: `ros2 service call /map_server/get_state lifecycle_msgs/srv/GetState`

## Configuration

### Agent Scenarios
Edit human agent configurations in `src/hunav_gazebo_wrapper/scenarios/*.yaml`

### Navigation Parameters
Tune Nav2 parameters in `src/social_nav_planner/config/navigation_groundtruth.yaml`

### RL Hyperparameters
Modify training configuration in `src/social_nav_rl/scripts/train_a2c.py`

## Evaluation Metrics

The HuNav Evaluator computes comprehensive social navigation metrics:

- **Task Success**: Goal completion rate, time to goal
- **Safety**: Collision rates (robot-on-person, person-on-robot)
- **Social Compliance**: 
  - Average/minimum distance to people
  - Personal space intrusions
  - Intimate space intrusions
- **Path Quality**: Path length, smoothness

## Future Work

See `src/social_nav_planner/notes.md` for TODO items and development roadmap.

## Citation

If you use this work in your research, please cite:

```bibtex
@software{sleiman2026socially,
  author = {Sleiman, Ralph},
  title = {Socially-Aware Navigation for Quadruped Robots using Deep Reinforcement Learning},
  year = {2026},
  url = {https://github.com/Rsleiman/Socially-Aware-Navigation}
}
```

## License

[Add your license information here]

## Acknowledgments

This project builds upon:
- [HuNavSim](https://github.com/robotics-upo/hunav_sim) - Human navigation simulation
- [Unitree Go2 ROS2](https://github.com/abizovnuralem/go2_ros2_sdk) - Go2 robot interface
- [Nav2](https://navigation.ros.org/) - ROS 2 navigation framework
- [BehaviorTree.CPP](https://www.behaviortree.dev/) - Behavior tree library

## Contact

**Author**: Ralph Sleiman  
**Email**: ralphsleiman16@gmail.com  
**GitHub**: [@Rsleiman](https://github.com/Rsleiman)

---

For more details on specific components, see the package-level READMEs in `src/social_nav_planner/` and `src/social_nav_rl/`.
