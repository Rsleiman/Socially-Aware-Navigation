# Socially-Aware Navigation Workspace

A comprehensive ROS2 workspace for socially-aware robotic navigation using the Unitree Go2 robot, HuNavSim human simulation, and reinforcement learning approaches.

## System Requirements

- **OS**: Ubuntu 22.04 LTS (tested)
- **ROS2**: Humble Hawksbill
- **Python**: 3.10+
- **C++**: C++17 compiler support
- **CMake**: 3.16.3+

## Table of Contents

- [Core Dependencies](#core-dependencies)
- [System Dependencies](#system-dependencies)
- [ROS2 Dependencies](#ros2-dependencies)
- [Python Dependencies](#python-dependencies)
- [External Libraries](#external-libraries)
- [Installation Guide](#installation-guide)
- [Building the Workspace](#building-the-workspace)
- [Running simulation.launch.py](#running-simulationlaunchpy)
- [Troubleshooting](#troubleshooting)

## Core Dependencies

### System Dependencies

```bash
# Essential build tools
sudo apt update
sudo apt install -y \
    build-essential \
    cmake \
    git \
    wget \
    curl \
    python3-pip \
    python3-dev \
    python3-setuptools \
    python3-wheel \
    libeigen3-dev \
    libboost-all-dev \
    libtinyxml2-dev \
    libzmq3-dev \
    libsqlite3-dev \
    libgtest-dev \
    pkg-config

# Gazebo dependencies
sudo apt install -y gazebo \
    libgazebo-dev \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-ros2-control

# Graphics and visualization

```bash
# Install Qt5 development packages
sudo apt install -y \
    qtchooser \
    qt5-qmake \
    qtbase5-dev \
    qtbase5-dev-tools \
    libqt5widgets5
```

### ROS2 Dependencies

```bash
# Core ROS2 packages
sudo apt install -y \
    ros-humble-desktop \
    ros-humble-rclcpp \
    ros-humble-rclpy \
    ros-humble-std-msgs \
    ros-humble-geometry-msgs \
    ros-humble-sensor-msgs \
    ros-humble-nav-msgs \
    ros-humble-visualization-msgs \
    ros-humble-tf2 \
    ros-humble-tf2-ros \
    ros-humble-tf2-geometry-msgs \
    ros-humble-urdf \
    ros-humble-xacro \
    ros-humble-robot-localization \
    ros-humble-joint-state-publisher \
    ros-humble-robot-state-publisher

# ROS2 Control and Controllers
sudo apt install -y \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-controller-manager \
    ros-humble-joint-state-broadcaster \
    ros-humble-effort-controllers \
    ros-humble-position-controllers \
    ros-humble-velocity-controllers

# Navigation Stack (Nav2)
sudo apt install -y \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-nav2-msgs \
    ros-humble-nav2-common \
    ros-humble-nav2-map-server \
    ros-humble-nav2-controller \
    ros-humble-nav2-planner \
    ros-humble-nav2-behaviors \
    ros-humble-nav2-bt-navigator \
    ros-humble-nav2-waypoint-follower \
    ros-humble-nav2-lifecycle-manager \
    ros-humble-nav2-costmap-2d \
    ros-humble-nav2-behavior-tree \
    ros-humble-nav2-util

# Velodyne LiDAR support
sudo apt install -y \
    ros-humble-velodyne \
    ros-humble-velodyne-gazebo-plugins \
    ros-humble-velodyne-description

# Additional utilities
sudo apt install -y \
    ros-humble-teleop-twist-keyboard \
    ros-humble-rviz2 \
    ros-humble-rqt \
    ros-humble-tinyxml2-vendor \
    ros-humble-message-filters \
    ros-humble-lifecycle-msgs \
    ros-humble-angles \
    python3-rosdep \
    python3-colcon-common-extensions \
    python3-colcon-mixin \
```

### Python Dependencies

```bash
# Install pip packages for the workspace
pip3 install --user \
# numpy>2 creates dependency issues
    "numpy<2" \ 
    pandas \
    "matplotlib>=3.8" \
    seaborn \
    scipy \
    shapely \
    angles \
    gymnasium \
    torch \
    torchvision \
    torchaudio \
    PyYAML \
    setuptools==58.2.0
```

### External Libraries

#### 1. LightSFM (Social Force Model Library)

```bash
cd ~/
git clone https://github.com/robotics-upo/lightsfm.git
cd lightsfm
make
sudo make install
```

#### 2. BehaviorTree.CPP 4.6+ (if not using workspace version)

This is included in the workspace, but if you need to install system-wide:

```bash
# Using Conan (recommended)
pip3 install --user conan
conan profile detect --force

# Or install dependencies manually
sudo apt install -y libzmq3-dev libsqlite3-dev
```

## Installation Guide

### Step 1: Setup ROS2 Environment

```bash
# Source ROS2
source /opt/ros/humble/setup.bash

# Initialize rosdep
sudo rosdep init
rosdep update
```

### Step 2: Create and Setup Workspace

```bash
# Create workspace
mkdir -p ~/socially_aware_nav_ws/src
cd ~/socially_aware_nav_ws

# Clone this repository (if not already done)
# git clone <your-repo-url> src/

# Setup Gazebo environment
echo "source /usr/share/gazebo/setup.sh" >> ~/.bashrc
source /usr/share/gazebo/setup.sh
```

### Step 3: Install Workspace Dependencies

```bash
cd ~/socially_aware_nav_ws
rosdep install --from-paths src --ignore-src -r -y
```

### Step 4: Additional Setup for People Detection

The `people_msgs` package is required but not available in ROS2 Humble repositories:

```bash
# This is already included in the workspace under src/people/
# No additional action needed
```

## Building the Workspace

### Standard Build

```bash
cd ~/socially_aware_nav_ws
colcon build

# For faster builds (parallel compilation)
colcon build --parallel-workers $(nproc)

# For specific packages only
colcon build --packages-select <package_name>
```

### Selective Build (recommended for development)

```bash
# Build core packages first
colcon build --packages-select \
    behaviortree_cpp \
    hunav_msgs \
    people_msgs \
    lightsfm

# Build robot packages
colcon build --packages-select \
    champ_msgs \
    champ_base \
    champ_description \
    go2_description \
    go2_config

# Build simulation packages
colcon build --packages-select \
    hunav_sim \
    hunav_agent_manager \
    hunav_evaluator \
    hunav_gazebo_wrapper \
    social_nav_planner
```

### Source the Workspace

```bash
source ~/socially_aware_nav_ws/install/setup.bash

# Add to bashrc for permanent sourcing
echo "source ~/socially_aware_nav_ws/install/setup.bash" >> ~/.bashrc
```

## Running simulation.launch.py

### Prerequisites Check

Before running the simulation, ensure:

1. **Gazebo Setup**: Make sure Gazebo environment variables are set
   ```bash
   source /usr/share/gazebo/setup.sh
   ```

2. **Workspace Sourced**: 
   ```bash
   source ~/socially_aware_nav_ws/install/setup.bash
   ```

3. **No conflicting processes**: Kill any existing Gazebo processes
   ```bash
   pkill -f gazebo
   pkill -f gzserver
   pkill -f gzclient
   ```

### Basic Launch

```bash
cd ~/socially_aware_nav_ws
source /opt/ros/humble/setup.bash
source /usr/share/gazebo/setup.sh
source install/setup.bash

ros2 launch social_nav_planner simulation.launch.py
```

### Launch with Parameters

```bash
# Launch with specific environment
ros2 launch social_nav_planner simulation.launch.py \
    environment_name:=default \
    configuration_file:=agents_experimenting.yaml \
    use_rviz:=true \
    spawn_go2:=true

# Launch with custom robot position
ros2 launch social_nav_planner simulation.launch.py \
    gzpose_x:=2.0 \
    gzpose_y:=1.0 \
    gzpose_Y:=1.57

# Launch with different FOV for sensors
ros2 launch social_nav_planner simulation.launch.py \
    fov_degrees:=180.0
```

### Available Launch Arguments

- `environment_name`: Gazebo world name (default: "default")
- `configuration_file`: HuNav agents configuration (default: "agents_experimenting.yaml")
- `use_rviz`: Launch RViz visualization (default: false)
- `spawn_go2`: Spawn Go2 robot (default: true)
- `gzpose_x/y/z`: Robot initial position (default: 0.0, 0.0, 0.25)
- `gzpose_R/P/Y`: Robot initial orientation (default: 0.0, 0.0, 1.0)
- `fov_degrees`: Sensor field of view (default: 270.0)
- `use_sim_time`: Use simulation time (default: true)

## Package Structure

### Core Components

1. **BehaviorTree.CPP**: Behavior tree framework for robot decision-making
2. **HuNavSim**: Human navigation simulation with social behaviors
3. **Unitree Go2**: Quadruped robot configuration and control
4. **Social Navigation Planner**: Custom navigation with social awareness
5. **Reinforcement Learning**: A2C-based policy learning for social navigation

### Key Packages

- `behaviortree_cpp`: Core behavior tree library
- `hunav_sim`: Human navigation simulator
- `hunav_gazebo_wrapper`: Gazebo integration for HuNav
- `social_nav_planner`: Social navigation planning system
- `social_nav_rl`: Reinforcement learning implementation
- `go2_config`: Go2 robot configuration
- `champ`: Quadruped controller framework
- `people`: People detection and tracking

## Troubleshooting

### Common Issues

1. **Gazebo won't start**:
   ```bash
   # Check for conflicting processes
   pkill -f gazebo
   
   # Verify Gazebo installation
   gazebo --version
   
   # Check Gazebo environment
   echo $GAZEBO_MODEL_PATH
   ```

2. **Build errors with BehaviorTree.CPP**:
   ```bash
   # Install missing dependencies
   sudo apt install -y libzmq3-dev libsqlite3-dev libgtest-dev
   
   # Try building with verbose output
   colcon build --packages-select behaviortree_cpp --event-handlers console_direct+
   ```

3. **Python import errors**:
   ```bash
   # Reinstall Python packages
   pip3 install --user --upgrade numpy pandas matplotlib torch gymnasium shapely angles
   ```

4. **LightSFM not found**:
   ```bash
   # Reinstall LightSFM
   cd ~/lightsfm
   make clean
   make
   sudo make install
   ```

5. **ROS2 package not found**:
   ```bash
   # Update package index and install missing packages
   sudo apt update
   rosdep update
   rosdep install --from-paths src --ignore-src -r -y
   ```

### Build Issues

```bash
# Clean build if encountering issues
rm -rf build/ install/ log/
colcon build

# Build with verbose output for debugging
colcon build --event-handlers console_direct+

# Build specific package with cmake verbose
colcon build --packages-select <package_name> --cmake-args -DCMAKE_VERBOSE_MAKEFILE=ON
```

### Runtime Issues

```bash
# Check node status
ros2 node list

# Check topics
ros2 topic list
ros2 topic echo /topic_name

# Check for transform issues
ros2 run tf2_ros tf2_echo map base_link

# Monitor system resources
htop
```

## Development Notes

### Adding New Dependencies

1. **System dependencies**: Add to this README and update rosdep
2. **Python dependencies**: Add to individual package setup.py or requirements
3. **ROS2 dependencies**: Add to package.xml files

### Code Style

- C++: Follow ROS2 style guidelines
- Python: Follow PEP8 with 4-space indentation
- CMake: Use modern CMake practices

### Testing

```bash
# Run tests for specific package
colcon test --packages-select <package_name>

# Run all tests
colcon test

# View test results
colcon test-result --verbose
```

## Contributing

When contributing to this workspace:

1. Update this README if adding new dependencies
2. Test your changes with a clean build
3. Ensure all packages build without warnings
4. Update package.xml files with new dependencies
5. Follow the existing code style and structure

## License

This workspace contains packages with various licenses. Check individual package directories for specific license information.

## Support

For issues specific to:
- **HuNavSim**: [HuNavSim GitHub Issues](https://github.com/robotics-upo/hunav_sim/issues)
- **BehaviorTree.CPP**: [BehaviorTree.CPP GitHub Issues](https://github.com/BehaviorTree/BehaviorTree.CPP/issues)
- **CHAMP**: [CHAMP GitHub Issues](https://github.com/chvmp/champ/issues)
- **General workspace issues**: Create an issue in this repository
