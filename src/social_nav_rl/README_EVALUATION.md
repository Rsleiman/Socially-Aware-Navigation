# Social Navigation Evaluation System

This system provides automated evaluation of trained social navigation policies using the HuNav Evaluator framework.

## Overview

The evaluation pipeline consists of:
1. **Evaluation Manager**: Orchestrates episode lifecycle and metric recording
2. **Policy Evaluation**: Loads trained models and runs deterministic evaluation
3. **Results Analyzer**: Processes metrics into summary reports and plots
4. **HuNav Evaluator**: Records detailed social navigation metrics

## Quick Start

### 1. Test the System
```bash
cd /path/to/go2_social_nav_ws
source install/setup.bash
python3 install/social_nav_rl/lib/social_nav_rl/eval_cli.py test
```

### 2. List Available Models
```bash
python3 install/social_nav_rl/lib/social_nav_rl/eval_cli.py list-models
```

### 3. Run Evaluation
```bash
# Use latest model with default settings
python3 install/social_nav_rl/lib/social_nav_rl/eval_cli.py launch

# Custom evaluation with specific environment and agent config
python3 install/social_nav_rl/lib/social_nav_rl/eval_cli.py launch \
    --model-path /path/to/model.pt \
    --experiment-tag cafe_evaluation \
    --num-episodes 20 \
    --action-mode holonomic \
    environment_name:=cafe \
    agent_configuration_file:=agents_cafe.yaml
```

### 4. Launch Directly with ROS2
```bash
ros2 launch social_nav_rl evaluate_with_metrics.launch.py \
    model_path:=latest \
    experiment_tag:=eval_test \
    num_episodes:=10 \
    environment_name:=default \
    agent_configuration_file:=agents_experimenting.yaml
```

## Configuration

### Metrics Configuration
Edit `config/eval_metrics.yaml` to select which metrics to compute:

**Enabled metrics for conversational group navigation:**
- Task success: `completed`, `time_to_reach_goal`, `path_length`
- Safety: `robot_on_person_collision`, `person_on_robot_collision` 
- Social comfort: `intimate/personal/social_space_intrusions`
- **Group interactions**: `group_intimate/personal/social_space_intrusions`
- Motion quality: `cumulative_heading_changes`, `avg_robot_linear_speed`

### Scenario Configuration
Edit `config/default_scenarios.yaml` to define evaluation scenarios:
- Robot spawn positions and orientations  
- Goal locations per scenario
- Environment names (matching `.world` files in `hunav_gazebo_wrapper/worlds/`)
- Agent configurations (matching `.yaml` files in `hunav_gazebo_wrapper/scenarios/`)

**Available environments:**
- `default` - Open space environment
- `cafe` - Cafe environment with realistic social groups
- `warehouse` - Warehouse environment with obstacles
- `house` - House environment
- `train` - Training environment
- `playground` - Playground environment

**Available agent configurations:**
- `agents_experimenting.yaml` - Small group for basic experiments
- `train_env_1.yaml` - Training environment 1 with multiple groups
- `train_env_2.yaml` - Training environment 2 with different configurations  
- `agents_cafe.yaml` - Cafe-specific agent setup
- `agents_warehouse.yaml` - Warehouse-specific agent setup
- `agents_house.yaml` - House-specific agent setup

## Architecture

### Components

1. **evaluation_manager_node.py**
   - Loads scenario configurations
   - Manages episode sequencing
   - Calls HuNav evaluator services (`hunav_start_recording`/`hunav_stop_recording`)

2. **policy_evaluation_node.py** 
   - Loads trained A2C models (latest or specified path)
   - Runs deterministic policy evaluation
   - Publishes cmd_vel commands and episode status

3. **results_analyzer_node.py**
   - Processes HuNav evaluator CSV outputs
   - Generates summary reports and plots
   - Saves results to `share/social_nav_rl/results/`

### Data Flow

```
Evaluation Manager → Policy Evaluation → Environment
       ↓                    ↓               ↓
   HuNav Start         cmd_vel           Observations
       ↓                    ↓               ↓
Episode Termination ←  Episode Status ←  Termination Flags
       ↓
   HuNav Stop → Metrics CSV → Results Analyzer → Reports/Plots
```

## Metrics Output

Results are saved in HuNav evaluator's results directory:
- `social_nav_eval_metrics_[experiment_tag].csv`: Final episode metrics
- `social_nav_eval_metrics_steps_[experiment_tag]_[run_id].csv`: Per-timestep data

Key metrics for social navigation:
- **Success rate**: Percentage of episodes reaching goal
- **Collision rate**: Human and obstacle collisions per episode  
- **Space intrusions**: Time spent violating proxemic spaces (individual + groups)
- **Motion smoothness**: Heading changes and velocity statistics
- **Task efficiency**: Path length and time to goal

## Troubleshooting

### Common Issues

1. **No models found**: Train a model first with `train_a2c.py`
2. **Service timeout**: Ensure `hunav_evaluator` node is running
3. **Missing topics**: Check that agent manager is publishing `/human_states` and `/robot_states`
4. **Import errors**: Run `colcon build` after making changes

### Dependencies
- hunav_evaluator and hunav_msgs packages
- PyYAML for configuration parsing
- Matplotlib/Seaborn for plotting (results_analyzer only)

## Next Steps

1. **Add more scenarios**: Edit `default_scenarios.yaml` with your test cases
2. **Custom metrics**: Extend HuNav evaluator with domain-specific measurements
3. **Batch evaluation**: Run multiple models/configurations in sequence
4. **Statistical analysis**: Compare different training runs and hyperparameters