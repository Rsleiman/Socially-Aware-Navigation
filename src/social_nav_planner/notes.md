### TODOs for later
1. ~~Find way to increase speed of robot~~
2. ~~Find a way to accept agent yamls with no goals~~
3. Look for threshold value that stops humans reacting to forces.
4. ~~Experiment with social forces configurations in yaml such that:~~
- ~~Humans dont react weirdly to each other~~
- ~~Humans stay still in static groups~~
Perhaps setting group's social forces to 0 while walking agents have positive social forces will be ideal.
5. ~~Investigate group_id parameter to see whether agents with same group id can ignore each other's forces:~~
6. ~~Get Nav2 Stack working~~
7. ~~Implement LIDAR (not needed for localisation due to GT amcl node but still needed for obstacle detection/costmap updating)~~
8. Create a node/py file that calls the metric recording starter and stopper so that we can add custom parameters into the starter.
9. Create wrapper/interface for plugging in a certain path planning algorithm into Nav2, and activate it once a goal is sent (Also, determine how we will send a goal. i.e, through a topic, action (like Nav2's NavigateToPose) or something else?)
10. Create evaluation scenarios (Based on what? A dataset? Arbitrary/custom? Generator node? (creates random group centre points, then random number of agents placed around centre and conforming to Hall's personal spaces, and all facing centre point)
11. Implement path planners
12. Once movements and interactions are confirmed visually via gazebo, find a way to remove visualisation of simulation without affecting functoinality for faster training/scripting runs etc.
13. Plan steps to automate and script the RL process (For every decision taken, you must justify, with evidence, why you are taking it in the dissertation vs the alternative. Eg. Does SANG do this? Is it simpler to implement? Is it more realistic of real world scenarios? etc.):
- CREATING ENVIRONMENTS
  - Create a node that generates random agent data (random number of groups, random group sizes, random group positions)
    - Every agent should be in a group
    - This data should satisfy certain constraints such as:
      - Group locations created within a certain boundary in a map
      - No overlapping agents groups
      - Agents not too close to walls/obstacles
      - Agents facing towards group centre # Not too important if we are not simulating walking agents
      - Abides to Hall's personal space rules
      - etc.
  - Create a node/script that creates an agent configuration yaml file from the above data
    - Ensure certain parameters are set correctly (cyclic_goals, goals = spawn point, etc) so that agents remain static. See agents_experiments.yaml for reference.
- ROBOT SPAWNING AND GOAL SETTING
  - Decide how the robot spawn position and goal position should be constrained:
    - Random positions within a certain boundary in the map?
    - Random spawn below group area boundary and random goal above group area boundary?
    - Random spawn outside group area boundary and random goal inside group area boundary (but not within a specific group's personal space)?
    - Fixed spawn and goal positions?
    - etc.
  - Depending on how we decide to choose spawn and goal positions, we may be able to iterate through a list of pre-defined positions instead of constantly resetting simulations (would just need to adapt when we start and stop metric recording and RL training) 
    - OR once a goal is reached (or timeout), we can teleport to a new random spawn and goal position without resetting the simulation
  - Create a node that sends random goals to the robot within the decided constraints.
  - Decide what consistutes as a goal being reached.
    - Do we care about orientation? LEANING TOWARDS NO
    - Do we want to include a tolerance radius around the goal point? LEANING TOWARDS YES
- LEARNING AND ROBOT CONTROL
  - Look into SANG paper and analyse the network architecture.
  - Decide on the action space
    - Discrete or continuous?
    - Holonomic or non-holonomic?
  - Create a node that implements the RL algorithm and interfaces with the robot control node to send goals and receive state information (robot position, input tensor)
  - Decide on the reward structure. Which hunav metrics do we want to include in the reward calculation and how do we weight them? Investigate the SANG paper's reward structure.
  - Create a node that calculates the reward from the hunav metrics and sends it to the learning node.
  - Create logic that teaches the network based on the reward received.
  - Create a node that saves the trained model weights at regular intervals or when a certain performance threshold is reached.
- SCRIPTING FOR TRAINING RUNS
  - Decide a number of runs for each configuration
  - Decide whether we want to save the generated data (robot spawn/goal positions, agent configuration data) for each run into a log file and script it from there. Essentially we are asking if we want to be able to exactly replicate a run for different path planners or different network architectures. I THINK YES
    - If so, do we create a script that processes each run in the log and executes ros2 launch commands with the relevant parameters?
    - If not, do we create a script that generates random data from the above nodes and executes ros2 launch commands with the relevant parameters, and then just execute this script multiple times?
  - Decide how to save trained model weights for each implementation.
- TESTING AND EVALUATION
  - Create a node/script that tests the trained model weights in a set of pre-defined scenarios (similar to training environment creation but with fixed agent configurations and robot spawn/goal positions)
  - Create a node/script that records the hunav metrics for each test run and saves them to a csv or similar file for easy comparison between different implementations.
  - Create visualisations of the results (graphs, charts, etc) to compare performance between different implementations.

14. ~~Decide on FOV and angular resolution of the goal decoder laser scan. (Currently 240 rays over 180 degrees, i.e, 0.75 degrees per ray).~~
  ~~- A narrower FOV means less chance of goals being detected, i.e. more -1 readings, potentially making it harder for the RL algorithm to learn.~~
  ~~- Maybe test an FOV of 270 degrees (i.e, 3/4 of a full circle), then experiment with num rays (240, 360, 480)~~
16. Move output and log files to a dedicated results folder outside of install dir?

### Notes
1. Must run 'source /usr/share/gazebo/setup.sh' before running launch files
2. z position field in people_msgs/msg/People refers to human's heading (-pi to pi), z velocity field is rate of change of heading.
3. If yaml files contain a ros__parameters field, everything in that field can be passed as parameters into a ros node and be directly callable from that ros node (see hunav_loader node in simulation.launch.py). yaml format:
```
<node name>:
    ros__parameters:
        param1: value
        param2:
            subparam: value
            subparam: value
...
```
4. hunav_rviz_panel doesn't change the map parameter when saving an agents_\<map\>.yaml configuration. It stays as 'cafe'. # This doesn't seem to cause any errors. But to be safe make sure that the map parameter matches the map you want to load.
5. hunav_rviz_panel does not seem to save the orientation of the agent from the hunavgoal topic into the yaml. TODO?
6. Only way currently to keep agents static is to set one goal = to spawn point and set cyclic_goals to true
7. With the above configuration (note 6), agents are stuck at the goal coord and cannot react to other social forces. TODO: Maybe just ignore reaction requirement and treat agents as completely static?
8. ~~The "group_*_intrusions" metrics in metrics.yaml leads to errors in HunavEvaluatorNode:~~ FIXED
9. Frame tree: map -> go2 is published by HunavSim, map -> odom -> base_footprint is published by nav2. We do not need to worry about connecting the go2 and odom/base_footprint trees because both are based off of Gazebo's ground truth.
10. In current iteration, odom frame orientation is fixed to map orientation. Only position is transformed from map -> odom. This is because the odom -> footprint tf handled by champ's ekf includes the robots rotation wrt to the map. Including rotation in map -> odom frame for nav2 stack to work would result in double counting the robot's rotation from map -> .. -> footprint. Work-around currently is to not tf the orientation from map -> odom. TODO: Once Nav2 stack is fully working (local/global costmap updating), check if this causes issues. Hopefully nav2 doesnt think the robot is always pointing same direction as map.
11. To add the custom social navigation planner:

- Create plugin that inherits from nav2_core::GlobalPlanner
- Update navigation_groundtruth.yaml to use the plugin:
   ```yaml
   planner_server:
     ros__parameters:
       planner_plugins: ["SocialPlanner"]
       SocialPlanner:
         plugin: "my_package::MySocialPlanner"
         # custom parameters
   ```
- Build and test
12. costmap layer explanations:
- StaticLayer:  Loads an occupancy grid from a map yaml (and pgm) file. Marked cells (black pixels) in the map become obstacles. Represents fixed obstacles and walls in the environment. __only in global costmap__
- ObstacleLayer: Takes a sensor topic and marks/clears cells in this costmap dynamically. __both costmaps__
- VoxelLayer: A voxel is the 3d equivalent of a pixel. This layer stores obstacles as 3d objects via depth sensors/3D Lidar. Then its projected onto a 2D costmap. __remove__
- InflationLayer: Takes all marked cells (i.e locations where objects are detected) and "inflates" them outward to create a cost gradient. Allows safer navigating (DWB and TEB use it) and makes navigation more robust to incorrect sensor data. __both costmaps__
13. For our people_msgs decoder, the bins 0 -> 240 go from right to the left of the robot (anti-clockwise).