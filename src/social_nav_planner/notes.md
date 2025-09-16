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
7. Implement LIDAR (not needed for localisation due to GT amcl node but still needed for obstacle detection/costmap updating)
8. Create a node/py file that calls the metric recording starter and stopper so that we can add custom parameters into the starter.
9. Create wrapper/interface for plugging in a certain path planning algorithm into Nav2, and activate it once a goal is sent (Also, determine how we will send a goal. i.e, through a topic, action (like Nav2's NavigateToPose) or something else?)
10. Create evaluation scenarios (Based on what? A dataset? Arbitrary/custom? Generator node? (creates random group centre points, then random number of agents placed around centre and conforming to Hall's personal spaces, and all facing centre point)
11. Implement path planners
12. Once movements and interactions are confirmed visually via gazebo, find a way to remove visualisation of simulation without affecting functoinality for faster training/scripting runs etc.

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