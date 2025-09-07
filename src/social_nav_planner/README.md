### TODOs for later
1. ~~Find way to increase speed of robot~~
2. ~~Find a way to accept agent yamls with no goals~~
3. Look for threshold value that stops humans reacting to forces.
4. ~~Experiment with social forces configurations in yaml such that:~~
- ~~Humans dont react weirdly to each other~~
- ~~Humans stay still in static groups~~
Perhaps setting group's social forces to 0 while walking agents have positive social forces will be ideal.
5. ~~Investigate group_id parameter to see whether agents with same group id can ignore each other's forces:~~
6. Create a node/py file that calls the metric recording starter and stopper so that we can add custom parameters into the starter.
7. Create evaluation scenarios



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