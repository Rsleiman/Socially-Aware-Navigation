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
