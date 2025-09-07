# Use Hunav Evaluator to compute metrics after a simulation
# 1. Create a function that calls the start recording service
# 2. Input request params according to the StartEvaluation.srv file:
    # # agents and robot
    # geometry_msgs/PoseStamped robot_goal
    # string experiment_tag "exp_1"
    # int32 run_id 0
    # ---
# 3. Create a function that calls the stop recording service (no params needed)