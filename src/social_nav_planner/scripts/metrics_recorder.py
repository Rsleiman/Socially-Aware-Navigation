# Use Hunav Evaluator to compute metrics after a simulation
# 1. Create a function that calls the start recording service
# 2. Input request params according to the StartEvaluation.srv file:
    # # agents and robot
    # geometry_msgs/PoseStamped robot_goal
    # string experiment_tag "exp_1"
    # int32 run_id 0
    # ---
# 3. Create a function that calls the stop recording service (no params needed)

# TODO:
# - Add a NavigateToPose done_callback so we can call the StopEvaluation service once goal pose has been navigated to. 
# - Replace robot_goal topic subscriber with a callback that activates once a goal action is sent
# - Abort evaluation functionality in case of interruptions (e.g. new goal, shutdown, manaual stop)
# - Should we stack goals? If so, implement a queue system.

import rclpy
from rclpy.node import Node
from hunav_sim.hunav_msgs.srv import StartEvaluation, StopEvaluation
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose

class MetricsRecorder(Node):
    def __init__(self):
        super().__init__('metrics_recorder')
        self.cli_start = self.create_client(StartEvaluation, 'start_evaluation')
        self.cli_stop = self.create_client(StopEvaluation, 'stop_evaluation')
        self.goal_sub = self.create_subscription(PoseStamped, 'robot_goal', self.goal_callback, 10)
        self.metric_count = 0
        self.experiment_tag = self.declare_parameter('experiment_tag', 'exp_1').get_parameter_value().string_value # Pass metrics.yaml into parameters argument in launch file

    # Send start request when a new goal is received
    def goal_callback(self, msg):
        req = StartEvaluation.Request()

        req.robot_goal = msg
        req.experiment_tag = self.experiment_tag
        req.run_id = self.metric_count

        self.future_start = self.cli_start.call_async(req)
        rclpy.spin_until_future_complete(self, self.future_start)

        if self.future_start.result() is not None: #TODO: Check whether the output of .result is a None vs Not None or a true vs false (i.e the response of StartEvaluation.srv)
            self.get_logger().info('Evaluation started for run_id: %d' % self.metric_count)
        else:
            self.get_logger().error('Exception while calling service: %r' % self.future_start.exception())
        
        self.metric_count += 1


    # Functionality to send stop request once goal has been reached (or once timeout complete?).
    def placeholder(self):
        pass

def main(args=None):
    rclpy.init(args=args)
    metrics_recorder = MetricsRecorder()

    try:
        rclpy.spin(metrics_recorder)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up and shutdown
        if rclpy.ok():
            metrics_recorder.get_logger().info("Shutting down metrics recorder node...")
            metrics_recorder.destroy_node()
            rclpy.shutdown()