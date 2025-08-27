#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header

from a_star_planner import PathPlanner

class PathPlannerNode(Node):
    def __init__(self):
        super().__init__("path_planner_node")
        self.get_logger().info("Path Planner Node Initialised")

        self.map_sub = self.create_subscription(OccupancyGrid, "/map", self.map_callback, 10) #may have to be "/map"
        self.path_pub = self.create_publisher(Path, "/planned_path", 10)

        self.planner = PathPlanner()

        self.start = (0, 0) # In pixel coordinates
        self.goal = (100, 100)

    def map_callback(self, msg: OccupancyGrid):
        self.get_logger().info("Received map data")

        width = msg.info.width
        height = msg.info.height
        data = msg.data

        grid = [[0 if data[y*width + x] < 50 else 1 for x in range(width)] for y in range(height)]
        self.planner.grid = grid
        self.planner.set_start_and_goal(self.start, self.goal)

        path = self.planner.find_path()
        if path:
            # Debugging
            print(f"A* Path output: {path}")
            print(f"Resolution: {msg.info.resolution}") # If resolution = x, each cell/pixel in the grid = x metres × x metres in the real world.
            print(f"Origin: {msg.info.origin.position}")
            self.get_logger().info("Path found, publishing...")
            ros_path = self.path_to_msg(path, msg.info.resolution, msg.info.origin.position)
            self.path_pub.publish(ros_path)
            self.get_logger().info("Published planned path")
        else:
            self.get_logger().warn("No path found")

    def path_to_msg(self, path, resolution, origin):
        ros_path = Path()
        ros_path.header = Header()
        ros_path.header.stamp = self.get_clock().now().to_msg()
        ros_path.header.frame_id = "map" # Or odom?

        for (x, y) in path:
            pose = PoseStamped()
            pose.header = ros_path.header
            pose.pose.position.x = origin.x + x * resolution # Maybe do origin.x + x * 0.5 * resolution to centre the pose in the cell.
            pose.pose.position.y = origin.y + y * resolution # Maybe do origin.x + x * 0.5 * resolution to centre the pose in the cell.
            pose.pose.orientation.w = 1.0
            ros_path.poses.append(pose)

        return ros_path
    
def main(args=None):
    rclpy.init(args=args)
    node = PathPlannerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()