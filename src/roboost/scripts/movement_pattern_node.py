#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from ament_index_python import get_package_share_directory
import yaml
import os


class DataCollectionNode(Node):
    def __init__(self):
        super().__init__("movement_pattern_node")
        self.publisher_ = self.create_publisher(Twist, "/cmd_vel", 10)

        # Load movement patterns from the YAML file
        self.movement_patterns = self.load_movement_patterns()
        self.current_pattern_index = 0
        self.start_time = self.get_clock().now()

        self.timer = self.create_timer(0.1, self.timer_callback)

    def load_movement_patterns(self):
        package_share_directory = get_package_share_directory("roboost")
        conf_file = os.path.join(
            package_share_directory, "config", "movement_patterns.yaml"
        )
        with open(conf_file, "r") as file:
            config = yaml.safe_load(file)
        return config["movement_patterns"]

    def timer_callback(self):
        current_time = self.get_clock().now()
        pattern = self.movement_patterns[self.current_pattern_index]
        elapsed = (current_time - self.start_time).nanoseconds * 1e-9

        if elapsed > pattern["duration"]:
            self.current_pattern_index += 1
            if self.current_pattern_index >= len(self.movement_patterns):
                self.current_pattern_index = 0  # Restart the sequence
            self.start_time = current_time
            return

        twist = Twist()
        if "x" in pattern:
            twist.linear.x = pattern["x"]
        if "y" in pattern:
            twist.linear.y = pattern["y"]
        if "z" in pattern:
            twist.angular.z = pattern["z"]
        self.publisher_.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    data_collection_node = DataCollectionNode()
    rclpy.spin(data_collection_node)
    data_collection_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
