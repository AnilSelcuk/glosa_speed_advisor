#!/usr/bin/env python3
"""
Helper for filling the "approaches" table of config/glosa.param.yaml.

Run it, then use the RViz "Publish Point" tool twice:
  1. click a point on the ego lane, 20 m or more BEFORE the stop line
  2. click the centre of the stop line of the ego lane

It prints stop_line_x, stop_line_y and heading_deg, ready to paste into the YAML.

    ros2 run glosa_speed_advisor stop_line_helper
"""
import math

import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node


def heading_deg(x1: float, y1: float, x2: float, y2: float) -> float:
    """Direction from point 1 to point 2 (0 = +x axis, counter-clockwise)."""
    return math.degrees(math.atan2(y2 - y1, x2 - x1))


class StopLineHelper(Node):
    def __init__(self):
        super().__init__("stop_line_helper")
        self.declare_parameter("topic", "/clicked_point")
        self.first = None
        self.create_subscription(PointStamped, self.get_parameter("topic").value, self.on_point, 10)
        self._ask_first()

    def _ask_first(self):
        self.get_logger().info("Click a point on the lane BEFORE the stop line (Publish Point tool)")

    def on_point(self, msg: PointStamped):
        x, y = msg.point.x, msg.point.y
        if self.first is None:
            self.first = (x, y)
            self.get_logger().info(f"got ({x:.2f}, {y:.2f}). Now click the centre of the STOP LINE")
            return
        x1, y1 = self.first
        self.first = None
        dist = math.hypot(x - x1, y - y1)
        if dist < 5.0:
            self.get_logger().warn(f"the two points are only {dist:.1f} m apart: heading may be inaccurate")
        print(f"\n        stop_line_x: {x:.2f}\n        stop_line_y: {y:.2f}\n"
              f"        heading_deg: {heading_deg(x1, y1, x, y):.1f}\n", flush=True)
        self._ask_first()


def main():
    rclpy.init()
    node = StopLineHelper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
