#!/usr/bin/env python3
"""
ROS 2 node: provides /api/routing/set_route_points and generates lanelet2 from route.
"""

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from autoware_adapi_v1_msgs.srv import SetRoutePoints

from lanelet2_generator import generate


def pose_to_array(pose: Pose) -> np.ndarray:
    """Convert geometry_msgs/Pose to [x, y, z, qx, qy, qz, qw]."""
    p = pose.position
    o = pose.orientation
    return np.array([p.x, p.y, p.z, o.x, o.y, o.z, o.w], dtype=np.float64)


_SERVICE_TOPIC = "/api/routing/set_route_points"


class RouteToLaneletNode(Node):
    def __init__(self):
        super().__init__("route_to_lanelet_node")

        self.declare_parameter("output_path", "")
        self.declare_parameter("mgrs", "33TWN")
        self.declare_parameter("width", 2.0)
        self.declare_parameter("speed_limit", 30)
        self.declare_parameter("min_distance", "")
        self.declare_parameter("step", 1)
        self.declare_parameter("split_distance", "500")
        self.declare_parameter("split_direction_deg", "")
        self.declare_parameter("split_direction_window_m", "")
        self.declare_parameter("bidirectional", True)

        self.srv = self.create_service(
            SetRoutePoints,
            _SERVICE_TOPIC,
            self._handle_set_route_points,
        )
        self.get_logger().info(f"Service {_SERVICE_TOPIC} ready")

    def _handle_set_route_points(self, request, response):
        output_path = self.get_parameter("output_path").get_parameter_value().string_value
        if not output_path:
            self.get_logger().error("output_path parameter not set")
            response.status.success = False
            response.status.message = "output_path not configured in launch file"
            return response

        waypoints = list(request.waypoints)
        goal = request.goal
        poses_list = [pose_to_array(p) for p in waypoints] + [pose_to_array(goal)]
        if len(poses_list) < 2:
            response.status.success = False
            response.status.message = "Need at least goal; waypoints + goal recommended"
            return response

        poses = np.array(poses_list)

        def _opt_float(val):
            if val is None or val == "":
                return None
            try:
                return float(val)
            except (ValueError, TypeError):
                return None

        mgrs = self.get_parameter("mgrs").get_parameter_value().string_value
        width = float(self.get_parameter("width").value)
        speed_limit = float(self.get_parameter("speed_limit").value)
        min_distance = _opt_float(self.get_parameter("min_distance").value)
        step = self.get_parameter("step").get_parameter_value().integer_value
        split_distance = _opt_float(self.get_parameter("split_distance").value)
        max_deg = _opt_float(self.get_parameter("split_direction_deg").value)
        window_m = _opt_float(self.get_parameter("split_direction_window_m").value)
        bidirectional = bool(self.get_parameter("bidirectional").value)

        try:
            result = generate(
                output_dir=output_path,
                poses=poses,
                width=width,
                mgrs=mgrs,
                min_distance=min_distance,
                step=step,
                split_distance=split_distance,
                max_direction_change_deg=max_deg,
                direction_change_window_m=window_m,
                speed_limit=speed_limit,
                bidirectional=bidirectional,
            )
            self.get_logger().info(f"Generated lanelet2 map: {result}")
            response.status.success = True
            response.status.message = f"Saved to {result}"
        except Exception as e:
            self.get_logger().error(f"Lanelet generation failed: {e}")
            response.status.success = False
            response.status.message = str(e)

        return response


def main(args=None):
    rclpy.init(args=args)
    node = RouteToLaneletNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
