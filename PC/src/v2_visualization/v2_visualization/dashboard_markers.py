#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32
from visualization_msgs.msg import Marker, MarkerArray


class DashboardMarkers(Node):
    def __init__(self):
        super().__init__("dashboard_markers")

        self.fsr_normalized = None
        self.motor_command_deg = None
        self.motor_real_deg = None

        self.pub = self.create_publisher(MarkerArray, "/dashboard/markers", 10)

        self.create_subscription(Float32, "/fsr/normalized", self.fsr_cb, 10)
        self.create_subscription(Float32, "/motor/command_angle_deg", self.motor_command_cb, 10)
        self.create_subscription(Float32, "/motor/real_angle_deg", self.motor_real_cb, 10)

        self.timer = self.create_timer(0.2, self.publish_markers)

        self.get_logger().info("Dashboard markers started")
        self.get_logger().info("Waiting for real topics: /fsr/normalized, /motor/command_angle_deg, /motor/real_angle_deg")

    def fsr_cb(self, msg):
        self.fsr_normalized = max(0.0, min(1.0, float(msg.data)))

    def motor_command_cb(self, msg):
        self.motor_command_deg = float(msg.data)

    def motor_real_cb(self, msg):
        self.motor_real_deg = float(msg.data)

    def make_text_marker(self, marker_id, text, x, y, z, scale=0.08):
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "dashboard_text"
        marker.id = marker_id
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = float(x)
        marker.pose.position.y = float(y)
        marker.pose.position.z = float(z)
        marker.pose.orientation.w = 1.0
        marker.scale.z = float(scale)
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0
        marker.text = text
        return marker

    def make_fsr_bar(self):
        value = 0.0 if self.fsr_normalized is None else self.fsr_normalized
        height = max(0.02, 0.45 * value)

        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "fsr_level"
        marker.id = 100
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = -0.45
        marker.pose.position.y = -0.35
        marker.pose.position.z = 0.08 + height / 2.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.08
        marker.scale.y = 0.08
        marker.scale.z = height

        marker.color.r = value
        marker.color.g = 1.0 - value
        marker.color.b = 0.1
        marker.color.a = 0.9

        return marker

    def publish_markers(self):
        markers = MarkerArray()

        markers.markers.append(self.make_text_marker(
            1,
            "FSR LEVEL",
            -0.45,
            -0.35,
            0.62,
            0.07
        ))

        if self.fsr_normalized is None:
            fsr_text = "FSR: waiting for real data"
        else:
            fsr_text = f"FSR: {self.fsr_normalized * 100.0:.1f}%"

        markers.markers.append(self.make_text_marker(
            2,
            fsr_text,
            -0.45,
            -0.35,
            0.54,
            0.055
        ))

        markers.markers.append(self.make_fsr_bar())

        if self.motor_command_deg is None:
            command_text = "Command angle: waiting"
        else:
            command_text = f"Command angle: {self.motor_command_deg:.1f} deg"

        if self.motor_real_deg is None:
            real_text = "Real angle: waiting"
        else:
            real_text = f"Real angle: {self.motor_real_deg:.1f} deg"

        if self.motor_command_deg is not None and self.motor_real_deg is not None:
            error = self.motor_command_deg - self.motor_real_deg
            error_text = f"Tracking error: {error:.1f} deg"
        else:
            error_text = "Tracking error: waiting"

        markers.markers.append(self.make_text_marker(10, "MOTOR TELEMETRY", 0.45, -0.35, 0.62, 0.07))
        markers.markers.append(self.make_text_marker(11, command_text, 0.45, -0.35, 0.54, 0.055))
        markers.markers.append(self.make_text_marker(12, real_text, 0.45, -0.35, 0.47, 0.055))
        markers.markers.append(self.make_text_marker(13, error_text, 0.45, -0.35, 0.40, 0.055))

        self.pub.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = DashboardMarkers()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
