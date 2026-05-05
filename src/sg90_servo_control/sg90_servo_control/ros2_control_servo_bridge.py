#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from control_msgs.msg import JointTrajectoryControllerState
from std_msgs.msg import Float32


class Ros2ControlServoBridge(Node):
    def __init__(self):
        super().__init__("ros2_control_servo_bridge")

        self.declare_parameter("controller_state_topic", "/hardware_arm_controller/controller_state")
        self.declare_parameter("servo_command_topic", "/sg90/angle_deg")
        self.declare_parameter("controlled_joint", "base_yaw_joint")

        self.declare_parameter("joint_min_rad", -1.5708)
        self.declare_parameter("joint_max_rad", 1.5708)
        self.declare_parameter("servo_min_deg", 0.0)
        self.declare_parameter("servo_max_deg", 180.0)
        self.declare_parameter("invert_direction", False)
        self.declare_parameter("min_publish_delta_deg", 0.5)

        self.controller_state_topic = self.get_parameter("controller_state_topic").value
        self.servo_command_topic = self.get_parameter("servo_command_topic").value
        self.controlled_joint = self.get_parameter("controlled_joint").value

        self.joint_min_rad = float(self.get_parameter("joint_min_rad").value)
        self.joint_max_rad = float(self.get_parameter("joint_max_rad").value)
        self.servo_min_deg = float(self.get_parameter("servo_min_deg").value)
        self.servo_max_deg = float(self.get_parameter("servo_max_deg").value)
        self.invert_direction = bool(self.get_parameter("invert_direction").value)
        self.min_publish_delta_deg = float(self.get_parameter("min_publish_delta_deg").value)

        self.last_published_servo_deg = None

        self.servo_pub = self.create_publisher(Float32, self.servo_command_topic, 10)

        self.create_subscription(
            JointTrajectoryControllerState,
            self.controller_state_topic,
            self.controller_state_callback,
            10,
        )

        self.get_logger().info("ROS2 control to SG90 bridge started")
        self.get_logger().info(f"Controller state topic: {self.controller_state_topic}")
        self.get_logger().info(f"Servo command topic: {self.servo_command_topic}")
        self.get_logger().info(f"Controlled joint: {self.controlled_joint}")

    def clamp(self, value, low, high):
        return max(low, min(high, float(value)))

    def joint_rad_to_servo_deg(self, joint_rad):
        joint_rad = self.clamp(joint_rad, self.joint_min_rad, self.joint_max_rad)

        if abs(self.joint_max_rad - self.joint_min_rad) < 1e-9:
            ratio = 0.5
        else:
            ratio = (joint_rad - self.joint_min_rad) / (self.joint_max_rad - self.joint_min_rad)

        if self.invert_direction:
            ratio = 1.0 - ratio

        servo_deg = self.servo_min_deg + ratio * (self.servo_max_deg - self.servo_min_deg)
        return self.clamp(servo_deg, self.servo_min_deg, self.servo_max_deg)

    def should_publish(self, servo_deg):
        if self.last_published_servo_deg is None:
            return True
        return abs(servo_deg - self.last_published_servo_deg) >= self.min_publish_delta_deg

    def controller_state_callback(self, msg):
        joint_names = list(msg.joint_names)

        if self.controlled_joint not in joint_names:
            self.get_logger().warn(
                f"Joint {self.controlled_joint} not found in controller state joints: {joint_names}",
                throttle_duration_sec=2.0,
            )
            return

        joint_index = joint_names.index(self.controlled_joint)

        if joint_index >= len(msg.desired.positions):
            self.get_logger().warn(
                "desired.positions does not contain controlled joint index",
                throttle_duration_sec=2.0,
            )
            return

        desired_joint_rad = float(msg.desired.positions[joint_index])
        servo_deg = self.joint_rad_to_servo_deg(desired_joint_rad)

        if not self.should_publish(servo_deg):
            return

        out = Float32()
        out.data = float(servo_deg)
        self.servo_pub.publish(out)
        self.last_published_servo_deg = servo_deg

        self.get_logger().info(
            f"{self.controlled_joint}: {desired_joint_rad:.4f} rad "
            f"({math.degrees(desired_joint_rad):.1f} deg) -> SG90 {servo_deg:.1f} deg"
        )


def main(args=None):
    rclpy.init(args=args)
    node = Ros2ControlServoBridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()

    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
