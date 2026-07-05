import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from std_msgs.msg import String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist


def clamp(value, low, high):
    return max(low, min(high, value))


class ManualJointStatePublisher(Node):
    def __init__(self):
        super().__init__("manual_joint_state_publisher_node")

        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.declare_parameter("joint_1_name", "joint_1_y")
        self.declare_parameter("joint_2_name", "joint_2_z")
        self.declare_parameter("jaw_joint_name", "mobile_jaw_joint_y")

        self.declare_parameter("joint_1_min", -0.314)
        self.declare_parameter("joint_1_max", 0.314)
        self.declare_parameter("joint_2_min", -0.314)
        self.declare_parameter("joint_2_max", 0.314)
        self.declare_parameter("jaw_min", 0.0)
        self.declare_parameter("jaw_max", 0.314)

        self.declare_parameter("joint_velocity_gain", 0.55)
        self.declare_parameter("jaw_velocity_gain", 0.45)

        self.declare_parameter("idle_home_joint_1", 0.0)
        self.declare_parameter("idle_home_joint_2", 0.0)
        self.declare_parameter("idle_home_jaw", 0.0)

        self.joint_names = [
            self.get_parameter("joint_1_name").value,
            self.get_parameter("joint_2_name").value,
            self.get_parameter("jaw_joint_name").value,
        ]

        self.limits = [
            (float(self.get_parameter("joint_1_min").value), float(self.get_parameter("joint_1_max").value)),
            (float(self.get_parameter("joint_2_min").value), float(self.get_parameter("joint_2_max").value)),
            (float(self.get_parameter("jaw_min").value), float(self.get_parameter("jaw_max").value)),
        ]

        self.home_position = [
            float(self.get_parameter("idle_home_joint_1").value),
            float(self.get_parameter("idle_home_joint_2").value),
            float(self.get_parameter("idle_home_jaw").value),
        ]

        self.joint_velocity_gain = float(self.get_parameter("joint_velocity_gain").value)
        self.jaw_velocity_gain = float(self.get_parameter("jaw_velocity_gain").value)

        self.mode = "IDLE"
        self.cmd = Twist()
        self.last_time = time.time()

        self.create_subscription(String, "/system/mode", self.on_mode, qos)
        self.create_subscription(Twist, "/manual/command", self.on_command, 10)

        self.pub = self.create_publisher(JointState, "/joint_states", 10)
        self.status_pub = self.create_publisher(String, "/system/status", 10)

        self.position = [0.0, 0.0, 0.0]
        self.velocity = [0.0, 0.0, 0.0]

        self.timer = self.create_timer(1.0 / 30.0, self.tick)

        self.get_logger().info("Manual RViz joint-state publisher ready.")
        self.get_logger().info(f"Publishing /joint_states for: {self.joint_names}")
        self.get_logger().info("R19C behavior: MANUAL_SIM moves, ESTOP freezes, IDLE instantly centers URDF.")

    def publish_status(self, text: str):
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)

    def on_mode(self, msg: String):
        if msg.data == self.mode:
            return

        self.mode = msg.data

        if self.mode == "ESTOP":
            self.cmd = Twist()
            self.velocity = [0.0, 0.0, 0.0]
            self.publish_joint_state()
            text = "ESTOP: URDF frozen at current position."
            self.get_logger().warn(text)
            self.publish_status(text)

        elif self.mode == "IDLE":
            self.cmd = Twist()
            self.position = list(self.home_position)
            self.velocity = [0.0, 0.0, 0.0]
            self.publish_joint_state()
            text = "IDLE: URDF returned to center/home."
            self.get_logger().info(text)
            self.publish_status(text)

        elif self.mode == "MANUAL_SIM":
            self.get_logger().info("Mode received: MANUAL_SIM")
            self.publish_status("MANUAL SIM: URDF follows keyboard/joystick.")

        elif self.mode == "MOVEIT_PLAN":
            self.cmd = Twist()
            self.velocity = [0.0, 0.0, 0.0]
            self.publish_joint_state()
            self.get_logger().info("Mode received: MOVEIT_PLAN")
            self.publish_status("MOVEIT PLAN: manual URDF simulator holds current state.")

        else:
            self.get_logger().info(f"Mode received: {self.mode}")

    def on_command(self, msg: Twist):
        if self.mode != "MANUAL_SIM":
            self.cmd = Twist()
            return

        self.cmd = msg

    def publish_joint_state(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = list(self.position)
        msg.velocity = list(self.velocity)
        msg.effort = [0.0, 0.0, 0.0]
        self.pub.publish(msg)

    def tick(self):
        now = time.time()
        dt = max(0.0, min(0.10, now - self.last_time))
        self.last_time = now

        if self.mode == "MANUAL_REAL":
            return

        if self.mode == "MANUAL_SIM":
            self.velocity[0] = float(self.cmd.linear.y) * self.joint_velocity_gain
            self.velocity[1] = float(self.cmd.linear.z) * self.joint_velocity_gain
            self.velocity[2] = float(self.cmd.angular.y) * self.jaw_velocity_gain

            for i in range(3):
                self.position[i] = clamp(
                    self.position[i] + self.velocity[i] * dt,
                    self.limits[i][0],
                    self.limits[i][1],
                )

        elif self.mode == "ESTOP":
            self.cmd = Twist()
            self.velocity = [0.0, 0.0, 0.0]

        elif self.mode == "IDLE":
            # Keep centered.
            self.cmd = Twist()
            self.position = list(self.home_position)
            self.velocity = [0.0, 0.0, 0.0]

        else:
            # MOVEIT_PLAN and unknown modes: hold current state.
            self.cmd = Twist()
            self.velocity = [0.0, 0.0, 0.0]

        self.publish_joint_state()


def main(args=None):
    rclpy.init(args=args)
    node = ManualJointStatePublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
