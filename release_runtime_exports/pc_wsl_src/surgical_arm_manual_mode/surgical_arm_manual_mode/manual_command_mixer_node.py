import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from std_msgs.msg import String
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist


class ManualCommandMixer(Node):
    def __init__(self):
        super().__init__("manual_command_mixer_node")

        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.declare_parameter("require_joystick_deadman_in_manual_sim", False)

        self.require_deadman = bool(self.get_parameter("require_joystick_deadman_in_manual_sim").value)

        self.mode = "IDLE"

        self.last_joy = None
        self.last_joy_time = 0.0

        self.last_keyboard = Twist()
        self.last_keyboard_time = 0.0

        self.create_subscription(String, "/system/mode", self.on_mode, qos)
        self.create_subscription(Joy, "/manual/joystick", self.on_joy, 10)
        self.create_subscription(Twist, "/manual/keyboard_cmd", self.on_keyboard, 10)

        self.pub = self.create_publisher(Twist, "/manual/command", 10)
        self.status_pub = self.create_publisher(String, "/system/status", 10)

        self.timer = self.create_timer(0.02, self.publish_command)

        self.get_logger().info("Manual command mixer ready.")
        self.get_logger().info("R19C: Keyboard has priority. Joystick works in MANUAL_SIM without deadman by default.")
        self.get_logger().info(f"Joystick deadman required in MANUAL_SIM: {self.require_deadman}")

    def publish_status(self, text: str):
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)

    def on_mode(self, msg: String):
        if msg.data != self.mode:
            self.mode = msg.data
            self.get_logger().info(f"Mode received: {self.mode}")

            if self.mode == "ESTOP":
                self.publish_status("ESTOP: manual commands blocked.")
            elif self.mode == "IDLE":
                self.publish_status("IDLE: manual commands blocked, URDF centered by state publisher.")
            elif self.mode == "MANUAL_SIM":
                self.publish_status("MANUAL SIM: keyboard and joystick enabled. Joystick deadman is not required.")
            elif self.mode == "MOVEIT_PLAN":
                self.publish_status("MOVEIT PLAN: manual commands blocked, Publish Point allowed.")

    def on_joy(self, msg: Joy):
        self.last_joy = msg
        self.last_joy_time = time.time()

    def on_keyboard(self, msg: Twist):
        self.last_keyboard = msg
        self.last_keyboard_time = time.time()

    def keyboard_active(self):
        cmd = self.last_keyboard
        active = (
            abs(cmd.linear.y) > 0.001
            or abs(cmd.linear.z) > 0.001
            or abs(cmd.angular.y) > 0.001
        )
        return active and (time.time() - self.last_keyboard_time < 0.30)

    def publish_command(self):
        out = Twist()

        if self.mode != "MANUAL_SIM":
            self.pub.publish(out)
            return

        # Keyboard has priority over joystick.
        if self.keyboard_active():
            self.pub.publish(self.last_keyboard)
            return

        if self.last_joy is None or time.time() - self.last_joy_time > 0.30:
            self.pub.publish(out)
            return

        axes = list(self.last_joy.axes)
        buttons = list(self.last_joy.buttons)

        b_pressed = len(buttons) > 0 and buttons[0] == 1
        d_pressed = len(buttons) > 1 and buttons[1] == 1

        if self.require_deadman and not d_pressed:
            self.pub.publish(out)
            return

        x_axis = axes[0] if len(axes) > 0 else 0.0
        y_axis = axes[1] if len(axes) > 1 else 0.0

        # Joystick controls the two visible joints directly in MANUAL_SIM.
        out.linear.y = float(x_axis)
        out.linear.z = float(y_axis)

        # Optional buttons for jaw:
        # B opens jaw, D closes jaw. If D is wired badly, joystick axes still work.
        if b_pressed:
            out.angular.y = 1.0
        elif d_pressed:
            out.angular.y = -1.0

        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ManualCommandMixer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
