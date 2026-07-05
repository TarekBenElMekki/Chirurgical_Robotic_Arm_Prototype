import socket
import time

import rclpy
from rclpy.node import Node

from std_msgs.msg import Empty, String
from sensor_msgs.msg import Joy


def clamp(value, low, high):
    return max(low, min(high, value))


class ManualUdpJoystickReceiver(Node):
    def __init__(self):
        super().__init__("manual_udp_joystick_receiver_node")

        self.declare_parameter("listen_ip", "0.0.0.0")
        self.declare_parameter("listen_port", 5005)
        self.declare_parameter("x_center", 513.0)
        self.declare_parameter("y_center", 520.0)
        self.declare_parameter("deadzone", 0.08)

        self.listen_ip = self.get_parameter("listen_ip").value
        self.listen_port = int(self.get_parameter("listen_port").value)
        self.x_center = float(self.get_parameter("x_center").value)
        self.y_center = float(self.get_parameter("y_center").value)
        self.deadzone = float(self.get_parameter("deadzone").value)

        self.last_packet_time = 0.0
        self.last_seq = None
        self.refresh_requested_time = time.time()

        self.pub = self.create_publisher(Joy, "/manual/joystick", 10)
        self.status_pub = self.create_publisher(String, "/manual/joystick_status", 10)
        self.create_subscription(Empty, "/manual/joystick_refresh", self.on_refresh, 10)

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((self.listen_ip, self.listen_port))
        self.sock.settimeout(0.03)

        self.get_logger().info(f"Listening for optional Windows joystick UDP on {self.listen_ip}:{self.listen_port}")
        self.get_logger().info("Joystick is OPTIONAL. Keyboard manual mode works without joystick.")

        self.udp_timer = self.create_timer(0.01, self.poll_udp)
        self.status_timer = self.create_timer(0.50, self.publish_status)

    def on_refresh(self, _msg):
        self.refresh_requested_time = time.time()
        self.get_logger().info("Joystick refresh/search requested from dock.")
        self.publish_status(force_searching=True)

    def publish_status(self, force_searching=False):
        now = time.time()
        msg = String()

        if force_searching:
            msg.data = "Joystick: SEARCHING..."
        elif self.last_packet_time <= 0:
            msg.data = "Joystick: NOT CONNECTED"
        elif now - self.last_packet_time < 1.5:
            seq = "?" if self.last_seq is None else str(self.last_seq)
            msg.data = f"Joystick: CONNECTED UDP seq={seq}"
        else:
            msg.data = "Joystick: LOST"

        self.status_pub.publish(msg)

    def parse_payload(self, text: str):
        fields = {}
        for part in text.strip().split(","):
            if "=" not in part:
                continue
            k, v = part.split("=", 1)
            fields[k.strip()] = v.strip()

        seq = int(fields.get("SEQ", "0"))
        x = int(fields["X"])
        y = int(fields["Y"])
        b = int(fields.get("B", "0"))
        d = int(fields.get("D", "0"))
        return seq, x, y, b, d

    def normalize_axis(self, raw: int, center: float):
        value = (float(raw) - center) / 512.0
        value = clamp(value, -1.0, 1.0)
        if abs(value) < self.deadzone:
            return 0.0
        return value

    def poll_udp(self):
        for _ in range(10):
            try:
                data, _addr = self.sock.recvfrom(2048)
            except socket.timeout:
                return
            except Exception as exc:
                self.get_logger().warn(f"UDP receive error: {exc}")
                return

            text = data.decode("utf-8", errors="ignore").strip()

            try:
                seq, x, y, b, d = self.parse_payload(text)
            except Exception:
                self.get_logger().warn(f"Ignored malformed joystick packet: {text}")
                continue

            self.last_packet_time = time.time()
            self.last_seq = seq

            msg = Joy()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.axes = [
                self.normalize_axis(x, self.x_center),
                self.normalize_axis(y, self.y_center),
            ]
            msg.buttons = [int(b), int(d)]
            self.pub.publish(msg)

            if seq % 100 == 0:
                self.get_logger().info(
                    f"JOY seq={seq} axes=({msg.axes[0]:+.2f},{msg.axes[1]:+.2f}) B={b} D={d}"
                )


def main(args=None):
    rclpy.init(args=args)
    node = ManualUdpJoystickReceiver()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
