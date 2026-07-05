import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import PointStamped
from std_msgs.msg import String


class RvizClickedPointModeGate(Node):
    def __init__(self):
        super().__init__("rviz_clicked_point_mode_gate_node")

        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.mode = "IDLE"

        self.create_subscription(String, "/system/mode", self.on_mode, qos)
        self.create_subscription(PointStamped, "/clicked_point", self.on_clicked_point, 10)

        self.forward_pub = self.create_publisher(PointStamped, "/mode_guarded/clicked_point", 10)
        self.status_pub = self.create_publisher(String, "/system/status", 10)

        self.get_logger().info("RViz Publish Point mode gate ready.")
        self.get_logger().info("Publish Point is forwarded only in MOVEIT_PLAN mode.")

    def on_mode(self, msg: String):
        if msg.data != self.mode:
            self.mode = msg.data
            self.get_logger().info(f"Mode received: {self.mode}")

    def publish_status(self, text: str):
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)

    def on_clicked_point(self, msg: PointStamped):
        if self.mode != "MOVEIT_PLAN":
            text = f"RViz Publish Point blocked. Current mode is {self.mode}. Select MOVEIT PLAN first."
            self.get_logger().warn(text)
            self.publish_status(text)
            return

        self.get_logger().info("RViz Publish Point accepted in MOVEIT_PLAN mode.")
        self.publish_status("RViz Publish Point accepted in MOVEIT PLAN mode.")
        self.forward_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = RvizClickedPointModeGate()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
