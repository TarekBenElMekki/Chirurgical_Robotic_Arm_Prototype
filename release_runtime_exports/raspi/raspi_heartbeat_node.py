#!/usr/bin/env python3
import socket
import subprocess
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String


class RaspiHeartbeatNode(Node):
    def __init__(self):
        super().__init__("raspi_heartbeat_node")

        self.declare_parameter("target_host", "")
        self.declare_parameter("publish_rate_hz", 1.0)

        self.target_host = str(self.get_parameter("target_host").value)
        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)

        self.latency_pub = self.create_publisher(Float32, "/raspi/link_latency_ms", 10)
        self.log_pub = self.create_publisher(String, "/system/log", 10)

        self.start_time = time.time()
        self.last_log_time = 0.0

        period = 1.0 / max(0.1, self.publish_rate_hz)
        self.timer = self.create_timer(period, self.tick)

        self.publish_log("Raspberry Pi heartbeat node started")

        if self.target_host:
            self.publish_log(f"Latency target: {self.target_host}")
        else:
            self.publish_log("No latency target configured; publishing heartbeat latency")

    def publish_log(self, text: str):
        msg = String()
        msg.data = str(text)
        self.log_pub.publish(msg)
        self.get_logger().info(str(text))

    def ping_latency_ms(self) -> float:
        if not self.target_host:
            return 0.0

        try:
            result = subprocess.run(
                ["ping", "-c", "1", "-W", "1", self.target_host],
                capture_output=True,
                text=True,
                timeout=2.0,
            )

            if result.returncode != 0:
                return -1.0

            for line in result.stdout.splitlines():
                if "time=" in line:
                    value = line.split("time=", 1)[1].split(" ", 1)[0]
                    return float(value)

            return -1.0

        except Exception:
            return -1.0

    def tick(self):
        latency = self.ping_latency_ms()

        msg = Float32()
        msg.data = float(latency if latency >= 0.0 else 999.0)
        self.latency_pub.publish(msg)

        now = time.time()
        if now - self.last_log_time >= 1.0:
            self.last_log_time = now
            hostname = socket.gethostname()
            uptime = int(now - self.start_time)

            if latency >= 0.0:
                self.publish_log(
                    f"Raspberry Pi online: {hostname}, latency {latency:.1f} ms, uptime {uptime}s"
                )
            else:
                self.publish_log(
                    f"Raspberry Pi online: {hostname}, PC ping unavailable, uptime {uptime}s"
                )


def main(args=None):
    rclpy.init(args=args)
    node = RaspiHeartbeatNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
