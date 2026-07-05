#!/usr/bin/env python3
import threading
import time

import cv2
import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32
from sensor_msgs.msg import Image
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point


class RvizCameraForceBridge(Node):
    def __init__(self):
        super().__init__("rviz_camera_force_bridge")

        self.declare_parameter("stream_url", "http://100.112.77.4:8081/stream.mjpg")
        self.declare_parameter("camera_topic", "/camera/image_raw")
        self.declare_parameter("force_topic", "/force/newton")
        self.declare_parameter("marker_topic", "/force/rviz_marker")
        self.declare_parameter("frame_id", "base_link")
        self.declare_parameter("publish_fps", 15.0)
        self.declare_parameter("force_limit_n", 15.0)

        self.stream_url = str(self.get_parameter("stream_url").value)
        self.camera_topic = str(self.get_parameter("camera_topic").value)
        self.force_topic = str(self.get_parameter("force_topic").value)
        self.marker_topic = str(self.get_parameter("marker_topic").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.publish_fps = float(self.get_parameter("publish_fps").value)
        self.force_limit_n = float(self.get_parameter("force_limit_n").value)

        self.image_pub = self.create_publisher(Image, self.camera_topic, 10)
        self.marker_pub = self.create_publisher(Marker, self.marker_topic, 10)

        self.force_value = 0.0
        self.latest_frame = None
        self.latest_frame_time = 0.0
        self.lock = threading.Lock()
        self.running = True

        self.create_subscription(Float32, self.force_topic, self.on_force, 10)

        self.capture_thread = threading.Thread(target=self.capture_loop, daemon=True)
        self.capture_thread.start()

        period = 1.0 / max(1.0, self.publish_fps)
        self.timer = self.create_timer(period, self.publish_tick)

        self.get_logger().info(f"RViz bridge camera input: {self.stream_url}")
        self.get_logger().info(f"Publishing camera topic: {self.camera_topic}")
        self.get_logger().info(f"Subscribing force topic: {self.force_topic}")
        self.get_logger().info(f"Publishing force marker: {self.marker_topic}")

    def on_force(self, msg):
        self.force_value = float(msg.data)

    def capture_loop(self):
        while self.running:
            cap = cv2.VideoCapture(self.stream_url)
            try:
                cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            except Exception:
                pass

            if not cap.isOpened():
                self.get_logger().warn("Could not open camera stream for RViz; retrying...")
                time.sleep(1.0)
                continue

            self.get_logger().info("RViz camera stream opened")

            while self.running:
                ok, frame = cap.read()

                if not ok or frame is None:
                    self.get_logger().warn("Camera stream read failed; reconnecting...")
                    break

                with self.lock:
                    self.latest_frame = frame
                    self.latest_frame_time = time.time()

            try:
                cap.release()
            except Exception:
                pass

            time.sleep(0.5)

    def publish_tick(self):
        self.publish_latest_image()
        self.publish_force_marker()

    def publish_latest_image(self):
        with self.lock:
            frame = None if self.latest_frame is None else self.latest_frame.copy()

        if frame is None:
            return

        h, w = frame.shape[:2]

        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera_frame"
        msg.height = h
        msg.width = w
        msg.encoding = "bgr8"
        msg.is_bigendian = False
        msg.step = w * 3
        msg.data = frame.tobytes()

        self.image_pub.publish(msg)

    def publish_force_marker(self):
        force = max(0.0, min(self.force_value, self.force_limit_n))
        ratio = force / max(0.001, self.force_limit_n)

        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = self.frame_id
        marker.ns = "fsr_force"
        marker.id = 1
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD

        marker.pose.position.x = 0.30
        marker.pose.position.y = -0.35
        marker.pose.position.z = 0.45

        marker.pose.orientation.w = 1.0

        marker.scale.z = 0.045
        marker.text = f"FSR FORCE: {force:.2f} N / {self.force_limit_n:.0f} N"

        if ratio < 0.60:
            marker.color.r = 0.1
            marker.color.g = 1.0
            marker.color.b = 0.25
        elif ratio < 0.85:
            marker.color.r = 1.0
            marker.color.g = 0.75
            marker.color.b = 0.05
        else:
            marker.color.r = 1.0
            marker.color.g = 0.1
            marker.color.b = 0.05

        marker.color.a = 1.0

        self.marker_pub.publish(marker)

    def destroy_node(self):
        self.running = False
        super().destroy_node()


def main():
    rclpy.init()
    node = RvizCameraForceBridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
