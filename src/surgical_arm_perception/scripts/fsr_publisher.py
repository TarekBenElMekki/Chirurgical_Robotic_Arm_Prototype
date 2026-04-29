#!/usr/bin/env python3
import math
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, Float32MultiArray


class FsrPublisher(Node):
    def __init__(self):
        super().__init__('fsr_publisher')

        self.declare_parameter('mock_mode', True)
        self.declare_parameter('publish_rate_hz', 30.0)
        self.declare_parameter('sensor_count', 4)
        self.declare_parameter('topic_raw', '/fsr/raw')
        self.declare_parameter('topic_normalized', '/fsr/normalized')

        self.mock_mode = bool(self.get_parameter('mock_mode').value)
        self.rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self.sensor_count = int(self.get_parameter('sensor_count').value)
        self.topic_raw = self.get_parameter('topic_raw').value
        self.topic_normalized = self.get_parameter('topic_normalized').value

        self.raw_pub = self.create_publisher(Int32MultiArray, self.topic_raw, 10)
        self.norm_pub = self.create_publisher(Float32MultiArray, self.topic_normalized, 10)

        period = 1.0 / max(self.rate_hz, 1.0)
        self.timer = self.create_timer(period, self.timer_cb)

        self.get_logger().info(
            f'FSR publisher started. mock_mode={self.mock_mode}, '
            f'sensors={self.sensor_count}, rate={self.rate_hz} Hz'
        )

        if self.mock_mode:
            self.get_logger().warn('FSR is running in MOCK mode. Replace with ADC acquisition later.')
        else:
            self.get_logger().warn('Real ADC mode not implemented yet. Raspberry Pi needs an external ADC for FSR analog input.')

    def read_mock_values(self):
        t = time.time()
        values = []
        for i in range(self.sensor_count):
            v = 2048 + 1200 * math.sin(t * 1.5 + i)
            values.append(int(max(0, min(4095, v))))
        return values

    def timer_cb(self):
        if self.mock_mode:
            raw_values = self.read_mock_values()
        else:
            raw_values = [0] * self.sensor_count

        norm_values = [float(v) / 4095.0 for v in raw_values]

        raw_msg = Int32MultiArray()
        raw_msg.data = raw_values
        self.raw_pub.publish(raw_msg)

        norm_msg = Float32MultiArray()
        norm_msg.data = norm_values
        self.norm_pub.publish(norm_msg)


def main():
    rclpy.init()
    node = FsrPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
