#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Float32

import spidev


class FsrMcp3202Publisher(Node):
    def __init__(self):
        super().__init__("fsr_mcp3202_publisher")

        self.spi = spidev.SpiDev()
        self.spi.open(0, 0)
        self.spi.max_speed_hz = 1000000
        self.spi.mode = 0

        self.pub_raw = self.create_publisher(Int32, "/fsr/raw", 10)
        self.pub_norm = self.create_publisher(Float32, "/fsr/normalized", 10)

        self.timer = self.create_timer(0.1, self.loop)

        self.get_logger().info("Publisher STARTED (scaled to 450)")

    def read_adc(self):
        resp = self.spi.xfer2([0b01101000, 0x00])
        return ((resp[0] & 0x0F) << 8) | resp[1]

    def loop(self):
        adc = self.read_adc()

        # 🔴 SCALE TO 450 INSTEAD OF 4095
        adc_clamped = min(adc, 450.0)

        norm = adc_clamped / 450.0
        norm = max(0.0, min(1.0, norm))

        self.get_logger().info(f"ADC={adc}  NORM={norm:.2f}")

        self.pub_raw.publish(Int32(data=int(adc)))
        self.pub_norm.publish(Float32(data=float(norm)))


def main(args=None):
    rclpy.init(args=args)
    node = FsrMcp3202Publisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
