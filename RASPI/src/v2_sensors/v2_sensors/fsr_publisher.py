#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32, Float32

try:
    import spidev
except ImportError:
    spidev = None


class FsrPublisher(Node):
    def __init__(self):
        super().__init__("fsr_publisher")

        self.declare_parameter("spi_bus", 0)
        self.declare_parameter("spi_device", 0)
        self.declare_parameter("spi_speed_hz", 1000000)
        self.declare_parameter("publish_rate_hz", 30.0)

        self.declare_parameter("adc_max_value", 4095.0)
        self.declare_parameter("calibration_max_raw", 450.0)
        self.declare_parameter("force_max_n", 10.0)

        self.declare_parameter("raw_topic", "/fsr/raw")
        self.declare_parameter("normalized_topic", "/fsr/normalized")
        self.declare_parameter("force_topic", "/fsr/force_n")

        if spidev is None:
            raise RuntimeError(
                "Python module spidev not found. On Raspberry Pi run: sudo apt install python3-spidev"
            )

        self.spi_bus = int(self.get_parameter("spi_bus").value)
        self.spi_device = int(self.get_parameter("spi_device").value)
        self.spi_speed_hz = int(self.get_parameter("spi_speed_hz").value)
        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)

        self.adc_max_value = float(self.get_parameter("adc_max_value").value)
        self.calibration_max_raw = float(self.get_parameter("calibration_max_raw").value)
        self.force_max_n = float(self.get_parameter("force_max_n").value)

        self.raw_topic = self.get_parameter("raw_topic").value
        self.normalized_topic = self.get_parameter("normalized_topic").value
        self.force_topic = self.get_parameter("force_topic").value

        self.spi = spidev.SpiDev()
        self.spi.open(self.spi_bus, self.spi_device)
        self.spi.max_speed_hz = self.spi_speed_hz
        self.spi.mode = 0

        self.raw_pub = self.create_publisher(Int32, self.raw_topic, 10)
        self.norm_pub = self.create_publisher(Float32, self.normalized_topic, 10)
        self.force_pub = self.create_publisher(Float32, self.force_topic, 10)

        period = 1.0 / max(self.publish_rate_hz, 1.0)
        self.timer = self.create_timer(period, self.timer_callback)

        self.get_logger().info("Real MCP3202 FSR publisher started")
        self.get_logger().info(f"SPI: bus={self.spi_bus}, device={self.spi_device}, speed={self.spi_speed_hz}")
        self.get_logger().info(f"Topics: {self.raw_topic}, {self.normalized_topic}, {self.force_topic}")

    def read_adc_channel_0(self):
        # MCP3202 single-ended channel 0 command.
        resp = self.spi.xfer2([0b01101000, 0x00])
        return ((resp[0] & 0x0F) << 8) | resp[1]

    def timer_callback(self):
        raw = int(self.read_adc_channel_0())

        clamped = max(0.0, min(float(raw), self.calibration_max_raw))
        normalized = clamped / max(self.calibration_max_raw, 1.0)
        normalized = max(0.0, min(1.0, normalized))

        force_n = normalized * self.force_max_n

        self.raw_pub.publish(Int32(data=raw))
        self.norm_pub.publish(Float32(data=float(normalized)))
        self.force_pub.publish(Float32(data=float(force_n)))

    def destroy_node(self):
        try:
            if hasattr(self, "spi"):
                self.spi.close()
        except Exception as exc:
            print(f"[fsr_publisher] SPI cleanup warning: {exc}")

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        node = FsrPublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        print(f"[fsr_publisher] ERROR: {exc}")
    finally:
        if node is not None:
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
