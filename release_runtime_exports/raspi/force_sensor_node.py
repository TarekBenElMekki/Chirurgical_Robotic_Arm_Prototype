#!/usr/bin/env python3
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String

try:
    import spidev
except Exception:
    spidev = None


class ForceSensorNode(Node):
    def __init__(self):
        super().__init__("force_sensor_node")

        self.declare_parameter("spi_bus", 0)
        self.declare_parameter("spi_device", 0)
        self.declare_parameter("adc_channel", 0)
        self.declare_parameter("vref", 3.3)
        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("force_limit_n", 15.0)
        self.declare_parameter("newton_per_count", 0.01)
        self.declare_parameter("auto_zero", True)
        self.declare_parameter("zero_samples", 30)

        self.spi_bus = int(self.get_parameter("spi_bus").value)
        self.spi_device = int(self.get_parameter("spi_device").value)
        self.adc_channel = int(self.get_parameter("adc_channel").value)
        self.vref = float(self.get_parameter("vref").value)
        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.force_limit_n = float(self.get_parameter("force_limit_n").value)
        self.newton_per_count = float(self.get_parameter("newton_per_count").value)
        self.auto_zero = bool(self.get_parameter("auto_zero").value)
        self.zero_samples = int(self.get_parameter("zero_samples").value)

        self.force_pub = self.create_publisher(Float32, "/force/newton", 10)
        self.limit_pub = self.create_publisher(Float32, "/force/limit_n", 10)
        self.raw_pub = self.create_publisher(Float32, "/force/raw", 10)
        self.voltage_pub = self.create_publisher(Float32, "/force/voltage", 10)
        self.log_pub = self.create_publisher(String, "/force/log", 10)

        self.spi = None
        self.zero_raw = 0.0
        self.last_log_time = 0.0

        self.open_spi()

        if self.spi is not None and self.auto_zero:
            self.zero_raw = self.measure_zero()
        else:
            self.zero_raw = 0.0

        self.publish_log(
            f"Force sensor node started: MCP3202 CH{self.adc_channel}, zero_raw={self.zero_raw:.1f}"
        )

        period = 1.0 / max(1.0, self.publish_rate_hz)
        self.timer = self.create_timer(period, self.tick)

    def publish_log(self, text: str):
        msg = String()
        msg.data = str(text)
        self.log_pub.publish(msg)
        self.get_logger().info(str(text))

    def open_spi(self):
        if spidev is None:
            self.publish_log("ERROR: python3-spidev is not installed")
            return

        try:
            self.spi = spidev.SpiDev()
            self.spi.open(self.spi_bus, self.spi_device)
            self.spi.max_speed_hz = 1000000
            self.spi.mode = 0
            self.publish_log(f"SPI opened: /dev/spidev{self.spi_bus}.{self.spi_device}")
        except Exception as exc:
            self.spi = None
            self.publish_log(f"ERROR: could not open SPI: {exc}")

    def read_mcp3202(self, channel: int) -> int:
        if self.spi is None:
            return -1

        channel = 1 if int(channel) == 1 else 0

        # MCP3202 single-ended read:
        # byte0 = start bit
        # byte1 = SGL/DIFF + channel selection
        # result = 12-bit value from response bytes
        response = self.spi.xfer2([0x01, (0x02 | channel) << 6, 0x00])
        raw = ((response[1] & 0x0F) << 8) | response[2]
        return int(raw)

    def measure_zero(self) -> float:
        samples = []
        for _ in range(max(1, self.zero_samples)):
            value = self.read_mcp3202(self.adc_channel)
            if value >= 0:
                samples.append(value)
            time.sleep(0.02)

        if not samples:
            return 0.0

        return sum(samples) / len(samples)

    def raw_to_voltage(self, raw: int) -> float:
        raw = max(0, min(4095, int(raw)))
        return (raw / 4095.0) * self.vref

    def raw_to_force_n(self, raw: int) -> float:
        adjusted = max(0.0, float(raw) - float(self.zero_raw))
        force_n = adjusted * self.newton_per_count
        return max(0.0, min(self.force_limit_n, force_n))

    def publish_float(self, pub, value: float):
        msg = Float32()
        msg.data = float(value)
        pub.publish(msg)

    def tick(self):
        raw = self.read_mcp3202(self.adc_channel)

        if raw < 0:
            now = time.time()
            if now - self.last_log_time > 5.0:
                self.last_log_time = now
                self.publish_log("Force sensor SPI read failed")
            return

        voltage = self.raw_to_voltage(raw)
        force_n = self.raw_to_force_n(raw)

        self.publish_float(self.raw_pub, float(raw))
        self.publish_float(self.voltage_pub, voltage)
        self.publish_float(self.force_pub, force_n)
        self.publish_float(self.limit_pub, self.force_limit_n)

        now = time.time()
        if now - self.last_log_time >= 0.20:
            self.last_log_time = now
            self.publish_log(
                f"Force sensor online: raw={raw}, voltage={voltage:.3f} V, force={force_n:.2f} N"
            )


def main(args=None):
    rclpy.init(args=args)
    node = ForceSensorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            if node.spi is not None:
                node.spi.close()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
