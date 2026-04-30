#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32, Float32

try:
    import spidev
except ImportError:
    spidev = None


class FsrMcp3202Publisher(Node):
    def __init__(self):
        super().__init__("fsr_mcp3202_publisher")

        self.declare_parameter("bus", 0)
        self.declare_parameter("device", 0)
        self.declare_parameter("sample_rate", 20.0)

        self.declare_parameter("vcc", 3.3)
        self.declare_parameter("r_fixed", 10000.0)

        self.declare_parameter("adc_min_calib", 0.0)
        self.declare_parameter("adc_max_calib", 250.0)
        self.declare_parameter("invert_normalized", False)

        self.declare_parameter("min_resistance_ohm", 50.0)

        self.bus = int(self.get_parameter("bus").value)
        self.device = int(self.get_parameter("device").value)
        self.sample_rate = float(self.get_parameter("sample_rate").value)

        self.vcc = float(self.get_parameter("vcc").value)
        self.r_fixed = float(self.get_parameter("r_fixed").value)
        self.adc_min_calib = float(self.get_parameter("adc_min_calib").value)
        self.adc_max_calib = float(self.get_parameter("adc_max_calib").value)
        self.invert_normalized = bool(self.get_parameter("invert_normalized").value)
        self.min_resistance_ohm = float(self.get_parameter("min_resistance_ohm").value)

        self.adc_max = 4095.0

        self.pub_raw = self.create_publisher(Int32, "/fsr/raw", 10)
        self.pub_voltage = self.create_publisher(Float32, "/fsr/voltage", 10)
        self.pub_resistance = self.create_publisher(Float32, "/fsr/resistance_ohm", 10)
        self.pub_force_g = self.create_publisher(Float32, "/fsr/force_g", 10)
        self.pub_force_n = self.create_publisher(Float32, "/fsr/force_n", 10)
        self.pub_normalized = self.create_publisher(Float32, "/fsr/normalized", 10)

        if spidev is None:
            raise RuntimeError(
                "python3-spidev is not installed. Run: sudo apt install python3-spidev"
            )

        self.spi = spidev.SpiDev()
        self.spi.open(self.bus, self.device)
        self.spi.max_speed_hz = 1000000
        self.spi.mode = 0

        period = 1.0 / self.sample_rate
        self.timer = self.create_timer(period, self.loop)

        self.get_logger().info("FSR MCP3202 publisher started")
        self.get_logger().info("Publishing /fsr/raw /fsr/voltage /fsr/resistance_ohm /fsr/force_g /fsr/force_n /fsr/normalized")

    def read_mcp3202_ch0(self):
        response = self.spi.xfer2([0b01101000, 0x00])
        adc = ((response[0] & 0x0F) << 8) | response[1]
        return adc

    @staticmethod
    def clamp(value, low, high):
        return max(low, min(high, value))

    def compute_normalized(self, adc):
        denominator = self.adc_max_calib - self.adc_min_calib

        if abs(denominator) < 1e-6:
            return 0.0

        normalized = (adc - self.adc_min_calib) / denominator
        normalized = self.clamp(normalized, 0.0, 1.0)

        if self.invert_normalized:
            normalized = 1.0 - normalized

        return normalized

    def compute_force(self, adc):
        if adc <= 0:
            return None, None, None

        if adc >= self.adc_max:
            return None, None, None

        resistance_ohm = self.r_fixed * (adc / (self.adc_max - adc))

        if resistance_ohm < self.min_resistance_ohm:
            return resistance_ohm, None, None

        resistance_kohm = resistance_ohm / 1000.0

        force_g = math.pow(336.04 / resistance_kohm, 1.0 / 0.712)
        force_n = force_g * 0.00981

        return resistance_ohm, force_g, force_n

    def loop(self):
        adc = self.read_mcp3202_ch0()

        voltage = adc * self.vcc / self.adc_max
        normalized = self.compute_normalized(adc)
        resistance_ohm, force_g, force_n = self.compute_force(adc)

        self.pub_raw.publish(Int32(data=int(adc)))
        self.pub_voltage.publish(Float32(data=float(voltage)))
        self.pub_normalized.publish(Float32(data=float(normalized)))

        if resistance_ohm is not None:
            self.pub_resistance.publish(Float32(data=float(resistance_ohm)))

        if force_g is not None:
            self.pub_force_g.publish(Float32(data=float(force_g)))

        if force_n is not None:
            self.pub_force_n.publish(Float32(data=float(force_n)))

    def destroy_node(self):
        try:
            self.spi.close()
        except Exception:
            pass

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = FsrMcp3202Publisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
