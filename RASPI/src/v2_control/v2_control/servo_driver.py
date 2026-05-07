#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

try:
    import lgpio
except ImportError:
    lgpio = None


class ServoDriver(Node):
    def __init__(self):
        super().__init__("servo_driver")

        self.declare_parameter("gpiochip", 0)
        self.declare_parameter("gpio_pin", 18)

        self.declare_parameter("min_angle_deg", 0.0)
        self.declare_parameter("max_angle_deg", 180.0)

        self.declare_parameter("min_pulse_us", 1000.0)
        self.declare_parameter("max_pulse_us", 2000.0)
        self.declare_parameter("pwm_frequency_hz", 50.0)

        self.declare_parameter("command_topic", "/motor/command_angle_deg")
        self.declare_parameter("real_angle_topic", "/motor/real_angle_deg")
        self.declare_parameter("error_topic", "/motor/error_deg")

        self.declare_parameter("min_command_delta_deg", 0.5)
        self.declare_parameter("hold_pwm", True)
        self.declare_parameter("detach_on_shutdown", False)
        self.declare_parameter("release_after_command_sec", 0.6)

        self.gpiochip = int(self.get_parameter("gpiochip").value)
        self.gpio_pin = int(self.get_parameter("gpio_pin").value)

        self.min_angle = float(self.get_parameter("min_angle_deg").value)
        self.max_angle = float(self.get_parameter("max_angle_deg").value)

        self.min_pulse_us = float(self.get_parameter("min_pulse_us").value)
        self.max_pulse_us = float(self.get_parameter("max_pulse_us").value)
        self.pwm_frequency_hz = float(self.get_parameter("pwm_frequency_hz").value)

        self.command_topic = self.get_parameter("command_topic").value
        self.real_angle_topic = self.get_parameter("real_angle_topic").value
        self.error_topic = self.get_parameter("error_topic").value

        self.min_command_delta_deg = float(self.get_parameter("min_command_delta_deg").value)
        self.hold_pwm = bool(self.get_parameter("hold_pwm").value)
        self.detach_on_shutdown = bool(self.get_parameter("detach_on_shutdown").value)
        self.release_after_command_sec = float(self.get_parameter("release_after_command_sec").value)

        self.release_timer = None
        self.last_command_angle = None
        self.last_estimated_real_angle = None

        if lgpio is None:
            raise RuntimeError(
                "Python module lgpio not found. On Raspberry Pi run: sudo apt install python3-lgpio"
            )

        self.gpio_handle = lgpio.gpiochip_open(self.gpiochip)

        self.real_pub = self.create_publisher(Float32, self.real_angle_topic, 10)
        self.error_pub = self.create_publisher(Float32, self.error_topic, 10)

        self.create_subscription(
            Float32,
            self.command_topic,
            self.command_callback,
            10,
        )

        self.get_logger().info("SG90 servo driver started")
        self.get_logger().info(f"GPIO chip: /dev/gpiochip{self.gpiochip}")
        self.get_logger().info(f"GPIO pin BCM: {self.gpio_pin}")
        self.get_logger().info(f"Command topic: {self.command_topic}")
        self.get_logger().info(f"Pulse range: {self.min_pulse_us:.0f}-{self.max_pulse_us:.0f} us")
        self.get_logger().info(f"PWM frequency: {self.pwm_frequency_hz:.1f} Hz")
        self.get_logger().info(f"Hold PWM: {self.hold_pwm}")
        self.get_logger().info(f"Release after command: {self.release_after_command_sec:.2f} s")

    def clamp_angle(self, angle):
        return max(self.min_angle, min(self.max_angle, float(angle)))

    def angle_to_pulse_us(self, angle):
        angle = self.clamp_angle(angle)

        if abs(self.max_angle - self.min_angle) < 1e-9:
            return (self.min_pulse_us + self.max_pulse_us) / 2.0

        ratio = (angle - self.min_angle) / (self.max_angle - self.min_angle)
        pulse = self.min_pulse_us + ratio * (self.max_pulse_us - self.min_pulse_us)
        return max(self.min_pulse_us, min(self.max_pulse_us, pulse))

    def pulse_to_duty_cycle_percent(self, pulse_us):
        period_us = 1_000_000.0 / self.pwm_frequency_hz
        return 100.0 * pulse_us / period_us

    def write_servo(self, angle):
        safe_angle = self.clamp_angle(angle)
        pulse_us = int(round(self.angle_to_pulse_us(safe_angle)))

        if hasattr(lgpio, "tx_servo"):
            lgpio.tx_servo(
                self.gpio_handle,
                self.gpio_pin,
                pulse_us,
                int(self.pwm_frequency_hz),
                0,
                0,
            )
        else:
            duty = self.pulse_to_duty_cycle_percent(pulse_us)
            lgpio.tx_pwm(
                self.gpio_handle,
                self.gpio_pin,
                self.pwm_frequency_hz,
                duty,
            )

        self.last_estimated_real_angle = safe_angle

        self.real_pub.publish(Float32(data=float(safe_angle)))

        if self.last_command_angle is None:
            error = 0.0
        else:
            error = float(self.last_command_angle - safe_angle)

        self.error_pub.publish(Float32(data=error))

        self.get_logger().info(
            f"SG90 write: {safe_angle:.1f} deg -> {pulse_us} us"
        )

    def detach_servo(self):
        try:
            if hasattr(lgpio, "tx_servo"):
                lgpio.tx_servo(
                    self.gpio_handle,
                    self.gpio_pin,
                    0,
                    int(self.pwm_frequency_hz),
                    0,
                    0,
                )
            else:
                lgpio.tx_pwm(
                    self.gpio_handle,
                    self.gpio_pin,
                    self.pwm_frequency_hz,
                    0,
                )

            self.get_logger().info("Servo PWM detached")
        except Exception as exc:
            self.get_logger().warn(f"Servo detach failed: {exc}")

    def schedule_detach(self):
        if self.release_timer is not None:
            self.release_timer.cancel()

        def detach_once():
            self.detach_servo()
            if self.release_timer is not None:
                self.release_timer.cancel()
                self.release_timer = None

        self.release_timer = self.create_timer(
            max(0.05, self.release_after_command_sec),
            detach_once,
        )

    def command_callback(self, msg):
        angle = self.clamp_angle(msg.data)

        if self.last_command_angle is not None:
            if abs(angle - self.last_command_angle) < self.min_command_delta_deg:
                return

        self.last_command_angle = angle
        self.write_servo(angle)

        if not self.hold_pwm:
            self.schedule_detach()

    def destroy_node(self):
        try:
            if hasattr(self, "gpio_handle"):
                if self.detach_on_shutdown:
                    self.detach_servo()
                lgpio.gpiochip_close(self.gpio_handle)
        except Exception as exc:
            print(f"[servo_driver] cleanup warning: {exc}")

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        node = ServoDriver()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        print(f"[servo_driver] ERROR: {exc}")
    finally:
        if node is not None:
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
