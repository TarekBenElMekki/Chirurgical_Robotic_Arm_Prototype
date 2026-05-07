#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

try:
    import lgpio
except ImportError:
    lgpio = None


class Sg90ServoNode(Node):
    def __init__(self):
        super().__init__("sg90_servo_node")

        self.declare_parameter("gpio_pin", 18)
        self.declare_parameter("gpiochip", 0)

        self.declare_parameter("min_angle_deg", 0.0)
        self.declare_parameter("max_angle_deg", 180.0)
        self.declare_parameter("home_angle_deg", 90.0)

        self.declare_parameter("min_pulse_us", 1000.0)
        self.declare_parameter("max_pulse_us", 2000.0)
        self.declare_parameter("pwm_frequency_hz", 50.0)

        # Modes:
        # hold              -> keep PWM active, servo holds position
        # move_then_detach  -> move to angle, wait, then stop PWM
        self.declare_parameter("control_mode", "move_then_detach")
        self.declare_parameter("min_move_time_sec", 0.35)
        self.declare_parameter("max_move_time_sec", 1.60)
        self.declare_parameter("seconds_per_degree", 0.008)
        self.declare_parameter("extra_settle_time_sec", 0.20)

        self.declare_parameter("command_topic", "/sg90/angle_deg")
        self.declare_parameter("state_topic", "/sg90/state_deg")
        self.declare_parameter("publish_rate_hz", 10.0)

        self.declare_parameter("detach_on_shutdown", True)

        self.gpio_pin = int(self.get_parameter("gpio_pin").value)
        self.gpiochip = int(self.get_parameter("gpiochip").value)

        self.min_angle = float(self.get_parameter("min_angle_deg").value)
        self.max_angle = float(self.get_parameter("max_angle_deg").value)
        self.home_angle = float(self.get_parameter("home_angle_deg").value)

        self.min_pulse_us = float(self.get_parameter("min_pulse_us").value)
        self.max_pulse_us = float(self.get_parameter("max_pulse_us").value)
        self.pwm_frequency_hz = float(self.get_parameter("pwm_frequency_hz").value)

        self.control_mode = str(self.get_parameter("control_mode").value)
        self.min_move_time_sec = float(self.get_parameter("min_move_time_sec").value)
        self.max_move_time_sec = float(self.get_parameter("max_move_time_sec").value)
        self.seconds_per_degree = float(self.get_parameter("seconds_per_degree").value)
        self.extra_settle_time_sec = float(self.get_parameter("extra_settle_time_sec").value)

        self.command_topic = self.get_parameter("command_topic").value
        self.state_topic = self.get_parameter("state_topic").value
        self.detach_on_shutdown = bool(self.get_parameter("detach_on_shutdown").value)

        if lgpio is None:
            raise RuntimeError(
                "lgpio Python module not found. Install it with: sudo apt install python3-lgpio"
            )

        self.gpio_handle = lgpio.gpiochip_open(self.gpiochip)

        self.current_angle = self.clamp_angle(self.home_angle)
        self.detached = False
        self.detach_timer = None

        self.state_pub = self.create_publisher(Float32, self.state_topic, 10)

        self.create_subscription(
            Float32,
            self.command_topic,
            self.angle_callback,
            10,
        )

        publish_rate = float(self.get_parameter("publish_rate_hz").value)
        timer_period = 1.0 / max(publish_rate, 1.0)
        self.state_timer = self.create_timer(timer_period, self.publish_state)

        self.write_angle(self.current_angle)

        self.get_logger().info("SG90 servo node started with LGPIO")
        self.get_logger().info(f"GPIO chip: /dev/gpiochip{self.gpiochip}")
        self.get_logger().info(f"GPIO pin: BCM {self.gpio_pin}")
        self.get_logger().info(f"Command topic: {self.command_topic}")
        self.get_logger().info(f"State topic: {self.state_topic}")
        self.get_logger().info(f"Control mode: {self.control_mode}")
        self.get_logger().info(
            f"Angle range: {self.min_angle:.1f}° to {self.max_angle:.1f}°"
        )
        self.get_logger().info(
            f"Pulse range: {self.min_pulse_us:.0f} us to {self.max_pulse_us:.0f} us"
        )
        self.get_logger().info(f"PWM frequency: {self.pwm_frequency_hz:.1f} Hz")
        self.get_logger().info(f"Home angle: {self.current_angle:.1f}°")

    def clamp_angle(self, angle):
        return max(self.min_angle, min(self.max_angle, float(angle)))

    def angle_to_pulse_us(self, angle):
        angle = self.clamp_angle(angle)

        if self.max_angle == self.min_angle:
            return (self.min_pulse_us + self.max_pulse_us) / 2.0

        ratio = (angle - self.min_angle) / (self.max_angle - self.min_angle)
        pulse_us = self.min_pulse_us + ratio * (self.max_pulse_us - self.min_pulse_us)

        return max(self.min_pulse_us, min(self.max_pulse_us, pulse_us))

    def pulse_us_to_duty_cycle_percent(self, pulse_us):
        period_us = 1_000_000.0 / self.pwm_frequency_hz
        return 100.0 * pulse_us / period_us

    def send_servo_pulse(self, pulse_us):
        pulse_us = int(round(pulse_us))

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
            duty_cycle = self.pulse_us_to_duty_cycle_percent(pulse_us)
            lgpio.tx_pwm(
                self.gpio_handle,
                self.gpio_pin,
                self.pwm_frequency_hz,
                duty_cycle,
            )

        self.detached = False

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

            self.detached = True
            self.get_logger().info("Servo PWM stopped / detached")
        except Exception as exc:
            self.get_logger().warn(f"Could not detach servo PWM: {exc}")

    def cancel_detach_timer(self):
        if self.detach_timer is not None:
            try:
                self.detach_timer.cancel()
                self.destroy_timer(self.detach_timer)
            except Exception:
                pass

            self.detach_timer = None

    def compute_move_time(self, old_angle, new_angle):
        distance = abs(float(new_angle) - float(old_angle))
        move_time = distance * self.seconds_per_degree + self.extra_settle_time_sec

        move_time = max(self.min_move_time_sec, move_time)
        move_time = min(self.max_move_time_sec, move_time)

        return move_time

    def schedule_detach(self, move_time):
        self.cancel_detach_timer()

        def detach_once():
            self.detach_servo()
            self.cancel_detach_timer()

        self.detach_timer = self.create_timer(move_time, detach_once)

    def write_angle(self, angle):
        old_angle = self.current_angle
        safe_angle = self.clamp_angle(angle)
        pulse_us = self.angle_to_pulse_us(safe_angle)

        self.cancel_detach_timer()
        self.send_servo_pulse(pulse_us)

        self.current_angle = safe_angle

        self.get_logger().info(
            f"SG90 command: {self.current_angle:.1f} deg -> {pulse_us:.0f} us"
        )

        if self.control_mode in ["move_then_detach", "pulse", "detach"]:
            move_time = self.compute_move_time(old_angle, safe_angle)
            self.get_logger().info(
                f"Move-then-detach enabled. PWM will stop after {move_time:.2f} s"
            )
            self.schedule_detach(move_time)

    def angle_callback(self, msg):
        self.write_angle(msg.data)

    def publish_state(self):
        msg = Float32()
        msg.data = float(self.current_angle)
        self.state_pub.publish(msg)

    def destroy_node(self):
        self.cancel_detach_timer()

        try:
            if hasattr(self, "gpio_handle"):
                if self.detach_on_shutdown:
                    self.detach_servo()

                lgpio.gpiochip_close(self.gpio_handle)
        except Exception as exc:
            print(f"[sg90_servo_node] GPIO cleanup warning: {exc}")

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = None

    try:
        node = Sg90ServoNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        print(f"[sg90_servo_node] ERROR: {exc}")
    finally:
        if node is not None:
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
