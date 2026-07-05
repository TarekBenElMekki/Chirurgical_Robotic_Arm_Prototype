#!/usr/bin/env python3
import time
import sys
import termios
import tty
import argparse
import lgpio
import csv
from datetime import datetime


def clamp(value, vmin, vmax):
    return max(vmin, min(vmax, value))


def pulse_to_angle(pulse_us, min_us, max_us):
    return (pulse_us - min_us) * 180.0 / (max_us - min_us)


def read_key():
    ch = sys.stdin.read(1)

    if ch == "\x03":  # CTRL+C
        raise KeyboardInterrupt

    if ch == "\x1b":
        seq = ch + sys.stdin.read(2)

        if seq == "\x1b[A":
            return "UP"
        if seq == "\x1b[B":
            return "DOWN"
        if seq == "\x1b[C":
            return "RIGHT"
        if seq == "\x1b[D":
            return "LEFT"

        return "ESC"

    return ch.lower()


class ServoMotor:
    def __init__(
        self,
        chip,
        name,
        gpio,
        min_us,
        center_us,
        max_us,
        start_us,
        freq_hz,
        release_after_move=True,
    ):
        self.chip = chip
        self.name = name
        self.gpio = gpio

        self.min_us = min_us
        self.center_us = center_us
        self.max_us = max_us
        self.freq_hz = freq_hz

        self.current_us = int(clamp(start_us, min_us, max_us))
        self.target_us = self.current_us

        self.release_after_move = release_after_move

        lgpio.gpio_claim_output(self.chip, self.gpio)

    def send_once(self, target_us, sec_per_600us, extra_settle_s):
        target_us = int(clamp(target_us, self.min_us, self.max_us))
        old_us = self.current_us
        delta_us = abs(target_us - old_us)

        if delta_us == 0:
            return 0.0, old_us, target_us

        self.target_us = target_us

        lgpio.tx_servo(
            self.chip,
            self.gpio,
            target_us,
            self.freq_hz,
            0,
            0
        )

        wait_s = (delta_us * sec_per_600us / 600.0) + extra_settle_s
        time.sleep(wait_s)

        if self.release_after_move:
            lgpio.tx_servo(self.chip, self.gpio, 0)

        self.current_us = target_us

        return wait_s, old_us, target_us

    def release(self):
        lgpio.tx_servo(self.chip, self.gpio, 0)

    def angle(self):
        return pulse_to_angle(self.current_us, self.min_us, self.max_us)


def print_help(args):
    print("")
    print("======================================================")
    print("  3-SERVO SINGLE-COMMAND KEYBOARD TEST")
    print("======================================================")
    print("")
    print("Controls:")
    print("  LEFT  / RIGHT  : GPIO17 motor - / + step")
    print("  DOWN  / UP     : GPIO18 motor - / + step")
    print("  A     / D      : GPIO19 motor - / + step")
    print("  C              : slow return all motors to center")
    print("  SPACE          : release PWM signal on all motors")
    print("  P              : print current values")
    print("  Q              : quit without auto-centering")
    print("")
    print(f"Limits: min={args.min_us} us | center={args.center_us} us | max={args.max_us} us")
    print(f"Step: {args.step_us} us")
    print(f"Move wait: {args.sec_per_600us} s per 600 us")
    print(f"Center step: {args.center_step_us} us")
    print(f"Center wait: {args.center_sec_per_600us} s per 600 us")
    print(f"Release after move: {args.release_after_move}")
    print("")
    print("Press keys now.")
    print("======================================================")
    print("")


def print_status(motors):
    print("")
    for m in motors:
        print(
            f"{m.name:8s} | GPIO {m.gpio:2d} | "
            f"pulse={m.current_us:4d} us | "
            f"angle={m.angle():6.1f} deg"
        )
    print("")


def log_action(log_file, key, motor_name, gpio, old_us, new_us, wait_s):
    with open(log_file, "a", newline="") as f:
        writer = csv.writer(f)
        writer.writerow([
            datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3],
            key,
            motor_name,
            gpio,
            old_us,
            new_us,
            wait_s,
        ])


def command_motor(key_name, motor, direction, args):
    requested_target = motor.current_us + direction * args.step_us
    requested_target = int(clamp(requested_target, args.min_us, args.max_us))

    wait_s, old_us, new_us = motor.send_once(
        requested_target,
        args.sec_per_600us,
        args.extra_settle_s
    )

    if old_us == new_us:
        print(
            f"{key_name:5s} | {motor.name} GPIO {motor.gpio} | "
            f"LIMIT REACHED | pulse={new_us} us | angle={motor.angle():.1f} deg"
        )
    else:
        print(
            f"{key_name:5s} | {motor.name} GPIO {motor.gpio} | "
            f"{old_us} -> {new_us} us | "
            f"angle={motor.angle():.1f} deg | wait={wait_s:.3f} s"
        )

    log_action(args.log_file, key_name, motor.name, motor.gpio, old_us, new_us, wait_s)


def slow_center_motor(motor, args):
    print(f"Centering {motor.name} GPIO {motor.gpio} slowly...")

    while motor.current_us != args.center_us:
        if motor.current_us < args.center_us:
            next_us = min(motor.current_us + args.center_step_us, args.center_us)
        else:
            next_us = max(motor.current_us - args.center_step_us, args.center_us)

        wait_s, old_us, new_us = motor.send_once(
            next_us,
            args.center_sec_per_600us,
            args.extra_settle_s
        )

        print(
            f"  {motor.name} | {old_us} -> {new_us} us | "
            f"angle={motor.angle():.1f} deg | wait={wait_s:.3f} s"
        )

        log_action(args.log_file, "CENTER", motor.name, motor.gpio, old_us, new_us, wait_s)


def main(args):
    h = lgpio.gpiochip_open(0)

    motors = []

    with open(args.log_file, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow([
            "time",
            "key",
            "motor",
            "gpio",
            "old_us",
            "new_us",
            "wait_s",
        ])

    try:
        x_motor = ServoMotor(
            h,
            "X motor",
            args.x_gpio,
            args.min_us,
            args.center_us,
            args.max_us,
            args.start_us,
            args.freq_hz,
            args.release_after_move
        )

        y_motor = ServoMotor(
            h,
            "Y motor",
            args.y_gpio,
            args.min_us,
            args.center_us,
            args.max_us,
            args.start_us,
            args.freq_hz,
            args.release_after_move
        )

        z_motor = ServoMotor(
            h,
            "Z motor",
            args.z_gpio,
            args.min_us,
            args.center_us,
            args.max_us,
            args.start_us,
            args.freq_hz,
            args.release_after_move
        )

        motors = [x_motor, y_motor, z_motor]

        if args.send_start_on_launch:
            print("Sending start position to all motors...")
            for m in motors:
                wait_s, old_us, new_us = m.send_once(
                    args.start_us,
                    args.sec_per_600us,
                    args.extra_settle_s
                )
                print(f"{m.name} GPIO {m.gpio}: {old_us} -> {new_us} us")

        print_help(args)
        print_status(motors)

        fd = sys.stdin.fileno()
        old_terminal = termios.tcgetattr(fd)

        try:
            tty.setcbreak(fd)

            while True:
                key = read_key()

                if key == "q":
                    print("Q pressed: quitting without auto-centering.")
                    break

                elif key == "LEFT":
                    command_motor("LEFT", x_motor, -1, args)

                elif key == "RIGHT":
                    command_motor("RIGHT", x_motor, +1, args)

                elif key == "DOWN":
                    command_motor("DOWN", y_motor, -1, args)

                elif key == "UP":
                    command_motor("UP", y_motor, +1, args)

                elif key == "a":
                    command_motor("A", z_motor, -1, args)

                elif key == "d":
                    command_motor("D", z_motor, +1, args)

                elif key == "c":
                    print("C pressed: slow return all motors to center.")
                    for m in motors:
                        slow_center_motor(m, args)
                    print_status(motors)

                elif key == " ":
                    print("SPACE pressed: releasing PWM signal on all motors.")
                    for m in motors:
                        m.release()

                elif key == "p":
                    print_status(motors)

                elif key == "h":
                    print_help(args)

        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_terminal)

    except KeyboardInterrupt:
        print("\nCTRL+C detected: stopping without auto-centering.")

    finally:
        print("Releasing PWM and closing gpiochip.")
        for m in motors:
            try:
                m.release()
            except Exception:
                pass

        try:
            lgpio.gpiochip_close(h)
        except Exception:
            pass

        print(f"Log saved to: {args.log_file}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Single-command keyboard test for 3 servos using lgpio."
    )

    parser.add_argument("--x-gpio", type=int, default=17)
    parser.add_argument("--y-gpio", type=int, default=18)
    parser.add_argument("--z-gpio", type=int, default=19)

    parser.add_argument("--freq-hz", type=int, default=50)

    parser.add_argument("--min-us", type=int, default=1450)
    parser.add_argument("--center-us", type=int, default=1500)
    parser.add_argument("--max-us", type=int, default=1550)

    parser.add_argument("--start-us", type=int, default=1500)

    parser.add_argument("--step-us", type=int, default=5)

    parser.add_argument(
        "--sec-per-600us",
        type=float,
        default=0.17,
        help="Dynamic wait time. Original working logic used 0.17 s per 600 us."
    )

    parser.add_argument(
        "--center-sec-per-600us",
        type=float,
        default=0.60,
        help="Slower wait used during return to center."
    )

    parser.add_argument("--center-step-us", type=int, default=5)
    parser.add_argument("--extra-settle-s", type=float, default=0.05)

    parser.add_argument(
        "--release-after-move",
        action="store_true",
        default=True,
        help="Stop PWM after each movement, like your test_HS_servo.py."
    )

    parser.add_argument(
        "--hold-after-move",
        dest="release_after_move",
        action="store_false",
        help="Keep PWM active after movement. Use only if the servo does not jitter while holding."
    )

    parser.add_argument(
        "--send-start-on-launch",
        action="store_true",
        help="Send start_us to all motors when launching. Disabled by default for safety."
    )

    parser.add_argument(
        "--log-file",
        type=str,
        default="/home/tbem/servo_keyboard_singlecmd_log.csv"
    )

    args = parser.parse_args()

    main(args)
