import sys
from typing import Set

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from std_msgs.msg import Empty, String
from geometry_msgs.msg import Twist

try:
    from PyQt5.QtCore import Qt, QTimer
    from PyQt5.QtGui import QFont
    from PyQt5.QtWidgets import QApplication, QWidget, QPushButton, QLabel, QHBoxLayout, QVBoxLayout
except Exception as exc:
    print("PyQt5 is required for mode_selector_dock_node.")
    print("Install in WSL with: sudo apt install -y python3-pyqt5")
    raise exc


MODES = ["IDLE", "MANUAL_SIM", "MOVEIT_PLAN", "ESTOP", "MANUAL_REAL"]


class ModeRosNode(Node):
    def __init__(self):
        super().__init__("mode_selector_dock_node")

        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.current_mode = "IDLE"
        self.joystick_status = "Joystick: SEARCHING"

        self.mode_pub = self.create_publisher(String, "/system/mode", qos)
        self.keyboard_pub = self.create_publisher(Twist, "/manual/keyboard_cmd", 10)
        self.joystick_refresh_pub = self.create_publisher(Empty, "/manual/joystick_refresh", 10)

        self.create_subscription(String, "/manual/joystick_status", self.on_joystick_status, 10)
        self.create_subscription(String, "/system/status", self.on_system_status, 10)

        self.last_system_status = ""
        self.publish_mode("IDLE")

    def safe_publish(self, pub, msg):
        if not rclpy.ok():
            return
        try:
            pub.publish(msg)
        except Exception as exc:
            self.get_logger().warn(f"Safe publish ignored during shutdown: {exc}")

    def publish_mode(self, mode: str):
        self.current_mode = mode
        msg = String()
        msg.data = mode
        self.safe_publish(self.mode_pub, msg)
        self.get_logger().info(f"Mode set to {mode}")

    def publish_keyboard_cmd(self, cmd: Twist):
        self.safe_publish(self.keyboard_pub, cmd)

    def request_joystick_refresh(self):
        self.safe_publish(self.joystick_refresh_pub, Empty())
        self.get_logger().info("Joystick refresh/search requested.")

    def on_joystick_status(self, msg: String):
        self.joystick_status = msg.data

    def on_system_status(self, msg: String):
        self.last_system_status = msg.data


class ModeDock(QWidget):
    def __init__(self, ros_node: ModeRosNode):
        super().__init__()
        self.ros_node = ros_node
        self.pressed: Set[int] = set()
        self.buttons = {}

        self.setWindowTitle("Surgical Arm Mode Selector - V0.4.2 KEYBOARD FIRST")
        self.setWindowFlags(Qt.WindowStaysOnTopHint | Qt.Window)
        self.setFocusPolicy(Qt.StrongFocus)

        root = QVBoxLayout()
        root.setContentsMargins(12, 12, 12, 12)

        title = QLabel("Surgical Arm Mode Selector")
        title.setAlignment(Qt.AlignCenter)
        title.setFont(QFont("Arial", 13, QFont.Bold))
        root.addWidget(title)

        row = QHBoxLayout()
        row.setSpacing(10)

        for mode in MODES:
            btn = QPushButton(mode.replace("_", " "))
            btn.setMinimumHeight(48)
            btn.setMinimumWidth(120)
            btn.setFocusPolicy(Qt.NoFocus)
            btn.clicked.connect(lambda checked=False, m=mode: self.set_mode(m))
            self.buttons[mode] = btn
            row.addWidget(btn)

        self.buttons["MANUAL_REAL"].setEnabled(False)
        self.buttons["MANUAL_REAL"].setToolTip("Prepared for Raspberry Pi deployment later.")

        root.addLayout(row)

        status_row = QHBoxLayout()
        self.mode_status = QLabel("Mode: IDLE | Keyboard: READY")
        self.mode_status.setAlignment(Qt.AlignLeft)
        self.mode_status.setFont(QFont("Arial", 10))
        status_row.addWidget(self.mode_status)

        self.joy_status = QLabel("Joystick: SEARCHING")
        self.joy_status.setAlignment(Qt.AlignCenter)
        self.joy_status.setFont(QFont("Arial", 10))
        status_row.addWidget(self.joy_status)

        self.refresh_button = QPushButton("↻ Refresh Joy")
        self.refresh_button.setMinimumHeight(32)
        self.refresh_button.clicked.connect(self.refresh_joystick)
        status_row.addWidget(self.refresh_button)

        root.addLayout(status_row)

        self.system_status = QLabel("System: Publish Point is allowed only in MOVEIT PLAN mode.")
        self.system_status.setAlignment(Qt.AlignCenter)
        self.system_status.setFont(QFont("Arial", 9))
        root.addWidget(self.system_status)

        self.setLayout(root)

        self.setStyleSheet("""
            QWidget {
                background-color: #20242c;
                color: white;
            }
            QLabel {
                color: white;
            }
            QPushButton {
                color: white;
                background-color: #4a5263;
                border: 1px solid #8b93a6;
                border-radius: 10px;
                padding: 8px 14px;
                font-weight: bold;
                font-size: 12px;
            }
            QPushButton:hover {
                background-color: #626d86;
            }
            QPushButton:disabled {
                color: #888888;
                background-color: #444444;
            }
        """)

        self.resize(960, 180)
        self.move(180, 730)
        self.highlight_mode("IDLE")

        self.ros_timer = QTimer()
        self.ros_timer.timeout.connect(self.spin_ros_once)
        self.ros_timer.start(20)

        self.keyboard_timer = QTimer()
        self.keyboard_timer.timeout.connect(self.publish_keyboard)
        self.keyboard_timer.start(50)

        self.ui_timer = QTimer()
        self.ui_timer.timeout.connect(self.update_status_labels)
        self.ui_timer.start(250)

        print("R19A_MODE_DOCK_KEYBOARD_FIRST_CREATED", flush=True)

    def spin_ros_once(self):
        if not rclpy.ok():
            return
        try:
            rclpy.spin_once(self.ros_node, timeout_sec=0.0)
        except Exception as exc:
            print(f"R19A_SPIN_IGNORED_DURING_SHUTDOWN: {exc}", flush=True)

    def refresh_joystick(self):
        self.ros_node.request_joystick_refresh()
        self.joy_status.setText("Joystick: SEARCHING...")
        self.raise_()
        self.activateWindow()
        self.setFocus()

    def update_status_labels(self):
        self.joy_status.setText(self.ros_node.joystick_status)

        mode = self.ros_node.current_mode
        pressed_keys = []
        for name, key in [("W", Qt.Key_W), ("S", Qt.Key_S), ("A", Qt.Key_A), ("D", Qt.Key_D), ("Q", Qt.Key_Q), ("E", Qt.Key_E)]:
            if key in self.pressed:
                pressed_keys.append(name)

        keyboard_state = "READY" if not pressed_keys else "ACTIVE " + "+".join(pressed_keys)
        self.mode_status.setText(f"Mode: {mode} | Keyboard: {keyboard_state}")

        if self.ros_node.last_system_status:
            self.system_status.setText("System: " + self.ros_node.last_system_status)

    def set_mode(self, mode: str):
        if mode == "MANUAL_REAL":
            return

        self.ros_node.publish_mode(mode)
        self.highlight_mode(mode)
        self.raise_()
        self.activateWindow()
        self.setFocus()

    def highlight_mode(self, mode: str):
        for name, btn in self.buttons.items():
            if name == mode:
                if name == "ESTOP":
                    btn.setStyleSheet("background-color: #b71c1c; color: white;")
                elif name == "MANUAL_SIM":
                    btn.setStyleSheet("background-color: #118a4f; color: white;")
                elif name == "MOVEIT_PLAN":
                    btn.setStyleSheet("background-color: #5c4fc4; color: white;")
                else:
                    btn.setStyleSheet("background-color: #1e6fa8; color: white;")
            else:
                btn.setStyleSheet("")

    def keyPressEvent(self, event):
        key = event.key()

        if key == Qt.Key_Escape:
            self.set_mode("ESTOP")
            return

        if key == Qt.Key_Space:
            self.pressed.clear()
            self.publish_keyboard()
            return

        self.pressed.add(key)

    def keyReleaseEvent(self, event):
        self.pressed.discard(event.key())

    def publish_keyboard(self):
        cmd = Twist()

        if self.ros_node.current_mode != "MANUAL_SIM":
            self.ros_node.publish_keyboard_cmd(cmd)
            return

        speed = 1.0

        if Qt.Key_W in self.pressed:
            cmd.linear.z += speed
        if Qt.Key_S in self.pressed:
            cmd.linear.z -= speed
        if Qt.Key_D in self.pressed:
            cmd.linear.y += speed
        if Qt.Key_A in self.pressed:
            cmd.linear.y -= speed
        if Qt.Key_E in self.pressed:
            cmd.angular.y += speed
        if Qt.Key_Q in self.pressed:
            cmd.angular.y -= speed

        self.ros_node.publish_keyboard_cmd(cmd)

    def closeEvent(self, event):
        try:
            self.ros_timer.stop()
            self.keyboard_timer.stop()
            self.ui_timer.stop()
        except Exception:
            pass
        event.accept()


def main(args=None):
    rclpy.init(args=args)
    ros_node = ModeRosNode()

    app = QApplication(sys.argv)
    dock = ModeDock(ros_node)
    dock.show()
    dock.raise_()
    dock.activateWindow()
    dock.setFocus()

    print("R19A_MODE_DOCK_KEYBOARD_FIRST_SHOWN", flush=True)

    try:
        exit_code = app.exec_()
    finally:
        try:
            ros_node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()

    sys.exit(exit_code)


if __name__ == "__main__":
    main()
