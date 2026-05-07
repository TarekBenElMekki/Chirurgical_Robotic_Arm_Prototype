#!/usr/bin/env python3

import math
import sys

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32, Int32

from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QPainter, QPen, QFont, QColor
from PyQt5.QtWidgets import QApplication, QWidget


class GaugeWidget(QWidget):
    def __init__(self):
        super().__init__()

        self.normalized = 0.0
        self.force_n = 0.0
        self.raw_adc = 0

        self.setMinimumSize(520, 420)

    def set_values(self, normalized, force_n, raw_adc):
        self.normalized = max(0.0, min(1.0, normalized))
        self.force_n = force_n
        self.raw_adc = raw_adc
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        w = self.width()
        h = self.height()

        cx = w // 2
        cy = int(h * 0.72)
        radius = min(w, h) * 0.38

        start_angle_deg = 210
        span_angle_deg = -240

        painter.fillRect(self.rect(), QColor(18, 18, 22))

        painter.setPen(QColor(240, 240, 240))
        painter.setFont(QFont("Arial", 24, QFont.Bold))
        painter.drawText(0, 25, w, 45, Qt.AlignCenter, "FSR FORCE DIAL")

        # Background arc
        pen = QPen(QColor(70, 70, 80), 28)
        pen.setCapStyle(Qt.RoundCap)
        painter.setPen(pen)

        painter.drawArc(
            int(cx - radius),
            int(cy - radius),
            int(radius * 2),
            int(radius * 2),
            start_angle_deg * 16,
            span_angle_deg * 16,
        )

        # Value color zones
        if self.normalized < 0.35:
            color = QColor(0, 210, 100)
            label = "LOW"
        elif self.normalized < 0.70:
            color = QColor(255, 180, 0)
            label = "MEDIUM"
        else:
            color = QColor(240, 50, 50)
            label = "HIGH"

        # Value arc
        pen = QPen(color, 28)
        pen.setCapStyle(Qt.RoundCap)
        painter.setPen(pen)

        value_span = span_angle_deg * self.normalized

        painter.drawArc(
            int(cx - radius),
            int(cy - radius),
            int(radius * 2),
            int(radius * 2),
            start_angle_deg * 16,
            int(value_span * 16),
        )

        # Needle
        angle_deg = start_angle_deg + span_angle_deg * self.normalized
        angle_rad = math.radians(angle_deg)

        needle_len = radius * 0.78
        x2 = cx + needle_len * math.cos(angle_rad)
        y2 = cy - needle_len * math.sin(angle_rad)

        painter.setPen(QPen(QColor(245, 245, 245), 5))
        painter.drawLine(cx, cy, int(x2), int(y2))

        painter.setBrush(QColor(245, 245, 245))
        painter.drawEllipse(cx - 10, cy - 10, 20, 20)

        # Percentage
        painter.setFont(QFont("Arial", 46, QFont.Bold))
        painter.setPen(QColor(245, 245, 245))
        percent = int(self.normalized * 100)
        painter.drawText(0, int(h * 0.37), w, 70, Qt.AlignCenter, f"{percent}%")

        # Pressure label
        painter.setFont(QFont("Arial", 20, QFont.Bold))
        painter.setPen(color)
        painter.drawText(0, int(h * 0.54), w, 35, Qt.AlignCenter, label)

        # Details
        painter.setFont(QFont("Arial", 14))
        painter.setPen(QColor(210, 210, 210))
        painter.drawText(
            0,
            int(h * 0.86),
            w,
            30,
            Qt.AlignCenter,
            f"Force: {self.force_n:.3f} N     Raw ADC: {self.raw_adc}",
        )


class FsrGaugeNode(Node):
    def __init__(self, widget):
        super().__init__("fsr_gauge_visualizer")

        self.widget = widget

        self.normalized = 0.0
        self.force_n = 0.0
        self.raw_adc = 0

        self.create_subscription(Float32, "/fsr/normalized", self.normalized_callback, 10)
        self.create_subscription(Float32, "/fsr/force_n", self.force_callback, 10)
        self.create_subscription(Int32, "/fsr/raw", self.raw_callback, 10)

        self.get_logger().info("FSR gauge visualizer started")
        self.get_logger().info("Subscribing to /fsr/normalized /fsr/force_n /fsr/raw")

    def update_widget(self):
        self.widget.set_values(self.normalized, self.force_n, self.raw_adc)

    def normalized_callback(self, msg):
        self.normalized = msg.data
        self.update_widget()

    def force_callback(self, msg):
        self.force_n = msg.data
        self.update_widget()

    def raw_callback(self, msg):
        self.raw_adc = msg.data
        self.update_widget()


def main(args=None):
    rclpy.init(args=args)

    app = QApplication(sys.argv)

    widget = GaugeWidget()
    widget.setWindowTitle("FSR Force Gauge")
    widget.show()

    node = FsrGaugeNode(widget)

    timer = QTimer()
    timer.timeout.connect(lambda: rclpy.spin_once(node, timeout_sec=0.001))
    timer.start(10)

    exit_code = app.exec_()

    node.destroy_node()
    rclpy.shutdown()

    sys.exit(exit_code)


if __name__ == "__main__":
    main()
