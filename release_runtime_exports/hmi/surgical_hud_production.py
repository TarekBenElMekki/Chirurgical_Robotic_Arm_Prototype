#!/usr/bin/env python3
import argparse
import json
import math
import os
import re
import subprocess
import sys
import threading
import time
from dataclasses import dataclass, field
from typing import Optional, List, Tuple

try:
    import cv2
except Exception:
    cv2 = None

from PyQt5.QtCore import Qt, QTimer, QRectF, QPointF, QLineF, QSize
from PyQt5.QtGui import (
    QColor,
    QFont,
    QFontDatabase,
    QImage,
    QPainter,
    QPen,
    QPixmap,
    QLinearGradient,
    QRadialGradient,
    QBrush,
    QPolygonF,
    QPainterPath,
)
from PyQt5.QtWidgets import QApplication, QLabel, QMainWindow, QWidget

try:
    from PyQt5.QtSvg import QSvgRenderer
except Exception:
    QSvgRenderer = None

try:
    import qtawesome as qta
except Exception:
    qta = None


@dataclass
class HudState:
    case_id: str = "S-0142"
    action: str = "SUTURING"
    operator_name: str = "TAREK BEN EL MEKKI"
    latency_ms: int = 28
    fps: int = 30
    force_n: float = 2.8
    depth_mm: int = 42
    last_action: str = "JAW CLOSE"
    session_start_time: float = field(default_factory=time.time)
    notifications: List[Tuple[str, str]] = field(default_factory=lambda: [
        ("00:00:00", "Session started"),
    ])



def add_surgery_pv_log(state, text):
    """
    Append one line to the surgery PV/session log.
    Used by ROS callbacks and by the local file log bridge.
    """
    try:
        text = " ".join(str(text).splitlines()).strip()
        if not text:
            return

        start = float(getattr(state, "session_start_time", time.time()))
        elapsed = max(0, int(time.time() - start))
        stamp = f"{elapsed // 3600:02d}:{(elapsed % 3600) // 60:02d}:{elapsed % 60:02d}"

        logs = list(getattr(state, "notifications", []))
        if not logs:
            logs = [("00:00:00", "Session started")]

        logs.append((stamp, text))
        state.notifications = logs[-10:]
        setattr(state, "pv_scroll_offset", 0)
    except Exception:
        pass


class InputOverlayState:
    """
    Runtime input state used only by the HMI overlay.

    Current real hardware mapping:
    - /manual/joystick axes[0] / axes[1]  -> L-stick / D-pad direction icon
    - /manual/joystick buttons[0]         -> green A icon
    - /manual/joystick buttons[1]         -> red B icon

    Icons are output-only and appear only while the real input is active.
    """

    def __init__(self):
        self.lock = threading.Lock()
        self.connected = False
        self.status_text = "INPUT: NO JOYSTICK"
        self.last_seen = 0.0
        self.lx = 0.0
        self.ly = 0.0
        self.deadzone = 0.07
        self.fade_s = 2.50
        self.last_active = {
            "stick": 0.0,
            "up": 0.0,
            "down": 0.0,
            "left": 0.0,
            "right": 0.0,
            "button_a": 0.0,
            "button_b": 0.0,
        }

    def update_status(self, text: str):
        with self.lock:
            self.status_text = text
            if "CONNECTED" in text:
                self.connected = True
                self.last_seen = time.time()
            elif "LOST" in text or "NOT CONNECTED" in text:
                self.connected = False

    def update_joy(self, axes, buttons):
        now = time.time()

        x = float(axes[0]) if len(axes) > 0 else 0.0
        y = float(axes[1]) if len(axes) > 1 else 0.0

        b0 = int(buttons[0]) if len(buttons) > 0 else 0
        b1 = int(buttons[1]) if len(buttons) > 1 else 0

        with self.lock:
            self.connected = True
            self.status_text = "INPUT: JOYSTICK ACTIVE"
            self.last_seen = now
            self.lx = x
            self.ly = y

            if abs(x) > self.deadzone or abs(y) > self.deadzone:
                self.last_active["stick"] = now

                if x > self.deadzone:
                    self.last_active["right"] = now
                if x < -self.deadzone:
                    self.last_active["left"] = now
                if y > self.deadzone:
                    self.last_active["up"] = now
                if y < -self.deadzone:
                    self.last_active["down"] = now

            if b0 == 1:
                self.last_active["button_a"] = now

            if b1 == 1:
                self.last_active["button_b"] = now

    def snapshot(self):
        now = time.time()

        with self.lock:
            if self.connected and now - self.last_seen > 2.0:
                connected = False
                status = "INPUT: JOYSTICK LOST"
            else:
                connected = self.connected
                status = self.status_text

            active = {}
            alpha = {}

            for key, stamp in self.last_active.items():
                dt = now - stamp
                is_active = dt <= self.fade_s
                active[key] = is_active
                alpha[key] = max(0.0, 1.0 - dt / self.fade_s) if is_active else 0.0

            return {
                "connected": connected,
                "status": status,
                "lx": self.lx,
                "ly": self.ly,
                "active": active,
                "alpha": alpha,
            }


class RosInputBridge:
    """
    Optional ROS bridge for the HMI.

    It subscribes to:
    - /manual/joystick
    - /manual/joystick_status

    It never publishes commands, so the HMI remains output-only.
    """

    def __init__(self, input_state: InputOverlayState, enabled: bool, hmi_state=None):
        self.input_state = input_state
        self.hmi_state = hmi_state
        self.enabled = enabled
        self.thread = None
        self.node = None
        self.rclpy = None

        if enabled:
            self.start()

    def start(self):
        self.thread = threading.Thread(target=self._spin, daemon=True)
        self.thread.start()

    def _spin(self):
        try:
            import rclpy
            from rclpy.node import Node
            from sensor_msgs.msg import Joy
            from std_msgs.msg import String
            from std_msgs.msg import Float32
        except Exception as exc:
            self.input_state.update_status(f"INPUT: ROS UNAVAILABLE")
            print(f"HMI ROS input bridge unavailable: {exc}", flush=True)
            return

        self.rclpy = rclpy

        class _InputNode(Node):
            def __init__(self, outer):
                super().__init__("surgical_hmi_input_overlay_bridge")
                self.outer = outer
                self.create_subscription(Joy, "/manual/joystick", self.on_joy, 10)
                self.create_subscription(String, "/manual/joystick_status", self.on_status, 10)
                self.create_subscription(Float32, "/force/newton", self.on_force_newton, 10)
                self.create_subscription(Float32, "/force/limit_n", self.on_force_limit, 10)
                self.create_subscription(Float32, "/link/latency_ms", self.on_link_latency, 10)
                self.create_subscription(Float32, "/raspi/link_latency_ms", self.on_link_latency, 10)
                self.create_subscription(String, "/hmi/log", self.on_system_log, 10)
                self.create_subscription(String, "/surgery/log", self.on_system_log, 10)
                self.create_subscription(String, "/system/log", self.on_system_log, 10)
                self.create_subscription(String, "/camera/log", self.on_system_log, 10)
                self.create_subscription(String, "/force/log", self.on_system_log, 10)
                self.create_subscription(String, "/manual/log", self.on_system_log, 10)
                self.get_logger().info("HMI input overlay bridge subscribed to joystick, force, link and surgery PV log topics")

            def on_joy(self, msg):
                self.outer.input_state.update_joy(list(msg.axes), list(msg.buttons))

            def on_status(self, msg):
                self.outer.input_state.update_status(msg.data)

            def on_force_newton(self, msg):
                try:
                    if self.outer.hmi_state is not None:
                        self.outer.hmi_state.force_n = max(0.0, float(msg.data))
                        self.outer.hmi_state.force_online = True
                        self.outer.hmi_state.force_last_seen = time.time()
                except Exception:
                    pass

            def on_force_limit(self, msg):
                try:
                    value = float(msg.data)
                    if self.outer.hmi_state is not None and value > 0.0:
                        self.outer.hmi_state.force_limit_n = value
                except Exception:
                    pass

            def on_link_latency(self, msg):
                try:
                    value = float(msg.data)
                    if self.outer.hmi_state is not None and value >= 0.0:
                        self.outer.hmi_state.latency_ms = int(round(value))
                        self.outer.hmi_state.link_online = True
                        self.outer.hmi_state.link_last_seen = time.time()
                except Exception:
                    pass

            def on_system_log(self, msg):
                try:
                    if self.outer.hmi_state is not None:
                        add_surgery_pv_log(self.outer.hmi_state, msg.data)
                except Exception:
                    pass

        try:
            rclpy.init(args=None)
            self.node = _InputNode(self)
            rclpy.spin(self.node)
        except Exception as exc:
            self.input_state.update_status("INPUT: ROS BRIDGE ERROR")
            print(f"HMI ROS input bridge error: {exc}", flush=True)
        finally:
            try:
                if self.node is not None:
                    self.node.destroy_node()
            except Exception:
                pass
            try:
                if rclpy.ok():
                    rclpy.shutdown()
            except Exception:
                pass

class VideoLayer(QLabel):
    """
    Production video layer.

    Normal production behavior:
    - read Raspberry Pi camera stream with OpenCV;
    - draw the live frame first;
    - if no stream/camera is available, draw a high-quality grey feed-lost slate;
    - HUD overlay is painted above this layer by HudOverlay.
    """

    def __init__(self, camera_index: Optional[int], background_path: Optional[str], stream_url: Optional[str] = None, parent=None):
        super().__init__(parent)
        self.setAlignment(Qt.AlignCenter)
        self.setStyleSheet("background-color: black;")

        self.camera_index = camera_index
        self.stream_url = stream_url
        self.background_path = background_path  # Legacy/debug only. Not used for production fallback.
        self.cap = None

        self.frame_counter = 0
        self.last_fps_time = time.time()
        self.measured_fps = 0

        self.last_reconnect_attempt = 0.0
        self.reconnect_period_s = 2.0
        self.feed_ok = False
        self.feed_status = "WAITING FOR RASPBERRY PI CAMERA STREAM"

        self.open_capture()

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_frame)
        self.timer.start(33)

    def source_label(self) -> str:
        if self.stream_url:
            return self.stream_url
        if self.camera_index is not None:
            return f"LOCAL CAMERA INDEX {self.camera_index}"
        return ""

    def open_capture(self):
        self.feed_ok = False

        if self.cap is not None:
            try:
                self.cap.release()
            except Exception:
                pass
            self.cap = None

        if cv2 is None:
            self.feed_status = "OPENCV NOT AVAILABLE"
            return

        source = None

        if self.stream_url:
            source = self.stream_url
            self.feed_status = "CONNECTING TO RASPBERRY PI STREAM"
        elif self.camera_index is not None:
            source = int(self.camera_index)
            self.feed_status = "CONNECTING TO LOCAL CAMERA"
        else:
            self.feed_status = "NO CAMERA SOURCE CONFIGURED"
            return

        self.cap = cv2.VideoCapture(source)

        try:
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
            self.cap.set(cv2.CAP_PROP_FPS, 30)
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        except Exception:
            pass

    def close(self):
        if self.cap is not None:
            self.cap.release()
            self.cap = None

    def maybe_reconnect(self):
        now = time.time()
        if now - self.last_reconnect_attempt < self.reconnect_period_s:
            return
        self.last_reconnect_attempt = now
        self.open_capture()

    def update_frame(self):
        img = None

        if self.cap is not None and self.cap.isOpened():
            ok, frame = self.cap.read()
            if ok and frame is not None:
                self.feed_ok = True
                self.feed_status = "LIVE"
                frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                h, w, ch = frame.shape
                img = QImage(frame.data, w, h, ch * w, QImage.Format_RGB888).copy()
            else:
                self.feed_ok = False
                self.feed_status = ""
                self.maybe_reconnect()
        else:
            self.feed_ok = False
            self.maybe_reconnect()

        if img is None:
            img = self.make_feed_lost_frame(max(16, self.width()), max(16, self.height()))

        pix = QPixmap.fromImage(img)
        self.setPixmap(pix.scaled(self.size(), Qt.KeepAspectRatioByExpanding, Qt.SmoothTransformation))

        if self.feed_ok:
            self.frame_counter += 1

        now = time.time()
        if now - self.last_fps_time >= 1.0:
            self.measured_fps = self.frame_counter if self.feed_ok else 0
            self.frame_counter = 0
            self.last_fps_time = now


    def make_feed_lost_frame(self, w: int, h: int) -> QImage:
        """
        Offline camera background:
        pure black only.
        No text, no icon, no card, no dimming.
        """
        img = QImage(max(16, int(w)), max(16, int(h)), QImage.Format_ARGB32)
        img.fill(QColor(0, 0, 0, 255))
        return img

class HudOverlay(QWidget):
    def __init__(self, state: HudState, video_layer: VideoLayer, input_state: InputOverlayState, parent=None):
        super().__init__(parent)
        self.state = state
        self.video_layer = video_layer
        self.input_state = input_state
        self.start_time = time.time()
        self.pv_scroll_offset = 0

        # Local file-based PV bridge.
        # Any subsystem can append one line to this file:
        # runtime/surgery_pv.log
        self.pv_file_path = os.path.join(
            os.path.dirname(os.path.abspath(__file__)),
            "runtime",
            "surgery_pv.log",
        )
        try:
            os.makedirs(os.path.dirname(self.pv_file_path), exist_ok=True)
            with open(self.pv_file_path, "w", encoding="utf-8") as f:
                f.write("")
            self.pv_file_offset = 0
        except Exception:
            self.pv_file_offset = 0


        self.font_family = self.pick_old_screen_font()

        self.setAttribute(Qt.WA_TransparentForMouseEvents, True)
        self.setAttribute(Qt.WA_TranslucentBackground, True)
        self.setAutoFillBackground(False)

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.refresh)
        self.timer.start(50)

    def pick_old_screen_font(self):
        available = set(QFontDatabase().families())

        # Preference order: avionics / HUD / Garmin-like / old terminal.
        preferred = [
            "Eurostile Extended",
            "Eurostile",
            "Microgramma",
            "OCR A Extended",
            "OCR A Std",
            "Bank Gothic",
            "BankGothic Lt BT",
            "Agency FB",
            "Bahnschrift SemiCondensed",
            "Bahnschrift",
            "DIN Alternate",
            "DIN 1451",
            "Share Tech Mono",
            "Orbitron",
            "Rajdhani",
            "IBM Plex Mono",
            "Cascadia Mono",
            "Consolas",
            "DejaVu Sans Mono",
            "Liberation Mono",
            "Courier New",
            "Monospace",
        ]

        for name in preferred:
            if name in available:
                return name

        return "DejaVu Sans Mono"


    def refresh(self):
        t = time.time() - self.start_time
        self.state.fps = self.video_layer.measured_fps if self.video_layer.feed_ok else 0
        self.poll_surgery_pv_file()

        # HMI-P2C persistent input icon visual test.
        if getattr(self, "input_test_mode", False):
            phase = int((time.time() - self.start_time) * 2.0) % 4
            if phase == 0:
                self.input_state.update_joy([0.90, 0.00], [0, 0])
            elif phase == 1:
                self.input_state.update_joy([-0.90, 0.00], [0, 0])
            elif phase == 2:
                self.input_state.update_joy([0.00, 0.90], [1, 0])
            else:
                self.input_state.update_joy([0.00, -0.90], [0, 1])

        self.update()

    def font_mono(self, px: int, bold: bool = False) -> QFont:
        f = QFont(self.font_family)
        f.setPixelSize(px)
        f.setBold(bold)
        f.setFixedPitch(True)
        f.setStyleHint(QFont.TypeWriter)
        f.setLetterSpacing(QFont.PercentageSpacing, 122)
        f.setCapitalization(QFont.AllUppercase)

        try:
            f.setStyleStrategy(QFont.PreferQuality)
        except Exception:
            pass

        return f


    def paintEvent(self, event):
        p = QPainter(self)
        try:
            p.setRenderHint(QPainter.Antialiasing)
            p.setRenderHint(QPainter.TextAntialiasing)

            w = self.width()
            h = self.height()

            self.draw_glass_panels(p, w, h)
            self.draw_top_telemetry(p, w, h)
            self.draw_force_scale(p, w, h)
            self.draw_depth_scale(p, w, h)
            if not self.video_layer.feed_ok:
                self.draw_camera_off_modal(p, w, h)
            self.draw_live_input_icons(p, w, h)
            self.draw_bottom_output_bar(p, w, h)
            self.draw_frame(p, w, h)
        finally:
            p.end()

    def draw_glass_panels(self, p, w, h):
        # No full-screen dimming and no dark overlay bars.
        # The HMI elements are drawn directly above the black video layer.
        return

    def draw_frame(self, p, w, h):
        p.setPen(QPen(QColor(210, 225, 225, 115), 1))
        p.drawRect(QRectF(8.5, 8.5, w - 17, h - 17))
        p.setPen(QPen(QColor(210, 225, 225, 82), 1))
        p.drawLine(QLineF(8, 46, w - 8, 46))
        p.drawLine(QLineF(8, h - 88, w - 8, h - 88))

    def draw_old_text(self, p, point, text, size=20, bold=False, alpha=245):
        p.setFont(self.font_mono(size, bold))
        visible_alpha = max(0, min(255, int(alpha)))

        p.setPen(QColor(0, 0, 0, int(visible_alpha * 0.20)))
        p.drawText(QPointF(point.x() + 1.0, point.y() + 1.0), text)

        p.setPen(QColor(245, 248, 248, visible_alpha))
        p.drawText(point, text)

    def draw_library_icon(self, p, icon_names, x, y, size=15, alpha=180):
        """
        Draw gray icon from QtAwesome / Font Awesome library.
        No custom icon geometry.
        """
        if qta is None:
            return False

        for icon_name in icon_names:
            try:
                icon = qta.icon(icon_name, color=QColor(170, 174, 174, alpha))
                pix = icon.pixmap(QSize(size, size))
                p.drawPixmap(int(x), int(y - size + 3), pix)
                return True
            except Exception:
                pass

        return False

    def draw_header_item(self, p, x, y, icon_names, text, icon_size=15, text_size=13, alpha=232):
        drew_icon = self.draw_library_icon(p, icon_names, x, y, icon_size, 178)
        text_x = x + 23 if drew_icon else x
        self.draw_old_text(p, QPointF(text_x, y), text, text_size, True, alpha)

    def draw_header_item_right(self, p, right_x, y, icon_names, text, icon_size=15, text_size=13, alpha=232):
        # Right-aligned header element: useful for putting time at the extreme right.
        p.setFont(self.font_mono(text_size, True))
        try:
            text_w = p.fontMetrics().horizontalAdvance(text)
        except Exception:
            text_w = len(text) * text_size * 0.68

        text_x = right_x - text_w
        icon_x = text_x - 24

        self.draw_library_icon(p, icon_names, icon_x, y, icon_size, 178)
        self.draw_old_text(p, QPointF(text_x, y), text, text_size, True, alpha)

    def draw_top_telemetry(self, p, w, h):
        elapsed = int(time.time() - self.start_time)
        minutes = elapsed // 60
        seconds = elapsed % 60

        y = 34

        link_online = bool(getattr(self.state, "link_online", False)) and (
            time.time() - float(getattr(self.state, "link_last_seen", 0.0)) < 3.0
        )
        link_text = f"LINK {int(getattr(self.state, 'latency_ms', 0)):02d} MS" if link_online else "LINK OFF"

        time_text = f"T 00:{minutes:02d}:{seconds:02d}"

        cam_online = bool(getattr(self.video_layer, "feed_ok", False))
        cam_text = f"{self.state.fps:02d} FPS" if cam_online else "CAM OFFLINE"

        # Left side
        self.draw_header_item(
            p,
            48,
            y,
            ["fa6s.id-card", "fa5s.id-card", "fa.address-card"],
            self.state.case_id,
            icon_size=14,
            text_size=13,
        )

        self.draw_old_text(p, QPointF(214, y), "|", 13, True, 190)

        # Action: icon replaces "ACT:"
        self.draw_header_item(
            p,
            260,
            y,
            ["fa6s.scissors", "fa5s.cut", "fa.scissors", "fa6s.hand", "fa5s.hand-paper"],
            self.state.action,
            icon_size=15,
            text_size=13,
        )

        self.draw_old_text(p, QPointF(510, y), "|", 13, True, 190)

        # Surgeon/operator
        self.draw_header_item(
            p,
            570,
            y,
            ["fa6s.user-doctor", "fa5s.user-md", "fa.user-md", "fa6s.user", "fa5s.user"],
            self.state.operator_name,
            icon_size=15,
            text_size=13,
        )

        # Right side, with TIME at the extreme right.
        self.draw_header_item_right(
            p,
            w - 42,
            y,
            ["fa6s.clock", "fa5s.clock", "fa.clock-o"],
            time_text,
            icon_size=15,
            text_size=13,
        )

        self.draw_header_item_right(
            p,
            w - 305,
            y,
            ["fa6s.video", "fa5s.video", "fa.video-camera"] if cam_online else ["fa6s.video-slash", "fa5s.video-slash", "fa.video-camera"],
            cam_text,
            icon_size=15,
            text_size=13,
        )

        self.draw_header_item_right(
            p,
            w - 565,
            y,
            ["fa6s.satellite-dish", "fa5s.satellite-dish", "fa6s.signal", "fa5s.signal", "fa.wifi"],
            link_text,
            icon_size=15,
            text_size=13,
        )


    def draw_force_scale(self, p, w, h):
        x = 98.0
        y0 = 122.0
        y1 = h - 122.0
        height = y1 - y0

        limit_n = max(0.1, float(getattr(self.state, "force_limit_n", 15.0)))
        force_n = max(0.0, float(getattr(self.state, "force_n", 0.0)))
        force_online = bool(getattr(self.state, "force_online", False)) and (time.time() - float(getattr(self.state, "force_last_seen", 0.0)) < 2.5)

        p.setFont(self.font_mono(18, False))

        if not force_online:
            p.setPen(QColor(190, 190, 190, 120))
            p.drawText(QPointF(50, y0 + 4), "0 N")
            p.drawText(QPointF(32, y1 + 4), f"{limit_n:g} N")

            p.setPen(QPen(QColor(170, 170, 170, 90), 3))
            p.drawLine(QLineF(x, y0, x, y1))

            p.setPen(QPen(QColor(180, 180, 180, 82), 1))
            for i in range(4):
                yy = y0 + height * i / 3.0
                p.drawLine(QLineF(x - 7, yy, x + 7, yy))

            tag_x = x + 34
            tag_y = y0 + 54

            p.setBrush(QColor(0, 0, 0, 48))
            p.setPen(QPen(QColor(180, 180, 180, 85), 1))
            p.drawRoundedRect(QRectF(tag_x, tag_y, 148, 42), 5, 5)

            p.setBrush(QColor(190, 190, 190, 115))
            p.setPen(Qt.NoPen)
            p.drawEllipse(QPointF(tag_x + 14, tag_y + 21), 4, 4)

            p.setBrush(Qt.NoBrush)
            p.setPen(QColor(205, 205, 205, 145))
            p.setFont(self.font_mono(11, True))
            p.drawText(QPointF(tag_x + 28, tag_y + 17), "FORCE INPUT")

            p.setPen(QColor(235, 235, 235, 190))
            p.setFont(self.font_mono(15, True))
            p.drawText(QPointF(tag_x + 28, tag_y + 36), "NO SIGNAL")
            return

        p.setPen(QColor(235, 238, 238, 225))
        p.drawText(QPointF(50, y0 + 4), "0 N")
        p.drawText(QPointF(32, y1 + 4), f"{limit_n:g} N")

        green_end = y0 + height * 0.55
        yellow_end = y0 + height * 0.88

        p.setPen(QPen(QColor(0, 215, 72, 210), 3))
        p.drawLine(QLineF(x, y0, x, green_end))

        p.setPen(QPen(QColor(210, 220, 18, 215), 3))
        p.drawLine(QLineF(x, green_end, x, yellow_end))

        p.setPen(QPen(QColor(230, 30, 35, 220), 3))
        p.drawLine(QLineF(x, yellow_end, x, y1))

        p.setPen(QPen(QColor(235, 238, 238, 130), 1))
        for i in range(4):
            yy = y0 + height * i / 3.0
            p.drawLine(QLineF(x - 7, yy, x + 7, yy))

        marker_y = y0 + height * max(0.0, min(force_n / limit_n, 1.0))
        p.setPen(QPen(QColor(235, 238, 238, 165), 1))
        p.drawLine(QLineF(x - 14, marker_y, x + 14, marker_y))
        self.draw_old_text(p, QPointF(x + 24, marker_y + 7), f"{force_n:.1f} N", 18, False, 210)

    def draw_depth_scale(self, p, w, h):
        # HMI-P5:
        # Depth ladder removed.
        # RViz will become the dedicated visualization interface for camera/FSR.
        return


    def draw_reticle(self, p, w, h):
        # Center reticle/star removed.
        return

    def draw_camera_off_modal(self, p, w, h):
        """
        Floating camera offline overlay.
        No box. No card. No dimming.
        Icon comes from QtAwesome / Font Awesome icon library.
        """
        cx = w * 0.50
        cy = h * 0.47

        if qta is not None:
            icon = None

            # Try newest Font Awesome first, then older aliases.
            for icon_name in [
                "fa6s.video-slash",
                "fa5s.video-slash",
                "fa.video-camera",
            ]:
                try:
                    icon = qta.icon(icon_name, color="#E8ECEC")
                    break
                except Exception:
                    icon = None

            if icon is not None:
                pix = icon.pixmap(QSize(86, 86))
                p.save()
                p.setOpacity(0.92)
                p.drawPixmap(int(cx - 43), int(cy - 86), pix)
                p.restore()

        p.setFont(self.font_mono(28, True))
        p.setPen(QColor(245, 248, 248, 238))
        p.drawText(QRectF(0, cy + 8, w, 42), Qt.AlignCenter, "CAMERA FEED LOST")

        p.setFont(self.font_mono(15, False))
        p.setPen(QColor(225, 228, 228, 205))
        p.drawText(QRectF(0, cy + 55, w, 26), Qt.AlignCenter, "WAITING FOR RASPBERRY PI VIDEO STREAM")

        p.setPen(QColor(190, 195, 195, 175))
        p.drawText(QRectF(0, cy + 85, w, 26), Qt.AlignCenter, "RASPBERRY PI STREAM NOT CONFIGURED")

    def poll_surgery_pv_file(self):
        """
        Read new lines appended to runtime/surgery_pv.log and display them
        in the Surgery PV. This works even if ROS discovery is unavailable.
        """
        try:
            path = getattr(self, "pv_file_path", "")
            if not path or not os.path.exists(path):
                return

            offset = int(getattr(self, "pv_file_offset", 0))

            with open(path, "r", encoding="utf-8", errors="replace") as f:
                f.seek(offset)
                lines = f.readlines()
                self.pv_file_offset = f.tell()

            for line in lines:
                line = line.strip()
                if line:
                    add_surgery_pv_log(self.state, line)
                    self.pv_scroll_offset = 0
                    setattr(self.state, "pv_scroll_offset", 0)

        except Exception:
            pass

    def scroll_surgery_pv(self, direction):
        """
        Scroll Surgery PV history.
        direction > 0 = older logs
        direction < 0 = newer/latest logs
        """
        try:
            logs = list(getattr(self.state, "notifications", []))
            max_offset = max(0, len(logs) - 2)

            current = int(getattr(self, "pv_scroll_offset", 0))
            current += int(direction)
            current = max(0, min(max_offset, current))

            self.pv_scroll_offset = current
            setattr(self.state, "pv_scroll_offset", current)
            self.update()
        except Exception:
            pass

    def draw_bottom_library_icon(self, p, icon_names, x, y, size=16, alpha=190):
        if qta is None:
            return False

        for icon_name in icon_names:
            try:
                icon = qta.icon(icon_name, color=QColor(185, 188, 188, alpha))
                pix = icon.pixmap(QSize(size, size))
                p.drawPixmap(int(x), int(y - size + 3), pix)
                return True
            except Exception:
                pass

        return False

    def draw_input_standby_indicator(self, p, x, y):
        self.draw_bottom_library_icon(
            p,
            ["fa6s.gamepad", "fa5s.gamepad", "fa.gamepad", "fa6s.keyboard", "fa5s.keyboard"],
            x,
            y,
            size=17,
            alpha=150,
        )
        self.draw_old_text(p, QPointF(x + 26, y), "INPUT STANDBY", 12, True, 150)

    def draw_bottom_output_bar(self, p, w, h):
        top = h - 88.0

        p.setPen(QPen(QColor(235, 238, 238, 115), 1))
        p.drawLine(QLineF(w * 0.50, top + 16, w * 0.50, h - 16))

        # Left: command echo / last action.
        self.draw_old_text(p, QPointF(w * 0.205, top + 22), "LAST ACTION", 13, True, 225)

        self.draw_bottom_library_icon(
            p,
            ["fa6s.terminal", "fa5s.terminal", "fa6s.hand-pointer", "fa5s.hand-pointer"],
            w * 0.207,
            top + 58,
            size=15,
            alpha=185,
        )
        self.draw_old_text(p, QPointF(w * 0.225, top + 60), self.state.last_action, 13, True, 225)

        # Neutral input indicator when no input is currently pressed.
        snap = self.input_state.snapshot()
        if not any(snap["active"].values()):
            self.draw_input_standby_indicator(p, w * 0.325, top + 60)

        # Right: surgery PV / live session log.
        logs = list(getattr(self.state, "notifications", []))
        if not logs:
            logs = [("00:00:00", "Session started")]

        max_offset = max(0, len(logs) - 2)
        offset = int(getattr(self, "pv_scroll_offset", getattr(self.state, "pv_scroll_offset", 0)))
        offset = max(0, min(max_offset, offset))
        self.pv_scroll_offset = offset

        end = len(logs) - offset
        start = max(0, end - 2)
        visible_logs = logs[start:end]

        self.draw_old_text(p, QPointF(w * 0.645, top + 22), "SURGERY PV", 13, True, 225)

        if max_offset > 0:
            indicator = "LATEST" if offset == 0 else f"HISTORY -{offset}"
            self.draw_old_text(p, QPointF(w * 0.785, top + 22), indicator, 10, True, 145)
            self.draw_bottom_library_icon(
                p,
                ["fa6s.clock-rotate-left", "fa5s.history", "fa.history"],
                w * 0.755,
                top + 22,
                size=13,
                alpha=125,
            )

        y = top + 50
        for stamp, message in visible_logs:
            message = " ".join(str(message).splitlines()).strip()
            if len(message) > 64:
                message = message[:61] + "..."
            self.draw_old_text(p, QPointF(w * 0.575, y), f"[{stamp}]  {message}", 12, False, 220)
            y += 22


    # ------------------------------------------------------------
    # Live joystick icon overlay
    # ------------------------------------------------------------
    def draw_live_input_icons(self, p, w, h):
        snap = self.input_state.snapshot()
        active = snap["active"]

        # Nothing pressed: do not show the old large fake controls.
        # Bottom bar already shows a subtle library-based INPUT STANDBY indicator.
        if not any(active.values()):
            return

        base_y = h - 230.0

        if active.get("stick", False):
            self.draw_l_stick_icon(
                p,
                x=160.0,
                y=base_y,
                lx=snap["lx"],
                ly=snap["ly"],
                alpha=snap["alpha"].get("stick", 1.0),
                active=active,
                alpha_map=snap["alpha"],
            )

        if active.get("button_a", False):
            self.draw_round_button(
                p,
                cx=w - 225.0,
                cy=base_y + 20.0,
                label="A",
                fill=QColor(30, 225, 50),
                alpha=snap["alpha"].get("button_a", 1.0),
            )

        if active.get("button_b", False):
            self.draw_round_button(
                p,
                cx=w - 145.0,
                cy=base_y + 20.0,
                label="B",
                fill=QColor(235, 25, 35),
                alpha=snap["alpha"].get("button_b", 1.0),
            )


    def with_alpha(self, color, alpha_factor):
        c = QColor(color)
        c.setAlpha(max(0, min(255, int(c.alpha() * alpha_factor))))
        return c

    def draw_l_stick_icon(self, p, x, y, lx, ly, alpha, active, alpha_map):
        a = max(0.15, min(1.0, alpha))
        s = 1.0

        # Top L stick circle, close to the uploaded controller style.
        p.setPen(QPen(QColor(0, 0, 0, int(230 * a)), 4))
        p.setBrush(QColor(245, 245, 245, int(235 * a)))
        p.drawEllipse(QPointF(x, y - 72), 58 * s, 38 * s)

        p.setPen(QPen(QColor(255, 255, 255, int(210 * a)), 2))
        p.setBrush(QColor(245, 245, 245, int(80 * a)))
        p.drawEllipse(QPointF(x, y - 78), 43 * s, 27 * s)

        p.setFont(self.font_mono(38, True))
        p.setPen(QColor(0, 0, 0, int(245 * a)))
        p.drawText(QRectF(x - 35, y - 101, 70, 55), Qt.AlignCenter, "L")

        # Stem shadow.
        p.setPen(Qt.NoPen)
        p.setBrush(QColor(245, 245, 245, int(225 * a)))
        p.drawRoundedRect(QRectF(x - 40, y - 48, 80, 24), 8, 8)

        # D-pad style cross.
        cx = x
        cy = y + 20
        size = 38
        gap = 4

        self.draw_dpad_segment(p, QRectF(cx - size / 2, cy - size * 1.5 - gap, size, size), "up", active, alpha_map, a)
        self.draw_dpad_segment(p, QRectF(cx - size / 2, cy + size * 0.5 + gap, size, size), "down", active, alpha_map, a)
        self.draw_dpad_segment(p, QRectF(cx - size * 1.5 - gap, cy - size / 2, size, size), "left", active, alpha_map, a)
        self.draw_dpad_segment(p, QRectF(cx + size * 0.5 + gap, cy - size / 2, size, size), "right", active, alpha_map, a)

        # Center block.
        p.setPen(QPen(QColor(20, 20, 20, int(180 * a)), 3))
        p.setBrush(QColor(155, 155, 155, int(220 * a)))
        p.drawRect(QRectF(cx - size / 2, cy - size / 2, size, size))

    def draw_dpad_segment(self, p, rect, key, active, alpha_map, global_alpha):
        local_alpha = alpha_map.get(key, 0.0) if active.get(key, False) else 0.0
        fill = QColor(220, 220, 220, int(245 * max(local_alpha, 0.0))) if local_alpha > 0.0 else QColor(145, 145, 145, int(175 * global_alpha))

        p.setPen(QPen(QColor(30, 30, 30, int(220 * global_alpha)), 3))
        p.setBrush(fill)
        p.drawRect(rect)

        # Small arrow marker.
        p.setPen(QPen(QColor(20, 20, 20, int(230 * global_alpha)), 2))
        cx = rect.center().x()
        cy = rect.center().y()
        s = min(rect.width(), rect.height()) * 0.24

        if key == "up":
            pts = [QPointF(cx, cy - s), QPointF(cx - s, cy + s), QPointF(cx + s, cy + s)]
        elif key == "down":
            pts = [QPointF(cx, cy + s), QPointF(cx - s, cy - s), QPointF(cx + s, cy - s)]
        elif key == "left":
            pts = [QPointF(cx - s, cy), QPointF(cx + s, cy - s), QPointF(cx + s, cy + s)]
        else:
            pts = [QPointF(cx + s, cy), QPointF(cx - s, cy - s), QPointF(cx - s, cy + s)]

        p.setBrush(QColor(235, 235, 235, int(230 * global_alpha)))
        p.drawPolygon(*pts)

    def draw_round_button(self, p, cx, cy, label, fill, alpha):
        a = max(0.15, min(1.0, alpha))
        r = 36.0

        # Outer shadow.
        p.setPen(QPen(QColor(0, 0, 0, int(220 * a)), 5))
        p.setBrush(QColor(10, 10, 10, int(90 * a)))
        p.drawEllipse(QPointF(cx + 2, cy + 3), r + 5, r + 5)

        # Main button.
        p.setPen(QPen(QColor(0, 0, 0, int(230 * a)), 4))
        p.setBrush(self.with_alpha(fill, a))
        p.drawEllipse(QPointF(cx, cy), r, r)

        # Top highlight.
        p.setPen(QPen(QColor(255, 255, 255, int(90 * a)), 2))
        p.setBrush(Qt.NoBrush)
        p.drawEllipse(QPointF(cx - 4, cy - 6), r * 0.58, r * 0.38)

        # Letter.
        p.setFont(self.font_mono(34, True))
        p.setPen(QColor(0, 0, 0, int(245 * a)))
        p.drawText(QRectF(cx - r, cy - r, r * 2, r * 2 + 4), Qt.AlignCenter, label)

class SurgicalHudCanvas(QWidget):
    def __init__(self, state: HudState, camera_index: Optional[int], background_path: Optional[str], stream_url: Optional[str], input_state: InputOverlayState):
        super().__init__()
        self.input_state = input_state
        self.video = VideoLayer(camera_index=camera_index, background_path=background_path, stream_url=stream_url, parent=self)
        self.overlay = HudOverlay(state=state, video_layer=self.video, input_state=input_state, parent=self)
        self.setFocusPolicy(Qt.StrongFocus)
        self.overlay.raise_()

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self.video.setGeometry(0, 0, self.width(), self.height())
        self.overlay.setGeometry(0, 0, self.width(), self.height())
        self.overlay.raise_()

    def wheelEvent(self, event):
        try:
            delta = event.angleDelta().y()
            if delta > 0:
                self.overlay.scroll_surgery_pv(1)
            elif delta < 0:
                self.overlay.scroll_surgery_pv(-1)
            event.accept()
        except Exception:
            super().wheelEvent(event)

    def close(self):
        self.video.close()
        super().close()


class SurgicalHudWindow(QMainWindow):
    def __init__(self, state: HudState, camera_index: Optional[int], background_path: Optional[str], stream_url: Optional[str], input_state: InputOverlayState):
        super().__init__()
        self.setWindowTitle("Surgical HMI Production")
        self.state = state
        self.input_state = input_state
        self.canvas = SurgicalHudCanvas(state, camera_index, background_path, stream_url, input_state)
        self.setCentralWidget(self.canvas)
        self.setFocusPolicy(Qt.StrongFocus)
        self.resize(1600, 900)

    def keyPressEvent(self, event):
        if event.key() == Qt.Key_Escape:
            self.close()
        elif event.key() == Qt.Key_F11:
            if self.isFullScreen():
                self.showNormal()
            else:
                self.showFullScreen()
        elif event.key() in (Qt.Key_PageUp, Qt.Key_Up):
            self.canvas.overlay.scroll_surgery_pv(1)
        elif event.key() in (Qt.Key_PageDown, Qt.Key_Down):
            self.canvas.overlay.scroll_surgery_pv(-1)
        else:
            super().keyPressEvent(event)


def load_state(config_path: str) -> HudState:
    state = HudState()

    # Real/offline defaults. No fake force/link data until ROS data arrives.
    state.force_limit_n = 15.0
    state.force_online = False
    state.force_last_seen = 0.0
    state.link_online = False
    state.link_last_seen = 0.0
    state.latency_ms = 0
    state.notifications = [("00:00:00", "Session started")]
    state.pv_scroll_offset = 0

    if not config_path or not os.path.exists(config_path):
        return state

    try:
        with open(config_path, "r", encoding="utf-8-sig") as f:
            cfg = json.load(f)
    except Exception:
        return state

    try:
        state.case_id = str(cfg.get("case_id", state.case_id))
        state.action = str(cfg.get("action", state.action))
        state.operator_name = str(cfg.get("operator_name", state.operator_name))
        state.depth_mm = int(cfg.get("depth_mm", state.depth_mm))
        state.last_action = str(cfg.get("last_action", state.last_action))
        state.force_limit_n = float(cfg.get("force_limit_n", state.force_limit_n))

        raw_notifications = cfg.get("notifications", None)
        if isinstance(raw_notifications, list) and raw_notifications:
            cleaned = []
            for item in raw_notifications:
                if isinstance(item, (list, tuple)) and len(item) >= 2:
                    cleaned.append((str(item[0]), str(item[1])))
            if cleaned:
                state.notifications = cleaned
    except Exception:
        pass

    return state


def parse_args():
    parser = argparse.ArgumentParser(description="Surgical HMI Production")

    parser.add_argument("--config", default=None)
    parser.add_argument("--camera", type=int, default=None)

    parser.add_argument("--stream", default=None)
    parser.add_argument("--stream-url", dest="stream_url", default=None)

    parser.add_argument("--background", dest="background", default=None)
    parser.add_argument("--background-path", dest="background_path", default=None)

    parser.add_argument("--fullscreen", action="store_true")
    parser.add_argument("--input-test", action="store_true")

    parser.set_defaults(ros_input=False)
    parser.add_argument("--ros-input", dest="ros_input", action="store_true")
    parser.add_argument("--no-ros-input", dest="ros_input", action="store_false")

    args = parser.parse_args()

    if args.stream_url is None:
        args.stream_url = args.stream
    if args.stream is None:
        args.stream = args.stream_url

    if args.background_path is None:
        args.background_path = args.background
    if args.background is None:
        args.background = args.background_path

    return args


def main():
    app = QApplication(sys.argv)
    args = parse_args()

    state = load_state(args.config)
    input_state = InputOverlayState()

    win = SurgicalHudWindow(
        state=state,
        camera_index=args.camera,
        background_path=args.background_path,
        stream_url=args.stream_url,
        input_state=input_state,
    )

    if bool(getattr(args, "input_test", False)):
        input_state.update_joy([0.90, 0.70], [1, 1])
        try:
            win.canvas.overlay.input_test_mode = True
        except Exception:
            pass

    win._ros_bridge = None
    if bool(getattr(args, "ros_input", False)):
        try:
            win._ros_bridge = RosInputBridge(
                input_state=input_state,
                enabled=True,
                hmi_state=state,
            )
        except Exception as exc:
            print(f"HMI ROS bridge startup warning: {exc}", flush=True)

    if bool(getattr(args, "fullscreen", False)):
        win.showFullScreen()
    else:
        win.show()

    sys.exit(app.exec_())


if __name__ == "__main__":
    main()

