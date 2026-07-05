#!/usr/bin/env python3
import subprocess
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

try:
    import cv2
    import numpy as np
except Exception:
    cv2 = None
    np = None


class CameraMjpegServer(Node):
    def __init__(self):
        super().__init__("camera_mjpeg_server")

        self.declare_parameter("device", "/dev/video0")
        self.declare_parameter("width", 640)
        self.declare_parameter("height", 480)
        self.declare_parameter("pixelformat", "GB10")
        self.declare_parameter("port", 8081)
        self.declare_parameter("jpeg_quality", 75)
        self.declare_parameter("stream_fps", 20.0)
        self.declare_parameter("gamma", 0.60)
        self.declare_parameter("contrast", 1.20)
        self.declare_parameter("brightness", 10)
        self.declare_parameter("saturation", 1.15)
        self.declare_parameter("auto_white_balance", True)
        self.declare_parameter("sharpen", 0.15)
        self.declare_parameter("red_gain", 1.25)
        self.declare_parameter("green_gain", 1.00)
        self.declare_parameter("blue_gain", 0.88)
        self.declare_parameter("stable_gray", False)

        self.device = str(self.get_parameter("device").value)
        self.width = int(self.get_parameter("width").value)
        self.height = int(self.get_parameter("height").value)
        self.pixelformat = str(self.get_parameter("pixelformat").value)
        self.port = int(self.get_parameter("port").value)
        self.jpeg_quality = int(self.get_parameter("jpeg_quality").value)
        self.stream_fps = float(self.get_parameter("stream_fps").value)
        self.gamma = float(self.get_parameter("gamma").value)
        self.contrast = float(self.get_parameter("contrast").value)
        self.brightness = int(self.get_parameter("brightness").value)
        self.saturation = float(self.get_parameter("saturation").value)
        self.auto_white_balance = bool(self.get_parameter("auto_white_balance").value)
        self.sharpen = float(self.get_parameter("sharpen").value)
        self.red_gain = float(self.get_parameter("red_gain").value)
        self.green_gain = float(self.get_parameter("green_gain").value)
        self.blue_gain = float(self.get_parameter("blue_gain").value)
        self.stable_gray = bool(self.get_parameter("stable_gray").value)

        self.log_pub = self.create_publisher(String, "/camera/log", 10)

        self.running = True
        self.proc = None
        self.latest_jpeg = None
        self.latest_stamp = 0.0
        self.frame_count = 0
        self.last_frame_count = 0
        self.measured_fps = 0.0

        self.lock = threading.Lock()
        self.condition = threading.Condition(self.lock)

        self.publish_log(
            f"Camera RAW Bayer MJPEG server starting: {self.device}, "
            f"{self.width}x{self.height}, {self.pixelformat}, port {self.port}"
        )

        self.capture_thread = threading.Thread(target=self.capture_loop, daemon=True)
        self.capture_thread.start()

        self.httpd = ThreadingHTTPServer(("0.0.0.0", self.port), self.make_handler())
        self.http_thread = threading.Thread(target=self.httpd.serve_forever, daemon=True)
        self.http_thread.start()

        self.publish_log(f"Camera stream ready: http://0.0.0.0:{self.port}/stream.mjpg")
        self.status_timer = self.create_timer(1.0, self.status_tick)

    def publish_log(self, text: str):
        msg = String()
        msg.data = str(text)
        self.log_pub.publish(msg)
        self.get_logger().info(str(text))

    def frame_size_bytes(self):
        if self.pixelformat == "GB10":
            return self.width * self.height * 2
        if self.pixelformat == "pGAA":
            return int(self.width * self.height * 10 / 8)
        raise RuntimeError(f"Unsupported pixelformat: {self.pixelformat}")

    def start_v4l2_stream(self):
        cmd = [
            "v4l2-ctl",
            "-d", self.device,
            f"--set-fmt-video=width={self.width},height={self.height},pixelformat={self.pixelformat}",
            "--stream-mmap=3",
            "--stream-count=100000000",
            "--stream-to=-",
        ]

        self.publish_log("Starting v4l2 raw stream: " + " ".join(cmd))

        return subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            bufsize=0,
        )

    def read_exact(self, stream, size):
        data = bytearray()

        while self.running and len(data) < size:
            chunk = stream.read(size - len(data))
            if not chunk:
                return None
            data.extend(chunk)

        return bytes(data)

    def gb10_to_bgr(self, data):
        raw16 = np.frombuffer(data, dtype=np.uint16)

        if raw16.size < self.width * self.height:
            return None

        raw16 = raw16[: self.width * self.height].reshape((self.height, self.width))
        raw10 = raw16 & 0x03FF

        raw8 = np.clip(raw10 >> 2, 0, 255).astype(np.uint8)

        if bool(getattr(self, "stable_gray", False)):
            return cv2.cvtColor(raw8, cv2.COLOR_GRAY2BGR)

        # OV5647 media graph reports GBGB/RGRG pattern.
        bgr = cv2.cvtColor(raw8, cv2.COLOR_BAYER_GB2BGR)

        return bgr

    def pgaa_to_bgr(self, data):
        # Packed 10-bit Bayer is more compact, but GB10 is simpler and already works.
        # Keep this placeholder so the node fails clearly if pGAA is selected.
        raise RuntimeError("pGAA conversion not enabled; use pixelformat:=GB10")

    def raw_to_bgr(self, data):
        if self.pixelformat == "GB10":
            return self.gb10_to_bgr(data)
        if self.pixelformat == "pGAA":
            return self.pgaa_to_bgr(data)
        return None


    def apply_auto_white_balance(self, bgr):
        if not self.auto_white_balance:
            return bgr

        try:
            img = bgr.astype(np.float32)
            b, g, r = cv2.split(img)

            mb = max(float(np.mean(b)), 1.0)
            mg = max(float(np.mean(g)), 1.0)
            mr = max(float(np.mean(r)), 1.0)

            gray = (mb + mg + mr) / 3.0
            strength = 0.65

            sb = 1.0 + ((gray / mb) - 1.0) * strength
            sg = 1.0 + ((gray / mg) - 1.0) * strength
            sr = 1.0 + ((gray / mr) - 1.0) * strength

            out = cv2.merge([b * sb, g * sg, r * sr])
            return np.clip(out, 0, 255).astype(np.uint8)
        except Exception:
            return bgr

    def apply_channel_gains(self, bgr):
        try:
            img = bgr.astype(np.float32)
            b, g, r = cv2.split(img)

            b *= max(0.40, min(2.50, float(self.blue_gain)))
            g *= max(0.40, min(2.50, float(self.green_gain)))
            r *= max(0.40, min(2.50, float(self.red_gain)))

            out = cv2.merge([b, g, r])
            return np.clip(out, 0, 255).astype(np.uint8)
        except Exception:
            return bgr

    def apply_gamma(self, bgr):
        gamma = max(0.20, min(2.50, float(self.gamma)))

        if abs(gamma - 1.0) < 0.02:
            return bgr

        table = ((np.arange(256, dtype=np.float32) / 255.0) ** gamma) * 255.0
        table = np.clip(table, 0, 255).astype(np.uint8)
        return cv2.LUT(bgr, table)

    def apply_saturation(self, bgr):
        sat = max(0.0, min(3.0, float(self.saturation)))

        if abs(sat - 1.0) < 0.02:
            return bgr

        try:
            hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV).astype(np.float32)
            hsv[:, :, 1] *= sat
            hsv[:, :, 1] = np.clip(hsv[:, :, 1], 0, 255)
            return cv2.cvtColor(hsv.astype(np.uint8), cv2.COLOR_HSV2BGR)
        except Exception:
            return bgr

    def apply_sharpen(self, bgr):
        amount = max(0.0, min(1.5, float(self.sharpen)))

        if amount <= 0.01:
            return bgr

        try:
            blurred = cv2.GaussianBlur(bgr, (0, 0), 1.0)
            return cv2.addWeighted(bgr, 1.0 + amount, blurred, -amount, 0)
        except Exception:
            return bgr

    def enhance_bgr(self, bgr):
        # 1) Correct color cast from raw Bayer conversion
        bgr = self.apply_auto_white_balance(bgr)
        bgr = self.apply_channel_gains(bgr)

        # 2) Brighten shadows without exploding highlights
        bgr = self.apply_gamma(bgr)

        # 3) Contrast and brightness
        bgr = cv2.convertScaleAbs(
            bgr,
            alpha=max(0.5, min(3.0, float(self.contrast))),
            beta=max(-100, min(100, int(self.brightness)))
        )

        # 4) Restore saturation
        bgr = self.apply_saturation(bgr)

        # 5) Slight sharpening after compression
        bgr = self.apply_sharpen(bgr)

        return bgr

    def capture_loop(self):
        if cv2 is None or np is None:
            self.publish_log("ERROR: OpenCV or NumPy missing")
            return

        frame_bytes = self.frame_size_bytes()

        while self.running:
            try:
                self.proc = self.start_v4l2_stream()

                if self.proc.stdout is None:
                    self.publish_log("ERROR: v4l2 stream stdout unavailable")
                    time.sleep(1.0)
                    continue

                encode_period = 1.0 / max(1.0, float(self.stream_fps))
                next_encode_time = 0.0

                while self.running:
                    data = self.read_exact(self.proc.stdout, frame_bytes)

                    if data is None:
                        self.publish_log("Camera raw stream ended; reconnecting")
                        break

                    now = time.time()
                    if now < next_encode_time:
                        continue
                    next_encode_time = now + encode_period

                    bgr = self.raw_to_bgr(data)

                    if bgr is None:
                        self.publish_log("Camera raw conversion failed")
                        continue

                    if not bool(getattr(self, "stable_gray", False)):
                        bgr = self.enhance_bgr(bgr)

                    ok, encoded = cv2.imencode(
                        ".jpg",
                        bgr,
                        [int(cv2.IMWRITE_JPEG_QUALITY), max(20, min(95, self.jpeg_quality))]
                    )

                    if ok:
                        jpeg = encoded.tobytes()
                        with self.condition:
                            self.latest_jpeg = jpeg
                            self.latest_stamp = time.time()
                            self.frame_count += 1
                            self.condition.notify_all()

            except Exception as exc:
                self.publish_log(f"Camera capture error: {exc}")

            finally:
                try:
                    if self.proc is not None:
                        self.proc.terminate()
                        self.proc.wait(timeout=1.0)
                except Exception:
                    try:
                        self.proc.kill()
                    except Exception:
                        pass

                self.proc = None

            time.sleep(0.5)

    def get_latest_jpeg(self, timeout=1.0):
        deadline = time.time() + timeout

        with self.condition:
            while self.running and self.latest_jpeg is None and time.time() < deadline:
                self.condition.wait(timeout=0.1)

            return self.latest_jpeg

    def make_handler(self):
        node = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, fmt, *args):
                return

            def do_GET(self):
                if self.path in ["/", "/status"]:
                    body = (
                        "Raspberry Pi Surgical Camera Stream\n"
                        "mode: RAW Bayer GB10 to MJPEG\n"
                        "stream: /stream.mjpg\n"
                        "snapshot: /snapshot.jpg\n"
                        f"fps: {node.measured_fps:.1f}\n"
                    ).encode("utf-8")

                    self.send_response(200)
                    self.send_header("Content-Type", "text/plain")
                    self.send_header("Content-Length", str(len(body)))
                    self.end_headers()
                    self.wfile.write(body)
                    return

                if self.path.startswith("/snapshot.jpg"):
                    jpeg = node.get_latest_jpeg(timeout=2.0)

                    if jpeg is None:
                        self.send_response(503)
                        self.end_headers()
                        return

                    self.send_response(200)
                    self.send_header("Content-Type", "image/jpeg")
                    self.send_header("Content-Length", str(len(jpeg)))
                    self.end_headers()
                    self.wfile.write(jpeg)
                    return

                if self.path.startswith("/stream.mjpg"):
                    self.send_response(200)
                    self.send_header("Age", "0")
                    self.send_header("Cache-Control", "no-cache, private")
                    self.send_header("Pragma", "no-cache")
                    self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
                    self.end_headers()

                    while node.running:
                        jpeg = node.get_latest_jpeg(timeout=2.0)

                        if jpeg is None:
                            continue

                        try:
                            self.wfile.write(b"--frame\r\n")
                            self.wfile.write(b"Content-Type: image/jpeg\r\n")
                            self.wfile.write(f"Content-Length: {len(jpeg)}\r\n\r\n".encode("ascii"))
                            self.wfile.write(jpeg)
                            self.wfile.write(b"\r\n")
                            self.wfile.flush()
                            time.sleep(1.0 / max(1.0, float(node.stream_fps)))
                        except Exception:
                            break

                    return

                self.send_response(404)
                self.end_headers()

        return Handler

    def status_tick(self):
        now_count = self.frame_count
        self.measured_fps = float(now_count - self.last_frame_count)
        self.last_frame_count = now_count

        age = time.time() - self.latest_stamp if self.latest_stamp else 999.0

        if age < 2.0:
            self.publish_log(
                f"Camera online: {self.width}x{self.height}, {self.pixelformat}, "
                f"fps={self.measured_fps:.1f}, stream=/stream.mjpg"
            )
        else:
            self.publish_log("Camera waiting for RAW frames")

    def destroy_node(self):
        self.running = False

        try:
            self.httpd.shutdown()
        except Exception:
            pass

        try:
            if self.proc is not None:
                self.proc.terminate()
        except Exception:
            pass

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraMjpegServer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
