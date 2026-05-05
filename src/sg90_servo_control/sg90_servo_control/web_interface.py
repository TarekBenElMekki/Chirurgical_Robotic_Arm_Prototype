#!/usr/bin/env python3

import json
import queue
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


HTML_PAGE = r"""<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1.0" />
  <title>SG90 Servo Control</title>
  <style>
    * {
      box-sizing: border-box;
    }

    body {
      margin: 0;
      min-height: 100vh;
      background: #050505;
      color: #f5f5f5;
      font-family: Arial, Helvetica, sans-serif;
      display: flex;
      align-items: center;
      justify-content: center;
    }

    .panel {
      width: min(920px, 94vw);
      background: #121212;
      border: 1px solid #2a2a2a;
      border-radius: 24px;
      padding: 32px;
      box-shadow: 0 24px 80px rgba(0, 0, 0, 0.5);
    }

    h1 {
      margin: 0 0 8px;
      font-size: 36px;
      letter-spacing: -0.04em;
    }

    .subtitle {
      margin: 0 0 28px;
      color: #aaa;
      font-size: 16px;
    }

    .angle-display {
      font-size: clamp(64px, 12vw, 130px);
      font-weight: 800;
      text-align: center;
      margin: 20px 0;
      letter-spacing: -0.08em;
    }

    .angle-display span {
      font-size: 0.35em;
      color: #aaa;
      letter-spacing: 0;
    }

    input[type="range"] {
      width: 100%;
      margin: 24px 0;
      accent-color: #ffffff;
    }

    .controls {
      display: grid;
      grid-template-columns: 1fr auto;
      gap: 14px;
      margin-top: 18px;
    }

    input[type="number"] {
      width: 100%;
      padding: 18px;
      border-radius: 16px;
      border: 1px solid #333;
      background: #080808;
      color: white;
      font-size: 24px;
      outline: none;
    }

    button {
      border: 0;
      border-radius: 16px;
      background: #f5f5f5;
      color: #000;
      padding: 0 28px;
      font-size: 18px;
      font-weight: 800;
      cursor: pointer;
    }

    button:hover {
      background: #dcdcdc;
    }

    .preset-grid {
      display: grid;
      grid-template-columns: repeat(5, 1fr);
      gap: 10px;
      margin-top: 16px;
    }

    .preset-grid button {
      padding: 16px 10px;
      background: #242424;
      color: #f5f5f5;
      border: 1px solid #333;
    }

    .preset-grid button:hover {
      background: #333;
    }

    .status {
      display: grid;
      grid-template-columns: repeat(3, 1fr);
      gap: 12px;
      margin-top: 24px;
    }

    .card {
      background: #080808;
      border: 1px solid #252525;
      border-radius: 18px;
      padding: 16px;
    }

    .label {
      color: #888;
      font-size: 13px;
      text-transform: uppercase;
      letter-spacing: 0.08em;
      margin-bottom: 8px;
    }

    .value {
      font-size: 22px;
      font-weight: 700;
    }

    .warning {
      margin-top: 20px;
      color: #ffc766;
      background: rgba(255, 199, 102, 0.08);
      border: 1px solid rgba(255, 199, 102, 0.25);
      border-radius: 14px;
      padding: 14px;
      font-size: 14px;
      line-height: 1.5;
    }

    @media (max-width: 720px) {
      .controls {
        grid-template-columns: 1fr;
      }

      button {
        padding: 18px;
      }

      .preset-grid {
        grid-template-columns: repeat(2, 1fr);
      }

      .status {
        grid-template-columns: 1fr;
      }
    }
  </style>
</head>
<body>
  <main class="panel">
    <h1>SG90 Servo Control</h1>
    <p class="subtitle">Send an exact angle command to the ROS 2 topic <b>/sg90/angle_deg</b>.</p>

    <div class="angle-display">
      <span id="angleText">90.0</span><span>°</span>
    </div>

    <input id="angleSlider" type="range" min="0" max="180" step="0.1" value="90" />

    <div class="controls">
      <input id="angleInput" type="number" min="0" max="180" step="0.1" value="90.0" />
      <button onclick="sendAngle()">Send Angle</button>
    </div>

    <div class="preset-grid">
      <button onclick="setAndSend(0)">0°</button>
      <button onclick="setAndSend(45)">45°</button>
      <button onclick="setAndSend(90)">90°</button>
      <button onclick="setAndSend(135)">135°</button>
      <button onclick="setAndSend(180)">180°</button>
    </div>

    <section class="status">
      <div class="card">
        <div class="label">Requested</div>
        <div class="value" id="requestedValue">90.0°</div>
      </div>

      <div class="card">
        <div class="label">Servo State</div>
        <div class="value" id="stateValue">waiting...</div>
      </div>

      <div class="card">
        <div class="label">Connection</div>
        <div class="value" id="connectionValue">starting...</div>
      </div>
    </section>

  </main>

  <script>
    const slider = document.getElementById("angleSlider");
    const input = document.getElementById("angleInput");
    const angleText = document.getElementById("angleText");
    const requestedValue = document.getElementById("requestedValue");
    const stateValue = document.getElementById("stateValue");
    const connectionValue = document.getElementById("connectionValue");

    function clampAngle(value) {
      let angle = Number(value);

      if (Number.isNaN(angle)) {
        angle = 90.0;
      }

      angle = Math.max(0.0, Math.min(180.0, angle));
      return Math.round(angle * 10) / 10;
    }

    function setAngleLocal(value) {
      const angle = clampAngle(value);
      slider.value = angle;
      input.value = angle.toFixed(1);
      angleText.textContent = angle.toFixed(1);
      requestedValue.textContent = angle.toFixed(1) + "°";
      return angle;
    }

    slider.addEventListener("input", () => {
      setAngleLocal(slider.value);
    });

    input.addEventListener("input", () => {
      setAngleLocal(input.value);
    });

    input.addEventListener("keydown", (event) => {
      if (event.key === "Enter") {
        sendAngle();
      }
    });

    async function sendAngle() {
      const angle = setAngleLocal(input.value);

      try {
        const response = await fetch("/api/angle", {
          method: "POST",
          headers: {
            "Content-Type": "application/json",
          },
          body: JSON.stringify({ angle }),
        });

        const data = await response.json();

        if (!response.ok) {
          throw new Error(data.error || "Failed to send angle");
        }

        setAngleLocal(data.angle);
        connectionValue.textContent = "connected";
      } catch (error) {
        connectionValue.textContent = "error";
        alert(error.message);
      }
    }

    function setAndSend(angle) {
      setAngleLocal(angle);
      sendAngle();
    }

    async function refreshState() {
      try {
        const response = await fetch("/api/state");
        const data = await response.json();

        connectionValue.textContent = "connected";

        if (data.last_commanded_angle !== null) {
          requestedValue.textContent = Number(data.last_commanded_angle).toFixed(1) + "°";
        }

        if (data.last_state_angle !== null) {
          stateValue.textContent = Number(data.last_state_angle).toFixed(1) + "°";
        } else {
          stateValue.textContent = "waiting...";
        }
      } catch (error) {
        connectionValue.textContent = "error";
      }
    }

    setAngleLocal(90.0);
    refreshState();
    setInterval(refreshState, 500);
  </script>
</body>
</html>
"""


class Sg90WebInterface(Node):
    def __init__(self):
        super().__init__("sg90_web_interface")

        self.declare_parameter("host", "0.0.0.0")
        self.declare_parameter("port", 8080)

        self.declare_parameter("min_angle_deg", 0.0)
        self.declare_parameter("max_angle_deg", 180.0)
        self.declare_parameter("default_angle_deg", 90.0)

        self.declare_parameter("command_topic", "/sg90/angle_deg")
        self.declare_parameter("state_topic", "/sg90/state_deg")

        self.host = self.get_parameter("host").value
        self.port = int(self.get_parameter("port").value)

        self.min_angle = float(self.get_parameter("min_angle_deg").value)
        self.max_angle = float(self.get_parameter("max_angle_deg").value)
        self.default_angle = float(self.get_parameter("default_angle_deg").value)

        self.command_topic = self.get_parameter("command_topic").value
        self.state_topic = self.get_parameter("state_topic").value

        self.command_pub = self.create_publisher(Float32, self.command_topic, 10)
        self.state_sub = self.create_subscription(
            Float32,
            self.state_topic,
            self.state_callback,
            10,
        )

        self.command_queue = queue.Queue()
        self.last_commanded_angle = None
        self.last_state_angle = None

        self.command_timer = self.create_timer(0.05, self.publish_pending_command)

        handler = self.make_handler()
        self.http_server = ThreadingHTTPServer((self.host, self.port), handler)
        self.http_thread = threading.Thread(
            target=self.http_server.serve_forever,
            daemon=True,
        )
        self.http_thread.start()

        self.get_logger().info("SG90 web interface started")
        self.get_logger().info(f"Open: http://{self.host}:{self.port}")
        self.get_logger().info(f"Publishing commands to: {self.command_topic}")
        self.get_logger().info(f"Reading state from: {self.state_topic}")

    def clamp_angle(self, angle):
        return max(self.min_angle, min(self.max_angle, float(angle)))

    def enqueue_angle(self, angle):
        safe_angle = self.clamp_angle(angle)
        self.command_queue.put(safe_angle)
        self.last_commanded_angle = safe_angle
        return safe_angle

    def publish_pending_command(self):
        latest_angle = None

        while True:
            try:
                latest_angle = self.command_queue.get_nowait()
            except queue.Empty:
                break

        if latest_angle is None:
            return

        msg = Float32()
        msg.data = float(latest_angle)
        self.command_pub.publish(msg)
        self.last_commanded_angle = latest_angle

        self.get_logger().info(
            f"Web command published: {latest_angle:.1f} deg",
            throttle_duration_sec=0.1,
        )

    def state_callback(self, msg):
        self.last_state_angle = float(msg.data)

    def make_handler(self):
        node = self

        class ServoRequestHandler(BaseHTTPRequestHandler):
            def log_message(self, format, *args):
                return

            def send_text(self, text, status=200, content_type="text/html"):
                payload = text.encode("utf-8")
                self.send_response(status)
                self.send_header("Content-Type", content_type)
                self.send_header("Content-Length", str(len(payload)))
                self.end_headers()
                self.wfile.write(payload)

            def send_json(self, data, status=200):
                payload = json.dumps(data).encode("utf-8")
                self.send_response(status)
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(payload)))
                self.end_headers()
                self.wfile.write(payload)

            def do_GET(self):
                if self.path == "/" or self.path == "/index.html":
                    self.send_text(HTML_PAGE)
                    return

                if self.path == "/api/state":
                    self.send_json(
                        {
                            "min_angle": node.min_angle,
                            "max_angle": node.max_angle,
                            "last_commanded_angle": node.last_commanded_angle,
                            "last_state_angle": node.last_state_angle,
                            "command_topic": node.command_topic,
                            "state_topic": node.state_topic,
                        }
                    )
                    return

                self.send_json({"error": "Not found"}, status=404)

            def do_POST(self):
                if self.path != "/api/angle":
                    self.send_json({"error": "Not found"}, status=404)
                    return

                try:
                    content_length = int(self.headers.get("Content-Length", "0"))
                    body = self.rfile.read(content_length).decode("utf-8")
                    data = json.loads(body)
                    angle = float(data["angle"])
                    safe_angle = node.enqueue_angle(angle)

                    self.send_json(
                        {
                            "ok": True,
                            "angle": safe_angle,
                            "command_topic": node.command_topic,
                        }
                    )
                except Exception as exc:
                    self.send_json({"error": str(exc)}, status=400)

        return ServoRequestHandler

    def destroy_node(self):
        try:
            self.http_server.shutdown()
            self.http_server.server_close()
        except Exception as exc:
            print(f"[sg90_web_interface] HTTP shutdown warning: {exc}")

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = None

    try:
        node = Sg90WebInterface()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        print(f"[sg90_web_interface] ERROR: {exc}")
    finally:
        if node is not None:
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
