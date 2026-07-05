#!/usr/bin/env bash
set -e

source ~/ros2_net.sh
source ~/pfe_raspi_ws/install/setup.bash

echo "Starting Raspberry Pi runtime nodes..."
echo "- raspi_heartbeat_node: 5 Hz link publish, real PC latency"
echo "- force_sensor_node: 100 Hz FSR publish"
echo "- camera_mjpeg_server: STABLE COLOR 640x480, 20 FPS, JPEG 55"

pids=()

cleanup() {
  echo ""
  echo "Stopping Raspberry Pi runtime nodes..."
  for p in "${pids[@]}"; do
    kill "$p" 2>/dev/null || true
  done
  wait 2>/dev/null || true
}

trap cleanup INT TERM EXIT

ros2 run pfe_raspi_runtime raspi_heartbeat_node --ros-args \
  -p publish_rate_hz:=5.0 \
  -p target_host:=100.77.116.47 &
pids+=($!)

sleep 1

ros2 run pfe_raspi_runtime force_sensor_node --ros-args \
  -p publish_rate_hz:=100.0 \
  -p zero_samples:=10 &
pids+=($!)

sleep 1

ros2 run pfe_raspi_runtime camera_mjpeg_server --ros-args \
  -p device:=/dev/video0 \
  -p width:=640 \
  -p height:=480 \
  -p pixelformat:=GB10 \
  -p port:=8081 \
  -p jpeg_quality:=55 \
  -p stream_fps:=20.0 \
  -p stable_gray:=false \
  -p gamma:=1.0 \
  -p contrast:=1.0 \
  -p brightness:=0 \
  -p saturation:=1.0 \
  -p auto_white_balance:=false \
  -p red_gain:=1.0 \
  -p green_gain:=1.0 \
  -p blue_gain:=1.0 \
  -p sharpen:=0.0 &
pids+=($!)

wait -n "${pids[@]}"
