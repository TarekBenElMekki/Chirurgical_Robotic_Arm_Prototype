#!/usr/bin/env bash

echo "== Raspberry Pi readiness check =="

echo ""
echo "[1] OS:"
uname -a

echo ""
echo "[2] ROS:"
if command -v ros2 >/dev/null; then
  ros2 --help >/dev/null && echo "ros2 command: OK"
else
  echo "ros2 command: MISSING"
fi

echo "ROS_DISTRO=${ROS_DISTRO:-NOT_SET}"

echo ""
echo "[3] Camera devices:"
ls -l /dev/video* 2>/dev/null || echo "No /dev/video* devices found"

echo ""
echo "[4] SPI devices:"
ls -l /dev/spidev* 2>/dev/null || echo "No /dev/spidev* devices found"

echo ""
echo "[5] Python modules:"
python3 - <<'PY'
modules = ["lgpio", "spidev"]
for m in modules:
    try:
        __import__(m)
        print(f"{m}: OK")
    except Exception as exc:
        print(f"{m}: MISSING ({exc})")
PY

echo ""
echo "[6] ROS packages:"
for pkg in camera_ros compressed_image_transport; do
  if ros2 pkg prefix "$pkg" >/dev/null 2>&1; then
    echo "$pkg: OK"
  else
    echo "$pkg: MISSING"
  fi
done

echo ""
echo "[7] v2 packages:"
for pkg in v2_control v2_description v2_sensors; do
  if ros2 pkg prefix "$pkg" >/dev/null 2>&1; then
    echo "$pkg: OK"
  else
    echo "$pkg: not built/sourced yet"
  fi
done

echo ""
echo "Readiness check complete."
