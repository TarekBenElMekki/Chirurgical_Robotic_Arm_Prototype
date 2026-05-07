#!/usr/bin/env bash
set -e

echo "== Raspberry Pi dependency installer =="

if [ -z "$ROS_DISTRO" ]; then
  echo "ROS_DISTRO is not set."
  echo "Run: source /opt/ros/humble/setup.bash"
  exit 1
fi

sudo apt update

sudo apt install -y \
  python3-lgpio \
  python3-spidev \
  v4l-utils \
  ros-${ROS_DISTRO}-camera-ros \
  ros-${ROS_DISTRO}-compressed-image-transport

echo "Adding user to gpio/video/spi groups if they exist..."
for group in gpio video spi; do
  if getent group "$group" >/dev/null; then
    sudo usermod -aG "$group" "$USER"
    echo "Added $USER to $group"
  else
    echo "Group $group does not exist, skipping"
  fi
done

echo ""
echo "Enable SPI if not already enabled:"
echo "  sudo raspi-config"
echo "  Interface Options -> SPI -> Enable"
echo ""
echo "If group membership changed, reboot:"
echo "  sudo reboot"
echo ""
echo "Done."
