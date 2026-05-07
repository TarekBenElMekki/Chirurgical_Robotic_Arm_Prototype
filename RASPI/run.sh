#!/usr/bin/env bash
set -eo pipefail

cd "$(dirname "$0")"

ROS_DISTRO_NAME="${ROS_DISTRO:-humble}"
ROS_SETUP="/opt/ros/${ROS_DISTRO_NAME}/setup.bash"

source_ros() {
  if [ -f "$ROS_SETUP" ]; then
    source "$ROS_SETUP"
  else
    echo "ERROR: ROS setup not found: $ROS_SETUP"
    echo "Install ROS 2 Humble or set ROS_DISTRO correctly."
    exit 1
  fi
}

source_workspace() {
  source_ros
  if [ -f "install/setup.bash" ]; then
    source install/setup.bash
  else
    echo "Workspace not built yet. Run: ./run.sh build"
    exit 1
  fi
}

install_deps() {
  source_ros
  chmod +x install_raspi_dependencies.sh check_raspi_ready.sh
  ./install_raspi_dependencies.sh
}

build_ws() {
  source_ros
  colcon build --symlink-install
}

clean_ws() {
  rm -rf build install log
}

check_ws() {
  source_workspace
  chmod +x check_raspi_ready.sh
  ./check_raspi_ready.sh
  echo ""
  echo "== RASPI package executables =="
  ros2 pkg executables v2_control || true
  ros2 pkg executables v2_sensors || true
  echo ""
  echo "== RASPI launch arguments =="
  ros2 launch v2_control raspi_control.launch.py --show-args
  ros2 launch v2_sensors raspi_sensors.launch.py --show-args
}

control_no_servo() {
  source_workspace
  export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
  export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
  ros2 launch v2_control raspi_control.launch.py enable_servo:=false
}

control_with_servo() {
  source_workspace
  export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
  export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
  ros2 launch v2_control raspi_control.launch.py enable_servo:=true
}

sensors() {
  source_workspace
  export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
  export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
  ros2 launch v2_sensors raspi_sensors.launch.py enable_camera:=true enable_fsr:=true
}

full_no_servo() {
  source_workspace
  export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
  export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"

  ros2 launch v2_control raspi_control.launch.py enable_servo:=false &
  CONTROL_PID=$!

  ros2 launch v2_sensors raspi_sensors.launch.py enable_camera:=true enable_fsr:=true &
  SENSORS_PID=$!

  trap 'kill $CONTROL_PID $SENSORS_PID 2>/dev/null || true' EXIT INT TERM
  wait
}

case "${1:-help}" in
  install)
    install_deps
    ;;
  build)
    build_ws
    ;;
  rebuild)
    clean_ws
    build_ws
    ;;
  clean)
    clean_ws
    ;;
  check)
    check_ws
    ;;
  control)
    control_no_servo
    ;;
  control-servo)
    control_with_servo
    ;;
  sensors)
    sensors
    ;;
  start|full)
    full_no_servo
    ;;
  help|*)
    echo "Usage: ./run.sh {install|build|rebuild|clean|check|control|control-servo|sensors|start|full}"
    ;;
esac
