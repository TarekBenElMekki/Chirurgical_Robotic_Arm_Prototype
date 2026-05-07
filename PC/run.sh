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
  sudo apt update
  sudo apt install -y \
    python3-colcon-common-extensions \
    ros-${ROS_DISTRO_NAME}-xacro \
    ros-${ROS_DISTRO_NAME}-robot-state-publisher \
    ros-${ROS_DISTRO_NAME}-rviz2 \
    ros-${ROS_DISTRO_NAME}-moveit \
    ros-${ROS_DISTRO_NAME}-compressed-image-transport
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
  echo "== PC package executables =="
  ros2 pkg executables v2_visualization || true
  echo ""
  echo "== PC launch arguments =="
  ros2 launch v2_visualization pc_dashboard.launch.py --show-args
}

start_ws() {
  source_workspace
  export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
  export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
  ros2 launch v2_visualization pc_dashboard.launch.py
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
  start)
    start_ws
    ;;
  help|*)
    echo "Usage: ./run.sh {install|build|rebuild|clean|check|start}"
    ;;
esac
