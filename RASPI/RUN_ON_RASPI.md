# V2 Raspberry Pi Runtime Guide

## 1. Install dependencies

Run:

    cd ~/v2/RASPI
    source /opt/ros/humble/setup.bash
    chmod +x install_raspi_dependencies.sh check_raspi_ready.sh
    ./install_raspi_dependencies.sh

If groups were changed, reboot:

    sudo reboot

## 2. Build

Run:

    cd ~/v2/RASPI
    source /opt/ros/humble/setup.bash
    colcon build --symlink-install
    source install/setup.bash

## 3. Check readiness

Run:

    cd ~/v2/RASPI
    source /opt/ros/humble/setup.bash
    source install/setup.bash
    ./check_raspi_ready.sh

## 4. Network-only control test

Run:

    export ROS_DOMAIN_ID=42
    export ROS_LOCALHOST_ONLY=0
    ros2 launch v2_control raspi_control.launch.py enable_servo:=false

## 5. Servo test

Use external 5V power and common GND.

Run:

    ros2 launch v2_control raspi_control.launch.py enable_servo:=true

In another terminal:

    ros2 topic pub --once /motor/command_angle_deg std_msgs/msg/Float32 "{data: 90.0}"
    ros2 topic pub --once /motor/command_angle_deg std_msgs/msg/Float32 "{data: 60.0}"
    ros2 topic pub --once /motor/command_angle_deg std_msgs/msg/Float32 "{data: 120.0}"

## 6. Sensor test

Run:

    ros2 launch v2_sensors raspi_sensors.launch.py enable_camera:=true enable_fsr:=true

Camera topic:

    /camera/image_raw/compressed

FSR topics:

    /fsr/raw
    /fsr/normalized
    /fsr/force_n

## 7. Full system

Pi terminal 1:

    ros2 launch v2_control raspi_control.launch.py enable_servo:=true

Pi terminal 2:

    ros2 launch v2_sensors raspi_sensors.launch.py enable_camera:=true enable_fsr:=true

PC terminal:

    cd ~/v2/PC
    source install/setup.bash
    export ROS_DOMAIN_ID=42
    export ROS_LOCALHOST_ONLY=0
    ros2 launch v2_visualization pc_dashboard.launch.py
