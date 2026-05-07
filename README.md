# Chirurgical Robotic Arm Prototype

## Version status

- v0.6.0: previous stable milestone with the original src architecture, FSR, SG90 servo, perception, and ros2_control experiments.
- v1.0.0: new production-oriented architecture split into PC and RASPI.

## v1 architecture

This version separates the system by machine:

PC:
- v2_description
- v2_moveit_config
- v2_visualization

RASPI:
- v2_description
- v2_control
- v2_sensors

## PC side

The PC runs RViz, MoveIt, the robot description, the unified dashboard markers, the compressed camera display, the FSR level display, and the motor telemetry display.

Main launch:

    cd ~/v2/PC
    source install/setup.bash
    export ROS_DOMAIN_ID=42
    export ROS_LOCALHOST_ONLY=0
    ros2 launch v2_visualization pc_dashboard.launch.py

## Raspberry Pi side

The Raspberry Pi runs the final-position trajectory controller, SG90 servo driver, CSI camera launch through camera_ros, and MCP3202 FSR publisher.

Main control launch:

    cd ~/v2/RASPI
    source install/setup.bash
    export ROS_DOMAIN_ID=42
    export ROS_LOCALHOST_ONLY=0
    ros2 launch v2_control raspi_control.launch.py enable_servo:=false

Real servo mode:

    ros2 launch v2_control raspi_control.launch.py enable_servo:=true

Sensor launch:

    ros2 launch v2_sensors raspi_sensors.launch.py enable_camera:=true enable_fsr:=true

## Important design decision

MoveIt may generate many trajectory points. The Raspberry Pi controller intentionally uses only the final point:

    trajectory.points[-1]

This means the servo receives only the final planned target, not every intermediate MoveIt point.

## Topics

Camera:

    /camera/image_raw/compressed

FSR:

    /fsr/raw
    /fsr/normalized
    /fsr/force_n

Motor:

    /motor/command_angle_deg
    /motor/real_angle_deg
    /motor/error_deg

## Raspberry Pi deployment

The RASPI folder contains:

    install_raspi_dependencies.sh
    check_raspi_ready.sh
    RUN_ON_RASPI.md

Before running on the Pi:

    cd ~/v2/RASPI
    source /opt/ros/humble/setup.bash
    ./install_raspi_dependencies.sh

Then build:

    colcon build --symlink-install
    source install/setup.bash

## Git milestones

- v0.6.0: old architecture completion milestone.
- v1.0.0: PC/RASPI split with unified RViz dashboard and production-ready Raspberry Pi workspace.
- Development continues on v1-dev.
