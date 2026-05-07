# Chirurgical Robotic Arm Prototype

Active development branch:

    v1.1-dev

Stable main milestone:

    v1.0.0

Use v1.1-dev for the next PC-to-Raspberry-Pi integration tests.

## Architecture

PC side:

    PC/src/v2_description
    PC/src/v2_moveit_config
    PC/src/v2_visualization

Raspberry Pi side:

    RASPI/src/v2_description
    RASPI/src/v2_control
    RASPI/src/v2_sensors

## Validated on Raspberry Pi

The Raspberry Pi local system has already been validated:

    RASPI workspace builds.
    camera_ros detects the CSI camera.
    /camera/image_raw/compressed publishes at about 15-16 Hz.
    FSR publishes /fsr/raw, /fsr/normalized, /fsr/force_n.
    final_position_controller accepts FollowJointTrajectory goals.
    The controller uses only the final trajectory point.
    Full Pi launch works with camera + FSR + control together.

## Important design decision

MoveIt may generate many trajectory points.

The Raspberry Pi controller uses only:

    trajectory.points[-1]

So the motor receives only the final planned target.

## Main topics

Camera:

    /camera/image_raw
    /camera/image_raw/compressed
    /camera/camera_info

FSR:

    /fsr/raw
    /fsr/normalized
    /fsr/force_n

Motor:

    /motor/command_angle_deg
    /motor/real_angle_deg
    /motor/error_deg

Joint states:

    /joint_states
    /joint_states_fresh

MoveIt action:

    /arm_controller/follow_joint_trajectory

# Native Ubuntu PC integration guide

Use this guide after booting into the native Ubuntu 22.04 PC.

Do not use WSL for the final ROS networking test. WSL builds the code correctly, but WSL did not discover Raspberry Pi ROS topics over Tailscale.

Assumptions:

    Raspberry Pi IP: 100.112.77.4
    Raspberry Pi user: tbem
    Branch: v1.1-dev
    ROS_DOMAIN_ID: 42
    ROS_LOCALHOST_ONLY: 0

## 1. Clone or update the repo on native Ubuntu PC

Run on the native Ubuntu PC:

    cd ~

    if [ ! -d Chirurgical_Robotic_Arm_Prototype/.git ]; then
      git clone git@github.com:TarekBenElMekki/Chirurgical_Robotic_Arm_Prototype.git
    fi

    cd ~/Chirurgical_Robotic_Arm_Prototype
    git fetch --all --tags --prune
    git switch v1.1-dev
    git pull origin v1.1-dev
    git status

Expected:

    nothing to commit, working tree clean

## 2. Install and build the PC workspace

Run on native Ubuntu PC:

    cd ~/Chirurgical_Robotic_Arm_Prototype/PC
    ./run.sh install
    ./run.sh rebuild
    ./run.sh check

Expected:

    3 packages finished
    v2_visualization dashboard_markers

## 3. Update and check the Raspberry Pi from the PC

Run on native Ubuntu PC:

    ssh tbem@100.112.77.4 'bash -lc "
    cd ~/Chirurgical_Robotic_Arm_Prototype
    git fetch --all --tags --prune
    git switch v1.1-dev
    git pull origin v1.1-dev
    cd RASPI
    ./run.sh rebuild
    ./run.sh check
    "'

Expected:

    camera_ros: OK
    compressed_image_transport: OK
    v2_control: OK
    v2_description: OK
    v2_sensors: OK

## 4. Start the full Raspberry Pi system

Open terminal 1 on native Ubuntu PC:

    ssh tbem@100.112.77.4 'bash -lc "
    cd ~/Chirurgical_Robotic_Arm_Prototype/RASPI
    source /opt/ros/humble/setup.bash
    source install/setup.bash
    export ROS_DOMAIN_ID=42
    export ROS_LOCALHOST_ONLY=0
    ./run.sh full
    "'

Keep this terminal open.

Expected:

    Final position controller started
    Real MCP3202 FSR publisher started
    camera configured with 640x480-RGB888 stream

Non-blocking camera warnings:

    Camera calibration file not found
    unknown control: NoiseReductionMode

## 5. Test ROS topic discovery from native Ubuntu PC

Open terminal 2 on native Ubuntu PC:

    source /opt/ros/humble/setup.bash
    source ~/Chirurgical_Robotic_Arm_Prototype/PC/install/setup.bash

    export ROS_DOMAIN_ID=42
    export ROS_LOCALHOST_ONLY=0

    ros2 topic list | sort

Expected topics:

    /camera/camera_info
    /camera/image_raw
    /camera/image_raw/compressed
    /fsr/force_n
    /fsr/normalized
    /fsr/raw
    /joint_states
    /joint_states_fresh
    /motor/command_angle_deg
    /motor/error_deg
    /motor/real_angle_deg
    /parameter_events
    /rosout

If only these appear:

    /parameter_events
    /rosout

then DDS discovery over Tailscale is failing. The code is not the problem.

## 6. Test FSR from native Ubuntu PC

Run:

    ros2 topic echo /fsr/normalized --once

Expected:

    data: number between 0.0 and 1.0

## 7. Test compressed camera from native Ubuntu PC

Run:

    ros2 topic hz /camera/image_raw/compressed

Expected:

    average rate: around 15 Hz

Press Ctrl+C after several lines.

## 8. Test PC-to-Pi action command

Run:

    ros2 action send_goal /arm_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{
      trajectory: {
        joint_names: ['base_yaw_joint'],
        points: [
          {positions: [0.0], time_from_start: {sec: 1, nanosec: 0}},
          {positions: [0.5], time_from_start: {sec: 2, nanosec: 0}}
        ]
      }
    }"

Expected on PC:

    Goal accepted
    Goal finished with status: SUCCEEDED
    error_string: Final point accepted

Expected on Pi terminal:

    Received MoveIt trajectory with 2 points. Used final point only.

## 9. Launch RViz dashboard

Open terminal 3 on native Ubuntu PC:

    cd ~/Chirurgical_Robotic_Arm_Prototype/PC

    source /opt/ros/humble/setup.bash
    source install/setup.bash

    export ROS_DOMAIN_ID=42
    export ROS_LOCALHOST_ONLY=0

    ./run.sh start

Expected:

    You can start planning now!
    Dashboard markers started

In RViz, check:

    Robot model visible.
    MotionPlanning panel loaded.
    Camera display uses /camera/image_raw/compressed.
    Dashboard markers visible.
    FSR level updates.
    Motor command and real angle text visible.

## 10. RViz MoveIt execution test

In RViz:

    Select planning group: arm
    Set a valid goal.
    Click Plan.
    Click Execute.

Expected on Pi terminal:

    Received MoveIt trajectory with N points. Used final point only.

## 11. Cleanup after tests

Run from native Ubuntu PC:

    ssh tbem@100.112.77.4 'bash -lc "
    ros2 daemon stop 2>/dev/null || true
    pkill -f \"ros2 launch\" 2>/dev/null || true
    pkill -f \"final_position_controller\" 2>/dev/null || true
    pkill -f \"servo_driver\" 2>/dev/null || true
    pkill -f \"fsr_publisher\" 2>/dev/null || true
    pkill -f \"camera_node\" 2>/dev/null || true
    sudo fuser -k /dev/video0 /dev/media0 /dev/media1 2>/dev/null || true
    sudo fuser -k /dev/gpiochip0 /dev/spidev0.0 2>/dev/null || true
    ros2 daemon start
    echo Pi cleanup done.
    "'

Stop PC RViz with Ctrl+C.
