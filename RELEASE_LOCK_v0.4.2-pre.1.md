# Chirurgical Robotic Arm Prototype v0.4.2-pre.1

First pre-release lock.

Locked features:
- Correct RViz autonomous planning interface with 2D Workspace Matrix.
- Publish Point safety guard is kept.
- Publish Point flow: /clicked_point -> rviz_clicked_point_mode_gate_node -> /mode_guarded/clicked_point -> workspace_2d_interactive_node.
- User selects MOVEIT_PLAN mode to allow RViz Publish Point goals.
- Raspberry Pi OV5647 camera is streamed as MJPEG.
- Stable camera profile: 640x480, RAW Bayer GB10, color enabled, target 20 FPS, JPEG quality 55.
- PC bridge republishes camera to /camera/image_raw for RViz.
- FSR sensor publishes /force/newton.
- RViz bridge publishes /force/rviz_marker.
- HMI depth ladder removed.
- RViz is the main autonomous planning visualization interface.

Runtime order:
1. On Raspberry Pi:
   ~/pfe_raspi_ws/run_raspi_runtime.sh

2. On PC WSL:
   source /opt/ros/humble/setup.bash
   source /home/master_thesis/PFE_Surgical_Arm_Portable_V0_4_2_MANUAL_RVIZ_SYNC_DEV/Runtime/WSL/ros2_net.sh
   source /home/master_thesis/PFE_Surgical_Arm_Portable_V0_4_2_MANUAL_RVIZ_SYNC_DEV/Workspaces/PC_WSL/surgical_arm_ws/install/setup.bash
   ros2 launch surgical_arm_mouse_control wsl_moveit_rviz_only.launch.py

Expected topics:
- /camera/image_raw
- /force/newton
- /force/rviz_marker
- /surgical_arm/workspace_matrix
- /clicked_point
- /mode_guarded/clicked_point
