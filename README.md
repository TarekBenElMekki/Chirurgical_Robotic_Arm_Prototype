# 🦾 Chirurgical Robotic Arm Prototype (v0.1)

## 📌 Overview

This project implements a **distributed robotic system** using:

- 🖥️ PC → Motion planning (MoveIt 2 + RViz)
- 🍓 Raspberry Pi → Execution (ros2_control)
- 🔗 Custom gateway → Robust trajectory handling
- 📊 Monitoring → Real-time state observation

The system allows a user to:
- Plan motion in RViz
- Execute trajectories
- Monitor execution in real time

---

## 🧠 Architecture


RViz (PC)
↓
MoveIt 2
↓
/arm_controller/follow_joint_trajectory
↓
Trajectory Gateway (Raspberry Pi)
↓
/hardware_arm_controller/follow_joint_trajectory
↓
ros2_control
↓
Joint states feedback


---

## ⚙️ Features

- ✅ Full MoveIt → ros2_control pipeline
- ✅ Robust trajectory retiming (gateway node)
- ✅ Real-time feedback monitoring
- ✅ Dual RViz setup (control + monitoring)
- ✅ Clean debugging (only changes displayed)
- ✅ One-launch-per-machine architecture

---

## 🚀 How to Run (Linux)

### 🔵 Raspberry Pi side

```bash
source ~/surgical_arm_ws/install/setup.bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0

ros2 launch surgical_arm_bringup pi_robot.launch.py use_rviz:=true
🟢 PC side
source ~/surgical_arm_ws/install/setup.bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0

ros2 launch surgical_arm_bringup pc_moveit.launch.py
🟡 Debug (change monitor)
ros2 run surgical_arm_bringup change_monitor.py
🪟 One-Click Windows Launch

You can launch the full system using a single .bat file.

📄 run_system.bat
@echo off

echo Starting ROS2 Surgical Arm System...

REM === RASPI SIDE ===
start cmd /k wsl -e bash -lc "source ~/surgical_arm_ws/install/setup.bash && export ROS_DOMAIN_ID=42 && export ROS_LOCALHOST_ONLY=0 && ros2 launch surgical_arm_bringup pi_robot.launch.py use_rviz:=true"

REM === PC SIDE ===
start cmd /k wsl -e bash -lc "source ~/surgical_arm_ws/install/setup.bash && export ROS_DOMAIN_ID=42 && export ROS_LOCALHOST_ONLY=0 && ros2 launch surgical_arm_bringup pc_moveit.launch.py"

REM === DEBUG (CHANGE MONITOR) ===
start cmd /k wsl -e bash -lc "source ~/surgical_arm_ws/install/setup.bash && export ROS_DOMAIN_ID=42 && export ROS_LOCALHOST_ONLY=0 && ros2 run surgical_arm_bringup change_monitor.py"

echo System launched.
pause
▶️ Usage
Save as run_system.bat
Double-click
System starts automatically:
Raspberry Pi execution
PC MoveIt control
Debug monitor
🔍 Monitoring

The system includes a custom node:

change_monitor.py

It prints ONLY when values change:

Joint positions
Desired controller values
Actual controller values

This avoids noisy logs and improves debugging.

🔧 Custom Components
🔁 Trajectory Gateway
Fixes invalid MoveIt trajectories
Ensures time consistency
Prevents controller rejection
📡 Joint State Freshener
Updates timestamps
Ensures MoveIt receives valid state
📊 Status Monitor
Publishes execution status
Helps track received commands
📁 Project Structure
surgical_arm_ws/
└── src/
    └── surgical_arm_bringup/
        ├── launch/
        │   ├── pc_moveit.launch.py
        │   └── pi_robot.launch.py
        ├── config/
        ├── scripts/
        │   ├── trajectory_gateway.py
        │   ├── change_monitor.py
        │   ├── joint_state_freshener.py
        │   └── status_monitor.py
        └── urdf/
⚠️ Known Limitations
❗ URDF lacks collision geometry (visual only)
❗ Uses mock hardware (no real motors yet)
❗ Camera may not work on WSL (/dev/video0)
🔜 Next Steps
🔧 Integrate real motor drivers (GPIO / PWM)
📐 Add collision geometry to URDF
📊 Add logging & performance analysis
🤖 Improve control precision
👨‍🔬 Author

Tarek Ben El Mekki
Master Thesis – Surgical Robotics System

📌 Version
v0.1 – Stable execution + monitoring system

---

# 🚀 Final step (push it)

```bash
git add README.md
git commit -m "v0.1: full professional README with architecture and launcher"
git push
🧠 Result

Your repo now looks like:

✔ Professional
✔ Understandable
✔ Runnable in one click
✔ Thesis-quality documentation
