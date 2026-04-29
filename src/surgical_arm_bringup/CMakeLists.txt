cmake_minimum_required(VERSION 3.8)
project(surgical_arm_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config urdf rviz
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/status_monitor.py
  scripts/trajectory_gateway.py
  scripts/pwm_mapper.py
  scripts/joint_state_freshener.py
  scripts/change_monitor.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
