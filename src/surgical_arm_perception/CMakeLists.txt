cmake_minimum_required(VERSION 3.8)
project(surgical_arm_perception)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config rviz
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/fsr_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
