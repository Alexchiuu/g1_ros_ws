cmake_minimum_required(VERSION 3.8)
project(g1_gazebo)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config worlds rviz
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS scripts/position_bridge.py scripts/base_tf_broadcaster.py scripts/overlap_monitor.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
