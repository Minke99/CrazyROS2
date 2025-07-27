cmake_minimum_required(VERSION 3.5)
project(cf_ctrl)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
...

install(PROGRAMS
  scripts/joystick_node.py
  scripts/mocap_quad_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()

