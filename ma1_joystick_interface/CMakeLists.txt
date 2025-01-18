cmake_minimum_required(VERSION 3.8)
project(ma1_joystick_interface)

find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  ${PROJECT_NAME}/${PROJECT_NAME}_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
