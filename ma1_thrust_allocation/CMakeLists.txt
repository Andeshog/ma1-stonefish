cmake_minimum_required(VERSION 3.8)
project(ma1_thrust_allocation)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  ${PROJECT_NAME}/ma1_thrust_allocation_node.py
  ${PROJECT_NAME}/ma1_actuators.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
