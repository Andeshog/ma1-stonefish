cmake_minimum_required(VERSION 3.8)
project(ma1_qp)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OsqpEigen REQUIRED)

include_directories(include)

add_executable(ma1_qp 
  src/thrust_allocator_node.cpp
  src/thrust_allocator.cpp
)

ament_target_dependencies(ma1_qp rclcpp geometry_msgs sensor_msgs Eigen3 OsqpEigen)

target_link_libraries(ma1_qp OsqpEigen::OsqpEigen)

install(TARGETS
  ma1_qp
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
