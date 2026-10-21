cmake_minimum_required(VERSION 3.5)
project(car_base_v2)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(origincar_msg REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(serial REQUIRED)

include_directories(
  include
)

add_executable(car_base_v2_node
  src/main.cpp
  src/car_base_v2_node.cpp
  src/frame_parser.cpp
)
ament_target_dependencies(car_base_v2_node
  rclcpp
  sensor_msgs
  geometry_msgs
  origincar_msg
  diagnostic_msgs
  serial
)

install(TARGETS
  car_base_v2_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
