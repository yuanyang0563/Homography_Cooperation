cmake_minimum_required (VERSION 3.10)
project(ur3_control)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include)

add_executable(control src/control.cpp src/manipulator.cpp src/common.cpp)
ament_target_dependencies(control rclcpp sensor_msgs std_msgs)
