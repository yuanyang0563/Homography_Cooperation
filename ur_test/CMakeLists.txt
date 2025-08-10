cmake_minimum_required (VERSION 3.10)
project(ur3_control)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(regulation src/regulation.cpp)
ament_target_dependencies(regulation rclcpp sensor_msgs std_msgs)
