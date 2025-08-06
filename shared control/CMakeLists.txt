cmake_minimum_required (VERSION 3.5)
project(kinova_control)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(regulation src/regulation.cpp)
ament_target_dependencies(regulation rclcpp sensor_msgs geometry_msgs)

find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(ibvs src/ibvs.cpp)
ament_target_dependencies(ibvs rclcpp sensor_msgs geometry_msgs cv_bridge OpenCV)
