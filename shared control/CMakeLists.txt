cmake_minimum_required (VERSION 3.10)
project(kinova_control)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(VISP REQUIRED)

include_directories(include)
include_directories(${VISP_INCLUDE_DIRS})

add_executable(control src/control.cpp src/manipulator.cpp src/common.cpp)
ament_target_dependencies(control rclcpp sensor_msgs geometry_msgs)
target_link_libraries(control ${VISP_LIBRARIES})
