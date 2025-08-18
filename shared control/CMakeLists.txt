cmake_minimum_required (VERSION 3.5)
project(kinova_control)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(include)



add_executable(
	regulation 
	src/regulation.cpp 
	src/manipulator.cpp 
	src/common.cpp
)
ament_target_dependencies(regulation rclcpp sensor_msgs geometry_msgs)



find_package(VISP REQUIRED)

include_directories(${VISP_INCLUDE_DIRS})

add_executable(ibvs src/ibvs.cpp)
ament_target_dependencies(ibvs rclcpp sensor_msgs geometry_msgs)
target_link_libraries(ibvs ${VISP_LIBRARIES})

add_executable(pbvs src/pbvs.cpp)
ament_target_dependencies(pbvs rclcpp sensor_msgs geometry_msgs)
target_link_libraries(pbvs ${VISP_LIBRARIES})

add_executable(hbvs src/hbvs.cpp)
ament_target_dependencies(hbvs rclcpp sensor_msgs geometry_msgs)
target_link_libraries(hbvs ${VISP_LIBRARIES})
