cmake_minimum_required(VERSION 3.8)
project(sllidar_ros2_node)

if(NOT CMAKE_BUILD_TYPE)
	set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)

# LidarModule + ObstacleAvoidance.
# Phase 1: fusion_node chi subscribe /scan de quan sat, KHONG goi ObstacleAvoidance
add_library(lidar_module STATIC
	lidar_node.cpp
	obstacle_avoidance.cpp
)

target_include_directories(lidar_module PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
	$<INSTALL_INTERFACE:include>
)

target_link_libraries(lidar_module PUBLIC ${OpenCV_LIBS})
ament_target_dependencies(lidar_module sensor_msgs)

install(FILES lidar_node.hpp obstacle_avoidance.hpp
	DESTINATION include
)

install(TARGETS lidar_module
	ARCHIVE DESTINATION lib
	LIBRARY DESTINATION lib
	RUNTIME DESTINATION bin
)

ament_export_include_directories(include)
ament_export_libraries(lidar_module)
ament_export_dependencies(OpenCV sensor_msgs)

ament_package()