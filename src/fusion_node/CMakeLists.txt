cmake_minimum_required(VERSION 3.8)
project(fusion_node)

if(NOT CMAKE_BUILD_TYPE)
	set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED)

# Thu vien detector, driver serial va module lidar
find_package(camera_node REQUIRED)
find_package(esp32s3_node REQUIRED)
find_package(sllidar_ros2_node REQUIRED)

add_executable(fusion_node
	fusion_viz_node.cpp
)

target_include_directories(fusion_node PRIVATE
	${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(fusion_node ${OpenCV_LIBS})

ament_target_dependencies(fusion_node
	rclcpp
	sensor_msgs
	std_msgs
	camera_node
	esp32s3_node
	sllidar_ros2_node
)

install(TARGETS fusion_node
	DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
	DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()