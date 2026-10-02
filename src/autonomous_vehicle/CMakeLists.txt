cmake_minimum_required(VERSION 3.8)
project(autonomous_vehicle)

if(NOT CMAKE_CXX_STANDARD)
	set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Threads REQUIRED)

add_executable(autonomous_vehicle_node
	src/camera_lane.cpp
	src/fusion_viz_node.cpp
	src/lidar_module.cpp
	src/obstacle_avoidance.cpp
	src/serial_esp32.cpp
)

target_include_directories(autonomous_vehicle_node PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<INSTALL_INTERFACE:include>
	${OpenCV_INCLUDE_DIRS}
)

ament_target_dependencies(autonomous_vehicle_node
	rclcpp
	sensor_msgs
	std_msgs
	cv_bridge
)

target_link_libraries(autonomous_vehicle_node
	${OpenCV_LIBS}
	Threads::Threads
)

install(TARGETS autonomous_vehicle_node
	DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
	DESTINATION include
)
install(DIRECTORY launch/
	DESTINATION share/${PROJECT_NAME}/launch
)
install(DIRECTORY rviz/
	DESTINATION share/${PROJECT_NAME}/rviz
)

# ============================================================================
# TESTS
# ============================================================================
if(BUILD_TESTING)
	find_package(ament_cmake_gtest REQUIRED)

	ament_add_gtest(test_lidar_module
		test/test_lidar_module.cpp
		src/lidar_module.cpp
	)
	target_include_directories(test_lidar_module PRIVATE
		${CMAKE_CURRENT_SOURCE_DIR}/include
		${OpenCV_INCLUDE_DIRS}
	)
	target_link_libraries(test_lidar_module ${OpenCV_LIBS})
	ament_target_dependencies(test_lidar_module sensor_msgs)

	# ------------------------------------------------------------------------
	# Bench đo FPS end-to-end của pipeline lane, CẦN camera thật.
	# Không dùng ament_add_gtest vì phụ thuộc phần cứng, chạy tay:
	#     ros2 run autonomous_vehicle test_camera_lane 20 30
	# (20 = giây, 30 = FPS mục tiêu)
	# ------------------------------------------------------------------------
	add_executable(test_camera_lane
		test/test_camera_lane.cpp
		src/camera_lane.cpp
	)
	target_include_directories(test_camera_lane PRIVATE
		${CMAKE_CURRENT_SOURCE_DIR}/include
		${OpenCV_INCLUDE_DIRS}
	)
	target_link_libraries(test_camera_lane ${OpenCV_LIBS})
	find_package(Threads REQUIRED)
	target_link_libraries(test_camera_lane Threads::Threads)
	install(TARGETS test_camera_lane DESTINATION lib/${PROJECT_NAME})
endif()

ament_package()