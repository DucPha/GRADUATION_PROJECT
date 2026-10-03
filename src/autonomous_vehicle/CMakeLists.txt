cmake_minimum_required(VERSION 3.8)
project(autonomous_vehicle)

if(NOT CMAKE_CXX_STANDARD)
	set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Threads REQUIRED)

# fusion_viz_node.cpp dùng jsoncpp để dựng JSON trạng thái. Không có
# find_package(jsoncpp) thì các symbol Json::Value::*, Json::FastWriter vẫn
# được compile nhưng không có lib để link -> undefined reference.
find_package(PkgConfig REQUIRED)
pkg_check_modules(JSONCPP REQUIRED IMPORTED_TARGET jsoncpp)

add_executable(autonomous_vehicle_node
	src/camera_lane.cpp
	src/fusion_viz_node.cpp
	src/lane_mask.cpp
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
	builtin_interfaces
	cv_bridge
)

target_link_libraries(autonomous_vehicle_node
	${OpenCV_LIBS}
	Threads::Threads
	PkgConfig::JSONCPP
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

ament_export_include_directories(include)
ament_export_dependencies(rclcpp sensor_msgs std_msgs builtin_interfaces cv_bridge)

# ============================================================================
# TESTS
#
# Khối này phải khai trước ament_package() và phải truyền include của
# package: cả header đều #include <opencv2/opencv.hpp> nên thiếu
# ${CMAKE_CURRENT_SOURCE_DIR}/include là fail ngay ở bước compile.
# ============================================================================
if(BUILD_TESTING)
	find_package(ament_cmake_gtest REQUIRED)

	# lidar_module.hpp kéo theo opencv2/opencv.hpp -> cần cả include và lib.
	ament_add_gtest(test_lidar_module
		test/test_lidar_module.cpp
		src/lidar_module.cpp
	)
	target_include_directories(test_lidar_module PRIVATE
		${CMAKE_CURRENT_SOURCE_DIR}/include
		${OpenCV_INCLUDE_DIRS}
	)
	target_link_libraries(test_lidar_module
		${OpenCV_LIBS}
		Threads::Threads
	)
	ament_target_dependencies(test_lidar_module sensor_msgs)

	# Đây là chạy thử có phần cứng (mở camera thật, in FPS và tỉ lệ lane
	# valid), không phải unit test nên dùng add_executable + main riêng.
	add_executable(test_camera_lane
		test/test_camera_lane.cpp
		src/camera_lane.cpp
		src/lane_mask.cpp
	)
	target_include_directories(test_camera_lane PRIVATE
		${CMAKE_CURRENT_SOURCE_DIR}/include
		${OpenCV_INCLUDE_DIRS}
	)
	target_link_libraries(test_camera_lane
		${OpenCV_LIBS}
		Threads::Threads
	)
	install(TARGETS test_camera_lane
		DESTINATION lib/${PROJECT_NAME}
	)
endif()

ament_package()
