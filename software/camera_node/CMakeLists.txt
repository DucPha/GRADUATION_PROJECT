cmake_minimum_required(VERSION 3.8)
project(camera_node)

if(NOT CMAKE_BUILD_TYPE)
	set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(OpenCV REQUIRED)

# Thu vien detector, khong phu thuoc rclcpp
add_library(camera_lane STATIC
	src/camera_node.cpp
)

target_include_directories(camera_lane PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<INSTALL_INTERFACE:include>
)

target_link_libraries(camera_lane PUBLIC ${OpenCV_LIBS})

install(DIRECTORY include/
	DESTINATION include
)

install(TARGETS camera_lane
	ARCHIVE DESTINATION lib
	LIBRARY DESTINATION lib
	RUNTIME DESTINATION bin
)

ament_export_include_directories(include)
ament_export_libraries(camera_lane)
ament_export_dependencies(OpenCV)

ament_package()