cmake_minimum_required(VERSION 3.8)
project(esp32s3_node)

if(NOT CMAKE_BUILD_TYPE)
	set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)

# Thu vien giao tiep serial toi ESP32, khong phu thuoc rclcpp
add_library(esp32s3_serial STATIC
	src/esp32s3_node.cpp
)

target_include_directories(esp32s3_serial PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<INSTALL_INTERFACE:include>
)

install(DIRECTORY include/
	DESTINATION include
)

install(TARGETS esp32s3_serial
	ARCHIVE DESTINATION lib
	LIBRARY DESTINATION lib
	RUNTIME DESTINATION bin
)

ament_export_include_directories(include)
ament_export_libraries(esp32s3_serial)

ament_package()