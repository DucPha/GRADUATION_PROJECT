cmake_minimum_required(VERSION 3.8)
project(fusion_node)

# Default to Release if not specified
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED)

# Local package dependencies (exported via ament)
find_package(camera_node REQUIRED)
find_package(esp32s3_node REQUIRED)
find_package(sllidar_ros2_node REQUIRED)

add_executable(fusion_node
  fusion_viz_node.cpp
)

# fusion_viz_node.cpp include opencv2/core/mat.hpp -> can truc tiep OpenCV_INCLUDE_DIRS
target_include_directories(fusion_node PRIVATE
  ${OpenCV_INCLUDE_DIRS}
)

# ROS 2 dependencies + 3 thu vien local (camera_node / esp32s3_node / sllidar_ros2_node
# export duoc lib qua ament_export_libraries, ament_target_dependencies se resolve duong dan tuyet doi)
ament_target_dependencies(fusion_node
  rclcpp
  sensor_msgs
  std_msgs
  camera_node
  esp32s3_node
  sllidar_ros2_node
)

# OpenCV: PHAI dung plain signature (khong PUBLIC/PRIVATE) vi ament_target_dependencies
# phia tren dung plain signature cho target_link_libraries - CMake cam tron 2 signature.
# ${OpenCV_LIBS} = imported target opencv_* (khong ton tai OpenCV::OpenCV tren OpenCV 4.6).
target_link_libraries(fusion_node ${OpenCV_LIBS})

# Export dependencies for downstream packages (if any)
ament_export_dependencies(rclcpp sensor_msgs std_msgs camera_node esp32s3_node sllidar_ros2_node OpenCV)

install(TARGETS fusion_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()