cmake_minimum_required(VERSION 3.8)
project(camera_node)

# Default to Release if not specified
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(OpenCV REQUIRED)
# libjpeg-turbo: tu giai ma MJPG o 1/2 kich thuoc (nhanh gap doi, im canh bao)
find_package(JPEG REQUIRED)

# Thu vien detector, khong phu thuoc rclcpp
add_library(camera_lane STATIC
  camera_node.cpp
)

# OpenCV_INCLUDE_DIRS bat buoc: OpenCV 4.6 tren Ubuntu khong export include tu ${OpenCV_LIBS}
target_include_directories(camera_lane PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
)

# ${OpenCV_LIBS} = danh sach imported target opencv_core/opencv_imgproc... (khong phai OpenCV::OpenCV - target nay khong ton tai)
target_link_libraries(camera_lane PUBLIC ${OpenCV_LIBS} JPEG::JPEG)

# Export dependencies for downstream packages
ament_export_include_directories(include)
ament_export_libraries(camera_lane)
ament_export_dependencies(OpenCV JPEG)

install(FILES camera_node.hpp
  DESTINATION include
)

install(TARGETS camera_lane
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_package()