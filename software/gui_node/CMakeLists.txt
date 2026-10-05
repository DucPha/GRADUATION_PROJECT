cmake_minimum_required(VERSION 3.16)
project(autonomous_vehicle_gui)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(JSONCPP REQUIRED IMPORTED_TARGET jsoncpp)

# Qt5 chỉ dùng Core/Gui/Widgets. Không kéo QtCharts hay Qwt: các trend plot
# được vẽ bằng QPainter, nhanh hơn Qwt và không thêm dependency.
find_package(Qt5 REQUIRED COMPONENTS Core Gui Widgets)

add_executable(autonomous_vehicle_gui_node
  src/dashboard_main.cpp
  src/dashboard_node.cpp
  src/dashboard_window.cpp
  src/widgets/lidar_map_widget.cpp
  src/widgets/image_panel.cpp
  src/widgets/trend_plot.cpp
  src/widgets/node_link_table.cpp
  src/widgets/param_panel.cpp
  src/widgets/header_bar.cpp
  include/dashboard_node.hpp
  include/dashboard_window.hpp
  include/theme.hpp
  include/widgets/header_bar.hpp
  include/widgets/image_panel.hpp
  include/widgets/lidar_map_widget.hpp
  include/widgets/node_link_table.hpp
  include/widgets/param_panel.hpp
  include/widgets/trend_plot.hpp
)

target_include_directories(autonomous_vehicle_gui_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${JSONCPP_INCLUDE_DIRS}
)

target_link_libraries(autonomous_vehicle_gui_node
  PkgConfig::JSONCPP
  Qt5::Core
  Qt5::Gui
  Qt5::Widgets
)

ament_target_dependencies(autonomous_vehicle_gui_node
  rclcpp
  sensor_msgs
  std_msgs
  builtin_interfaces
)

install(TARGETS autonomous_vehicle_gui_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_dependencies(rclcpp sensor_msgs std_msgs builtin_interfaces)
ament_package()