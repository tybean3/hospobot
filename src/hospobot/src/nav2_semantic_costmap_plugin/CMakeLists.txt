cmake_minimum_required(VERSION 3.8)
project(nav2_semantic_costmap_plugin)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)
find_package(hospobot_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_util REQUIRED)

add_library(nav2_semantic_costmap_plugin SHARED
  src/semantic_layer.cpp
)

target_include_directories(nav2_semantic_costmap_plugin
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(nav2_semantic_costmap_plugin
  rclcpp
  nav2_costmap_2d
  pluginlib
  hospobot_interfaces
  geometry_msgs
  nav2_util
)

pluginlib_export_plugin_description_file(nav2_costmap_2d plugins.xml)

install(TARGETS nav2_semantic_costmap_plugin
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(nav2_semantic_costmap_plugin)
ament_export_dependencies(
  rclcpp
  nav2_costmap_2d
  pluginlib
  hospobot_interfaces
  geometry_msgs
  nav2_util
)

ament_package()
