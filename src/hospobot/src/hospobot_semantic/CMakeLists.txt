cmake_minimum_required(VERSION 3.8)
project(hospobot_semantic)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(hospobot_interfaces REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Costmap Plugin
add_library(dense_semantic_layer SHARED src/dense_semantic_layer.cpp)
target_include_directories(dense_semantic_layer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(dense_semantic_layer
  rclcpp nav2_costmap_2d pluginlib nav_msgs)

# BT Nodes
add_library(deadlock_detector_condition SHARED src/deadlock_detector_condition.cpp)
target_include_directories(deadlock_detector_condition PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(deadlock_detector_condition
  rclcpp behaviortree_cpp geometry_msgs)

add_library(vlm_semantic_intervention_action SHARED src/vlm_semantic_intervention_action.cpp)
target_include_directories(vlm_semantic_intervention_action PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(vlm_semantic_intervention_action
  rclcpp behaviortree_cpp sensor_msgs geometry_msgs hospobot_interfaces)

install(TARGETS
  dense_semantic_layer
  deadlock_detector_condition
  vlm_semantic_intervention_action
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY include/ behavior_trees/
  DESTINATION include)

install(DIRECTORY behavior_trees/
  DESTINATION share/${PROJECT_NAME}/behavior_trees)

install(PROGRAMS
  scripts/semantic_costmap_publisher.py
  scripts/vlm_intervention_server.py
  DESTINATION lib/${PROJECT_NAME}
)

pluginlib_export_plugin_description_file(nav2_costmap_2d plugins.xml)

ament_export_libraries(
  dense_semantic_layer
  deadlock_detector_condition
  vlm_semantic_intervention_action)
ament_export_include_directories(include)

ament_package()
