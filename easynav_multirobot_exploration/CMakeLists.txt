cmake_minimum_required(VERSION 3.20)
project(easynav_multirobot_exploration)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(easynav_system REQUIRED)

add_library(choose_frontier_goal_bt_node SHARED src/${PROJECT_NAME}/ChooseFrontierGoal.cpp)
add_library(is_goal_valid_bt_node SHARED src/${PROJECT_NAME}/IsGoalValid.cpp)
add_library(go_to_pose_bt_node SHARED src/${PROJECT_NAME}/GoToPose.cpp)
add_library(get_exploration_data_bt_node SHARED src/${PROJECT_NAME}/GetExplorationData.cpp)
add_library(is_explored SHARED src/${PROJECT_NAME}/IsExplored.cpp)

set(plugin_libs
  choose_frontier_goal_bt_node
  is_goal_valid_bt_node
  go_to_pose_bt_node
  get_exploration_data_bt_node
  is_explored
)

# Libraries
foreach(bt_plugin ${plugin_libs})
  target_include_directories(${bt_plugin} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${behaviortree_cpp_INCLUDE_DIRS}
  )
  target_link_libraries(${bt_plugin} PUBLIC
    rclcpp::rclcpp
    behaviortree_cpp::behaviortree_cpp
    easynav_system::easynav_system
    tf2::tf2
    tf2_ros::tf2_ros
    ${nav_msgs_TARGETS}
    ${visualization_msgs_TARGETS}
    ${geometry_msgs_TARGETS}
    ${tf2_geometry_msgs_TARGETS}
  )
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

add_library(${PROJECT_NAME} SHARED src/${PROJECT_NAME}/utils/MapDataSaver.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}
  rclcpp::rclcpp
  ${nav_msgs_TARGETS}
)

# Executables
add_executable(explorer src/explorer.cpp)
target_link_libraries(explorer PUBLIC ${plugin_libs})

add_executable(save_map_data src/save_map_data.cpp)
target_link_libraries(save_map_data ${PROJECT_NAME})

# Install
install(TARGETS
  ${plugin_libs}
  ${PROJECT_NAME}
  explorer
  save_map_data
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY
  config
  launch
  rviz
  behavior_trees
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()