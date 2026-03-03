cmake_minimum_required(VERSION 3.20)
project(easynav_frontier_maps_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR}/cmake_modules)

find_package(ament_cmake REQUIRED)
find_package(easynav_common REQUIRED)
find_package(easynav_core REQUIRED)
find_package(pluginlib REQUIRED)
find_package(yaets REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(easynav_costmap_common REQUIRED)
find_package(OpenCV REQUIRED)
find_package(visualization_msgs REQUIRED)


add_library(${PROJECT_NAME} SHARED
  src/easynav_frontier_maps_manager/FrontierMapsManager.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME} PUBLIC
  easynav_common::easynav_common
  easynav_core::easynav_core
  easynav_costmap_common::easynav_costmap_common
  pluginlib::pluginlib
  yaets::yaets
  ${nav_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${visualization_msgs_TARGETS}
  ${OpenCV_LIBS}
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(TARGETS
  ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   set(ament_cmake_copyright_FOUND TRUE)
#   set(ament_cmake_cpplint_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()

#   find_package(ament_cmake_gtest REQUIRED)
#   add_subdirectory(tests)
# endif()

ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME})

# Register the plugin
pluginlib_export_plugin_description_file(easynav_core easynav_frontier_maps_manager_plugins.xml)

ament_export_dependencies(
  easynav_common
  easynav_core
  pluginlib
  nav_msgs
  yaets
  geometry_msgs
  nav_msgs
  easynav_costmap_common
  OpenCV
  visualization_msgs
)
ament_package()
