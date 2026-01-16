cmake_minimum_required(VERSION 3.5)
project(maze_world)

find_package(ament_cmake_ros REQUIRED)

install(DIRECTORY worlds models maps
    DESTINATION share/${PROJECT_NAME}
)

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/maze_world.dsv.in")

ament_export_dependencies(gazebo_ros)

ament_package()

