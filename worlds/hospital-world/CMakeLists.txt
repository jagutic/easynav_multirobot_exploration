cmake_minimum_required(VERSION 3.5)
project(hospital_world)

find_package(ament_cmake_ros REQUIRED)

install(DIRECTORY worlds models maps photos
    DESTINATION share/${PROJECT_NAME}
)

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/hospital_world.dsv.in")

ament_export_dependencies(gazebo_ros)

ament_package()

