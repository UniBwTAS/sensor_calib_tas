cmake_minimum_required(VERSION 3.0.2)
project(lidar_reference_point_detection)

add_compile_options(-std=c++17)

find_package(catkin_simple REQUIRED)

catkin_simple()

find_package(Ceres)

include_directories(
    ${catkin_INCLUDE_DIRS}
    include
)

cs_add_executable(hole_pattern_detector
    src/hole_pattern_detector/hole_pattern_detector_node.cpp
    src/hole_pattern_detector/hole_pattern_detector.cpp
    )
add_dependencies(hole_pattern_detector ${PROJECT_NAME}_gencfg)
target_link_libraries(hole_pattern_detector ceres)

cs_install()
cs_export()
