cmake_minimum_required(VERSION 3.0.2)
project(radar_target_detection)

add_compile_options(-std=c++17)

find_package(catkin_simple REQUIRED)

catkin_simple()

include_directories(
    ${catkin_INCLUDE_DIRS}
    include
)

cs_add_executable(doppler_simulator
    src/doppler_simulator/doppler_simulator_node.cpp
    src/doppler_simulator/doppler_simulator.cpp
    )
add_dependencies(doppler_simulator ${PROJECT_NAME}_gencfg)

cs_install()
cs_export()
