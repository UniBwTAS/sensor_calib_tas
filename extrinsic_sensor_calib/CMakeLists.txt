cmake_minimum_required(VERSION 2.8.3)
project(extrinsic_sensor_calib)

add_compile_options(-std=c++17)

find_package(catkin_simple REQUIRED)

catkin_simple()

find_package(Ceres 1.14.0 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Boost COMPONENTS serialization REQUIRED)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIR}
)

cs_add_executable(extrinsic_sensor_calib
    src/extrinsic_sensor_calib/sensor_calib_node.cpp
    src/extrinsic_sensor_calib/sensor_calib.cpp
    src/extrinsic_sensor_calib/impl/calibrate.cpp
    src/extrinsic_sensor_calib/impl/helper.cpp
    src/extrinsic_sensor_calib/impl/init.cpp
    src/extrinsic_sensor_calib/impl/input.cpp
    src/extrinsic_sensor_calib/impl/output.cpp
    )
add_dependencies(extrinsic_sensor_calib ${PROJECT_NAME}_gencfg)

target_link_libraries(extrinsic_sensor_calib
    ceres
    glog
    lapack
    blas
    cxsparse
    cholmod
    umfpack
    ${SUITESPARSE_LIBRARIES}
    gflags
    ${OpenCV_LIBRARIES}
    ${Boost_LIBRARIES}
    )
