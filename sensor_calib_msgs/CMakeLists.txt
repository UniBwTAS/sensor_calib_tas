cmake_minimum_required(VERSION 2.8.3)
project(sensor_calib_msgs)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  geometry_msgs
  message_generation
)

add_message_files(
  FILES
  CartesianPointMeasurement.msg
  CartesianPointMeasurements.msg
  DetectionStati.msg
  DetectionStatus.msg
  EstimatedPoses.msg
  MeasurementIndex.msg
  MeasurementIndices.msg
  OptimizeSensorPose.msg
  OptimizeSensorPoses.msg
  PolarPointMeasurement.msg
  PolarPointMeasurements.msg
  Reset.msg
  Residual.msg
  ResidualGroup.msg
  Residuals.msg
  UseResidual.msg
  UseResiduals.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
)

catkin_package(
    CATKIN_DEPENDS message_runtime
)
