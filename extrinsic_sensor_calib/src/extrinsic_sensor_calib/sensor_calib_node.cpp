/*
 * Copyright (c) 2025, Perception for Autonomous Driving Lab,
 * Institute for Autonomous Driving, Universität der Bundeswehr München
 * All rights reserved.
 *
 * Author: Bianca Forkel
 *
 * This file is part of the sensor_calib_tas package.
 * Licensed under the BSD 3-Clause License.
 * See the LICENSE file in the project root for full license information.
 */

#include <dynamic_reconfigure/server.h>
#include <ros/ros.h>

#include "sensor_calib.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "extrinsic_sensor_calib");
    ros::NodeHandle nh("~");

    SensorCalib sensor_calib(nh);

    std::vector<ros::Subscriber> subscribers;

    // Subscribe detection topics
    for (const std::string& topic : sensor_calib.apriltag_detector_topics_)
    {
        subscribers.push_back(nh.subscribe<apriltags_msgs::AprilTagDetections>(
            topic, 1, &SensorCalib::callbackAprilTagDetections, &sensor_calib));
    }
    for (const std::string& topic : sensor_calib.lidar_reference_point_detector_topics_)
    {
        subscribers.push_back(nh.subscribe<sensor_calib_msgs::CartesianPointMeasurements>(
            topic, 1, &SensorCalib::callbackLiDARTargetDetections, &sensor_calib));
    }
    for (const std::string& topic : sensor_calib.radar_target_detector_topics_)
    {
        subscribers.push_back(nh.subscribe<sensor_calib_msgs::PolarPointMeasurements>(
            topic, 1, &SensorCalib::callbackRadarTargetDetections, &sensor_calib));
    }

    // Subscribe service-like topics
    subscribers.push_back(
        nh.subscribe<std_msgs::Bool>("requests/measure", 1, &SensorCalib::measureRequest, &sensor_calib));
    subscribers.push_back(nh.subscribe<std_msgs::Empty>(
        "requests/remove_last_measurement", 1, &SensorCalib::removeLastMeasurementRequest, &sensor_calib));

    subscribers.push_back(
        nh.subscribe<std_msgs::Empty>("requests/solve", 1, &SensorCalib::solveRequest, &sensor_calib));
    subscribers.push_back(
        nh.subscribe<std_msgs::Empty>("requests/verify", 1, &SensorCalib::verifyRequest, &sensor_calib));

    subscribers.push_back(nh.subscribe<sensor_calib_msgs::Reset>(
        "requests/reset_sensor_pose", 1, &SensorCalib::resetSensorPoseRequest, &sensor_calib));
    subscribers.push_back(nh.subscribe<sensor_calib_msgs::Reset>(
        "requests/reset_board_pose", 1, &SensorCalib::resetBoardPoseRequest, &sensor_calib));

    subscribers.push_back(
        nh.subscribe<std_msgs::Bool>("requests/preview", 1, &SensorCalib::previewRequest, &sensor_calib));

    subscribers.push_back(
        nh.subscribe<std_msgs::String>("requests/save_to_file", 1, &SensorCalib::saveToFileRequest, &sensor_calib));
    subscribers.push_back(
        nh.subscribe<std_msgs::String>("requests/load_from_file", 1, &SensorCalib::loadFromFileRequest, &sensor_calib));

    subscribers.push_back(nh.subscribe<sensor_calib_msgs::UseResiduals>(
        "requests/measurements_to_use", 1, &SensorCalib::updateUsedMeasurements, &sensor_calib));
    subscribers.push_back(nh.subscribe<sensor_calib_msgs::OptimizeSensorPoses>(
        "requests/sensor_poses_to_optimize", 1, &SensorCalib::updateOptimizedSensorPoses, &sensor_calib));
    subscribers.push_back(nh.subscribe<sensor_calib_msgs::MeasurementIndex>(
        "requests/measurement_idx_to_visualize", 1, &SensorCalib::updateVisualizedMeasurement, &sensor_calib));

    // Setup dynamic reconfigure
    dynamic_reconfigure::Server<extrinsic_sensor_calib::ExtrinsicSensorCalibConfig> reconfigure_server;
    reconfigure_server.setCallback(boost::bind(&SensorCalib::reconfigure, &sensor_calib, _1, _2));

    ros::Rate loop_rate(5);

    while (ros::ok())
    {
        ros::spinOnce();

        sensor_calib.publishResults();

        loop_rate.sleep();
    }

    return 0;
}
