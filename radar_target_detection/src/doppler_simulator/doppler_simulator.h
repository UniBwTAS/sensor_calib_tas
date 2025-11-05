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

#pragma once

#include <ros/ros.h>

#include <sensor_calib_msgs/PolarPointMeasurement.h>
#include <radar_msgs/DetectionRecord.h>

#include <radar_target_detection/DopplerSimulatorConfig.h>

class DopplerSimulator
{
    using FrameID = std::string;
    using TargetID = std::int16_t;

  public:
    DopplerSimulator(ros::NodeHandle& nh) noexcept;

    void reconfigure(radar_target_detection::DopplerSimulatorConfig& config, uint32_t level);

    void callbackRadar(const radar_msgs::DetectionRecord::ConstPtr& msg) noexcept;
    void callbackTimer(const ros::TimerEvent& event) noexcept;

  private:
    void parseBoardConfig();

    radar_target_detection::DopplerSimulatorConfig config_;

    std::map<TargetID, double> target_speeds_;
    std::map<FrameID, std::map<TargetID, std::vector<sensor_calib_msgs::PolarPointMeasurement>>> target_measurements_;

    bool target_found_{false};
    int count_{0};

    std::map<std::string, ros::Publisher> pub_measurements_; // one publisher per sensor
    ros::Publisher pub_marker_;

    ros::NodeHandle nh_;
};