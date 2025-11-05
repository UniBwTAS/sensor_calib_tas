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

#include <XmlRpcException.h>
#include <visualization_msgs/Marker.h>

#include <sensor_calib_msgs/PolarPointMeasurement.h>
#include <sensor_calib_msgs/PolarPointMeasurements.h>

#include "doppler_simulator.h"

DopplerSimulator::DopplerSimulator(ros::NodeHandle& nh) noexcept : nh_(nh)
{
    parseBoardConfig();

    pub_marker_ = nh.advertise<visualization_msgs::Marker>("detection_markers", 1, true);
}

void DopplerSimulator::parseBoardConfig()
{
    // Read board configuration
    XmlRpc::XmlRpcValue board_config;
    if (!nh_.getParam("board_config", board_config))
    {
        ROS_FATAL("Failed to get parameter 'board_config'");
        ros::shutdown();
        std::exit(-1);
    }
    else
    {
        try
        {
            ROS_ASSERT(board_config.getType() == XmlRpc::XmlRpcValue::TypeArray);

            for (int32_t i = 0; i < board_config.size(); i++)
            {
                ROS_ASSERT(board_config[i].getType() == XmlRpc::XmlRpcValue::TypeStruct);
                const XmlRpc::XmlRpcValue& board_description = board_config[i];

                // Radar target
                if (board_description.hasMember("radar_targets"))
                {
                    ROS_ASSERT(board_description["radar_targets"].getType() == XmlRpc::XmlRpcValue::TypeArray);
                    const XmlRpc::XmlRpcValue& member_radar_targets = board_description[(std::string) "radar_targets"];

                    for (int32_t j = 0; j < member_radar_targets.size(); j++)
                    {
                        ROS_ASSERT(member_radar_targets[j].getType() == XmlRpc::XmlRpcValue::TypeStruct);
                        const XmlRpc::XmlRpcValue& radar_target = member_radar_targets[j];

                        ROS_ASSERT(radar_target["id"].getType() == XmlRpc::XmlRpcValue::TypeInt);
                        const int target_id = radar_target["id"];

                        ROS_ASSERT(radar_target["speed"].getType() == XmlRpc::XmlRpcValue::TypeDouble);
                        target_speeds_[target_id] = radar_target["speed"];
                    }
                }
            }
        }
        catch (XmlRpc::XmlRpcException e)
        {
            ROS_FATAL_STREAM("Error parsing board config: " << e.getMessage().c_str());
            ros::shutdown();
            std::exit(-1);
        }
    }
}

void DopplerSimulator::reconfigure(radar_target_detection::DopplerSimulatorConfig& config, uint32_t level)
{
    config_ = config;
}

void DopplerSimulator::callbackRadar(const radar_msgs::DetectionRecord::ConstPtr& msg) noexcept
{
    // Get sensor and create publisher if seen for the first time
    const FrameID& frame_id = msg->header.frame_id;
    if (!pub_measurements_.count(frame_id))
    {
        std::string topic = frame_id;
        const std::string substring = "sensor/radar/";
        if (std::size_t pos = topic.find(substring) != std::string::npos)
            topic.erase(pos - 1, substring.size());
        pub_measurements_[frame_id] = nh_.advertise<sensor_calib_msgs::PolarPointMeasurements>(topic, 1, false);

        for (const auto& target_elem : target_speeds_)
        {
            const TargetID target_id = target_elem.first;
            target_measurements_[frame_id][target_id] = std::vector<sensor_calib_msgs::PolarPointMeasurement>{};
        }
    }

    sensor_calib_msgs::PolarPointMeasurements target_measurements;
    target_measurements.header = msg->header;

    // Iterate over detections
    for (const radar_msgs::Detection& detection : msg->detections)
    {
        for (const auto& target_elem : target_speeds_)
        {
            const TargetID& target_id = target_elem.first;
            const double target_speed = target_elem.second;

            if (std::abs(detection.radial_speed.value - target_speed) <= config_.allowed_deviation)
            {
                sensor_calib_msgs::PolarPointMeasurement meas;
                meas.id = target_id;
                meas.range = detection.range.value;
                meas.azimuth = detection.azimuth.value;
                meas.elevation = detection.elevation.value;
                target_measurements_[frame_id][target_id].push_back(meas);
            }
        }
    }
}

void DopplerSimulator::callbackTimer(const ros::TimerEvent& event) noexcept
{
    bool found_doppler_simulator = false;

    for (auto& measurements_of_sensor_elem : target_measurements_)
    {
        const FrameID& frame_id = measurements_of_sensor_elem.first;

        sensor_calib_msgs::PolarPointMeasurements target_measurements;
        target_measurements.header.frame_id = frame_id;
        target_measurements.header.stamp = ros::Time::now();

        visualization_msgs::Marker marker;
        marker.header.frame_id = frame_id;
        marker.header.stamp = ros::Time::now();
        marker.ns = "doppler_simulator_" + frame_id;
        marker.id = 0;
        marker.type = visualization_msgs::Marker::SPHERE_LIST;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.01;
        marker.color.a = 1.0;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;

        for (auto& measurements_for_target_elem : measurements_of_sensor_elem.second)
        {
            const TargetID& target_id = measurements_for_target_elem.first;

            const int n = measurements_for_target_elem.second.size();

            if (n > 0)
            {
                double sum_range = 0;
                double sum_azimuth = 0;
                double sum_elevation = 0;

                for (const sensor_calib_msgs::PolarPointMeasurement& measurement : measurements_for_target_elem.second)
                {
                    sum_range += measurement.range;
                    sum_azimuth += measurement.azimuth;
                    sum_elevation += measurement.elevation;

                    geometry_msgs::Point p;
                    p.x = measurement.range * cos(measurement.azimuth) * cos(measurement.elevation);
                    p.y = measurement.range * sin(measurement.azimuth) * cos(measurement.elevation);
                    p.z = measurement.range * sin(measurement.elevation);
                    marker.points.push_back(p);
                }

                sensor_calib_msgs::PolarPointMeasurement meas;
                meas.id = target_id;
                meas.range = sum_range / n;
                meas.azimuth = sum_azimuth / n;
                meas.elevation = sum_elevation / n;
                target_measurements.measured_points.push_back(meas);

                pub_marker_.publish(marker);
            }

            measurements_for_target_elem.second.clear();
        }

        if (target_measurements.measured_points.size() > 0)
            found_doppler_simulator = true;

        pub_measurements_.at(frame_id).publish(target_measurements);
    }

    target_found_ = found_doppler_simulator;
}
