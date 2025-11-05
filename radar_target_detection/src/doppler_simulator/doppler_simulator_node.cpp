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

#include "doppler_simulator.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "doppler_simulator_detector");
    ros::NodeHandle nh("~");

    DopplerSimulator doppler_simulator(nh);

    dynamic_reconfigure::Server<radar_target_detection::DopplerSimulatorConfig> reconfigure_server;
    reconfigure_server.setCallback(boost::bind(&DopplerSimulator::reconfigure, &doppler_simulator, _1, _2));

    std::string radar_topic_name;
    nh.param("topic", radar_topic_name, static_cast<std::string>("/sensor/radar/umrr/detections"));
    ros::Subscriber radar_sub = nh.subscribe<radar_msgs::DetectionRecord>(
        radar_topic_name, 20, &DopplerSimulator::callbackRadar, &doppler_simulator);

    const ros::Timer timer = nh.createTimer(ros::Duration(1), &DopplerSimulator::callbackTimer, &doppler_simulator);

    while (ros::ok())
    {
        ros::spin();
    }

    return 0;
}
