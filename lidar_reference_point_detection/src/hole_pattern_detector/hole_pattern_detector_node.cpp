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

#include "hole_pattern_detector.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "hole_pattern_detector");
    ros::NodeHandle nh("~");

    HolePatternDetector hole_pattern_detector(nh);

    dynamic_reconfigure::Server<lidar_reference_point_detection::HolePatternDetectorConfig> reconfigure_server;
    reconfigure_server.setCallback(boost::bind(&HolePatternDetector::reconfigure, &hole_pattern_detector, _1, _2));

    ros::Subscriber point_cloud_sub = nh.subscribe<pcl::PointCloud<pcl::PointXYZR>>(
        "point_cloud", 1, &HolePatternDetector::callbackPointCloud, &hole_pattern_detector);

    while (ros::ok())
    {
        ros::spin();
    }

    return 0;
}
