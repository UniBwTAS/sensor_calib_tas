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
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgcodecs.hpp>
#include <boost/algorithm/string/replace.hpp>

#include <std_msgs/String.h>
#include <tf2/LinearMath/Quaternion.h>
#include <visualization_msgs/Marker.h>

#include <sensor_calib_msgs/DetectionStati.h>
#include <sensor_calib_msgs/EstimatedPoses.h>
#include <sensor_calib_msgs/MeasurementIndices.h>
#include <sensor_calib_msgs/Residuals.h>

#include "sensor_calib.h"

SensorCalib::SensorCalib(ros::NodeHandle& nh) : nh_(nh)
{
    // Advertise status topic
    status_pub_ = nh.advertise<std_msgs::String>("status", 10, true);

    // Get parameters
    if (!nh_.getParam("reference_frame", reference_frame_id_))
    {
        ROS_FATAL_STREAM_NAMED("Config", "Failed to get parameter 'reference_frame'");
        publishStatusMsg("Failed to get parameter 'reference_frame'");
        ros::shutdown();
        std::exit(-1);
    }

    // Get LiDAR sensors
    {
        XmlRpc::XmlRpcValue lidar_configs;
        if (!nh_.getParam("lidars", lidar_configs))
        {
            ROS_ERROR_STREAM_NAMED("Config", "Failed to get parameter 'lidars'");
        }
        else
        {
            for (int i = 0; i < lidar_configs.size(); i++)
            {
                const XmlRpc::XmlRpcValue& lidar_config = lidar_configs[i];

                if (lidar_config.hasMember("frame_id"))
                {
                    ROS_ASSERT(lidar_config["frame_id"].getType() == XmlRpc::XmlRpcValue::TypeString);
                    FrameID frame_id = lidar_config["frame_id"];
                    if (lidar_config.hasMember("detector_topic"))
                    {
                        lidar_frame_ids_.push_back(frame_id);
                        ROS_ASSERT(lidar_config["detector_topic"].getType() == XmlRpc::XmlRpcValue::TypeString);
                        lidar_reference_point_detector_topics_.push_back(lidar_config["detector_topic"]);

                        measurement_uncertainties_[frame_id].sigma_cartesian =
                            xmlRpcGetDoubleWithDefault(lidar_config, "sigma", 0.01);

                        if (!lidar_config.hasMember("sigma"))
                        {
                            ROS_WARN_STREAM_NAMED("Config",
                                                  "Using default measurement uncertainty for LiDAR '"
                                                      << frame_id << "' due to missing parameter 'sigma'");
                        }
                    }
                    else
                    {
                        ROS_ERROR_STREAM_NAMED(
                            "Config", "Ignoring LiDAR '" << frame_id << "' due to missing parameter 'detector_topic'");
                    }
                }
                else
                {
                    ROS_ERROR_STREAM_NAMED("Config", "Ignoring LiDAR with missing parameter 'frame id'");
                }
            }
        }
    }

    // Get camera sensors
    {
        XmlRpc::XmlRpcValue camera_configs;
        if (!nh_.getParam("cameras", camera_configs))
        {
            ROS_FATAL_STREAM_NAMED("Config", "Failed to get parameter 'cameras'");
            publishStatusMsg("Failed to get parameter 'cameras'");
            ros::shutdown();
            std::exit(-1);
        }

        for (int i = 0; i < camera_configs.size(); i++)
        {
            const XmlRpc::XmlRpcValue& camera_config = camera_configs[i];

            if (camera_config.hasMember("frame_id"))
            {
                ROS_ASSERT(camera_config["frame_id"].getType() == XmlRpc::XmlRpcValue::TypeString);
                FrameID frame_id = camera_config["frame_id"];
                if (camera_config.hasMember("detector_topic"))
                {
                    camera_frame_ids_.push_back(frame_id);
                    ROS_ASSERT(camera_config["detector_topic"].getType() == XmlRpc::XmlRpcValue::TypeString);
                    apriltag_detector_topics_.push_back(camera_config["detector_topic"]);

                    measurement_uncertainties_[frame_id].sigma_pixel =
                        xmlRpcGetDoubleWithDefault(camera_config, "sigma", 0.2);

                    if (!camera_config.hasMember("sigma"))
                    {
                        ROS_WARN_STREAM_NAMED("Config",
                                              "Using default measurement uncertainty for camera '"
                                                  << frame_id << "' due to missing parameter 'sigma'");
                    }

                    if (camera_config.hasMember("camera_topic"))
                    {
                        ROS_ASSERT(camera_config["camera_topic"].getType() == XmlRpc::XmlRpcValue::TypeString);
                        camera_topics_.push_back(camera_config["camera_topic"]);
                    }
                    else
                    {
                        camera_topics_.push_back("/" + frame_id);
                    }

                    if (camera_config.hasMember("rectified"))
                    {
                        ROS_ASSERT(camera_config["rectified"].getType() == XmlRpc::XmlRpcValue::TypeBoolean);
                        camera_intrinsics_[frame_id].rectified = camera_config["rectified"];
                    }
                    else
                    {
                        camera_intrinsics_[frame_id].rectified = false;
                        ROS_WARN_STREAM_NAMED("Config",
                                              "Assuming unrectified image for camera '"
                                                  << frame_id << "' due to missing parameter 'rectified'");
                    }
                }
                else
                {
                    ROS_ERROR_STREAM_NAMED(
                        "Config", "Ignoring camera '" << frame_id << "' due to missing parameter 'detector_topic'");
                }
            }
            else
            {
                ROS_ERROR_STREAM_NAMED("Config", "Ignoring camera with missing parameter 'frame id'");
            }
        }
    }

    // Get radar sensors
    {
        XmlRpc::XmlRpcValue radar_configs;
        if (!nh_.getParam("radars", radar_configs))
        {
            ROS_ERROR_STREAM_NAMED("Config", "Failed to get parameter 'radars'");
        }
        else
        {
            for (int i = 0; i < radar_configs.size(); i++)
            {
                const XmlRpc::XmlRpcValue& radar_config = radar_configs[i];

                if (radar_config.hasMember("frame_id"))
                {
                    ROS_ASSERT(radar_config["frame_id"].getType() == XmlRpc::XmlRpcValue::TypeString);
                    FrameID frame_id = radar_config["frame_id"];
                    if (radar_config.hasMember("detector_topic"))
                    {
                        radar_frame_ids_.push_back(frame_id);
                        ROS_ASSERT(radar_config["detector_topic"].getType() == XmlRpc::XmlRpcValue::TypeString);
                        radar_target_detector_topics_.push_back(radar_config["detector_topic"]);

                        measurement_uncertainties_[frame_id].sigma_polar =
                            PolarCoordinate(xmlRpcGetDoubleWithDefault(radar_config, "sigma_range", 0.1),
                                            xmlRpcGetDoubleWithDefault(radar_config, "sigma_azimuth", 0.5),
                                            xmlRpcGetDoubleWithDefault(radar_config, "sigma_elevation", 9));

                        if (!radar_config.hasMember("sigma_range") || !radar_config.hasMember("sigma_azimuth") ||
                            !radar_config.hasMember("sigma_elevation"))
                        {
                            ROS_WARN_STREAM_NAMED(
                                "Config",
                                "Using default measurement uncertainty for radar '"
                                    << frame_id
                                    << "' due to missing parameters 'sigma_range', 'sigma_azimuth', 'sigma_elevation'");
                        }
                    }
                    else
                    {
                        ROS_ERROR_STREAM_NAMED(
                            "Config", "Ignoring radar '" << frame_id << "' due to missing parameter 'detector_topic'");
                    }
                }
                else
                {
                    ROS_ERROR_STREAM_NAMED("Config", "Ignoring radar with missing parameter 'frame id'");
                }
            }
        }
    }

    // Read board configuration
    XmlRpc::XmlRpcValue board_config;
    if (!nh_.getParam("board_config/boards", board_config))
    {
        ROS_FATAL_STREAM_NAMED("Config", "Failed to get parameter 'board_config/boards'");
        publishStatusMsg("Failed to get parameter 'board_config/boards'");
        ros::shutdown();
        std::exit(-1);
    }
    else
    {
        try
        {
            parseBoardConfig(board_config);
        }
        catch (XmlRpc::XmlRpcException e)
        {
            ROS_FATAL_STREAM_NAMED("Config", "Error parsing board config: " << e.getMessage().c_str());
            publishStatusMsg("Error parsing board config");
            ros::shutdown();
            std::exit(-1);
        }
    }

    ROS_INFO_STREAM_NAMED("Config", "Loaded " << calibration_boards_.size() << " calibration targets");

    // Init detection stati
    for (const std::string& frame_id : camera_frame_ids_)
    {
        sensor_calib_msgs::DetectionStatus msg;
        msg.stamp = ros::Time(0);
        detection_stati_[frame_id].sensor = frame_id;
    }
    for (const std::string& frame_id : lidar_frame_ids_)
    {
        sensor_calib_msgs::DetectionStatus msg;
        msg.stamp = ros::Time(0);
        detection_stati_[frame_id].sensor = frame_id;
    }
    for (const std::string& frame_id : radar_frame_ids_)
    {
        sensor_calib_msgs::DetectionStatus msg;
        msg.stamp = ros::Time(0);
        detection_stati_[frame_id].sensor = frame_id;
    }

    // Advertise GUI and visualization topics
    sensor_poses_pub_ = nh.advertise<sensor_calib_msgs::EstimatedPoses>("sensor_poses", 10, true);
    board_poses_pub_ = nh.advertise<sensor_calib_msgs::EstimatedPoses>("board_poses", 10, true);
    residuals_pub_ = nh.advertise<sensor_calib_msgs::Residuals>("residuals", 10, true);
    overall_residuals_pub_ = nh.advertise<sensor_calib_msgs::Residuals>("overall_residuals", 10, true);
    marker_pub_ = nh.advertise<visualization_msgs::Marker>("debug_markers", 100, true);
    detection_status_pub_ = nh.advertise<sensor_calib_msgs::DetectionStati>("detection_status", 10, true);
    measurement_indices_pub_ = nh.advertise<sensor_calib_msgs::MeasurementIndices>("measurement_indices", 10, true);

    // Advertise the camera publishers
    initCameraPublishers();

    publishStatusMsg("Init...");

    initSensorPoses();
    initIntrinsics();
    initialized_ = true;

    calibrate(false);
}

void SensorCalib::initCameraPublishers() noexcept
{
    // Advertise publishers for debug images
    for (const FrameID& frame_id : camera_frame_ids_)
    {
        if (debug_image_pub_.count(frame_id) == 0)
        {
            debug_image_pub_[frame_id] = nh_.advertise<sensor_msgs::Image>("debug_images/" + frame_id, 10, true);
        }
    }
}

void SensorCalib::parseBoardConfig(const XmlRpc::XmlRpcValue& boards)
{
    ROS_ASSERT(boards.getType() == XmlRpc::XmlRpcValue::TypeArray);

    for (int32_t i = 0; i < boards.size(); i++)
    {
        BoardConfig board_config;

        ROS_ASSERT(boards[i].getType() == XmlRpc::XmlRpcValue::TypeStruct);
        const XmlRpc::XmlRpcValue& board_description = boards[i];

        // Name
        std::string board_name;
        if (board_description.hasMember("name"))
        {
            ROS_ASSERT(board_description["name"].getType() == XmlRpc::XmlRpcValue::TypeString);
            board_name = static_cast<std::string>(board_description["name"]);
        }
        else
        {
            std::stringstream board_name_stream;
            board_name_stream << "board_" << i;
            board_name = board_name_stream.str();
        }

        // AprilTags
        if (board_description.hasMember("layout"))
        {
            ROS_ASSERT(board_description["layout"].getType() == XmlRpc::XmlRpcValue::TypeArray);
            const XmlRpc::XmlRpcValue& member_tags = board_description["layout"];

            for (int32_t j = 0; j < member_tags.size(); j++)
            {
                ROS_ASSERT(member_tags[j].getType() == XmlRpc::XmlRpcValue::TypeStruct);
                const XmlRpc::XmlRpcValue& tag = member_tags[j];

                ROS_ASSERT(tag["id"].getType() == XmlRpc::XmlRpcValue::TypeInt);
                const int id = tag["id"];
                const std::string id_str = std::to_string(id);

                ROS_ASSERT(tag["size"].getType() == XmlRpc::XmlRpcValue::TypeDouble);
                board_config.apriltags[id_str].size = tag["size"];

                board_config.apriltags[id_str].tag_pose.translation =
                    Eigen::Vector3d(xmlRpcGetDoubleWithDefault(tag, "x", 0.),
                                    xmlRpcGetDoubleWithDefault(tag, "y", 0.),
                                    xmlRpcGetDoubleWithDefault(tag, "z", 0.));
                board_config.apriltags[id_str].tag_pose.rotation =
                    Eigen::Quaterniond(xmlRpcGetDoubleWithDefault(tag, "qw", 1.),
                                       xmlRpcGetDoubleWithDefault(tag, "qx", 0.),
                                       xmlRpcGetDoubleWithDefault(tag, "qy", 0.),
                                       xmlRpcGetDoubleWithDefault(tag, "qz", 0.));
            }
        }

        // LiDAR reference points
        if (board_description.hasMember("lidar_reference_points"))
        {
            ROS_ASSERT(board_description["lidar_reference_points"].getType() == XmlRpc::XmlRpcValue::TypeArray);
            const XmlRpc::XmlRpcValue& member_lidar_reference_points = board_description["lidar_reference_points"];

            for (int32_t j = 0; j < member_lidar_reference_points.size(); j++)
            {
                ROS_ASSERT(member_lidar_reference_points[j].getType() == XmlRpc::XmlRpcValue::TypeStruct);
                const XmlRpc::XmlRpcValue& lidar_reference_point = member_lidar_reference_points[j];

                ROS_ASSERT(lidar_reference_point["id"].getType() == XmlRpc::XmlRpcValue::TypeInt);
                const int id = lidar_reference_point["id"];
                board_config.lidar_reference_points[id].position.x() =
                    xmlRpcGetDoubleWithDefault(lidar_reference_point, "x", 0);
                board_config.lidar_reference_points[id].position.y() =
                    xmlRpcGetDoubleWithDefault(lidar_reference_point, "y", 0);
                board_config.lidar_reference_points[id].position.z() =
                    xmlRpcGetDoubleWithDefault(lidar_reference_point, "z", 0);
                board_config.lidar_reference_points[id].radius =
                    xmlRpcGetDoubleWithDefault(lidar_reference_point, "radius", 0.12);
            }
        }

        // Radar targets
        if (board_description.hasMember("radar_targets"))
        {
            ROS_ASSERT(board_description["radar_targets"].getType() == XmlRpc::XmlRpcValue::TypeArray);
            const XmlRpc::XmlRpcValue& member_radar_targets = board_description["radar_targets"];

            for (int32_t j = 0; j < member_radar_targets.size(); j++)
            {
                ROS_ASSERT(member_radar_targets[j].getType() == XmlRpc::XmlRpcValue::TypeStruct);
                const XmlRpc::XmlRpcValue& radar_target = member_radar_targets[j];

                ROS_ASSERT(radar_target["id"].getType() == XmlRpc::XmlRpcValue::TypeInt);
                const int id = radar_target["id"];
                board_config.radar_targets[id].x() = xmlRpcGetDoubleWithDefault(radar_target, "x", 0);
                board_config.radar_targets[id].y() = xmlRpcGetDoubleWithDefault(radar_target, "y", 0);
                board_config.radar_targets[id].z() = xmlRpcGetDoubleWithDefault(radar_target, "z", 0);
            }
        }

        calibration_boards_[board_name] = board_config;

        ROS_INFO_STREAM_NAMED("Config",
                              "Added new calibration target '"
                                  << board_name << "' with " << board_config.apriltags.size() << " AprilTags, "
                                  << board_config.lidar_reference_points.size() << " LiDAR reference points and "
                                  << board_config.radar_targets.size() << " radar targets.");
    }
}

// from apriltag_ros/common_functions.cpp
// see https://docs.ros.org/en/noetic/api/apriltag_ros/html/common__functions_8cpp_source.html
double SensorCalib::xmlRpcGetDoubleWithDefault(const XmlRpc::XmlRpcValue& xmlValue,
                                               const std::string& field,
                                               const double defaultValue) const
{
    if (xmlValue.hasMember(field))
    {
        ROS_ASSERT((xmlValue[field].getType() == XmlRpc::XmlRpcValue::TypeDouble) ||
                   (xmlValue[field].getType() == XmlRpc::XmlRpcValue::TypeInt));
        if (xmlValue[field].getType() == XmlRpc::XmlRpcValue::TypeInt)
        {
            const int tmp = xmlValue[field];
            return tmp;
        }
        else
        {
            return xmlValue[field];
        }
    }
    else
    {
        return defaultValue;
    }
}
