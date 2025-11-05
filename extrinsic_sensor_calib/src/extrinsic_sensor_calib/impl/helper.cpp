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

#include "../cost_functors.hpp"
#include "../sensor_calib.h"

Pose SensorCalib::getPoseFromTF(const std::string& from_frame_id, const std::string& to_frame_id) const noexcept
{
    while (true)
    {
        geometry_msgs::TransformStamped transform_stamped;

        try
        {
            ROS_INFO_STREAM_NAMED("Initialization",
                                  "Getting transform from '" << from_frame_id << "' to '" << to_frame_id << "'...");

            transform_stamped =
                tf_buffer_.lookupTransform(from_frame_id, to_frame_id, ros::Time(0), ros::Duration(0.1));
        }
        catch (const std::exception& /*e*/)
        {
            ROS_WARN_STREAM_NAMED("Initialization",
                                  "No tf available for the initialization of frame '" << to_frame_id
                                                                                      << "', trying again...");
            publishStatusMsg("No tf available for the initialization of frame '" + to_frame_id + "', trying again...");
            continue;
        }

        Pose result;

        result.translation.x() = transform_stamped.transform.translation.x;
        result.translation.y() = transform_stamped.transform.translation.y;
        result.translation.z() = transform_stamped.transform.translation.z;

        result.rotation.x() = transform_stamped.transform.rotation.x;
        result.rotation.y() = transform_stamped.transform.rotation.y;
        result.rotation.z() = transform_stamped.transform.rotation.z;
        result.rotation.w() = transform_stamped.transform.rotation.w;

        return result;
    }
}

std::string SensorCalib::timeIndexStr(const MeasurementIndex idx, const size_t id) const noexcept
{
    std::stringstream ss;
    ss << std::setw(3) << std::setfill('0') << idx;

    if (id != -1)
    {
        ss << "_" << std::setw(2) << std::setfill('0') << id;
    }

    return ss.str();
}

bool SensorCalib::isCamera(const FrameID& frame_id) const noexcept
{
    return std::find(camera_frame_ids_.begin(), camera_frame_ids_.end(), frame_id) != camera_frame_ids_.end();
}

bool SensorCalib::isLiDAR(const FrameID& frame_id) const noexcept
{
    return std::find(lidar_frame_ids_.begin(), lidar_frame_ids_.end(), frame_id) != lidar_frame_ids_.end();
}

bool SensorCalib::isRadar(const FrameID& frame_id) const noexcept
{
    return std::find(radar_frame_ids_.begin(), radar_frame_ids_.end(), frame_id) != radar_frame_ids_.end();
}

Pose SensorCalib::convertToReferenceFrame(const Pose& pose, const FrameID& sensor_frame_id) const noexcept
{
    const Eigen::Isometry3d pose_in_sensor = Eigen::Translation3d(pose.translation) * Eigen::Isometry3d(pose.rotation);

    const Pose sensor_pose = config_.use_optimized_sensor_pose_for_board_init ?
                                 sensor_poses_.at(std::make_pair(reference_frame_id_, sensor_frame_id)) :
                                 initial_sensor_poses_.at(std::make_pair(reference_frame_id_, sensor_frame_id));
    const Eigen::Isometry3d reference_from_sensor = sensor_pose.transform();

    const Eigen::Isometry3d ref_from_board = reference_from_sensor * pose_in_sensor;

    Pose ref_to_board;
    ref_to_board.translation = ref_from_board.translation();
    ref_to_board.rotation = ref_from_board.rotation();

    return ref_to_board;
}