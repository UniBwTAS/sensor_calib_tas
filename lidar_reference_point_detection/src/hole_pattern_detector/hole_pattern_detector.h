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

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <lidar_reference_point_detection/HolePatternDetectorConfig.h>

#include "pcl_point_type.h"
#include "types.h"

class HolePatternDetector
{
  public:
    HolePatternDetector(ros::NodeHandle& nh) noexcept;

    void reconfigure(lidar_reference_point_detection::HolePatternDetectorConfig& config, uint32_t level) noexcept;

    void callbackPointCloud(const pcl::PointCloud<pcl::PointXYZR>::ConstPtr& point_cloud) noexcept;

  private:
    void parseBoardConfig() noexcept;
    double xmlRpcGetDoubleWithDefault(const XmlRpc::XmlRpcValue& xmlValue,
                                      const std::string& field,
                                      const double defaultValue) const noexcept;

    bool findCalibrationBoard(const pcl::PointCloud<pcl::PointXYZR>::ConstPtr& point_cloud,
                              pcl::PointCloud<pcl::PointXYZR>& board_cloud,
                              const std_msgs::Header& header) noexcept;
    bool detectEdgePoints(const pcl::PointCloud<pcl::PointXYZR>& board_cloud,
                          pcl::PointCloud<pcl::PointXYZR>& edge_points) noexcept;
    bool determineHolePositions(const pcl::PointCloud<pcl::PointXYZR>& edge_points,
                                Eigen::Vector3d& translation,
                                Eigen::Quaterniond& rotation,
                                std::map<ReferencePointID, Eigen::Vector3d>& lidar_predicted_holes) const noexcept;

    void visualizeClusterBoundingBox(const pcl::PointXYZR& obb_position,
                                     const Eigen::Matrix3f& obb_rotation,
                                     const double large_side_length,
                                     const double small_side_length,
                                     const double depth,
                                     const bool found_board,
                                     const std_msgs::Header& header) noexcept;

    void publishResults(const Eigen::Vector3d& translation,
                        const Eigen::Quaterniond& rotation,
                        const std::map<ReferencePointID, Eigen::Vector3d>& lidar_predicted_holes,
                        const std_msgs::Header& header) noexcept;

    lidar_reference_point_detection::HolePatternDetectorConfig config_;

    std::map<ReferencePointID, LiDARReferencePointConfig> lidar_reference_points_;
    double board_width_;
    double board_height_;
    Eigen::Isometry3d lidar_from_base_link_;
    bool lidar_from_base_link_valid_{false};

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_{tf_buffer_};
    tf2_ros::TransformBroadcaster transform_broadcaster_;

    ros::Publisher pub_cartesian_;
    ros::Publisher pub_marker_;
    ros::Publisher pub_edge_points_;
    ros::Publisher pub_board_;
    ros::Publisher pub_debug_marker_;

    ros::NodeHandle nh_;
};