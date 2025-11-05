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

#define PCL_NO_PRECOMPILE

#include <pcl/common/distances.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <XmlRpcException.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_eigen/tf2_eigen.h>
#include <visualization_msgs/Marker.h>

#include <sensor_calib_msgs/CartesianPointMeasurement.h>
#include <sensor_calib_msgs/CartesianPointMeasurements.h>

#include "cost_functors.hpp"
#include "hole_pattern_detector.h"

HolePatternDetector::HolePatternDetector(ros::NodeHandle& nh) noexcept : nh_(nh)
{
    parseBoardConfig();

    pub_cartesian_ = nh_.advertise<sensor_calib_msgs::CartesianPointMeasurements>("detections", 1);
    pub_marker_ = nh_.advertise<visualization_msgs::Marker>("detection_markers", 10, false);
    pub_edge_points_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZR>>("edge_points", 1, false);
    pub_board_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZR>>("segmented_board", 1, false);
    pub_debug_marker_ = nh_.advertise<visualization_msgs::Marker>("debug_markers", 50, false);
}

void HolePatternDetector::parseBoardConfig() noexcept
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

                // LiDAR reference points
                if (board_description.hasMember("lidar_reference_points"))
                {
                    ROS_ASSERT(board_description["lidar_reference_points"].getType() == XmlRpc::XmlRpcValue::TypeArray);
                    const XmlRpc::XmlRpcValue& member_lidar_reference_points =
                        board_description["lidar_reference_points"];

                    for (int32_t j = 0; j < member_lidar_reference_points.size(); j++)
                    {
                        ROS_ASSERT(member_lidar_reference_points[j].getType() == XmlRpc::XmlRpcValue::TypeStruct);
                        const XmlRpc::XmlRpcValue& lidar_reference_point = member_lidar_reference_points[j];

                        ROS_ASSERT(lidar_reference_point["id"].getType() == XmlRpc::XmlRpcValue::TypeInt);
                        const int id = lidar_reference_point["id"];
                        lidar_reference_points_[id].position.x() =
                            xmlRpcGetDoubleWithDefault(lidar_reference_point, "x", 0);
                        lidar_reference_points_[id].position.y() =
                            xmlRpcGetDoubleWithDefault(lidar_reference_point, "y", 0);
                        lidar_reference_points_[id].position.z() =
                            xmlRpcGetDoubleWithDefault(lidar_reference_point, "z", 0);
                        lidar_reference_points_[id].radius =
                            xmlRpcGetDoubleWithDefault(lidar_reference_point, "radius", 0.12);
                    }

                    board_width_ = xmlRpcGetDoubleWithDefault(board_description, "width", 0);
                    board_height_ = xmlRpcGetDoubleWithDefault(board_description, "height", 0);
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

// from apriltag_ros/common_functions.cpp
// see https://docs.ros.org/en/noetic/api/apriltag_ros/html/common__functions_8cpp_source.html
double HolePatternDetector::xmlRpcGetDoubleWithDefault(const XmlRpc::XmlRpcValue& xmlValue,
                                                       const std::string& field,
                                                       const double defaultValue) const noexcept
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

void HolePatternDetector::reconfigure(lidar_reference_point_detection::HolePatternDetectorConfig& config,
                                      uint32_t level) noexcept
{
    config_ = config;
}

void HolePatternDetector::callbackPointCloud(const pcl::PointCloud<pcl::PointXYZR>::ConstPtr& point_cloud) noexcept
{
    // Init lidar mounting angle from tf
    if (!lidar_from_base_link_valid_)
    {
        try
        {
            lidar_from_base_link_ = tf2::transformToEigen(tf_buffer_.lookupTransform(
                point_cloud->header.frame_id, "base_link", ros::Time(0), ros::Duration(0.5)));
            lidar_from_base_link_valid_ = true;
        }
        catch (const std::exception& e)
        {
            ROS_WARN_STREAM("Could not get tf from base_link to " << point_cloud->header.frame_id);
            return;
        }
    }

    // Init ROS header
    std_msgs::Header header;
    header.frame_id = point_cloud->header.frame_id;
    header.stamp = pcl_conversions::fromPCL(point_cloud->header.stamp);

    // Plane segmentation, clustering, and board detection
    pcl::PointCloud<pcl::PointXYZR> board_cloud;
    bool success = findCalibrationBoard(point_cloud, board_cloud, header);
    if (!success)
        return;

    // Edge point detection
    pcl::PointCloud<pcl::PointXYZR> edge_points;
    success = detectEdgePoints(board_cloud, edge_points);
    if (!success)
        return;

    // Hole detection using optimization
    Eigen::Vector3d translation{Eigen::Vector3d::Zero()};
    Eigen::Quaterniond rotation{Eigen::Quaterniond::Identity()};
    std::map<ReferencePointID, Eigen::Vector3d> lidar_predicted_holes;
    success = determineHolePositions(edge_points, translation, rotation, lidar_predicted_holes);
    if (!success)
        return;

    publishResults(translation, rotation, lidar_predicted_holes, header);
}

bool HolePatternDetector::findCalibrationBoard(const pcl::PointCloud<pcl::PointXYZR>::ConstPtr& point_cloud,
                                               pcl::PointCloud<pcl::PointXYZR>& board_cloud,
                                               const std_msgs::Header& header) noexcept
{
    pcl::PointCloud<pcl::PointXYZR>::Ptr input_cloud = point_cloud->makeShared();
    pcl::PointCloud<pcl::PointXYZR>::Ptr tmp_cloud(new pcl::PointCloud<pcl::PointXYZR>);

    // Prepare to segment planes parallel to base_link z to find board
    // Assumption: Calibration board and robot (not sensor!) are upright
    pcl::PointCloud<pcl::PointXYZR>::Ptr plane_cloud(new pcl::PointCloud<pcl::PointXYZR>);
    pcl::SACSegmentation<pcl::PointXYZR> plane_segmentation;
    plane_segmentation.setModelType(pcl::SACMODEL_PARALLEL_PLANE);
    plane_segmentation.setMethodType(pcl::SAC_RANSAC);
    const Eigen::Vector3f up_vector = lidar_from_base_link_.rotation().col(2).cast<float>();
    plane_segmentation.setAxis(up_vector);
    plane_segmentation.setEpsAngle(config_.max_angle_to_upright * M_PI / 180.0);
    plane_segmentation.setDistanceThreshold(config_.plane_distance_threshold);
    plane_segmentation.setMaxIterations(1000000);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ExtractIndices<pcl::PointXYZR> plane_extract;

    // Prepare clustering
    pcl::PointCloud<pcl::PointXYZR>::Ptr cluster_cloud(new pcl::PointCloud<pcl::PointXYZR>);
    pcl::search::KdTree<pcl::PointXYZR>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZR>);
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZR> ec;
    ec.setClusterTolerance(config_.cluster_tolerance);
    ec.setMinClusterSize(config_.min_num_points_on_board);
    ec.setMaxClusterSize(config_.max_num_points_on_board);
    ec.setSearchMethod(tree);

    // Prepare bounding box calculation
    pcl::MomentOfInertiaEstimation<pcl::PointXYZR> feature_extractor;
    pcl::PointXYZR obb_min_pt, obb_max_pt, obb_position;
    Eigen::Matrix3f obb_rotation;

    bool found_board = false;

    // Keep segmenting planes until board is found or input cloud is almost empty
    while (input_cloud->size() > config_.min_num_points_on_board)
    {
        // Segment plane
        plane_segmentation.setInputCloud(input_cloud);
        plane_segmentation.segment(*inliers, *coefficients);

        if (inliers->indices.size() < config_.min_num_points_on_board)
        {
            break;
        }

        // Get points belonging to segmented plane
        plane_extract.setInputCloud(input_cloud);
        plane_extract.setIndices(inliers);
        plane_extract.setNegative(false);
        plane_extract.filter(*plane_cloud);

        // Extract clusters inside of plane
        tree->setInputCloud(plane_cloud);
        ec.setInputCloud(plane_cloud);
        ec.extract(cluster_indices);

        // Check if one of the extracted clusters is the calibration board
        for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end();
             ++it)
        {
            // Get points belonging to extracted cluster
            for (const auto& index : it->indices)
            {
                cluster_cloud->push_back((*plane_cloud)[index]);
            }

            found_board = true;

            // Get bounding box of the cluster
            feature_extractor.setInputCloud(cluster_cloud);
            feature_extractor.compute();
            feature_extractor.getOBB(obb_min_pt, obb_max_pt, obb_position, obb_rotation);
            const double large_side_length = std::abs(obb_max_pt.x - obb_min_pt.x);
            const double small_side_length = std::abs(obb_max_pt.y - obb_min_pt.y);
            const double depth = std::abs(obb_max_pt.z - obb_min_pt.z);

            // Get bounding box dimensions: find cluster orientation
            const Eigen::Vector3f obb_x = obb_rotation.col(0);
            const Eigen::Vector3f obb_y = obb_rotation.col(1);
            double width, height;
            if (std::atan2(obb_y.cross(up_vector).norm(), obb_y.dot(up_vector)) <
                std::atan2(obb_x.cross(up_vector).norm(), obb_x.dot(up_vector)))
            {
                height = small_side_length;
                width = large_side_length;
            }
            else
            {
                height = large_side_length;
                width = small_side_length;
            }

            // Check if cluster has the size of the calibration board
            if (depth > 2 * config_.plane_distance_threshold + config_.depth_tolerance)
            {
                found_board = false;
            }
            if (width > board_width_ + config_.width_tolerance || width < board_width_ - config_.width_tolerance)
            {
                found_board = false;
            }
            if (height > board_height_ + config_.height_tolerance || height < board_height_ - config_.height_tolerance)
            {
                found_board = false;
            }

            // Check angle between cluster normal and view vector from cluster to lidar
            // Assumption: Board points towards sensor. Set normal_tolerance to 360 to deactivate check.
            const Eigen::Vector3f obb_normal = obb_rotation.col(2);
            const Eigen::Vector3f obb_to_lidar = obb_position.getVector3fMap();
            const double angle = std::atan2(obb_normal.cross(obb_to_lidar).norm(), obb_normal.dot(obb_to_lidar));
            if (config_.normal_tolerance != 360 && angle > config_.normal_tolerance * M_PI / 180.0 &&
                abs(angle - M_PI) > config_.normal_tolerance * M_PI / 180.0)
            {
                found_board = false;
            }

            visualizeClusterBoundingBox(
                obb_position, obb_rotation, large_side_length, small_side_length, depth, found_board, header);

            if (found_board)
            {
                break;
            }
            else
            {
                // Go to next cluster
                cluster_cloud->clear();
            }
        }

        if (found_board)
        {
            board_cloud = *cluster_cloud;
            break;
        }
        else
        {
            // Remove segmented plane from input point cloud and segment next plane
            plane_extract.setNegative(true);
            plane_extract.filter(*tmp_cloud);
            input_cloud->swap(*tmp_cloud);
        }
    }

    if (!found_board)
    {
        ROS_DEBUG_STREAM("Could not find calibration board");
        return false;
    }

    // Publish debug point cloud showing segmented board points
    board_cloud.header = point_cloud->header;
    pub_board_.publish(board_cloud);

    return true;
}

bool HolePatternDetector::detectEdgePoints(const pcl::PointCloud<pcl::PointXYZR>& board_cloud,
                                           pcl::PointCloud<pcl::PointXYZR>& edge_points) noexcept
{
    // Get one point cloud per ring
    std::map<uint16_t, pcl::PointCloud<pcl::PointXYZR>> rings;
    for (const pcl::PointXYZR& p : board_cloud.points)
    {
        rings[p.ring].push_back(p);
    }

    // Along each ring: Compute distances to previous and next points and obtain edge points based on thresholds
    for (const auto& rings_elem : rings)
    {
        const pcl::PointCloud<pcl::PointXYZR>& ring = rings_elem.second;
        for (pcl::PointCloud<pcl::PointXYZR>::const_iterator pt = ring.begin() + 1; pt < ring.end() - 1; pt++)
        {
            const pcl::PointXYZR& prev = *(pt - 1);
            const pcl::PointXYZR& succ = *(pt + 1);
            const float distance = std::max(pcl::euclideanDistance(prev, *pt), pcl::euclideanDistance(*pt, succ));
            if (distance >= config_.min_distance_between_edge_points &&
                distance <= config_.max_distance_between_edge_points)
            {
                edge_points.push_back(*pt);
            }
        }
    }

    if (edge_points.size() < 2 * lidar_reference_points_.size())
    {
        ROS_DEBUG_STREAM("Could not find enough edge points: only " << edge_points.size());
        return false;
    }

    // Publish debug point cloud
    edge_points.header = board_cloud.header;
    pub_edge_points_.publish(edge_points);

    return true;
}

bool HolePatternDetector::determineHolePositions(
    const pcl::PointCloud<pcl::PointXYZR>& edge_points,
    Eigen::Vector3d& translation,
    Eigen::Quaterniond& rotation,
    std::map<ReferencePointID, Eigen::Vector3d>& lidar_predicted_holes) const noexcept
{
    // Build optimization problem to get translation and rotation of calibration board
    // Assumption: Only one calibration board with lidar hole pattern
    ceres::Problem optimization_problem;
    optimization_problem.AddParameterBlock(translation.data(), 3);
    optimization_problem.AddParameterBlock(rotation.coeffs().data(), 4, new ceres::EigenQuaternionParameterization());

    // Add residual for every edge point, and init translation with mean of all edge points
    // Assumption: Board origin is in the middle of all holes
    {
        int i = 0;
        Eigen::Vector3d mean{Eigen::Vector3d::Zero()};
        for (const pcl::PointXYZR& p : edge_points)
        {
            const ceres::ResidualBlockId residual_block_id =
                optimization_problem.AddResidualBlock(new EdgePointError::CostFunction(new EdgePointError(
                                                          p.x, p.y, p.z, lidar_reference_points_, config_.r_limit)),
                                                      nullptr,
                                                      translation.data(),
                                                      rotation.coeffs().data());
            i++;

            mean.x() += p.x;
            mean.y() += p.y;
            mean.z() += p.z;
        }

        mean /= i;
        translation = mean;
    }

    // Init rotation based on gravity vector
    {
        Eigen::Vector3d x, y, z;

        const Eigen::Vector3d up_vector = lidar_from_base_link_.rotation().col(2);

        z = -translation.normalized();
        y = up_vector;
        x = y.cross(z);

        Eigen::Matrix3d rot;
        rot.col(0) = x;
        rot.col(1) = y;
        rot.col(2) = z;

        rotation = Eigen::Quaterniond(rot);
        rotation.normalize();
    }

    // Optimization: Estimate transformation between lidar and board and get hole positions in lidar coordinates
    {
        ceres::Solver::Options solve_options;
        solve_options.linear_solver_type = ceres::LinearSolverType::SPARSE_NORMAL_CHOLESKY;
        solve_options.max_num_iterations = 500;

        ceres::Solver::Summary summary;
        ceres::Solve(solve_options, &optimization_problem, &summary);
        ROS_DEBUG_STREAM(summary.BriefReport());

        if (summary.termination_type != ceres::CONVERGENCE)
        {
            ROS_DEBUG_STREAM("Could not determine hole positions in lidar point cloud: No convergence");
            return false;
        }

        const Eigen::Isometry3d lidar_from_board = Eigen::Translation3d(translation) * Eigen::Isometry3d(rotation);

        for (const auto& reference_point_elem : lidar_reference_points_)
        {
            const CartesianCoordinate& board_hole_position = reference_point_elem.second.position;
            lidar_predicted_holes[reference_point_elem.first] = lidar_from_board * board_hole_position;
        }
    }

    return true;
}

void HolePatternDetector::visualizeClusterBoundingBox(const pcl::PointXYZR& obb_position,
                                                      const Eigen::Matrix3f& obb_rotation,
                                                      const double large_side_length,
                                                      const double small_side_length,
                                                      const double depth,
                                                      const bool found_board,
                                                      const std_msgs::Header& header) noexcept
{
    const Eigen::Vector3f obb_x = obb_rotation.col(0);
    const Eigen::Vector3f obb_y = obb_rotation.col(1);
    const Eigen::Vector3f obb_z = obb_rotation.col(1);
    visualization_msgs::Marker marker;
    marker.header = header;
    marker.ns = "bounding_box";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = obb_position.x;
    marker.pose.position.y = obb_position.y;
    marker.pose.position.z = obb_position.z;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.color.a = 1.0;
    marker.color.r = found_board ? 0.0 : 1.0;
    marker.color.g = found_board ? 1.0 : 0.0;
    marker.color.b = 0.0;
    marker.scale.x = 0.05;

    auto to_ros_point = [](const Eigen::Vector3f eigen_point)
    {
        geometry_msgs::Point ros_p;
        ros_p.x = eigen_point.x();
        ros_p.y = eigen_point.y();
        ros_p.z = eigen_point.z();
        return ros_p;
    };

    marker.points.push_back(
        to_ros_point(obb_x * -0.5 * large_side_length + obb_y * -0.5 * small_side_length + obb_z * -0.5 * depth));
    marker.points.push_back(
        to_ros_point(obb_x * +0.5 * large_side_length + obb_y * -0.5 * small_side_length + obb_z * -0.5 * depth));
    marker.points.push_back(
        to_ros_point(obb_x * +0.5 * large_side_length + obb_y * +0.5 * small_side_length + obb_z * -0.5 * depth));
    marker.points.push_back(
        to_ros_point(obb_x * -0.5 * large_side_length + obb_y * +0.5 * small_side_length + obb_z * -0.5 * depth));
    marker.points.push_back(
        to_ros_point(obb_x * -0.5 * large_side_length + obb_y * -0.5 * small_side_length + obb_z * -0.5 * depth));
    marker.points.push_back(
        to_ros_point(obb_x * -0.5 * large_side_length + obb_y * -0.5 * small_side_length + obb_z * +0.5 * depth));
    marker.points.push_back(
        to_ros_point(obb_x * +0.5 * large_side_length + obb_y * -0.5 * small_side_length + obb_z * +0.5 * depth));
    marker.points.push_back(
        to_ros_point(obb_x * +0.5 * large_side_length + obb_y * -0.5 * small_side_length + obb_z * -0.5 * depth));
    marker.points.push_back(
        to_ros_point(obb_x * +0.5 * large_side_length + obb_y * -0.5 * small_side_length + obb_z * +0.5 * depth));
    marker.points.push_back(
        to_ros_point(obb_x * +0.5 * large_side_length + obb_y * +0.5 * small_side_length + obb_z * +0.5 * depth));
    marker.points.push_back(
        to_ros_point(obb_x * +0.5 * large_side_length + obb_y * +0.5 * small_side_length + obb_z * -0.5 * depth));
    marker.points.push_back(
        to_ros_point(obb_x * +0.5 * large_side_length + obb_y * +0.5 * small_side_length + obb_z * +0.5 * depth));
    marker.points.push_back(
        to_ros_point(obb_x * -0.5 * large_side_length + obb_y * +0.5 * small_side_length + obb_z * +0.5 * depth));
    marker.points.push_back(
        to_ros_point(obb_x * -0.5 * large_side_length + obb_y * +0.5 * small_side_length + obb_z * -0.5 * depth));
    marker.points.push_back(
        to_ros_point(obb_x * -0.5 * large_side_length + obb_y * +0.5 * small_side_length + obb_z * +0.5 * depth));
    marker.points.push_back(
        to_ros_point(obb_x * -0.5 * large_side_length + obb_y * -0.5 * small_side_length + obb_z * +0.5 * depth));

    pub_debug_marker_.publish(marker);
}

void HolePatternDetector::publishResults(const Eigen::Vector3d& translation,
                                         const Eigen::Quaterniond& rotation,
                                         const std::map<ReferencePointID, Eigen::Vector3d>& lidar_predicted_holes,
                                         const std_msgs::Header& header) noexcept
{
    // Publish hole positions as detections
    {
        sensor_calib_msgs::CartesianPointMeasurements board_measurements;
        board_measurements.header = header;

        for (const auto& p_elem : lidar_predicted_holes)
        {
            const Eigen::Vector3d& p = p_elem.second;
            sensor_calib_msgs::CartesianPointMeasurement point_meas;
            point_meas.id = p_elem.first;
            point_meas.point.x = p.x();
            point_meas.point.y = p.y();
            point_meas.point.z = p.z();
            board_measurements.measured_points.push_back(point_meas);
        }

        pub_cartesian_.publish(board_measurements);
    }

    // Publish hole positions as markers
    {
        visualization_msgs::Marker marker;
        marker.header = header;
        marker.ns = "reference_points";
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.color.a = 1.0;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;

        for (const auto& p_elem : lidar_predicted_holes)
        {
            const Eigen::Vector3d& p = p_elem.second;
            marker.id = p_elem.first;
            marker.pose.position.x = p.x();
            marker.pose.position.y = p.y();
            marker.pose.position.z = p.z();
            const double hole_diameter = lidar_reference_points_[p_elem.first].radius * 2;
            marker.scale.x = hole_diameter;
            marker.scale.y = hole_diameter;
            marker.scale.z = hole_diameter;
            pub_marker_.publish(marker);
        }
    }

    // Publish board position as tf
    {
        geometry_msgs::TransformStamped transform_stamped;
        transform_stamped.header = header;
        transform_stamped.child_frame_id = "hole_board_" + header.frame_id;

        transform_stamped.transform.translation.x = translation.x();
        transform_stamped.transform.translation.y = translation.y();
        transform_stamped.transform.translation.z = translation.z();

        transform_stamped.transform.rotation.x = rotation.x();
        transform_stamped.transform.rotation.y = rotation.y();
        transform_stamped.transform.rotation.z = rotation.z();
        transform_stamped.transform.rotation.w = rotation.w();
        transform_broadcaster_.sendTransform(transform_stamped);
    }
}
