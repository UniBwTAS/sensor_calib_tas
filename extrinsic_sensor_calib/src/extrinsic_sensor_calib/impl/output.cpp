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

#include <fstream>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc.hpp>

#include <visualization_msgs/Marker.h>

#include <sensor_calib_msgs/DetectionStati.h>
#include <sensor_calib_msgs/EstimatedPoses.h>
#include <sensor_calib_msgs/MeasurementIndices.h>

#include "../cost_functors.hpp"
#include "../sensor_calib.h"

void SensorCalib::publishResults() noexcept
{
    // Publish the status message again to keep the GUI updated.
    publishStatusMsg();

    // Publish detection stati (to GUI)
    {
        sensor_calib_msgs::DetectionStati detection_stati_msg;
        detection_stati_msg.header.stamp = ros::Time::now();
        for (const auto& detection_stati_elem : detection_stati_)
        {
            detection_stati_msg.detection_stati.push_back(detection_stati_elem.second);
        }
        detection_status_pub_.publish(detection_stati_msg);
    }

    geometry_msgs::TransformStamped transform_stamped;
    transform_stamped.header.stamp = ros::Time::now();
    transform_stamped.header.frame_id = reference_frame_id_;

    // Publish board poses (to TF and GUI)
    if (board_poses_changed_)
    {
        sensor_calib_msgs::EstimatedPoses board_poses_msg;

        for (const auto& board_poses_elem : board_poses_)
        {
            const BoardID& board_id = board_poses_elem.first;
            const std::map<MeasurementIndex, Pose>& board_poses = board_poses_elem.second;

            for (const auto& board_pose_elem : board_poses)
            {
                const MeasurementIndex t = board_pose_elem.first;
                const Pose& board_pose = board_pose_elem.second;

                transform_stamped.child_frame_id = board_id + "_" + timeIndexStr(t);

                transform_stamped.transform.translation.x = board_pose.translation.x();
                transform_stamped.transform.translation.y = board_pose.translation.y();
                transform_stamped.transform.translation.z = board_pose.translation.z();

                transform_stamped.transform.rotation.x = board_pose.rotation.x();
                transform_stamped.transform.rotation.y = board_pose.rotation.y();
                transform_stamped.transform.rotation.z = board_pose.rotation.z();
                transform_stamped.transform.rotation.w = board_pose.rotation.w();

                board_poses_msg.poses.push_back(transform_stamped);

                static_transform_broadcaster_.sendTransform(transform_stamped);
            }
        }

        board_poses_pub_.publish(board_poses_msg);
        board_poses_changed_ = false;
    }

    // Publish sensor poses (to TF and GUI)
    if (sensor_poses_changed_)
    {
        sensor_calib_msgs::EstimatedPoses sensor_poses_msg;

        // Publish sensor poses to TF (on separate _calibrated frames)
        publishSensorPosesToTf("_calibrated");

        // Publish sensor poses for GUI
        for (const auto& sensor_poses_elem : sensor_poses_)
        {
            const std::pair<FrameID, FrameID> frame_ids = sensor_poses_elem.first;
            const Pose& sensor_pose = sensor_poses_elem.second;

            const FrameID& from_frame_id = frame_ids.first;
            const FrameID& to_frame_id = frame_ids.second;

            if (from_frame_id == to_frame_id)
            {
                continue;
            }

            Pose pose = sensor_pose;

            // For GUI, convert from camera coordinates (z pointing forward) to vehicle coordinates (x pointing forward)
            const Eigen::Quaterniond cam_to_veh_rotation{-0.5, -0.5, 0.5, -0.5};
            if (isCamera(frame_ids.first))
            {
                pose.translation = cam_to_veh_rotation.inverse() * pose.translation;
                pose.rotation = cam_to_veh_rotation.inverse() * pose.rotation;
            }
            if (isCamera(frame_ids.second))
            {
                // The translation stays the same, if the child frame has to be rotated!
                pose.rotation = pose.rotation * cam_to_veh_rotation;
            }

            transform_stamped.header.frame_id = frame_ids.first;
            transform_stamped.child_frame_id = frame_ids.second;

            transform_stamped.transform.translation.x = pose.translation.x();
            transform_stamped.transform.translation.y = pose.translation.y();
            transform_stamped.transform.translation.z = pose.translation.z();

            transform_stamped.transform.rotation.x = pose.rotation.x();
            transform_stamped.transform.rotation.y = pose.rotation.y();
            transform_stamped.transform.rotation.z = pose.rotation.z();
            transform_stamped.transform.rotation.w = pose.rotation.w();

            sensor_poses_msg.poses.push_back(transform_stamped);
        }

        sensor_poses_pub_.publish(sensor_poses_msg);
        sensor_poses_changed_ = false;
    }

    // Publish markers and debug image
    {
        const cv::viz::Color measurement_color = cv::viz::Color::yellow();
        const cv::viz::Color prediction_color = cv::viz::Color::green();
        const cv::viz::Color error_color = cv::viz::Color::black();
        const cv::viz::Color accumulated_color = cv::viz::Color::white();

        visualizeMeasurements(measurement_color);
        visualizeExpectedMeasurements(prediction_color);
        visualizeImageMeasurements(measurement_color, prediction_color, accumulated_color, error_color);
    }

    // Publish measurement ids
    if (measurements_changed_)
    {
        sensor_calib_msgs::MeasurementIndices msg;
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = reference_frame_id_;

        auto get_indices = [&](const auto& measurements)
        {
            for (const auto& [idx, val] : measurements)
            {
                sensor_calib_msgs::MeasurementIndex idx_msg;
                idx_msg.measurement_idx = idx;
                for (int id = 0; id < val.size(); id++)
                {
                    idx_msg.measurement_id = id;
                    msg.measurement_indices.push_back(idx_msg);
                }
            }
        };

        if (isLiDAR(reference_frame_id_))
            get_indices(lidar_measurements_[reference_frame_id_]);
        else if (isCamera(reference_frame_id_))
            get_indices(camera_measurements_[reference_frame_id_]);
        else
            get_indices(radar_measurements_[reference_frame_id_]);

        measurement_indices_pub_.publish(msg);

        measurements_changed_ = false;
    }
}

void SensorCalib::publishStatusMsg(const std::string& status_msg) const noexcept
{
    std_msgs::String msg;
    msg.data = status_msg;
    status_pub_.publish(msg);
}

void SensorCalib::publishSensorPosesToTf(const std::string& suffix, const bool initial) noexcept
{
    geometry_msgs::TransformStamped transform_stamped;
    transform_stamped.header.stamp = ros::Time::now();

    const std::map<std::pair<FrameID, FrameID>, Pose>& sensor_poses = initial ? initial_sensor_poses_ : sensor_poses_;

    for (const auto& sensor_poses_elem : sensor_poses)
    {
        const std::pair<FrameID, FrameID> frame_ids = sensor_poses_elem.first;
        const Pose& sensor_pose = sensor_poses_elem.second;

        if (frame_ids.first == frame_ids.second)
        {
            continue;
        }

        transform_stamped.header.frame_id = frame_ids.first;
        transform_stamped.child_frame_id = frame_ids.second;

        transform_stamped.transform.translation.x = sensor_pose.translation.x();
        transform_stamped.transform.translation.y = sensor_pose.translation.y();
        transform_stamped.transform.translation.z = sensor_pose.translation.z();

        transform_stamped.transform.rotation.x = sensor_pose.rotation.x();
        transform_stamped.transform.rotation.y = sensor_pose.rotation.y();
        transform_stamped.transform.rotation.z = sensor_pose.rotation.z();
        transform_stamped.transform.rotation.w = sensor_pose.rotation.w();

        // Publish new_calibrated frames
        if (!suffix.empty())
        {
            if (transform_stamped.header.frame_id != reference_frame_id_)
            {
                transform_stamped.header.frame_id += suffix;
            }

            if (transform_stamped.child_frame_id != reference_frame_id_)
            {
                transform_stamped.child_frame_id += suffix;
            }

            static_transform_broadcaster_.sendTransform(transform_stamped);
        }

        // Additionally, overwrite real tf frame
        if (preview_ || initial)
        {
            transform_stamped.header.frame_id = frame_ids.first;
            transform_stamped.child_frame_id = frame_ids.second;
            static_transform_broadcaster_.sendTransform(transform_stamped);
        }
    }
}

void SensorCalib::visualizeMeasurements(const cv::viz::Color& measurement_color) noexcept
{
    std_msgs::ColorRGBA color;
    color.a = 1.0;
    color.r = measurement_color[2] * 255;
    color.g = measurement_color[1] * 255;
    color.b = measurement_color[0] * 255;

    // Publish LiDAR measurement with specified id (otherwise last)
    for (const auto& lidar_measurements_elem : lidar_measurements_)
    {
        const FrameID& frame_id = lidar_measurements_elem.first;
        const std::map<MeasurementIndex, std::vector<LiDARMeasurement>>& measurements_for_lidar =
            lidar_measurements_elem.second;

        if (measurements_for_lidar.size() > 0)
        {
            visualization_msgs::Marker marker;
            marker.header.frame_id = frame_id;
            marker.header.stamp = ros::Time::now();
            marker.ns = "measured_lidar_reference_points_" + frame_id;
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.orientation.w = 1.0;
            marker.color = color;

            int measurement_idx = std::min(measurement_idx_to_debug_, num_board_poses_ - 1);
            if (measurement_idx == -1)
            {
                measurement_idx = num_board_poses_ - 1;
            }

            const std::vector<LiDARMeasurement>& measurements_for_lidar_for_measidx =
                measurements_for_lidar.at(measurement_idx);

            int measurement_id = std::min(measurement_id_to_debug_, int(measurements_for_lidar_for_measidx.size()) - 1);
            if (measurement_id == -1)
            {
                measurement_id = measurements_for_lidar_for_measidx.size() - 1;
            }

            const LiDARMeasurement& selected_measurement = measurements_for_lidar_for_measidx.at(measurement_id);

            for (const auto& board_elem : selected_measurement)
            {
                const BoardID& board_id = board_elem.first;

                for (const auto& measured_point_elem : board_elem.second)
                {
                    const ReferencePointID& reference_point_id = measured_point_elem.first;
                    marker.id = reference_point_id;

                    const CartesianCoordinate& measured_point = measured_point_elem.second;
                    marker.pose.position.x = measured_point.x();
                    marker.pose.position.y = measured_point.y();
                    marker.pose.position.z = measured_point.z();

                    const double hole_radius =
                        calibration_boards_[board_id].lidar_reference_points[reference_point_id].radius;
                    marker.scale.x = hole_radius * 2;
                    marker.scale.y = hole_radius * 2;
                    marker.scale.z = hole_radius * 2;

                    marker_pub_.publish(marker);
                }
            }
        }
    }

    // Publish radar measurement with specified id (otherwise last)
    for (const auto& radar_measurements_elem : radar_measurements_)
    {
        const FrameID& frame_id = radar_measurements_elem.first;
        const std::map<MeasurementIndex, std::vector<RadarMeasurement>>& measurements_for_radar =
            radar_measurements_elem.second;

        if (measurements_for_radar.size() > 0)
        {
            visualization_msgs::Marker marker;
            marker.header.frame_id = frame_id;
            marker.header.stamp = ros::Time::now();
            marker.ns = "measured_radar_targets_" + frame_id;
            marker.id = 0;
            marker.type = visualization_msgs::Marker::SPHERE_LIST;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = 0.1;
            marker.scale.y = 0.1;
            marker.scale.z = 0.1;
            marker.color = color;

            int measurement_idx = std::min(measurement_idx_to_debug_, num_board_poses_ - 1);
            if (measurement_idx == -1)
            {
                measurement_idx = num_board_poses_ - 1;
            }

            const std::vector<RadarMeasurement>& measurements_for_radar_for_measidx =
                measurements_for_radar.at(measurement_idx);

            int measurement_id = std::min(measurement_id_to_debug_, int(measurements_for_radar_for_measidx.size()) - 1);
            if (measurement_id == -1)
            {
                measurement_id = measurements_for_radar_for_measidx.size() - 1;
            }

            const RadarMeasurement& selected_measurement = measurements_for_radar_for_measidx.at(measurement_id);

            for (const auto& board_elem : selected_measurement)
            {
                for (const auto& target_elem : board_elem.second)
                {
                    const PolarCoordinate& radar_target = target_elem.second;
                    const Eigen::Vector3d radar_target_cartesian = radar_target.cartesian();

                    geometry_msgs::Point p;
                    p.x = radar_target_cartesian.x();
                    p.y = radar_target_cartesian.y();
                    p.z = radar_target_cartesian.z();
                    marker.points.push_back(p);
                }
            }

            if (marker.points.size() > 0)
            {
                marker_pub_.publish(marker);
            }
        }
    }
}

void SensorCalib::visualizeExpectedMeasurements(const cv::viz::Color& prediction_color) noexcept
{
    std_msgs::ColorRGBA color;
    color.a = 1.0;
    color.r = prediction_color[2] * 255;
    color.g = prediction_color[1] * 255;
    color.b = prediction_color[0] * 255;

    // Publish expected apriltag positions
    for (const auto& board_elem : calibration_boards_)
    {
        const BoardID& board_id = board_elem.first;
        const BoardConfig& board_config = board_elem.second;

        if (board_poses_.count(board_id) && board_config.apriltags.size() > 0)
        {
            int measurement_idx = std::min(measurement_idx_to_debug_, int(board_poses_.at(board_id).size()) - 1);
            if (measurement_idx == -1)
            {
                measurement_idx = board_poses_.at(board_id).size() - 1;
            }

            visualization_msgs::Marker marker;
            marker.header.frame_id = board_id + "_" + timeIndexStr(measurement_idx);
            marker.header.stamp = ros::Time::now();
            marker.ns = "apriltags_from_" + board_id;
            marker.id = 0;
            marker.type = visualization_msgs::Marker::CUBE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.orientation.w = 1.0;
            marker.scale.z = 0.01;
            marker.color = color;

            for (const auto& tag_elem : board_config.apriltags)
            {
                const TagID& tag_id = tag_elem.first;
                marker.id = std::stoi(tag_id);

                const AprilTagConfig& tag_config = tag_elem.second;
                marker.pose.position.x = tag_config.board_from_tag().translation().x();
                marker.pose.position.y = tag_config.board_from_tag().translation().y();
                marker.pose.position.z = tag_config.board_from_tag().translation().z();

                const double tag_size = board_config.apriltags.at(tag_id).size;
                marker.scale.x = tag_size;
                marker.scale.y = tag_size;

                marker_pub_.publish(marker);
            }
        }
    }

    // Publish expected lidar measurement
    {
        for (const auto& board_elem : calibration_boards_)
        {
            const BoardID& board_id = board_elem.first;
            if (board_poses_.count(board_id))
            {
                int measurement_idx = std::min(measurement_idx_to_debug_, int(board_poses_.at(board_id).size()) - 1);
                if (measurement_idx == -1)
                {
                    measurement_idx = board_poses_.at(board_id).size() - 1;
                }

                const BoardConfig& board_config = board_elem.second;

                visualization_msgs::Marker marker;
                marker.header.frame_id = board_id + "_" + timeIndexStr(measurement_idx);
                marker.header.stamp = ros::Time::now();
                marker.ns = "lidar_reference_points_from_" + board_id;
                marker.type = visualization_msgs::Marker::SPHERE;
                marker.action = visualization_msgs::Marker::ADD;
                marker.pose.orientation.w = 1.0;
                marker.color = color;

                for (const auto& target_elem : board_config.lidar_reference_points)
                {
                    const ReferencePointID& reference_point_id = target_elem.first;
                    marker.id = reference_point_id;

                    const Eigen::Vector3d& target_pos = target_elem.second.position;
                    marker.pose.position.x = target_pos.x();
                    marker.pose.position.y = target_pos.y();
                    marker.pose.position.z = target_pos.z();

                    const double hole_radius =
                        calibration_boards_[board_id].lidar_reference_points[reference_point_id].radius;
                    marker.scale.x = hole_radius * 2;
                    marker.scale.y = hole_radius * 2;
                    marker.scale.z = hole_radius * 2;

                    marker_pub_.publish(marker);
                }
            }
        }
    }

    // Publish expected radar measurement
    {
        for (const auto& board_elem : calibration_boards_)
        {
            const BoardID& board_id = board_elem.first;
            if (board_poses_.count(board_id))
            {
                int measurement_idx = std::min(measurement_idx_to_debug_, int(board_poses_.at(board_id).size()) - 1);
                if (measurement_idx == -1)
                {
                    measurement_idx = board_poses_.at(board_id).size() - 1;
                }

                const BoardConfig& board_config = board_elem.second;
                visualization_msgs::Marker marker;
                marker.header.frame_id = board_id + "_" + timeIndexStr(measurement_idx);
                marker.header.stamp = ros::Time::now();
                marker.ns = "radar_targets_from_" + board_id;
                marker.id = 0;
                marker.type = visualization_msgs::Marker::SPHERE_LIST;
                marker.action = visualization_msgs::Marker::ADD;
                marker.pose.orientation.w = 1.0;
                marker.scale.x = 0.1;
                marker.scale.y = 0.1;
                marker.scale.z = 0.01;
                marker.color = color;

                for (const auto& target_elem : board_config.radar_targets)
                {
                    const Eigen::Vector3d& target_pos = target_elem.second;

                    geometry_msgs::Point p;
                    p.x = target_pos.x();
                    p.y = target_pos.y();
                    p.z = target_pos.z();
                    marker.points.push_back(p);
                }

                marker_pub_.publish(marker);
            }
        }
    }
}

void SensorCalib::visualizeImageMeasurements(const cv::viz::Color& measurement_color,
                                             const cv::viz::Color& prediction_color,
                                             const cv::viz::Color& accumulated_color,
                                             const cv::viz::Color& error_color) noexcept
{
    for (const std::string& camera_frame_id : camera_frame_ids_)
    {
        if (debug_image_pub_.count(camera_frame_id) == 0)
        {
            continue;
        }

        if (debug_image_pub_.at(camera_frame_id).getNumSubscribers() == 0)
        {
            continue;
        }

        cv::Mat result_img;

        int measurement_idx = std::min(measurement_idx_to_debug_, int(num_board_poses_) - 1);
        if (measurement_idx == -1)
        {
            measurement_idx = num_board_poses_ - 1;
        }

        result_img = cv::Mat(
            cv::Size(camera_intrinsics_[camera_frame_id].img_width, camera_intrinsics_[camera_frame_id].img_height),
            CV_8UC3);
        result_img.setTo(cv::Scalar(128, 128, 128));

        if (camera_measurements_.count(camera_frame_id))
        {
            const std::map<MeasurementIndex, std::vector<CameraMeasurement>>& measurements_for_camera =
                camera_measurements_.at(camera_frame_id);

            // Measured vs. predicted image positions
            if (measurement_idx >= 0 && measurements_for_camera.count(measurement_idx))
            {
                const std::vector<CameraMeasurement>& measurements_for_camera_for_measidx =
                    measurements_for_camera.at(measurement_idx);

                int measurement_id =
                    std::min(measurement_id_to_debug_, int(measurements_for_camera_for_measidx.size()) - 1);
                if (measurement_id == -1)
                {
                    measurement_id = measurements_for_camera_for_measidx.size() - 1;
                }

                if (config_.show_camera_image_in_debug_image && //
                    (camera_measurement_images_.count(camera_frame_id) > 0) &&
                    (camera_measurement_images_.at(camera_frame_id).count(measurement_idx)) &&
                    (camera_measurement_images_.at(camera_frame_id).at(measurement_idx).size() > measurement_id) &&
                    (!camera_measurement_images_.at(camera_frame_id).at(measurement_idx)[measurement_id].empty()))
                {
                    result_img =
                        camera_measurement_images_.at(camera_frame_id).at(measurement_idx)[measurement_id].clone();
                }

                const CameraMeasurement& camera_board_measurement = measurements_for_camera_for_measidx[measurement_id];
                for (const auto& camera_board_measurement_elem : camera_board_measurement)
                {
                    const BoardID& board_id = camera_board_measurement_elem.first;
                    const CameraBoardMeasurement& tag_measurements = camera_board_measurement_elem.second;

                    if (board_poses_.count(board_id) == 0 || board_poses_.at(board_id).count(measurement_idx) == 0)
                    {
                        continue;
                    }

                    for (const auto& tag_measurements_elem : tag_measurements)
                    {
                        const TagID& tag_id = tag_measurements_elem.first;
                        const TagMeasurement& tag_measurement = tag_measurements_elem.second;

                        const AprilTagConfig& tag_config = calibration_boards_.at(board_id).apriltags.at(tag_id);

                        std::array<std::pair<cv::Point, cv::Point>, 4> pts;

                        for (int i = 0; i < 4; i++)
                        {
                            Eigen::Vector3d tag_corner_position;
                            const double half_tag_size = tag_config.size / 2.0;

                            switch (i)
                            {
                                case 0:
                                    tag_corner_position = Eigen::Vector3d(-half_tag_size, -half_tag_size, 0.0);
                                    break;

                                case 1:
                                    tag_corner_position = Eigen::Vector3d(+half_tag_size, -half_tag_size, 0.0);
                                    break;

                                case 2:
                                    tag_corner_position = Eigen::Vector3d(+half_tag_size, +half_tag_size, 0.0);
                                    break;

                                case 3:
                                default:
                                    tag_corner_position = Eigen::Vector3d(-half_tag_size, +half_tag_size, 0.0);
                                    break;
                            }
                            const Eigen::Vector3d board_point_position =
                                tag_config.board_from_tag() * tag_corner_position;
                            const Eigen::Isometry3d ref_from_board =
                                Eigen::Translation3d(board_poses_.at(board_id).at(measurement_idx).translation) *
                                Eigen::Isometry3d(board_poses_.at(board_id).at(measurement_idx).rotation);
                            const Eigen::Isometry3d camera_from_ref =
                                sensor_poses_.at(std::make_pair(reference_frame_id_, camera_frame_id)).transform();

                            const Eigen::Vector3d camera_point =
                                camera_from_ref * ref_from_board * board_point_position;

                            const Eigen::Vector2d predicted_img_pos = CameraTagReprojectionError::predictCornerPosition(
                                camera_point, camera_intrinsics_.at(camera_frame_id));
                            pts[i].first = cv::Point2i(predicted_img_pos.x(), predicted_img_pos.y());
                        }

                        // Measured image points
                        int i = 0;
                        for (const auto& corner_measurement : tag_measurement)
                        {
                            pts[i].second = cv::Point2f(corner_measurement.x(), corner_measurement.y());

                            i++;
                        }

                        // Draw the points
                        for (const auto& p : pts)
                        {
                            // Connection line
                            if (config_.show_error_lines)
                            {
                                cv::line(result_img, p.first, p.second, error_color, 2);
                            }

                            // Predicted
                            if (config_.show_pred_corner_points)
                            {
                                cv::circle(result_img, p.first, 6, prediction_color, -1);
                                cv::circle(result_img, p.first, 7, cv::viz::Color::black(), 2);
                            }

                            // Measured
                            if (config_.show_measured_corner_points)
                            {
                                cv::circle(result_img, p.second, 6, measurement_color, -1);
                                cv::circle(result_img, p.second, 7, cv::viz::Color::black(), 2);
                            }
                        }
                    }
                }
            }

            // All accumulated measurements
            if (config_.show_all_measurements)
            {
                for (const auto& measurements_for_camera_for_measidx_elem : measurements_for_camera)
                {
                    const std::vector<CameraMeasurement>& measurements_for_camera_for_measidx =
                        measurements_for_camera_for_measidx_elem.second;

                    for (const CameraMeasurement& measurement_for_camera_for_measidx :
                         measurements_for_camera_for_measidx)
                    {
                        for (const auto& measurement_for_camera_for_measidx_for_board_elem :
                             measurement_for_camera_for_measidx)
                        {
                            const BoardID& board_id = measurement_for_camera_for_measidx_for_board_elem.first;
                            const CameraBoardMeasurement& tag_measurements =
                                measurement_for_camera_for_measidx_for_board_elem.second;

                            for (const auto& tag_measurements_elem : tag_measurements)
                            {
                                const TagMeasurement& tag_measurement = tag_measurements_elem.second;

                                for (const auto& corner_measurement : tag_measurement)
                                {
                                    cv::circle(result_img,
                                               cv::Point(std::round(corner_measurement.x()),
                                                         std::round(corner_measurement.y())),
                                               5,
                                               accumulated_color,
                                               -1);

                                    cv::circle(result_img,
                                               cv::Point(std::round(corner_measurement.x()),
                                                         std::round(corner_measurement.y())),
                                               7,
                                               cv::viz::Color::black(),
                                               2);
                                }
                            }
                        }
                    }
                }
            }
        }

        std_msgs::Header header;
        header.frame_id = camera_frame_id;
        header.stamp = ros::Time::now();

        cv_bridge::CvImage cv_image(header, "bgr8", result_img);
        debug_image_pub_[camera_frame_id].publish(cv_image.toImageMsg());
    }
}