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

#include <boost/serialization/array.hpp>

#define EIGEN_DENSEBASE_PLUGIN "EigenDenseBaseAddons.h"

#include <fstream>
#include <future>

#include <cv_bridge/cv_bridge.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/serialization/map.hpp>
#include <boost/serialization/set.hpp>
#include <boost/serialization/vector.hpp>

#include <image_geometry/pinhole_camera_model.h>

#include <sensor_calib_msgs/DetectionStatus.h>
#include <sensor_calib_msgs/PolarPointMeasurement.h>

#include "../sensor_calib.h"

void SensorCalib::reconfigure(extrinsic_sensor_calib::ExtrinsicSensorCalibConfig& config, uint32_t level)
{
    config_ = config;

    calibrate(false);
}

void SensorCalib::callbackAprilTagDetections(const apriltags_msgs::AprilTagDetections::ConstPtr& msg) noexcept
{
    current_apriltag_detections_[msg->header.frame_id] = msg;

    sensor_calib_msgs::DetectionStatus status_msg;
    status_msg.sensor = msg->header.frame_id;
    status_msg.stamp = msg->header.stamp;
    status_msg.num_detections = msg->detections.size();
    detection_stati_[msg->header.frame_id] = status_msg;
}

void SensorCalib::callbackLiDARTargetDetections(
    const sensor_calib_msgs::CartesianPointMeasurements::ConstPtr& msg) noexcept
{
    current_lidar_reference_point_detections_[msg->header.frame_id] = msg;

    sensor_calib_msgs::DetectionStatus status_msg;
    status_msg.sensor = msg->header.frame_id;
    status_msg.stamp = msg->header.stamp;
    status_msg.num_detections = msg->measured_points.size();
    detection_stati_[msg->header.frame_id] = status_msg;
}

void SensorCalib::callbackRadarTargetDetections(const sensor_calib_msgs::PolarPointMeasurements::ConstPtr& msg) noexcept
{
    current_radar_target_detections_[msg->header.frame_id] = msg;

    sensor_calib_msgs::DetectionStatus status_msg;
    status_msg.sensor = msg->header.frame_id;
    status_msg.stamp = msg->header.stamp;
    status_msg.num_detections = msg->measured_points.size();
    detection_stati_[msg->header.frame_id] = status_msg;
}

void SensorCalib::measureRequest(const std_msgs::Bool::ConstPtr& new_board_pose) noexcept
{
    measure(new_board_pose->data);
    calibrate(false);
    measurements_changed_ = true;
}

void SensorCalib::removeLastMeasurementRequest(const std_msgs::Empty::ConstPtr&) noexcept
{
    deleteLastMeasurement();
    calibrate(false);
    measurements_changed_ = true;
}

void SensorCalib::solveRequest(const std_msgs::Empty::ConstPtr&) noexcept
{
    calibrate(true);
    calibrate(false);
}

void SensorCalib::verifyRequest(const std_msgs::Empty::ConstPtr&) noexcept
{
    const bool set_all_sensor_poses_constant = config_.set_all_sensor_poses_constant;
    const bool set_all_board_poses_constant = config_.set_all_board_poses_constant;

    config_.set_all_sensor_poses_constant = true;
    config_.set_all_board_poses_constant = false;

    // Calculate the validation => Optimize the board poses with constant calibration
    calibrate(true);

    config_.set_all_sensor_poses_constant = set_all_sensor_poses_constant;
    config_.set_all_board_poses_constant = set_all_board_poses_constant;

    // Update the residuals without optimizing
    calibrate(false);
}

void SensorCalib::resetSensorPoseRequest(const sensor_calib_msgs::Reset::ConstPtr& msg) noexcept
{
    if (msg->from_frame_id == "" && msg->to_frame_id == "")
    {
        // Reset all sensor poses
        if (msg->zero) // to zero
        {
            resetSensorPosesToZero();
        }
        else if (msg->current_tf) // to current tf
        {
            const std::map<std::pair<FrameID, FrameID>, Pose> initial_sensor_poses_save = initial_sensor_poses_;
            initSensorPoses();
            initial_sensor_poses_ = initial_sensor_poses_save;
        }
        else // to initial sensor poses
        {
            sensor_poses_ = initial_sensor_poses_;
        }
    }
    else
    {
        // Reset one sensor pose
        if (msg->zero) // to zero
        {
            sensor_poses_[std::make_pair(msg->from_frame_id, msg->to_frame_id)] = Pose();

            if (isCamera(msg->to_frame_id))
            {
                // Avoid division by 0
                sensor_poses_[std::make_pair(msg->from_frame_id, msg->to_frame_id)].translation.z() = 1;
            }
            if (isRadar(msg->to_frame_id) && config_.set_radar_z_constant)
            {
                const Pose& original_pose = initial_sensor_poses_[std::make_pair(msg->from_frame_id, msg->to_frame_id)];
                sensor_poses_[std::make_pair(msg->from_frame_id, msg->to_frame_id)].translation.z() =
                    original_pose.translation.z();
            }
        }
        else if (msg->current_tf) // to current tf
        {

            sensor_poses_[std::make_pair(msg->from_frame_id, msg->to_frame_id)] =
                getPoseFromTF(msg->from_frame_id, msg->to_frame_id);
        }
        else // to initial sensor pose
        {
            sensor_poses_[std::make_pair(msg->from_frame_id, msg->to_frame_id)] =
                initial_sensor_poses_[std::make_pair(msg->from_frame_id, msg->to_frame_id)];
        }
    }

    sensor_poses_changed_ = true;

    calibrate(false);
}

void SensorCalib::resetBoardPoseRequest(const sensor_calib_msgs::Reset::ConstPtr& msg) noexcept
{
    if (msg->from_frame_id == "" && msg->to_frame_id == "")
    {
        // Reset all board poses
        board_poses_ = initial_board_poses_;
    }
    else
    {
        // Reset one board pose
        const BoardID& board_id = msg->to_frame_id.substr(0, msg->to_frame_id.size() - 4);
        const MeasurementIndex meas_idx =
            std::stoi(msg->to_frame_id.substr(msg->to_frame_id.size() - 3, msg->to_frame_id.size()));

        board_poses_[board_id][meas_idx] = initial_board_poses_[board_id][meas_idx];
    }

    board_poses_changed_ = true;

    calibrate(false);
}

void SensorCalib::previewRequest(const std_msgs::Bool::ConstPtr& msg) noexcept
{
    preview_ = msg->data;
    publishSensorPosesToTf(std::string(), !preview_);
}

void SensorCalib::updateUsedMeasurements(const sensor_calib_msgs::UseResiduals::ConstPtr& msg) noexcept
{
    for (const sensor_calib_msgs::UseResidual& r : msg->use_residuals)
    {
        use_measurement_[std::make_pair(r.group, r.name)] = r.use;
    }
}

void SensorCalib::updateOptimizedSensorPoses(const sensor_calib_msgs::OptimizeSensorPoses::ConstPtr& msg) noexcept
{
    for (const sensor_calib_msgs::OptimizeSensorPose& p : msg->optimize_sensor_poses)
    {
        optimize_sensor_pose_[std::make_pair(p.from_frame, p.to_frame)] =
            std::make_pair(p.optimize_translation, p.optimize_rotation);
    }
}

void SensorCalib::updateVisualizedMeasurement(const sensor_calib_msgs::MeasurementIndex::ConstPtr& msg) noexcept
{
    measurement_idx_to_debug_ = msg->measurement_idx;
    measurement_id_to_debug_ = msg->measurement_id;
}

void SensorCalib::saveToFileRequest(const std_msgs::String::ConstPtr& msg) const noexcept
{
    ROS_INFO_STREAM_NAMED("IO", "Saving to file '" << msg->data << "'");

    try
    {
        std::ofstream ofs_full(msg->data);
        boost::archive::binary_oarchive oa_full(ofs_full);

        oa_full << *this;
    }
    catch (boost::archive::archive_exception& e)
    {
        ROS_WARN_STREAM_NAMED("IO", "Exception opening file: " << e.what());
    }
}

void SensorCalib::loadFromFileRequest(const std_msgs::String::ConstPtr& msg) noexcept
{
    ROS_INFO_STREAM_NAMED("IO", "Loading from file '" << msg->data << "'");

    try
    {
        std::ifstream ifs(msg->data);
        boost::archive::binary_iarchive ia(ifs);

        if (num_board_poses_ == 0)
        {
            ia >> *this;
        }
        else
        {
            SensorCalib other_camera_calib(nh_);
            ia >> other_camera_calib;

            for (const auto& map_elem : other_camera_calib.board_poses_)
            {
                const BoardID& board_id = map_elem.first;
                const std::map<MeasurementIndex, Pose>& pose_map = map_elem.second;
                for (auto& pose_map_elem : pose_map)
                {
                    const MeasurementIndex meas_idx = pose_map_elem.first;
                    const Pose& pose = pose_map_elem.second;

                    if (other_camera_calib.reference_frame_id_ == reference_frame_id_)
                    {
                        board_poses_[board_id][meas_idx + num_board_poses_] = pose;
                    }
                    else
                    {
                        const Eigen::Isometry3d other_reference_from_board =
                            Eigen::Translation3d(pose.translation) * Eigen::Isometry3d(pose.rotation);

                        const FrameID other_reference_frame_id = other_camera_calib.reference_frame_id_;

                        const Eigen::Isometry3d reference_from_other_reference =
                            Eigen::Translation3d(
                                sensor_poses_.at(std::make_pair(reference_frame_id_, other_reference_frame_id))
                                    .translation) *
                            Eigen::Isometry3d(
                                sensor_poses_.at(std::make_pair(reference_frame_id_, other_reference_frame_id))
                                    .rotation);

                        const Eigen::Isometry3d ref_from_board =
                            reference_from_other_reference * other_reference_from_board;

                        Pose ref_to_board;
                        ref_to_board.translation = ref_from_board.translation();
                        ref_to_board.rotation = ref_from_board.rotation();

                        board_poses_[board_id][meas_idx + num_board_poses_] = ref_to_board;
                    }
                }
            }
            for (const auto& map_elem : other_camera_calib.initial_board_poses_)
            {
                const BoardID& board_id = map_elem.first;
                const std::map<MeasurementIndex, Pose>& pose_map = map_elem.second;
                for (const auto& pose_map_elem : pose_map)
                {
                    const MeasurementIndex meas_idx = pose_map_elem.first;
                    const Pose& pose = pose_map_elem.second;

                    if (other_camera_calib.reference_frame_id_ == reference_frame_id_)
                    {
                        initial_board_poses_[board_id][meas_idx + num_board_poses_] = pose;
                    }
                    else
                    {
                        const Eigen::Isometry3d other_reference_from_board =
                            Eigen::Translation3d(pose.translation) * Eigen::Isometry3d(pose.rotation);

                        const FrameID other_reference_frame_id = other_camera_calib.reference_frame_id_;

                        const Eigen::Isometry3d reference_from_other_reference =
                            Eigen::Translation3d(
                                sensor_poses_.at(std::make_pair(reference_frame_id_, other_reference_frame_id))
                                    .translation) *
                            Eigen::Isometry3d(
                                sensor_poses_.at(std::make_pair(reference_frame_id_, other_reference_frame_id))
                                    .rotation);

                        const Eigen::Isometry3d ref_from_board =
                            reference_from_other_reference * other_reference_from_board;

                        Pose ref_to_board;
                        ref_to_board.translation = ref_from_board.translation();
                        ref_to_board.rotation = ref_from_board.rotation();

                        initial_board_poses_[board_id][meas_idx + num_board_poses_] = ref_to_board;
                    }
                }
            }

            if (other_camera_calib.lidar_measurements_.size() > 0)
            {
                for (const auto& map_elem : other_camera_calib.lidar_measurements_)
                {
                    const FrameID& frame_id = map_elem.first;
                    const std::map<MeasurementIndex, std::vector<LiDARMeasurement>>& meas_map = map_elem.second;
                    for (const auto& meas_map_elem : meas_map)
                    {
                        const MeasurementIndex meas_idx = meas_map_elem.first;
                        const std::vector<LiDARMeasurement>& meas_vec = meas_map_elem.second;
                        lidar_measurements_[frame_id][meas_idx + num_board_poses_] = meas_vec;
                    }
                }
            }
            else
            {
                for (const FrameID& frame_id : lidar_frame_ids_)
                {
                    for (int idx = 0; idx < other_camera_calib.num_board_poses_; idx++)
                    {
                        lidar_measurements_[frame_id][idx + num_board_poses_].push_back(LiDARMeasurement());
                        ROS_WARN_STREAM_NAMED("IO", "Added empty measurement for '" << frame_id << "'.");
                    }
                }
            }

            for (const auto& map_elem : other_camera_calib.camera_measurements_)
            {
                const FrameID& frame_id = map_elem.first;
                const std::map<MeasurementIndex, std::vector<CameraMeasurement>>& meas_map = map_elem.second;
                for (const auto& meas_map_elem : meas_map)
                {
                    const MeasurementIndex meas_idx = meas_map_elem.first;
                    const std::vector<CameraMeasurement>& meas_vec = meas_map_elem.second;
                    camera_measurements_[frame_id][meas_idx + num_board_poses_] = meas_vec;
                }
            }

            for (const auto& map_elem : other_camera_calib.camera_measurement_images_)
            {
                const FrameID& frame_id = map_elem.first;
                const std::map<MeasurementIndex, std::vector<cv::Mat>>& img_map = map_elem.second;
                for (const auto& img_map_elem : img_map)
                {
                    const MeasurementIndex meas_idx = img_map_elem.first;
                    const std::vector<cv::Mat>& img_vec = img_map_elem.second;
                    camera_measurement_images_[frame_id][meas_idx + num_board_poses_] = img_vec;
                }
            }

            if (other_camera_calib.radar_measurements_.size() > 0)
            {
                for (const auto& map_elem : other_camera_calib.radar_measurements_)
                {
                    const FrameID& frame_id = map_elem.first;
                    const std::map<MeasurementIndex, std::vector<RadarMeasurement>>& meas_map = map_elem.second;
                    for (const auto& meas_map_elem : meas_map)
                    {
                        const MeasurementIndex meas_idx = meas_map_elem.first;
                        const std::vector<RadarMeasurement>& meas_vec = meas_map_elem.second;
                        radar_measurements_[frame_id][meas_idx + num_board_poses_] = meas_vec;
                    }
                }
            }
            else
            {
                for (const FrameID& frame_id : radar_frame_ids_)
                {
                    for (int idx = 0; idx < other_camera_calib.num_board_poses_; idx++)
                    {
                        radar_measurements_[frame_id][idx + num_board_poses_].push_back(RadarMeasurement());
                        ROS_WARN_STREAM_NAMED("IO", "Added empty measurement for '" << frame_id << "'.");
                    }
                }
            }

            num_board_poses_ += other_camera_calib.num_board_poses_;
        }

        // Init potentially missing publishers
        initCameraPublishers();

        sensor_poses_changed_ = true;
        board_poses_changed_ = true;
        measurements_changed_ = true;

        calibrate(false);
    }
    catch (boost::archive::archive_exception& e)
    {
        ROS_WARN_STREAM_NAMED("IO", "Exception opening file: " << e.what());
    }
}

void SensorCalib::measure(bool new_board_pose) noexcept
{
    ROS_INFO_STREAM_NAMED("Measurement", "Starting a new measurement process...");

    int num_measurements_for_board_pose = 0;

    if (new_board_pose || num_board_poses_ == 0)
    {
        num_board_poses_++;
    }
    else
    {
        for (const auto& lidar_measurements_elem : lidar_measurements_)
        {
            const std::map<MeasurementIndex, std::vector<LiDARMeasurement>>& lidar_measurements =
                lidar_measurements_elem.second;
            num_measurements_for_board_pose = std::max(
                num_measurements_for_board_pose, static_cast<int>(lidar_measurements.at(num_board_poses_ - 1).size()));
        }

        for (const auto& camera_measurements_elem : camera_measurements_)
        {
            const std::map<MeasurementIndex, std::vector<CameraMeasurement>>& camera_measurements =
                camera_measurements_elem.second;
            num_measurements_for_board_pose = std::max(
                num_measurements_for_board_pose, static_cast<int>(camera_measurements.at(num_board_poses_ - 1).size()));
        }

        for (const auto& radar_measurements_elem : radar_measurements_)
        {
            const std::map<MeasurementIndex, std::vector<RadarMeasurement>>& radar_measurements =
                radar_measurements_elem.second;
            num_measurements_for_board_pose = std::max(
                num_measurements_for_board_pose, static_cast<int>(radar_measurements.at(num_board_poses_ - 1).size()));
        }
    }

    bool measurements_complete = true;
    measurements_complete &= measureAprilTags();
    measurements_complete &= measureLiDARTargets();
    measurements_complete &= measureRadarTargets();

    if (!measurements_complete)
    {
        const bool got_measurement = fillUpMeasurements(num_measurements_for_board_pose + 1);
        if (!got_measurement)
        {
            if (new_board_pose)
            {
                num_board_poses_--;
            }
            return;
        }
    }

    initBoardPoses();
}

bool SensorCalib::measureLiDARTargets() noexcept
{
    publishStatusMsg("Collecting LiDAR measurements");

    bool got_all_measurements = true;

    for (const FrameID& frame_id : lidar_frame_ids_)
    {
        ROS_DEBUG_STREAM_NAMED("Measurement", "Collecting measurements for frame '" << frame_id << "'...");

        if (current_lidar_reference_point_detections_.count(frame_id) &&
            ros::Time::now() - current_lidar_reference_point_detections_.at(frame_id)->header.stamp <
                ros::Duration(config_.max_measurement_age))
        {
            sensor_calib_msgs::CartesianPointMeasurements::ConstPtr msg =
                current_lidar_reference_point_detections_.at(frame_id);

            // Add new LiDAR measurement
            LiDARMeasurement new_lidar_measurement;
            int num_measurements = 0;
            for (const auto& board_elem : calibration_boards_)
            {
                const BoardID& board_id = board_elem.first;

                for (const sensor_calib_msgs::CartesianPointMeasurement& measured_point : msg->measured_points)
                {
                    if (board_elem.second.lidar_reference_points.count(measured_point.id))
                    {
                        new_lidar_measurement[board_id][measured_point.id] =
                            CartesianCoordinate(measured_point.point.x, measured_point.point.y, measured_point.point.z);
                        num_measurements++;
                    }
                }
            }

            if (num_measurements > 0)
            {
                lidar_measurements_[frame_id][num_board_poses_ - 1].push_back(new_lidar_measurement);
            }
            else
            {
                got_all_measurements = false;
            }

            ROS_INFO_STREAM_NAMED("Measurement",
                                  "Found " << num_measurements << " reference point measurements for LiDAR '"
                                           << frame_id << "'");
        }
        else
        {
            got_all_measurements = false;

            ROS_WARN_STREAM_NAMED("Measurement", "No or old measurements for frame '" << frame_id);
        }
    }

    publishStatusMsg();

    return got_all_measurements;
}

bool SensorCalib::measureAprilTags() noexcept
{
    publishStatusMsg("Collecting AprilTag measurements");

    bool got_all_measurements = true;

    for (const FrameID& frame_id : camera_frame_ids_)
    {
        ROS_DEBUG_STREAM_NAMED("Measurement", "Collecting measurements for frame '" << frame_id << "'...");

        if (current_apriltag_detections_.count(frame_id))
        {
            apriltags_msgs::AprilTagDetections::ConstPtr msg = current_apriltag_detections_.at(frame_id);

            if (ros::Time::now() - msg->header.stamp < ros::Duration(config_.max_measurement_age))
            {
                // Add all apriltag (corner) measurements to one new camera measurement
                CameraMeasurement new_camera_measurement;
                int num_measurements = 0;
                for (const auto& board_elem : calibration_boards_)
                {
                    const BoardID& board_id = board_elem.first;

                    for (const auto& tag : msg->detections)
                    {
                        if (board_elem.second.apriltags.count(tag.id))
                        {
                            for (int i = 0; i < 4; i++)
                            {
                                new_camera_measurement[board_id][tag.id][i] =
                                    Eigen::Vector2d(tag.corners_px[i].x, tag.corners_px[i].y);
                            }
                            num_measurements++;
                        }
                    }
                }

                if (num_measurements > 0)
                {
                    camera_measurements_[frame_id][num_board_poses_ - 1].push_back(new_camera_measurement);

                    // Add corresponding image
                    cv::Mat apriltag_input_image = cv_bridge::toCvShare(msg->input_image, msg)->image.clone();
                    camera_measurement_images_[frame_id][num_board_poses_ - 1].push_back(apriltag_input_image.clone());
                }
                else
                {
                    got_all_measurements = false;
                }

                ROS_INFO_STREAM_NAMED("Measurement",
                                      "Found " << num_measurements << " apriltag measurements for camera '" << frame_id
                                               << "'");
            }
            else
            {
                got_all_measurements = false;
                ROS_WARN_STREAM_NAMED("Measurement", "Old measurements for frame '" << frame_id);
            }
        }
        else
        {
            got_all_measurements = false;
            ROS_WARN_STREAM_NAMED("Measurement", "No measurements for frame '" << frame_id);
        }
    }

    publishStatusMsg();

    return got_all_measurements;
}

bool SensorCalib::measureRadarTargets() noexcept
{
    publishStatusMsg("Collecting radar measurements");

    bool got_all_measurements = true;

    for (const FrameID& frame_id : radar_frame_ids_)
    {
        ROS_DEBUG_STREAM_NAMED("Measurement", "Collecting measurements for frame'" << frame_id << "'...");

        if (current_radar_target_detections_.count(frame_id) &&
            ros::Time::now() - current_radar_target_detections_.at(frame_id)->header.stamp <
                ros::Duration(config_.max_measurement_age))
        {
            sensor_calib_msgs::PolarPointMeasurements::ConstPtr msg = current_radar_target_detections_.at(frame_id);

            // Add all radar target measurements to one new radar measurement
            RadarMeasurement new_radar_measurement;
            int num_measurements = 0;
            for (const auto& board_elem : calibration_boards_)
            {
                const BoardID& board_id = board_elem.first;

                for (const sensor_calib_msgs::PolarPointMeasurement& target : msg->measured_points)
                {
                    if (board_elem.second.radar_targets.count(target.id))
                    {
                        new_radar_measurement[board_id][target.id] =
                            PolarCoordinate(target.range, target.azimuth, target.elevation);
                        num_measurements++;
                    }
                }
            }

            if (num_measurements > 0)
            {
                radar_measurements_[frame_id][num_board_poses_ - 1].push_back(new_radar_measurement);
            }
            else
            {
                got_all_measurements = false;
            }

            ROS_INFO_STREAM_NAMED(
                "Measurement", "Found " << num_measurements << " target measurements for radar '" << frame_id << "'");
        }
        else
        {
            got_all_measurements = false;

            ROS_WARN_STREAM_NAMED("Measurement", "No or old measurements for frame '" << frame_id);
        }
    }

    publishStatusMsg();

    return got_all_measurements;
}

bool SensorCalib::fillUpMeasurements(const int num_measurements_for_board_pose) noexcept
{
    bool got_new_measurement = false;

    for (const FrameID& frame_id : lidar_frame_ids_)
    {
        if (lidar_measurements_[frame_id].count(num_board_poses_ - 1) &&
            lidar_measurements_[frame_id].at(num_board_poses_ - 1).size() >= num_measurements_for_board_pose)
        {
            got_new_measurement = true;
        }
    }

    for (const FrameID& frame_id : camera_frame_ids_)
    {
        if (camera_measurements_[frame_id].count(num_board_poses_ - 1) &&
            camera_measurements_[frame_id].at(num_board_poses_ - 1).size() >= num_measurements_for_board_pose)
        {
            got_new_measurement = true;
        }
    }

    for (const FrameID& frame_id : radar_frame_ids_)
    {
        if (radar_measurements_[frame_id].count(num_board_poses_ - 1) &&
            radar_measurements_[frame_id].at(num_board_poses_ - 1).size() >= num_measurements_for_board_pose)
        {
            got_new_measurement = true;
        }
    }

    if (got_new_measurement)
    {
        for (const FrameID& frame_id : lidar_frame_ids_)
        {
            if (!lidar_measurements_[frame_id].count(num_board_poses_ - 1) ||
                lidar_measurements_[frame_id].at(num_board_poses_ - 1).size() < num_measurements_for_board_pose)
            {
                lidar_measurements_[frame_id][num_board_poses_ - 1].push_back(LiDARMeasurement());
                ROS_WARN_STREAM_NAMED("Measurement", "Added empty measurement for '" << frame_id << "'.");
            }

            ROS_DEBUG_STREAM_NAMED("Measurement",
                                   "fillUpMeasurements: " << frame_id << " now has "
                                                          << lidar_measurements_[frame_id].size() << " measurements.");
        }

        for (const FrameID& frame_id : camera_frame_ids_)
        {
            if (!camera_measurements_[frame_id].count(num_board_poses_ - 1) ||
                camera_measurements_[frame_id].at(num_board_poses_ - 1).size() < num_measurements_for_board_pose)
            {
                camera_measurements_[frame_id][num_board_poses_ - 1].push_back(CameraMeasurement());
                ROS_WARN_STREAM_NAMED("Measurement", "Added empty measurement for '" << frame_id << "'.");
            }
            ROS_DEBUG_STREAM_NAMED("Measurement",
                                   "fillUpMeasurements: " << frame_id << " now has "
                                                          << camera_measurements_[frame_id].size() << " measurements.");
        }

        for (const FrameID& frame_id : radar_frame_ids_)
        {
            if (!radar_measurements_[frame_id].count(num_board_poses_ - 1) ||
                radar_measurements_[frame_id].at(num_board_poses_ - 1).size() < num_measurements_for_board_pose)
            {
                radar_measurements_[frame_id][num_board_poses_ - 1].push_back(RadarMeasurement());
                ROS_WARN_STREAM_NAMED("Measurement", "Added empty measurement for '" << frame_id << "'.");
            }

            ROS_DEBUG_STREAM_NAMED("Measurement",
                                   "fillUpMeasurements: " << frame_id << " now has "
                                                          << radar_measurements_[frame_id].size() << " measurements.");
        }

        return true;
    }
    else
    {
        ROS_WARN_STREAM_NAMED("Measurement", "Got no valid measurements");
        return false;
    }
}

void SensorCalib::deleteLastMeasurement() noexcept
{
    if (num_board_poses_ > 0)
    {
        ROS_WARN_STREAM_NAMED("Measurement", "Deleting last measurement");

        const std::size_t current_measurement_index = num_board_poses_ - 1;
        std::size_t measurements_for_measurement_index = 0;

        for (auto& lidar_measurements_elem : lidar_measurements_)
        {
            std::map<MeasurementIndex, std::vector<LiDARMeasurement>>& lidar_measurements =
                lidar_measurements_elem.second;
            lidar_measurements.at(num_board_poses_ - 1).pop_back();
            measurements_for_measurement_index =
                std::max(measurements_for_measurement_index, lidar_measurements.at(num_board_poses_ - 1).size());
        }

        for (auto& camera_measurements_elem : camera_measurements_)
        {
            std::map<MeasurementIndex, std::vector<CameraMeasurement>>& camera_measurements =
                camera_measurements_elem.second;
            camera_measurements.at(num_board_poses_ - 1).pop_back();
            measurements_for_measurement_index =
                std::max(measurements_for_measurement_index, camera_measurements.at(num_board_poses_ - 1).size());
        }

        for (auto& camera_measurement_images_elem : camera_measurement_images_)
        {
            std::map<MeasurementIndex, std::vector<cv::Mat>>& camera_measurement_images =
                camera_measurement_images_elem.second;
            if (camera_measurement_images.count(num_board_poses_ - 1))
                camera_measurement_images.at(num_board_poses_ - 1).pop_back();
        }

        for (auto& radar_measurements_elem : radar_measurements_)
        {
            std::map<MeasurementIndex, std::vector<RadarMeasurement>>& radar_measurements =
                radar_measurements_elem.second;
            radar_measurements.at(num_board_poses_ - 1).pop_back();
            measurements_for_measurement_index =
                std::max(measurements_for_measurement_index, radar_measurements.at(num_board_poses_ - 1).size());
        }

        if (measurements_for_measurement_index == 0)
        {
            for (auto& lidar_measurements_elem : lidar_measurements_)
            {
                std::map<MeasurementIndex, std::vector<LiDARMeasurement>>& lidar_measurements =
                    lidar_measurements_elem.second;
                lidar_measurements.erase(current_measurement_index);
            }

            for (auto& camera_measurements_elem : camera_measurements_)
            {
                std::map<MeasurementIndex, std::vector<CameraMeasurement>>& camera_measurements =
                    camera_measurements_elem.second;
                camera_measurements.erase(current_measurement_index);
            }

            for (auto& camera_measurement_images_elem : camera_measurement_images_)
            {
                std::map<MeasurementIndex, std::vector<cv::Mat>>& camera_measurement_images =
                    camera_measurement_images_elem.second;
                camera_measurement_images.erase(current_measurement_index);
            }

            for (auto& radar_measurements_elem : radar_measurements_)
            {
                std::map<MeasurementIndex, std::vector<RadarMeasurement>>& radar_measurements =
                    radar_measurements_elem.second;
                radar_measurements.erase(current_measurement_index);
            }

            for (auto& board_poses_elem : board_poses_)
            {
                std::map<MeasurementIndex, Pose>& board_poses = board_poses_elem.second;
                board_poses.erase(current_measurement_index);
            }

            for (auto& initial_board_poses_elem : initial_board_poses_)
            {
                std::map<MeasurementIndex, Pose>& initial_board_poses = initial_board_poses_elem.second;
                initial_board_poses.erase(current_measurement_index);
            }

            num_board_poses_--;
        }
    }
}