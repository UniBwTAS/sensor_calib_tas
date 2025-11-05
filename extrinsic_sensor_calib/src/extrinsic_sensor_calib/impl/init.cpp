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

#include <Eigen/Core>

#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>

#include <image_geometry/pinhole_camera_model.h>

#include "../cost_functors.hpp"
#include "../sensor_calib.h"

void SensorCalib::initSensorPoses() noexcept
{
    ROS_INFO_STREAM_NAMED("Initialization", "Initializing sensor poses...");
    publishStatusMsg("Initializing sensor poses...");

    // Init LiDAR poses
    for (const auto& frame_id : lidar_frame_ids_)
    {
        ROS_DEBUG_STREAM_NAMED("Initialization", "Init sensor pose for '" << frame_id << "'");
        sensor_poses_[std::make_pair(reference_frame_id_, frame_id)] = getPoseFromTF(reference_frame_id_, frame_id);
    }

    // Init camera poses
    for (const auto& frame_id : camera_frame_ids_)
    {
        ROS_DEBUG_STREAM_NAMED("Initialization", "Init sensor pose for '" << frame_id << "'");
        sensor_poses_[std::make_pair(reference_frame_id_, frame_id)] = getPoseFromTF(reference_frame_id_, frame_id);
    }

    // Init radar poses
    for (const auto& frame_id : radar_frame_ids_)
    {
        ROS_DEBUG_STREAM_NAMED("Initialization", "Init sensor pose for '" << frame_id << "'");
        sensor_poses_[std::make_pair(reference_frame_id_, frame_id)] = getPoseFromTF(reference_frame_id_, frame_id);
    }

    initial_sensor_poses_ = sensor_poses_;

    sensor_poses_changed_ = true;

    publishStatusMsg();
}

void SensorCalib::resetSensorPosesToZero() noexcept
{
    ROS_INFO_STREAM_NAMED("Initialization", "Resetting sensor poses...");
    publishStatusMsg("Resetting sensor poses...");

    // Reset LiDAR poses
    for (const auto& frame_id : lidar_frame_ids_)
    {
        sensor_poses_[std::make_pair(reference_frame_id_, frame_id)] = Pose();
    }

    // Reset camera poses
    for (const auto& frame_id : camera_frame_ids_)
    {
        sensor_poses_[std::make_pair(reference_frame_id_, frame_id)] = Pose();
        // Avoid division by 0
        sensor_poses_[std::make_pair(reference_frame_id_, frame_id)].translation.z() = 1;
    }

    // Reset radar poses
    for (const auto& frame_id : radar_frame_ids_)
    {
        sensor_poses_[std::make_pair(reference_frame_id_, frame_id)] = Pose();

        if (config_.set_radar_z_constant)
        {
            ROS_DEBUG_STREAM_NAMED("Initialization", "Reset sensor pose for '" << frame_id << "'");
            const Pose original_pose = getPoseFromTF(reference_frame_id_, frame_id);
            sensor_poses_[std::make_pair(reference_frame_id_, frame_id)].translation.z() =
                original_pose.translation.z();
        }
    }

    publishStatusMsg();

    sensor_poses_changed_ = true;
}

void SensorCalib::initIntrinsics() noexcept
{
    ROS_INFO_STREAM_NAMED("Initialization", "Initializing intrinsics...");
    publishStatusMsg("Initializing intrinsics...");

    // Init intrinsics -> Wait for camera info
    for (const std::string& camera_topic : camera_topics_)
    {
        boost::shared_ptr<sensor_msgs::CameraInfo const> cam_info;
        while (cam_info == NULL)
        {
            ROS_INFO_STREAM_NAMED("Initialization",
                                  "Waiting for camera info on topic '" << camera_topic << "/camera_info'...");
            cam_info = ros::topic::waitForMessage<sensor_msgs::CameraInfo>(camera_topic + "/camera_info");
        }
        const FrameID& frame_id = cam_info->header.frame_id;

        image_geometry::PinholeCameraModel camera_model;
        camera_model.fromCameraInfo(*cam_info);
        camera_intrinsics_[frame_id].img_width = camera_model.rawRoi().size().width;
        camera_intrinsics_[frame_id].img_height = camera_model.rawRoi().size().height;

        cv::cv2eigen(camera_model.intrinsicMatrix(), camera_intrinsics_.at(frame_id).intrinsic_matrix);
        cv::cv2eigen(camera_model.projectionMatrix(), camera_intrinsics_.at(frame_id).projection_matrix);
        cv::cv2eigen(camera_model.rotationMatrix(), camera_intrinsics_.at(frame_id).rectification_matrix);
        camera_intrinsics_.at(frame_id).distortion_parameters(0) = camera_model.distortionCoeffs().at<double>(0);
        camera_intrinsics_.at(frame_id).distortion_parameters(1) = camera_model.distortionCoeffs().at<double>(1);
        camera_intrinsics_.at(frame_id).distortion_parameters(2) = camera_model.distortionCoeffs().at<double>(2);
        camera_intrinsics_.at(frame_id).distortion_parameters(3) = camera_model.distortionCoeffs().at<double>(3);
        camera_intrinsics_.at(frame_id).distortion_parameters(4) = camera_model.distortionCoeffs().at<double>(4);
    }

    publishStatusMsg();
}

void SensorCalib::initBoardPoses() noexcept
{
    ROS_INFO_STREAM_NAMED("Initialization", "Initializing board poses...");
    publishStatusMsg("Initializing board poses...");

    // Look for uninitialized boards and collect which sensor detected how many reference points of this board
    std::map<BoardID, bool> board_initialized;
    std::map<BoardID, std::multimap<size_t, FrameID>> measurements_for_new_board;

    // Iterate through LiDAR measurements
    for (const auto& lidar_measurements_elem : lidar_measurements_)
    {
        const FrameID& lidar_frame_id = lidar_measurements_elem.first;
        const LiDARMeasurement& current_measurement_for_lidar = lidar_measurements_elem.second.rbegin()->second.back();

        // Iterate through detected boards
        for (const auto& current_measurement_for_lidar_for_board_elem : current_measurement_for_lidar)
        {
            const BoardID& board_id = current_measurement_for_lidar_for_board_elem.first;
            board_initialized[board_id] = board_poses_[board_id].count(num_board_poses_ - 1);

            // Found uninitialized board
            if (!board_initialized.at(board_id))
            {
                // If at least 3 reference points were detected: Save measurement for initialization
                const size_t num_detections = current_measurement_for_lidar_for_board_elem.second.size();
                if (num_detections >= 3)
                {
                    // Remember number of detections. Reference sensor gets max number of detections such that reference
                    // sensor is preferred
                    measurements_for_new_board[board_id].insert(std::make_pair(
                        lidar_frame_id == reference_frame_id_ ? std::numeric_limits<size_t>::max() : num_detections,
                        lidar_frame_id));
                }
            }
        }
    }
    // Iterate through camera measurements
    for (const auto& camera_measurements_elem : camera_measurements_)
    {
        const FrameID& camera_frame_id = camera_measurements_elem.first;
        const CameraMeasurement& current_measurement_for_camera =
            camera_measurements_elem.second.rbegin()->second.back();

        // Iterate through detected boards
        for (const auto& current_measurement_for_camera_for_board_elem : current_measurement_for_camera)
        {
            const BoardID& board_id = current_measurement_for_camera_for_board_elem.first;
            board_initialized[board_id] = board_poses_[board_id].count(num_board_poses_ - 1);

            // Found uninitialized board
            if (!board_initialized.at(board_id))
            {
                // If at least 1 AprilTag (= 4 corners) was detected: Save measurement for initialization
                const size_t num_detections = current_measurement_for_camera_for_board_elem.second.size() * 4;
                if (num_detections >= 4)
                {
                    // Remember number of detections. Reference sensor gets max number of detections such that reference
                    // sensor is preferred
                    measurements_for_new_board[board_id].insert(std::make_pair(
                        camera_frame_id == reference_frame_id_ ? std::numeric_limits<size_t>::max() : num_detections,
                        camera_frame_id));
                }
            }
        }
    }
    // Iterate through radar measurements
    for (const auto& radar_measurements_elem : radar_measurements_)
    {
        const FrameID& radar_frame_id = radar_measurements_elem.first;
        const RadarMeasurement& current_measurement_for_radar = radar_measurements_elem.second.rbegin()->second.back();

        // Iterate through detected boards
        for (const auto& current_measurement_for_radar_for_board_elem : current_measurement_for_radar)
        {
            const BoardID& board_id = current_measurement_for_radar_for_board_elem.first;
            board_initialized[board_id] = board_poses_[board_id].count(num_board_poses_ - 1);

            // Found uninitialized board
            if (!board_initialized.at(board_id))
            {
                // If at least 3 reference points were detected: Save measurement for initialization
                const size_t num_detections = current_measurement_for_radar_for_board_elem.second.size();
                if (num_detections >= 3)
                {
                    // Remember number of detections. Reference sensor gets max number of detections such that reference
                    // sensor is preferred
                    measurements_for_new_board[board_id].insert(std::make_pair(
                        radar_frame_id == reference_frame_id_ ? std::numeric_limits<size_t>::max() : num_detections,
                        radar_frame_id));
                }
            }
        }
    }

    // Initialize board pose using measurement with the most reference point detections (but prefer reference sensor)
    for (const auto& measurements_for_board_elem : measurements_for_new_board)
    {
        const BoardID& board_id = measurements_for_board_elem.first;
        const std::multimap<size_t, FrameID> frame_ids_with_measurements = measurements_for_board_elem.second;

        // Try measurement with most reference point detections (beginning with reference sensor)
        for (auto it = frame_ids_with_measurements.rbegin();
             it != frame_ids_with_measurements.rend() && !board_initialized.at(board_id);
             it++)
        {
            const FrameID& frame_id = it->second;

            std::optional<Pose> pose;
            size_t num_measurements = 0; // Do not use it->first to get real num for reference sensor
            if (isCamera(frame_id))
            {
                pose = getBoardPoseFromCameraMeasurement(
                    board_id, frame_id, camera_measurements_[frame_id][num_board_poses_ - 1].back()[board_id]);
                num_measurements = camera_measurements_[frame_id][num_board_poses_ - 1].back()[board_id].size();
            }
            else if (isLiDAR(frame_id))
            {
                pose = getBoardPoseFromLiDARMeasurement(
                    board_id, lidar_measurements_[frame_id][num_board_poses_ - 1].back()[board_id]);
                num_measurements = lidar_measurements_[frame_id][num_board_poses_ - 1].back()[board_id].size();
            }
            else if (isRadar(frame_id))
            {
                pose = getBoardPoseFromRadarMeasurement(
                    board_id, radar_measurements_[frame_id][num_board_poses_ - 1].back()[board_id]);
                num_measurements = radar_measurements_[frame_id][num_board_poses_ - 1].back()[board_id].size();
            }

            if (pose)
            {
                board_poses_[board_id][num_board_poses_ - 1] = convertToReferenceFrame(*pose, frame_id);
                board_initialized.at(board_id) = true;
                initial_board_poses_[board_id][num_board_poses_ - 1] = board_poses_[board_id][num_board_poses_ - 1];

                ROS_INFO_STREAM_NAMED("Initialization",
                                      "Initialized pose for board '" << board_id << "' by " << num_measurements
                                                                     << " reference points detected in frame '"
                                                                     << frame_id << "'");
            }
            else
            {
                ROS_INFO_STREAM_NAMED("Initialization",
                                      "Failed to initialize pose for board '" << board_id << "' by " << num_measurements
                                                                              << " reference points detected in frame '"
                                                                              << frame_id << "'");
            }
        }
    }

    // Check, if there is a board, which is still not initialized -> init to zero
    for (const auto& board_initialized_elem : board_initialized)
    {
        if (!board_initialized_elem.second)
        {
            const BoardID& board_id = board_initialized_elem.first;
            ROS_WARN_STREAM_NAMED("Initialization",
                                  "Could not initialize pose for board '" << board_id
                                                                          << "', initializing to zero/identity");
            board_poses_[board_id][num_board_poses_ - 1] = Pose();
        }
    }

    board_poses_changed_ = true;
}

std::optional<Pose>
SensorCalib::getBoardPoseFromCameraMeasurement(const BoardID& board_id,
                                               const FrameID& camera_frame_id,
                                               const CameraBoardMeasurement& tag_measurements) const noexcept
{
    ROS_DEBUG_STREAM_NAMED("Initialization", "Initializing board `" << board_id << "` by camera measurements");

    // Collect points in OpenCV format
    std::vector<cv::Point3f> object_points_cv;
    std::vector<cv::Point2f> image_points_cv;
    for (const auto& tag_measurement_elem : tag_measurements)
    {
        const TagID& tag_id = tag_measurement_elem.first;
        const TagMeasurement& tag_measurement = tag_measurement_elem.second;

        const Eigen::Isometry3d& board_from_tag =
            calibration_boards_.at(board_id).apriltags.at(tag_id).board_from_tag();
        const double half_tag_size = calibration_boards_.at(board_id).apriltags.at(tag_id).size / 2.;

        for (int i = 0; i < 4; i++)
        {
            image_points_cv.emplace_back(tag_measurement[i].x(), tag_measurement[i].y());

            Eigen::Vector3d tag_corner_position;

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

            const Eigen::Vector3d board_point_position = board_from_tag * tag_corner_position;

            object_points_cv.emplace_back(board_point_position.x(), board_point_position.y(), board_point_position.z());
        }
    }

    // Get calibration in OpenCV format
    cv::Mat camera_matrix;
    cv::Mat dist_coeffs;
    cv::eigen2cv(camera_intrinsics_.at(camera_frame_id).intrinsic_matrix, camera_matrix);
    cv::eigen2cv(camera_intrinsics_.at(camera_frame_id).distortion_parameters, dist_coeffs);

    // Use OpenCV solvePnP
    cv::Mat rvec_cv;
    cv::Mat tvec_cv;
    const bool success = cv::solvePnP(
        object_points_cv, image_points_cv, camera_matrix, dist_coeffs, rvec_cv, tvec_cv, false, cv::SOLVEPNP_ITERATIVE);

    // Convert result to pose
    if (success)
    {
        Eigen::Vector3f rvec;
        Eigen::Vector3f tvec;

        cv::cv2eigen(rvec_cv, rvec);
        cv::cv2eigen(tvec_cv, tvec);

        Pose result_pose;
        result_pose.translation = tvec.cast<double>();
        result_pose.rotation =
            Eigen::Quaterniond(Eigen::AngleAxisd(rvec.cast<double>().norm(), rvec.cast<double>().normalized()));

        return std::optional<Pose>(result_pose);
    }
    else
    {
        return std::nullopt;
    }
}

std::optional<Pose>
SensorCalib::getBoardPoseFromLiDARMeasurement(const BoardID& board_id,
                                              const LiDARBoardMeasurement& lidar_target_measurements) const noexcept
{
    ROS_DEBUG_STREAM_NAMED("Initialization", "Initializing board `" << board_id << "` by LiDAR measurements");

    Pose identity;
    Pose lidar_to_board;

    // Create temporary optimization problem for the single measurement
    ceres::Problem optimization_problem;

    // Add residual blocks for every reference point
    for (const auto& lidar_reference_point_measurement_elem : lidar_target_measurements)
    {
        const ReferencePointID point_id = lidar_reference_point_measurement_elem.first;
        const CartesianCoordinate& lidar_reference_point_measurement = lidar_reference_point_measurement_elem.second;

        optimization_problem.AddResidualBlock(
            new CartesianPointError::CostFunction(
                new CartesianPointError(calibration_boards_.at(board_id).lidar_reference_points.at(point_id).position,
                                        lidar_reference_point_measurement,
                                        1)),
            nullptr,
            lidar_to_board.translation.data(),
            lidar_to_board.rotation.coeffs().data(),
            identity.translation.data(),
            identity.rotation.coeffs().data());
    }
    // Set sensor pose = identity constant
    optimization_problem.SetParameterBlockConstant(identity.translation.data());
    optimization_problem.SetParameterBlockConstant(identity.rotation.coeffs().data());

    // Solve
    ceres::Solver::Options solve_options;
    solve_options.linear_solver_type = ceres::LinearSolverType::SPARSE_NORMAL_CHOLESKY;
    solve_options.num_linear_solver_threads = 1;
    solve_options.max_num_iterations = 1000;
    solve_options.max_solver_time_in_seconds = config_.max_solver_time;
    ceres::Solver::Summary summary;
    ceres::Solve(solve_options, &optimization_problem, &summary);
    ROS_DEBUG_STREAM_NAMED("Initialization",
                           summary.BriefReport().c_str()
                               << ", Time: " << summary.total_time_in_seconds * 1000.0 << "ms");

    const bool success = summary.IsSolutionUsable() && summary.final_cost < 1; // meter
    return success ? std::optional<Pose>(lidar_to_board) : std::nullopt;
}

std::optional<Pose>
SensorCalib::getBoardPoseFromRadarMeasurement(const BoardID& board_id,
                                              const RadarBoardMeasurement& radar_target_measurements) const noexcept
{
    ROS_DEBUG_STREAM_NAMED("Initialization", "Initializing board `" << board_id << "` by radar measurements");

    Pose identity;
    Pose radar_to_board;

    // Create temporary optimization problem for the single measurement
    ceres::Problem optimization_problem;

    // Add residual blocks for every target
    for (const auto& target_elem : radar_target_measurements)
    {
        const TargetID& target_id = target_elem.first;
        const PolarCoordinate& radar_target_measurement = target_elem.second;

        optimization_problem.AddResidualBlock(new PolarCoordinateError::CostFunction(new PolarCoordinateError(
                                                  calibration_boards_.at(board_id).radar_targets.at(target_id),
                                                  radar_target_measurement,
                                                  PolarCoordinate(1, 1, 1))),
                                              nullptr,
                                              radar_to_board.translation.data(),
                                              radar_to_board.rotation.coeffs().data(),
                                              identity.translation.data(),
                                              identity.rotation.coeffs().data());
    }
    // Set sensor pose = identity constant
    optimization_problem.SetParameterBlockConstant(identity.translation.data());
    optimization_problem.SetParameterBlockConstant(identity.rotation.coeffs().data());

    // Solve
    ceres::Solver::Options solve_options;
    solve_options.linear_solver_type = ceres::LinearSolverType::SPARSE_NORMAL_CHOLESKY;
    solve_options.num_linear_solver_threads = 1;
    solve_options.max_num_iterations = 1000;
    solve_options.max_solver_time_in_seconds = config_.max_solver_time;
    ceres::Solver::Summary summary;
    ceres::Solve(solve_options, &optimization_problem, &summary);
    ROS_DEBUG_STREAM_NAMED("Initialization",
                           summary.BriefReport().c_str()
                               << ", Time: " << summary.total_time_in_seconds * 1000.0 << "ms");

    const bool success = summary.IsSolutionUsable() && summary.final_cost < 1; // meter
    return success ? std::optional<Pose>(radar_to_board) : std::nullopt;
}
