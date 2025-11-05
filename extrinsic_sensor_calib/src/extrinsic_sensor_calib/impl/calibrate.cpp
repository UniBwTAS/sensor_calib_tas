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

#include <opencv2/core.hpp>

#include <sensor_calib_msgs/Residuals.h>

#include "../cost_functors.hpp"
#include "../sensor_calib.h"

void SensorCalib::calibrate(const bool solve) noexcept
{
    if (solve)
    {
        ROS_INFO_STREAM_NAMED("Optimization", "----------- calibrate() -----------");
        ROS_INFO_STREAM_NAMED("Optimization", "Building the optimization problem...");
    }

    for (const FrameID& frame_id : lidar_frame_ids_)
    {
        int board_count = 0;

        if (lidar_measurements_.count(frame_id))
        {
            const std::map<MeasurementIndex, std::vector<LiDARMeasurement>>& measurements_for_lidar =
                lidar_measurements_.at(frame_id);
            for (const auto& measurements_for_lidar_for_measidx_elem : measurements_for_lidar)
            {
                const std::vector<LiDARMeasurement>& measurements_for_lidar_for_measidx =
                    measurements_for_lidar_for_measidx_elem.second;

                for (const LiDARMeasurement& measurement_for_lidar_for_measidx : measurements_for_lidar_for_measidx)
                {
                    board_count += measurement_for_lidar_for_measidx.size();
                }
            }

            if (solve)
                ROS_INFO_STREAM_NAMED("Optimization",
                                      "Got " << board_count << " board measurements for '" << frame_id << "' ("
                                             << measurements_for_lidar.size() << " different poses).");
        }
    }

    for (const FrameID& frame_id : camera_frame_ids_)
    {
        int board_count = 0;

        if (camera_measurements_.count(frame_id))
        {
            const std::map<MeasurementIndex, std::vector<CameraMeasurement>>& measurements_for_camera =
                camera_measurements_.at(frame_id);
            for (const auto& measurements_for_camera_for_measidx_elem : measurements_for_camera)
            {
                const std::vector<CameraMeasurement>& measurements_for_camera_for_measidx =
                    measurements_for_camera_for_measidx_elem.second;

                for (const CameraMeasurement& measurement_for_camera_for_measidx : measurements_for_camera_for_measidx)
                {
                    board_count += measurement_for_camera_for_measidx.size();
                }
            }

            if (solve)
                ROS_INFO_STREAM_NAMED("Optimization",
                                      "Got " << board_count << " board measurements for '" << frame_id << "' ("
                                             << measurements_for_camera.size() << " different poses).");
        }
    }

    for (const FrameID& frame_id : radar_frame_ids_)
    {
        int board_count = 0;

        if (radar_measurements_.count(frame_id))
        {
            const std::map<MeasurementIndex, std::vector<RadarMeasurement>>& measurements_for_radar =
                radar_measurements_.at(frame_id);
            for (const auto& measurements_for_radar_for_measidx_elem : measurements_for_radar)
            {
                const std::vector<RadarMeasurement>& measurements_for_radar_for_measidx =
                    measurements_for_radar_for_measidx_elem.second;

                for (const RadarMeasurement& measurement_for_radar_for_measidx : measurements_for_radar_for_measidx)
                {
                    board_count += measurement_for_radar_for_measidx.size();
                }
            }

            if (solve)
                ROS_INFO_STREAM_NAMED("Optimization",
                                      "Got " << board_count << " board measurements for '" << frame_id << "' ("
                                             << measurements_for_radar.size() << " different poses).");
        }
    }

    if (!initialized_)
    {
        return;
    }

    ceres::Problem optimization_problem;

    // Add sensor pose parameter blocks
    for (auto& sensor_pose_elem : sensor_poses_)
    {
        Pose& sensor_pose = sensor_pose_elem.second;

        // Add parameter blocks
        optimization_problem.AddParameterBlock(sensor_pose.translation.data(), 3);
        optimization_problem.AddParameterBlock(
            sensor_pose.rotation.coeffs().data(), 4, new ceres::EigenQuaternionParameterization());

        // Optionally set z of radar sensors constant
        if (isRadar(sensor_pose_elem.first.second) && config_.set_radar_z_constant)
        {
            ceres::SubsetParameterization* subset_parameterization = new ceres::SubsetParameterization(3, {2});
            optimization_problem.SetParameterization(sensor_pose.translation.data(), subset_parameterization);
        }

        // Set identity sensor pose constant
        if (sensor_pose_elem.first.first == sensor_pose_elem.first.second)
        {
            optimization_problem.SetParameterBlockConstant(sensor_pose.translation.data());
            optimization_problem.SetParameterBlockConstant(sensor_pose.rotation.coeffs().data());
        }

        // Optional: set translation constant
        if (config_.set_all_sensor_poses_constant || (optimize_sensor_pose_.count(sensor_pose_elem.first) &&
                                                      !optimize_sensor_pose_.at(sensor_pose_elem.first).first))
        {
            optimization_problem.SetParameterBlockConstant(sensor_pose.translation.data());
        }
        // Optional: set rotation constant
        if (config_.set_all_sensor_poses_constant || (optimize_sensor_pose_.count(sensor_pose_elem.first) &&
                                                      !optimize_sensor_pose_.at(sensor_pose_elem.first).second))
        {
            optimization_problem.SetParameterBlockConstant(sensor_pose.rotation.coeffs().data());
        }
    }

    // Add stationary board pose parameter blocks
    for (auto& board_poses_elem : board_poses_)
    {
        const BoardID& board_id = board_poses_elem.first;
        std::map<MeasurementIndex, Pose>& board_poses = board_poses_elem.second;

        for (auto& board_pose_elem : board_poses)
        {
            Pose& board_pose = board_pose_elem.second;

            optimization_problem.AddParameterBlock(board_pose.translation.data(), 3);
            optimization_problem.AddParameterBlock(
                board_pose.rotation.coeffs().data(), 4, new ceres::EigenQuaternionParameterization());

            // Optional set constant
            if (config_.set_all_board_poses_constant)
            {
                optimization_problem.SetParameterBlockConstant(board_pose.translation.data());
                optimization_problem.SetParameterBlockConstant(board_pose.rotation.coeffs().data());
            }
        }
    }

    // Add LiDAR residual blocks
    std::map<FrameID, std::map<MeasurementIndex, std::vector<BoardResidualBlockIDs>>> lidar_measurement_residual_blocks;
    for (const auto& lidar_measurements_elem : lidar_measurements_)
    {
        const FrameID& lidar_frame_id = lidar_measurements_elem.first;
        const std::map<MeasurementIndex, std::vector<LiDARMeasurement>>& measurements_for_lidar =
            lidar_measurements_elem.second;

        std::map<MeasurementIndex, std::vector<BoardResidualBlockIDs>>& board_residual_block_IDs_for_lidar =
            lidar_measurement_residual_blocks[lidar_frame_id];

        for (const auto& measurements_for_lidar_for_measidx_elem : measurements_for_lidar)
        {
            const MeasurementIndex meas_idx = measurements_for_lidar_for_measidx_elem.first;
            const std::vector<LiDARMeasurement>& measurements_for_lidar_for_measidx =
                measurements_for_lidar_for_measidx_elem.second;

            std::vector<BoardResidualBlockIDs>& board_residual_block_IDs_for_lidar_for_measidx =
                board_residual_block_IDs_for_lidar[meas_idx];
            board_residual_block_IDs_for_lidar_for_measidx.resize(measurements_for_lidar_for_measidx.size());

            for (int id = 0; id < measurements_for_lidar_for_measidx.size(); id++)
            {
                const LiDARMeasurement& measurement_for_lidar_for_measidx = measurements_for_lidar_for_measidx[id];

                for (const auto& measurement_for_lidar_for_measidx_for_board_elem : measurement_for_lidar_for_measidx)
                {
                    const BoardID& board_id = measurement_for_lidar_for_measidx_for_board_elem.first;
                    const LiDARBoardMeasurement& lidar_board_measurement =
                        measurement_for_lidar_for_measidx_for_board_elem.second;

                    const std::string group = lidar_frame_id;
                    const std::string name = board_id + "_" + timeIndexStr(meas_idx, id);
                    const std::pair<std::string, std::string> key = std::make_pair(group, name);

                    if (solve && use_measurement_.count(key) && !use_measurement_.at(key))
                        continue;

                    for (const auto& lidar_reference_point_measurement_elem : lidar_board_measurement)
                    {
                        const ReferencePointID point_id = lidar_reference_point_measurement_elem.first;
                        const CartesianCoordinate& lidar_reference_point_measurement =
                            lidar_reference_point_measurement_elem.second;

                        ROS_DEBUG_STREAM_NAMED("Optimization",
                                               "Adding residual block for LiDAR reference point `"
                                                   << point_id << "` on board `" << board_id << "` observed by LiDAR `"
                                                   << lidar_frame_id << "` at meas_idx=" << meas_idx);

                        const ceres::ResidualBlockId residual_block_id = optimization_problem.AddResidualBlock(
                            new CartesianPointError::CostFunction(new CartesianPointError(
                                calibration_boards_.at(board_id).lidar_reference_points.at(point_id).position,
                                lidar_reference_point_measurement,
                                measurement_uncertainties_.at(lidar_frame_id).sigma_cartesian)),
                            nullptr,
                            board_poses_.at(board_id).at(meas_idx).translation.data(),
                            board_poses_.at(board_id).at(meas_idx).rotation.coeffs().data(),
                            sensor_poses_.at(std::make_pair(reference_frame_id_, lidar_frame_id)).translation.data(),
                            sensor_poses_.at(std::make_pair(reference_frame_id_, lidar_frame_id))
                                .rotation.coeffs()
                                .data());

                        board_residual_block_IDs_for_lidar_for_measidx[id][board_id].push_back(residual_block_id);
                    }
                }
            }
        }
    }

    // Add camera residual blocks
    std::map<FrameID, std::map<MeasurementIndex, std::vector<BoardResidualBlockIDs>>>
        camera_measurement_residual_blocks;
    for (const auto& camera_measurements_elem : camera_measurements_)
    {
        const FrameID& camera_frame_id = camera_measurements_elem.first;
        const std::map<MeasurementIndex, std::vector<CameraMeasurement>>& measurements_for_camera =
            camera_measurements_elem.second;

        std::map<MeasurementIndex, std::vector<BoardResidualBlockIDs>>& board_residual_block_IDs_for_camera =
            camera_measurement_residual_blocks[camera_frame_id];

        for (const auto& measurements_for_camera_for_measidx_elem : measurements_for_camera)
        {
            const MeasurementIndex meas_idx = measurements_for_camera_for_measidx_elem.first;
            const std::vector<CameraMeasurement>& measurements_for_camera_for_measidx =
                measurements_for_camera_for_measidx_elem.second;

            std::vector<BoardResidualBlockIDs>& board_residual_block_IDs_for_camera_for_measidx =
                board_residual_block_IDs_for_camera[meas_idx];
            board_residual_block_IDs_for_camera_for_measidx.resize(measurements_for_camera_for_measidx.size());

            for (int id = 0; id < measurements_for_camera_for_measidx.size(); id++)
            {
                const CameraMeasurement& measurement_for_camera_for_measidx = measurements_for_camera_for_measidx[id];

                for (const auto& measurement_for_camera_for_measidx_for_board_elem : measurement_for_camera_for_measidx)
                {
                    const BoardID& board_id = measurement_for_camera_for_measidx_for_board_elem.first;
                    const CameraBoardMeasurement& tag_measurements =
                        measurement_for_camera_for_measidx_for_board_elem.second;

                    const std::string group = camera_frame_id;
                    const std::string name = board_id + "_" + timeIndexStr(meas_idx, id);
                    const std::pair<std::string, std::string> key = std::make_pair(group, name);

                    if (solve && use_measurement_.count(key) && !use_measurement_.at(key))
                        continue;

                    for (const auto& tag_measurement_elem : tag_measurements)
                    {
                        const TagID& tag_id = tag_measurement_elem.first;
                        const TagMeasurement& tag_measurement = tag_measurement_elem.second;

                        ceres::ResidualBlockId residual_block_id{nullptr};
                        if ((board_poses_.count(board_id) > 0) && (board_poses_.at(board_id).count(meas_idx) > 0))
                        {
                            ROS_DEBUG_STREAM_NAMED("Optimization",
                                                   "Add residual block for corner `"
                                                       << "` of AprilTag `" << tag_id << "` on board `" << board_id
                                                       << "` observed by camera `" << camera_frame_id << "`...");

                            residual_block_id = optimization_problem.AddResidualBlock(
                                new CameraTagReprojectionError::CostFunction(new CameraTagReprojectionError(
                                    calibration_boards_.at(board_id).apriltags.at(tag_id).size,
                                    tag_measurement,
                                    calibration_boards_.at(board_id).apriltags.at(tag_id).board_from_tag(),
                                    camera_intrinsics_.at(camera_frame_id),
                                    measurement_uncertainties_.at(camera_frame_id).sigma_pixel)),
                                nullptr,
                                board_poses_.at(board_id).at(meas_idx).translation.data(),
                                board_poses_.at(board_id).at(meas_idx).rotation.coeffs().data(),
                                sensor_poses_.at(std::make_pair(reference_frame_id_, camera_frame_id))
                                    .translation.data(),
                                sensor_poses_.at(std::make_pair(reference_frame_id_, camera_frame_id))
                                    .rotation.coeffs()
                                    .data());
                        }

                        if (residual_block_id)
                        {
                            board_residual_block_IDs_for_camera_for_measidx[id][board_id].push_back(residual_block_id);
                        }
                    }
                }
            }
        }
    }

    // Add radar residual blocks
    std::map<FrameID, std::map<MeasurementIndex, std::vector<BoardResidualBlockIDs>>> radar_measurement_residual_blocks;
    for (const auto& radar_measurements_elem : radar_measurements_)
    {
        const FrameID& radar_frame_id = radar_measurements_elem.first;
        const std::map<MeasurementIndex, std::vector<RadarMeasurement>>& measurements_for_radar =
            radar_measurements_elem.second;

        std::map<MeasurementIndex, std::vector<BoardResidualBlockIDs>>& board_residual_block_IDs_for_radar =
            radar_measurement_residual_blocks[radar_frame_id];

        for (const auto& measurements_for_radar_for_measidx_elem : measurements_for_radar)
        {
            const MeasurementIndex meas_idx = measurements_for_radar_for_measidx_elem.first;
            const std::vector<RadarMeasurement>& measurements_for_radar_for_measidx =
                measurements_for_radar_for_measidx_elem.second;

            std::vector<BoardResidualBlockIDs>& board_residual_block_IDs_for_radar_for_measidx =
                board_residual_block_IDs_for_radar[meas_idx];
            board_residual_block_IDs_for_radar_for_measidx.resize(measurements_for_radar_for_measidx.size());

            for (int id = 0; id < measurements_for_radar_for_measidx.size(); id++)
            {
                const RadarMeasurement& measurement_for_radar_for_measidx = measurements_for_radar_for_measidx[id];

                for (const auto& measurement_for_radar_for_measidx_for_board_elem : measurement_for_radar_for_measidx)
                {
                    const BoardID& board_id = measurement_for_radar_for_measidx_for_board_elem.first;
                    const RadarBoardMeasurement& radar_target_measurements =
                        measurement_for_radar_for_measidx_for_board_elem.second;

                    const std::string group = radar_frame_id;
                    const std::string name = board_id + "_" + timeIndexStr(meas_idx, id);
                    const std::pair<std::string, std::string> key = std::make_pair(group, name);

                    if (solve && use_measurement_.count(key) && !use_measurement_.at(key))
                        continue;

                    for (const auto& target_elem : radar_target_measurements)
                    {
                        const TargetID& target_id = target_elem.first;
                        const PolarCoordinate& radar_target_measurement = target_elem.second;

                        ROS_DEBUG_STREAM_NAMED("Optimization",
                                               "Adding residual block for radar target `"
                                                   << target_id << "` on board `" << board_id << "` observed by radar `"
                                                   << radar_frame_id << "` at meas_idx=" << meas_idx);

                        const ceres::ResidualBlockId residual_block_id = optimization_problem.AddResidualBlock(
                            new PolarCoordinateError::CostFunction(
                                new PolarCoordinateError(calibration_boards_.at(board_id).radar_targets.at(target_id),
                                                         radar_target_measurement,
                                                         measurement_uncertainties_.at(radar_frame_id).sigma_polar)),
                            nullptr,
                            board_poses_.at(board_id).at(meas_idx).translation.data(),
                            board_poses_.at(board_id).at(meas_idx).rotation.coeffs().data(),
                            sensor_poses_.at(std::make_pair(reference_frame_id_, radar_frame_id)).translation.data(),
                            sensor_poses_.at(std::make_pair(reference_frame_id_, radar_frame_id))
                                .rotation.coeffs()
                                .data());

                        board_residual_block_IDs_for_radar_for_measidx[id][board_id].push_back(residual_block_id);
                    }
                }
            }
        }
    }

    if (solve)
    {
        ROS_INFO_STREAM_NAMED("Optimization", "Solving the optimization problem...");

        ceres::Solver::Options solve_options;
        solve_options.linear_solver_type = ceres::LinearSolverType::SPARSE_NORMAL_CHOLESKY;
        solve_options.num_linear_solver_threads = 1;
        solve_options.max_num_iterations = 1000;
        solve_options.max_solver_time_in_seconds = config_.max_solver_time;

        CeresIterationCallback iteration_callback{*this};
        solve_options.callbacks.push_back(&iteration_callback);
        solve_options.update_state_every_iteration = true;

        publishStatusMsg("Starting optimization...");

        ceres::Solver::Summary summary;
        ceres::Solve(solve_options, &optimization_problem, &summary);
        ROS_INFO_STREAM_NAMED("Optimization",
                              summary.BriefReport().c_str()
                                  << ", Time: " << summary.total_time_in_seconds * 1000.0 << "ms");
        ROS_DEBUG_STREAM_NAMED("Optimization", summary.FullReport());

        publishStatusMsg(
            "Optimization done: " +
            std::string(summary.termination_type == ceres::CONVERGENCE ? "CONVERGENCE" : "NO_CONVERGENCE") +
            ". Initial cost: " + std::to_string(summary.initial_cost) +
            " => Final cost: " + std::to_string(summary.final_cost));

        sensor_poses_changed_ = true;
        board_poses_changed_ = true;
    }
    else
    {
        // Calculate and publish Residuals
        ROS_INFO_STREAM_NAMED("Optimization", "Calculating the residuals...");
        sensor_calib_msgs::Residuals current_residuals_msg_;
        sensor_calib_msgs::Residuals overall_residuals_msg_;
        {
            ceres::Problem::EvaluateOptions eval_options;
            eval_options.apply_loss_function = false;

            double cost;
            std::vector<double> residuals;

            // Evaluate LiDAR Residuals
            for (const auto& lidar_res_elem : lidar_measurement_residual_blocks)
            {
                const FrameID& frame_id = lidar_res_elem.first;
                const std::map<MeasurementIndex, std::vector<BoardResidualBlockIDs>>&
                    board_residual_block_IDs_for_lidar = lidar_res_elem.second;

                std::map<BoardID, Eigen::VectorXd> board_residuals;

                for (const auto& board_residual_block_IDs_for_lidar_for_measidx_elem :
                     board_residual_block_IDs_for_lidar)
                {
                    const MeasurementIndex meas_idx = board_residual_block_IDs_for_lidar_for_measidx_elem.first;
                    const std::vector<BoardResidualBlockIDs>& board_residual_block_IDs_for_lidar_for_measidx =
                        board_residual_block_IDs_for_lidar_for_measidx_elem.second;

                    for (int id = 0; id < board_residual_block_IDs_for_lidar_for_measidx.size(); id++)
                    {
                        for (const auto& elem : board_residual_block_IDs_for_lidar_for_measidx[id])
                        {
                            const BoardID& board_id = elem.first;
                            const std::vector<ceres::ResidualBlockId>& residual_block_ids = elem.second;

                            eval_options.residual_blocks = residual_block_ids;

                            optimization_problem.Evaluate(eval_options, &cost, &residuals, nullptr, nullptr);

                            Eigen::VectorXd r;
                            r.resize(residual_block_ids.size());

                            const double sigma = measurement_uncertainties_.at(frame_id).sigma_cartesian;

                            for (int i = 0; i < residual_block_ids.size(); i++)
                            {
                                r(i) = std::sqrt(std::pow(residuals[i * 3 + 0] * sigma, 2) +
                                                 std::pow(residuals[i * 3 + 1] * sigma, 2) +
                                                 std::pow(residuals[i * 3 + 2] * sigma, 2));
                            }

                            sensor_calib_msgs::ResidualGroup new_residual_group_msg;
                            new_residual_group_msg.group = frame_id;
                            new_residual_group_msg.name = board_id + "_" + timeIndexStr(meas_idx, id);
                            new_residual_group_msg.unit = "cm";
                            new_residual_group_msg.rmse = std::sqrt(r.array().square().mean()) * 100.0;
                            new_residual_group_msg.count = residual_block_ids.size();
                            new_residual_group_msg.min_residual = r.minCoeff() * 100.0;
                            new_residual_group_msg.max_residual = r.maxCoeff() * 100.0;
                            current_residuals_msg_.residual_groups.push_back(new_residual_group_msg);

                            if (!board_residuals.count(board_id))
                            {
                                board_residuals[board_id] = Eigen::VectorXd(0);
                            }
                            const Eigen::VectorXd old = board_residuals[board_id];
                            board_residuals[board_id].resize(old.size() + r.size());
                            board_residuals[board_id] << old, r;
                        }
                    }
                }

                for (const auto& elem : board_residuals)
                {
                    const BoardID& board_id = elem.first;
                    const Eigen::VectorXd& residuals = elem.second;

                    sensor_calib_msgs::ResidualGroup board_residual_group_msg;
                    board_residual_group_msg.group = frame_id;
                    board_residual_group_msg.name = board_id;
                    board_residual_group_msg.unit = "cm";
                    board_residual_group_msg.rmse = std::sqrt(residuals.array().square().mean()) * 100.0;
                    board_residual_group_msg.count = residuals.size();
                    board_residual_group_msg.min_residual = residuals.minCoeff() * 100.0;
                    board_residual_group_msg.max_residual = residuals.maxCoeff() * 100.0;
                    overall_residuals_msg_.residual_groups.push_back(board_residual_group_msg);
                }
            }

            // Evaluate Camera Residuals
            for (const auto& camera_res_elem : camera_measurement_residual_blocks)
            {
                const FrameID& frame_id = camera_res_elem.first;
                const std::map<MeasurementIndex, std::vector<BoardResidualBlockIDs>>&
                    board_residual_block_IDs_for_camera = camera_res_elem.second;

                std::map<BoardID, Eigen::VectorXd> board_residuals;

                for (const auto& board_residual_block_IDs_for_camera_for_measidx_elem :
                     board_residual_block_IDs_for_camera)
                {
                    const MeasurementIndex meas_idx = board_residual_block_IDs_for_camera_for_measidx_elem.first;
                    const std::vector<BoardResidualBlockIDs>& board_residual_block_IDs_for_camera_for_measidx =
                        board_residual_block_IDs_for_camera_for_measidx_elem.second;

                    for (int id = 0; id < board_residual_block_IDs_for_camera_for_measidx.size(); id++)
                    {
                        for (const auto& elem : board_residual_block_IDs_for_camera_for_measidx[id])
                        {
                            const BoardID& board_id = elem.first;
                            const std::vector<ceres::ResidualBlockId>& residual_block_ids = elem.second;

                            eval_options.residual_blocks = residual_block_ids;

                            optimization_problem.Evaluate(eval_options, &cost, &residuals, nullptr, nullptr);

                            Eigen::VectorXd r;
                            r.resize(residual_block_ids.size() * 4);

                            const double sigma = measurement_uncertainties_.at(frame_id).sigma_pixel;

                            for (int i = 0; i < residual_block_ids.size(); i++)
                            {
                                r(i * 4 + 0) = std::sqrt(std::pow(residuals[i * 12 + 0] * sigma, 2) +
                                                         std::pow(residuals[i * 12 + 1] * sigma, 2));
                                r(i * 4 + 1) = std::sqrt(std::pow(residuals[i * 12 + 3] * sigma, 2) +
                                                         std::pow(residuals[i * 12 + 4] * sigma, 2));
                                r(i * 4 + 2) = std::sqrt(std::pow(residuals[i * 12 + 6] * sigma, 2) +
                                                         std::pow(residuals[i * 12 + 7] * sigma, 2));
                                r(i * 4 + 3) = std::sqrt(std::pow(residuals[i * 12 + 9] * sigma, 2) +
                                                         std::pow(residuals[i * 12 + 10] * sigma, 2));
                            }

                            sensor_calib_msgs::ResidualGroup new_residual_group_msg;
                            new_residual_group_msg.group = frame_id;
                            new_residual_group_msg.name = board_id + "_" + timeIndexStr(meas_idx, id);
                            new_residual_group_msg.unit = "px";
                            new_residual_group_msg.rmse = std::sqrt(r.array().square().mean());
                            new_residual_group_msg.count = residual_block_ids.size() * 4;
                            new_residual_group_msg.min_residual = r.minCoeff();
                            new_residual_group_msg.max_residual = r.maxCoeff();
                            current_residuals_msg_.residual_groups.push_back(new_residual_group_msg);

                            if (!board_residuals.count(board_id))
                            {
                                board_residuals[board_id] = Eigen::VectorXd(0);
                            }
                            const Eigen::VectorXd old = board_residuals[board_id];
                            board_residuals[board_id].resize(old.size() + r.size());
                            board_residuals[board_id] << old, r;
                        }
                    }
                }

                for (const auto& elem : board_residuals)
                {
                    const BoardID& board_id = elem.first;
                    const Eigen::VectorXd& residuals = elem.second;

                    sensor_calib_msgs::ResidualGroup board_residual_group_msg;
                    board_residual_group_msg.group = frame_id;
                    board_residual_group_msg.name = board_id;
                    board_residual_group_msg.unit = "px";
                    board_residual_group_msg.rmse = std::sqrt(residuals.array().square().mean());
                    board_residual_group_msg.count = residuals.size();
                    board_residual_group_msg.min_residual = residuals.minCoeff();
                    board_residual_group_msg.max_residual = residuals.maxCoeff();
                    overall_residuals_msg_.residual_groups.push_back(board_residual_group_msg);
                }
            }

            // Evaluate Radar Residuals (in Cartesian Coordinates)
            for (const auto& radar_measurements_elem : radar_measurements_)
            {
                const FrameID& radar_frame_id = radar_measurements_elem.first;
                const std::map<MeasurementIndex, std::vector<RadarMeasurement>>& measurements_for_radar =
                    radar_measurements_elem.second;

                std::map<BoardID, Eigen::VectorXd> board_residuals;

                std::map<MeasurementIndex, std::vector<BoardResidualBlockIDs>>& board_residual_block_IDs_for_radar =
                    radar_measurement_residual_blocks[radar_frame_id];

                for (const auto& measurements_for_radar_for_measidx_elem : measurements_for_radar)
                {
                    const MeasurementIndex meas_idx = measurements_for_radar_for_measidx_elem.first;
                    const std::vector<RadarMeasurement>& measurements_for_radar_for_measidx =
                        measurements_for_radar_for_measidx_elem.second;

                    std::vector<BoardResidualBlockIDs>& board_residual_block_IDs_for_radar_for_measidx =
                        board_residual_block_IDs_for_radar[meas_idx];
                    board_residual_block_IDs_for_radar_for_measidx.resize(measurements_for_radar_for_measidx.size());

                    for (int id = 0; id < measurements_for_radar_for_measidx.size(); id++)
                    {
                        const RadarMeasurement& measurement_for_radar_for_measidx =
                            measurements_for_radar_for_measidx[id];

                        for (const auto& measurement_for_radar_for_measidx_for_board_elem :
                             measurement_for_radar_for_measidx)
                        {
                            const BoardID& board_id = measurement_for_radar_for_measidx_for_board_elem.first;
                            const RadarBoardMeasurement& radar_target_measurements =
                                measurement_for_radar_for_measidx_for_board_elem.second;

                            const std::string group = radar_frame_id;
                            const std::string name = board_id + "_" + timeIndexStr(meas_idx, id);

                            Eigen::VectorXd r(radar_target_measurements.size());
                            int i = 0;
                            for (const auto target_elem : radar_target_measurements)
                            {
                                const TargetID& target_id = target_elem.first;
                                const PolarCoordinate& radar_target_measurement = target_elem.second;

                                const Eigen::Vector3d ref_t_ref2board =
                                    board_poses_.at(board_id).at(meas_idx).translation;
                                const Eigen::Vector3d ref_t_ref2sensor =
                                    sensor_poses_.at(std::make_pair(reference_frame_id_, radar_frame_id)).translation;

                                const Eigen::Quaterniond ref_R_board = Eigen::Map<const Eigen::Quaterniond>(
                                    board_poses_.at(board_id).at(meas_idx).rotation.coeffs().data());
                                const Eigen::Quaterniond ref_R_sensor = Eigen::Map<const Eigen::Quaterniond>(
                                    sensor_poses_.at(std::make_pair(reference_frame_id_, radar_frame_id))
                                        .rotation.coeffs()
                                        .data());

                                const Eigen::Isometry3d ref_from_board =
                                    Eigen::Translation3d(ref_t_ref2board) * Eigen::Isometry3d(ref_R_board);
                                const Eigen::Isometry3d ref_from_sensor =
                                    Eigen::Translation3d(ref_t_ref2sensor) * Eigen::Isometry3d(ref_R_sensor);
                                const Eigen::Isometry3d sensor_from_ref = ref_from_sensor.inverse();

                                const Eigen::Vector3d predicted_sensor_point =
                                    sensor_from_ref * ref_from_board *
                                    calibration_boards_.at(board_id).radar_targets.at(target_id);

                                r(i++) = (predicted_sensor_point - radar_target_measurement.cartesian()).norm();
                            }

                            sensor_calib_msgs::ResidualGroup new_residual_group_msg;
                            new_residual_group_msg.group = radar_frame_id;
                            new_residual_group_msg.name = board_id + "_" + timeIndexStr(meas_idx, id);
                            new_residual_group_msg.unit = "cm";
                            new_residual_group_msg.rmse = std::sqrt(r.array().square().mean()) * 100.0;
                            new_residual_group_msg.count = i;
                            new_residual_group_msg.min_residual = r.minCoeff() * 100.0;
                            new_residual_group_msg.max_residual = r.maxCoeff() * 100.0;
                            current_residuals_msg_.residual_groups.push_back(new_residual_group_msg);

                            if (!board_residuals.count(board_id))
                            {
                                board_residuals[board_id] = Eigen::VectorXd(0);
                            }
                            const Eigen::VectorXd old = board_residuals[board_id];
                            board_residuals[board_id].resize(old.size() + r.size());
                            board_residuals[board_id] << old, r;
                        }
                    }
                }

                for (const auto& elem : board_residuals)
                {
                    const BoardID& board_id = elem.first;
                    const Eigen::VectorXd& residuals = elem.second;

                    sensor_calib_msgs::ResidualGroup board_residual_group_msg;
                    board_residual_group_msg.group = radar_frame_id;
                    board_residual_group_msg.name = board_id;
                    board_residual_group_msg.unit = "cm";
                    board_residual_group_msg.rmse = std::sqrt(residuals.array().square().mean()) * 100.0;
                    board_residual_group_msg.count = residuals.size();
                    board_residual_group_msg.min_residual = residuals.minCoeff() * 100.0;
                    board_residual_group_msg.max_residual = residuals.maxCoeff() * 100.0;
                    overall_residuals_msg_.residual_groups.push_back(board_residual_group_msg);
                }
            }
        }
        residuals_pub_.publish(current_residuals_msg_);
        overall_residuals_pub_.publish(overall_residuals_msg_);
    }
    if (!solve)
        ROS_INFO_STREAM_NAMED("Optimization", "Idle");
}
