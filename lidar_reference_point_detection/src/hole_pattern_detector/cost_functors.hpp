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

#include <ceres/ceres.h>
#include <Eigen/Core>
#include <Eigen/Dense>

#include "types.h"

class EdgePointError
{
  public:
    using CostFunction = ceres::AutoDiffCostFunction<EdgePointError, 2, 3, 4>;

    EdgePointError(const float x,
                   const float y,
                   const float z,
                   const std::map<ReferencePointID, LiDARReferencePointConfig>& board_holes,
                   const double r_limit)
        : edge_point_(x, y, z), board_holes_(board_holes), r_limit_(r_limit)
    {
    }

    template<typename T>
    bool operator()(const T* const lidar_to_board_translation_raw,
                    const T* const lidar_to_board_quaternion_raw,
                    T* residuals) const noexcept
    {
        using std::abs;
        using std::min;

        using Vector3 = Eigen::Matrix<T, 3, 1>;
        using Quaternion = Eigen::Quaternion<T>;
        using HTM = Eigen::Transform<T, 3, Eigen::Isometry>;
        using Vector2 = Eigen::Matrix<T, 2, 1>;

        const Vector3 lidar_t_board = Eigen::Map<const Vector3>(lidar_to_board_translation_raw);
        const Quaternion lidar_R_board = Eigen::Map<const Quaternion>(lidar_to_board_quaternion_raw);

        const HTM lidar_from_board = Eigen::Translation<T, 3>(lidar_t_board) * HTM(lidar_R_board);
        const Vector3 board_edge_point = lidar_from_board.inverse() * edge_point_.cast<T>();
        const Vector2 board_edge_point_xy(board_edge_point.x(), board_edge_point.y());

        residuals[0] = static_cast<T>(r_limit_);
        residuals[1] = static_cast<T>(0);

        for (const auto& reference_point_elem : board_holes_)
        {
            const LiDARReferencePointConfig& hole_config = reference_point_elem.second;
            const Eigen::Vector2d board_hole_position(hole_config.position.x(), hole_config.position.y());
            const double& hole_radius = hole_config.radius;
            const T hole_xy_error = abs((board_edge_point_xy - board_hole_position.cast<T>()).norm() - hole_radius);

            if (hole_xy_error < residuals[0])
            {
                residuals[0] = min(hole_xy_error, residuals[0]);
                residuals[1] = board_edge_point.z() - hole_config.position.z();
            }
        }

        return true;
    }

    const Eigen::Vector3d edge_point_;
    const std::map<ReferencePointID, LiDARReferencePointConfig> board_holes_;
    const double r_limit_;
};