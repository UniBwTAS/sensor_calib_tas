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

#include "types.h"

class CameraTagReprojectionError
{
  public:
    using CostFunction = ceres::AutoDiffCostFunction<CameraTagReprojectionError, 12, 3, 4, 3, 4>;

    CameraTagReprojectionError(const double tag_size,
                               const std::array<Eigen::Vector2d, 4>& measured_image_corner_positions,
                               const Eigen::Isometry3d& board_from_tag,
                               const CameraIntrinsics& camera_intrinsics,
                               const double sigma)
        : measured_image_corner_positions_(measured_image_corner_positions),
          tag_size_(tag_size),
          board_from_tag_(board_from_tag),
          camera_intrinsics_(camera_intrinsics),
          sigma_(sigma)
    {
    }

    // adapted from image_geometry/pinhole_camera_model: PinholeCameraModel::unrectifyPoint (ROS diamondback)
    template<typename T>
    static Eigen::Matrix<T, 2, 1> unrectifyPoint(const Eigen::Matrix<T, 2, 1>& uv_rect,
                                                 const CameraIntrinsics& camera_intrinsics)
    {
        const T fx = static_cast<T>(camera_intrinsics.projection_matrix(0, 0));
        const T fy = static_cast<T>(camera_intrinsics.projection_matrix(1, 1));
        const T cx = static_cast<T>(camera_intrinsics.projection_matrix(0, 2));
        const T cy = static_cast<T>(camera_intrinsics.projection_matrix(1, 2));
        const T Tx = static_cast<T>(camera_intrinsics.projection_matrix(0, 3));
        const T Ty = static_cast<T>(camera_intrinsics.projection_matrix(1, 3));
        const T k1 = static_cast<T>(camera_intrinsics.distortion_parameters(0));
        const T k2 = static_cast<T>(camera_intrinsics.distortion_parameters(1));
        const T p1 = static_cast<T>(camera_intrinsics.distortion_parameters(2));
        const T p2 = static_cast<T>(camera_intrinsics.distortion_parameters(3));
        const T k3 = static_cast<T>(camera_intrinsics.distortion_parameters(4));

        // x <- (u - c'x) / f'x
        // y <- (v - c'y) / f'y
        // c'x, f'x, etc. (primed) come from "new camera matrix" P[0:3, 0:3].
        T x = (uv_rect.x() - cx - Tx) / fx;
        T y = (uv_rect.y() - cy - Ty) / fy;
        // [X Y W]^T <- R^-1 * [x y 1]^T
        T X = camera_intrinsics.rectification_matrix(0, 0) * x + camera_intrinsics.rectification_matrix(1, 0) * y +
              camera_intrinsics.rectification_matrix(2, 0);
        T Y = camera_intrinsics.rectification_matrix(0, 1) * x + camera_intrinsics.rectification_matrix(1, 1) * y +
              camera_intrinsics.rectification_matrix(2, 1);
        T W = camera_intrinsics.rectification_matrix(0, 2) * x + camera_intrinsics.rectification_matrix(1, 2) * y +
              camera_intrinsics.rectification_matrix(2, 2);
        // x' <- X/W, y' <- Y/W
        T xp = X / W;
        T yp = Y / W;
        // x'' <- x'(1+k1*r^2+k2*r^4+k3*r^6) + 2p1*x'*y' + p2(r^2+2x'^2)
        // y'' <- y'(1+k1*r^2+k2*r^4+k3*r^6) + p1(r^2+2y'^2) + 2p2*x'*y'
        // where r^2 = x'^2 + y'^2
        T r2 = xp * xp + yp * yp;
        T r4 = r2 * r2;
        T r6 = r4 * r2;
        T a1 = static_cast<T>(2) * xp * yp;
        T barrel_correction = static_cast<T>(1) + k1 * r2 + k2 * r4 + k3 * r6;
        T xpp = xp * barrel_correction + p1 * a1 + p2 * (r2 + static_cast<T>(2) * (xp * xp));
        T ypp = yp * barrel_correction + p1 * (r2 + static_cast<T>(2) * (yp * yp)) + p2 * a1;
        // map_x(u,v) <- x''fx + cx
        // map_y(u,v) <- y''fy + cy
        // cx, fx, etc. come from original camera matrix K.
        return Eigen::Matrix<T, 2, 1>(
            xpp * camera_intrinsics.intrinsic_matrix(0, 0) + camera_intrinsics.intrinsic_matrix(0, 2),
            ypp * camera_intrinsics.intrinsic_matrix(1, 1) + camera_intrinsics.intrinsic_matrix(1, 2));
    }

    template<typename T>
    static Eigen::Matrix<T, 2, 1> predictCornerPosition(const Eigen::Matrix<T, 3, 1>& camera_point,
                                                        const CameraIntrinsics& camera_intrinsics) noexcept
    {
        Eigen::Matrix<T, 2, 1> uv_rect =
            (camera_intrinsics.projection_matrix * camera_point.homogeneous()).hnormalized();

        if (camera_intrinsics.rectified)
        {
            return uv_rect;
        }
        else
        {
            return unrectifyPoint(uv_rect, camera_intrinsics);
        }
    }

    template<typename T>
    bool operator()(const T* const ref_to_board_translation_raw,
                    const T* const ref_to_board_quaternion_raw,
                    const T* const ref_to_camera_translation_raw,
                    const T* const ref_to_camera_quaternion_raw,
                    T* residuals) const noexcept
    {
        using Vector3 = Eigen::Matrix<T, 3, 1>;
        using Quaternion = Eigen::Quaternion<T>;
        using HTM = Eigen::Transform<T, 3, Eigen::Isometry>;

        const Vector3 ref_t_ref2board = Eigen::Map<const Vector3>(ref_to_board_translation_raw);
        const Vector3 ref_t_ref2camera = Eigen::Map<const Vector3>(ref_to_camera_translation_raw);

        const Quaternion ref_R_board = Eigen::Map<const Quaternion>(ref_to_board_quaternion_raw);
        const Quaternion ref_R_camera = Eigen::Map<const Quaternion>(ref_to_camera_quaternion_raw);

        const HTM ref_from_board = Eigen::Translation<T, 3>(ref_t_ref2board) * HTM(ref_R_board);
        const HTM ref_from_camera = Eigen::Translation<T, 3>(ref_t_ref2camera) * HTM(ref_R_camera);
        const HTM camera_from_ref = ref_from_camera.inverse();

        // Iterate over corners
        for (int i = 0; i < 4; i++)
        {
            Eigen::Vector3d tag_corner_position;
            const double half_tag_size = tag_size_ / 2.0;

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

            const Vector3 board_point_position = board_from_tag_ * tag_corner_position.cast<T>();
            const Vector3 camera_point_position = camera_from_ref * ref_from_board * board_point_position;

            const Eigen::Matrix<T, 2, 1> predicted_image_position =
                predictCornerPosition(camera_point_position, camera_intrinsics_);

            residuals[i * 3 + 0] =
                (predicted_image_position.x() - static_cast<T>(measured_image_corner_positions_[i].x())) *
                static_cast<T>(1.0 / sigma_);
            residuals[i * 3 + 1] =
                (predicted_image_position.y() - static_cast<T>(measured_image_corner_positions_[i].y())) *
                static_cast<T>(1.0 / sigma_);

            // Points cannot be behind the camera
            if (camera_point_position[2] <= static_cast<T>(0))
            {
                residuals[i * 3 + 2] = camera_point_position[2] * static_cast<T>(1000);
            }
            else
            {
                residuals[i * 3 + 2] = static_cast<T>(0);
            }
        }

        return true;
    }

  private:
    const double tag_size_;
    const std::array<Eigen::Vector2d, 4> measured_image_corner_positions_;
    const Eigen::Isometry3d board_from_tag_;
    const CameraIntrinsics camera_intrinsics_;
    const double sigma_;
};

class CartesianPointError
{
  public:
    using CostFunction = ceres::AutoDiffCostFunction<CartesianPointError, 3, 3, 4, 3, 4>;

    CartesianPointError(const CartesianCoordinate& board_to_reference_point,
                        const CartesianCoordinate& measured_position,
                        const double sigma)
        : board_to_reference_point_(board_to_reference_point),
          measured_reference_point_position_(measured_position),
          sigma_(sigma)
    {
    }

    template<typename T>
    bool operator()(const T* const ref_to_board_translation_raw,
                    const T* const ref_to_board_quaternion_raw,
                    const T* const ref_to_sensor_translation_raw,
                    const T* const ref_to_sensor_quaternion_raw,
                    T* residuals) const noexcept
    {
        using Vector3 = Eigen::Matrix<T, 3, 1>;
        using Quaternion = Eigen::Quaternion<T>;
        using HTM = Eigen::Transform<T, 3, Eigen::Isometry>;

        const Vector3 ref_t_ref2board = Eigen::Map<const Vector3>(ref_to_board_translation_raw);
        const Vector3 ref_t_ref2sensor = Eigen::Map<const Vector3>(ref_to_sensor_translation_raw);

        const Quaternion ref_R_board = Eigen::Map<const Quaternion>(ref_to_board_quaternion_raw);
        const Quaternion ref_R_sensor = Eigen::Map<const Quaternion>(ref_to_sensor_quaternion_raw);

        const HTM ref_from_board = Eigen::Translation<T, 3>(ref_t_ref2board) * HTM(ref_R_board);
        const HTM ref_from_sensor = Eigen::Translation<T, 3>(ref_t_ref2sensor) * HTM(ref_R_sensor);
        const HTM sensor_from_ref = ref_from_sensor.inverse();

        const CartesianCoordinate board_point_position = board_to_reference_point_;
        const Vector3 predicted_sensor_point = sensor_from_ref * ref_from_board * board_point_position.cast<T>();

        residuals[0] = (predicted_sensor_point.x() - static_cast<T>(measured_reference_point_position_.x())) *
                       static_cast<T>(1.0 / sigma_);
        residuals[1] = (predicted_sensor_point.y() - static_cast<T>(measured_reference_point_position_.y())) *
                       static_cast<T>(1.0 / sigma_);
        residuals[2] = (predicted_sensor_point.z() - static_cast<T>(measured_reference_point_position_.z())) *
                       static_cast<T>(1.0 / sigma_);

        return true;
    }

    const CartesianCoordinate board_to_reference_point_;
    const CartesianCoordinate measured_reference_point_position_;
    const double sigma_;
};

class PolarCoordinateError
{
  public:
    using CostFunction = ceres::AutoDiffCostFunction<PolarCoordinateError, 3, 3, 4, 3, 4>;

    PolarCoordinateError(const CartesianCoordinate& board_to_target,
                         const PolarCoordinate& detection,
                         const PolarCoordinate& sigma)
        : board_to_target_(board_to_target), detection_(detection), sigma_(sigma)
    {
    }

    template<typename T>
    bool operator()(const T* const ref_to_board_translation_raw,
                    const T* const ref_to_board_quaternion_raw,
                    const T* const ref_to_sensor_translation_raw,
                    const T* const ref_to_sensor_quaternion_raw,
                    T* residuals) const noexcept
    {
        using std::asin;
        using std::atan2;
        using std::sqrt;

        using Vector3 = Eigen::Matrix<T, 3, 1>;
        using Quaternion = Eigen::Quaternion<T>;
        using HTM = Eigen::Transform<T, 3, Eigen::Isometry>;

        const Vector3 ref_t_ref2board = Eigen::Map<const Vector3>(ref_to_board_translation_raw);
        const Vector3 ref_t_ref2sensor = Eigen::Map<const Vector3>(ref_to_sensor_translation_raw);

        const Quaternion ref_R_board = Eigen::Map<const Quaternion>(ref_to_board_quaternion_raw);
        const Quaternion ref_R_sensor = Eigen::Map<const Quaternion>(ref_to_sensor_quaternion_raw);

        const HTM ref_from_board = Eigen::Translation<T, 3>(ref_t_ref2board) * HTM(ref_R_board);
        const HTM ref_from_sensor = Eigen::Translation<T, 3>(ref_t_ref2sensor) * HTM(ref_R_sensor);
        const HTM sensor_from_ref = ref_from_sensor.inverse();

        const Vector3 predicted_sensor_point = sensor_from_ref * ref_from_board * board_to_target_.cast<T>();

        const T predicted_range = sqrt(predicted_sensor_point.x() * predicted_sensor_point.x() +
                                       predicted_sensor_point.y() * predicted_sensor_point.y() +
                                       predicted_sensor_point.z() * predicted_sensor_point.z());
        const T predicted_azimuth = atan2(predicted_sensor_point.y(), predicted_sensor_point.x());
        const T predicted_elevation = asin(predicted_sensor_point.z() / predicted_range);

        residuals[0] = (predicted_range - static_cast<T>(detection_.range)) * static_cast<T>(1.0 / sigma_.range);
        residuals[1] = (predicted_azimuth - static_cast<T>(detection_.azimuth)) *
                       static_cast<T>(1.0 / (sigma_.azimuth * M_PI / 180.0));
        residuals[2] = (predicted_elevation - static_cast<T>(detection_.elevation)) *
                       static_cast<T>(1.0 / (sigma_.elevation * M_PI / 180.0));

        return true;
    }

    const CartesianCoordinate board_to_target_;
    const PolarCoordinate detection_;
    const PolarCoordinate sigma_;
};