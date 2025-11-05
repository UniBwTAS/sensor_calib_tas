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

struct Pose
{
    Eigen::Vector3d translation{Eigen::Vector3d::Zero()};
    Eigen::Quaterniond rotation{Eigen::Quaterniond::Identity()};

    Eigen::Isometry3d transform() const
    {
        return (Eigen::Translation3d(translation) * Eigen::Isometry3d(rotation)).inverse();
    }

    template<typename Archive>
    void serialize(Archive& ar, const unsigned int version)
    {
        ar & translation;
        ar & rotation.coeffs();
    }
};

struct PolarCoordinate
{
    double range;
    double azimuth;
    double elevation;

    PolarCoordinate() : range(0), azimuth(0), elevation(0)
    {
    }
    PolarCoordinate(double r, double a, double e) : range(r), azimuth(a), elevation(e)
    {
    }

    PolarCoordinate& operator+=(const PolarCoordinate& rhs)
    {
        range += rhs.range;
        azimuth += rhs.azimuth;
        elevation += rhs.elevation;
        return *this;
    }
    PolarCoordinate& operator/=(const int i)
    {
        range /= i;
        azimuth /= i;
        elevation /= i;
        return *this;
    }

    Eigen::Vector3d cartesian() const
    {
        Eigen::Vector3d cartesian;
        cartesian.x() = range * cos(azimuth) * cos(elevation);
        cartesian.y() = range * sin(azimuth) * cos(elevation);
        cartesian.z() = range * sin(elevation);
        return cartesian;
    }

    template<typename Archive>
    void serialize(Archive& ar, const unsigned int version)
    {
        ar & range;
        ar & azimuth;
        ar & elevation;
    }
};

using BoardID = std::string;
using FrameID = std::string;
using MeasurementIndex = std::size_t;

using ReferencePointID = std::uint8_t;
using CartesianCoordinate = Eigen::Vector3d;
using LiDARBoardMeasurement = std::map<ReferencePointID, CartesianCoordinate>;
using LiDARMeasurement = std::map<BoardID, LiDARBoardMeasurement>;

using TagID = std::string;
using ImageCoordinate = Eigen::Vector2d;
using TagMeasurement = std::array<ImageCoordinate, 4>;
using CameraBoardMeasurement = std::map<TagID, TagMeasurement>;
using CameraMeasurement = std::map<BoardID, CameraBoardMeasurement>;

using TargetID = std::int16_t;
using RadarBoardMeasurement = std::map<TargetID, PolarCoordinate>;
using RadarMeasurement = std::map<BoardID, RadarBoardMeasurement>;

using BoardResidualBlockIDs = std::map<BoardID, std::vector<ceres::ResidualBlockId>>;

struct AprilTagConfig
{
    double size;
    Pose tag_pose;

    Eigen::Isometry3d board_from_tag() const noexcept
    {
        return Eigen::Translation3d(tag_pose.translation) * tag_pose.rotation;
    };

    template<typename Archive>
    void serialize(Archive& ar, const unsigned int /*version*/)
    {
        ar & size;
        ar & tag_pose;
    }
};

struct LiDARReferencePointConfig
{
    double radius;
    CartesianCoordinate position;

    template<typename Archive>
    void serialize(Archive& ar, const unsigned int /*version*/)
    {
        ar & radius;
        ar & position;
    }
};

struct BoardConfig
{
    std::map<TagID, AprilTagConfig> apriltags;
    std::map<ReferencePointID, LiDARReferencePointConfig> lidar_reference_points;
    std::map<TargetID, CartesianCoordinate> radar_targets;

    template<typename Archive>
    void serialize(Archive& ar, const unsigned int version)
    {
        ar & apriltags;
        ar & lidar_reference_points;
        ar & radar_targets;
    }
};

struct Uncertainties
{
    double sigma_pixel;
    double sigma_cartesian;
    PolarCoordinate sigma_polar;

    template<typename Archive>
    void serialize(Archive& ar, const unsigned int /*version*/)
    {
        ar & sigma_pixel;
        ar & sigma_cartesian;
        ar & sigma_polar;
    }
};

struct CameraIntrinsics
{
    bool rectified;
    double img_width;
    double img_height;

    Eigen::Matrix3d intrinsic_matrix;
    Eigen::Matrix<double, 3, 4> projection_matrix;
    Eigen::Matrix<double, 5, 1> distortion_parameters;
    Eigen::Matrix<double, 3, 3> rectification_matrix;

    template<typename Archive>
    void serialize(Archive& ar, const unsigned int version)
    {
        ar & rectified;
        ar & img_width;
        ar & img_height;
        ar & intrinsic_matrix;
        ar & projection_matrix;
        ar & distortion_parameters;
        ar & rectification_matrix;
    }
};