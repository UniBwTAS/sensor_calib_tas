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

#include <pcl/impl/pcl_base.hpp>
#include <pcl_ros/point_cloud.h>

namespace pcl
{
struct PointXYZR
{
    PCL_ADD_POINT4D
    std::uint16_t ring;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
} // namespace pcl
POINT_CLOUD_REGISTER_POINT_STRUCT(pcl::PointXYZR, (float, x, x)(float, y, y)(float, z, z)(std::uint16_t, ring, ring))