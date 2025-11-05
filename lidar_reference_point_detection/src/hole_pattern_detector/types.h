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

using ReferencePointID = std::uint8_t;
using CartesianCoordinate = Eigen::Vector3d;

struct LiDARReferencePointConfig
{
    double radius;
    CartesianCoordinate position;
};