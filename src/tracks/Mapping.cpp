/*
 * Copyright (c) 2020-2021, Marco Sánchez Beeckman
 * All rights reserved.
 *
 * This source code is licensed under the BSD-style license found in the
 * LICENSE file in the root directory of this source tree.
 */

#include <vector>

#include <opencv2/core.hpp>

#include "Mapping.h"

namespace tfg {

Mapping::Mapping() {}
Mapping::Mapping(const std::vector<cv::Vec2f> &origin, const std::vector<cv::Vec2f> &destination, const std::vector<unsigned int> &trajectory) {
    this->origin = origin;
    this->destination = destination;
    this->trajectory = trajectory;
}

Mapping::~Mapping() {}

/**
 * Add an origin-destination pair of points to the mapping, and assign it to a trajectory.
 * @param p0 The origin point.
 * @param p1 The destination point.
 * @param label The trajectory the points belong to.
 */
void Mapping::addPoint(cv::Vec2f &p0, cv::Vec2f &p1, unsigned int label) {
    origin.push_back(p0);
    destination.push_back(p1);
    trajectory.push_back(label);
}

} // namespace tfg