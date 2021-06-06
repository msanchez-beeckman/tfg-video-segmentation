/*
 * Copyright (c) 2020-2021, Marco Sánchez Beeckman
 * All rights reserved.
 *
 * This source code is licensed under the BSD-style license found in the
 * LICENSE file in the root directory of this source tree.
 */

#ifndef TFG_VIDEO_SEGMENTATION_TRACKING_H
#define TFG_VIDEO_SEGMENTATION_TRACKING_H

#include <opencv2/core.hpp>

#include "TrackTable.h"

namespace tfg {

void computeStructureTensorEigenvalues(const cv::Mat &image, cv::Mat &lambda1, cv::Mat &lambda2);
void addTracksToUncoveredZones(const cv::Mat &image, int frame, tfg::TrackTable &trackTable, std::vector<float> &weights, int trackDensity, int coverRadius, double rho, bool keepStructurelessPoints);
void followExistingTracks(const cv::Mat &flow, const cv::Mat &rflow, int frame, tfg::TrackTable &trackTable, bool trackMotionBoundaries);

} // namespace tfg

#endif // TFG_VIDEO_SEGMENTATION_TRACKING_H