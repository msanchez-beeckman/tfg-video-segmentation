/*
 * Copyright (c) 2020-2021, Marco Sánchez Beeckman
 * All rights reserved.
 *
 * This source code is licensed under the BSD-style license found in the
 * LICENSE file in the root directory of this source tree.
 */

#ifndef TFG_VIDEO_SEGMENTATION_MOTIONMODEL_H
#define TFG_VIDEO_SEGMENTATION_MOTIONMODEL_H

#include <opencv2/core.hpp>

#include "TrackTable.h"


namespace tfg {

class MotionModel {
private:
    std::shared_ptr<tfg::TrackTable> trackTable;
    std::vector<float> weights2;
    float tau2;

    std::vector<cv::Matx33f> homographies;
    std::vector<float> residuals2;
    float cost;

    void computeHomographiesWLS();
    void computeHomographiesRANSAC(int iterations, float tolerance, std::vector<std::vector<int>> &inliers);
    void computeResiduals2();
    void computeModelCost();

public:
    MotionModel(std::shared_ptr<tfg::TrackTable> &trackTable, float tau2);
    ~MotionModel();

    void fitFromWeights(std::vector<float> &weights2);
    void fitFromRANSAC(int iterations, float tolerance, std::vector<std::vector<int>> &inliers);

    void printHomography(int n);
    inline std::vector<float> getResiduals2() const {
        return residuals2;
    };
    inline float getCost() const {
        return cost;
    };
};

} // namespace tfg

#endif // TFG_VIDEO_SEGMENTATION_MOTIONMODEL_H