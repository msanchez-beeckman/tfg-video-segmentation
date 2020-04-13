
#ifndef TFG_VIDEO_SEGMENTATION_TRACKING_H
#define TFG_VIDEO_SEGMENTATION_TRACKING_H

#include <opencv4/opencv2/core.hpp>
#include "TrackTable.h"

namespace tfg {

    void computeStructureTensorEigenvalues(const cv::Mat &image, cv::Mat &lambda1, cv::Mat &lambda2);
    void addTracksToUncoveredZones(const cv::Mat &image, int frame, tfg::TrackTable &trackTable, std::vector<float> &weights, int trackDensity, int coverRadius, double rho);
    void followExistingTracks(const cv::Mat &flow, const cv::Mat &rflow, int frame, tfg::TrackTable &trackTable);

}

#endif //TFG_VIDEO_SEGMENTATION_TRACKING_H