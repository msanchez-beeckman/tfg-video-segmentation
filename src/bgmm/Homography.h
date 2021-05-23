#ifndef TFG_VIDEO_SEGMENTATION_HOMOGRAPHY_H
#define TFG_VIDEO_SEGMENTATION_HOMOGRAPHY_H

#include <opencv2/core.hpp>

#include "MotionModel.h"
#include "TrackTable.h"


namespace tfg {

std::vector<float> getWeights2(std::vector<float> &residuals, float tau2);
std::vector<float> getWeightsFromInliers(std::vector<std::vector<int>> &inliers, std::shared_ptr<tfg::TrackTable> &trackTable);

void computeHomographyRANSAC(const std::vector<cv::Vec2f> &p0, const std::vector<cv::Vec2f> &p1, int n, int niter, float tolerance, cv::Matx33f &H, std::vector<int> &inliers);
void computeHomographyWLS(const std::vector<cv::Vec2f> &p0, const std::vector<cv::Vec2f> &p1, int n, const std::vector<unsigned int> &trajectories, const std::vector<float> &weights2, cv::Matx33f &H);
void isotropicNormalization(const std::vector<cv::Vec2f> &points, std::vector<cv::Vec2f> &normalizedPoints, cv::Vec2f &center, cv::Vec2f &scale);
void IRLS(std::shared_ptr<tfg::MotionModel> &model, std::shared_ptr<tfg::TrackTable> &trackTable, std::vector<float> &weights2, float tau2);

} // namespace tfg

#endif // TFG_VIDEO_SEGMENTATION_HOMOGRAPHY_H
