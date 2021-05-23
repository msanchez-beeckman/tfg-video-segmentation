#ifndef TFG_VIDEO_SEGMENTATION_GRID_H
#define TFG_VIDEO_SEGMENTATION_GRID_H

#include <array>

#include <opencv2/core.hpp>

#include "TrackTable.h"


namespace tfg {

class Grid {
private:
    using Index = std::array<int, 6>;
    using Value = cv::Vec4f;

    static constexpr int DIMENSIONS = 6;
    std::array<float, 6> scales;

    std::vector<cv::Mat> images;
    std::vector<cv::Mat> masks;
    cv::SparseMat data;

    void splatValue(const Index &index, const Value &value);
    void sliceIndex(const Index &index, Value &value);

    void getNeighbours(const Index &index, std::vector<Index> &neighbours, std::vector<float> &weights) const;
    void scaleIndex(const Index &index, std::array<float, 6> &scaledIndex) const;
    float getNeighbourWeight(const std::array<float, 6> &scaledIndex, const Index &neighbourIndex) const;

public:
    Grid();
    Grid(const std::array<float, 6> &scales, const std::vector<cv::Mat> &images, const std::vector<cv::Mat> &masks);
    ~Grid();

    void splatMass();
    void splatTrackWeights(const tfg::TrackTable &trackTable, const std::vector<float> &weights, int texturelessRadius, float bgBias);
    void graphCut(float lambda_u, float lambda_s, float minEdgeCost, const std::array<float, 6> &W);
    void slice(std::vector<cv::Mat> &outMasks, float threshold);

};

} // namespace tfg

#endif // TFG_VIDEO_SEGMENTATION_GRID_H