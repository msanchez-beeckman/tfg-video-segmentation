
/*
 * Copyright (c) 2020-2021, Marco Sánchez Beeckman
 * All rights reserved.
 *
 * This source code is licensed under the BSD-style license found in the
 * LICENSE file in the root directory of this source tree.
 */

#ifndef TFG_VIDEO_SEGMENTATION_RANDOMWALKER_H
#define TFG_VIDEO_SEGMENTATION_RANDOMWALKER_H

#include <fstream>
#include <unordered_map>

#include <opencv2/core.hpp>

#include "TrackTable.h"


namespace tfg {
class RandomWalker {
private:
    std::shared_ptr<tfg::TrackTable> trackTable;
    // std::unordered_map<int, std::vector<float>> probabilities;
    std::vector<std::vector<float>> probabilities;
    unsigned int numberOfLabels;
    unsigned int unlabeledTracks;
    unsigned int labeledTracks;
    
public:
    RandomWalker(std::shared_ptr<tfg::TrackTable> &trackTable);
    ~RandomWalker();

    void seed(const std::unordered_map<int, cv::Mat> &seedImages);
    void seedDavis(const std::unordered_map<int, cv::Mat> &seedImages);
    void propagateSeeds(float lambda);
    void writeProbabilities(std::ofstream &file);
};

} // namespace tfg

#endif // TFG_VIDEO_SEGMENTATION_RANDOMWALKER_H