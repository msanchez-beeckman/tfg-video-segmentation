/*
 * Copyright (c) 2020-2021, Marco Sánchez Beeckman
 * All rights reserved.
 *
 * This source code is licensed under the BSD-style license found in the
 * LICENSE file in the root directory of this source tree.
 */

#ifndef TFG_VIDEO_SEGMENTATION_IOUTILS_H
#define TFG_VIDEO_SEGMENTATION_IOUTILS_H

#include <cstring>
#include <fstream>
#include <vector>

namespace tfg {

void splitString(const std::string &line, std::vector<std::string> &words, char delimiter=' ');
void readWeights(std::ifstream &weightFile, std::vector<float> &weights);
void writeWeights(const std::vector<float> &weights, std::ofstream &weightFile);
void readWeightsMultilabel(std::ifstream &weightFile, std::vector<std::vector<float>> &weights);
void groupLabelWeights(const std::vector<std::vector<float>> &weights, std::vector<float> &groupedWeights, int foregroundLabel);

} // namespace tfg

#endif // TFG_VIDEO_SEGMENTATION_IOUTILS_H