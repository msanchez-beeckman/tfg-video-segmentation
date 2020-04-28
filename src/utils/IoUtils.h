
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
    void groupLabelWeights(const std::vector<std::vector<float>> &weights, std::vector<float> &groupedWeights);
}

#endif //TFG_VIDEO_SEGMENTATION_IOUTILS_H