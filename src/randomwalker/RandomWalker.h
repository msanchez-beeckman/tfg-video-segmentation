
#ifndef TFG_VIDEO_SEGMENTATION_RANDOMWALKER_H
#define TFG_VIDEO_SEGMENTATION_RANDOMWALKER_H

#include <unordered_map>
#include <fstream>
#include "TrackTable.h"

namespace tfg {
    class RandomWalker {
        private:
            std::shared_ptr<tfg::TrackTable> trackTable;
            std::unordered_map<int, std::vector<float>> probabilities;
            unsigned int numberOfLabels;
            unsigned int unlabeledTracks;
            unsigned int labeledTracks;
            
        public:
            RandomWalker(std::shared_ptr<tfg::TrackTable> &trackTable);
            ~RandomWalker();

            void seed(const std::unordered_map<int, cv::Mat> &seedImages);
            void propagateSeeds(float lambda);
            void writeProbabilities(std::ofstream &file);
    };
}

#endif //TFG_VIDEO_SEGMENTATION_RANDOMWALKER_H