#include <unordered_map>
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
            void propagateSeeds();
            void writeProbabilities(std::ostream &file);
    };
}