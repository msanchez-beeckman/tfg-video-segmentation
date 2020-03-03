
#ifndef TFG_VIDEO_SEGMENTATION_CONSENSUSVOTER_H
#define TFG_VIDEO_SEGMENTATION_CONSENSUSVOTER_H

#include <vector>
#include <opencv4/opencv2/core.hpp>

namespace tfg {

    class ConsensusVoter {
        private:
            std::vector<float> votes;
            std::vector<cv::Mat> saliencyScores;
            std::vector<int> motionType;

            void computeSaliency(const cv::Mat &flowu, const cv::Mat &flowv, cv::Mat &saliency);

        public:
            ConsensusVoter();
            ~ConsensusVoter();
            void initializeSaliencyScores(std::istream &flowFile);
    };
}

#endif //TFG_VIDEO_SEGMENTATION_CONSENSUSVOTER_H