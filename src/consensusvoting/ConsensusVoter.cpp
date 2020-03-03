
#include <algorithm>
#include "ConsensusVoter.h"

namespace tfg {

    ConsensusVoter::ConsensusVoter() {}
    ConsensusVoter::~ConsensusVoter() {}

    void ConsensusVoter::initializeSaliencyScores(std::istream &flowFile) {
        saliencyScores.clear();
        std::string line;

        std::getline(file, line);
        const unsigned int NUMBER_OF_FRAMES = std::stoi(line);
        this->saliencyScores.reserve(NUMBER_OF_FRAMES);
        int framesWithDominantMotion = 0;

        for(unsigned int f = 0; f < NUMBER_OF_FRAMES; f++) {
            std::getline(file, line);
            const unsigned int NUMBER_OF_FLOWS = std::stoi(line);

            cv::Mat frameScore;
            for(unsigned int i = 0; i < NUMBER_OF_FLOWS; i++) {
                std::getline(file, line);
                cv::Mat flowu = cv::imread(line, cv::IMREAD_ANYDEPTH);

                std::getline(file, line);
                cv::Mat flowv = cv::imread(line, cv::IMREAD_ANYDEPTH);

                cv::Mat saliency;
                computeSaliency(flowu, flowv, saliency);
            }
        }
    }

    void ConsensusVoter::computeSaliency(const cv::Mat &flowu, const cv::Mat &flowv, cv::Mat &saliency) {
        // Compute the square of the modulus of the flow
        cv::Mat flow2 = flowu.mul(flowu) + flowv.mul(flowv);
        flow2 = flow2.reshape(0, 1);
        std::vector<float> flowData2;
        flow2.row(0).copyTo(flowData2);

        // Compute the median of the square of the modulus of the flow
        std::nth_element(flowData2.begin(), flowData2.begin() + flowData2.size()/2, flowData2.end());
        float median;
        if(flowData2.size()%2 == 1) {
            median = flowData2[flowData2.size()/2];
        } else {
            auto it = std::max_element(flowData2.begin(), flowData2.begin() + flowData2.size()/2);
            median = (*it + flowData2[flowData2.size()/2])/2.0f;
        }

        // TODO: continue
    }
}