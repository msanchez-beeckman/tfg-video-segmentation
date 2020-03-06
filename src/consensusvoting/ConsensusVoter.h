
#ifndef TFG_VIDEO_SEGMENTATION_CONSENSUSVOTER_H
#define TFG_VIDEO_SEGMENTATION_CONSENSUSVOTER_H

#include <vector>
#include <opencv4/opencv2/core.hpp>
#include <eigen3/Eigen/Sparse>

namespace tfg {

    class ConsensusVoter {
        private:
            std::vector<float> votes;
            std::vector<cv::Mat> saliencyScores;

            bool computeMotionSaliency(const cv::Mat &flowu, const cv::Mat &flowv, cv::Mat &saliency, int patchSize, float staticTh, float transTh, int orientationBins);
            void elementwiseMean(const std::vector<cv::Mat> &src, cv::Mat &dst);
            void normalizeByMaxOfSequence(std::vector<cv::Mat> &sequence);

        public:
            ConsensusVoter();
            ConsensusVoter(int estimateSpPerFrame, int numberOfFrames);
            ~ConsensusVoter();
            bool initializeMotionSaliencyScores(std::istream &flowFile, float minimumPercentageValidity);

            void initializeVotesFromSaliencyInFrame(int frame, const cv::Mat &pixelLabels, int numberOfSuperpixels);

            void reachConsensus(const Eigen::SparseMatrix<float> &transitionMatrix, const std::vector<int> &frameBeginningIndices, int iterations, std::vector<float> &finalVotes);
    };
}

#endif //TFG_VIDEO_SEGMENTATION_CONSENSUSVOTER_H