
#ifndef TFG_VIDEO_SEGMENTATION_CONSENSUSVOTER_H
#define TFG_VIDEO_SEGMENTATION_CONSENSUSVOTER_H

#include <vector>
#include <fstream>
#include <opencv4/opencv2/core.hpp>
#include <eigen3/Eigen/Sparse>
#include "Region.h"

namespace tfg {

    class ConsensusVoter {
        private:
            std::vector<tfg::Region> superpixels;
            std::vector<float> votes;

            std::vector<cv::Mat> saliencyScores;
            std::vector<int> frameBeginningIndex;

            cv::Mat descriptors;
            Eigen::SparseMatrix<float, Eigen::RowMajor> transitionMatrix;

            bool computeMotionSaliency(const cv::Mat &flowu, const cv::Mat &flowv, cv::Mat &saliency, int patchSize, float staticTh, float transTh, int orientationBins);
            void elementwiseMean(const std::vector<cv::Mat> &src, cv::Mat &dst);
            void normalizeByMaxOfSequence(std::vector<cv::Mat> &sequence);

            void computeDescriptors();

            void correctVotesForSmallBlobs(cv::Mat &matrix, int spBegin, int spEnd, const cv::Mat &regionLabels, float threshold, float relativeSize);

        public:
            ConsensusVoter();
            ConsensusVoter(int estimateSpPerFrame, int numberOfFrames);
            ~ConsensusVoter();

            bool initializeMotionSaliencyScores(std::ifstream &flowFile, float minimumPercentageValidity);
            void saveSaliencies(const std::string &folder, const std::string &fileName);

            void addRegionsByFrame(std::vector<tfg::Region> &spInFrame);
            void initializeVotesInFrame(int frame, const cv::Mat &pixelLabels, int numberOfSuperpixels);
            void computeTransitionMatrix(int F, int L, float sigma2);
            void reachConsensus(int iterations);
            void getSegmentation(std::vector<cv::Mat> &masks, float threshold, bool removeSmallBlobs);

    };
}

#endif //TFG_VIDEO_SEGMENTATION_CONSENSUSVOTER_H