
#include <iostream>
#include <algorithm>
#include <opencv4/opencv2/imgcodecs.hpp>
#include <opencv4/opencv2/imgproc.hpp>
#include <eigen3/Eigen/Sparse>
#include <eigen3/Eigen/Dense>
#include "ConsensusVoter.h"

namespace tfg {

    ConsensusVoter::ConsensusVoter() {}
    ConsensusVoter::ConsensusVoter(int estimateSpPerFrame, int numberOfFrames) {
        const int estimatedTotalSuperpixels = estimateSpPerFrame * numberOfFrames;
        this->votes.reserve(estimatedTotalSuperpixels);
        this->saliencyScores.reserve(numberOfFrames);
    }
    ConsensusVoter::~ConsensusVoter() {}

    bool ConsensusVoter::initializeMotionSaliencyScores(std::istream &flowFile, float minimumPercentageValidity) {
        saliencyScores.clear();
        std::string line;

        std::getline(flowFile, line);
        const unsigned int NUMBER_OF_FRAMES = std::stoi(line);
        this->saliencyScores.reserve(NUMBER_OF_FRAMES);
        int framesWithDominantMotion = 0;

        for(unsigned int f = 0; f < NUMBER_OF_FRAMES; f++) {
            std::getline(flowFile, line);
            const unsigned int NUMBER_OF_FLOWS = std::stoi(line);

            std::vector<cv::Mat> motionSaliencies;
            for(unsigned int i = 0; i < NUMBER_OF_FLOWS; i++) {
                std::getline(flowFile, line);
                cv::Mat flowu = cv::imread(line, cv::IMREAD_ANYDEPTH);

                std::getline(flowFile, line);
                cv::Mat flowv = cv::imread(line, cv::IMREAD_ANYDEPTH);

                cv::Mat saliency;
                bool existsDominantMotion = computeMotionSaliency(flowu, flowv, saliency, 5, 0.5f, 0.75f, 10);
                if(existsDominantMotion) {
                    motionSaliencies.push_back(saliency);
                }
            }

            cv::Mat frameScore = cv::Mat::zeros(1, 1, CV_32FC1);
            if(motionSaliencies.size() > 0) {
                elementwiseMean(motionSaliencies, frameScore);
                framesWithDominantMotion += 1;
            }

            this->saliencyScores.push_back(frameScore);
        }

        // Not sure about this.
        // The saliency score must be a number between 0 and 1, so dividing by the maximum value seems to be
        // the most straightforward way to normalize the sequence of images.
        normalizeByMaxOfSequence(this->saliencyScores);
        
        return (framesWithDominantMotion > NUMBER_OF_FRAMES * minimumPercentageValidity);
    }

    bool ConsensusVoter::computeMotionSaliency(const cv::Mat &flowu, const cv::Mat &flowv, cv::Mat &saliency, int patchSize, float staticTh, float transTh, int orientationBins) {

        // Compute the magnitude and angle of the flow
        cv::Mat magnitude;
        cv::magnitude(flowu, flowv, magnitude);
        std::vector<float> magnitudeData;
        magnitude.reshape(0, 1).row(0).copyTo(magnitudeData);

        // Compute the median of the magnitude of the flow
        std::nth_element(magnitudeData.begin(), magnitudeData.begin() + magnitudeData.size()/2, magnitudeData.end());
        float median;
        if(magnitudeData.size()%2 == 1) {
            median = magnitudeData[magnitudeData.size()/2];
        } else {
            auto it = std::max_element(magnitudeData.begin(), magnitudeData.begin() + magnitudeData.size()/2);
            median = (*it + magnitudeData[magnitudeData.size()/2])/2.0f;
        }

        // If the background is static, the saliency score is 
        if(median < staticTh) {
            std::cout << "Static" << std::endl;
            cv::Mat magnitudeDeviation = magnitude.mul(magnitude);
            cv::Mat kernel = cv::Mat::ones(patchSize, patchSize, CV_32FC1) / (patchSize * patchSize);
            cv::filter2D(magnitudeDeviation, saliency, -1, kernel);
            return true;
        }

        // Reaching here means that the background is not static, so check for a dominant translational motion
        // To do so, compute an histogram of the orientations, and check if one bin has more than transTh*100% of the entries
        cv::Mat orientation;
        cv::phase(flowu, flowv, orientation);
        int onlyChannel[] = {0};
        int histSize[] = {orientationBins};
        float orientationRange[] = {0, 2*CV_PI};
        const float* ranges[] = {orientationRange};
        cv::Mat orientationHistogram;

        cv::calcHist(&orientation, 1, onlyChannel, cv::Mat(), orientationHistogram, 1, histSize, ranges);
        cv::Scalar sumHist = cv::sum(orientationHistogram);
        orientationHistogram /= sumHist(0) + (sumHist(0) == 0);

        double minVal, maxVal;
        int minPos[2], maxPos[2];
        cv::minMaxIdx(orientationHistogram, &minVal, &maxVal, &minPos[0], &maxPos[0]);

        // If the percentage of the largest bin is greater than a threshhold, we consider that the frame has dominant translational motion
        if(maxVal > transTh) {
            std::cout << "Dominant translational motion" << std::endl;
            // Compute the deviation of the angles with respect to the class mark of the bin with the hightest value
            // Since an angle does not change by adding or substracting 2pi to it, the desired deviation is the elementwise minimum of
            // the deviations adding -2pi, 0, and 2pi.
            float classMark = (2*maxPos[0] + 1)*2*CV_PI / (2 * orientationBins);
            cv::Mat angleDeviation = (orientation - classMark).mul(orientation - classMark);
            cv::min(angleDeviation, (orientation - classMark + 2 * CV_PI).mul(orientation - classMark + 2 * CV_PI), angleDeviation);
            cv::min(angleDeviation, (orientation - classMark - 2 * CV_PI).mul(orientation - classMark - 2 * CV_PI), angleDeviation);

            cv::Mat kernel = cv::Mat::ones(patchSize, patchSize, CV_32FC1) / (patchSize * patchSize);
            cv::filter2D(angleDeviation, saliency, -1, kernel);
            return true;
        }

        return false;
    }

    void ConsensusVoter::elementwiseMean(const std::vector<cv::Mat> &src, cv::Mat &dst) {
        dst.release();

        const unsigned int VECTOR_SIZE = src.size();
        dst = cv::Mat::zeros(src[0].size(), CV_32FC1);
        for(unsigned int i = 0; i < VECTOR_SIZE; i++) {
            dst += src[i];
        }
        dst = dst / VECTOR_SIZE;
    }

    void ConsensusVoter::normalizeByMaxOfSequence(std::vector<cv::Mat> &sequence) {
        float max = 0;
        for(unsigned int i = 0; i < sequence.size(); i++) {
            double minVal, maxVal;
            cv::minMaxIdx(sequence[i], &minVal, &maxVal);
            max = maxVal > max ? maxVal : max;
        }

        if(max == 0) return;

        for(unsigned int i = 0; i < sequence.size(); i++) {
            sequence[i] = sequence[i] / max;
        }
    }

    void ConsensusVoter::initializeVotesFromSaliencyInFrame(int frame, const cv::Mat &pixelLabels, int numberOfSuperpixels) {
        cv::Mat saliencyOfFrame;
        cv::resize(this->saliencyScores[frame], saliencyOfFrame, pixelLabels.size());

        for(unsigned int i = 0; i < numberOfSuperpixels; i++) {
            cv::Mat superpixelMask(pixelLabels == i);
            cv::Scalar regionMeanVote = cv::mean(saliencyOfFrame, superpixelMask);
            float vote = regionMeanVote(0);
            this->votes.push_back(vote);
        }
    }

    void ConsensusVoter::reachConsensus(const Eigen::SparseMatrix<float> &transitionMatrix, const std::vector<int> &frameBeginningIndices, int iterations, std::vector<float> &finalVotes) {
        Eigen::Map<Eigen::VectorXf> updatedVotes(votes.data(), votes.size());

        for(int t = 0; t < iterations; t++) {
            updatedVotes = transitionMatrix * updatedVotes;

            for(unsigned int f = 0; f < frameBeginningIndices.size(); f++) {
                int superpixelsInFrame = (f == (frameBeginningIndices.size() - 1)) ? (votes.size() - frameBeginningIndices[f]) : (frameBeginningIndices[f + 1] - frameBeginningIndices[f]);
                float maxVoteInFrame = updatedVotes.segment(frameBeginningIndices[f], superpixelsInFrame).maxCoeff();
                updatedVotes.segment(frameBeginningIndices[f], superpixelsInFrame) /= maxVoteInFrame + (maxVoteInFrame <= 0);
            }
        }

        finalVotes = votes;
    }
}