
#include <iostream>
#include <algorithm>
#include <numeric>
#include <opencv4/opencv2/ml/ml.hpp>
#include <opencv4/opencv2/imgcodecs.hpp>
#include <opencv4/opencv2/imgproc.hpp>
#include <eigen3/Eigen/Sparse>
#include <eigen3/Eigen/Dense>
#include "ConsensusVoter.h"

namespace tfg {

    ConsensusVoter::ConsensusVoter() {}
    ConsensusVoter::ConsensusVoter(int estimateSpPerFrame, int numberOfFrames) {
        const int estimatedTotalSuperpixels = estimateSpPerFrame * numberOfFrames;
        this->superpixels.reserve(estimatedTotalSuperpixels);
        this->votes.reserve(estimatedTotalSuperpixels);
        this->saliencyScores.reserve(numberOfFrames);
        this->frameBeginningIndex.reserve(numberOfFrames);
    }
    ConsensusVoter::~ConsensusVoter() {}

    bool ConsensusVoter::initializeMotionSaliencyScores(std::istream &flowFile, float minimumPercentageValidity) {
        this->saliencyScores.clear();
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
                bool existsDominantMotion = computeMotionSaliency(flowu, flowv, saliency, 5, 1.0f, 0.55f, 10);
                if(existsDominantMotion) {
                    std::cout << " in flow " << i << " of frame " << f << std::endl;
                    motionSaliencies.push_back(saliency);
                }
            }

            cv::Mat frameScore = cv::Mat::zeros(1, 1, CV_32FC1);
            if(motionSaliencies.size() > 0) {
                elementwiseMean(motionSaliencies, frameScore);
                framesWithDominantMotion += 1;
            } else {
                std::cout << "No dominant motion in frame " << f << ": saliency initialized to 0" << std::endl;
            }

            this->saliencyScores.push_back(frameScore);
        }


        // The saliency score must be a number between 0 and 1, so dividing by the maximum value is
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
            std::cout << "Static motion";
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
            std::cout << "Translational motion";
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
        dst /= VECTOR_SIZE;
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

    void ConsensusVoter::saveSaliencies(const std::string &folder, const std::string &fileName) {

        for(unsigned int f = 0; f < this->saliencyScores.size(); f++) {
            cv::Mat imageToBeSaved;
            this->saliencyScores[f].convertTo(imageToBeSaved, CV_8UC1, 255);
            std::stringstream ss;
            ss << folder << fileName << f << ".jpg";
            std::string saveAs = ss.str();
            cv::imwrite(saveAs, imageToBeSaved);
        }
    }

    void ConsensusVoter::addRegionsByFrame(std::vector<tfg::Region> &spInFrame) {
        const int nextFrameIndex = this->frameBeginningIndex.size() == 0 ? 0 : this->frameBeginningIndex.back() + spInFrame.size();
        this->frameBeginningIndex.push_back(nextFrameIndex);
        std::move(spInFrame.begin(), spInFrame.end(), std::back_inserter(this->superpixels));
    }

    void ConsensusVoter::initializeVotesInFrame(int frame, const cv::Mat &pixelLabels, int numberOfSuperpixels) {
        cv::Mat saliencyOfFrame;
        cv::resize(this->saliencyScores[frame], saliencyOfFrame, pixelLabels.size());

        for(unsigned int i = 0; i < numberOfSuperpixels; i++) {
            cv::Mat superpixelMask(pixelLabels == i);
            cv::Scalar regionMeanVote = cv::mean(saliencyOfFrame, superpixelMask);
            const float vote = regionMeanVote(0);
            this->votes.push_back(vote);
        }
    }

    void ConsensusVoter::computeTransitionMatrix(int F, int L, float sigma2) {
        computeDescriptors();

        const unsigned int NUMBER_OF_REGIONS = this->superpixels.size();
        const unsigned int NUMBER_OF_FRAMES = this->frameBeginningIndex.size();
        const int M = L*(2*F + 1);
        Eigen::SparseMatrix<float, Eigen::RowMajor> transM(NUMBER_OF_REGIONS, NUMBER_OF_REGIONS);
        std::vector<Eigen::Triplet<float>> transMEntries;
        transMEntries.reserve(M * NUMBER_OF_REGIONS);
        transMEntries.reserve(L*(2*F + 1) * NUMBER_OF_REGIONS);

        for(int f = 0; f < NUMBER_OF_FRAMES; f++) {
            // Extract the submatrix corresponding to the descriptors of the regions in the current frame
            cv::Rect currentFrameDelimiter;
            currentFrameDelimiter.y = this->frameBeginningIndex[f];
            currentFrameDelimiter.height = (f == (NUMBER_OF_FRAMES - 1)) ? (NUMBER_OF_REGIONS - this->frameBeginningIndex[f]) : (this->frameBeginningIndex[f + 1] - this->frameBeginningIndex[f]);
            currentFrameDelimiter.x = 0;
            currentFrameDelimiter.width = this->descriptors.cols;
            cv::Mat descriptorsInFrame(this->descriptors, currentFrameDelimiter);

            std::cout << "Extracted descriptors of superpixels in frame " << f << std::endl;

            // Extract the submatrix corresponding to the descriptors of all the regions in the neighbouring frames
            // TODO: right now the window is smaller for the frames at the beginning and at the end, but we still ask
            // for the M nearest neighbours: for this reason, the transition matrix is not symmetrical. As a way to
            // correct this and improve the efficiency of the function, it's interesting to try to limit the window
            // to only search forward (F + 1 frames, making yMinWindow = f), and limit the number of nearest neighbours to L*(searchWindow.height)

            cv::Rect searchWindow;
            const int yMinWindow = ((f - F) < 0) ? 0 : (f - F);
            const int yMaxWindow = ((f + F) > (NUMBER_OF_FRAMES - 1)) ? (NUMBER_OF_FRAMES - 1) : (f + F);
            searchWindow.y = this->frameBeginningIndex[yMinWindow];
            searchWindow.height = (yMaxWindow == (NUMBER_OF_FRAMES - 1)) ? (NUMBER_OF_REGIONS - this->frameBeginningIndex[yMinWindow]) : (this->frameBeginningIndex[yMaxWindow + 1] - this->frameBeginningIndex[yMinWindow]);
            searchWindow.x = 0;
            searchWindow.width = this->descriptors.cols;
            cv::Mat descriptorsInNeighbouringFrames(this->descriptors, searchWindow);

            std::cout << "Extracted descriptors of neighbours of frame " << f << std::endl;

            // Keep track of the position of every neighbouring region
            // Put them in a Mat object so the KDTree returns the indices of the nearest neighbours
            std::vector<float> neighbourIndices(searchWindow.height);
            std::iota(neighbourIndices.begin(), neighbourIndices.end(), searchWindow.y);
            std::cout << "Frame " << f << " iota: from " << neighbourIndices[0] << " to " << neighbourIndices.back() << std::endl;
            cv::Mat regionIndices(neighbourIndices.size(), 1, CV_32FC1, neighbourIndices.data());

            std::cout << "Created matrix with neighbour indices" << std::endl;

            // Create a KDTree and train it using the descriptors in the neighbouring frames
            // Each sample belongs to a column, and its response is the index of the region
            cv::Ptr<cv::ml::KNearest> tree = cv::ml::KNearest::create();
            tree->train(descriptorsInNeighbouringFrames, cv::ml::ROW_SAMPLE, regionIndices);

            std::cout << "Created and trained KDTree " << f << std::endl;

            // Search for the nearest M neighbours inside of the samples.
            // Since there can be more than one good match per frame, we search up to 4 regions per frame
            // for a total of M = L*(2*F + 1) nearest neighbours
            // const int M = L*(yMaxWindow - yMinWindow + 1);
            cv::Mat NNIndices;
            cv::Mat result; // unused for our purposes, but required for the function findNearest
            cv::Mat NNDistances2;
            tree->findNearest(descriptorsInFrame, M, result, NNIndices, NNDistances2);

            std::cout << "Found nearest neighbours of superpixels in frame " << f << '\n' << std::endl;

            // Knowing who the nearest neighbours are and the distance to them, define a weight based on
            // the distance, and create a random walk transition matrix (which is sparse)
            for(int y = 0; y < currentFrameDelimiter.height; y++) {
                const int ownIndex = currentFrameDelimiter.y + y;
                for(int x = 0; x < M; x++) {
                    const int neighbourIndex = static_cast<int>(NNIndices.at<float>(y, x));
                    float weight = 1.0f;
                    if(ownIndex != neighbourIndex) {
                        const float neighbourDistance2 = NNDistances2.at<float>(y, x);
                        weight = exp(-neighbourDistance2 / sigma2);
                    }
                    transMEntries.push_back(Eigen::Triplet<float>(ownIndex, neighbourIndex, weight));
                }
            }
        }
        transM.setFromTriplets(transMEntries.begin(), transMEntries.end());
        // std::cout << transM.bottomRows(1) << std::endl;

        std::cout << "Created sparse transition matrix" << std::endl;

        // Upon computing the transition matrix, normalize it by dividing each row by the sum of its elements
        Eigen::SparseMatrix<float, Eigen::RowMajor> normalizationMatrix(NUMBER_OF_REGIONS, NUMBER_OF_REGIONS);
        std::vector<Eigen::Triplet<float>> normalizationMatrixEntries;
        normalizationMatrixEntries.reserve(NUMBER_OF_REGIONS);
        for(int k = 0; k < transM.outerSize(); ++k) {
            float degree = 0.0f;
            for(Eigen::SparseMatrix<float, Eigen::RowMajor>::InnerIterator it(transM, k); it; ++it) {
                degree += it.value();
            }
            normalizationMatrixEntries.push_back(Eigen::Triplet<float>(k, k, 1 / degree));
        }
        normalizationMatrix.setFromTriplets(normalizationMatrixEntries.begin(), normalizationMatrixEntries.end());

        std::cout << "Created normalization matrix" << std::endl;

        // TODO: measure efficiency difference between computing normalization matrix and multiplying transM by it (as it is now),
        // and looping through transM a second time and directly modifying the value through it.valueRef()
        this->transitionMatrix = normalizationMatrix * transM;

    }

    void ConsensusVoter::computeDescriptors() {
        this->descriptors.release();

        std::vector<cv::Mat> descriptorsVector;
        descriptorsVector.reserve(this->superpixels.size());
        for(unsigned int r = 0; r < this->superpixels.size(); r++) {
            cv::Mat singleRegionDescriptor = this->superpixels[r].getDescriptor();
            descriptorsVector.push_back(singleRegionDescriptor);
        }
        cv::vconcat(descriptorsVector, this->descriptors);
    }

    void ConsensusVoter::reachConsensus(int iterations) {
        Eigen::Map<Eigen::VectorXf> updatedVotes(this->votes.data(), this->votes.size());

        // std::cout << transitionMatrix.bottomRows(1) << std::endl;

        for(int t = 0; t < iterations; t++) {
            updatedVotes = transitionMatrix * updatedVotes;

            for(unsigned int f = 0; f < this->frameBeginningIndex.size(); f++) {
                const int superpixelsInFrame = (f == (frameBeginningIndex.size() - 1)) ? (votes.size() - frameBeginningIndex[f]) : (frameBeginningIndex[f + 1] - frameBeginningIndex[f]);
                const float maxVoteInFrame = updatedVotes.segment(frameBeginningIndex[f], superpixelsInFrame).maxCoeff();
                updatedVotes.segment(frameBeginningIndex[f], superpixelsInFrame) /= maxVoteInFrame + (maxVoteInFrame <= 0);
            }
        }
    }

    void ConsensusVoter::getSegmentation(std::vector<cv::Mat> &masks, float threshold) {
        const unsigned int NUMBER_OF_FRAMES = this->frameBeginningIndex.size();
        const unsigned int NUMBER_OF_REGIONS = this->superpixels.size();
        masks.clear();
        masks.reserve(NUMBER_OF_FRAMES);
        for(unsigned int f = 0; f < NUMBER_OF_FRAMES; f++) {
            const int spBegin = this->frameBeginningIndex[f];
            const int spEnd = (f == (NUMBER_OF_FRAMES - 1)) ? (NUMBER_OF_REGIONS - 1) : (this->frameBeginningIndex[f + 1] - 1);
            cv::Mat framePixelLabels = this->superpixels[spBegin].getFrameLabels();

            cv::Mat maskNotThresholded(framePixelLabels.size(), CV_32FC1);
            for(int sp = spBegin; sp <= spEnd; sp++) {
                const int spLabelInFrame = sp - spBegin;
                cv::Mat spLocation(framePixelLabels == spLabelInFrame);
                maskNotThresholded.setTo(this->votes[sp], spLocation);
            }
            correctVotesForSmallBlobs(maskNotThresholded, spBegin, spEnd, framePixelLabels, threshold, 0.25);
            cv::Mat mask(maskNotThresholded > threshold);
            masks.push_back(mask);
        }
    }

    void ConsensusVoter::correctVotesForSmallBlobs(cv::Mat &matrix, int spBegin, int spEnd, const cv::Mat &regionLabels, float threshold, float relativeSize) {
        cv::Mat mask(matrix > threshold);
        cv::Mat labels;
        cv::Mat stats;
        cv::Mat centroids;
        const int numberOfCC = cv::connectedComponentsWithStats(mask, labels, stats, centroids, 8, CV_16U, cv::CCL_DEFAULT);

        double minArea, maxArea;
        cv::minMaxIdx(stats.col(cv::CC_STAT_AREA).rowRange(1, stats.rows), &minArea, &maxArea);

        // Correct values in non-thresholded mask
        for(int i = 1; i < numberOfCC; i++) {
            int area = stats.at<int>(i, cv::CC_STAT_AREA);
            if(area > maxArea * relativeSize) continue;

            cv::Mat labelMask(labels == i);
            matrix.setTo(threshold - 0.00001, labelMask);
        }

        // Correct values in votes vector
        for(int s = spBegin; s <= spEnd; s++) {
            const int superpixelLabel = s - spBegin;
            cv::Mat region(regionLabels == superpixelLabel);
            cv::Scalar regionMean = cv::mean(matrix, region);
            const float correctedVote = regionMean(0);
            this->votes[s] = correctedVote;
        }
    }
    
}