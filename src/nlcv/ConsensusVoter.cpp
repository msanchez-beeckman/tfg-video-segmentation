/*
 * Copyright (c) 2020-2021, Marco Sánchez Beeckman
 * All rights reserved.
 *
 * This source code is licensed under the BSD-style license found in the
 * LICENSE file in the root directory of this source tree.
 */

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <numeric>

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/ml/ml.hpp>

#include "ImageUtils.h"
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

/**
 * Initialize motion saliency scores for each image, using precomputed optical flows.
 * The scores are stored in OpenCV's Mat objects, and are the same size as the read flows.
 * As such, the saliency scores need not be the same size as the original images, and are resized
 * to the correct values when initializing the votes. This is done this way in case the flows are computed
 * on scaled down images to diminish the execution time.
 * @param flowFile A .txt file containing the absolute path to each image's optical flow .tiff files.
 * @param minimumPercentageValidity Minimum percentage of frames with dominant motion that should exist for the computation to be considered successful.
 * @return True if the majority (at least minimumPercentageValidity) of frames have a dominant motion, false otherwise.
 */
bool ConsensusVoter::initializeMotionSaliencyScores(std::ifstream &flowFile, float minimumPercentageValidity) {
    this->saliencyScores.clear();
    std::string line;

    std::getline(flowFile, line);
    const int NUMBER_OF_FRAMES = std::stoi(line); // The first line of the file tells the number of frames
    this->saliencyScores.reserve(NUMBER_OF_FRAMES);
    int framesWithDominantMotion = 0;

    // Each frame has a certain number of optical flows attributed to it, typically both forward and backward
    for(int f = 0; f < NUMBER_OF_FRAMES; f++) {
        std::getline(flowFile, line);
        const int NUMBER_OF_FLOWS = std::stoi(line); // The first line of a particular frame tells the number of flows

        std::vector<cv::Mat> motionSaliencies;
        // For each flow which the frame is the origin of, compute a motion saliency score
        for(int i = 0; i < NUMBER_OF_FLOWS; i++) {
            std::getline(flowFile, line);
            cv::Mat flowu = cv::imread(line, cv::IMREAD_ANYDEPTH);

            std::getline(flowFile, line);
            cv::Mat flowv = cv::imread(line, cv::IMREAD_ANYDEPTH);

            cv::Mat saliency;
            // Call an auxiliary method to compute motion saliency scores from an optical flow
            bool existsDominantMotion = computeMotionSaliency(flowu, flowv, saliency, 5, 1.0f, 0.55f, 10);
            // If a dominant motion exists, store temporarily the corresponding saliencies
            if(existsDominantMotion) {
                std::cout << " in flow " << i << " of frame " << f << '\n';
                motionSaliencies.push_back(saliency);
            }
        }

        // Extract the mean of the motion saliencies associated to the frame, and attribute it to it
        cv::Mat frameScore = cv::Mat::zeros(1, 1, CV_32FC1);
        if(motionSaliencies.size() > 0) {
            elementwiseMean(motionSaliencies, frameScore);
            framesWithDominantMotion += 1;
        } else {
            std::cout << "No dominant motion in frame " << f << ": saliency initialized to 0" << '\n';
        }

        this->saliencyScores.push_back(frameScore);
    }


    // The saliency score must be a number between 0 and 1, so dividing by the maximum value of the sequence (not per frame) is
    // the most straightforward way to normalize the sequence of images.
    normalizeByMaxOfSequence(this->saliencyScores);
    
    // If not enough frames have a dominant motion, consider the process a failure
    // In that case, visual saliency is the way to go
    return (framesWithDominantMotion > NUMBER_OF_FRAMES * minimumPercentageValidity);
}

/**
 * Compute motion saliency from two floating point matrices of optical flows (horizontal and vertical).
 * This method looks for two different kinds of motion: first, it tries to tell if the frame is static by comparing the median of the magnitude of the 
 * flow and comparing to a low value; then, if the frame is not static, it searches for a dominant translational motion by computing a histogram of the phases
 * of the flow, and looking at the height of the largest bin.
 * The saliency is defined as the square of the deviation of the pixel value (magnitude of the flow in the static case, angle in the translational case) from the
 * dominant value (0 in the static case, the class mark of the largest bin in the translational case).
 * @param flowu A floating point matrix containing the horizontal optical flow.
 * @param flowv A floating point matrix containing the vertical optical flow.
 * @param saliency Output matrix containing the saliency scores for each pixel.
 * @param patchSize The patch size to smooth the saliencies.
 * @param staticTh The threshold value to compare with the median of the flow magnitude to tell if a frame is static or not.
 * @param transTh The threshold percentage to compare with the height of the largest bin to tell if a frame has a dominant translational motion.
 * @param orientationBins The number of bins for the histogram of flow angles.
 * @return True if the flow has either static or translational dominant motion, false otherwise.
 */
bool ConsensusVoter::computeMotionSaliency(const cv::Mat &flowu, const cv::Mat &flowv, cv::Mat &saliency, int patchSize, float staticTh, float transTh, int orientationBins) {

    // Compute the magnitude of the flow
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

    // If the median is low enough, the background is static
    // In that case, the saliency score is the square of each pixel's magnitude
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

/**
 * Compute the elementwise mean of a vector of matrices.
 * @param src A collection of OpenCV matrices.
 * @param dst An output matrix where each element contains the mean of the values in that position.
 */
void ConsensusVoter::elementwiseMean(const std::vector<cv::Mat> &src, cv::Mat &dst) {
    dst.release();

    const unsigned int VECTOR_SIZE = src.size();
    dst = cv::Mat::zeros(src[0].size(), CV_32FC1);
    for(unsigned int i = 0; i < VECTOR_SIZE; i++) {
        dst += src[i];
    }
    dst /= VECTOR_SIZE;
}

/**
 * Normalize a sequence of matrices so all the elements take values in the range [0, 1]
 * by dividing all the entries by the maximum of the whole sequence.
 * @param sequence A sequence of matrices.
 */
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

/**
 * Save saliency scores to a sequence of grayscale images.
 * @param folder The folder where the new files will be stored.
 * @param fileName A prefix for the name of the new files.
 */
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

void ConsensusVoter::mergeSaliencyWithImages(const std::vector<cv::Mat> &images, std::vector<cv::Mat> &mergedImages) {
    mergedImages.clear();
    mergedImages.resize(images.size());
    for(unsigned int f = 0; f < images.size(); f++) {
        cv::Mat imageLab;
        cv::cvtColor(images[f], imageLab, cv::COLOR_BGR2Lab);
        std::vector<cv::Mat> imageChannels;
        cv::split(imageLab, imageChannels);

        cv::Mat saliencyOfFrame;
        cv::resize(this->saliencyScores[f], saliencyOfFrame, images[f].size());
        cv::Mat fourthChannel;
        saliencyOfFrame.convertTo(fourthChannel, CV_8UC1, 255.0, 0.0);
        imageChannels.push_back(fourthChannel);

        cv::merge(imageChannels, mergedImages[f]);
    }
}

/**
 * Add a group of regions belonging to a single frame to the list of superpixels.
 * @param spInFrame A vector of Region objects containing information of superpixels in a frame.
 */
void ConsensusVoter::addRegionsByFrame(std::vector<tfg::Region> &spInFrame) {
    const int nextFrameIndex = this->frameBeginningIndex.size() == 0 ? 0 : this->frameBeginningIndex.back() + spInFrame.size();
    this->frameBeginningIndex.push_back(nextFrameIndex);
    std::move(spInFrame.begin(), spInFrame.end(), std::back_inserter(this->superpixels));
}

/**
 * Initialize votes from previously computed saliency scores, by attributing to each region the mean of the scores
 * of the pixels contained inside it.
 * This is done per-frame, and the saliency matrix of the frame is resized to be the same as the image in case the flows
 * were computed on a shrunken image. If the sizes do not match, a bilineal interpolation is performed on the saliencies.
 */
void ConsensusVoter::initializeVotesInFrame(int frame, const cv::Mat &pixelLabels, int numberOfSuperpixels) {
    cv::Mat saliencyOfFrame;
    cv::resize(this->saliencyScores[frame], saliencyOfFrame, pixelLabels.size());

    for(int i = 0; i < numberOfSuperpixels; i++) {
        cv::Mat superpixelMask(pixelLabels == i);
        cv::Scalar regionMeanVote = cv::mean(saliencyOfFrame, superpixelMask);
        const float vote = regionMeanVote(0);
        this->votes.push_back(vote);
    }
}

/**
 * Compute a random walk transition matrix. This is a sparse matrix that stores costs
 * between neighbouring regions, and is multiplied by a vector of region votes for them to
 * "reach consensus" (update the region votes with a weighted average of their neighbours').
 * The neighbours are computed using a KDTree-search, and a cost is attributed between two neighbours
 * based on the distance between their region descriptors.
 * @param F The number of surrounding frames forwards and backwards to search for nearest neighbours.
 * @param L A multiplier that indicates the number of possible good matches per frame to find nearest neighbours.
 * @param sigma2 The variance of the exponential variable, higher is less restrictive.
 */
void ConsensusVoter::computeTransitionMatrix(int F, int L, float sigma2) {
    computeDescriptors();

    const int NUMBER_OF_REGIONS = static_cast<int>(this->superpixels.size());
    const int NUMBER_OF_FRAMES = static_cast<int>(this->frameBeginningIndex.size());
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

        std::cout << "Extracted descriptors of superpixels in frame " << f << '\n';

        // Extract the submatrix corresponding to the descriptors of all the regions in the neighbouring frames
        cv::Rect searchWindow;
        const int yMinWindow = ((f - F) < 0) ? 0 : (f - F);
        const int yMaxWindow = ((f + F) > (NUMBER_OF_FRAMES - 1)) ? (NUMBER_OF_FRAMES - 1) : (f + F);
        searchWindow.y = this->frameBeginningIndex[yMinWindow];
        searchWindow.height = (yMaxWindow == (NUMBER_OF_FRAMES - 1)) ? (NUMBER_OF_REGIONS - this->frameBeginningIndex[yMinWindow]) : (this->frameBeginningIndex[yMaxWindow + 1] - this->frameBeginningIndex[yMinWindow]);
        searchWindow.x = 0;
        searchWindow.width = this->descriptors.cols;
        cv::Mat descriptorsInNeighbouringFrames(this->descriptors, searchWindow);

        std::cout << "Extracted descriptors of neighbours of frame " << f << '\n';

        // Keep track of the position of every neighbouring region
        // Put them in a Mat object so the KDTree returns the indices of the nearest neighbours
        std::vector<float> neighbourIndices(searchWindow.height);
        std::iota(neighbourIndices.begin(), neighbourIndices.end(), searchWindow.y);
        std::cout << "Frame " << f << " iota: from " << neighbourIndices[0] << " to " << neighbourIndices.back() << '\n';
        cv::Mat regionIndices(neighbourIndices.size(), 1, CV_32FC1, neighbourIndices.data());

        std::cout << "Created matrix with neighbour indices" << '\n';

        // Create a KDTree and train it using the descriptors in the neighbouring frames
        // Each sample belongs to a column, and its response is the index of the region
        cv::Ptr<cv::ml::KNearest> tree = cv::ml::KNearest::create();
        tree->train(descriptorsInNeighbouringFrames, cv::ml::ROW_SAMPLE, regionIndices);

        std::cout << "Created and trained KDTree " << f << '\n';

        // Search for the nearest M neighbours inside of the samples.
        // Since there can be more than one good match per frame, we search up to L regions per frame
        // for a total of M = L*(2*F + 1) nearest neighbours
        // const int M = L*(yMaxWindow - yMinWindow + 1);
        cv::Mat NNIndices;
        cv::Mat result; // unused for our purposes, but required for the function findNearest
        cv::Mat NNDistances2;
        tree->findNearest(descriptorsInFrame, M, result, NNIndices, NNDistances2);

        std::cout << "Found nearest neighbours of superpixels in frame " << f << '\n' << '\n';

        // Knowing who the nearest neighbours are and the distance to them, define a weight based on
        // the distance, and create a random walk transition matrix (which is sparse)
        for(int y = 0; y < currentFrameDelimiter.height; y++) {
            const int ownIndex = currentFrameDelimiter.y + y;
            for(int x = 0; x < M; x++) {
                const int neighbourIndex = static_cast<int>(NNIndices.at<float>(y, x));
                float weight = 1.0f;
                if(ownIndex != neighbourIndex) {
                    const float neighbourDistance2 = NNDistances2.at<float>(y, x);
                    weight = std::exp(-neighbourDistance2 / sigma2);
                }
                transMEntries.push_back(Eigen::Triplet<float>(ownIndex, neighbourIndex, weight));
            }
        }
    }
    transM.setFromTriplets(transMEntries.begin(), transMEntries.end());

    std::cout << "Created sparse transition matrix" << '\n';

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

    std::cout << "Created normalization matrix" << '\n';

    // TODO: measure efficiency difference between computing normalization matrix and multiplying transM by it (as it is now),
    // and looping through transM a second time and directly modifying the value through it.valueRef()
    this->transitionMatrix = normalizationMatrix * transM;

}

/**
 * Get matrix of descriptors. This basically concatenates the already computed descriptors of each region into a single matrix to be used for
 * the nearest neighbour search.
 */
void ConsensusVoter::computeDescriptors() {
    this->descriptors.release();

    std::vector<cv::Mat> descriptorsVector;
    descriptorsVector.reserve(this->superpixels.size());
    for(unsigned int r = 0; r < this->superpixels.size(); r++) {
        // cv::Mat singleRegionDescriptor = this->superpixels[r].getDescriptor();
        cv::Mat singleRegionDescriptor = this->superpixels[r].getDescriptor();
        descriptorsVector.push_back(singleRegionDescriptor);
    }
    cv::vconcat(descriptorsVector, this->descriptors);
}

/**
 * Update the votes by multiplying them by the transition matrix.
 * @param iterations The number of times the votes are to be multiplied by the matrix.
 */
void ConsensusVoter::reachConsensus(int iterations) {
    const int NUMBER_OF_FRAMES = static_cast<int>(this->frameBeginningIndex.size());
    const int NUMBER_OF_VOTES = static_cast<int>(this->votes.size());

    Eigen::Map<Eigen::VectorXf> updatedVotes(this->votes.data(), NUMBER_OF_VOTES);

    for(int t = 0; t < iterations; t++) {
        updatedVotes = transitionMatrix * updatedVotes;

        for(int f = 0; f < NUMBER_OF_FRAMES; f++) {
            const int superpixelsInFrame = (f == (NUMBER_OF_FRAMES - 1)) ? (NUMBER_OF_VOTES - frameBeginningIndex[f]) : (frameBeginningIndex[f + 1] - frameBeginningIndex[f]);
            const float maxVoteInFrame = updatedVotes.segment(frameBeginningIndex[f], superpixelsInFrame).maxCoeff();
            updatedVotes.segment(frameBeginningIndex[f], superpixelsInFrame) /= maxVoteInFrame + (maxVoteInFrame <= 0);
        }
    }
}

/**
 * Threshold the existing votes and get a sequence of masks from them.
 * @param masks The output vector of masks.
 * @param threshold The threshold value.
 * @param smallBlobsThreshold Relative size to the biggest object below which small non-connected blobs should be removed.
 */
void ConsensusVoter::getSegmentation(std::vector<cv::Mat> &masks, float threshold, float smallBlobsThreshold) {
    const int NUMBER_OF_FRAMES = static_cast<int>(this->frameBeginningIndex.size());
    const int NUMBER_OF_REGIONS = static_cast<int>(this->superpixels.size());
    masks.clear();
    masks.reserve(NUMBER_OF_FRAMES);
    for(int f = 0; f < NUMBER_OF_FRAMES; f++) {
        const int spBegin = this->frameBeginningIndex[f];
        const int spEnd = (f == (NUMBER_OF_FRAMES - 1)) ? (NUMBER_OF_REGIONS - 1) : (this->frameBeginningIndex[f + 1] - 1);
        // cv::Mat framePixelLabels = this->superpixels[spBegin].getFrameLabels();
        cv::Mat framePixelLabels = this->superpixels[spBegin].getFrameLabels();

        // Set the values of the mask (before thresholding) by superpixel: each pixel of a superpixel is attributed the same
        // value as the vote for the region
        cv::Mat maskNotThresholded(framePixelLabels.size(), CV_32FC1);
        for(int sp = spBegin; sp <= spEnd; sp++) {
            const int spLabelInFrame = sp - spBegin;
            cv::Mat spLocation(framePixelLabels == spLabelInFrame);
            maskNotThresholded.setTo(this->votes[sp], spLocation);
        }

        // The segmented images tend to have small non-connected blobs that are similar in color to the foreground,
        // but that are really background. This function eliminates the blobs that are much smaller than the biggest object
        // (that is assumed to be the real foreground)
        correctVotesForSmallBlobs(maskNotThresholded, spBegin, spEnd, framePixelLabels, threshold, smallBlobsThreshold);
        cv::Mat mask(maskNotThresholded > threshold);
        masks.push_back(mask);
    }
}

/**
 * Eliminate connected components of a mask whose size with respect to the biggest connected component is much smaller.
 * Then, correct the votes for the involved superpixels by setting them below the threshold.
 * @param matrix A non-thresholded mask whose pixels contain values between 0 and 1.
 * @param spBegin The index of the first superpixel of the frame the matrix corresponds to.
 * @param spEnd The index of the last superpixel of the frame the matrix corresponds to.
 * @param regionLabels A labeling of the image that separates the different superpixels in the frame.
 * @param threshold A threshold to separate background from foreground.
 * @param relativeSize The minimum relative size the blobs should at least have with respect to the biggest object in order no to be removed.
 */
void ConsensusVoter::correctVotesForSmallBlobs(cv::Mat &matrix, int spBegin, int spEnd, const cv::Mat &regionLabels, float threshold, float relativeSize) {
    tfg::removeSmallBlobs(matrix, threshold, relativeSize);

    // Correct values in votes vector
    for(int s = spBegin; s <= spEnd; s++) {
        const int superpixelLabel = s - spBegin;
        cv::Mat region(regionLabels == superpixelLabel);
        cv::Scalar regionMean = cv::mean(matrix, region);
        const float correctedVote = regionMean(0);
        this->votes[s] = correctedVote;
    }
}
    
} // namespace tfg