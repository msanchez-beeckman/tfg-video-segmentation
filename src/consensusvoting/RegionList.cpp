
#include <algorithm>
#include <numeric>
#include <cmath>
#include <opencv4/opencv2/ml/ml.hpp>
#include <eigen3/Eigen/Dense>
#include "RegionList.h"

namespace tfg {

    RegionList::RegionList() {}
    RegionList::RegionList(unsigned int estimateSpPerFrame, unsigned int numberOfFrames) {
        const unsigned int estimatedTotalSuperpixels = estimateSpPerFrame * numberOfFrames;
        this->superpixels.reserve(estimatedTotalSuperpixels);
        this->frameBeginningIndex.reserve(numberOfFrames);
    }

    RegionList::~RegionList() {}

    void RegionList::addNewFrame(std::vector<tfg::Region> &spInFrame) {
        unsigned int nextFrameIndex = frameBeginningIndex.size() == 0 ? 0 : frameBeginningIndex.back() + spInFrame.size();
        frameBeginningIndex.push_back(nextFrameIndex);
        std::move(spInFrame.begin(), spInFrame.end(), std::back_inserter(superpixels));
    }

    void RegionList::computeDescriptors() {
        descriptors.release();

        for(unsigned int r = 0; r < superpixels.size(); r++) {
            cv::Mat singleRegionDescriptor = superpixels[r].getDescriptor();
            cv::vconcat(descriptors, singleRegionDescriptor, descriptors);
        }
    }

    Eigen::SparseMatrix<float> RegionList::transitionMatrix(int F, int L, float sigma2) {
        const unsigned int NUMBER_OF_REGIONS = superpixels.size();
        const int M = L*(2*F + 1);
        Eigen::SparseMatrix<float> transM(NUMBER_OF_REGIONS, NUMBER_OF_REGIONS);
        std::vector<Eigen::Triplet<float>> transMEntries;
        transMEntries.reserve(M * NUMBER_OF_REGIONS);

        for(unsigned int f = 0; f < frameBeginningIndex.size(); f++) {
            // Extract the submatrix corresponding to the descriptors of the regions in the current frame
            cv::Rect currentFrameDelimiter;
            currentFrameDelimiter.y = frameBeginningIndex[f];
            currentFrameDelimiter.height = f == frameBeginningIndex.size() - 1 ? superpixels.size() - frameBeginningIndex[f] : frameBeginningIndex[f + 1] - frameBeginningIndex[f];
            currentFrameDelimiter.x = 0;
            currentFrameDelimiter.width = descriptors.cols;
            cv::Mat descriptorsInFrame(descriptors, currentFrameDelimiter);

            // Extract the submatrix corresponding to the descriptors of all the regions in the neighbouring frames
            cv::Rect searchWindow;
            const int yMinWindow = f - F < 0 ? 0 : f - F;
            const int yMaxWindow = f + F > frameBeginningIndex.size() - 1 ? frameBeginningIndex.size() - 1 : f + F;
            searchWindow.y = frameBeginningIndex[yMinWindow];
            searchWindow.height = yMaxWindow == frameBeginningIndex.size() - 1 ? superpixels.size() - frameBeginningIndex[yMinWindow] : frameBeginningIndex[yMaxWindow + 1] - frameBeginningIndex[yMinWindow];
            searchWindow.x = 0;
            searchWindow.width = descriptors.cols;
            cv::Mat descriptorsInNeighbouringFrames(descriptors, searchWindow);

            // Keep track of the position of every neighbouring region
            // Put them in a Mat object so the KDTree returns the indices of the nearest neighbours
            std::vector<float> neighbourIndices(searchWindow.width);
            std::iota(neighbourIndices.begin(), neighbourIndices.end(), searchWindow.y);
            cv::Mat regionIndices(neighbourIndices.size(), 1, CV_32FC1, neighbourIndices.data());

            // Create a KDTree and train it using the descriptors in the neighbouring frames
            // Each sample belongs to a column, and its response is the index of the region
            cv::Ptr<cv::ml::KNearest> tree = cv::ml::KNearest::create();
            tree->train(descriptorsInNeighbouringFrames, cv::ml::ROW_SAMPLE, regionIndices);

            // Search for the nearest M neighbours inside of the samples.
            // Since there can be more than one good match per frame, we search up to 4 regions per frame
            // for a total of M = L*(2*F + 1) nearest neighbours
            cv::Mat NNIndices;
            cv::Mat result; // unused for our purposes, but required for the function findNearest
            cv::Mat NNDistances2;
            tree->findNearest(descriptorsInFrame, M, result, NNIndices, NNDistances2);

            // Knowing who the nearest neighbours are and the distance to them, define a weight based on
            // the distance, and create a random walk transition matrix (which is sparse)
            for(int y = 0; y < currentFrameDelimiter.height; y++) {
                // int ownIndex = static_cast<int>(NNIndices.at<float>(y, 0));
                const int ownIndex = static_cast<int>(regionIndices.at<float>(0, y));
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

        // Upon computing the transition matrix, normalize it by dividing each row by the sum of its elements
        Eigen::SparseMatrix<float> normalizationMatrix(NUMBER_OF_REGIONS, NUMBER_OF_REGIONS);
        std::vector<Eigen::Triplet<float>> normalizationMatrixEntries;
        normalizationMatrixEntries.reserve(NUMBER_OF_REGIONS);
        for(int k = 0; k < transM.outerSize(); ++k) {
            float degree = 0.0f;
            for(Eigen::SparseMatrix<float>::InnerIterator it(transM, k); it; ++it) {
                degree += it.value();
            }
            normalizationMatrixEntries.push_back(Eigen::Triplet<float>(k, k, 1 / degree));
        }
        normalizationMatrix.setFromTriplets(normalizationMatrixEntries.begin(), normalizationMatrixEntries.end());

        // TODO: measure efficiency difference between computing normalization matrix and multiplying transM by it (as it is now),
        // and looping through transM a second time and directly modifying the value through it.valueRef()

        Eigen::SparseMatrix<float> normalizedTransM = normalizationMatrix * transM;
        return normalizedTransM;

    }

    // void computeNearestNeighbours(int F = 15, int L = 4) {
    //     for(unsigned int f = 0; f < frameBeginningIndex.size(); f++) {
    //         // Extract the submatrix corresponding to the descriptors of the regions in the current frame
    //         cv::Rect currentFrameDelimiter;
    //         currentFrameDelimiter.y = frameBeginningIndex[f];
    //         currentFrameDelimiter.height = f == frameBeginningIndex.size() - 1 ? superpixels.size() - frameBeginningIndex[f] : frameBeginningIndex[f + 1] - frameBeginningIndex[f];
    //         currentFrameDelimiter.x = 0;
    //         currentFrameDelimiter.width = descriptor.cols;
    //         cv::Mat descriptorsInFrame(descriptors, currentFrameDelimiter);

    //         // Extract the submatrix corresponding to the descriptors of all the regions in the neighbouring frames
    //         cv::Rect searchWindow;
    //         const int yMinWindow = f - F < 0 ? 0 : f - F;
    //         const int yMaxWindow = f + F > frameBeginningIndex.size() - 1 ? frameBeginningIndex.size() - 1 : f + F;
    //         searchWindow.y = frameBeginningIndex[yMinWindow];
    //         searchWindow.height = yMaxWindow == frameBeginningIndex.size() - 1 ? superpixels.size() - frameBeginningIndex[yMinWindow] : frameBeginningIndex[yMaxWindow + 1] - frameBeginningIndex[yMinWindow];
    //         searchWindow.x = 0;
    //         searchWindow.width = descriptor.cols;
    //         cv::Mat descriptorsInNeighbouringFrames(descriptors, searchWindow);

    //         // Keep track of the position of every neighbouring region
    //         // Put them in a Mat object so the KDTree returns the indices of the nearest neighbours
    //         std::vector<float> neighbourIndices(searchWindow.width);
    //         std::iota(neighbourIndices.begin(), neighbourIndices.end(), searchWindow.y);
    //         cv::Mat regionIndices(neighBourIndices.size(), 1, CV_32FC1, neighbourIndices.data());

    //         // Create a KDTree and train it using the descriptors in the neighbouring frames
    //         // Each sample belongs to a column, and its response is the index of the region
    //         cv::Ptr<cv::ml::KNearest> tree = cv::ml::KNearest::create();
    //         tree->train(descriptorsInNeighbouringFrames, cv::ml::ROW_SAMPLE, regionIndices);

    //         // Search for the nearest M neighbours inside of the samples.
    //         // Since there can be more than one good match per frame, we search up to 4 regions per frame
    //         // for a total of M = L*(2*F + 1) nearest neighbours
    //         cv::Mat NNIndices;
    //         cv::Mat result; // unused for our purposes, but required for the function findNearest
    //         cv::Mat NNDistances2;
    //         int M = L*(2*F + 1);
    //         tree->findNearest(descriptorsInFrame, M, result, NNIndices, NNDistances2);
    //     }
    // }
}