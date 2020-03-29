#include <iostream>
#include <cmath>
#include <opencv4/opencv2/imgproc.hpp>
#include "maxflow-v3.01/graph.h"
#include "Grid.h"

namespace tfg {

    Grid::Grid() {}
    Grid::~Grid() {}

    Grid::Grid(const std::array<float, 6> &scales, const std::vector<cv::Mat> &images) {
        this->scales = scales;
        this->images = images;

        std::array<int, 6> sizes;
        const int scaledFrames = std::ceil(images.size() * scales[0]);
        sizes[0] = scaledFrames;

        const int scaledX = std::ceil(images[0].cols * scales[1]);
        sizes[1] = scaledX;

        const int scaledY = std::ceil(images[0].rows * scales[2]);
        sizes[2] = scaledY;

        for(int c = 0; c < images[0].channels(); c++) {
            const int scaledC = std::ceil(256 * scales[c + 3]);
            sizes[c + 3] = scaledC;
        }

        data.create(DIMENSIONS, sizes.data(), CV_32FC4);
    }

    void Grid::splatMass() {
        const Value value(0.0f, 0.0f, 1.0f, 0.0f);
        for(unsigned int f = 0; f < images.size(); f++) {
            for(int r = 0; r < images[f].rows; r++) {
                const cv::Vec3b* rowPtr = images[f].ptr<cv::Vec3b>(r);
                for(int c = 0; c < images[f].cols; c++) {
                    const Index index = {static_cast<int>(f), c, r, rowPtr[c](0), rowPtr[c](1), rowPtr[c](2)};
                    splatValue(index, value);
                }
            }
        }
    }

    void Grid::splatTrackWeights(const tfg::TrackTable &trackTable, const std::vector<float> &weights, int texturelessRadius, float bgBias) {
        std::vector<cv::Mat> texturelessIndicators;
        texturelessIndicators.reserve(images.size());
        // Initialize textureless indicator matrices: a value of 0 indicates that the pixel has a track located
        // at a distance of at most texturelessRadius pixels. Otherwise, the region is considered textureless.
        for(unsigned int f = 0; f < images.size(); f++) {
            cv::Mat texture(images[f].rows, images[f].cols, CV_8UC1);
            texture.setTo(255);
            // texture.setTo(0);
            // for(int r = 0; r < images[f]; r += 8) {
            //     const cv::Vec3b* rowPtr = images[f].ptr<cv::Vec3b>(r);
            //     for(int c = 0; c < images[f].cols; c += 8) {
            //         rowPtr[c] = 255;
            //     }
            // }
            texturelessIndicators.push_back(texture);
        }

        // Splat the foreground and background confidence values for each track
        // At the same time, fill a circle of radius texturelessRadius with zeros around
        // the track for the corresponding textureless indicator matrix, so that pixels inside it are not declared textureless
        const unsigned int NUMBER_OF_TRACKS = trackTable.numberOfTracks();
        for(unsigned int t = 0; t < NUMBER_OF_TRACKS; t++) {
            const unsigned int FIRST_FRAME = trackTable.firstFrameOfTrack(t);
            const unsigned int DURATION = trackTable.durationOfTrack(t);
            const std::vector<cv::Vec2f> points = trackTable.pointsInTrack(t);
            const float weight = weights[t];
            for(unsigned int i = 0; i < DURATION; i++) {
                const int COL = std::round(points[FIRST_FRAME + i](0));
                const int ROW = std::round(points[FIRST_FRAME + i](1));
                const cv::Vec3b& color = images[FIRST_FRAME + i].at<cv::Vec3b>(ROW, COL);

                cv::circle(texturelessIndicators[FIRST_FRAME + i], cv::Point2i(COL, ROW), texturelessRadius, cv::Scalar(0), -1);

                Index index = {static_cast<int>(FIRST_FRAME + i), COL, ROW, color(0), color(1), color(2)};
                Value value(0.0f, 0.0f, 0.0f, 0.0f);
                value(weight < 0.5 ? 0 : 1) = 2 * std::abs(weight - 0.5);
                splatValue(index, value);
            }
        }

        // Splat a weak background confidence value to textureless regions
        const Value syntheticBg(0.0f, bgBias, 0.0f, 0.0f);
        for(unsigned int f = 0; f < texturelessIndicators.size(); f++) {
            for(int r = 0; r < texturelessIndicators[f].rows; r++) {
                const unsigned char* rowPtr = texturelessIndicators[f].ptr<unsigned char>(r);
                for(int c = 0; c < texturelessIndicators[f].cols; c++) {
                    if(rowPtr[c] == 0) continue;
                    const cv::Vec3b& color = images[f].at<cv::Vec3b>(r, c);
                    const Index index = {static_cast<int>(f), c, r, color(0), color(1), color(2)};
                    splatValue(index, syntheticBg);
                }
            }
        }
    }

    void Grid::splatValue(const Index &index, const Value &value) {
        std::vector<Index> neighbours;
        std::vector<float> weights;
        getNeighbours(index, neighbours, weights);

        for(unsigned int n = 0; n < neighbours.size(); n++) {
            data.ref<Value>(neighbours[n].data()) += weights[n] * value;
        }
    }

    void Grid::slice(std::vector<cv::Mat> &masks, float threshold) {
        masks.clear();
        masks.reserve(images.size());
        for(unsigned int f = 0; f < images.size(); f++) {
            cv::Mat slicedFrame(images[f].size(), CV_32FC1);
            for(int r = 0; r < images[f].rows; r++) {
                const cv::Vec3b* frameRowPtr = images[f].ptr<cv::Vec3b>(r);
                float* slicedFrameRowPtr = slicedFrame.ptr<float>(r);
                for(int c = 0; c < images[f].cols; c++) {
                    const Index index = {static_cast<int>(f), c, r, frameRowPtr[c](0), frameRowPtr[c](1), frameRowPtr[c](2)};
                    Value slicedValue;
                    sliceIndex(index, slicedValue);
                    slicedFrameRowPtr[c] = slicedValue(3); // The fourth component of a Value is its fg/bg label after the graph cut
                }
            }
            cv::medianBlur(slicedFrame, slicedFrame, 3);
            cv::Mat mask(slicedFrame > threshold);
            masks.push_back(mask);
        }
    }

    void Grid::sliceIndex(const Index &index, Value &value) {
        std::vector<Index> neighbours;
        std::vector<float> weights;
        getNeighbours(index, neighbours, weights);

        value(0) = 0; value(1) = 0; value(2) = 0; value(3) = 0;
        for(unsigned int n = 0; n < neighbours.size(); n++) {
            const Value neighbourValue = data.ref<Value>(neighbours[n].data());
            value += weights[n] * neighbourValue;
        }
    }

    void Grid::getNeighbours(const Index &index, std::vector<Index> &neighbours, std::vector<float> &weights) const {
        std::array<float, 6> scaledIndex;
        scaleIndex(index, scaledIndex);

        neighbours.clear();
        neighbours.reserve(DIMENSIONS + 1);
        weights.clear();
        weights.reserve(DIMENSIONS + 1);

        Index nearestNeighbour;
        Index directionsToNeighbours;
        // First of all, get the index of the cell nearest to the scaled coordinates
        // Also keep the direction of the vector that goes from the cell to the scaled coordinates,
        // since the sign of each of its coordinates indicates where the other neighbours are (one for each dimension)
        float weightSum = 0.0f;
        for(int d = 0; d < DIMENSIONS; d++) {
            const int nearestGridComponent = std::round(scaledIndex[d]);
            const int directionToNextNeighbour = (scaledIndex[d] < nearestGridComponent) ? -1 : 1;
            nearestNeighbour[d] = nearestGridComponent;
            directionsToNeighbours[d] = directionToNextNeighbour;
        }
        const float weightNN = getNeighbourWeight(scaledIndex, nearestNeighbour);
        weightSum += weightNN;
        neighbours.push_back(nearestNeighbour);
        weights.push_back(weightNN);

        // After having identified the nearest neighbour and its weight, do the same to its adjacent
        // cells whose projection (for each dimension) is closest the the scaled index coordinates
        for(int d = 0; d < DIMENSIONS; d++) {
            Index adjacentNeighbour = nearestNeighbour;
            adjacentNeighbour[d] += directionsToNeighbours[d];
            const float weightAdjNeighbour = getNeighbourWeight(scaledIndex, adjacentNeighbour);
            weightSum += weightAdjNeighbour;
            neighbours.push_back(adjacentNeighbour);
            weights.push_back(weightAdjNeighbour);
        }

        // Normalize the weights so they sum 1
        for(int n = 0; n < DIMENSIONS + 1; n++) {
            weights[n] /= weightSum;
        }

    }

    void Grid::scaleIndex(const Index &index, std::array<float, 6> &scaledIndex) const {
        for(int d = 0; d < DIMENSIONS; d++) {
            const float scaledComponent = index[d] * this->scales[d];
            // scaledComponent += 0.0001;
            scaledIndex[d] = scaledComponent;
        }
    }

    float Grid::getNeighbourWeight(const std::array<float, 6> &scaledIndex, const Index &neighbourIndex) const {
        float weight = 1.0f;
        for(int d = 0; d < DIMENSIONS; d++) {
            weight *= 1 - std::abs(scaledIndex[d] - neighbourIndex[d]);
        }
        return weight;
    }
}