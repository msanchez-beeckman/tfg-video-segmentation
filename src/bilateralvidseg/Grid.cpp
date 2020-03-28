#include <iostream>
#include <cmath>
#include "Grid.h"

namespace tfg {

    Grid::Grid() {}
    Grid::~Grid() {}

    Grid::Grid(int dims, const std::vector<float> &scales) : DIMENSIONS(dims), SCALES(scales) {}

    void Grid::splatMass(const std::vector<cv::Mat> &images) {

    }

    void Grid::splatValue(const Index &index, const Value &value) {
        std::vector<Index> neighbours;
        std::vector<float> weights;
        getNeighbours(index, neighbours, weights);

        for(unsigned int n = 0; n < neighbours.size(); n++) {
            const Index& neighbour = neighbours[n];
            data.ref<Value>(&neighbour[0]) += weights[n] * value;
        }
    }

    void Grid::getNeighbours(const Index &index, std::vector<Index> &neighbours, std::vector<float> &weights) const {
        std::vector<float> scaledIndex;
        scaleIndex(index, scaledIndex);

        neighbours.clear();
        neighbours.reserve(DIMENSIONS + 1);
        weights.clear();
        weights.reserve(DIMENSIONS + 1);

        Index nearestNeighbour;
        nearestNeighbour.reserve(DIMENSIONS);
        Index directionsToNeighbours;
        directionsToNeighbours.reserve(DIMENSIONS);
        // First of all, get the index of the cell nearest to the scaled coordinates
        // Also keep the direction of the vector that goes from the cell to the scaled coordinates,
        // since the sign of each of its coordinates indicates where the other neighbours are (one for each dimension)
        float weightSum = 0.0f;
        for(int d = 0; d < DIMENSIONS; d++) {
            const int nearestGridComponent = std::round(scaledIndex[d]);
            const int directionToNextNeighbour = (scaledIndex[d] < nearestGridComponent) ? -1 : 1;
            nearestNeighbour.push_back(nearestGridComponent);
            directionsToNeighbours.push_back(directionToNextNeighbour);
        }
        const float weightNN = getNeighbourWeight(scaledIndex, nearestNeighbour);
        weightSum += weightNN;
        neighbours.push_back(nearestNeighbour);
        weights.push_back(weightNN);

        // After having identified the nearest neighbour and its weight, do the same to its adjacent
        // cells whose projection (for each dimension) is closest the the scaled index coordinates
        for(int d = 0; d < DIMENSIONS; d++) {
            Index adjacentNeighbour = nearestNeighbour;
            adjacentNeighbour[d] += directionToNextNeighbour[d];
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

    void Grid::scaleIndex(const Index &index, std::vector<float> &scaledIndex) const {
        scaledIndex.clear();
        scaledIndex.reserve(DIMENSIONS);
        for(int d = 0; d < DIMENSIONS; d++) {
            const float scaledComponent = index[d] * SCALES[d];
            // scaledComponent += 0.0001;
            scaledIndex.push_back(scaledComponent);
        }
    }

    float getNeighbourWeight(const std::vector<float> &scaledIndex, const Index &neighbourIndex) const {
        float weight = 1.0f;
        for(int d = 0; d < DIMENSIONS; d++) {
            weight *= 1 - std::abs(scaledIndex[d] - neighbourIndex[d]);
        }
        return weight;
    }
}