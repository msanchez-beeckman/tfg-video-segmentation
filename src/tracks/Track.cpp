/*
 * Copyright (c) 2020-2021, Marco Sánchez Beeckman
 * All rights reserved.
 *
 * This source code is licensed under the BSD-style license found in the
 * LICENSE file in the root directory of this source tree.
 */

#include <cmath>
#include <iostream>
#include <limits>

#include <opencv2/core.hpp>

#include "Track.h"

namespace tfg {

Track::Track() {
    this->label = -1;
}

Track::Track(const std::vector<cv::Vec2f> &coordinates, const unsigned int initFrame) {
    this->coordinates = coordinates;
    this->initFrame = initFrame;
    this->label = -1;
}

Track::~Track() {}

void Track::addPoint(const cv::Vec2f &point) {
    coordinates.push_back(point);
}

void Track::addPoint(const cv::Vec2f &point, const cv::Vec3b &color) {
    coordinates.push_back(point);
    colors.push_back(color);
}

void Track::obtainColors(const std::vector<cv::Mat> &sequence) {
    colors.resize(coordinates.size());
    for(unsigned int i = 0; i < coordinates.size(); i++) {
        const int COL = std::round(coordinates[i](0));
        const int ROW = std::round(coordinates[i](1));
        const cv::Vec3b& color = sequence[initFrame + i].at<cv::Vec3b>(ROW, COL);
        colors[i] = color;
    }
}

void Track::setNumber(unsigned int number) {
    this->number = number;
}

void Track::setLabel(int label) {
    this->label = label;
}

/**
 * Compute squared distance between two tracks, defined in eqs. (1) and (4) of the paper:
 * 
 * P. Ochs, J. Malik, and T. Brox.
 * Segmentation of moving objects by long term video analysis.
 * IEEE Transactions on Pattern Analysis and Machine Intelligence, 36(6): 1187-1200, Jun 2014.
 * DOI: 10.1109/TPAMI.2013.242
 * 
 * @param trackB A second track.
 * @param flowVariances The variances of the flow in each frame, to scale the distances (a movement has more significance if the rest of the frame is not moving so much).
 */
float Track::distance2(const Track &trackB, const std::vector<float> &flowVariances) const {

    const float averageSpatialDistance = this->averageSpatialDistance(trackB);
    // const float maximalSpatialDistance = this->maximalSpatialDistance(trackB);
    const float averageColorDistance = this->averageColorDistance(trackB);
    const float maximalMotionDistance = this->maximalMotionDistance(trackB, flowVariances);

    // std::cout << "Sp: " << averageSpatialDistance << " Color: " << averageColorDistance << " Motion: " << maximalMotionDistance << '\n';
    const float distance2 = averageSpatialDistance*maximalMotionDistance*maximalMotionDistance;
    return distance2;
}

float Track::maximalMotionDistance(const Track &trackB, const std::vector<float> &flowVariances) const {
    // The distance is computed using only the common frames between the tracks
    constexpr int MIN_COMMON_FRAMES = 5;
    const int firstCommonFrame = std::max(initFrame, trackB.getInitFrame());
    const int lastCommonFrame = std::min(initFrame + this->getDuration() - 1, trackB.getInitFrame() + trackB.getDuration() - 1);
    if(lastCommonFrame - firstCommonFrame + 1 < MIN_COMMON_FRAMES) return std::numeric_limits<float>::infinity(); // If not enough common frames, consider an infinite distance
    // if(firstCommonFrame > lastCommonFrame) return std::numeric_limits<float>::infinity();

    // The motion distance of two tracks at time t is defined as the norm of the difference of their partial derivatives w.r.t. the time dimension,
    // divided by the variation of the optical flow at that time.
    // The maximal motion distance is the maximum of the motion distances for all the frames where the tracks coincide
    float maxDistance = 0.0f;
    for(unsigned int frame = firstCommonFrame; frame < lastCommonFrame; frame++) {
        cv::Vec2f derivativeA, derivativeB;
        this->deriveForwardDifferences(frame, derivativeA);
        trackB.deriveForwardDifferences(frame, derivativeB);

        const float motionDistance = cv::norm(derivativeA - derivativeB) / std::sqrt(flowVariances[frame]);

        if(motionDistance > maxDistance) maxDistance = motionDistance;
    }

    return maxDistance;

}

/**
 * Approximate derivative in a frame using forward differences.
 * @param frame The frame where the derivative should be calculated.
 * @param derivative An output vector containing the derivative.
 */
void Track::deriveForwardDifferences(unsigned int frame, cv::Vec2f &derivative) const {
    const unsigned int duration = this->getDuration();
    // assert(frame >= initFrame && frame < initFrame + duration - 1);

    const int framesForward = 5;
    const int step = frame + framesForward < initFrame + duration ? framesForward : initFrame + duration - frame - 1;
    derivative = (coordinates[frame - initFrame + step] - coordinates[frame - initFrame])/step;
}

/**
 * Compute the average spatial distance between two tracks.
 * This is done calculating the euclidean distance between the points in the tracks for their common frames, and then averaging them.
 * @param trackB Another track.
 */
float Track::averageSpatialDistance(const Track &trackB) const {
    const unsigned int firstCommonFrame = std::max(initFrame, trackB.getInitFrame());
    const unsigned int lastCommonFrame = std::min(initFrame + this->getDuration() - 1, trackB.getInitFrame() + trackB.getDuration() - 1);
    if(firstCommonFrame > lastCommonFrame) return std::numeric_limits<float>::infinity();

    std::vector<cv::Vec2f> trackBcoordinates = trackB.getPoints();
    float distanceSum = 0.0f;
    for(unsigned int frame = firstCommonFrame; frame <= lastCommonFrame; frame++) {
        distanceSum += cv::norm(this->coordinates[frame - initFrame] - trackBcoordinates[frame - trackB.getInitFrame()]);
    }
    float averageDistance = distanceSum / (lastCommonFrame - firstCommonFrame + 1);
    return averageDistance;
}

/**
 * Compute the maximal spatial distance between two tracks.
 * This is done calculating the euclidean distance between the points in the tracks for their common frames, and then keeping the biggest number.
 * @param trackB Another track.
 */
float Track::maximalSpatialDistance(const Track &trackB) const {
    const unsigned int firstCommonFrame = std::max(initFrame, trackB.getInitFrame());
    const unsigned int lastCommonFrame = std::min(initFrame + this->getDuration() - 1, trackB.getInitFrame() + trackB.getDuration() - 1);
    if(firstCommonFrame > lastCommonFrame) return std::numeric_limits<float>::infinity();

    std::vector<cv::Vec2f> trackBcoordinates = trackB.getPoints();
    float maxDistance = 0.0f;
    for(unsigned int frame = firstCommonFrame; frame <= lastCommonFrame; frame++) {
        float distance = cv::norm(this->coordinates[frame - initFrame] - trackBcoordinates[frame - trackB.getInitFrame()]);
        maxDistance = distance > maxDistance ? distance : maxDistance;
    }
    return maxDistance;
}

float Track::averageColorDistance(const Track &trackB) const {
    const unsigned int firstCommonFrame = std::max(initFrame, trackB.getInitFrame());
    const unsigned int lastCommonFrame = std::min(initFrame + this->getDuration() - 1, trackB.getInitFrame() + trackB.getDuration() - 1);
    if(firstCommonFrame > lastCommonFrame) return std::numeric_limits<float>::infinity();

    std::vector<cv::Vec3b> trackBcolors = trackB.getColors();
    float distanceSum = 0.0f;
    for(unsigned int frame = firstCommonFrame; frame <= lastCommonFrame; frame++) {
        distanceSum += cv::norm(this->colors[frame - initFrame] - trackBcolors[frame - trackB.getInitFrame()]);
    }
    float averageDistance = distanceSum / (lastCommonFrame - firstCommonFrame + 1);
    return averageDistance;
}

} // namespace