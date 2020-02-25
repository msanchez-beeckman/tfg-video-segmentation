#include <limits>
#include <iostream>
#include "Track.h"

namespace tfg {

    Track::Track() {
        this->label = -1;
    }

    Track::Track(const std::vector<cv::Vec2f> &coordinates, const unsigned int initFrame) {
        this->coordinates = coordinates;
        this->initFrame = initFrame;
        this->duration = coordinates.size();
        this->label = -1;
    }

    Track::~Track() {}

    void Track::setNumber(unsigned int number) {
        this->number = number;
    }

    void Track::setLabel(int label) {
        this->label = label;
    }

    float Track::maximalMotionDistance2(const Track &trackB, const std::vector<float> &flowVariances) const {
        const unsigned int MIN_COMMON_FRAMES = 1;
        const unsigned int firstCommonFrame = std::max(initFrame, trackB.getInitFrame());
        const unsigned int lastCommonFrame = std::min(initFrame + duration - 1, trackB.getInitFrame() + trackB.getDuration() - 1);
        if(lastCommonFrame - firstCommonFrame + 1 < MIN_COMMON_FRAMES) return std::numeric_limits<float>::infinity();

        // float averageSpatialDistance = this->averageSpatialDistance(trackB);
        float maximalSpatialDistance = this->maximalSpatialDistance(trackB);
        float maxDistance2 = 0.0f;
        for(unsigned int frame = firstCommonFrame; frame < lastCommonFrame; frame++) {
            cv::Vec2f derivativeA, derivativeB;
            this->deriveForwardDifferences(frame, derivativeA);
            trackB.deriveForwardDifferences(frame, derivativeB);

            // float frameDistance2 = averageSpatialDistance * cv::norm(derivativeA - derivativeB, cv::NORM_L2SQR);
            float frameDistance2 = maximalSpatialDistance * cv::norm(derivativeA - derivativeB, cv::NORM_L2SQR);
            frameDistance2 /= flowVariances[frame];

            if(frameDistance2 > maxDistance2) maxDistance2 = frameDistance2;
        }

        return maxDistance2;
    }

    void Track::deriveForwardDifferences(unsigned int frame, cv::Vec2f &derivative) const {
        assert(frame >= initFrame && frame < initFrame + duration - 1);

        const int framesForward = 5;
        const int step = frame + framesForward < initFrame + duration ? framesForward : initFrame + duration - frame - 1;
        derivative = (coordinates[frame - initFrame + step] - coordinates[frame - initFrame])/step;
    }

    float Track::averageSpatialDistance(const Track &trackB) const {
        const unsigned int firstCommonFrame = std::max(initFrame, trackB.getInitFrame());
        const unsigned int lastCommonFrame = std::min(initFrame + duration - 1, trackB.getInitFrame() + trackB.getDuration() - 1);
        if(firstCommonFrame > lastCommonFrame) return std::numeric_limits<float>::infinity();

        std::vector<cv::Vec2f> trackBcoordinates = trackB.getPoints();
        float distanceSum = 0.0f;
        for(unsigned int frame = firstCommonFrame; frame <= lastCommonFrame; frame++) {
            distanceSum += cv::norm(this->coordinates[frame - initFrame] - trackBcoordinates[frame - trackB.getInitFrame()]);
        }
        float averageDistance = distanceSum / (lastCommonFrame - firstCommonFrame + 1);
        return averageDistance;
    }

    float Track::maximalSpatialDistance(const Track &trackB) const {
        const unsigned int firstCommonFrame = std::max(initFrame, trackB.getInitFrame());
        const unsigned int lastCommonFrame = std::min(initFrame + duration - 1, trackB.getInitFrame() + trackB.getDuration() - 1);
        if(firstCommonFrame > lastCommonFrame) return std::numeric_limits<float>::infinity();

        std::vector<cv::Vec2f> trackBcoordinates = trackB.getPoints();
        float maxDistance = 0.0f;
        for(unsigned int frame = firstCommonFrame; frame <= lastCommonFrame; frame++) {
            float distance = cv::norm(this->coordinates[frame - initFrame] - trackBcoordinates[frame - trackB.getInitFrame()]);
            maxDistance = distance > maxDistance ? distance : maxDistance;
        }
        return maxDistance;
    }


}