#include <limits>
#include "Track.h"

namespace tfg {
    unsigned int Track::count = 0;

    Track::Track() {
        ++Track::count;
        this->label = -1;
    }

    Track::Track(const std::vector<cv::Vec2f> &coordinates, const unsigned int initFrame) {
        ++Track::count;
        this->coordinates = coordinates;
        this->initFrame = initFrame;
        this->duration = coordinates.size();
        this->label = -1;
    }

    Track::~Track() {
        --Track::count;
    }

    void Track::setLabel(int label) {
        this->label = label;
    }

    float Track::maximalMotionDistance2(const Track &trackB, const std::vector<float> &flowVariances) const {
        const unsigned int firstCommonFrame = std::max(initFrame, trackB.getInitFrame());
        const unsigned int lastCommonFrame = std::min(initFrame + duration - 1, trackB.getInitFrame() + trackB.getDuration() - 1);
        if(firstCommonFrame > lastCommonFrame) return std::numeric_limits<float>::infinity();

        float averageSpatialDistance = this->averageSpatialDistance(trackB);
        float maxDistance2 = 0.0f;
        for(unsigned int frame = firstCommonFrame; frame <= lastCommonFrame; frame++) {
            cv::Vec2f derivativeA = this->deriveForwardDifferences(frame);
            cv::Vec2f derivativeB = trackB.deriveForwardDifferences(frame);

            float frameDistance2 = averageSpatialDistance * cv::norm(derivativeA - derivativeB, cv::NORM_L2SQR);
            frameDistance2 /= flowVariances[frame];

            if(frameDistance2 > maxDistance2) maxDistance2 = frameDistance2;
        }

        return maxDistance2;
    }

    cv::Vec2f Track::deriveForwardDifferences(unsigned int frame) const {
        assert(frame >= initFrame && frame < initFrame + duration);

        cv::Vec2f derivative;
        if(frame == initFrame + duration - 1) { // For the last frame, use backward differences
            const int framesBackward = 5;
            const int step = frame - framesBackward >= initFrame ? framesBackward : frame - initFrame;
            derivative = (coordinates[frame - initFrame] - coordinates[frame - initFrame - step])/step;
        } else {
            const int framesForward = 5;
            const int step = frame + framesForward < initFrame + duration ? framesForward : initFrame + duration - frame - 1;
            derivative = (coordinates[frame - initFrame + step] - coordinates[frame - initFrame])/step;
        }
        return derivative;
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


}