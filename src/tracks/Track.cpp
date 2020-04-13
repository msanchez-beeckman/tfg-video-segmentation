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
        this->label = -1;
    }

    Track::~Track() {}

    void Track::addPoint(const cv::Vec2f &point) {
        coordinates.push_back(point);
    }

    void Track::setNumber(unsigned int number) {
        this->number = number;
    }

    void Track::setLabel(int label) {
        this->label = label;
    }

    /**
     * Compute squared maximal motion distance between two tracks, defined in eqs. 1 and 4 of the paper:
     * 
     * P. Ochs, J. Malik, and T. Brox.
     * Segmentation of moving objects by long term video analysis.
     * IEEE Transactions on Pattern Analysis and Machine Intelligence, 36(6): 1187-1200, Jun 2014.
     * DOI: 10.1109/TPAMI.2013.242
     * 
     * @param trackB A second track.
     * @param flowVariances The variances of the flow in each frame, to scale the distances (a movement has more significance if the rest of the frame is not moving so much).
     */
    float Track::maximalMotionDistance2(const Track &trackB, const std::vector<float> &flowVariances) const {
        // The distance is computed using only the common frames between the tracks
        const unsigned int MIN_COMMON_FRAMES = 1;
        const unsigned int firstCommonFrame = std::max(initFrame, trackB.getInitFrame());
        const unsigned int lastCommonFrame = std::min(initFrame + this->getDuration() - 1, trackB.getInitFrame() + trackB.getDuration() - 1);
        if(lastCommonFrame - firstCommonFrame + 1 < MIN_COMMON_FRAMES) return std::numeric_limits<float>::infinity(); // If not enough common frames, consider an infinite distance

        // float averageSpatialDistance = this->averageSpatialDistance(trackB);
        float maximalSpatialDistance = this->maximalSpatialDistance(trackB);
        float maxDistance2 = 0.0f;
        // The motion distance in frame is defined multiplying a spatial distance by the difference of derivatives (in that frame) of the curves formed by the trajectories, scaled
        // by a variance parameter (normally, the variance of the flow in the frame).
        // The maximal motion distance is the highest value obtained for all common frames
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

    /**
     * Approximate derivative in a frame using forward differences.
     * @param frame The frame where the derivative should be calculated.
     * @param derivative An output vector containing the derivative.
     */
    void Track::deriveForwardDifferences(unsigned int frame, cv::Vec2f &derivative) const {
        const unsigned int duration = this->getDuration();
        assert(frame >= initFrame && frame < initFrame + duration - 1);

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


}