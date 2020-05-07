#include <opencv4/opencv2/calib3d/calib3d.hpp>
#include <iostream>
#include <chrono>
#include "MotionModel.h"
#include "Homography.h"

namespace tfg {

    /**
     * Construct a new empty MotionModel object.
     * @param trackTable A table containing precomputed tracks and mappings.
     * @param tau2 Tau squared, an inlier threshold for equations (2) to (4) in Szeliski's paper.
     */
    MotionModel::MotionModel(std::shared_ptr<tfg::TrackTable> &trackTable, float tau2) {
        this->trackTable = trackTable;
        this->tau2 = tau2;
    }
    MotionModel::~MotionModel() {}

    /**
     * Fit the model from precomputed weights, and compute its residuals and cost.
     * @param weights2 Squared weights of the tracks.
     */
    void MotionModel::fitFromWeights(std::vector<float> &weights2) {
        this->weights2 = weights2;

        computeHomographiesWLS();
        computeResiduals2();
        computeModelCost();
    }

    void MotionModel::computeHomographiesWLS() {
        homographies.clear();
        homographies.reserve(trackTable->numberOfFrames());

        // For each frame, get the points that have origin in it and a destination in the next frame, and compute an homography using WLS
        for(unsigned int f = 0; f < trackTable->numberOfFrames(); f++) {
            std::vector<cv::Vec2f> origin = trackTable->originPointsInFrame(f);
            std::vector<cv::Vec2f> destination = trackTable->destinationPointsInFrame(f);
            std::vector<unsigned int> trajectories = trackTable->trajectoriesInFrame(f);

            cv::Matx33f H;
            tfg::computeHomographyWLS(origin, destination, origin.size(), trajectories, weights2, H);
            homographies.push_back(H);
        }
    }

    void MotionModel::computeResiduals2() {
        residuals2.clear();
        residuals2.reserve(trackTable->numberOfTracks());

        // For each track, compute its maximum reprojection error and asign it as the track's residual
        for(unsigned int t = 0; t < trackTable->numberOfTracks(); t++) {
            const std::vector<cv::Vec2f> coordinates = trackTable->pointsInTrack(t);

            float maxReprojectionError2 = 0.0f;
            // For each point in the trajectory, apply the corresponding homography to it and compare the result to the actual destination.
            // Keep the maximum error, which is used as the residual for the frame.
            for(unsigned int i = 0; i < coordinates.size() - 1; i++) {
                cv::Vec3f pointL;
                cv::Vec3f pointR;
                // Convert the points to projective coordinates
                pointL(0) = coordinates[i](0);   pointL(1) = coordinates[i](1);   pointL(2) = 1.0f;
                pointR(0) = coordinates[i+1](0); pointR(1) = coordinates[i+1](1); pointR(2) = 1.0f;

                // Apply the homography, and divide the result by the third component
                cv::Vec3f pred = (homographies[trackTable->firstFrameOfTrack(t) + i])*pointL;
                pred(0) = pred(0)/pred(2); pred(1) = pred(1) / pred(2); pred(2) = 1.0f;
                
                float reprojectionError2 = cv::norm(pointR - pred, cv::NORM_L2SQR);
                if(reprojectionError2 > maxReprojectionError2) maxReprojectionError2 = reprojectionError2;
            }

            residuals2.push_back(maxReprojectionError2);
        }
    }

    void MotionModel::computeModelCost() {
        cost = 0.0f;
        for(unsigned int i = 0; i < residuals2.size(); i++) {
            // Cost given by eq. (4) of Szeliski's paper
            float trackCost = tau2 < residuals2[i] ? tau2/4 : (residuals2[i]/2)*(1-residuals2[i]/(2*tau2));
            cost += trackCost;
        }
    }

    /**
     * Fit the model using RANSAC, and compute its residuals and cost.
     * @param iterations Number of iterations used in RANSAC.
     * @param tolerance Inlier tolerance.
     * @param inliers Output vector of track indexes, corresponding to the RANSAC inliers.
     */
    void MotionModel::fitFromRANSAC(int iterations, float tolerance, std::vector<std::vector<int>> &inliers) {
        computeHomographiesRANSAC(iterations, tolerance, inliers);
        computeResiduals2();
        computeModelCost();
    }

    void MotionModel::computeHomographiesRANSAC(int iterations, float tolerance, std::vector<std::vector<int>> &inliers) {
        homographies.clear();
        inliers.clear();
        homographies.reserve(trackTable->numberOfFrames());
        inliers.reserve(trackTable->numberOfFrames());

        // For each frame, get the points that have origin in it and a destination in the next frame, and compute an homography using RANSAC
        for(unsigned int f = 0; f < trackTable->numberOfFrames(); f++) {

            std::vector<cv::Vec2f> origin = trackTable->originPointsInFrame(f);
            std::vector<cv::Vec2f> destination = trackTable->destinationPointsInFrame(f);
            cv::Matx33f H;
            std::vector<int> frameInliers;
            tfg::computeHomographyRANSAC(origin, destination, origin.size(), iterations, tolerance, H, frameInliers);
            inliers.push_back(frameInliers);
            homographies.push_back(H);
        }
    }

    void MotionModel::printHomography(int n) {
        assert(n < homographies.size());
        for(int i = 0; i < 3; i++) {
            for(int j = 0; j < 3; j++) {
                std::cout << homographies[n](i, j) << " ";
            }
            std::cout << '\n';
        }
        std::cout << '\n';
    }  
}