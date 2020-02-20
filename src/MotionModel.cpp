#include <opencv4/opencv2/calib3d/calib3d.hpp>
#include <iostream>
#include <chrono>
#include "MotionModel.h"
#include "Homography.h"

namespace tfg {

    MotionModel::MotionModel(std::shared_ptr<tfg::TrackTable> &trackTable, float tau) {
        this->trackTable = trackTable;
        this->tau = tau;
    }
    MotionModel::~MotionModel() {}

    void MotionModel::fitFromWeights(std::vector<float> &weights2) {
        this->weights2 = weights2;

        computeHomographiesWLS();
        computeResiduals2();
        computeModelCost();
    }

    void MotionModel::computeHomographiesWLS() {
        homographies.clear();
        homographies.reserve(trackTable->numberOfFrames());
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
        for(unsigned int t = 0; t < trackTable->numberOfTracks(); t++) {
            const std::vector<cv::Vec2f> coordinates = trackTable->pointsInTrack(t);

            float maxReprojectionError2 = 0.0f;
            for(unsigned int i = 0; i < coordinates.size() - 1; i++) {
                cv::Vec3f pointL;
                cv::Vec3f pointR;
                pointL(0) = coordinates[i](0);   pointL(1) = coordinates[i](1);   pointL(2) = 1.0f;
                pointR(0) = coordinates[i+1](0); pointR(1) = coordinates[i+1](1); pointR(2) = 1.0f;

                cv::Vec3f pred = (homographies[trackTable->firstFrameOfTrack(t) + i])*pointL;
                
                float reprojectionError2 = cv::norm(pointR - pred, cv::NORM_L2SQR);
                if(reprojectionError2 > maxReprojectionError2) maxReprojectionError2 = reprojectionError2;
            }

            residuals2.push_back(maxReprojectionError2);
        }
    }

    void MotionModel::computeModelCost() {
        cost = 0.0f;
        for(unsigned int i = 0; i < residuals2.size(); i++) {
            float trackCost = tau*tau < residuals2[i] ? tau*tau/4 : (residuals2[i]/2)*(1-residuals2[i]/(2*tau*tau));
            cost += trackCost;
        }
    }

    void MotionModel::fitFromRANSAC(std::vector<int> &inliers) {
        computeHomographiesRANSAC(inliers);
        computeResiduals2();
        computeModelCost();
    }

    void MotionModel::computeHomographiesRANSAC(std::vector<int> &inliers) {
        homographies.clear();
        homographies.reserve(trackTable->numberOfFrames());
        for(unsigned int f = 0; f < trackTable->numberOfFrames(); f++) {

            std::vector<cv::Vec2f> origin = trackTable->originPointsInFrame(f);
            std::vector<cv::Vec2f> destination = trackTable->destinationPointsInFrame(f);
            cv::Matx33f H;
            // cv::Matx33f H = cv::findHomography(origin, destination, cv::RANSAC, 1.0);
            tfg::computeHomographyRANSAC(origin, destination, origin.size(), 500, 1.0f, H, inliers);
            homographies.push_back(H);
        }
    }

    void MotionModel::printHomography(int n) {
        assert(n < homographies.size());
        for(int i = 0; i < 3; i++) {
            for(int j = 0; j < 3; j++) {
                std::cout << homographies[n](i, j) << " ";
            }
            std::cout << std::endl;
        }
        std::cout << std::endl;
    }  
}