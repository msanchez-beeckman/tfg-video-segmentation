#include <chrono>
#include "MotionModel.h"
#include "Homography.h"

namespace tfg {

    MotionModel::MotionModel() {}
    MotionModel::~MotionModel() {}

    void MotionModel::fitFromWeights(tfg::TrackTable &trackTable, std::vector<float> &weights2, float tau) {
        this->trackTable = trackTable;
        this->weights2 = weights2;
        this->tau = tau;

        computeHomographiesWLS();
        computeResiduals2();
        computeModelCost();
    }

    void MotionModel::computeHomographiesWLS() {
        homographies.clear();
        homographies.reserve(trackTable.numberOfFrames());
        for(unsigned int f = 0; f < trackTable.numberOfFrames(); f++) {
            cv::Matx33f H;
            homographies.push_back(H);

            std::vector<cv::Vec2f> origin = trackTable.originPointsInFrame(f);
            std::vector<cv::Vec2f> destination = trackTable.destinationPointsInFrame(f);
            std::vector<unsigned int> trajectories = trackTable.trajectoriesInFrame(f);

            tfg::computeHomographyWLS(origin, destination, origin.size(), trajectories, weights2, homographies[f]);
        }
    }

    void MotionModel::computeResiduals2() {
        residuals2.clear();
        residuals2.reserve(trackTable.numberOfTracks());
        for(unsigned int t = 0; t < tracks.size(); t++) {
            const std::vector<cv::Vec2f> coordinates = trackTable.pointsInTrack(t);

            float maxReprojectionError2 = 0.0f;
            for(unsigned int i = 0; i < coordinates.size() - 1; i++) {
                cv::Vec3f pointL;
                cv::Vec3f pointR;
                pointL(0) = coordinates[i](0);   pointL(1) = coordinates[i](1);   pointL(2) = 1.0f;
                pointR(0) = coordinates[i+1](0); pointR(1) = coordinates[i+1](1); pointR(2) = 1.0f;

                cv::Vec3f pred = (homographies[trackTable.firstFrameOfTrack(t) + i])*pointL;
                
                // cv::Vec3f diff = pointR - pred;
                // float reprojectionError2 = diff[0]*diff[0] + diff[1]*diff[1] + diff[2]*diff[2];
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

    void MotionModel::fitFromRANSAC(tfg::TrackTable &trackTable, float tau) {
        this->trackTable = trackTable;
        this->tau = tau;

        computeHomographiesRANSAC();
        computeResiduals2();
        computeModelCost();
    }

    void MotionModel::computeHomographiesRANSAC() {
        homographies.clear();
        homographies.reserve(trackTable.numberOfFrames());
        for(unsigned int f = 0; f < mappings.size(); f++) {
            cv::Matx33f H;
            homographies.push_back(H);

            std::vector<cv::Vec2f> origin = trackTable.originPointsInFrame(f);
            std::vector<cv::Vec2f> destination = trackTable.destinationPointsInFrame(f);

            tfg::computeHomographyRANSAC(origin, destination, origin.size(), 100, 0.8f, homographies[f]);
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