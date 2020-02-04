#include <chrono>
#include "MotionModel.h"
#include "Homography.h"

namespace tfg {

    MotionModel::MotionModel() {}
    MotionModel::~MotionModel() {}

    void MotionModel::fitFromWeights(std::vector<tfg::Track> &tracks, std::vector<tfg::Mapping> &mappings, std::vector<float> &weights2, float tau) {
        this->tracks = tracks;
        this->mappings = mappings;
        this->weights2 = weights2;
        this->tau = tau;

        computeHomographiesWLS();
        computeResiduals2();
        computeModelCost();
    }

    void MotionModel::computeHomographiesWLS() {
        homographies.clear();
        homographies.reserve(mappings.size());
        for(unsigned int f = 0; f < mappings.size(); f++) {
            libUSTG::laMatrix H(3,3);
            homographies.push_back(H);

            std::vector<float> originX = mappings[f].getOrigin_x();
            std::vector<float> originY = mappings[f].getOrigin_y();
            std::vector<float> destinationX = mappings[f].getDestination_x();
            std::vector<float> destinationY = mappings[f].getDestination_y();
            std::vector<unsigned int> trajectories = mappings[f].getTrajectories();

            tfg::compute_homography_WLS(&(originX[0]), &(originY[0]),
                                           &(destinationX[0]), &(destinationY[0]),
                                           originX.size(), trajectories, weights2, homographies[f]);
        }
    }

    void MotionModel::computeResiduals2() {
        residuals2.clear();
        residuals2.reserve(tracks.size());
        for(unsigned int t = 0; t < tracks.size(); t++) {
            const std::vector<float> coordinatesX = tracks[t].getPoints_x();
            const std::vector<float> coordinatesY = tracks[t].getPoints_y();

            float maxReprojectionError2 = 0.0f;
            for(unsigned int i = 0; i < coordinatesX.size() - 1; i++) {
                libUSTG::laVector p_start(3);
                libUSTG::laVector p_end(3);
                p_start[0] = coordinatesX[i]; p_start[1] = coordinatesY[i]; p_start[2] = 1;
                p_end[0] = coordinatesX[i + 1]; p_end[1] = coordinatesY[i + 1]; p_end[2] = 1;

                libUSTG::laVector pred_end = (homographies[tracks[t].getInitFrame() + i])*p_start;
                libUSTG::laVector diff(3);
                for(int i=0; i<3; i++) {
                    diff[i] = pred_end[i] - p_end[i];
                }

                float reprojectionError2 = diff[0]*diff[0] + diff[1]*diff[1] + diff[2]*diff[2];
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

    void MotionModel::fitFromRANSAC(std::vector<tfg::Track> &tracks, std::vector<tfg::Mapping> &mappings, float tau) {
        this->tracks = tracks;
        this->mappings = mappings;
        this->tau = tau;

        computeHomographiesRANSAC();
        computeResiduals2();
        computeModelCost();
    }

    void MotionModel::computeHomographiesRANSAC() {
        homographies.clear();
        homographies.reserve(mappings.size());
        for(unsigned int f = 0; f < mappings.size(); f++) {
            libUSTG::laMatrix H(3, 3);
            homographies.push_back(H);

            std::vector<float> originX = mappings[f].getOrigin_x();
            std::vector<float> originY = mappings[f].getOrigin_y();
            std::vector<float> destinationX = mappings[f].getDestination_x();
            std::vector<float> destinationY = mappings[f].getDestination_y();

            libUSTG::compute_ransac_planar_homography_n_points(&(originX[0]), &(originY[0]),
                                                           &(destinationX[0]), &(destinationY[0]),
                                                           originX.size(), 100, 2.0f, homographies[f], nullptr, 1);
            homographies[f] = homographies[f]/homographies[f][2][2];
        }
    }

    void MotionModel::printHomography(int n) {
        assert(n < homographies.size());
        tfg::printMatrix(homographies[n]);
    }
    
}