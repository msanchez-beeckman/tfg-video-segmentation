#ifndef TFG_VIDEO_SEGMENTATION_MOTIONMODEL_H
#define TFG_VIDEO_SEGMENTATION_MOTIONMODEL_H

#include "Track.h"

namespace tfg {

    class MotionModel {
    private:
        std::vector<tfg::Track> tracks;
        std::vector<tfg::Mapping> mappings;
        std::vector<float> weights2;
        float tau;

        std::vector<libUSTG::laMatrix> homographies;
        std::vector<float> residuals2;
        float cost;

        void computeHomographiesWLS();
        void computeHomographiesRANSAC();
        void computeResiduals2();
        void computeModelCost();
    
    public:
        MotionModel();
        ~MotionModel();

        void fitFromWeights(std::vector<tfg::Track> &tracks, std::vector<tfg::Mapping> &mappings, std::vector<float> &weights2, float tau);
        void fitFromRANSAC(std::vector<tfg::Track> &tracks, std::vector<tfg::Mapping> &mappings, float tau);

        void printHomography(int n);
        inline std::vector<float> getResiduals2() const {
            return residuals2;
        };
        inline float getCost() const {
            return cost;
        };
        inline std::vector<tfg::Track> getTracks() const {
            return tracks;
        }
        inline std::vector<tfg::Mapping> getMappings() const {
            return mappings;
        }
    };

}

#endif //TFG_VIDEO_SEGMENTATION_MOTIONMODEL_H