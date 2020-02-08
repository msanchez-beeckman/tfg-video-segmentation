#ifndef TFG_VIDEO_SEGMENTATION_MOTIONMODEL_H
#define TFG_VIDEO_SEGMENTATION_MOTIONMODEL_H

#include <opencv4/opencv2/core.hpp>
#include "TrackTable.h"

namespace tfg {

    class MotionModel {
    private:
        tfg::TrackTable trackTable;
        std::vector<float> weights2;
        float tau;

        std::vector<cv::Matx33f> homographies;
        std::vector<float> residuals2;
        float cost;

        void computeHomographiesWLS();
        void computeHomographiesRANSAC();
        void computeResiduals2();
        void computeModelCost();
    
    public:
        MotionModel();
        ~MotionModel();

        void fitFromWeights(tfg::TrackTable &trackTable, std::vector<float> &weights2, float tau);
        void fitFromRANSAC(tfg::TrackTable &trackTable, float tau);

        void printHomography(int n);
        inline std::vector<float> getResiduals2() const {
            return residuals2;
        };
        inline float getCost() const {
            return cost;
        };
        inline const tfg::TrackTable& getTrackTable() const {
            return tracks;
        };
    };

}

#endif //TFG_VIDEO_SEGMENTATION_MOTIONMODEL_H