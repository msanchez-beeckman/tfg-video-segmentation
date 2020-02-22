#ifndef TFG_VIDEO_SEGMENTATION_MOTIONMODEL_H
#define TFG_VIDEO_SEGMENTATION_MOTIONMODEL_H

#include <opencv4/opencv2/core.hpp>
#include "TrackTable.h"

namespace tfg {

    class MotionModel {
    private:
        std::shared_ptr<tfg::TrackTable> trackTable;
        std::vector<float> weights2;
        float tau;

        std::vector<cv::Matx33f> homographies;
        std::vector<float> residuals2;
        float cost;

        void computeHomographiesWLS();
        void computeHomographiesRANSAC(std::vector<std::vector<int>> &inliers);
        void computeResiduals2();
        void computeModelCost();
    
    public:
        MotionModel(std::shared_ptr<tfg::TrackTable> &trackTable, float tau);
        ~MotionModel();

        void fitFromWeights(std::vector<float> &weights2);
        void fitFromRANSAC(std::vector<std::vector<int>> &inliers);

        void printHomography(int n);
        inline std::vector<float> getResiduals2() const {
            return residuals2;
        };
        inline float getCost() const {
            return cost;
        };
    };

}

#endif //TFG_VIDEO_SEGMENTATION_MOTIONMODEL_H