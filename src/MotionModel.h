#ifndef TFG_VIDEO_SEGMENTATION_MOTIONMODEL_H
#define TFG_VIDEO_SEGMENTATION_MOTIONMODEL_H

#include <opencv4/opencv2/core.hpp>
#include "TrackTable.h"

namespace tfg {

    class MotionModel {
    private:
        std::vector<float> weights2;
        float tau;

        std::vector<cv::Matx33f> homographies;
        std::vector<float> residuals2;
        float cost;

        void computeHomographiesWLS(std::unique_ptr<tfg::TrackTable> &trackTable);
        void computeHomographiesRANSAC(std::unique_ptr<tfg::TrackTable> &trackTable, std::vector<int> &inliers);
        void computeResiduals2(std::unique_ptr<tfg::TrackTable> &trackTable);
        void computeModelCost();
    
    public:
        MotionModel();
        ~MotionModel();

        void fitFromWeights(std::unique_ptr<tfg::TrackTable> &trackTable, std::vector<float> &weights2, float tau);
        void fitFromRANSAC(std::unique_ptr<tfg::TrackTable> &trackTable, std::vector<int> &inliers, float tau);

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