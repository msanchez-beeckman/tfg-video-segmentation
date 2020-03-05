
#ifndef TFG_VIDEO_SEGMENTATION_REGIONLIST_H
#define TFG_VIDEO_SEGMENTATION_REGIONLIST_H

#include <vector>
#include <eigen3/Eigen/Sparse>
#include <opencv4/opencv2/core.hpp>
#include "Region.h"

namespace tfg {
    class RegionList {
        private:
            std::vector<tfg::Region> superpixels;
            std::vector<int> frameBeginningIndex;

            cv::Mat descriptors;

        public:
            RegionList();
            RegionList(int estimateSpPerFrame, int numberOfFrames);
            ~RegionList();

            void addNewFrame(std::vector<tfg::Region> &spInFrame);

            void computeDescriptors();
            Eigen::SparseMatrix<float> transitionMatrix(int F, int L, float sigma2);

            inline std::vector<int> getFrameBeginningIndices() {
                return frameBeginningIndex;
            };
    };
}

#endif //TFG_VIDEO_SEGMENTATION_REGIONLIST_H