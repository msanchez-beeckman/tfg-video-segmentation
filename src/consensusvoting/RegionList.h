
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
            std::vector<unsigned int> frameBeginningIndex;

            cv::Mat descriptors;

            // void computeNearestNeighbours(int F = 15, int L = 4);

        public:
            RegionList();
            RegionList(unsigned int estimateSpPerFrame, unsigned int numberOfFrames);
            ~RegionList();

            void addNewFrame(std::vector<tfg::Region> &spInFrame);

            void computeDescriptors();
            Eigen::SparseMatrix<float> transitionMatrix(int F, int L, float sigma2);
    };
}

#endif //TFG_VIDEO_SEGMENTATION_REGIONLIST_H