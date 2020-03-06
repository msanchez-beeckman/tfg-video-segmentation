
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
            void transitionMatrix(int F, int L, float sigma2, Eigen::SparseMatrix<float> &normalizedTransM);

            inline std::vector<int> getFrameBeginningIndices() const {
                return frameBeginningIndex;
            };

            inline cv::Mat getLabelsOfFrame(int frame) const {
                return superpixels[frameBeginningIndex[frame]].getFrameLabels();
            }
    };
}

#endif //TFG_VIDEO_SEGMENTATION_REGIONLIST_H