
#ifndef TFG_VIDEO_SEGMENTATION_GRID_H
#define TFG_VIDEO_SEGMENTATION_GRID_H

#include <opencv4/opencv2/core.hpp>
#include "TrackTable.h"


namespace tfg {

    class Grid {
        private:
            cv::SparseMat data;

        public:
            Grid();
            Grid(int dims, const std::vector<float> &scales);
            ~Grid();

            splatMass(const std::vector<cv::Mat> &images);
            splatTrackWeights(const std::vector<cv::Mat> &images, const tfg::TrackTable &trackTable, const std::vector<float> &weights);
            slice(const std::vector<cv::Mat> &images, std::vector<cv::Mat> &masks);


    };
}

#endif //TFG_VIDEO_SEGMENTATION_GRID_H