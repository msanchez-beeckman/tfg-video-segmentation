
#ifndef TFG_VIDEO_SEGMENTATION_GRID_H
#define TFG_VIDEO_SEGMENTATION_GRID_H

#include <opencv4/opencv2/core.hpp>
#include "TrackTable.h"


namespace tfg {

    class Grid {
        private:
            using Index = std::vector<int>;
            using Value = cv::Vec4f;

            const int DIMENSIONS;
            const std::vector<float> SCALES;

            cv::SparseMat data;

            void splatValue(const Index &index, const Value &value);
            void sliceIndex(const Index &index, Value &value);

            void getNeighbours(const Index &index, std::vector<Index> &neighbours, std::vector<float> &weights) const;
            void scaleIndex(const Index &index, std::vector<float> &scaledIndex) const;
            float getNeighbourWeight(const std::vector<float> &scaledIndex, const Index &neighbourIndex) const;

        public:
            Grid();
            Grid(int dims, const std::vector<float> &scales) : DIMENSIONS(dims), SCALES(scales);
            ~Grid();

            void splatMass(const std::vector<cv::Mat> &images);
            void splatTrackWeights(const std::vector<cv::Mat> &images, const tfg::TrackTable &trackTable, const std::vector<float> &weights);
            void slice(const std::vector<cv::Mat> &images, std::vector<cv::Mat> &masks);


    };
}

#endif //TFG_VIDEO_SEGMENTATION_GRID_H