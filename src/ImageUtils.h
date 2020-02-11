
#ifndef TFG_VIDEO_SEGMENTATION_IMAGEUTILS_H
#define TFG_VIDEO_SEGMENTATION_IMAGEUTILS_H

#include <opencv4/opencv2/core.hpp>
#include "TrackTable.h"

namespace tfg {

    bool point_in_image(int x, int y, int w, int h);
    int val_coord(int i, int w);

    void readImages(std::istream &file, std::vector<cv::Mat> &images);
    void paintTracks(std::unique_ptr<tfg::TrackTable> &trackTable, std::vector<float> &weights2, std::vector<cv::Mat> &images);
}

#endif //TFG_VIDEO_SEGMENTATION_IMAGEUTILS_H
