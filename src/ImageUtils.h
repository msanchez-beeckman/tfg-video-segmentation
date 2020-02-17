
#ifndef TFG_VIDEO_SEGMENTATION_IMAGEUTILS_H
#define TFG_VIDEO_SEGMENTATION_IMAGEUTILS_H

#include <opencv4/opencv2/core.hpp>
#include <unordered_map>
#include "TrackTable.h"

namespace tfg {

    bool point_in_image(int x, int y, int w, int h);
    int val_coord(int i, int w);

    void readImages(std::istream &file, std::vector<cv::Mat> &images);
    void readSeedImages(std::istream &file, std::unordered_map<int, cv::Mat> &seedImages);
    void paintTracks(std::unique_ptr<tfg::TrackTable> &trackTable, std::vector<float> &weights2, std::vector<cv::Mat> &images, std::string &folder);
    void paintSeededTracks(std::shared_ptr<tfg::TrackTable> &trackTable, std::vector<cv::Mat> &images, std::string &folder);
}

#endif //TFG_VIDEO_SEGMENTATION_IMAGEUTILS_H
