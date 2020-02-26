
#ifndef TFG_VIDEO_SEGMENTATION_IMAGEUTILS_H
#define TFG_VIDEO_SEGMENTATION_IMAGEUTILS_H

#include <opencv4/opencv2/core.hpp>
#include <unordered_map>
// #include "TrackTable.h"

namespace tfg {

    bool point_in_image(int x, int y, int w, int h);
    int val_coord(int i, int w);

    void readImages(std::istream &file, std::vector<cv::Mat> &images);
    void copyImages(const std::vector<cv::Mat> &images1, std::vector<cv::Mat> &images2);
    void readSeedImages(std::istream &file, std::unordered_map<int, cv::Mat> &seedImages);

    // void paintTracks(const std::shared_ptr<tfg::TrackTable> &trackTable, const std::vector<float> &weights2, std::vector<cv::Mat> images, const std::string &folder, const std::string &fileName);
    // void paintSeededTracks(const std::shared_ptr<tfg::TrackTable> &trackTable, std::vector<cv::Mat> images, const std::string &folder, const std::string &fileName);

    void drawPoint(cv::Mat &image, const cv::Vec2f &position, const cv::Vec3b &color);
    void drawLine(cv::Mat &image, const cv::Vec2f &origin, const cv::Vec2f &destination, const cv::Vec3b &color);
}

#endif //TFG_VIDEO_SEGMENTATION_IMAGEUTILS_H
