
#ifndef TFG_VIDEO_SEGMENTATION_IMAGEUTILS_H
#define TFG_VIDEO_SEGMENTATION_IMAGEUTILS_H

#include <opencv4/opencv2/core.hpp>
#include <fstream>
#include <unordered_map>

namespace tfg {

    bool point_in_image(int x, int y, int w, int h);
    int val_coord(int i, int w);

    void readImages(std::ifstream &file, std::vector<cv::Mat> &images);
    void copyImages(const std::vector<cv::Mat> &images1, std::vector<cv::Mat> &images2);
    void readSeedImages(std::ifstream &file, std::unordered_map<int, cv::Mat> &seedImages);

    void bgr2luv(const std::vector<cv::Mat> &images1, std::vector<cv::Mat> &images2);
    void luv2bgr(const std::vector<cv::Mat> &images1, std::vector<cv::Mat> &images2);

    void drawPoint(cv::Mat &image, const cv::Vec2f &position, const cv::Vec3b &color);
    void drawLine(cv::Mat &image, const cv::Vec2f &origin, const cv::Vec2f &destination, const cv::Vec3b &color);

    void removeSmallBlobs(cv::Mat &matrix, float threshold, float relativeSize);
    void saveMaskedImages(const std::vector<cv::Mat> &images, const std::vector<cv::Mat> &masks, const std::string &folder, const std::string &fileName);
    void saveOverlaidImages(const std::vector<cv::Mat> &images, const std::vector<cv::Mat> &masks, const std::string &folder, const std::string &fileName, float alpha=0.4f);
}

#endif //TFG_VIDEO_SEGMENTATION_IMAGEUTILS_H
