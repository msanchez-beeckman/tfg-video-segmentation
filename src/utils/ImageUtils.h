/*
 * Copyright (c) 2020-2021, Marco Sánchez Beeckman
 * All rights reserved.
 *
 * This source code is licensed under the BSD-style license found in the
 * LICENSE file in the root directory of this source tree.
 */

#ifndef TFG_VIDEO_SEGMENTATION_IMAGEUTILS_H
#define TFG_VIDEO_SEGMENTATION_IMAGEUTILS_H

#include <fstream>
#include <unordered_map>

#include <opencv2/core.hpp>


namespace tfg {

bool isPointInImage(int x, int y, int w, int h);
int val_coord(int i, int w);

void readImages(std::ifstream &file, std::vector<cv::Mat> &images);
void readFlowsFlo(std::ifstream &file, std::vector<cv::Mat> &flows);
void readFlowsTiff(std::ifstream &file, std::vector<cv::Mat> &flows);
void copyImages(const std::vector<cv::Mat> &images1, std::vector<cv::Mat> &images2);
void readSeedImages(std::ifstream &file, std::unordered_map<int, cv::Mat> &seedImages);

void bgr2luv(const std::vector<cv::Mat> &images1, std::vector<cv::Mat> &images2);
void luv2bgr(const std::vector<cv::Mat> &images1, std::vector<cv::Mat> &images2);

void drawPoint(cv::Mat &image, const cv::Vec2f &position, const cv::Vec3b &color);
void drawLine(cv::Mat &image, const cv::Vec2f &origin, const cv::Vec2f &destination, const cv::Vec3b &color);

void removeSmallBlobs(cv::Mat &matrix, float threshold, float relativeSize);
void saveMaskedImages(const std::vector<cv::Mat> &images, const std::vector<cv::Mat> &masks, const std::string &folder, const std::string &fileName, int firstNameIndex=0);
void saveOverlaidImages(const std::vector<cv::Mat> &images, const std::vector<cv::Mat> &masks, const std::string &folder, const std::string &fileName, float alpha=0.4f, int firstNameIndex=0);
void saveOverlaidMultilabeledImages(const std::vector<cv::Mat> &images, const std::vector<cv::Mat> &labelMasks, int numberOfLabels, const std::string &folder, const std::string &fileName, float alpha=0.4f, int firstNameIndex=0);

} // namespace tfg

#endif // TFG_VIDEO_SEGMENTATION_IMAGEUTILS_H
