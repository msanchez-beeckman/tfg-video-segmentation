#include <iostream>
#include <algorithm>
#include <opencv4/opencv2/core.hpp>
#include <opencv4/opencv2/imgproc.hpp>
#include <opencv4/opencv2/imgcodecs.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/math/special_functions/round.hpp>
#include "ImageUtils.h"

namespace tfg {

    bool point_in_image(int x, int y, int w, int h) {
        return x >= 0 && x < w && y >= 0 && y < h;
    }

    int val_coord(int i, int w) {
        int i_new = std::min(std::max(i, 0), w - 1);
        return i_new;
    }

    void readImages(std::istream &file, std::vector<cv::Mat> &images) {
        for(std::string line; std::getline(file, line); ) {
            cv::Mat image = cv::imread(line, cv::IMREAD_COLOR);
            images.push_back(image);
        }
    }

    void copyImages(const std::vector<cv::Mat> &images1, std::vector<cv::Mat> &images2) {
        const unsigned int NUMBER_OF_IMAGES = images1.size();
        images2.clear();
        images2.reserve(NUMBER_OF_IMAGES);

        for(unsigned int i = 0; i < NUMBER_OF_IMAGES; i++) {
            // cv::Mat imageCopy = images1[i].clone();
            cv::Mat imageCopy;
            images1[i].copyTo(imageCopy);
            images2.push_back(imageCopy);
        }
    }

    void readSeedImages(std::istream &file, std::unordered_map<int, cv::Mat> &seedImages) {
        for(std::string line; std::getline(file, line); ) {
            std::vector<std::string> words;
            boost::split(words, line, boost::is_any_of(" "));
            const int imageNumber = std::stoi(words[1]);
            cv::Mat seedImage = cv::imread(words[0], cv::IMREAD_COLOR);
            seedImages[imageNumber] = seedImage;
        }
    }

    void bgr2luv(const std::vector<cv::Mat> &images1, std::vector<cv::Mat> &images2) {
        images2.clear();
        images2.reserve(images1.size());
        for(unsigned int i = 0; i < images1.size(); i++) {
            cv::Mat otherImage;
            cv::cvtColor(images1[i], otherImage, cv::COLOR_BGR2Luv);
            images2.push_back(otherImage);
        }
    }

    void luv2bgr(const std::vector<cv::Mat> &images1, std::vector<cv::Mat> &images2) {
        images2.clear();
        images2.reserve(images1.size());
        for(unsigned int i = 0; i < images1.size(); i++) {
            cv::Mat otherImage;
            cv::cvtColor(images1[i], otherImage, cv::COLOR_Luv2BGR);
            images2.push_back(otherImage);
        }
    }

    void drawPoint(cv::Mat &image, const cv::Vec2f &position, const cv::Vec3b &color) {
        const int x = static_cast<int>(position(0));
        const int y = static_cast<int>(position(1));

        const int w = image.cols;
        const int h = image.rows;

        if(!point_in_image(x, y, w, h)) return;

        const int x_left = val_coord(x-1, w);
        const int x_right = val_coord(x+1, w);
        const int y_down = val_coord(y-1, h);
        const int y_up = val_coord(y+1, h);

        for(int i = x_left; i < x_right + 1; i++) {
            for(int j = y_down; j < y_up + 1; j++) {
                cv::Vec3b& pixel = image.at<cv::Vec3b>(j, i);
                pixel[2] = color[2];
                pixel[1] = color[1];
                pixel[0] = color[0];
            }
        }
    }

    void drawLine(cv::Mat &image, const cv::Vec2f &origin, const cv::Vec2f &destination, const cv::Vec3b &color) {
        const int xOrig = static_cast<int>(origin(0));
        const int yOrig = static_cast<int>(origin(1));
        const int xDest = static_cast<int>(destination(0));
        const int yDest = static_cast<int>(destination(1));

        const int w = image.cols;
        const int h = image.rows;

        if(!point_in_image(xOrig, yOrig, w, h) || !point_in_image(xDest, yDest, w, h)) return;

        drawPoint(image, origin, color); 

        const float distance = std::max(std::abs(xDest - xOrig), std::abs(yDest - yOrig));
        const float xStep = (xDest - xOrig) / distance;
        const float yStep = (yDest - yOrig) / distance;
        for(int i = 1; i <= distance; i++) {
            const int xPos = boost::math::iround(xOrig + i * xStep);
            const int yPos = boost::math::iround(yOrig + i * yStep);

            cv::Vec3b& pixel = image.at<cv::Vec3b>(yPos, xPos);
            pixel[2] = color[2];
            pixel[1] = color[1];
            pixel[0] = color[0];
        }
    }

    void removeSmallBlobs(cv::Mat &matrix, float threshold, float relativeSize) {
        cv::Mat mask(matrix > threshold);
        cv::Mat labels;
        cv::Mat stats;
        cv::Mat centroids;
        const int numberOfCC = cv::connectedComponentsWithStats(mask, labels, stats, centroids, 8, CV_16U, cv::CCL_DEFAULT);

        double minArea, maxArea;
        cv::minMaxIdx(stats.col(cv::CC_STAT_AREA).rowRange(1, stats.rows), &minArea, &maxArea);

        // Correct values in non-thresholded mask
        for(int i = 1; i < numberOfCC; i++) {
            int area = stats.at<int>(i, cv::CC_STAT_AREA);
            if(area > maxArea * relativeSize) continue;

            cv::Mat labelMask(labels == i);
            matrix.setTo(threshold - 0.00001, labelMask);
        }
    }

    void saveMaskedImages(const std::vector<cv::Mat> &images, const std::vector<cv::Mat> &masks, const std::string &folder, const std::string &fileName) {
        for(unsigned int i = 0; i < images.size(); i++) {
            cv::Mat maskedImage;
            images[i].copyTo(maskedImage, masks[i]);

            std::stringstream ss;
            ss << folder << fileName << i << ".png";
            std::string saveAs = ss.str();
            cv::imwrite(saveAs, maskedImage);
        }
    }

    void saveOverlaidImages(const std::vector<cv::Mat> &images, const std::vector<cv::Mat> &masks, const std::string &folder, const std::string &fileName, float alpha) {
        for(unsigned int i = 0; i < images.size(); i++) {

            cv::Mat redOverlay(images[i].size(), CV_8UC3);
            redOverlay.setTo(cv::Scalar(0, 0, 255), masks[i]);

            cv::Mat grayImage(images[i].size(), CV_8UC1);
            cv::cvtColor(images[i], grayImage, cv::COLOR_BGR2GRAY);

            cv::Mat overlaidImage(images[i].size(), CV_8UC3);
            cv::cvtColor(grayImage, overlaidImage, cv::COLOR_GRAY2BGR);

            redOverlay = (1 - alpha) * overlaidImage + alpha * redOverlay;
            redOverlay.copyTo(overlaidImage, masks[i]);
            
            std::stringstream ss;
            ss << folder << fileName << i << ".png";
            std::string saveAs = ss.str();
            cv::imwrite(saveAs, overlaidImage);
        }
    }
}
