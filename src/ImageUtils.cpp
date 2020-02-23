#include <iostream>
#include <opencv4/opencv2/core.hpp>
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

    void paintTracks(const std::shared_ptr<tfg::TrackTable> &trackTable, const std::vector<float> &weights2, std::vector<cv::Mat> images, const std::string &folder, const std::string &fileName) {
        for(unsigned int t = 0; t < trackTable->numberOfTracks(); t++) {
            std::vector<cv::Vec2f> points = trackTable->pointsInTrack(t);
            unsigned int initFrame = trackTable->firstFrameOfTrack(t);

            for(unsigned int f = 0; f < points.size(); f++) {
                cv::Vec3b color;
                color(2) = weights2[t] < 0.25 ? 0 : 255;
                color(1) = weights2[t] < 0.25 ? 255 : 0;
                // drawPoint(images[initFrame + f], points[f], color);
                const int remainingFrames = points.size() - f - 1;
                const int T = std::min(5, remainingFrames);
                if(T != 0) {
                    drawLine(images[initFrame + f], points[f], points[f + 1], color);
                } else {
                    drawPoint(images[initFrame + f], points[f], color);
                }
            }
        }

        for (unsigned int i = 0; i < images.size(); i++) {
            std::stringstream ss;
            ss << folder << fileName << i << ".png";
            std::string saveAs = ss.str();
            cv::imwrite(saveAs, images[i]);
        }
    }

    void paintSeededTracks(const std::shared_ptr<tfg::TrackTable> &trackTable, std::vector<cv::Mat> images, const std::string &folder, const std::string &fileName) {
        for(unsigned int t = 0; t < trackTable->numberOfTracks(); t++) {
            const int label = trackTable->labelOfTrack(t);
            std::vector<cv::Vec2f> points = trackTable->pointsInTrack(t);
            unsigned int initFrame = trackTable->firstFrameOfTrack(t);

            if(label < 0) continue;

            // std::cout << "Correctly labeled image" << std::endl;

            for(unsigned int f = 0; f < points.size(); f++) {
                cv::Vec3b color;
                color(2) = (label + 1) % 2 == 1 ? 255 : 0;
                color(1) = (label + 1) % 4 >= 2 ? 255 : 0;
                color(0) = (label + 1) % 8 >= 4 ? 255 : 0;
                // drawPoint(images[initFrame + f], points[f], color);
                const int remainingFrames = points.size() - f - 1;
                const int T = std::min(10, remainingFrames);
                if(T != 0) {
                    drawLine(images[initFrame + f], points[f], points[f + 1], color);
                } else {
                    drawPoint(images[initFrame + f], points[f], color);
                }
            }
        }

        for (unsigned int i = 0; i < images.size(); i++) {
            std::stringstream ss;
            ss << folder << fileName << i << ".png";
            std::string saveAs = ss.str();
            cv::imwrite(saveAs, images[i]);
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
}
