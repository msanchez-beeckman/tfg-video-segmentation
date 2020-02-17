#include <iostream>
#include <opencv4/opencv2/core.hpp>
#include <opencv4/opencv2/imgcodecs.hpp>
#include <boost/algorithm/string.hpp>
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

    void readSeedImages(std::istream &file, std::unordered_map<int, cv::Mat> &seedImages) {
        for(std::string line; std::getline(file, line); ) {
            std::vector<std::string> words;
            boost::split(words, line, boost::is_any_of(" "));
            const int imageNumber = std::stoi(words[1]);
            cv::Mat seedImage = cv::imread(words[0], cv::IMREAD_COLOR);
            seedImages[imageNumber] = seedImage;
        }
    }

    void paintTracks(std::unique_ptr<tfg::TrackTable> &trackTable, std::vector<float> &weights2, std::vector<cv::Mat> &images, std::string &folder) {
        for(unsigned int t = 0; t < trackTable->numberOfTracks(); t++) {
            std::vector<cv::Vec2f> points = trackTable->pointsInTrack(t);
            unsigned int initFrame = trackTable->firstFrameOfTrack(t);

            for(unsigned int f = 0; f < points.size(); f++) {
                int x = static_cast<int>(points[f](0));
                int y = static_cast<int>(points[f](1));

                // cv::Size imageSize = images[initFrame + f].size();
                // int w = imageSize.width;
                // int h = imageSize.height;
                int w = images[initFrame + f].cols;
                int h = images[initFrame + f].rows;

                if(!point_in_image(x, y, w, h)) continue;

                int x_left = val_coord(x-1, w);
                int x_right = val_coord(x+1, w);
                int y_down = val_coord(y-1, h);
                int y_up = val_coord(y+1, h);

                for(int i = x_left; i < x_right + 1; i++) {
                    for(int j = y_down; j < y_up + 1; j++) {
                        cv::Vec3b& color = images[initFrame + f].at<cv::Vec3b>(j, i);
                        color[1] = weights2[t] < 0.001 ? 255 : color[1];
                        color[2] = weights2[t] < 0.001 ? color[2] : 255;
                    }
                }
            }
        }

        for (unsigned int i = 0; i < images.size(); i++) {
            std::stringstream ss;
            ss << folder << "out" << i << ".png";
            std::string fileName = ss.str();
            cv::imwrite(fileName, images[i]);
        }
    }

    void paintSeededTracks(std::shared_ptr<tfg::TrackTable> &trackTable, std::vector<cv::Mat> &images, std::string &folder) {
        for(unsigned int t = 0; t < trackTable->numberOfTracks(); t++) {
            const int label = trackTable->labelOfTrack(t);
            std::vector<cv::Vec2f> points = trackTable->pointsInTrack(t);
            unsigned int initFrame = trackTable->firstFrameOfTrack(t);

            if(label < 0) continue;

            // std::cout << "Correctly labeled image" << std::endl;

            for(unsigned int f = 0; f < points.size(); f++) {
                int x = static_cast<int>(points[f](0));
                int y = static_cast<int>(points[f](1));
                int w = images[initFrame + f].cols;
                int h = images[initFrame + f].rows;

                if(!point_in_image(x, y, w, h)) continue;

                int x_left = val_coord(x-1, w);
                int x_right = val_coord(x+1, w);
                int y_down = val_coord(y-1, h);
                int y_up = val_coord(y+1, h);

                for(int i = x_left; i < x_right + 1; i++) {
                    for(int j = y_down; j < y_up + 1; j++) {
                        cv::Vec3b& color = images[initFrame + f].at<cv::Vec3b>(j, i);
                        // color[1] = label == 1 ? 255 : color[1];
                        // color[2] = label == 0 ? 255 : color[2];
                        color[2] = (label + 1) % 2 == 1 ? 255 : color[2];
                        color[1] = (label + 1) % 4 >= 2 ? 255 : color[1];
                        color[0] = (label + 1) % 8 >= 4 ? 255 : color[0];
                    }
                }
            }
        }

        for (unsigned int i = 0; i < images.size(); i++) {
            std::stringstream ss;
            ss << folder << "out" << i << ".png";
            std::string fileName = ss.str();
            cv::imwrite(fileName, images[i]);
        }
    }
}
