#include <iostream>
#include <chrono>
#include <opencv4/opencv2/imgproc/imgproc.hpp>
#include <opencv4/opencv2/objdetect/objdetect.hpp>
#include <cmath>
#include "Region.h"

namespace tfg {

    Region::Region() {}
    Region::~Region() {}

    Region::Region(int number, int frame, const cv::Mat &image, const cv::Mat &frameSuperpixelLabels) {
        this->number = number;
        this->frame = frame;
        this->image = image;
        this->frameSuperpixelLabels = frameSuperpixelLabels;

        // std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
        computeSuperpixelBoundaries();

        computeColorHistogramBGR(20);
        computeColorHistogramLAB(20);
        computeHOG(9, 6, 15);
        computeRelativeDistance();
        // std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
        // std::cout << "Descriptor of superpixel " << number << " in frame " << frame << " computed in " << (std::chrono::duration_cast<std::chrono::microseconds>(end-begin).count())/1000000.0 << " seconds." << std::endl;
    }

    void Region::computeSuperpixelBoundaries() {
        int xmax = 0;
        int xmin = frameSuperpixelLabels.cols - 1;
        int ymax = 0;
        int ymin = frameSuperpixelLabels.rows - 1;
        for(int x = 0; x < frameSuperpixelLabels.cols; x++) {
            for(int y = 0; y < frameSuperpixelLabels.rows; y++) {
                int value = frameSuperpixelLabels.at<int>(y, x);
                if(value != this->number) continue;
                ymin = y < ymin ? y : ymin;
                ymax = y > ymax ? y : ymax;
                xmin = x < xmin ? x : xmin;
                xmax = x > xmax ? x : xmax;
            }
        }
        boundaries.x = xmin;
        boundaries.y = ymin;
        boundaries.width = xmax - xmin + 1;
        boundaries.height = ymax - ymin + 1;
    }

    void Region::computeColorHistogramBGR(int nbins) {
        colorHistBGRDescriptor.release();

        cv::Mat mask(frameSuperpixelLabels == number);

        cv::Mat histB, histG, histR;
        int channelB[] = {0};
        int channelG[] = {1};
        int channelR[] = {2};
        int histSize[] = {nbins};
        float bgrRange[] = {0, 256};
        const float* ranges[] = {bgrRange};

        cv::calcHist(&image, 1, channelB, mask, histB, 1, histSize, ranges);
        cv::Scalar sumHistB = cv::sum(histB);
        histB /= sumHistB(0) + (sumHistB(0) == 0);
        histB = histB.reshape(0, 1);

        cv::calcHist(&image, 1, channelG, mask, histG, 1, histSize, ranges);
        cv::Scalar sumHistG = cv::sum(histG);
        histG /= sumHistG(0) + (sumHistG(0) == 0);
        histG = histG.reshape(0, 1);

        cv::calcHist(&image, 1, channelR, mask, histR, 1, histSize, ranges);
        cv::Scalar sumHistR = cv::sum(histR);
        histR /= sumHistR(0) + (sumHistR(0) == 0);
        histR = histR.reshape(0, 1);

        std::vector<cv::Mat> histVec = {histB, histG, histR};
        cv::hconcat(histVec, colorHistBGRDescriptor);
    }

    void Region::computeColorHistogramLAB(int nbins) {
        colorHistLABDescriptor.release();

        cv::Mat imageLab;
        cv::cvtColor(image, imageLab, cv::COLOR_BGR2Lab);
        cv::Mat mask(frameSuperpixelLabels == number);

        cv::Mat histL, histA, histB;
        int channelL[] = {0};
        int channelA[] = {1};
        int channelB[] = {2};
        int histSize[] = {nbins};

        // OpenCV converts the Lab values to range [0..255] in 8 bit images
        // float lRange[] = {0, 101};
        float lRange[] = {0, 256};
        float abRange[] = {0, 256};
        // float abRange[] = {-128, 128};
        const float* rangesL[] = {lRange};
        const float* rangesAB[] = {abRange};

        cv::calcHist(&imageLab, 1, channelL, mask, histL, 1, histSize, rangesL);
        cv::Scalar sumHistL = cv::sum(histL);
        histL /= sumHistL(0) + (sumHistL(0) == 0);
        histL = histL.reshape(0, 1);

        cv::calcHist(&imageLab, 1, channelA, mask, histA, 1, histSize, rangesAB);
        cv::Scalar sumHistA = cv::sum(histA);
        histA /= sumHistA(0) + (sumHistA(0) == 0);
        histA = histA.reshape(0, 1);

        cv::calcHist(&imageLab, 1, channelB, mask, histB, 1, histSize, rangesAB);
        cv::Scalar sumHistB = cv::sum(histB);
        histB /= sumHistB(0) + (sumHistB(0) == 0);
        histB = histB.reshape(0, 1);

        std::vector<cv::Mat> histVec = {histL, histA, histB};
        cv::hconcat(histVec, colorHistLABDescriptor);
    }

    void Region::computeHOG(int ncells, int nbins, int patchSize) {
        cv::Mat patch = computeRegionOfInterest(patchSize);
        std::vector<float> descriptors;
        int cellSize = static_cast<int>(patchSize / sqrt(ncells));
        cv::HOGDescriptor hogd(cv::Size(patchSize, patchSize), cv::Size(patchSize, patchSize), cv::Size(cellSize, cellSize), cv::Size(cellSize, cellSize), nbins);
        hogd.compute(patch, descriptors);

        // std::cout << "Cell size: " << cellSize << std::endl;
        // std::cout << "Patch size: " << patch.rows << " x " << patch.cols << std::endl;
        // std::cout << "HOG descriptor size: " << descriptors.size() << " / " << ncells * nbins << std::endl << std::endl;

        // Make bins sum 1
        // cv::Mat unnormalizedDescriptor(1, ncells * nbins, CV_32FC1, &descriptors[0]);
        // cv::Scalar sumDesc = cv::sum(unnormalizedDescriptor);
        // this->HOGDescriptor = unnormalizedDescriptor / (sumDesc(0) + (sumDesc(0) == 0));
        
        this->HOGDescriptor = cv::Mat(1, ncells * nbins, CV_32FC1, &descriptors[0]);
    }

    cv::Mat Region::computeRegionOfInterest(int patchSize) {
        // After delimiting the superpixel, extract a patch around its center
        // If the patch is not inside the image, displace the rectangle so it fits in it
        // even if it's not around the center of the superpixel anymore (HOG needs the size
        // to be fixed)
        int xmin = static_cast<int>(boundaries.x + (boundaries.width - patchSize)/2);
        xmin = xmin < 0 ? 0 : xmin;
        // int xmax = static_cast<int>(boundaries.x + (boundaries.width + patchSize)/2 + 1);
        // xmax = xmax > frameSuperpixelLabels.cols ? frameSuperpixelLabels.cols : xmax;
        if(xmin + patchSize > frameSuperpixelLabels.cols) {
            xmin = frameSuperpixelLabels.cols - patchSize;
        }

        int ymin = static_cast<int>(boundaries.y + (boundaries.height - patchSize)/2);
        ymin = ymin < 0 ? 0 : ymin;
        // int ymax = static_cast<int>(boundaries.y + (boundaries.height + patchSize)/2 + 1);
        // ymax = ymax > frameSuperpixelLabels.rows ? frameSuperpixelLabels.rows : ymax;
        if(ymin + patchSize > frameSuperpixelLabels.rows) {
            ymin = frameSuperpixelLabels.rows - patchSize;
        }

        cv::Rect patchDelimiter;
        patchDelimiter.x = xmin;
        patchDelimiter.y = ymin;
        // patchDelimiter.width = xmax - xmin;
        // patchDelimiter.height = ymax - ymin;
        patchDelimiter.width = patchSize;
        patchDelimiter.height = patchSize;

        cv::Mat patch(image, patchDelimiter);
        return patch;
    }

    void Region::computeRelativeDistance() {
        float centerX = (boundaries.x + boundaries.width)/2;
        float centerY = (boundaries.y + boundaries.height)/2;

        // Normalize the position of the center of the superpixel with respect to the center of the image
        // That way, the center of the image is at (0.5, 0.5)
        float relativeCenterX = centerX / image.cols;
        float relativeCenterY = centerY / image.rows;

        std::vector<float> relativePosition = {relativeCenterX, relativeCenterY};
        this->relativeDistanceDescriptor = cv::Mat(1, 2, CV_32FC1, &relativePosition[0]);
    }

    cv::Mat Region::getDescriptor() const {
        std::vector<cv::Mat> descriptorVec = {colorHistBGRDescriptor, colorHistLABDescriptor, HOGDescriptor, relativeDistanceDescriptor};
        // std::vector<cv::Mat> descriptorVec = {colorHistBGRDescriptor, relativeDistanceDescriptor};
        cv::Mat descriptorMat;
        cv::hconcat(descriptorVec, descriptorMat);
        return descriptorMat;
    }
}