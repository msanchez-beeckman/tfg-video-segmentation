#include <opencv4/opencv2/imgproc/imgproc.hpp>
#include <opencv4/opencv2/objdetect/objdetect.hpp>
#include <cmath>
#include "Region.h"

namespace tfg {

    Region::Region() {}
    Region::~Region() {}

    Region::Region(unsigned int number, unsigned int frame, const cv::Mat &image, const cv::Mat &mask) {
        this->number = number;
        this->frame = frame;
        this->image = image;
        this->mask = mask;

        computeSuperpixelBoundaries();

        computeColorHistogramBGR(20);
        computeColorHistogramLAB(20);
        computeHOG(9, 6, 15);
        computeRelativeDistance();
    }

    void Region::computeSuperpixelBoundaries() {
        int xmax = 0;
        int xmin = mask.cols - 1;
        int ymax = 0;
        int ymin = mask.rows - 1;
        for(int x = 0; x < mask.cols; x++) {
            for(int y = 0; y < mask.rows; y++) {
                unsigned char value = mask.at<unsigned char>(y, x);
                if(value == 0) continue;
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

        cv::Mat histB, histG, histR;
        int channelB[] = {0};
        int channelG[] = {1};
        int channelR[] = {2};
        int histSize[] = {nbins};
        float bgrRange[] = {0, 256};
        const float* ranges[] = {bgrRange};

        cv::calcHist(&imageROI, 1, channelB, maskROI, histB, 1, histSize, ranges);
        int sumHistB = cv::sum(histB);
        histB /= sumHistB + (sumHistB == 0);

        cv::calcHist(&imageROI, 1, channelG, maskROI, histG, 1, histSize, ranges);
        int sumHistG = cv::sum(histG);
        histG /= sumHistG + (sumHistG == 0);

        cv::calcHist(&imageROI, 1, channelR, maskROI, histR, 1, histSize, ranges);
        int sumHistR = cv::sum(histR);
        histR /= sumHistR + (sumHistR == 0);

        std::vector<cv::Mat> histVec = {histB, histG, histR};
        cv::vconcat(histVec, colorHistBGRDescriptor);
    }

    void Region::computeColorHistogramLAB(int nbins) {
        colorHistLABDescriptor.release();

        cv::Mat imageLab;
        cv::cvtColor(imageROI, imageLab, cv::COLOR_BGR2Lab);

        cv::Mat histL, histA, histB;
        int channelL[] = {0};
        int channelA[] = {1};
        int channelB[] = {2};
        int histSize[] = {nbins};
        float lRange[] = {0, 101};
        float abRange[] = {-128, 128};
        const float* rangesL[] = {lRange};
        const float* rangesAB[] = {abRange};

        cv::calcHist(&imageLab, 1, channelL, maskROI, histL, 1, histSize, rangesL);
        int sumHistL = cv::sum(histL);
        histL /= sumHistL + (sumHistL == 0);

        cv::calcHist(&imageLab, 1, channelA, maskROI, histA, 1, histSize, rangesAB);
        int sumHistA = cv::sum(histA);
        histA /= sumHistA + (sumHistA == 0);

        cv::calcHist(&imageLab, 1, channelB, maskROI, histB, 1, histSize, rangesAB);
        int sumHistB = cv::sum(histB);
        histB /= sumHistB + (sumHistB == 0);

        std::vector<cv::Mat> histVec = {histL, histA, histB};
        cv::vconcat(histVec, colorHistLABDescriptor);
    }

    void Region::computeHOG(int ncells, int nbins, int patchSize) {
        cv::Mat patch = computeRegionOfInterest(patchSize);
        std::vector<float> descriptors;
        int cellSize = static_cast<int>(patchSize / sqrt(ncells));
        cv::HOGDescriptor hogd(cv::Size(patchSize, patchSize), cv::Size(patchSize, patchSize), cv::Size(cellSize, cellSize), cv::Size(cellSize, cellSize), nbins);
        hogd.compute(patch, descriptors);

        this->HOGDescriptor = cv::Mat(ncells * nbins, 1, CV_32FC1, &descriptors[0]);
    }

    cv::Mat Region::computeRegionOfInterest(int patchSize) {
        // After delimiting the superpixel, extract a patch around its center
        // Make sure that the patch is inside the image
        int xmin = static_cast<int>(boundaries.x + (boundaries.width - patchSize)/2);
        xmin = xmin < 0 ? 0 : xmin;
        int xmax = static_cast<int>(boundaries.x + (boundaries.width + patchSize)/2 + 1);
        xmax = xmax > mask.cols ? mask.cols : xmax;

        int ymin = static_cast<int>(boundaries.y + (boundaries.height - patchSize)/2);
        ymin = ymin < 0 ? 0 : ymin;
        int ymax = static_cast<int>(boundaries.y + (boundaries.height + patchSize)/2 + 1);
        ymax = ymax > mask.rows ? mask.rows : ymax;

        cv::Rect patchDelimiter;
        patchDelimiter.x = xmin;
        patchDelimiter.y = ymin;
        patchDelimiter.width = xmax - xmin;
        patchDelimiter.height = ymax - ymin;

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
        this->relativeDistanceDescriptor = cv::Mat(2, 1, CV_32FC1, &relativePosition[0]);
    }

    cv::Mat Region::getDescriptor() const {
        std::vector<cv::Mat> descriptorVec = {colorHistBGRDescriptor, colorHistLABDescriptor, HOGDescriptor, relativeDistanceDescriptor};
        cv::Mat descriptorMat;
        cv::vconcat(descriptorVec, descriptorMat);
        return descriptorMat;
    }
}