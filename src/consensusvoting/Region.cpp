#include <iostream>
#include <chrono>
#include <opencv4/opencv2/imgproc/imgproc.hpp>
#include <opencv4/opencv2/objdetect/objdetect.hpp>
#include <cmath>
#include "Region.h"

namespace tfg {

    Region::Region() {}
    Region::~Region() {}

    /**
     * Construct a Region object and attribute it a frame, a number inside the frame, the original full image, and the
     * superpixel labeling of it (so that the created Region is the one with the label equal to "number").
     * Then, compute its descriptors.
     * @param number The label of the superpixel inside the full image.
     * @param frame The frame number.
     * @param image The original image.
     * @param frameSuperpixelLabels The superpixel labeling of the full image.
     */
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
        computeRelativeCoordinates();
        // std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
        // std::cout << "Descriptor of superpixel " << number << " in frame " << frame << " computed in " << (std::chrono::duration_cast<std::chrono::microseconds>(end-begin).count())/1000000.0 << " seconds." << '\n';
    }

    /**
     * Compute the rectangle with minimal area such that all the region's pixels are inside it.
     */
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

    /**
     * Compute a color histogram using BGR as the color space.
     * @param nbins Number of bins of the histogram.
     */
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

    /**
     * Compute a color histogram using L*a*b* as the color space.
     * @param nbins Number of bins of the histogram.
     */
    void Region::computeColorHistogramLAB(int nbins) {
        colorHistLABDescriptor.release();

        cv::Mat imageFloat;
        image.convertTo(imageFloat, CV_32FC3, 1.0/255.0);

        cv::Mat imageLab;
        cv::cvtColor(imageFloat, imageLab, cv::COLOR_BGR2Lab);
        cv::Mat mask(frameSuperpixelLabels == number);

        cv::Mat histL, histA, histB;
        int channelL[] = {0};
        int channelA[] = {1};
        int channelB[] = {2};
        int histSize[] = {nbins};

        // OpenCV converts the Lab values to range [0..255] in 8 bit images
        float lRange[] = {0, 101};
        // float lRange[] = {0, 256};
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

    /**
     * Compute histogram of oriented gradients around a region of interest.
     * @param ncells The number of cells for the HOG.
     * @param nbins The number of bins for the HOG.
     * @param patchSize The size of the patch where to compute the HOG (the region of interest).
     */
    void Region::computeHOG(int ncells, int nbins, int patchSize) {
        this->HOGDescriptor.release();

        cv::Mat patch = computeRegionOfInterest(patchSize);
        std::vector<float> descriptors;
        const int cellSize = static_cast<int>(patchSize / sqrt(ncells));
        cv::HOGDescriptor hogd(cv::Size(patchSize, patchSize), cv::Size(patchSize, patchSize), cv::Size(cellSize, cellSize), cv::Size(cellSize, cellSize), nbins);
        hogd.compute(patch, descriptors);

        // std::cout << "Cell size: " << cellSize << '\n';
        // std::cout << "Patch size: " << patch.rows << " x " << patch.cols << '\n';
        // std::cout << "HOG descriptor size: " << descriptors.size() << " / " << ncells * nbins << '\n' << '\n';

        // Make bins sum 1
        // cv::Mat unnormalizedDescriptor(1, ncells * nbins, CV_32FC1, &descriptors[0]);
        // cv::Scalar sumDesc = cv::sum(unnormalizedDescriptor);
        // this->HOGDescriptor = unnormalizedDescriptor / (sumDesc(0) + (sumDesc(0) == 0));
        
        // this->HOGDescriptor = cv::Mat(1, ncells * nbins, CV_32FC1, descriptors.data());
        cv::Mat HOGMatrix(1, ncells * nbins, CV_32FC1, descriptors.data());
        std::vector<cv::Mat> HOGVector = {HOGMatrix};
        cv::hconcat(HOGVector, HOGDescriptor);
    }

    /**
     * Get a portion of matrix corresponding to a patch with certain size around the region.
     * This method tries for the patch to be around the center of the superpixel, but prioritizes
     * keeping the size of the patch the same, so sometimes (i. e. near the edges of the image) 
     * the region of interest is slightly displaced to preserve the patch size.
     */
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
        patchDelimiter.width = patchSize;
        patchDelimiter.height = patchSize;

        cv::Mat patch(image, patchDelimiter);
        return patch;
    }

    /**
     * Get the relative spatial coordinates of the region, calculated using its center.
     * The relative coordinates take values between 0 and 1, such that the center of the image
     * is at (0.5, 0.5).
     */
    void Region::computeRelativeCoordinates() {
        this->relativeCoordinatesDescriptor.release();

        const float centerX = (boundaries.x + boundaries.width)/2;
        const float centerY = (boundaries.y + boundaries.height)/2;

        // Normalize the position of the center of the superpixel with respect to the center of the image
        // That way, the center of the image is at (0.5, 0.5)
        const float relativeCenterX = centerX / image.cols;
        const float relativeCenterY = centerY / image.rows;

        std::vector<float> relativePosition = {relativeCenterX, relativeCenterY};
        cv::Mat relativePositionMat(1, 2, CV_32FC1, relativePosition.data());
        std::vector<cv::Mat> relativePositionVector = {relativePositionMat};
        cv::hconcat(relativePositionVector, relativeCoordinatesDescriptor);
    }

    /**
     * Concatenate the descriptors of the region into a single vector.
     */
    cv::Mat Region::getDescriptor() const {
        std::vector<cv::Mat> descriptorVec = {colorHistBGRDescriptor, colorHistLABDescriptor, HOGDescriptor, relativeCoordinatesDescriptor};
        // std::vector<cv::Mat> descriptorVec = {colorHistBGRDescriptor, colorHistLABDescriptor};
        cv::Mat descriptorMat;
        cv::hconcat(descriptorVec, descriptorMat);
        return descriptorMat;
    }
}