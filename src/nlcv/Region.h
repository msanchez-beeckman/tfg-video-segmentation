#ifndef TFG_VIDEO_SEGMENTATION_REGION_H
#define TFG_VIDEO_SEGMENTATION_REGION_H

#include <iostream>

#include <opencv2/core.hpp>


namespace tfg {

class Region {
private:
    int number;
    int frame;
    cv::Mat image;
    cv::Mat frameSuperpixelLabels;
    cv::Rect boundaries;

    cv::Mat colorHistBGRDescriptor;
    cv::Mat colorHistLABDescriptor;
    cv::Mat HOGDescriptor;
    cv::Mat relativeCoordinatesDescriptor;

    void computeSuperpixelBoundaries();
    void computeColorHistogramBGR(int nbins);
    void computeColorHistogramLAB(int nbins);
    void computeHOG(int ncells, int nbins, int patchSize);
    cv::Mat computeRegionOfInterest(int patchSize);
    void computeRelativeCoordinates();

public:
    Region();
    Region(int number, int frame, const cv::Mat &image, const cv::Mat &frameSuperpixelLabels);
    ~Region();

    cv::Mat getDescriptor() const;

    inline int getNumber() const {
        return number;
    };

    inline unsigned int getFrame() const {
        return frame;
    };

    inline cv::Mat getFrameLabels() const {
        return frameSuperpixelLabels;
    };

    inline void printCoordDescriptor() const {
        std::cout << relativeCoordinatesDescriptor << '\n';
    };
};

} // namespace tfg

#endif // TFG_VIDEO_SEGMENTATION_REGION_H