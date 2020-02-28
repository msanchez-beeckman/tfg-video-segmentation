
#ifndef TFG_VIDEO_SEGMENTATION_REGION_H
#define TFG_VIDEO_SEGMENTATION_REGION_H

#include <opencv4/opencv2/core.hpp>

namespace tfg {
    class Region {
        private:
            unsigned int number;
            unsigned int frame;
            cv::Mat image;
            cv::Mat mask;
            cv::Rect boundaries;

            cv::Mat colorHistBGRDescriptor;
            cv::Mat colorHistLABDescriptor;
            cv::Mat HOGDescriptor;
            cv::Mat relativeDistanceDescriptor;

            void computeSuperpixelBoundaries();
            void computeColorHistogramBGR(int nbins);
            void computeColorHistogramLAB(int nbins);
            void computeHOG(int ncells, int nbins, int patchSize);
            cv::Mat computeRegionOfInterest(int patchSize);
            void computeRelativeDistance();

        public:
            Region();
            Region(unsigned int number, unsigned int frame, const cv::Mat &image, const cv::Mat &mask);
            ~Region();

            cv::Mat getDescriptor() const;

            inline unsigned int getNumber() const {
                return number;
            };

            inline unsigned int getFrame() const {
                return frame;
            };
    };
}

#endif //TFG_VIDEO_SEGMENTATION_REGION_H