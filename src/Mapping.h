
#ifndef TFG_VIDEO_SEGMENTATION_MAPPING_H
#define TFG_VIDEO_SEGMENTATION_MAPPING_H

#include <opencv4/opencv2/core.hpp>

namespace tfg {

    class Mapping {
    private:
        std::vector<cv::Vec2f> origin;
        std::vector<cv::Vec2f> destination;
        std::vector<unsigned int> trajectory;

    public:
        Mapping();
        Mapping(const std::vector<cv::Vec2f> &origin, const std::vector<cv::Vec2f> &destination, const std::vector<unsigned int> &trajectory);

        ~Mapping();

        void addPoint(cv::Vec2f &p0, cv::Vec2f &p1, unsigned int label);

        inline std::vector<cv::Vec2f> getOrigin() const {
            return origin;
        };

        inline std::vector<cv::Vec2f> getDestination() const {
            return destination;
        };

        inline std::vector<unsigned int> getTrajectories() const {
            return trajectory;
        };
        
    };
}

#endif //TFG_VIDEO_SEGMENTATION_MAPPING_H