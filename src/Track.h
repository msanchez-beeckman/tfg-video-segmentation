
#ifndef TFG_VIDEO_SEGMENTATION_TRACK_H
#define TFG_VIDEO_SEGMENTATION_TRACK_H

#include <opencv4/opencv2/core.hpp>

namespace tfg {

    class Track {
    private:
        std::vector<cv::Vec2f> coordinates;
        unsigned int initFrame;
        unsigned int duration;
        unsigned int number;
        int label;

    public:

        Track();
        Track(const std::vector<cv::Vec2f> &coordinates, const unsigned int initFrame);

        ~Track();

        inline std::vector<cv::Vec2f> getPoints() const {
            return coordinates;
        };
        
        inline unsigned int getInitFrame() const {
            return initFrame;
        };

        inline unsigned int getDuration() const {
            return duration;
        };

        inline unsigned int getNumber() const {
            return number;
        }

        inline int getLabel() const {
            return label;
        }

        void setNumber(unsigned int number);
        void setLabel(int label);

        float maximalMotionDistance2(const Track &trackB, const std::vector<float> &flowVariances) const;
        void deriveForwardDifferences(unsigned int frame, cv::Vec2f &derivative) const;
        float averageSpatialDistance(const Track &trackB) const;
    };
}

#endif //TFG_VIDEO_SEGMENTATION_TRACK_H
