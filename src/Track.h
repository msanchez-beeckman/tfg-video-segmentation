
#ifndef TFG_VIDEO_SEGMENTATION_TRACK_H
#define TFG_VIDEO_SEGMENTATION_TRACK_H

#include <opencv4/opencv2/core.hpp>

namespace tfg {

    class Track {
    private:
        std::vector<cv::Vec2f> coordinates;
        unsigned int initFrame;
        unsigned int duration;

    public:
        static unsigned int count;

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
    };
}

#endif //TFG_VIDEO_SEGMENTATION_TRACK_H
