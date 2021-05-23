
#ifndef TFG_VIDEO_SEGMENTATION_TRACK_H
#define TFG_VIDEO_SEGMENTATION_TRACK_H

#include <opencv2/core.hpp>


namespace tfg {

class Track {
private:
    std::vector<cv::Vec2f> coordinates;
    std::vector<cv::Vec3b> colors;
    unsigned int initFrame;
    unsigned int number;
    int label;

public:

    Track();
    Track(const std::vector<cv::Vec2f> &coordinates, const unsigned int initFrame);

    ~Track();

    void addPoint(const cv::Vec2f &point);
    void addPoint(const cv::Vec2f &point, const cv::Vec3b &color);

    void obtainColors(const std::vector<cv::Mat> &sequence);

    inline const std::vector<cv::Vec2f>& getPoints() const {
        return coordinates;
    };

    inline const std::vector<cv::Vec3b>& getColors() const {
        return colors;
    };

    inline cv::Vec2f getLastPoint() const {
        return coordinates.back();
    };
    
    inline unsigned int getInitFrame() const {
        return initFrame;
    };

    inline unsigned int getDuration() const {
        return coordinates.size();
    };

    inline unsigned int getNumber() const {
        return number;
    }

    inline int getLabel() const {
        return label;
    }

    void setNumber(unsigned int number);
    void setLabel(int label);

    float distance2(const Track &trackB, const std::vector<float> &flowVariances) const;
    float averageSpatialDistance(const Track &trackB) const;
    float maximalSpatialDistance(const Track &trackB) const;
    float averageColorDistance(const Track &trackB) const;
    float maximalMotionDistance(const Track &trackB, const std::vector<float> &flowVariances) const;
    void deriveForwardDifferences(unsigned int frame, cv::Vec2f &derivative) const;
};

} // namespace tfg

#endif // TFG_VIDEO_SEGMENTATION_TRACK_H
