#include "Track.h"

namespace tfg {
    unsigned int Track::count = 0;

    Track::Track() {
        ++Track::count;
    }

    Track::Track(const std::vector<cv::Vec2f> &coordinates, const unsigned int initFrame) {
        ++count;
        this->coordinates = coordinates;
        this->initFrame = initFrame;
        this->duration = coordinates.size();
    }

    Track::~Track() {
        --Track::count;
    }
}