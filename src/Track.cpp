#include "Track.h"

namespace tfg {
    unsigned int Track::count = 0;

    Track::Track() {
        ++Track::count;
        this->label = -1;
    }

    Track::Track(const std::vector<cv::Vec2f> &coordinates, const unsigned int initFrame) {
        ++Track::count;
        this->coordinates = coordinates;
        this->initFrame = initFrame;
        this->duration = coordinates.size();
        this->label = -1;
    }

    Track::~Track() {
        --Track::count;
    }

    void Track::setLabel(int label) {
        this->label = label;
    }
}