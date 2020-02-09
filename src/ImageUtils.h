
#ifndef TFG_VIDEO_SEGMENTATION_IMAGEUTILS_H
#define TFG_VIDEO_SEGMENTATION_IMAGEUTILS_H

#include "Track.h"

namespace tfg {
    // void paintTracks(std::vector<tfg::Track> &tracks, std::vector<float> &weights, std::vector<libUSTG::cflimage> &iImages);

    bool point_in_image(int x, int y, int w, int h);
    int val_coord(int i, int w);

    // void readImages(std::ifstream &file, std::vector<libUSTG::cflimage> &iImages);
}

#endif //TFG_VIDEO_SEGMENTATION_IMAGEUTILS_H
