#include "ImageUtils.h"

namespace tfg {
    void paintTracks(std::vector<tfg::Track> &tracks, std::vector<float> &weights, std::vector<libUSTG::cflimage> &iImages) {
        for(unsigned int t = 0; t < tracks.size(); t++) {
            std::vector<float> points_x = tracks[t].getPoints_x();
            std::vector<float> points_y = tracks[t].getPoints_y();
            unsigned int initFrame = tracks[t].getInitFrame();

            for(unsigned int f = 0; f < points_x.size(); f++) {
                int x = (int) points_x[f];
                int y = (int) points_y[f];
                int w = iImages[initFrame + f].w();
                int h = iImages[initFrame + f].h();

                if(!point_in_image(x, y, w, h)) continue;

                int x_left = val_coord(x-1, w);
                int x_right = val_coord(x+1, w);
                int y_down = val_coord(y-1, h);
                int y_up = val_coord(y+1, h);

                for(int i = x_left; i < x_right + 1; i++) {
                    for(int j = y_down; j < y_up + 1; j++) {
                        int blue = weights[t] < 0.5 ? 255 : iImages[initFrame + f][w*h + j*w + i];
                        int green = weights[t] < 0.5 ? iImages[initFrame + f][2*w*h + j*w + i] : 255;

                        iImages[initFrame + f][w*h + j*w + i] = blue;
                        iImages[initFrame + f][2*w*h + j*w + i] = green;
                    }
                }
            }
        }

        for (unsigned int ii = 0; ii < iImages.size(); ii++) {
            std::stringstream ss;
            ss << "../results/" << "out" << ii << ".png";
            std::string fileName = ss.str();
            iImages[ii].save(fileName.c_str());
        }
    }

    bool point_in_image(int x, int y, int w, int h) {
        return x >= 0 && x < w && y >= 0 && y < h;
    }

    int val_coord(int i, int w) {
        int i_new = std::min(std::max(i, 0), w - 1);
        return i_new;
    }

    void readImages(std::ifstream &file, std::vector<libUSTG::cflimage> &iImages) {
        for(std::string line; std::getline(file, line); ) {
            libUSTG::cflimage image(1,1,1);
            image.load(line.c_str());
            iImages.push_back(image);
        }
    }
}
