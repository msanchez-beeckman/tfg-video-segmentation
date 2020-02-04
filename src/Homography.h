
#ifndef TFG_VIDEO_SEGMENTATION_HOMOGRAPHY_H
#define TFG_VIDEO_SEGMENTATION_HOMOGRAPHY_H

#include "Track.h"
#include "MotionModel.h"

namespace tfg {
    std::vector<float> getResidualSq(std::vector<libUSTG::laMatrix> &homographies, std::vector<Track> &tracks);
    std::vector<float> getWeights2(std::vector<float> &residuals, float tau);
    float getTotalCost(std::vector<float> &residuals, float tau);

    void printMatrix(libUSTG::laMatrix &matrix);
    void printVector(libUSTG::laVector &vector);
    void printVector(std::vector<float> &vector);

    void compute_homography_WLS(float *x0, float *y0, float *x1, float *y1, int n, std::vector<unsigned int> &trajectories, std::vector<float> &weights, libUSTG::laMatrix &H);
    // void IRLS(std::vector<Mapping> &mappings, std::vector<Track> &tracks, std::vector<libUSTG::laMatrix> &homographies, std::vector<float> &weights);
    void IRLS(std::shared_ptr<tfg::MotionModel> &model, std::vector<float> &weights2);

    void copyHomography(libUSTG::laMatrix &A, libUSTG::laMatrix &B);

    void writeWeights(std::ofstream &file, std::vector<float> &weights);
}

#endif //TFG_VIDEO_SEGMENTATION_HOMOGRAPHY_H
