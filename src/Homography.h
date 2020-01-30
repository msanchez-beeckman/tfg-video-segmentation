
#ifndef TFG_VIDEO_SEGMENTATION_HOMOGRAPHY_H
#define TFG_VIDEO_SEGMENTATION_HOMOGRAPHY_H

#include "Track.h"

namespace tfg {
    std::vector<float> getResidualSq(std::vector<libUSTG::laMatrix> &homographies, std::vector<Track> &tracks);
    void getWeights(std::vector<float> &residuals, float tau, std::vector<float> &W);
    float getTotalCost(std::vector<float> &residuals, float tau);

    void printMatrix(libUSTG::laMatrix &matrix);
    void printVector(libUSTG::laVector &vector);
    void printVector(std::vector<float> &vector);

    int compute_ransac_WLS(float *x0, float *y0, float *x1, float *y1, int n, std::vector<unsigned int> &trajectories, std::vector<float> &weights, int niter, float tolerance, libUSTG::laMatrix &H, int *accorded);
    void computeHomography_WLS(float *x0, float *y0, float *x1, float *y1, int n, std::vector<unsigned int> &trajectories, std::vector<float> &weights, libUSTG::laMatrix &H);
    void IRLS(std::vector<Mapping> &mappings, std::vector<Track> &tracks, std::vector<libUSTG::laMatrix> &homographies, std::vector<float> &weights);

    void writeWeights(std::ofstream &file, std::vector<float> &weights);
}

#endif //TFG_VIDEO_SEGMENTATION_HOMOGRAPHY_H
