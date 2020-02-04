#include <chrono>
#include <limits>
#include <opencv4/opencv2/core.hpp>
#include "Homography.h"

namespace tfg {
    // std::vector<float> getResidualSq(std::vector<libUSTG::laMatrix> &homographies, std::vector<Track> &tracks) {
    //     std::vector<float> residuals;
    //     residuals.reserve(tracks.size());

    //     for(unsigned int t = 0; t < tracks.size(); t++) {
    //         const std::vector<float> coordinates_x = tracks[t].getPoints_x();
    //         const std::vector<float> coordinates_y = tracks[t].getPoints_y();

    //         float maxReprojectionError = 0.0f;
    //         for(unsigned int i = 0; i < coordinates_x.size() - 1; i++) {
    //             libUSTG::laVector p_start(3);
    //             libUSTG::laVector p_end(3);
    //             p_start[0] = coordinates_x[i]; p_start[1] = coordinates_y[i]; p_start[2] = 1;
    //             p_end[0] = coordinates_x[i + 1]; p_end[1] = coordinates_y[i + 1]; p_end[2] = 1;


    //             libUSTG::laVector pred_end = (homographies[tracks[t].getInitFrame() + i])*p_start;
    //             libUSTG::laVector diff(3);
    //             for(int i=0; i<3; i++) {
    //                 diff[i] = pred_end[i] - p_end[i];
    //             }
    //             float reprojectionError = sqrt(diff[0]*diff[0] + diff[1]*diff[1] + diff[2]*diff[2]);

    //             if(reprojectionError > maxReprojectionError) maxReprojectionError = reprojectionError;
    //         }

    //         residuals.push_back(maxReprojectionError*maxReprojectionError);
    //     }
    //     return residuals;
    // }

    std::vector<float> getWeights2(std::vector<float> &residuals2, float tau) {
        std::vector<float> weights2(residuals2.size());
        for(unsigned int i = 0; i < residuals2.size(); i++) {
            float boundedResidual2 = residuals2[i] < tau*tau ? residuals2[i] : tau*tau;
            weights2[i] = 1 - boundedResidual2/(tau*tau);
        }
        return weights2;
    }

    // float getTotalCost(std::vector<float> &residualSq, float tau) {
    //     float totalCost = 0.0f;
    //     for(unsigned int i = 0; i < residualSq.size(); i++) {
    //         float trackCost = tau*tau < residualSq[i] ? tau*tau/4 : (residualSq[i]/2)*(1-residualSq[i]/(2*tau*tau));
    //         totalCost = totalCost + trackCost;
    //     }

    //     return totalCost;
    // }

    void printMatrix(libUSTG::laMatrix &matrix) {
        float **entries = matrix.v();
        for(int i = 0; i < matrix.nrows(); i++) {
            for(int j = 0; j < matrix.ncols(); j++) {
                std::cout << *(*(entries + j) + i) << " ";
            }
            std::cout << std::endl;
        }
        std::cout << std::endl;
    }

    void printVector(libUSTG::laVector &vector) {
        float *entries = vector.v();
        for(int i = 0; i < vector.size(); i++) {
            std::cout << *(entries + i) << std::endl;
        }
        std::cout << std::endl;
    }

    void printVector(std::vector<float> &vector) {
        for(unsigned int i = 0; i < vector.size(); i++) {
            std::cout << vector[i] << std::endl;
        }
        std::cout << std::endl;
    }

    void compute_homography_WLS(float *x0, float *y0, float *x1, float *y1, int n, std::vector<unsigned int> &trajectories, std::vector<float> &weights2, libUSTG::laMatrix &H)
    {

        // Compute Baricenter
        float lx = 0.0,  ly = 0.0;
        float rx = 0.0,  ry = 0.0;

        for (int i = 0; i < n; i++) {

            lx += x0[i];
            ly += y0[i];

            rx += x1[i];
            ry += y1[i];
        }

        lx /= (float) n;
        ly /= (float) n;
        rx /= (float) n;
        ry /= (float) n;


        /////////////// Normalize points without modifying original vectors

        float *px0 = new float[n];
        float *py0 = new float[n];

        float *px1 = new float[n];
        float *py1 = new float[n];

        float spl = 0.0f, spr = 0.0f;
        for(int i = 0; i < n; i++)
        {

            px0[i] = x0[i] - lx;
            py0[i] = y0[i] - ly;

            px1[i] = x1[i] - rx;
            py1[i] = y1[i] - ry;

            spl += sqrtf(px0[i] * px0[i] + py0[i]*py0[i]);
            spr += sqrtf(px1[i] * px1[i] + py1[i]*py1[i]);

        }


        spl = n * sqrtf(2.0f) / spl;
        spr = n * sqrtf(2.0f) / spr;

        for (int i = 0; i < n; i++) {

            px0[i] *= spl;
            py0[i] *= spl;

            px1[i] *= spr;
            py1[i] *= spr;

        }


        ////////////////////////// Minimization problem || Ah || /////////////////////////////

        libUSTG::laMatrix Tpl(3, 3), Timinv(3, 3);

        // similarity transformation of the plane
        Tpl[0][0] = spl; Tpl[0][1] = 0.0; Tpl[0][2] = -spl*lx;
        Tpl[1][0] = 0.0; Tpl[1][1] = spl; Tpl[1][2] = -spl*ly;
        Tpl[2][0] = 0.0; Tpl[2][1] = 0.0; Tpl[2][2] = 1.0;

        // inverse similarity transformation of the image
        Timinv[0][0] = 1.0f/spr; Timinv[0][1] =   0.0  ; Timinv[0][2] = rx;
        Timinv[1][0] =   0.0  ; Timinv[1][1] = 1.0f/spr; Timinv[1][2] = ry;
        Timinv[2][0] =   0.0  ; Timinv[2][1] =   0.0  ; Timinv[2][2] = 1.0;

        ///////////////// System matrix  ///////////////////////
        float AtA[9][9];
        cv::Mat matAtA(9, 9, CV_32FC1, &AtA[0][0]);
        matAtA.setTo(0);

        for(int i = 0; i < n; i++) {

            float	xpl = px0[i], ypl = py0[i],
                    xim = px1[i], yim = py1[i];

            float row1[] = {0.0f, 0.0f, 0.0f, -xpl, -ypl, -1.0f, yim * xpl, yim * ypl, yim};
            float row2[] = {xpl, ypl, 1.0f, 0.0f, 0.0f, 0.0f, -xim * xpl, -xim * ypl, -xim};
            float w2 = weights2[trajectories[i]];

            // Upper half of AtA
            for(int j = 0; j < 9; j++) {
                for(int k = j; k < 9; k++) {
                    AtA[j][k] += w2 * row1[j] * row1[k] + w2 * row2[j] * row2[k];
                }
            }

        }

        // Complete lower half using the fact that AtA is symmetrical
        // for(int j = 0; j < 9; j++) {
        //     for(int k = 0; k < j; k++) {
        //         AtA(j, k) = AtA(k, j);
        //     }
        // }
        cv::completeSymm(matAtA);


        // Eigen::SelfAdjointEigenSolver<Eigen::Matrix<float, 9, 9>> eigenSolver(AtA);
        // Eigen::Matrix<float, 9, 9> V = eigenSolver.eigenvectors();
        // Eigen::Matrix<float, 9, 1> vaps = eigenSolver.eigenvalues();

        cv::Mat eigvals;
        float eigvects[9][9];
        cv::Mat matEigvects(9, 9, CV_32FC1, &eigvects[0][0]);
        cv::eigen(matAtA, eigvals, matEigvects);

        cv::Matx33f V(eigvects[8]);
        // std::cout << "OpenCV matrix: " << std::endl;
        // std::cout << V << std::endl;

        ////////////////// Denormalize H = Timinv * V.col(imin)* Tpl;

        libUSTG::laMatrix H_hat(3, 3), H_result(3, 3);
        // int k = 0;
        // for(int i = 0; i < 3; i++) {
        //     for (int j = 0; j < 3; j++, k++) {
        //         //H_hat(i, j) = V(k, 8); //If svd is sorted the result is always last column
        //         H_hat[i][j] = V(k, 0);
        //     }
        // }
        for(int i = 0; i < 3; i++) {
            for(int j = 0; j < 3; j++) {
                H_hat[i][j] = V(i, j);
            }
        }

        // std::cout << "USTG matrix:" << std::endl;
        // tfg::printMatrix(H_hat);

        // std::cout << std::endl;
        // std::cout << "H_hat: " << std::endl;
        // printMatrix(H_hat);
        // std::cout << std::endl;

        // std::cout << std::endl;
        // std::cout << "Vector: " << std::endl;
        // std::cout << V << std::endl;

        // std::cout << std::endl;
        // std::cout << "Values: " << std::endl;
        // std::cout << vaps << std::endl;

        //Undo normalization
        H_result = Timinv * H_hat;
        H_result = H_result * Tpl;

        for(int i = 0; i < 3; i++) {
            for(int j = 0; j < 3; j++) {
                // H[i][j] = H_result(i, j);
                H[i][j] = H_result[i][j]/H_result[2][2];
            }
        }


        delete[] px0;
        delete[] py0;
        delete[] px1;
        delete[] py1;
    }

    // void IRLS(std::vector<Mapping> &mappings, std::vector<Track> &tracks, std::vector<libUSTG::laMatrix> &homographies, std::vector<float> &weights) {
    //     float s = 1.0f;
    //     unsigned int k = 0;

    //     std::vector<float> residualSq = tfg::getResidualSq(homographies, tracks);
    //     tfg::getWeights2(residualSq, 4, weights);
    //     // float cost = tfg::getTotalCost(residualSq, 4);
    //     float cost = 4*tfg::Track::count;
    //     std::cout << "Initial cost: " << cost << std::endl;

    //     bool converged = false;

    //     // Problem: the algorithm right now assumes that the first iteration of the WLS is an improvement over RANSAC
    //     // If the cost of the first WLS is greater than RANSAC's, then weightsTmp = weights = getWeights2(H0, T), so the weights
    //     // do not update and the algorithm gets stuck in an infinite loop.
    //     unsigned int numIter = 0;
    //     while(numIter < 20 && !converged) {
    //         std::vector<float> weightsTmp(tracks.size());
    //         std::vector<float> residualSq_lastStep = tfg::getResidualSq(homographies, tracks);
    //         tfg::getWeights2(residualSq_lastStep, 4, weightsTmp);

    //         // weightsTmp = s*weightsTmp + (1-s)*weights;
    //         for(unsigned int i = 0; i < weights.size(); i++) {
    //             weightsTmp[i] = s*weightsTmp[i] + (1-s)*weights[i];
    //         }

    //         std::vector<libUSTG::laMatrix> homographiesTmp;
    //         for(unsigned int f = 0; f < mappings.size(); f++) {
    //             libUSTG::laMatrix HTmp(3, 3);
    //             homographiesTmp.push_back(HTmp);

    //             std::vector<float> originX, originY, destinationX, destinationY;
    //             std::vector<unsigned int> trajectories;
    //             mappings[f].getFilteredMappingItems(originX, originY, destinationX, destinationY, trajectories, weightsTmp);

    //             std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    //             tfg::compute_homography_WLS(&(originX[0]), &(originY[0]),
    //                                        &(destinationX[0]), &(destinationY[0]),
    //                                        originX.size(), trajectories, weightsTmp, homographiesTmp[f]);
    //             std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    //             std::cout << "(" << k + 1 << ") Homography " << f << ": " << (std::chrono::duration_cast<std::chrono::microseconds>(end-begin).count())/1000000.0 << " seconds" << std::endl;
    //         }

    //         std::vector<float> residualSq_nextStep = tfg::getResidualSq(homographiesTmp, tracks);
    //         float costTmp = tfg::getTotalCost(residualSq_nextStep, 4);
    //         std::cout << "Iteration cost: " << costTmp << std::endl;
    //         std::cout << "Cost margin: " << std::abs((cost - costTmp) / cost) << std::endl;
    //         if(costTmp < cost) {
    //             if(std::abs((cost - costTmp) / cost) < 0.03) converged = true;
    //             cost = costTmp;
    //             k = k + 1;
    //             // homographiesTmp.swap(homographies);
    //             for(unsigned int h = 0; h < homographiesTmp.size(); h++) {
    //                 copyHomography(homographiesTmp[h], homographies[h]);
    //             }
    //             weightsTmp.swap(weights);
    //             s = 4*s < 1 ? 4*s : 1;
    //         } else {
    //             s = s/4;
    //         }
    //         numIter++;
    //     }
    // }

    void IRLS(std::shared_ptr<tfg::MotionModel> &model, std::vector<float> &weights2) {
        std::cout << "First homography of the initial model:" << std::endl;
        model->printHomography(0);

        std::vector<tfg::Track> tracks = model->getTracks();
        std::vector<tfg::Mapping> mappings = model->getMappings();

        float s = 1.0f;
        unsigned int k = 0;
        float bestCost = -1;

        std::vector<float> initialWeights2(weights2.size(), 1.0f);
        std::vector<float> refinedWeights2(weights2.size(), 1.0f);
        std::vector<float> bestWeights2(weights2.size(), 1.0f);

        bool converged = false;
        unsigned int numIter = 0;
        initialWeights2.swap(weights2);
        std::shared_ptr<tfg::MotionModel> refinedModel;
        while(numIter < 50 && !converged) {
            std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

            refinedModel = std::make_shared<tfg::MotionModel>();
            refinedModel->fitFromWeights(tracks, mappings, initialWeights2, 4);

            std::vector<float> residuals2 = refinedModel->getResiduals2();
            std::vector<float> currentWeights2 = tfg::getWeights2(residuals2, 4);
            for(unsigned int i = 0; i < weights2.size(); i++) {
                refinedWeights2[i] = s*currentWeights2[i] + (1-s)*bestWeights2[i];
            }
            float cost = refinedModel->getCost();

            std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

            std::cout << "(" << k + 1 << ") Iteration cost: " << cost << std::endl;
            std::cout << "(" << k + 1 << ") Cost margin: " << std::abs((cost - bestCost) / bestCost) << std::endl;
            std::cout << "(" << k + 1 << ") Time: " << (std::chrono::duration_cast<std::chrono::microseconds>(end-begin).count())/1000000.0 << " seconds" << std::endl;

            if(cost > bestCost && bestCost >= 0) {
                s = s/4;
            } else {
                if(cost > 0 && std::abs((cost - bestCost) / bestCost) < 0.00001) converged = true;
                bestCost = cost;
                k = k + 1;
                s = 4*s < 1 ? 4*s : 1;
                for(unsigned int i = 0; i < bestWeights2.size(); i++) {
                    bestWeights2[i] = initialWeights2[i];
                }
                model = refinedModel;
            }
            initialWeights2.swap(refinedWeights2);
            numIter++;
        }
        weights2.swap(bestWeights2);
        std::cout << "First homography of the best model:" << std::endl;
        model->printHomography(0);
    }


    void writeWeights(std::ofstream &file, std::vector<float> &weights) {
        for(float weight : weights) {
            file << weight << std::endl;
        }
    }

    void copyHomography(libUSTG::laMatrix &A, libUSTG::laMatrix &B) {
        for(int i = 0; i < B.nrows(); i++) {
            for(int j = 0; j < B.ncols(); j++) {
                B[i][j] = A[i][j];
            }
        }
    }
}
