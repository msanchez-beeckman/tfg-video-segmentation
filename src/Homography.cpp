#include <chrono>
#include <limits>
#include <random>
#include <array>
#include <opencv4/opencv2/core.hpp>
#include "Homography.h"

namespace tfg {

    std::vector<float> getWeights2(std::vector<float> &residuals2, float tau) {
        std::vector<float> weights2(residuals2.size());
        for(unsigned int i = 0; i < residuals2.size(); i++) {
            float boundedResidual2 = residuals2[i] < tau*tau ? residuals2[i] : tau*tau;
            weights2[i] = 1 - boundedResidual2/(tau*tau);
        }
        return weights2;
    }

    void printVector(std::vector<float> &vector) {
        for(unsigned int i = 0; i < vector.size(); i++) {
            std::cout << vector[i] << std::endl;
        }
        std::cout << std::endl;
    }

    void computeHomographyRANSAC(const std::vector<cv::Vec2f> &p0, const std::vector<cv::Vec2f> &p1, int n, int niter, float tolerance, cv::Matx33f &H, std::vector<int> &inliers) {
        
        float tolerance2 = tolerance * tolerance;
        unsigned int maxInliers = 0;
        std::vector<int> bestInliers;

        std::array<int, 4> randomIndices;
        std::random_device randomDevice;
        std::mt19937 mersenneTwister(randomDevice());
        std::uniform_int_distribution<int> uniformIntDist(0, n-1);

        for(int iter = 0; iter < niter; iter++) {

            // Sample 4 different numbers between 0 and n-1
            for(int i = 0; i < 4; i++) {
                int randomNumber = uniformIntDist(mersenneTwister);
                while(std::find(randomIndices.begin(), randomIndices.end(), randomNumber) != randomIndices.end()) {
                    randomNumber = uniformIntDist(mersenneTwister);
                }
                randomIndices[i] = randomNumber;
            }

            // Obtain the 4 points corresponding to the random indices
            std::vector<cv::Vec2f> pointsL(4); 
            std::vector<cv::Vec2f> pointsR(4);
            for(int i = 0; i < 4; i++) {
                pointsL[i] = p0[randomIndices[i]];
                pointsR[i] = p1[randomIndices[i]];
            }

            // Compute homography using Least Squares method
            cv::Matx33f Haux;
            std::vector<unsigned int> zeros(4, 0);
            std::vector<float> ones(1, 1.0f);
            tfg::computeHomographyWLS(pointsL, pointsR, 4, zeros, ones, Haux);

            // Obtain the inliers for this iteration
            std::vector<int> iterationInliers;
            iterationInliers.reserve(n);
            for(int i = 0; i < n; i++) {
                cv::Vec3f pointOrigin;
                cv::Vec3f pointDestination;
                pointOrigin(0) = p0[i](0);      pointOrigin(1) = p0[i](1);      pointOrigin(2) = 1.0f;
                pointDestination(0) = p1[i](0); pointDestination(1) = p1[i](1); pointDestination(2) = 1.0f;

                cv::Vec3f pred = Haux * pointOrigin;

                if(pred(2) == 0.0f) continue;

                pred(0) = pred(0)/pred(2); pred(1) = pred(1)/pred(2); pred(2) = 1.0f;
                float reprojectionError2 = cv::norm(pointDestination - pred, cv::NORM_L2SQR);
                if(reprojectionError2 < tolerance2) {
                    iterationInliers.push_back(i);
                }
            }

            // If the number of inliers has increased from the best iteration, update it along with the homography
            const unsigned int numInliers = iterationInliers.size();
            if(numInliers > maxInliers) {
                maxInliers = numInliers;
                bestInliers.swap(iterationInliers);

                H(0,0) = Haux(0,0); H(0,1) = Haux(0,1); H(0,2) = Haux(0,2);
                H(1,0) = Haux(1,0); H(1,1) = Haux(1,1); H(1,2) = Haux(1,2);
                H(2,0) = Haux(2,0); H(2,1) = Haux(2,1); H(2,2) = Haux(2,2);
            }

        }
        inliers.swap(bestInliers);
    }

    void computeHomographyWLS(std::vector<cv::Vec2f> &p0, std::vector<cv::Vec2f> &p1, int n, std::vector<unsigned int> &trajectories, std::vector<float> &weights2, cv::Matx33f &H) {
        
        // Normalize the left and right observations
        std::vector<cv::Vec2f> pnorm0;
        std::vector<cv::Vec2f> pnorm1;
        cv::Vec2f centerL(0, 0), centerR(0, 0);
        cv::Vec2f scaleL(0, 0), scaleR(0, 0);
        tfg::isotropicNormalization(p0, pnorm0, centerL, scaleL);
        tfg::isotropicNormalization(p1, pnorm1, centerR, scaleR);

        ////////////////////////// Minimization problem || Ah || /////////////////////////////

        cv::Matx33d Tpl, Timinv;

        // Similarity transformation of the plane
        Tpl(0, 0) = scaleL(0); Tpl(0, 1) = 0.0;       Tpl(0, 2) = -scaleL(0)*centerL(0);
        Tpl(1, 0) = 0.0;       Tpl(1, 1) = scaleL(1); Tpl(1, 2) = -scaleL(1)*centerL(1);
        Tpl(2, 0) = 0.0;       Tpl(2, 1) = 0.0;       Tpl(2, 2) = 1.0;

        // Inverse similarity transformation of the image
        Timinv(0, 0) = 1.0/scaleR(0); Timinv(0, 1) = 0.0;           Timinv(0, 2) = centerR(0);
        Timinv(1, 0) = 0.0;           Timinv(1, 1) = 1.0/scaleR(1); Timinv(1, 2) = centerR(1);
        Timinv(2, 0) = 0.0;           Timinv(2, 1) = 0.0;           Timinv(2, 2) = 1.0;

        // Build At*Wt*W*A
        double AtA[9][9];
        cv::Mat matAtA(9, 9, CV_64FC1, &AtA[0][0]);
        matAtA.setTo(0);
        for(int i = 0; i < n; i++) {
            float	xpl = pnorm0[i](0), ypl = pnorm0[i](1),
                    xim = pnorm1[i](0), yim = pnorm1[i](1);

            float row1[] = {0.0f, 0.0f, 0.0f, -xpl, -ypl, -1.0f, yim * xpl, yim * ypl, yim};
            float row2[] = {xpl, ypl, 1.0f, 0.0f, 0.0f, 0.0f, -xim * xpl, -xim * ypl, -xim};
            float w2 = weights2[trajectories[i]];

            // Upper half
            for(int j = 0; j < 9; j++) {
                for(int k = j; k < 9; k++) {
                    AtA[j][k] += w2 * row1[j] * row1[k] + w2 * row2[j] * row2[k];
                }
            }
        }
        // Complete lower half
        cv::completeSymm(matAtA);

        // Compute eigenvalues and eigenvectors
        cv::Mat eigvals;
        float eigvects[9][9];
        cv::Mat matEigvects(9, 9, CV_64FC1, &eigvects[0][0]);
        cv::eigen(matAtA, eigvals, matEigvects);

        // Form a matrix formed by the entries of the eigenvector corresponding to the smallest eigenvalue
        cv::Matx33d V(eigvects[8]);

        // Denormalize H = Timinv * V.col(imin)* Tpl;
        cv::Matx33d Vdenorm = Timinv * (V * Tpl);

        // Divide every entry by H(2, 2) so the bottom right entry is 1
        for(int i = 0; i < 3; i++) {
            for(int j = 0; j < 3; j++) {
                H(i, j) = (float) Vdenorm(i, j)/Vdenorm(2, 2);
            }
        }
    }

    void isotropicNormalization(const std::vector<cv::Vec2f> &points, std::vector<cv::Vec2f> &normalizedPoints, cv::Vec2f &center, cv::Vec2f &scale) {
        const unsigned int NUMBER_OF_POINTS = points.size();
        normalizedPoints.clear();
        normalizedPoints.reserve(NUMBER_OF_POINTS);

        // Compute baricenter of the observations
        center(0) = 0.0f; center(1) = 0.0f;
        for(int i = 0; i < NUMBER_OF_POINTS; i++) {
            center = center + points[i];
        }
        center = center / NUMBER_OF_POINTS;

        // Scaling so that the average distance from the center is sqrt(2)
        scale(0) = 0.0f; scale(1) = 0.0f;
        for(int i = 0; i < NUMBER_OF_POINTS; i++) {
            const float obsDistance = cv::norm(points[i] - center);
            scale(0) = scale(0) + obsDistance;
            scale(1) = scale(1) + obsDistance;
        }
        scale(0) = (sqrtf(2.0f) * NUMBER_OF_POINTS) / scale(0);
        scale(1) = (sqrtf(2.0f) * NUMBER_OF_POINTS) / scale(1);

        // Normalize the observations
        for(int i = 0; i < NUMBER_OF_POINTS; i++) {
            normalizedPoints[i](0) = (points[i](0) - center(0)) * scale(0);
            normalizedPoints[i](1) = (points[i](1) - center(1)) * scale(1);
        }
    }

    void IRLS(std::shared_ptr<tfg::MotionModel> &model, std::unique_ptr<tfg::TrackTable> &trackTable, std::vector<float> &weights2) {
        std::cout << "First homography of the initial model:" << std::endl;
        model->printHomography(0);

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
            refinedModel->fitFromWeights(trackTable, initialWeights2, 4);

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
}
