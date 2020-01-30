#include <armadillo>
#include <chrono>
#include "Homography.h"

namespace tfg {
    std::vector<float> getResidualSq(std::vector<libUSTG::laMatrix> &homographies, std::vector<Track> &tracks) {
        std::vector<float> residuals;
        residuals.reserve(tracks.size());

        for(unsigned int t = 0; t < tracks.size(); t++) {
            const std::vector<float> coordinates_x = tracks[t].getPoints_x();
            const std::vector<float> coordinates_y = tracks[t].getPoints_y();

            float maxReprojectionError = 0.0f;
            for(unsigned int i = 0; i < coordinates_x.size() - 1; i++) {
                libUSTG::laVector p_start(3);
                libUSTG::laVector p_end(3);
                p_start[0] = coordinates_x[i]; p_start[1] = coordinates_y[i]; p_start[2] = 1;
                p_end[0] = coordinates_x[i + 1]; p_end[1] = coordinates_y[i + 1]; p_end[2] = 1;


                libUSTG::laVector pred_end = (homographies[tracks[t].getInitFrame() + i])*p_start;
                libUSTG::laVector diff(3);
                for(int i=0; i<3; i++) {
                    diff[i] = pred_end[i] - p_end[i];
                }
                float reprojectionError = sqrt(diff[0]*diff[0] + diff[1]*diff[1] + diff[2]*diff[2]);

                if(reprojectionError > maxReprojectionError) maxReprojectionError = reprojectionError;
            }

            residuals.push_back(maxReprojectionError*maxReprojectionError);
        }
        return residuals;
    }

    void getWeights(std::vector<float> &residuals, float tau, std::vector<float> &W) {
        for(unsigned int i = 0; i < residuals.size(); i++) {
            float boundedResidualSq = residuals[i] < tau*tau ? residuals[i] : tau*tau;
            // W[i] = sqrt(1 - boundedResidualSq/(tau*tau)) > 0.05f ? sqrt(1 - boundedResidualSq/(tau*tau)) : 0.05f;
            W[i] = sqrt(1 - boundedResidualSq/(tau*tau));
        }
    }

    float getTotalCost(std::vector<float> &residualSq, float tau) {
        float totalCost = 0.0f;
        for(unsigned int i = 0; i < residualSq.size(); i++) {
            float trackCost = tau*tau < residualSq[i] ? tau*tau/4 : (residualSq[i]/2)*(1-residualSq[i]/(2*tau*tau));
            totalCost = totalCost + trackCost;
        }

        return totalCost;
    }

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

    int compute_ransac_WLS(float *x0, float *y0, float *x1, float *y1, int n, std::vector<unsigned int> &trajectories, std::vector<float> &weights, int niter, float tolerance, libUSTG::laMatrix &H, int *accorded)
    {


        // Initialize seed
        srand48( (long int) time (NULL) + (long int) getpid() );
        float tolerance2 = tolerance * tolerance;


        H.create(3,3);
        libUSTG::laMatrix Haux(3,3);



        int *paccorded = new int[n];
        int naccorded = 0;
        int pnaccorded = 0;

        for(int iter = 0; iter < niter; iter++)
        {


            // Choose 4 indexos from 1..n without repeated values
            int indexos[4];
            int acceptable = 0;
            while (!acceptable)
            {
                acceptable = 1;
                for(int i=0; i < 4; i++) indexos[i] = (int)  floor(drand48() * (float) n);

                // Check if indexos are repeated
                for(int i=0; i < 4; i++)
                    for(int j=i+1; j < 4; j++)
                        if (indexos[i] == indexos[j]) acceptable = 0;
            }





            // Store selected matches
            float px0[4] , py0[4], px1[4] , py1[4];
            std::vector<unsigned int> trj(4);
            for(int i=0; i < 4; i++)
            {
                px0[i] = x0[indexos[i]];
                py0[i] = y0[indexos[i]];
                trj[i] = trajectories[indexos[i]];

                px1[i] = x1[indexos[i]];
                py1[i] = y1[indexos[i]];
            }


            // Compute planar homography

            computeHomography_WLS(px0, py0, px1, py1, 4, trj, weights, Haux);


            // Which matches are according to this transformation
            pnaccorded = 0;
            for(int i=0; i < n; i++)
            {

                libUSTG::laVector vec(3), res(3);
                vec[0] = x0[i];
                vec[1] = y0[i];
                vec[2] = 1.0f;

                res = Haux * vec;

                if (res[2] != 0.0f) {

                    res[0] /= res[2]; res[1] /= res[2];

                    float dif = (res[0] - x1[i]) * (res[0] - x1[i]) + (res[1] - y1[i]) * (res[1] - y1[i]);

                    if (dif < tolerance2) {  paccorded[pnaccorded] = i; pnaccorded++; }

                }

            }


            // printf("%d: %d (%d) \n", iter, pnaccorded, naccorded);

            // if more according points --> save positions
            if (pnaccorded > naccorded)
            {

                naccorded = pnaccorded;

                if (accorded != NULL)
                    for(int i=0; i < naccorded; i++) accorded[i] = paccorded[i];

                H = Haux;


            }

        }


        delete[] paccorded;
        return naccorded;

    }

    void computeHomography_WLS(float *x0, float *y0, float *x1, float *y1, int n, std::vector<unsigned int> &trajectories, std::vector<float> &weights, libUSTG::laMatrix &H)
    {

        ////////////////// Compute Baricenter
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


        spl = sqrtf(2.0f) / spl;
        spr = sqrtf(2.0f) / spr;

        for (int i = 0; i < n; i++) {

            px0[i] *= spl;
            py0[i] *= spl;

            px1[i] *= spr;
            py1[i] *= spr;

        }


        ////////////////////////// Minimization problem || Ah || /////////////////////////////

        libUSTG::laMatrix Tpl(3, 3), Timinv(3, 3);
       /* arma::mat Tpl(3, 3);
        arma::mat Timinv(3, 3);*/

        // similarity transformation of the plane
        Tpl[0][0] = spl; Tpl[0][1] = 0.0; Tpl[0][2] = -spl*lx;
        Tpl[1][0] = 0.0; Tpl[1][1] = spl; Tpl[1][2] = -spl*ly;
        Tpl[2][0] = 0.0; Tpl[2][1] = 0.0; Tpl[2][2] = 1.0;


        // inverse similarity transformation of the image
        Timinv[0][0] = 1.0f/spr; Timinv[0][1] =   0.0  ; Timinv[0][2] = rx;
        Timinv[1][0] =   0.0  ; Timinv[1][1] = 1.0f/spr; Timinv[1][2] = ry;
        Timinv[2][0] =   0.0  ; Timinv[2][1] =   0.0  ; Timinv[2][2] = 1.0;

        // similarity transformation of the plane
        /*Tpl(0, 0) = spl; Tpl(0, 1) = 0.0; Tpl(0, 2) = -spl*lx;
        Tpl(1, 0) = 0.0; Tpl(1, 1) = spl; Tpl(1, 2) = -spl*ly;
        Tpl(2, 0) = 0.0; Tpl(2, 1) = 0.0; Tpl(2, 2) = 1.0;


        // inverse similarity transformation of the image
        Timinv(0, 0) = 1.0f/spr; Timinv(0, 1) =   0.0  ; Timinv(0, 2) = rx;
        Timinv(1, 0) =   0.0  ; Timinv(1, 1) = 1.0f/spr; Timinv(1, 2) = ry;
        Timinv(2, 0) =   0.0  ; Timinv(2, 1) =   0.0  ; Timinv(2, 2) = 1.0;*/

        ///////////////// System matrix  ///////////////////////

        libUSTG::laMatrix A(2*n, 9);
        // arma::mat A(2*n, 9);


        for(int i = 0, eq = 0; i < n; i++, eq++) {

            float	xpl = px0[i], ypl = py0[i],
                    xim = px1[i], yim = py1[i];

            A[eq][0] = A[eq][1] = A[eq][2] = 0.0;
            A[eq][3] = -xpl * weights[trajectories[i]];
            A[eq][4] = -ypl * weights[trajectories[i]];
            A[eq][5] = -1.0f * weights[trajectories[i]];
            A[eq][6] =  yim * xpl * weights[trajectories[i]];
            A[eq][7] =  yim * ypl * weights[trajectories[i]];
            A[eq][8] =  yim * weights[trajectories[i]];

            eq++;

            A[eq][0] =  xpl * weights[trajectories[i]];
            A[eq][1] =  ypl * weights[trajectories[i]];
            A[eq][2] =  1.0f * weights[trajectories[i]];
            A[eq][3] = A[eq][4] = A[eq][5] = 0.0;
            A[eq][6] = -xim * xpl * weights[trajectories[i]];
            A[eq][7] = -xim * ypl * weights[trajectories[i]];
            A[eq][8] = -xim * weights[trajectories[i]];
        }

        /*for(int i = 0, eq = 0; i < n; i++, eq++) {

            float	xpl = px0[i], ypl = py0[i],
                    xim = px1[i], yim = py1[i];

            A(eq, 0) = A(eq, 1) = A(eq, 2) = 0.0;
            A(eq, 3) = -xpl * weights[trajectories[i]];
            A(eq, 4) = -ypl * weights[trajectories[i]];
            A(eq, 5) = -1.0f * weights[trajectories[i]];
            A(eq, 6) =  yim * xpl * weights[trajectories[i]];
            A(eq, 7) =  yim * ypl * weights[trajectories[i]];
            A(eq, 8) =  yim * weights[trajectories[i]];

            eq++;

            A(eq, 0) =  xpl * weights[trajectories[i]];
            A(eq, 1) =  ypl * weights[trajectories[i]];
            A(eq, 2) =  1.0f * weights[trajectories[i]];
            A(eq, 3) = A(eq, 4) = A(eq, 5) = 0.0;
            A(eq, 6) = -xim * xpl * weights[trajectories[i]];
            A(eq, 7) = -xim * ypl * weights[trajectories[i]];
            A(eq, 8) = -xim * weights[trajectories[i]];
        }*/


        ///////////////// SVD  /////////////////
        /* Previous svd, it does not work for 2*n > 9, but it works for 4 points

         laMatrix U_original(2*n, 9), V_original(9, 9);
         laVector W_original(9);
         compute_svd(A, U_original, V_original, W_original);
         // Find the index of the least singular value
         int imin = 0;
         for (int i = 1; i <  9; i++)
         if ( W_original[i] < W_original[imin] ) imin = i;
         */

        libUSTG::laMatrix U(2*n, 2*n), V(9, 9);
        libUSTG::laVector W(std::min(2*n, 9));
        // arma::mat U(2*n, 2*n), V(9, 9);
        // arma::vec S(std::min(2*n, 9));

        libUSTG::SVD(A, U, W, V);
        // arma::svd(U, S, V, A, "dc");

        ////////////////// Denormalize H = Timinv * V.col(imin)* Tpl;

        libUSTG::laMatrix H_hat(3, 3), H_result(3, 3);
        // arma::mat H_hat(3, 3), H_result(3, 3);
        int k = 0;
        for(int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++, k++) {
                //H_hat(i, j) = V(k, 8); //If svd is sorted the result is always last column
                H_hat[i][j] = V[k][8];
            }
        }


        /* Previous svd, it does not work for 2*n > 9, but it works for 4 points
         int k = 0;
         for(int i = 0; i < 3; i++)
         for(int j = 0; j < 3; j++, k++)
         matrix[i][j] = V_original[k][imin];

         */


        //Undo normalization
        H_result = Timinv * H_hat;
        H_result = H_result * Tpl;

        for(int i = 0; i < 3; i++) {
            for(int j = 0; j < 3; j++) {
                // H[i][j] = H_result(i, j);
                H[i][j] = H_result[i][j];
            }
        }


        delete[] px0;
        delete[] py0;
        delete[] px1;
        delete[] py1;
    }

    void IRLS(std::vector<Mapping> &mappings, std::vector<Track> &tracks, std::vector<libUSTG::laMatrix> &homographies, std::vector<float> &weights) {
        float s = 1.0f;
        unsigned int k = 0;

        std::vector<float> residualSq = tfg::getResidualSq(homographies, tracks);
        tfg::getWeights(residualSq, 4, weights);
        float cost = tfg::getTotalCost(residualSq, 4);
        // float cost = 4*tfg::Track::count;
        std::cout << "Initial cost: " << cost << std::endl;

        bool converged = false;

        // Problem: update each homography at the same time, or let an homography converge and move on to the next one?
        // Option 1: -Not sure if it guarantees convergence, but weights are updated meaningfully.
        //           -Maybe needs one k for each homography? Then a flag is needed for each homography to indicate convergence
        // Option 2: -Individually guarantees convergence, but weights depend on other homographies that do not change.
        //           -Since max is used to compute residuals, there is a chance weights are not updated due to large residuals in other frames.


        // Problem: the algorithm right now assumes that the first iteration of the WLS is an improvement over RANSAC
        // If the cost of the first WLS is greater than RANSAC's, then weightsTmp = weights = getWeights(H0, T), so the weights
        // do not update and the algorithm gets stuck in an infinite loop.
        unsigned int numIter = 0;
        while(numIter < 20 && !converged) {
            std::vector<float> weightsTmp(tracks.size());
            std::vector<float> residualSq_lastStep = tfg::getResidualSq(homographies, tracks);
            tfg::getWeights(residualSq_lastStep, 4, weightsTmp);

            // weightsTmp = s*weightsTmp + (1-s)*weights;
            for(unsigned int i = 0; i < weights.size(); i++) {
                weightsTmp[i] = s*weightsTmp[i] + (1-s)*weights[i];
            }

            std::vector<libUSTG::laMatrix> homographiesTmp;
            for(unsigned int f = 0; f < mappings.size(); f++) {
                libUSTG::laMatrix HTmp(3, 3);
                homographiesTmp.push_back(HTmp);

                std::vector<float> originX, originY, destinationX, destinationY;
                std::vector<unsigned int> trajectories;
                mappings[f].getFilteredMappingItems(originX, originY, destinationX, destinationY, trajectories, weightsTmp);

                std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
                /*tfg::computeHomography_WLS(&(originX[0]), &(originY[0]),
                                                  &(destinationX[0]), &(destinationY[0]), originX.size(), trajectories, weightsTmp, homographiesTmp[f]);*/
                tfg::compute_ransac_WLS(&(originX[0]), &(originY[0]),
                                        &(destinationX[0]), &(destinationY[0]),
                                        originX.size(), trajectories, weightsTmp,
                                        50, 0.1f, homographiesTmp[f], nullptr);
                homographiesTmp[f] = homographiesTmp[f]/homographiesTmp[f][2][2];
                std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
                std::cout << "(" << k + 1 << ") Homography " << f << ": " << (std::chrono::duration_cast<std::chrono::microseconds>(end-begin).count())/1000000.0 << " seconds" << std::endl;
                if(f == 0) {
                    // tfg::printMatrix(homographies[f]);
                    tfg::printMatrix(homographiesTmp[f]);
                }
            }

            std::vector<float> residualSq_nextStep = tfg::getResidualSq(homographiesTmp, tracks);
            float costTmp = tfg::getTotalCost(residualSq_nextStep, 4);
            std::cout << "Iteration cost: " << costTmp << std::endl;
            std::cout << "Cost margin: " << std::abs((cost - costTmp) / cost) << std::endl;
            if(costTmp < cost) {
                if(std::abs((cost - costTmp) / cost) < 0.03) converged = true;
                cost = costTmp;
                k = k + 1;
                homographiesTmp.swap(homographies);
                weightsTmp.swap(weights);
                s = 4*s < 1 ? 4*s : 1;
            } else {
                s = s/4;
            }
            numIter++;
        }
    }

    void writeWeights(std::ofstream &file, std::vector<float> &weights) {
        for(float weight : weights) {
            file << weight << std::endl;
        }
    }
}
