#include <iostream>
#include <cmath>
#include <algorithm>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Sparse>
#include <chrono>
#include "RandomWalker.h"

namespace tfg {

    /**
     * Construct an instance of a RandomWalker object. The random walk is performed over a graph
     * where the nodes are tracks, so a trackTable object is required to initialize the RandomWalker.
     * @param trackTable A trackTable containing information of the tracks where the random walk is going to be performed.
     */
    RandomWalker::RandomWalker(std::shared_ptr<tfg::TrackTable> &trackTable) {
        this->trackTable = trackTable;
    }

    RandomWalker::~RandomWalker() {}

    /**
     * Label nodes (tracks) of the graph as foreground (there can be various foreground labels) or background from a series of seeded images.
     * An initial labeling is needed so that the labels can be propagated to unlabeled nodes.
     * @param seedImages An unordered map of OpenCV Mat objects, whose key is an integer that indicates the frame the seed is for.
     */
    void RandomWalker::seed(const std::unordered_map<int, cv::Mat> &seedImages) {
        std::cout << "Seeding tracks" << std::endl;

        // For each matrix of seeds, label each track passing through a seed with its corresponding tag.
        // If a track passes through two or more different seeds, tag it as unreliable and don't use it for propagation.
        for(auto& [ frame, image ] : seedImages) {
            for(unsigned int t = 0; t < trackTable->numberOfTracks(); t++) {
                const int trackLabel = trackTable->labelOfTrack(t);
                if(trackLabel == -2) continue; // If unreliable, do nothing to the track
                if(frame < trackTable->firstFrameOfTrack(t) || frame > trackTable->firstFrameOfTrack(t) + trackTable->durationOfTrack(t) - 1) continue;

                const cv::Vec2f position = trackTable->pointsInTrack(t)[frame - trackTable->firstFrameOfTrack(t)];
                const int posX = static_cast<int>(position(0));
                const int posY = static_cast<int>(position(1));
                const cv::Vec3b& color = image.at<cv::Vec3b>(posY, posX);

                // The labeling in the images must be done using colors the following way:
                // a BGR value of 0-0-255 (full red) corresponds to the background
                // a BGR value of 0-0-0 (no color) means there is no label
                // another BGR value using 255 for any combination of colors (e. g. 0-255-255) is a different foreground label
                // non-255 values are considered 0.
                int label = color[2]/255 + 2*(color[1]/255) + 4*(color[0]/255) - 1;
                int labelWithValidityCheck = trackLabel < 0 || trackLabel == label ? label : -2; // If a track is already labeled and it passes through a seed with a different tag, mark it as unreliable
                trackTable->setLabelToTrack(labelWithValidityCheck, t);
            }
        }
        
        // Sort tracks by label. The only purpose of this is to separate labeled and unlabeled tracks, so it is easier to mount the
        // system of equations for the propagation
        trackTable->sortTracksByLabel();
        this->numberOfLabels = trackTable->labelOfTrack(trackTable->numberOfTracks() - 1) + 1;
        this->unlabeledTracks = trackTable->indexOfFirstLabel();
        this->labeledTracks = trackTable->numberOfTracks() - unlabeledTracks;

        std::cout << "Sorted tracks according to labels" << std::endl;
        std::cout << "There are " << unlabeledTracks << " unlabeled tracks and " << labeledTracks << " labeled tracks" << std::endl;
    }

    /**
     * Propagate the labels of the graph. This is done by solving a system of equations, specifically
     * the one in eq. 10 in the following paper:
     * 
     * L. Grady.
     * Random Walks for Image Segmentation
     * IEEE Transactions on Pattern Analysis and Machine Intelligence, 28(11): 1768-1783, 2006
     * DOI: 10.1109/TPAMI.2006.233
     */
    void RandomWalker::propagateSeeds() {
        std::cout << "Beginning propagation of seeds" << std::endl;

        Eigen::SparseMatrix<float> laplacianUnlabeled(unlabeledTracks, unlabeledTracks);
        Eigen::SparseMatrix<float> minusBt(unlabeledTracks, labeledTracks);

        std::cout << "Laplacian and -Bt matrices created" << std::endl;

        const float lambda = 0.8f;
        // const std::vector<float> ones(trackTable->numberOfFrames(), 1.0f);

        std::cout << "Filling matrices for the system of equations" << std::endl;
        
        std::chrono::steady_clock::time_point beginFill = std::chrono::steady_clock::now();
        std::vector<Eigen::Triplet<float>> laplacianEntries;
        std::vector<Eigen::Triplet<float>> minusBtEntries;
        std::vector<float> unlabeledTracksDegrees(unlabeledTracks, 0.0f);
        laplacianEntries.reserve(unlabeledTracks * unlabeledTracks / 2);
        minusBtEntries.reserve(unlabeledTracks * labeledTracks / 2);
        
        // Fill the sparse matrices corresponding to L_U and -Bt (minus B transposed) in eq. 7 of Grady's paper
        // L_U is the graph laplacian of the subgraph formed by only the unlabeled tracks
        // B is the matrix of size |V_M| x |V_U| whose entry (i, j) corresponds to minus the weight of the edge from the ith labeled track to the jth unlabeled track
        for(unsigned int tA = 0; tA < unlabeledTracks; tA++) {
            for(unsigned int tB = tA + 1; tB < unlabeledTracks + labeledTracks; tB++) {

                // The weights between affinities are defined from a distance, defined in eq. 2
                // of the following paper:
                // P. Ochs, J. Malik, and T. Brox.
                // Segmentation of moving objects by long term video analysis.
                // IEEE Transactions on Pattern Analysis and Machine Intelligence, 36(6): 1187-1200, Jun 2014.
                // DOI: 10.1109/TPAMI.2013.242
                const float trackDistance2 = trackTable->distance2BetweenTracks(tA, tB);
                const float weightAB = std::exp(-lambda * trackDistance2);

                // If the weight between tracks is almost 0, consider it 0 and do not store it explicitly in the sparse matrix
                if(weightAB < 1e-8) continue;
                unlabeledTracksDegrees[tA] += weightAB;

                if(tB >= unlabeledTracks) {
                    minusBtEntries.push_back(Eigen::Triplet<float>(tA, tB - unlabeledTracks, weightAB));
                } else {
                    unlabeledTracksDegrees[tB] += weightAB;
                    laplacianEntries.push_back(Eigen::Triplet<float>(tA, tB, -weightAB));
                }
            }
            laplacianEntries.push_back(Eigen::Triplet<float>(tA, tA, unlabeledTracksDegrees[tA]));
        }
        laplacianUnlabeled.setFromTriplets(laplacianEntries.begin(), laplacianEntries.end());
        minusBt.setFromTriplets(minusBtEntries.begin(), minusBtEntries.end());
        std::chrono::steady_clock::time_point endFill = std::chrono::steady_clock::now();
        std::cout << "Filled in " << (std::chrono::duration_cast<std::chrono::microseconds>(endFill-beginFill).count())/1000000.0 << " seconds" << std::endl;


        std::cout << "Starting walking process" << std::endl;
        std::vector<Eigen::VectorXf> labelProbabilities;
        labelProbabilities.reserve(numberOfLabels - 1);

        // Solve the system of equations L_U*x = -Bt*m_s given in eq. 10 in Grady's paper for each label except the last one
        // m_s (s for each label) is a vector of length |V_M| that serves as an indicator for each label: a 1 in the ith entry means that
        // the ith labeled node is marked with label s, and a 0 means it is not
        for(int k = 0; k < numberOfLabels - 1; k++) {

            Eigen::SparseMatrix<float> M(labeledTracks, 1);
            std::vector<Eigen::Triplet<float>> Mentries;
            Mentries.reserve(labeledTracks);
            // Create indicator for the label
            for(int i = 0; i < labeledTracks; i++) {
                float labelIsPresent = trackTable->labelOfTrack(i + unlabeledTracks) == k ? 1 : 0;
                if(labelIsPresent != 0) Mentries.push_back(Eigen::Triplet<float>(i, 0, labelIsPresent));
            }
            M.setFromTriplets(Mentries.begin(), Mentries.end());

            // Get product -Bt*m_s, right hand of eq. 10 in Grady's paper
            Eigen::SparseMatrix<float> minusBtByM = minusBt * M;
            std::cout << "Solving system of equations for label " << k << std::endl;
            std::chrono::steady_clock::time_point beginSolve = std::chrono::steady_clock::now();

            // Solve the system of equations using an iterative conjugate gradient algorithm for self-adjoint sparse matrices
            Eigen::ConjugateGradient<Eigen::SparseMatrix<float>, Eigen::Upper> conjGrad;
            conjGrad.compute(laplacianUnlabeled);
            Eigen::VectorXf probabilityLabel = conjGrad.solve(minusBtByM);
            labelProbabilities.push_back(probabilityLabel);
            std::chrono::steady_clock::time_point endSolve = std::chrono::steady_clock::now();
            std::cout << "Solved in " << (std::chrono::duration_cast<std::chrono::microseconds>(endSolve-beginSolve).count())/1000000.0 << " seconds" << std::endl;
        }

        probabilities.clear();
        probabilities.reserve(unlabeledTracks + labeledTracks);

        // For every unlabeled track, save the probabilities for each label
        // and order them the original way
        for(unsigned int t = 0; t < unlabeledTracks; t++) {
            std::vector<float> trackLabelProbabilities;
            trackLabelProbabilities.reserve(numberOfLabels);
            float maxProbability = 0;
            float sumOfProbabilities = 0;
            unsigned int mostLikelyLabel = 0;
            for(unsigned int k = 0; k < numberOfLabels - 1; k++) {
                trackLabelProbabilities.push_back(labelProbabilities[k](t));
                sumOfProbabilities += labelProbabilities[k](t);
                if(labelProbabilities[k](t) > maxProbability) {
                    maxProbability = labelProbabilities[k](t);
                    mostLikelyLabel = k;
                }
            }
            float probabilityOfLastLabel = 1 - sumOfProbabilities;
            trackLabelProbabilities.push_back(probabilityOfLastLabel);
            if(probabilityOfLastLabel > maxProbability) {
                maxProbability = probabilityOfLastLabel;
                mostLikelyLabel = numberOfLabels - 1;
            }

            probabilities[trackTable->numberOfTrack(t)] = trackLabelProbabilities;
            trackTable->setLabelToTrack(mostLikelyLabel, t);
        }
        // Do the same for the labeled tracks
        for(unsigned int t = unlabeledTracks; t < unlabeledTracks + labeledTracks; t++) {
            std::vector<float> trackLabelProbabilities(numberOfLabels, 0.0f);
            trackLabelProbabilities[trackTable->labelOfTrack(t)] = 1.0f;
            probabilities[trackTable->numberOfTrack(t)] = trackLabelProbabilities;
        }
    }

    /**
     * Write in a file the probabilities that each track has for every label.
     * @param file The file where the probabilities are going to be stored.
     */
    void RandomWalker::writeProbabilities(std::ostream &file) {
        file << unlabeledTracks + labeledTracks << " " << numberOfLabels << std::endl;
        for(unsigned int t = 0; t < unlabeledTracks + labeledTracks; t++) {
            std::vector<float>& trackLabelProbabilities = probabilities.at(t);
            for(unsigned int k = 0; k < numberOfLabels - 1; k++) {
                file << trackLabelProbabilities[k] << " ";
            }
            file << trackLabelProbabilities[numberOfLabels - 1] << std::endl;
        }
    }

}