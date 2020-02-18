#include <iostream>
#include <cmath>
#include <algorithm>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Sparse>
#include <chrono>
#include "RandomWalker.h"

namespace tfg {

    RandomWalker::RandomWalker(std::shared_ptr<tfg::TrackTable> &trackTable) {
        this->trackTable = trackTable;
    }

    RandomWalker::~RandomWalker() {}

    void RandomWalker::seed(const std::unordered_map<int, cv::Mat> &seedImages) {
        std::cout << "Seeding tracks" << std::endl;

        for(auto& [ frame, image ] : seedImages) {
            for(unsigned int t = 0; t < trackTable->numberOfTracks(); t++) {
                const int trackLabel = trackTable->labelOfTrack(t);
                if(trackLabel == -2) continue;
                if(frame < trackTable->firstFrameOfTrack(t) || frame > trackTable->firstFrameOfTrack(t) + trackTable->durationOfTrack(t) - 1) continue;

                const cv::Vec2f position = trackTable->pointsInTrack(t)[frame - trackTable->firstFrameOfTrack(t)];
                const int posX = static_cast<int>(position(0));
                const int posY = static_cast<int>(position(1));
                const cv::Vec3b& color = image.at<cv::Vec3b>(posY, posX);

                int label = color[2]/255 + 2*(color[1]/255) + 4*(color[0]/255) - 1;
                int labelWithValidityCheck = trackLabel < 0 || trackLabel == label ? label : -2;
                trackTable->setLabelToTrack(labelWithValidityCheck, t);
            }
        }
        
        trackTable->sortTracksByLabel();
        this->numberOfLabels = trackTable->labelOfTrack(trackTable->numberOfTracks() - 1) + 1;
        this->unlabeledTracks = trackTable->indexOfFirstLabel();
        this->labeledTracks = trackTable->numberOfTracks() - unlabeledTracks;

        std::cout << "Sorted tracks according to labels" << std::endl;
        std::cout << "There are " << unlabeledTracks << " unlabeled tracks and " << labeledTracks << " labeled tracks" << std::endl;
    }

    void RandomWalker::propagateSeeds() {
        std::cout << "Beginning propagation of seeds" << std::endl;

        Eigen::SparseMatrix<float> laplacianUnlabeled(unlabeledTracks, unlabeledTracks);
        Eigen::SparseMatrix<float> minusBt(unlabeledTracks, labeledTracks);

        std::cout << "Laplacian and -Bt matrices created" << std::endl;

        const float lambda = 0.1f;
        const std::vector<float> ones(trackTable->numberOfFrames(), 1.0f);

        std::cout << "Filling matrices for the system of equations" << std::endl;
        
        std::chrono::steady_clock::time_point beginFill = std::chrono::steady_clock::now();
        std::vector<Eigen::Triplet<float>> laplacianEntries;
        std::vector<Eigen::Triplet<float>> minusBtEntries;
        std::vector<float> unlabeledTracksDegrees(unlabeledTracks, 0.0f);
        laplacianEntries.reserve(unlabeledTracks * unlabeledTracks / 2);
        minusBtEntries.reserve(unlabeledTracks * labeledTracks / 2);
        
        for(unsigned int tA = 0; tA < unlabeledTracks; tA++) {
            for(unsigned int tB = tA + 1; tB < unlabeledTracks + labeledTracks; tB++) {

                const float trackDistance2 = trackTable->distance2BetweenTracks(tA, tB, ones);
                const float weightAB = exp(-lambda * trackDistance2);

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
        for(int k = 0; k < numberOfLabels - 1; k++) {

            Eigen::SparseMatrix<float> M(labeledTracks, 1);
            std::vector<Eigen::Triplet<float>> Mentries;
            Mentries.reserve(labeledTracks);
            for(int i = 0; i < labeledTracks; i++) {
                float labelIsPresent = trackTable->labelOfTrack(i + unlabeledTracks) == k ? 1 : 0;
                if(labelIsPresent != 0) Mentries.push_back(Eigen::Triplet<float>(i, 0, labelIsPresent));
            }
            M.setFromTriplets(Mentries.begin(), Mentries.end());


            Eigen::SparseMatrix<float> minusBtByM = minusBt * M;
            std::cout << "Solving system of equations for label " << k << std::endl;
            std::chrono::steady_clock::time_point beginSolve = std::chrono::steady_clock::now();
            Eigen::ConjugateGradient<Eigen::SparseMatrix<float>, Eigen::Upper> conjGrad;
            conjGrad.compute(laplacianUnlabeled);
            Eigen::VectorXf probabilityLabel = conjGrad.solve(minusBtByM);
            labelProbabilities.push_back(probabilityLabel);
            std::chrono::steady_clock::time_point endSolve = std::chrono::steady_clock::now();
            std::cout << "Solved in " << (std::chrono::duration_cast<std::chrono::microseconds>(endSolve-beginSolve).count())/1000000.0 << " seconds" << std::endl;
        }

        probabilities.clear();
        probabilities.reserve(unlabeledTracks + labeledTracks);
        for(unsigned int t = 0; t < unlabeledTracks; t++) {
            std::vector<float> trackLabelProbabilities;
            trackLabelProbabilities.reserve(numberOfLabels);
            float maxProbability = 0;
            float sumOfProbabilities = 0;
            unsigned int mostLikelyLabel = 0;
            for(unsigned int k = 0; k < numberOfLabels - 1; k++) {
                trackLabelProbabilities.push_back(labelProbabilities[k](t));

                // std::cout << labelProbabilities[k](t) << " ";
                sumOfProbabilities += labelProbabilities[k](t);
                if(labelProbabilities[k](t) > maxProbability) {
                    maxProbability = labelProbabilities[k](t);
                    mostLikelyLabel = k;
                }
            }
            float probabilityOfLastLabel = 1 - sumOfProbabilities;
            // std::cout << probabilityOfLastLabel << std::endl;
            trackLabelProbabilities.push_back(probabilityOfLastLabel);
            if(probabilityOfLastLabel > maxProbability) {
                maxProbability = probabilityOfLastLabel;
                mostLikelyLabel = numberOfLabels - 1;
            }

            probabilities[trackTable->numberOfTrack(t)] = trackLabelProbabilities;
            trackTable->setLabelToTrack(mostLikelyLabel, t);
        }
        for(unsigned int t = unlabeledTracks; t < unlabeledTracks + labeledTracks; t++) {
            std::vector<float> trackLabelProbabilities(numberOfLabels, 0.0f);
            trackLabelProbabilities[trackTable->labelOfTrack(t)] = 1.0f;
            probabilities[trackTable->numberOfTrack(t)] = trackLabelProbabilities;
        }
    }

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