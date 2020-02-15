#include <boost/algorithm/string.hpp>
#include <string>
#include <iostream>
#include <cmath>
#include <algorithm>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Sparse>
#include <eigen3/Eigen/IterativeLinearSolvers>
#include <chrono>
#include "TrackTable.h"

namespace tfg {

    TrackTable::TrackTable() {}
    TrackTable::~TrackTable() {}

    void TrackTable::buildFromFile(std::istream &file) {
        readTracks(file);
        getMappingsFromTracks();
    }

    void TrackTable::readTracks(std::istream &file) {
        tracks.clear();
        unsigned int trackNumber = 0;
        for(std::string line; std::getline(file, line); ) {
            std::vector<std::string> words;
            boost::split(words, line, boost::is_any_of(" "));
            const unsigned int frameInit = std::stoi(words[0]);

            std::vector<cv::Vec2f> coordinates;
            for(std::vector<std::string>::size_type i = 1; i < words.size(); i = i + 2) {
                const float xCoord = std::stof(words[i]);
                const float yCoord = std::stof(words[i+1]);
                cv::Vec2f point(xCoord, yCoord);
                coordinates.push_back(point);
            }
            tfg::Track track(coordinates, frameInit);
            if(track.getDuration() < 5) continue;
            tracks.push_back(track);
            trackNumber++;
            track.setNumber(trackNumber);
        }
    }

    void TrackTable::getMappingsFromTracks() {
        mappings.clear();
        tfg::Mapping firstMapping;
        mappings.push_back(firstMapping);
        for(std::vector<Track>::size_type t = 0; t < tracks.size(); t++) {
            std::vector<cv::Vec2f> points = tracks[t].getPoints();
            for(unsigned int i = 0; i < points.size() - 1; i++) {
                const unsigned int frame = tracks[t].getInitFrame() + i;
                const std::vector<Mapping>::size_type listSize = mappings.size();

                if(frame > listSize - 1) {
                    Mapping newMapping;
                    newMapping.addPoint(points[i], points[i+1], t);
                    mappings.push_back(newMapping);
                } else {
                    mappings[frame].addPoint(points[i], points[i+1], t);
                }
            }
        }
    }

    void TrackTable::buildFromBroxFile(std::istream &file) {
        readTracksBrox(file);
        getMappingsFromTracks();
    }

    void TrackTable::readTracksBrox(std::istream &file) {
        tracks.clear();
        unsigned int trackNumber = 0;
        std::string line;

        std::getline(file, line);
        const unsigned int NUMBER_OF_FRAMES = std::stoi(line);

        std::getline(file, line);
        const unsigned int NUMBER_OF_TRACKS = std::stoi(line);
        tracks.reserve(NUMBER_OF_TRACKS);

        for(unsigned int t = 0; t < NUMBER_OF_TRACKS; t++) {
            std::getline(file, line);
            std::vector<std::string> words;
            boost::split(words, line, boost::is_any_of(" "));
            const unsigned int TRACK_DURATION = std::stoi(words[1]);
            std::vector<cv::Vec2f> coordinates;
            int frameInit;
            for(unsigned int i = 0; i < TRACK_DURATION; i++) {
                std::getline(file, line);
                boost::split(words, line, boost::is_any_of(" "));
                const float xCoord = std::stof(words[0]);
                const float yCoord = std::stof(words[1]);
                cv::Vec2f point(xCoord, yCoord);
                if(i == 0) frameInit = std::stoi(words[2]);

                coordinates.push_back(point);
            }
            if(TRACK_DURATION < 5) continue;
            Track track(coordinates, frameInit);
            trackNumber++;
            tracks.push_back(track);
            track.setNumber(trackNumber);
        }
    }

    void TrackTable::printMappings() const {
        for(unsigned int f = 0; f < mappings.size(); f++) {
            std::vector<cv::Vec2f> origin = mappings[f].getOrigin();
            std::vector<cv::Vec2f> destination = mappings[f].getDestination();
            std::vector<unsigned int> trajectories = mappings[f].getTrajectories();

            for(unsigned int i = 0; i < origin.size(); i++) {
                std::cout << "Frame " << f << " --> " << f + 1 << " (traj " << trajectories[i] << ")";
                std::cout << ": Point (" << origin[i](0) << ", " << origin[i](1);
                std::cout << ") maps to (" << destination[i](0) << ", " << destination[i](1) << ")" << std::endl;
            }
        }
    }

    void TrackTable::seed(const std::unordered_map<int, cv::Mat> &seedImages) {
        for(auto& [ frame, image ] : seedImages) {
            for(tfg::Track& track : tracks) {
                const int trackLabel = track.getLabel();
                if(trackLabel == -2) continue;
                if(frame < track.getInitFrame() || frame > track.getInitFrame() + track.getDuration() - 1) continue;

                const cv::Vec2f position = track.getPoints()[frame - track.getInitFrame()];
                const int posX = static_cast<int>(position(0));
                const int posY = static_cast<int>(position(1));
                const cv::Vec3b& color = image.at<cv::Vec3b>(posY, posX);

                // std::cout << "Position (" << posX << ", " << posY << ") has an RGB value of (" << (int) color[2] << ", " << (int) color[1] << ", " << (int) color[0] << ")" << std::endl;

                int label = color[2]/255 + 2*(color[1]/255) - 1;
                if(trackLabel >= 0 && trackLabel != label) {
                    track.setLabel(-2);
                    // std::cout << "Inconsistent data: trajectories with two different labels. Ignoring the trajectory." << std::endl;
                } else {
                    track.setLabel(label);
                    // std::cout << "Label: " << label << std::endl;
                }
            }
        }
    }

    void TrackTable::propagateSeedsRandomWalk(std::vector<float> &probabilities) {
        std::cout << "Beginning propagation" << std::endl;

        std::sort(tracks.begin(), tracks.end(), [](const tfg::Track &A, const tfg::Track &B) -> bool {return A.getLabel() < B.getLabel();});
        std::vector<tfg::Track>::iterator firstLabel = std::lower_bound(tracks.begin(), tracks.end(), 0, [](const tfg::Track &A, const int number) -> bool {return A.getLabel() < number;});
        const unsigned int UNLABELED_TRACKS = firstLabel - tracks.begin();
        const unsigned int LABELED_TRACKS = tracks.end() - firstLabel;

        std::cout << "Sorted tracks according to labels" << std::endl;
        std::cout << "There are " << UNLABELED_TRACKS << " unlabeled tracks and " << LABELED_TRACKS << " labeled tracks" << std::endl;

        //Eigen::MatrixXf laplacianUnlabeled(UNLABELED_TRACKS, UNLABELED_TRACKS);
        Eigen::SparseMatrix<float> laplacianUnlabeled(UNLABELED_TRACKS, UNLABELED_TRACKS);
        Eigen::MatrixXf minusBt(UNLABELED_TRACKS, LABELED_TRACKS);

        std::cout << "Laplacian and -Bt matrices created" << std::endl;

        const float lambda = 0.1;
        const std::vector<float> ones(mappings.size(), 1.0f);

        std::cout << "Filling matrices for the system of equations" << std::endl;
        
        std::chrono::steady_clock::time_point beginFill = std::chrono::steady_clock::now();
        std::vector<Eigen::Triplet<float>> laplacianEntries;
        laplacianEntries.reserve(UNLABELED_TRACKS * UNLABELED_TRACKS);
        for(unsigned int tA = 0; tA < UNLABELED_TRACKS; tA++) {
            float degreeTrackA = 0.0f;
            for(unsigned int tB = 0; tB < UNLABELED_TRACKS + LABELED_TRACKS; tB++) {
                if(tB == tA) continue;

                const float trackDistance2 = tracks[tA].maximalMotionDistance2(tracks[tB], ones);
                const float weightAB = exp(-lambda * trackDistance2);

                degreeTrackA += weightAB;

                if(tB < tA) continue;

                if(tB >= UNLABELED_TRACKS) {
                    minusBt(tA, tB - UNLABELED_TRACKS) = weightAB;
                } else {
                    if(weightAB > 1e-8) {
                        laplacianEntries.push_back(Eigen::Triplet<float>(tA, tB, -weightAB));
                        laplacianEntries.push_back(Eigen::Triplet<float>(tB, tA, -weightAB));
                    }
                }
            }
            laplacianEntries.push_back(Eigen::Triplet<float>(tA, tA, degreeTrackA));
        }
        laplacianUnlabeled.setFromTriplets(laplacianEntries.begin(), laplacianEntries.end());
        std::chrono::steady_clock::time_point endFill = std::chrono::steady_clock::now();
        std::cout << "Filled in " << (std::chrono::duration_cast<std::chrono::microseconds>(endFill-beginFill).count())/1000000.0 << " seconds" << std::endl;

        std::cout << "Creating matrix of label presence" << std::endl;
        Eigen::VectorXf M(LABELED_TRACKS);
        for(int i = 0; i < LABELED_TRACKS; i++) {
            M(i) = tracks[i + UNLABELED_TRACKS].getLabel() == 0 ? 1 : 0;
        }

        std::cout << "Multiplying -Bt and M" << std::endl;
        Eigen::VectorXf minusBtByM = minusBt * M;

        std::cout << "Solving system of equations" << std::endl;
        std::chrono::steady_clock::time_point beginSolve = std::chrono::steady_clock::now();
        // Eigen::VectorXf probabilityBackground = laplacianUnlabeled.ldlt().solve(minusBtByM);
        Eigen::ConjugateGradient<Eigen::SparseMatrix<float>, Eigen::Lower|Eigen::Upper> conjGrad;
        // conjGrad.compute(laplacianUnlabeled.selfadjointView<Eigen::Upper>());
        conjGrad.compute(laplacianUnlabeled);
        Eigen::VectorXf probabilityBackground = conjGrad.solve(minusBtByM);
        std::chrono::steady_clock::time_point endSolve = std::chrono::steady_clock::now();
        std::cout << "Solved in " << (std::chrono::duration_cast<std::chrono::microseconds>(endSolve-beginSolve).count())/1000000.0 << " seconds" << std::endl;

        for(unsigned int t = 0; t < UNLABELED_TRACKS; t++) {
            if(probabilityBackground(t) < 0.5) {
                tracks[t].setLabel(1);
            } else {
                tracks[t].setLabel(0);
            }
        }

        // std::cout << probabilityBackground << std::endl;

        std::cout << "Labels updated" << std::endl;
        probabilities.clear();
        probabilities.reserve(UNLABELED_TRACKS + LABELED_TRACKS);
        for(unsigned int t = 0; t < UNLABELED_TRACKS; t++) {
            probabilities[tracks[t].getNumber()] = probabilityBackground(t);
        }
        std::cout << "Put probabilities of unlabeled tracks in vector" << std::endl;
        for(unsigned int t = UNLABELED_TRACKS; t < UNLABELED_TRACKS + LABELED_TRACKS; t++) {
            probabilities[tracks[t].getNumber()] = tracks[t].getLabel() == 0 ? 1 : 0;
        }


    }
}