#include <boost/algorithm/string.hpp>
#include <string>
#include <iostream>
#include "TrackTable.h"

namespace tfg {

    TrackTable::TrackTable() {}
    TrackTable::~TrackTable() {}

    void TrackTable::buildFromFile(std::istream &file) {
        readTracks(file);
        getMappingsFromTracks();
        computeFlowStatistics();
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
            track.setNumber(trackNumber);
            trackNumber++;
            tracks.push_back(track);
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
        computeFlowStatistics();
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
            track.setNumber(trackNumber);
            trackNumber++;
            tracks.push_back(track);
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

    void TrackTable::computeFlowStatistics() {
        flowMeans.clear();
        flowVariances.clear();
        const unsigned int NUMBER_OF_MAPPINGS = mappings.size();
        flowMeans.reserve(NUMBER_OF_MAPPINGS);
        flowVariances.reserve(NUMBER_OF_MAPPINGS);

        for(unsigned int f = 0; f < NUMBER_OF_MAPPINGS; f++) {
            // Compute mean
            cv::Vec2f mean(0, 0);
            std::vector<cv::Vec2f> originPoints = mappings[f].getOrigin();
            std::vector<cv::Vec2f> destinationPoints = mappings[f].getDestination();
            const unsigned int NUMBER_OF_POINTS = originPoints.size();
            for(unsigned int i = 0; i < NUMBER_OF_POINTS; i++) {
                mean += destinationPoints[i] - originPoints[i];
            }
            mean(0) /= NUMBER_OF_POINTS; mean(1) /= NUMBER_OF_POINTS;
            flowMeans.push_back(mean);

            // Compute variance
            float variance = 0.0f;
            for(unsigned int i = 0; i < NUMBER_OF_POINTS; i++) {
                variance += cv::norm(destinationPoints[i] - originPoints[i] - mean, cv::NORM_L2SQR);
            }
            variance /= NUMBER_OF_POINTS;
            flowVariances.push_back(variance);
        }
    }

    void TrackTable::setLabelToTrack(int label, unsigned int track) {
        tracks[track].setLabel(label);
    }

    void TrackTable::sortTracksByLabel() {
        std::sort(tracks.begin(), tracks.end(), [](const tfg::Track &A, const tfg::Track &B) -> bool {return A.getLabel() < B.getLabel();});
    }

    void TrackTable::sortTracksByNumber() {
        std::sort(tracks.begin(), tracks.end(), [](const tfg::Track &A, const tfg::Track &B) -> bool {return A.getNumber() < B.getNumber();});
    }

    unsigned int TrackTable::indexOfFirstLabel() {
        std::vector<tfg::Track>::iterator firstLabel = std::lower_bound(tracks.begin(), tracks.end(), 0, [](const tfg::Track &A, const int number) -> bool {return A.getLabel() < number;});
        return firstLabel - tracks.begin();
    }
}