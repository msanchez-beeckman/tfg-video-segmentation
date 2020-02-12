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
    }

    void TrackTable::readTracks(std::istream &file) {
        tracks.clear();
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

    void TrackTable::seed(std::unordered_map<int, cv::Mat> &seedImages) {
        for(auto& [ frame, image ] : seedImages) {
            for(tfg::Track& track : tracks) {
                const int trackLabel = track.getLabel();
                if(trackLabel == -2) continue;
                if(frame < track.getInitFrame() || frame > track.getInitFrame() + track.getDuration() - 1) continue;

                const cv::Vec2f position = track.getPoints()[frame - track.getInitFrame()];
                const int posX = static_cast<int>(position(0));
                const int posY = static_cast<int>(position(1));
                cv::Vec3b& color = image.at<cv::Vec3b>(posY, posX);

                // std::cout << "Position (" << posX << ", " << posY << ") has an RGB value of (" << (int) color[2] << ", " << (int) color[1] << ", " << (int) color[0] << ")" << std::endl;

                int label = color[1]/255 + 2*(color[2]/255) - 1;
                if(trackLabel >= 0 && trackLabel != label) {
                    track.setLabel(-2);
                    std::cout << "Inconsistent data: trajectories with two different labels. Ignoring the trajectory." << std::endl;
                } else {
                    track.setLabel(label);
                    std::cout << "Label: " << label << std::endl;
                }
            }
        }
    }
}