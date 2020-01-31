#include "Track.h"

namespace tfg {
    unsigned int Track::count = 0;

    Track::Track() {
        ++Track::count;
    }

    Track::Track(const pairList<float> &coordinates, const unsigned int initFrame) {
        ++count;
        this->coordinates = coordinates;
        this->initFrame = initFrame;
        this->duration = coordinates.first.size();
    }

    Track::~Track() {
        --Track::count;
    }

    Mapping::Mapping() {}
    Mapping::Mapping(pairList<float> origin, pairList<float> destination, std::vector<unsigned int> trajectory) {
        this->origin = origin;
        this->destination = destination;
        this->trajectory = trajectory;
    }

    Mapping::~Mapping() {}

    void Mapping::addPoint(float x0, float y0, float x1, float y1, unsigned int label) {
        origin.first.push_back(x0);
        origin.second.push_back(y0);
        destination.first.push_back(x1);
        destination.second.push_back(y1);
        trajectory.push_back(label);
    }

    void Mapping::getFilteredMappingItems(std::vector<float> &f_origin_x, std::vector<float> &f_origin_y,
                                          std::vector<float> &f_destination_x, std::vector<float> &f_destination_y, std::vector<unsigned int> &f_trajectories,
                                          std::vector<float> &weights) const {

        f_origin_x.clear();
        f_origin_y.clear();
        f_destination_x.clear();
        f_destination_y.clear();
        f_trajectories.clear();

        for(unsigned int i = 0; i < trajectory.size(); i++) {
            if(weights[trajectory[i]] < 0.05) continue;

            f_origin_x.push_back(origin.first[i]);
            f_origin_y.push_back(origin.second[i]);
            f_destination_x.push_back(destination.first[i]);
            f_destination_y.push_back(destination.second[i]);
            f_trajectories.push_back(trajectory[i]);
        }
    }

    void readTracks(std::ifstream &file, std::vector<Track> &tracks) {
        for(std::string line; std::getline(file, line); ) {
            std::vector<std::string> words;
            boost::split(words, line, boost::is_any_of(" "));
            const unsigned int frameInit = std::stoi(words[0]);

            pairList<float> coordinates;
            for(std::vector<std::string>::size_type i = 1; i < words.size(); i = i + 2) {
                const int xCoord = std::stof(words[i]);
                const int yCoord = std::stof(words[i+1]);
                coordinates.first.push_back(xCoord);
                coordinates.second.push_back(yCoord);
            }
            Track track(coordinates, frameInit);
            if(track.getDuration() < 5) continue;
            tracks.push_back(track);
        }
    }

    void readTracksBrox(std::ifstream &file, std::vector<Track> &tracks) {
        std::string line;
        std::getline(file, line);
        const unsigned int NUMBER_OF_FRAMES = std::stoi(line);
        std::getline(file, line);
        const unsigned int NUMBER_OF_TRACKS = std::stoi(line);

        for(unsigned int t = 0; t < NUMBER_OF_TRACKS; t++) {
            std::getline(file, line);
            std::vector<std::string> words;
            boost::split(words, line, boost::is_any_of(" "));
            const unsigned int TRACK_DURATION = std::stoi(words[1]);
            pairList<float> coordinates;
            int frameInit;
            for(unsigned int i = 0; i < TRACK_DURATION; i++) {
                std::getline(file, line);
                boost::split(words, line, boost::is_any_of(" "));
                const int xCoord = std::stof(words[0]);
                const int yCoord = std::stof(words[1]);
                if(i == 0) frameInit = std::stoi(words[2]);

                coordinates.first.push_back(xCoord);
                coordinates.second.push_back(yCoord);
            }
            if(TRACK_DURATION < 5) continue;
            Track track(coordinates, frameInit);
            tracks.push_back(track);
        }
    }

    void getMappings(std::vector<Track> &tracks, std::vector<Mapping> &mappings) {
        Mapping firstMapping;
        mappings.push_back(firstMapping);
        for(std::vector<Track>::size_type t = 0; t < tracks.size(); t++) {
            std::vector<float> points_x = tracks[t].getPoints_x();
            std::vector<float> points_y = tracks[t].getPoints_y();
            for(unsigned int i = 0; i < points_x.size() - 1; i++) {
                const unsigned int frame = tracks[t].getInitFrame() + i;
                const std::vector<Mapping>::size_type listSize = mappings.size();

                if(frame > listSize - 1) {
                    Mapping newMapping;
                    newMapping.addPoint(points_x[i], points_y[i], points_x[i+1], points_y[i+1], t);
                    mappings.push_back(newMapping);
                } else {
                    mappings[frame].addPoint(points_x[i], points_y[i], points_x[i+1], points_y[i+1], t);
                }
            }
        }
    }

    void printMappings(std::vector<Mapping> &mappings) {
        for(unsigned int f = 0; f < mappings.size(); f++) {
            std::vector<float> originX = mappings[f].getOrigin_x();
            std::vector<float> originY = mappings[f].getOrigin_y();
            std::vector<float> destinationX = mappings[f].getDestination_x();
            std::vector<float> destinationY = mappings[f].getDestination_y();
            std::vector<unsigned int> trajectories = mappings[f].getTrajectories();

            for(unsigned int i = 0; i < originX.size(); i++) {
                std::cout << "Frame " << f << " --> " << f + 1 << " (traj " << trajectories[i] << ")";
                std::cout << ": Point (" << originX[i] << ", " << originY[i];
                std::cout << ") maps to (" << destinationX[i] << ", " << destinationY[i] << ")" << std::endl;
            }
        }
    }
}