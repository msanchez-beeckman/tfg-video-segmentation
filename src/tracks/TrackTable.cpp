#include <opencv4/opencv2/imgcodecs.hpp>
#include <string>
#include <iostream>
#include "ImageUtils.h"
#include "IoUtils.h"
#include "TrackTable.h"

namespace tfg {

    TrackTable::TrackTable() {}
    TrackTable::~TrackTable() {}

    void TrackTable::buildFromFile(std::ifstream &file, int minDuration) {
        readTracks(file, minDuration);
        getMappingsFromTracks();
        computeFlowStatistics();
    }

    /**
     * Read tracks from a file, using a format where each line represents a track, the first number of a line corresponds to its initial frame,
     * and the next numbers alternate between the x-axis value and the y-axis value of the points in the following frames.
     * @param file The file where the track information is stored.
     * @param minDuration The minimum duration of a track to add it to the table.
     */
    void TrackTable::readTracks(std::ifstream &file, int minDuration) {
        tracks.clear();
        unsigned int trackNumber = 0;
        for(std::string line; std::getline(file, line); ) {
            std::vector<std::string> words;
            // boost::split(words, line, boost::is_any_of(" "));
            tfg::splitString(line, words);
            const unsigned int frameInit = std::stoi(words[0]);

            std::vector<cv::Vec2f> coordinates;
            for(std::vector<std::string>::size_type i = 1; i < words.size(); i = i + 2) {
                const float xCoord = std::stof(words[i]);
                const float yCoord = std::stof(words[i+1]);
                cv::Vec2f point(xCoord, yCoord);
                coordinates.push_back(point);
            }
            tfg::Track track(coordinates, frameInit);
            if(track.getDuration() < minDuration) continue;
            track.setNumber(trackNumber);
            trackNumber++;
            tracks.push_back(track);
        }
    }

    /**
     * Get origin-destination pairs for points in tracks for each frame of a video. This is basically formatting the sequence of tracks
     * in an alternative way to make it easier to manipulate the points that are in a frame.
     */
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

    void TrackTable::buildFromBroxFile(std::ifstream &file, int minDuration) {
        readTracksBrox(file, minDuration);
        getMappingsFromTracks();
        computeFlowStatistics();
    }

    /**
     * Read tracks from a file that uses Brox's codification to store the tracks. The first line of the file stores the number of frames,
     * the second stores the number of tracks, and then follows a sequence for each track: its length, and the positions of the points with their corresponding frame.
     * @param file The file where the track information is stored.
     * @param minDuration The minimum duration of a track to add it to the table.
     */
    void TrackTable::readTracksBrox(std::ifstream &file, int minDuration) {
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
            // boost::split(words, line, boost::is_any_of(" "));
            tfg::splitString(line, words);
            const unsigned int TRACK_DURATION = std::stoi(words[1]);
            std::vector<cv::Vec2f> coordinates;
            int frameInit;
            for(unsigned int i = 0; i < TRACK_DURATION; i++) {
                std::getline(file, line);
                // boost::split(words, line, boost::is_any_of(" "));
                tfg::splitString(line, words);
                const float xCoord = std::stof(words[0]);
                const float yCoord = std::stof(words[1]);
                cv::Vec2f point(xCoord, yCoord);
                if(i == 0) frameInit = std::stoi(words[2]);

                coordinates.push_back(point);
            }
            if(TRACK_DURATION < minDuration) continue;
            Track track(coordinates, frameInit);
            track.setNumber(trackNumber);
            trackNumber++;
            tracks.push_back(track);
        }
    }

    /**
     * Print all the origin-destination correspondences.
     */
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

    /**
     * Paint the tracks over a sequence of images, coloring them according to some weights (red if their weight is > 0.5, red, green otherwise).
     * @param weights2 The squared weights attributed to each track.
     * @param images A sequence of images over which the tracks are going to be painted.
     * @param folder The folder where the output file will be placed.
     * @param fileName The name of the output image.
     */
    void TrackTable::paintWeightedTracks(const std::vector<float> &weights2, std::vector<cv::Mat> images, const std::string &folder, const std::string &fileName) const {
        for(unsigned int t = 0; t < this->numberOfTracks(); t++) {
            std::vector<cv::Vec2f> points = this->pointsInTrack(t);
            unsigned int initFrame = this->firstFrameOfTrack(t);

            for(unsigned int f = 0; f < points.size(); f++) {
                cv::Vec3b color;
                color(2) = weights2[t] < 0.25 ? 0 : 255;
                color(1) = weights2[t] < 0.25 ? 255 : 0;
                // drawPoint(images[initFrame + f], points[f], color);
                const int remainingFrames = points.size() - f - 1;
                const int T = std::min(5, remainingFrames);
                if(T != 0) {
                    tfg::drawLine(images[initFrame + f], points[f], points[f + 1], color);
                } else {
                    tfg::drawPoint(images[initFrame + f], points[f], color);
                }
            }
        }

        for (unsigned int i = 0; i < images.size(); i++) {
            std::stringstream ss;
            ss << folder << fileName << i << ".png";
            std::string saveAs = ss.str();
            cv::imwrite(saveAs, images[i]);
        }
    }

    /**
     * Paint the tracks over a sequence of images, coloring them according to their labels.
     * @param images A sequence of images over which the tracks are going to be painted.
     * @param folder The folder where the output file will be placed.
     * @param fileName The name of the output image.
     */
    void TrackTable::paintLabeledTracks(std::vector<cv::Mat> images, const std::string &folder, const std::string &fileName) const {
        for(unsigned int t = 0; t < this->numberOfTracks(); t++) {
            const int label = this->labelOfTrack(t);
            std::vector<cv::Vec2f> points = this->pointsInTrack(t);
            unsigned int initFrame = this->firstFrameOfTrack(t);

            if(label < 0) continue;

            for(unsigned int f = 0; f < points.size(); f++) {
                cv::Vec3b color;
                color(2) = (label + 1) % 2 == 1 ? 255 : 0;
                color(1) = (label + 1) % 4 >= 2 ? 255 : 0;
                color(0) = (label + 1) % 8 >= 4 ? 255 : 0;
                // tfg::drawPoint(images[initFrame + f], points[f], color);
                const int remainingFrames = points.size() - f - 1;
                const int T = std::min(10, remainingFrames);
                if(T != 0) {
                    tfg::drawLine(images[initFrame + f], points[f], points[f + 1], color);
                } else {
                    tfg::drawPoint(images[initFrame + f], points[f], color);
                }
            }
        }

        for (unsigned int i = 0; i < images.size(); i++) {
            std::stringstream ss;
            ss << folder << fileName << i << ".png";
            std::string saveAs = ss.str();
            cv::imwrite(saveAs, images[i]);
        }
    }

    /**
     * Compute mean and variance of the optical flow.
     */
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