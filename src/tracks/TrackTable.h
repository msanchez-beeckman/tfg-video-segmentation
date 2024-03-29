/*
 * Copyright (c) 2020-2021, Marco Sánchez Beeckman
 * All rights reserved.
 *
 * This source code is licensed under the BSD-style license found in the
 * LICENSE file in the root directory of this source tree.
 */

#ifndef TFG_VIDEO_SEGMENTATION_TRACKTABLE_H
#define TFG_VIDEO_SEGMENTATION_TRACKTABLE_H

#include <fstream>
#include <vector>

#include <opencv2/core.hpp>

#include "Mapping.h"
#include "Track.h"


namespace tfg {

class TrackTable {
private:
    std::vector<tfg::Track> tracks;
    std::vector<tfg::Mapping> mappings;

    std::vector<cv::Vec2f> flowMeans;
    std::vector<float> flowVariances;

    void readTracks(std::ifstream &file, int minDuration);
    void readTracksBrox(std::ifstream &file, int minDuration);

    void computeFlowStatistics();

public:
    TrackTable();
    ~TrackTable();

    TrackTable(std::vector<tfg::Track> &tracks);

    void buildFromFile(std::ifstream &file, int minDuration);
    void buildFromBroxFile(std::ifstream &file, int minDuration);

    void addColorInfo(const std::vector<cv::Mat> &sequence);

    void writeTracks(std::ofstream &file, int minDuration=2);

    void initializeFromPreviousTable(const tfg::TrackTable &previousTable);
    void addTrack(const tfg::Track &track);
    void addPointToTrack(const cv::Vec2f &point, unsigned int track);
    void addPointToTrack(const cv::Vec2f &point, const cv::Vec3b &color, unsigned int track);

    void getMappingsFromTracks();

    void sortTracksByLabel();
    void sortTracksByNumber();
    unsigned int indexOfFirstLabel();

    void printMappings() const;

    void paintWeightedTracks(const std::vector<float> &weights, std::vector<cv::Mat> images, const std::string &folder, const std::string &fileName, int minDuration=2, int firstNameIndex=0) const;
    void paintLabeledTracks(std::vector<cv::Mat> images, const std::string &folder, const std::string &fileName, int firstNameIndex=0) const;

    inline unsigned int numberOfFrames() const {
        return mappings.size();
    };

    inline unsigned int numberOfTracks() const {
        return tracks.size();
    };

    inline unsigned int firstFrameOfTrack(unsigned int track) const {
        return tracks[track].getInitFrame();
    };

    inline unsigned int durationOfTrack(unsigned int track) const {
        return tracks[track].getDuration();
    };

    inline const std::vector<cv::Vec2f>& pointsInTrack(unsigned int track) const {
        return tracks[track].getPoints();
    };

    inline std::vector<cv::Vec2f> originPointsInFrame(unsigned int frame) const {
        return mappings[frame].getOrigin();
    };

    inline std::vector<cv::Vec2f> destinationPointsInFrame(unsigned int frame) const {
        return mappings[frame].getDestination();
    };

    inline std::vector<cv::Vec2f> destinationPointsInLastFrame() const {
        return mappings.back().getDestination();
    };

    inline std::vector<unsigned int> trajectoriesInFrame(unsigned int frame) const {
        return mappings[frame].getTrajectories();
    };

    inline int labelOfTrack(unsigned int track) const {
        return tracks[track].getLabel();
    };

    void setLabelToTrack(int label, unsigned int track);

    inline int numberOfTrack(unsigned int track) const {
        return tracks[track].getNumber();
    };

    inline float distance2BetweenTracks(unsigned int A, unsigned int B) const {
        return tracks[A].distance2(tracks[B], flowVariances);
    };

    inline cv::Vec2f flowMeanOfFrame(unsigned int frame) const {
        return flowMeans[frame];
    };
    
};

} // namespace tfg

#endif // TFG_VIDEO_SEGMENTATION_TRACKTABLE_H