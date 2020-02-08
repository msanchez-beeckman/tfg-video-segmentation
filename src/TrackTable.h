#ifndef TFG_VIDEO_SEGMENTATION_TRACKTABLE_H
#define TFG_VIDEO_SEGMENTATION_TRACKTABLE_H

#include <opencv4/opencv2/core.hpp>
#include "Track.h"
#include "Mapping.h"

namespace tfg {

    class TrackTable {
    private:
        std::vector<tfg::Track> tracks;
        std::vector<tfg::Mapping> mappings;

        void readTracks(std::ifstream &file);
        void readTracksBrox(std::ifstream &file);
        void getMappingsFromTracks();
    
    public:
        TrackTable();
        ~TrackTable();

        void buildFromFile(std::ifstream &file);
        void buildFromBroxFile(std::ifstream &file);

        void printMappings() const;

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

        inline std::vector<cv::Vec2f> pointsInTrack(unsigned int track) const {
            return tracks[track].getPoints();
        };

        inline std::vector<cv::Vec2f> originPointsInFrame(unsigned int frame) const {
            return mappings[frame].getOrigin();
        };

        inline std::vector<cv::Vec2f> destinationPointsInFrame(unsigned int frame) const {
            return mappings[frame].getDestination();
        };

        inline std::vector<unsigned int> trajectoriesInFrame(unsigned int frame) const {
            return mappings[frame].getTrajectories();
        }

    };
}

#endif //TFG_VIDEO_SEGMENTATION_TRACKTABLE_H