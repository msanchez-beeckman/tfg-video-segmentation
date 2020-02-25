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

        std::vector<cv::Vec2f> flowMeans;
        std::vector<float> flowVariances;

        void readTracks(std::istream &file);
        void readTracksBrox(std::istream &file);
        void getMappingsFromTracks();

        void computeFlowStatistics();
    
    public:
        TrackTable();
        ~TrackTable();

        void buildFromFile(std::istream &file);
        void buildFromBroxFile(std::istream &file);

        void sortTracksByLabel();
        void sortTracksByNumber();
        unsigned int indexOfFirstLabel();

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
        };

        inline int labelOfTrack(unsigned int track) const {
            return tracks[track].getLabel();
        };

        void setLabelToTrack(int label, unsigned int track);

        inline int numberOfTrack(unsigned int track) const {
            return tracks[track].getNumber();
        };

        inline float distance2BetweenTracks(unsigned int A, unsigned int B) const {
            return tracks[A].maximalMotionDistance2(tracks[B], flowVariances);
        };

        inline cv::Vec2f flowMeanOfFrame(unsigned int frame) const {
            return flowMeans[frame];
        };
        

    };
}

#endif //TFG_VIDEO_SEGMENTATION_TRACKTABLE_H