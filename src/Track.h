
#ifndef TFG_VIDEO_SEGMENTATION_TRACK_H
#define TFG_VIDEO_SEGMENTATION_TRACK_H

#include <boost/algorithm/string.hpp>
#include "../library/libImage.h"

namespace tfg {
    template <typename T>
    using pairList = std::pair<std::vector<T>, std::vector<T>>;

    class Track {
    private:
        pairList<float> coordinates;
        unsigned int initFrame;
        unsigned int duration;

    public:
        static unsigned int count;

        Track();
        Track(const pairList<float> &coordinates, const unsigned int initFrame);

        ~Track();

        inline std::vector<float> getPoints_x() const {
            return coordinates.first;
        };
        inline std::vector<float> getPoints_y() const {
            return coordinates.second;
        };
        inline unsigned int getInitFrame() const {
            return initFrame;
        };
        inline unsigned int getDuration() const {
            return duration;
        };
    };

    class Mapping {
    private:
        pairList<float> origin;
        pairList<float> destination;
        std::vector<unsigned int> trajectory;

    public:
        Mapping();
        Mapping(pairList<float> origin, pairList<float> destination, std::vector<unsigned int> trajectory);

        ~Mapping();

        void addPoint(float x0, float y0, float x1, float y1, unsigned int label);

        inline std::vector<float> getOrigin_x() const {
            return origin.first;
        }

        inline std::vector<float> getOrigin_y() const {
            return origin.second;
        }

        inline std::vector<float> getDestination_x() const {
            return destination.first;
        }

        inline std::vector<float> getDestination_y() const {
            return destination.second;
        }

        inline std::vector<unsigned int> getTrajectories() const {
            return trajectory;
        }

        void getFilteredMappingItems(std::vector<float> &f_origin_x, std::vector<float> &f_origin_y,
                                     std::vector<float> &f_destination_x, std::vector<float> &f_destination_y, std::vector<unsigned int> &f_trajectories,
                                     std::vector<float> &weights) const;
    };

    void readTracks(std::ifstream &file, std::vector<Track> &tracks);
    void getMappings(std::vector<Track> &tracks, std::vector<Mapping> &mappings);
    void printMappings(std::vector<Mapping> &mappings);
}

#endif //TFG_VIDEO_SEGMENTATION_TRACK_H
