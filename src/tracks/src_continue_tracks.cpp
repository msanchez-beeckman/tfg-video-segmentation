#include <iostream>
#include <fstream>
#include <opencv4/opencv2/core.hpp>
#include <chrono>
#include "ImageUtils.h"
#include "Tracking.h"

int main(int argc, char* argv[]) {

    // Parse command line arguments
    const cv::String keys =
        "{h help usage ? |        | Print usage }"
        "{o outfolder    |        | Folder where the results of the track segmentation should be stored }"
        "{f flo          |        | Flag that indicates that the flows are in .flo format instead of .tiff }"
        "{d density      | 8      | Track density }"
        "{c coverRadius  | 20     | Minimum distance to the nearest track to initialize a new one in a pixel }"
        "{r rho          | 3.0    | Parameter used for Gaussian blur }"
        "{@images        |        | Text file containing the path to the images }"
        "{@flows         |        | Text file containing the path to the precomputed flows }"
        "{@rflows        |        | Text file containing the path to the precomputed reverse flows}"
        "{@prevtracks    | <none> | Text file containing the path to previous tracks in the sequence, if there are }"
        ;
    
    cv::CommandLineParser parser(argc, argv, keys);
    parser.about("Point tracking");

    if(parser.has("help") || argc == 1) {
        parser.printMessage();
        return EXIT_SUCCESS;
    }
    if(!parser.check()) {
        parser.printErrors();
        return EXIT_FAILURE;
    }

    const std::string imageFileName = parser.get<std::string>("@images");
    std::ifstream imageFile(imageFileName);
    std::vector<cv::Mat> images;
    tfg::readImages(imageFile, images);
    const unsigned int NUMBER_OF_IMAGES = images.size();

    const std::string flowFileName = parser.get<std::string>("@flows");
    const std::string bwdFlowFileName = parser.get<std::string>("@rflows");
    std::ifstream flowFile(flowFileName);
    std::ifstream bwdFlowFile(bwdFlowFileName);
    std::vector<cv::Mat> flows;
    std::vector<cv::Mat> bwdFlows;
    if(parser.has("flo")) {
        tfg::readFlowsFlo(flowFile, flows);
        tfg::readFlowsFlo(bwdFlowFile, bwdFlows);
    } else {
        tfg::readFlowsTiff(flowFile, flows);
        tfg::readFlowsTiff(bwdFlowFile, bwdFlows);
    }

    tfg::TrackTable previousTrackTable;
    tfg::TrackTable newTrackTable;
    const bool hasPreviousTracks = parser.has("@prevtracks");

    if(hasPreviousTracks) {
        const std::string trackFileName = parser.get<std::string>("@prevtracks");
        std::ifstream trackFile(trackFileName);
        previousTrackTable.buildFromFile(trackFile, 1);
        newTrackTable.initializeFromPreviousTable(previousTrackTable);
    }

    const int trackDensity = parser.get<int>("density");
    const int coverRadius = parser.get<int>("coverRadius");
    const double rho = parser.get<double>("rho");
    for(unsigned int f = 0; f < NUMBER_OF_IMAGES - 1; f++) {
        tfg::addTracksToUncoveredZones(images[f], f, newTrackTable, trackDensity, coverRadius, rho);
        tfg::followExistingTracks(flows[f], bwdFlows[f], newTrackTable, f);
    }

    // Continue: write track file and paint tracks (using tfg::TrackTable::paintWeightedTracks() with weight = 1)
    const std::string resultsFolder = parser.get<std::string>("outfolder");
    const std::string fileNames = "tracks";
    const std::vector<float> ones(newTrackTable.numberOfTracks(), 1.0f);
    newTrackTable.getMappingsFromTracks();
    newTrackTable.paintWeightedTracks(ones, images, resultsFolder, fileNames);

}