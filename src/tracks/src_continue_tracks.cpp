#include <iostream>
#include <fstream>
#include <opencv4/opencv2/core.hpp>
#include "ImageUtils.h"
#include "IoUtils.h"
#include "Tracking.h"

int main(int argc, char* argv[]) {

    // Parse command line arguments
    const cv::String keys =
        "{h help usage ?   |        | Print usage }"
        "{o outfolder      |        | Folder where the results of the track segmentation should be stored }"
        "{w outweights     |        | Text file where the resulting weights should be stored }"
        "{t trackFilePath  |        | Path to the output track file }"
        "{f flo            |        | Flag that indicates that the flows are in .flo format instead of .tiff }"
        "{d density        | 8      | Track density }"
        "{c coverRadius    | 8      | Minimum distance to the nearest track to initialize a new one in a pixel }"
        "{r rho            | 3.0    | Parameter used for Gaussian blur }"
        "{F firstNameIndex | 0      | The first index that should be appended at the end of the images' names }"
        "{S structure      |        | Do not remove points that do not show any structure in their vicinity (these points may be incorrectly tracked) }"
        "{B motionBoundary |        | Do not stop tracking points near motion boundaries (this can cause tracks to share the motion of various objects) }"
        "{@images          |        | Text file containing the path to the images }"
        "{@flows           |        | Text file containing the path to the precomputed flows }"
        "{@rflows          |        | Text file containing the path to the precomputed reverse flows}"
        "{@prevtracks      | <none> | Text file containing the path to previous tracks in the sequence, if there are }"
        "{@prevweights     | <none> | Text file containing the path to the weights of the previous tracks }"
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
    imageFile.close();
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
    flowFile.close();
    bwdFlowFile.close();

    tfg::TrackTable previousTrackTable;
    tfg::TrackTable newTrackTable;
    const bool hasPreviousTracks = parser.has("@prevtracks");

    if(hasPreviousTracks) {
        const std::string trackFileName = parser.get<std::string>("@prevtracks");
        std::ifstream trackFile(trackFileName);
        previousTrackTable.buildFromFile(trackFile, 1);
        trackFile.close();
        newTrackTable.initializeFromPreviousTable(previousTrackTable);
    }

    std::vector<float> weights;
    const bool hasPreviousWeights = parser.has("@prevweights");
    if(hasPreviousWeights) {
        const std::string weightsFileName = parser.get<std::string>("@prevweights");
        std::ifstream weightsFile(weightsFileName);
        tfg::readWeights(weightsFile, weights);
        weightsFile.close();
    }

    const int trackDensity = parser.get<int>("density");
    const int coverRadius = parser.get<int>("coverRadius");
    const double rho = parser.get<double>("rho");
    const bool keepStructurelessPoints = parser.has("structure");
    const bool trackMotionBoundaries = parser.has("motionBoundary");
    for(unsigned int f = 0; f < NUMBER_OF_IMAGES - 1; f++) {
        tfg::addTracksToUncoveredZones(images[f], f, newTrackTable, weights, trackDensity, coverRadius, rho, keepStructurelessPoints);
        tfg::followExistingTracks(flows[f], bwdFlows[f], f, newTrackTable, trackMotionBoundaries);
    }

    const std::string resultsFolder = parser.get<std::string>("outfolder");
    const std::string fileNames = "tracks";
    const int firstNameIndex = parser.get<int>("firstNameIndex");
    const std::vector<float> ones(newTrackTable.numberOfTracks(), 1.0f);
    newTrackTable.getMappingsFromTracks();
    newTrackTable.paintWeightedTracks(ones, images, resultsFolder, fileNames, 2, firstNameIndex);

    const std::string outTrackFileName = parser.get<std::string>("trackFilePath");
    std::ofstream outTrackFile(outTrackFileName);
    newTrackTable.writeTracks(outTrackFile);
    outTrackFile.close();

    if(parser.has("outweights")) {
        std::vector<float> filteredWeights;
        filteredWeights.reserve(weights.size());
        for(unsigned int i = 0; i < weights.size(); i++) {
            if(newTrackTable.durationOfTrack(i) < 2) continue;
            filteredWeights.push_back(weights[i]);
        }
        const std::string outWeightsFileName = parser.get<std::string>("outweights");
        std::ofstream outWeightsFile(outWeightsFileName);
        tfg::writeWeights(filteredWeights, outWeightsFile);
        outWeightsFile.close();
    }

    return EXIT_SUCCESS;
}