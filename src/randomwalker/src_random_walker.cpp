#include <iostream>
#include <fstream>
#include <opencv4/opencv2/core.hpp>
#include <unordered_map>
#include <chrono>
#include "TrackTable.h"
#include "RandomWalker.h"
#include "ImageUtils.h"

int main(int argc, char* argv[]) {

    // Parse command line arguments
    const cv::String keys =
        "{h help usage ?     |    | Print usage }"
        "{o outfolder        |    | Folder where the results of the track segmentation should be stored }"
        "{w outweights       |    | Text file where the resulting weights should be stored }"
        "{b brox             |    | Parse tracks using Brox's codification }"
        "{d minTrackDuration | 10 | Minimum track duration to take it into account }"
        "{@images            |    | Text file containing the path to the images to be segmented }"
        "{@seeds             |    | Text file containing the path and frame number of the seeds for the random walker }"
        "{@tracks            |    | Text file containing the path to the precomputed tracks }"
        ;
    
    cv::CommandLineParser parser(argc, argv, keys);
    parser.about("Track segmentation using propagation of seeds with random walker");

    if(parser.has("help") || argc == 1) {
        parser.printMessage();
        return EXIT_SUCCESS;
    }
    if(!parser.check()) {
        parser.printErrors();
        return EXIT_FAILURE;
    }


    // Read tracks from text file
    const std::string trackFileName = parser.get<std::string>("@tracks");
    std::ifstream trackFile(trackFileName);
    std::shared_ptr<tfg::TrackTable> trackTable = std::make_shared<tfg::TrackTable>();
    const int minTrackDuration = parser.get<int>("minTrackDuration");
    const bool hasBroxFlag = parser.has("brox");
    if(hasBroxFlag) {
        trackTable->buildFromBroxFile(trackFile, minTrackDuration);
    } else {
        trackTable->buildFromFile(trackFile, minTrackDuration);
    }
    trackFile.close();

    // Read images that serve as seeds
    const std::string seedFileName = parser.get<std::string>("@seeds");
    std::ifstream seedFile(seedFileName);
    std::unordered_map<int, cv::Mat> seedImages;
    tfg::readSeedImages(seedFile, seedImages);

    // Create the random walker than propagate the seeds to each track
    tfg::RandomWalker walker(trackTable);
    walker.seed(seedImages);
    walker.propagateSeeds();

    // Write the results in a file
    const std::string weightsFileName = parser.get<std::string>("outweights");
    std::ofstream weightsFile(weightsFileName);
    std::cout << "Writing in a file the probabilities of each label for each track" << std::endl;
    walker.writeProbabilities(weightsFile);

    // Paint the tracks according to their most likely label (using the same color as the seeds for the label)
    const std::string imageNamesFileName = parser.get<std::string>("@images");
    std::ifstream imageNamesFile(imageNamesFileName);
    std::vector<cv::Mat> images;
    tfg::readImages(imageNamesFile, images);

    const std::string resultsFolder = parser.get<std::string>("outfolder");
    const std::string fileName = "out";
    trackTable->paintLabeledTracks(images, resultsFolder, fileName);

    std::cout << "Painted tracks according to seeds" << std::endl;
    
    return EXIT_SUCCESS;
}