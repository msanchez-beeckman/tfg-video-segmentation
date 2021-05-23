#include <chrono>
#include <fstream>
#include <iostream>
#include <unordered_map>

#include <opencv2/core.hpp>

#include "TrackTable.h"
#include "RandomWalker.h"
#include "ImageUtils.h"

int main(int argc, char* argv[]) {

    // Parse command line arguments
    const cv::String keys =
        "{h help usage ?     |     | Print usage }"
        "{o outfolder        |     | Folder where the results of the track segmentation should be stored }"
        "{w outweights       |     | Text file where the resulting weights should be stored }"
        "{b brox             |     | Parse tracks using Brox's codification }"
        "{D davis            |     | Use Davis' ground truth as seeds }"
        "{d minTrackDuration | 5   | Minimum track duration to take it into account }"
        "{l lambda           | 0.1 | Scale parameter for affinity computations }"
        "{F firstNameIndex   | 0   | The first index that should be appended at the end of the images' names }"
        "{@images            |     | Text file containing the path to the images to be segmented }"
        "{@seeds             |     | Text file containing the path and frame number of the seeds for the random walker }"
        "{@tracks            |     | Text file containing the path to the precomputed tracks }"
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

    // Read the image sequence
    const std::string imageNamesFileName = parser.get<std::string>("@images");
    std::ifstream imageNamesFile(imageNamesFileName);
    std::vector<cv::Mat> images;
    tfg::readImages(imageNamesFile, images);
    imageNamesFile.close();

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
    trackTable->addColorInfo(images);

    // Read images that serve as seeds
    const std::string seedFileName = parser.get<std::string>("@seeds");
    std::ifstream seedFile(seedFileName);
    std::unordered_map<int, cv::Mat> seedImages;
    tfg::readSeedImages(seedFile, seedImages);
    seedFile.close();

    // Create the random walker than propagate the seeds to each track
    const bool usingDavisGT = parser.has("davis");
    tfg::RandomWalker walker(trackTable);
    if(usingDavisGT) {
        walker.seedDavis(seedImages);
    } else {
        walker.seed(seedImages);
    }
    const float lambda = parser.get<float>("lambda");
    walker.propagateSeeds(lambda);

    // Write the results in a file
    const std::string weightsFileName = parser.get<std::string>("outweights");
    std::ofstream weightsFile(weightsFileName);
    std::cout << "Writing in a file the probabilities of each label for each track" << '\n';
    walker.writeProbabilities(weightsFile);
    weightsFile.close();

    // Paint the tracks according to their most likely label (using the same color as the seeds for the label)
    const std::string resultsFolder = parser.get<std::string>("outfolder");
    const std::string fileName = "out";
    const int firstNameIndex = parser.get<int>("firstNameIndex");
    trackTable->paintLabeledTracks(images, resultsFolder, fileName, firstNameIndex);

    std::cout << "Painted tracks according to seeds" << '\n';
    
    return EXIT_SUCCESS;
}
