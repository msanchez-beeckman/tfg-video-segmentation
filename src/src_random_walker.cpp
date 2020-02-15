#include <iostream>
#include <opencv4/opencv2/core.hpp>
#include <unordered_map>
#include <chrono>
#include "TrackTable.h"
#include "ImageUtils.h"
#include "CmdParser.h"

int main(int argc, char* argv[]) {

    // Parse command line arguments
    std::vector<OptStruct *> options;
    OptStruct opt_outweights = {"w:", 0, "weights.txt", nullptr, "Text file where the resulting weights should be stored, in order corresponding to each track"}; options.push_back(&opt_outweights);
    OptStruct opt_outmodel = {"o:", 0, "./results/seedplusimages/", nullptr, "Folder where the results of the track segmentation should be stored"}; options.push_back(&opt_outmodel);
    OptStruct opt_brox = {"b", 0, nullptr, nullptr, "Parse tracks using Brox's codification"}; options.push_back(&opt_brox);

    std::vector<ParStruct *> parameters;
    ParStruct par_tracks = {"tracks", nullptr, "Text file containing codified tracks"}; parameters.push_back(&par_tracks);
    ParStruct par_seeds = {"seeds", nullptr, "Text file containing the path and frame number of the seeds for the random walker"}; parameters.push_back(&par_seeds);
    ParStruct par_images = {"images", nullptr, "Text file containing the path to the images whose tracks are being segmented"}; parameters.push_back(&par_images);

    if (!parsecmdline("homography", "Calculating homography between two images", argc, argv, options, parameters))
        return EXIT_FAILURE;


    // Read tracks from text file
    std::ifstream trackFile(par_tracks.value);
    std::unique_ptr<tfg::TrackTable> trackTable = std::make_unique<tfg::TrackTable>();
    if(opt_brox.flag) {
        trackTable->buildFromBroxFile(trackFile);
    } else {
        trackTable->buildFromFile(trackFile);
    }
    trackFile.close();

    // Read images that serve as seeds
    std::ifstream seedFile(par_seeds.value);
    std::unordered_map<int, cv::Mat> seedImages;
    tfg::readSeedImages(seedFile, seedImages);

    trackTable->seed(seedImages);
    std::vector<float> probabilities;
    trackTable->propagateSeedsRandomWalk(probabilities);

    std::ifstream imageNamesFile(par_images.value);
    std::vector<cv::Mat> images;
    tfg::readImages(imageNamesFile, images);
    std::string resultsFolder(opt_outmodel.value);
    tfg::paintSeededTracks(trackTable, images, resultsFolder);

    std::cout << "Painted tracks according to seeds" << std::endl;
    
    return EXIT_SUCCESS;
}