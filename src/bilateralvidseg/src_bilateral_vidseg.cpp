#include <iostream>
#include <chrono>
#include <opencv4/opencv2/core.hpp>
#include "CmdParser.h"
#include "ImageUtils.h"
#include "TrackTable.h"
#include "Grid.h"

int main(int argc, char* argv[]) {

    // Parse command line arguments
    std::vector<OptStruct *> options;
    OptStruct opt_outmodel = {"o:", 0, "./results/bvsegmentation/", nullptr, "Folder where the results of the segmentation should be stored"}; options.push_back(&opt_outmodel);
    OptStruct opt_brox = {"b", 0, nullptr, nullptr, "Parse tracks using Brox's codification"}; options.push_back(&opt_brox);

    std::vector<ParStruct *> parameters;
    ParStruct par_images = {"images", nullptr, "Text file containing the path to the images to be segmented"}; parameters.push_back(&par_images);
    ParStruct par_tracks = {"tracks", nullptr, "Text file containing the path to the precomputed tracks"}; parameters.push_back(&par_tracks);
    ParStruct par_weights = {"weights", nullptr, "Text file containing the weights of the tracks"}; parameters.push_back(&par_weights);

    if (!parsecmdline("homography", "Calculating homography between two images", argc, argv, options, parameters))
        return EXIT_FAILURE;
    
    std::chrono::steady_clock::time_point flag1 = std::chrono::steady_clock::now();
    // Read tracks from text file
    std::ifstream trackFile(par_tracks.value);
    tfg::TrackTable trackTable;
    if(opt_brox.flag) {
        trackTable.buildFromBroxFile(trackFile);
    } else {
        trackTable.buildFromFile(trackFile);
    }
    trackFile.close();
    std::chrono::steady_clock::time_point flag2 = std::chrono::steady_clock::now();
    std::cout << "TrackTable built in " << (std::chrono::duration_cast<std::chrono::microseconds>(flag2-flag1).count())/1000000.0 << " seconds." << std::endl;

    // Read weights of the tracks
    std::ifstream weightFile(par_weights.value);
    std::vector<float> weights;
    weights.reserve(trackTable.numberOfTracks());
    for(std::string line; std::getline(weightFile, line); ) {
        float w = std::stof(line);
        weights.push_back(w);
    }
    std::chrono::steady_clock::time_point flag3 = std::chrono::steady_clock::now();
    std::cout << "Track weights read in " << (std::chrono::duration_cast<std::chrono::microseconds>(flag3-flag2).count())/1000000.0 << " seconds." << std::endl;

    // Read images to be segmented
    std::ifstream imageNamesFile(par_images.value);
    std::vector<cv::Mat> images;
    std::vector<cv::Mat> imagesLuv;
    tfg::readImages(imageNamesFile, images);
    tfg::bgr2luv(images, imagesLuv);
    std::chrono::steady_clock::time_point flag4 = std::chrono::steady_clock::now();
    std::cout << "Images read and converted to Luv in " << (std::chrono::duration_cast<std::chrono::microseconds>(flag4-flag3).count())/1000000.0 << " seconds." << std::endl;

    // Create bilateral grid
    const std::array<float, 6> scales = {1/10.0f, 1/35.0f, 1/35.0f, 1/7.3f, 1/8.5f, 1/8.5f};
    tfg::Grid bilateralGrid(scales, imagesLuv);
    std::chrono::steady_clock::time_point flag5 = std::chrono::steady_clock::now();
    std::cout << "Bilateral grid created in " << (std::chrono::duration_cast<std::chrono::microseconds>(flag5-flag4).count())/1000000.0 << " seconds." << std::endl;

    // Splatting phase
    bilateralGrid.splatMass();
    bilateralGrid.splatTrackWeights(trackTable, weights, 32, 0.05f);
    std::chrono::steady_clock::time_point flag6 = std::chrono::steady_clock::now();
    std::cout << "Splatted mass and lifted track values in " << (std::chrono::duration_cast<std::chrono::microseconds>(flag6-flag5).count())/1000000.0 << " seconds." << std::endl;

    // Min cut phase
    const std::array<float, 6> W = {0.5f, 0.5f, 0.5f, 1.3f, 1.5f, 1.5f};
    bilateralGrid.graphCut(100, 0.001f, 0.01f, W);
    std::chrono::steady_clock::time_point flag7 = std::chrono::steady_clock::now();
    std::cout << "Performed graph cut in " << (std::chrono::duration_cast<std::chrono::microseconds>(flag7-flag6).count())/1000000.0 << " seconds." << std::endl;

    // Slicing phase
    std::vector<cv::Mat> masks;
    bilateralGrid.slice(masks, 0.25f);
    std::chrono::steady_clock::time_point flag8 = std::chrono::steady_clock::now();
    std::cout << "Sliced values in " << (std::chrono::duration_cast<std::chrono::microseconds>(flag8-flag7).count())/1000000.0 << " seconds." << std::endl;

    // Paint only the foreground of the original images
    const std::string resultsFolder = "./results/bvsegmentation/";
    const std::string fileNameSegmentation = "result";
    tfg::saveMaskedImages(images, masks, resultsFolder, fileNameSegmentation);


    return EXIT_SUCCESS;

}