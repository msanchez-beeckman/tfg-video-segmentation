#include <iostream>
#include <opencv4/opencv2/core.hpp>
#include <chrono>
#include "TrackTable.h"
#include "Homography.h"
#include "ImageUtils.h"
#include "MotionModel.h"
#include "CmdParser.h"

int main(int argc, char* argv[]) {

    // Parse command line arguments
    std::vector<OptStruct *> options;
    OptStruct opt_outweights = {"w:", 0, "weights.txt", nullptr, "Text file where the resulting weights should be stored, in order corresponding to each track"}; options.push_back(&opt_outweights);
    OptStruct opt_outmodel = {"o:", 0, "./results/model/", nullptr, "Folder where the results of the track segmentation should be stored"}; options.push_back(&opt_outmodel);
    OptStruct opt_brox = {"b", 0, nullptr, nullptr, "Parse tracks using Brox's codification"}; options.push_back(&opt_brox);

    std::vector<ParStruct *> parameters;
    ParStruct par_tracks = {"tracks", nullptr, "Text file containing codified tracks"}; parameters.push_back(&par_tracks);
    ParStruct par_images = {"images", nullptr, "Text file containing the path to the images whose tracks are being segmented"}; parameters.push_back(&par_images);

    if (!parsecmdline("bgmm", "Segmenting tracks", argc, argv, options, parameters))
        return EXIT_FAILURE;

    
    const float tau2 = 16.0f;


    // Read tracks from text file
    std::ifstream trackFile(par_tracks.value);
    std::shared_ptr<tfg::TrackTable> trackTable = std::make_shared<tfg::TrackTable>();
    if(opt_brox.flag) {
        trackTable->buildFromBroxFile(trackFile);
    } else {
        trackTable->buildFromFile(trackFile);
    }
    trackFile.close();

    std::shared_ptr<tfg::MotionModel> model = std::make_shared<tfg::MotionModel>(trackTable, tau2);
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    std::vector<std::vector<int>> inliers;
    model->fitFromRANSAC(inliers);
    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    std::cout << "(0) Cost: " << model->getCost() << std::endl;
    std::cout << "RANSAC total time: " << (std::chrono::duration_cast<std::chrono::microseconds>(end-begin).count())/1000000.0 << " seconds." << std::endl;

    
    std::vector<float> residuals2 = model->getResiduals2();
    std::vector<float> weights2 = tfg::getWeights2(residuals2, tau2);

    std::vector<float> inlierWeights = tfg::getWeightsFromInliers(inliers, trackTable);
    
    tfg::IRLS(model, trackTable, weights2, tau2);
    // tfg::IRLS(model, trackTable, inlierWeights);


    std::ofstream weightsFile(opt_outweights.value);
    tfg::writeWeights(weightsFile, weights2);
    // tfg::writeWeights(weightsFile, inlierWeights);



    std::ifstream imageNamesFile(par_images.value);
    std::vector<cv::Mat> images;
    tfg::readImages(imageNamesFile, images);

    std::string resultsFolder(opt_outmodel.value);
    std::string fileNameModel = "finalModel";
    std::string fileNameInliers = "ransacModel";
    // tfg::paintTracks(trackTable, weights2, images, resultsFolder, fileNameModel);
    // tfg::paintTracks(trackTable, inlierWeights, images, resultsFolder, fileNameInliers);
    trackTable->paintWeightedTracks(weights2, images, resultsFolder, fileNameModel);
    trackTable->paintWeightedTracks(inlierWeights, images, resultsFolder, fileNameInliers);


    return EXIT_SUCCESS;
}