#include <iostream>
#include <boost/algorithm/string.hpp>
#include <chrono>
#include "TrackTable.h"
#include "Homography.h"
#include "ImageUtils.h"
#include "MotionModel.h"
#include "CmdParser.h"

int main(int argc, char* argv[]) {

    // Parse command line arguments
    std::vector<OptStruct *> options;
    OptStruct opt_output = {"o:", 0, "out.txt", nullptr, "File with the weight of each trajectory"}; options.push_back(&opt_output);
    OptStruct opt_images = {"m:", 0, nullptr, nullptr, "File containing image names"}; options.push_back(&opt_images);
    OptStruct opt_brox = {"b", 0, nullptr, nullptr, "Parse tracks using Brox's codification"}; options.push_back(&opt_brox);

    std::vector<ParStruct *> parameters;
    ParStruct pinput = {"input", nullptr, "input file"}; parameters.push_back(&pinput);

    if (!parsecmdline("homography", "Calculating homography between two images", argc, argv, options, parameters))
        return EXIT_FAILURE;


    // Read tracks from text file
    std::ifstream trackFile(pinput.value);
    std::unique_ptr<tfg::TrackTable> trackTable = std::make_unique<tfg::TrackTable>();
    if(opt_brox.flag) {
        trackTable->buildFromBroxFile(trackFile);
    } else {
        trackTable->buildFromFile(trackFile);
    }
    trackFile.close();

    std::shared_ptr<tfg::MotionModel> model = std::make_shared<tfg::MotionModel>();
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    model->fitFromRANSAC(trackTable, 4);
    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    std::cout << "(0) Cost: " << model->getCost() << std::endl;
    std::cout << "RANSAC total time: " << (std::chrono::duration_cast<std::chrono::microseconds>(end-begin).count())/1000000.0 << " seconds." << std::endl;

    
    std::vector<float> residuals2 = model->getResiduals2();
    std::vector<float> weights2 = tfg::getWeights2(residuals2, 4);
    
    // tfg::IRLS(mappings, tracks, homographies, weights);
    tfg::IRLS(model, trackTable, weights2);


    // tfg::printVector(weights);
    std::ofstream weightsFile(opt_output.value);
    tfg::writeWeights(weightsFile, weights2);



    // std::ifstream imagesFile(opt_images.value);
    // std::vector<libUSTG::cflimage> iImages;
    // tfg::readImages(imagesFile, iImages);
    // tfg::paintTracks(tracks, weights2, iImages);


    return EXIT_SUCCESS;
}