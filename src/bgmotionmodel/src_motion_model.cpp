#include <iostream>
#include <fstream>
#include <opencv4/opencv2/core.hpp>
#include <chrono>
#include <cmath>
#include <algorithm>
#include "TrackTable.h"
#include "Homography.h"
#include "ImageUtils.h"
#include "IoUtils.h"
#include "MotionModel.h"

int main(int argc, char* argv[]) {

    // Parse command line arguments
    const cv::String keys =
        "{h help usage ?     |        | Print usage }"
        "{o outfolder        |        | Folder where the results of the track segmentation should be stored }"
        "{w outweights       |        | Text file where the resulting weights should be stored }"
        "{b brox             |        | Parse tracks using Brox's codification }"
        "{d minTrackDuration | 10     | Minimum track duration to take it into account }"
        "{i ransacIterations | 500    | Number of iterations for RANSAC }"
        "{e ransacEpsilon    | 2      | Tolerance used in RANSAC }"
        "{t tau2             | 16     | Square of the inlier threshold used to limit the cost of the residuals }"
        "{F firstNameIndex   | 0      | The first index that should be appended at the end of the images' names }"
        "{@images            |        | Text file containing the path to the images to be segmented }"
        "{@tracks            |        | Text file containing the path to the precomputed tracks }"
        "{@prevweights       | <none> | Text file containing the path to the previous weights of the tracks. This parameter must be accompanied with --minTrackDuration=2 }"
        ;
    
    cv::CommandLineParser parser(argc, argv, keys);
    parser.about("Track segmentation using background motion model");

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

    // Initialize model from RANSAC
    const float tau2 = parser.get<float>("tau2");
    std::shared_ptr<tfg::MotionModel> model = std::make_shared<tfg::MotionModel>(trackTable, tau2);
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    std::vector<std::vector<int>> inliers;
    const int ransacIterations = parser.get<int>("ransacIterations");
    const float ransacEpsilon = parser.get<float>("ransacEpsilon");
    model->fitFromRANSAC(ransacIterations, ransacEpsilon, inliers);
    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    std::cout << "(0) Cost: " << model->getCost() << '\n';
    std::cout << "RANSAC total time: " << (std::chrono::duration_cast<std::chrono::microseconds>(end-begin).count())/1000000.0 << " seconds." << '\n';

    // Get residuals and weights from initial model
    std::vector<float> residuals2 = model->getResiduals2();
    std::vector<float> weights2 = tfg::getWeights2(residuals2, tau2);
    // std::vector<float> inlierWeights = tfg::getWeightsFromInliers(inliers, trackTable);
    
    // Refine the model
    tfg::IRLS(model, trackTable, weights2, tau2);


    // Write the weights of each track to a file, to use it in the dense segmentation 
    std::vector<float> weightsNotSqr; weightsNotSqr.reserve(weights2.size());
    std::transform(weights2.begin(), weights2.end(), std::back_inserter(weightsNotSqr), [](float weight2) -> float { return std::sqrt(weight2); });
    if(parser.has("@prevweights")) {
        const std::string prevWeightsFileName = parser.get<std::string>("@prevweights");
        std::ifstream prevWeightsFile(prevWeightsFileName);
        std::vector<float> prevWeights;
        tfg::readWeights(prevWeightsFile, prevWeights);
        std::transform(weightsNotSqr.begin(), weightsNotSqr.end(), prevWeights.begin(), weightsNotSqr.begin(), [](float w1, float w2) -> float { return std::min(w1, w2); });
        // for(unsigned int i = 0; i < weightsNotSqr.size(); i++) {
        //     weightsNotSqr[i] = std::min(weightsNotSqr[i], prevWeights[i]);
        // }
        prevWeightsFile.close();
    }
    const std::string weightsFileName = parser.get<std::string>("outweights");
    std::ofstream weightsFile(weightsFileName);
    tfg::writeWeights(weightsNotSqr, weightsFile);
    weightsFile.close();


    // Read images, then paint the tracks over them using green for foreground and red for background
    const std::string imageNamesFileName = parser.get<std::string>("@images");
    std::ifstream imageNamesFile(imageNamesFileName);
    std::vector<cv::Mat> images;
    tfg::readImages(imageNamesFile, images);
    imageNamesFile.close();

    const std::string resultsFolder = parser.get<std::string>("outfolder");
    const std::string fileNameModel = "finalModel";
    // std::string fileNameInliers = "ransacModel";
    const int firstNameIndex = parser.get<int>("firstNameIndex");
    trackTable->paintWeightedTracks(weightsNotSqr, images, resultsFolder, fileNameModel, minTrackDuration, firstNameIndex);
    // trackTable->paintWeightedTracks(inlierWeights, images, resultsFolder, fileNameInliers);


    return EXIT_SUCCESS;
}