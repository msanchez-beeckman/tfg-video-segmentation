#include <iostream>
#include <fstream>
#include <chrono>
#include <opencv4/opencv2/core.hpp>
#include <opencv4/opencv2/imgcodecs.hpp>
#include "ImageUtils.h"
#include "IoUtils.h"
#include "TrackTable.h"
#include "Grid.h"

int main(int argc, char* argv[]) {

    // Parse command line arguments
    const cv::String keys =
        "{h help usage ?     |       | Print usage }"
        "{o outfolder        |       | Folder where the results of the segmentation should be stored }"
        "{b brox             |       | Parse tracks using Brox's codification }"
        "{d minTrackDuration | 5     | Minimum track duration to take it into account }"
        "{r tradius          | 32    | Minimum distance from region to tracks for it to be considered textureless }"
        "{g tbgbias          | 0.05  | Synthetic background observation for regions without reliable tracks }"
        "{u lambdau          | 100   | Unary cost weight for energy minimization }"
        "{s lambdas          | 0.001 | Smoothness cost weight for energy minimization }"
        "{e minEdgeCost      | 0     | Minimum edge cost on graph cut }"
        "{t threshold        | 0.25  | Threshold to obtain mask from sliced values }"
        "{F firstNameIndex   | 0     | The first index that should be appended at the end of the images' names }"
        "{M multilabel       |       | Perform a per-label segmentation instead of foreground/background }"
        "{@images            |       | Text file containing the path to the images to be segmented }"
        "{@tracks            |       | Text file containing the path to the precomputed tracks }"
        "{@weights           |       | Text file containing the weights of the tracks }"
        ;
    
    cv::CommandLineParser parser(argc, argv, keys);
    parser.about("Bilateral Video Segmentation");

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
    tfg::TrackTable trackTable;
    const int minTrackDuration = parser.get<int>("minTrackDuration");
    const bool hasBroxFlag = parser.has("brox");
    if(hasBroxFlag) {
        trackTable.buildFromBroxFile(trackFile, minTrackDuration);
    } else {
        trackTable.buildFromFile(trackFile, minTrackDuration);
    }
    trackFile.close();
    std::cout << "Built TrackTable" << '\n';

    // Read weights of the tracks
    const std::string weightFileName = parser.get<std::string>("@weights");
    std::ifstream weightFile(weightFileName);
    std::vector<std::vector<float>> labelWeights;
    tfg::readWeightsMultilabel(weightFile, labelWeights);
    const std::vector<float>& weightsBg = labelWeights[0];
    weightFile.close();
    std::cout << "Read track weights" << '\n';

    // Read images to be segmented
    const std::string imageNamesFileName = parser.get<std::string>("@images");
    std::ifstream imageNamesFile(imageNamesFileName);
    std::vector<cv::Mat> images;
    std::vector<cv::Mat> imagesLuv;
    tfg::readImages(imageNamesFile, images);
    imageNamesFile.close();
    tfg::bgr2luv(images, imagesLuv);
    std::cout << "Read images and converted them to Luv color space" << '\n';

    // Create bilateral grid
    constexpr std::array<float, 6> scales = {1/10.0f, 1/35.0f, 1/35.0f, 1/7.3f, 1/8.5f, 1/8.5f};
    std::vector<cv::Mat> initialMasks(images.size());
    std::vector<cv::Mat> labelMasks(images.size());
    for(unsigned int i = 0; i < initialMasks.size(); i++) {
        initialMasks[i] = 255 * cv::Mat::ones(images[i].size(), CV_8UC1);
        labelMasks[i] = cv::Mat::zeros(images[i].size(), CV_8UC1);
    }
    tfg::Grid bilateralGrid(scales, imagesLuv, initialMasks);
    std::cout << "Created bilateral grid for foreground/background segmentation" << '\n';

    // Splatting phase
    const int texturelessRadius = parser.get<int>("tradius");
    const float texturelessBgBias = parser.get<float>("tbgbias");
    std::chrono::steady_clock::time_point splatBeginTime = std::chrono::steady_clock::now();
    bilateralGrid.splatMass();
    bilateralGrid.splatTrackWeights(trackTable, weightsBg, texturelessRadius, texturelessBgBias);
    std::chrono::steady_clock::time_point splatEndTime = std::chrono::steady_clock::now();
    std::cout << "Splatted mass and lifted track values in " << (std::chrono::duration_cast<std::chrono::microseconds>(splatEndTime-splatBeginTime).count())/1000000.0 << " seconds." << '\n';

    // Min cut phase
    const float lambda_u = parser.get<float>("lambdau");
    const float lambda_s = parser.get<float>("lambdas");
    const float minEdgeCost = parser.get<float>("minEdgeCost");
    constexpr std::array<float, 6> W = {0.5f, 0.5f, 0.5f, 1.3f, 1.5f, 1.5f};
    std::chrono::steady_clock::time_point cutBeginTime = std::chrono::steady_clock::now();
    bilateralGrid.graphCut(lambda_u, lambda_s, minEdgeCost, W);
    std::chrono::steady_clock::time_point cutEndTime = std::chrono::steady_clock::now();
    std::cout << "Performed graph cut in " << (std::chrono::duration_cast<std::chrono::microseconds>(cutEndTime-cutBeginTime).count())/1000000.0 << " seconds." << '\n';

    // Slicing phase
    const float threshold = parser.get<float>("threshold");
    std::vector<cv::Mat> foregroundMasks;
    std::chrono::steady_clock::time_point sliceBeginTime = std::chrono::steady_clock::now();
    bilateralGrid.slice(foregroundMasks, threshold);
    std::chrono::steady_clock::time_point sliceEndTime = std::chrono::steady_clock::now();
    std::cout << "Sliced values in " << (std::chrono::duration_cast<std::chrono::microseconds>(sliceEndTime-sliceBeginTime).count())/1000000.0 << " seconds." << '\n';


    const bool multilabelEnabled = parser.has("multilabel");
    const int firstNameIndex = parser.get<int>("firstNameIndex");
    const std::string resultsFolder = parser.get<std::string>("outfolder");
    const std::string fileNameOverlaidImages = "resultOverlaid";
    if(multilabelEnabled) {
        const int numberOfLastLabel = static_cast<int>(labelWeights.size()) - 1;
        for(int l = 1; l <= numberOfLastLabel; l++) {
            std::cout << '\n' << "Starting segmentation of label " << l << '\n';
            std::vector<float> currentWeights;
            tfg::groupLabelWeights(labelWeights, currentWeights, l);

            std::chrono::steady_clock::time_point labelBeginTime = std::chrono::steady_clock::now();
            tfg::Grid currentGrid(scales, imagesLuv, foregroundMasks);
            currentGrid.splatMass();
            currentGrid.splatTrackWeights(trackTable, currentWeights, texturelessRadius, texturelessBgBias);
            currentGrid.graphCut(lambda_u, lambda_s, minEdgeCost, W);

            std::vector<cv::Mat> binaryMasks;
            currentGrid.slice(binaryMasks, threshold);
            std::chrono::steady_clock::time_point labelEndTime = std::chrono::steady_clock::now();
            std::cout << "Finished segmentation of label " << l << " in " << (std::chrono::duration_cast<std::chrono::microseconds>(labelEndTime-labelBeginTime).count())/1000000.0 << " seconds." << '\n';

            for(unsigned int f = 0; f < labelMasks.size(); f++) {
                labelMasks[f].setTo(cv::Scalar(l), binaryMasks[f]);
            }
        }
        tfg::saveOverlaidMultilabeledImages(images, labelMasks, numberOfLastLabel + 1, resultsFolder, fileNameOverlaidImages, 0.4f, firstNameIndex);
    } else {
        tfg::saveOverlaidImages(images, foregroundMasks, resultsFolder, fileNameOverlaidImages, 0.4f, firstNameIndex);
    }

    // Paint only the foreground of the original images
    const std::string fileNameMask = "mask";
    tfg::saveMaskedImages(foregroundMasks, foregroundMasks, resultsFolder, fileNameMask, firstNameIndex);
    const std::string fileNameSegmentation = "foreground";
    tfg::saveMaskedImages(images, foregroundMasks, resultsFolder, fileNameSegmentation, firstNameIndex);

    return EXIT_SUCCESS;

}
