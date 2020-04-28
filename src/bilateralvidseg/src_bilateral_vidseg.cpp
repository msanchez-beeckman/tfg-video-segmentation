#include <iostream>
#include <fstream>
#include <chrono>
#include <opencv4/opencv2/core.hpp>
#include "ImageUtils.h"
#include "IoUtils.h"
#include "TrackTable.h"
#include "Grid.h"

// void densifyObjects(const std::vector<cv::Mat> &imagesLuv,
//                     const std::vector<std::vector<float>> &labelWeights,
//                     const std::vector<cv::Mat> &masks,
//                     int firstLabel, int lastLabel,
//                     std::vector<cv::Mat> &labeledObjects,
//                     tfg::TrackTable &trackTable,
//                     const std::array<float, 6> &scales,
//                     const std::array<float, 6> &W,
//                     int texturelessRadius, float texturelessBgBias,
//                     float lambda_u, float lambda_s, float minEdgeCost,
//                     float threshold) {
//     if(firstLabel == lastLabel) {
//         for(unsigned int f = 0; f < masks.size(); f++) {
//             labeledObjects[f].setTo(firstLabel, masks[f]);
//         }
//         std::cout << "Determined pixels for label " << firstLabel << '\n';
//         return;
//     }
//     const int middleLabel = firstLabel + (lastLabel - firstLabel)/2;
//     std::cout << "Grid for labels [" << firstLabel << ", " << middleLabel << "] vs [" << middleLabel + 1 << ", " << lastLabel << "]" << '\n'; 
    
//     std::vector<float> weights;
//     tfg::groupLabelWeights(labelWeights, weights);

//     tfg::Grid bilateralGrid(scales, imagesLuv, masks);
//     bilateralGrid.splatMass();
//     bilateralGrid.splatTrackWeights(trackTable, weights, texturelessRadius, texturelessBgBias);
//     bilateralGrid.graphCut(lambda_u, lambda_s, minEdgeCost, W);
//     std::vector<cv::Mat> binaryMasks;
//     bilateralGrid.slice(binaryMasks, threshold);
//     std::cout << "Finished slicing" << '\n';
//     std::vector<cv::Mat> invertedBinaryMasks(binaryMasks);
//     for(unsigned int i = 0; i < binaryMasks.size(); i++) {
//         cv::bitwise_not(binaryMasks[i], invertedBinaryMasks[i]);
//         cv::bitwise_and(invertedBinaryMasks[i], masks[i], invertedBinaryMasks[i]);
//         // invertedBinaryMasks[i] = (255 - binaryMasks[i]) & masks[i];
//     }
//     std::cout << "Got binary masks" << '\n';

//     const std::vector<std::vector<float>> leftWeights(labelWeights.begin(), labelWeights.begin() + labelWeights.size()/2);
//     std::cout << "Got left half of weights vector" << '\n';
//     const std::vector<std::vector<float>> rightWeights(labelWeights.begin() + labelWeights.size()/2, labelWeights.end());
//     std::cout << "Got right half of weights vector" << '\n';

//     densifyObjects(imagesLuv, rightWeights, binaryMasks, middleLabel + 1, lastLabel, labeledObjects, trackTable, scales, W, texturelessRadius, texturelessBgBias, lambda_u, lambda_s, minEdgeCost, threshold);
//     densifyObjects(imagesLuv, leftWeights, invertedBinaryMasks, firstLabel, middleLabel, labeledObjects, trackTable, scales, W, texturelessRadius, texturelessBgBias, lambda_u, lambda_s, minEdgeCost, threshold);

// }

int main(int argc, char* argv[]) {

    // Parse command line arguments
    const cv::String keys =
        "{h help usage ?     |       | Print usage }"
        "{o outfolder        |       | Folder where the results of the segmentation should be stored }"
        "{b brox             |       | Parse tracks using Brox's codification }"
        "{d minTrackDuration | 10    | Minimum track duration to take it into account }"
        "{r tradius          | 32    | Minimum distance from region to tracks for it to be considered textureless }"
        "{g tbgbias          | 0.05  | Synthetic background observation for regions without reliable tracks }"
        "{u lambdau          | 100   | Unary cost weight for energy minimization }"
        "{s lambdas          | 0.001 | Smoothness cost weight for energy minimization }"
        "{e minEdgeCost      | 0     | Minimum edge cost on graph cut }"
        "{t threshold        | 0.25  | Threshold to obtain mask from sliced values }"
        "{F firstNameIndex   | 0     | The first index that should be appended at the end of the images' names }"
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
    
    std::chrono::steady_clock::time_point flag1 = std::chrono::steady_clock::now();
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
    std::chrono::steady_clock::time_point flag2 = std::chrono::steady_clock::now();
    std::cout << "TrackTable built in " << (std::chrono::duration_cast<std::chrono::microseconds>(flag2-flag1).count())/1000000.0 << " seconds." << std::endl;

    // Read weights of the tracks
    const std::string weightFileName = parser.get<std::string>("@weights");
    std::ifstream weightFile(weightFileName);
    std::vector<float> weights;
    tfg::readWeights(weightFile, weights);
    // std::vector<std::vector<float>> labelWeights;
    // tfg::readWeightsMultilabel(weightFile, labelWeights);
    weightFile.close();
    std::chrono::steady_clock::time_point flag3 = std::chrono::steady_clock::now();
    std::cout << "Track weights read in " << (std::chrono::duration_cast<std::chrono::microseconds>(flag3-flag2).count())/1000000.0 << " seconds." << std::endl;

    // Read images to be segmented
    const std::string imageNamesFileName = parser.get<std::string>("@images");
    std::ifstream imageNamesFile(imageNamesFileName);
    std::vector<cv::Mat> images;
    std::vector<cv::Mat> imagesLuv;
    tfg::readImages(imageNamesFile, images);
    imageNamesFile.close();
    tfg::bgr2luv(images, imagesLuv);
    std::chrono::steady_clock::time_point flag4 = std::chrono::steady_clock::now();
    std::cout << "Images read and converted to Luv in " << (std::chrono::duration_cast<std::chrono::microseconds>(flag4-flag3).count())/1000000.0 << " seconds." << std::endl;

    // Create bilateral grid
    const std::array<float, 6> scales = {1/10.0f, 1/35.0f, 1/35.0f, 1/7.3f, 1/8.5f, 1/8.5f};
    std::vector<cv::Mat> gridMasks(images.size(), cv::Mat(images.size(), CV_8UC1, 255));
    tfg::Grid bilateralGrid(scales, imagesLuv, gridMasks);
    std::chrono::steady_clock::time_point flag5 = std::chrono::steady_clock::now();
    std::cout << "Bilateral grid created in " << (std::chrono::duration_cast<std::chrono::microseconds>(flag5-flag4).count())/1000000.0 << " seconds." << std::endl;

    // Splatting phase
    const int texturelessRadius = parser.get<int>("tradius");
    const float texturelessBgBias = parser.get<float>("tbgbias");
    bilateralGrid.splatMass();
    bilateralGrid.splatTrackWeights(trackTable, weights, texturelessRadius, texturelessBgBias);
    std::chrono::steady_clock::time_point flag6 = std::chrono::steady_clock::now();
    std::cout << "Splatted mass and lifted track values in " << (std::chrono::duration_cast<std::chrono::microseconds>(flag6-flag5).count())/1000000.0 << " seconds." << std::endl;

    // Min cut phase
    const float lambda_u = parser.get<float>("lambdau");
    const float lambda_s = parser.get<float>("lambdas");
    const float minEdgeCost = parser.get<float>("minEdgeCost");
    const std::array<float, 6> W = {0.5f, 0.5f, 0.5f, 1.3f, 1.5f, 1.5f};
    bilateralGrid.graphCut(lambda_u, lambda_s, minEdgeCost, W);
    std::chrono::steady_clock::time_point flag7 = std::chrono::steady_clock::now();
    std::cout << "Performed graph cut in " << (std::chrono::duration_cast<std::chrono::microseconds>(flag7-flag6).count())/1000000.0 << " seconds." << std::endl;

    // Slicing phase
    const float threshold = parser.get<float>("threshold");
    std::vector<cv::Mat> masks;
    bilateralGrid.slice(masks, threshold);
    std::chrono::steady_clock::time_point flag8 = std::chrono::steady_clock::now();
    std::cout << "Sliced values in " << (std::chrono::duration_cast<std::chrono::microseconds>(flag8-flag7).count())/1000000.0 << " seconds." << std::endl;

    // const std::array<float, 6> scales = {1/10.0f, 1/35.0f, 1/35.0f, 1/7.3f, 1/8.5f, 1/8.5f};    
    // const int texturelessRadius = parser.get<int>("tradius");
    // const float texturelessBgBias = parser.get<float>("tbgbias");
    // const float lambda_u = parser.get<float>("lambdau");
    // const float lambda_s = parser.get<float>("lambdas");
    // const float minEdgeCost = parser.get<float>("minEdgeCost");
    // const std::array<float, 6> W = {0.5f, 0.5f, 0.5f, 1.3f, 1.5f, 1.5f};
    // const float threshold = parser.get<float>("threshold");
    // std::vector<cv::Mat> initialMasks(images.size());
    // std::vector<cv::Mat> labelMasks(images.size());
    // for(unsigned int i = 0; i < initialMasks.size(); i++) {
    //     initialMasks[i] = cv::Mat::ones(images[i].size(), CV_8UC1);
    //     labelMasks[i] = cv::Mat::zeros(images[i].size(), CV_8UC1);
    // }
    // const int numberOfLastLabel = static_cast<int>(labelWeights[0].size()) - 1;
    // densifyObjects(imagesLuv, labelWeights, initialMasks, 0, numberOfLastLabel, labelMasks, trackTable, scales, W, texturelessRadius, texturelessBgBias, lambda_u, lambda_s, minEdgeCost, threshold);

    // const int firstNameIndex = parser.get<int>("firstNameIndex");
    // const std::string resultsFolder = parser.get<std::string>("outfolder");
    // const std::string fileNameOverlaidImages = "resultOverlaid";
    // tfg::saveOverlaidMultilabeledImages(images, labelMasks, numberOfLastLabel + 1, resultsFolder, fileNameOverlaidImages, 0.4f, firstNameIndex);









    // Paint only the foreground of the original images
    const int firstNameIndex = parser.get<int>("firstNameIndex");
    const std::string resultsFolder = parser.get<std::string>("outfolder");
    const std::string fileNameMask = "mask";
    tfg::saveMaskedImages(masks, masks, resultsFolder, fileNameMask, firstNameIndex);
    const std::string fileNameSegmentation = "foreground";
    tfg::saveMaskedImages(images, masks, resultsFolder, fileNameSegmentation, firstNameIndex);
    const std::string fileNameOverlaidImages = "resultOverlaid";
    tfg::saveOverlaidImages(images, masks, resultsFolder, fileNameOverlaidImages, 0.4f, firstNameIndex);
    std::chrono::steady_clock::time_point flag9 = std::chrono::steady_clock::now();
    std::cout << "Saved final results in " << (std::chrono::duration_cast<std::chrono::microseconds>(flag9-flag8).count())/1000000.0 << " seconds." << std::endl;

    return EXIT_SUCCESS;

}