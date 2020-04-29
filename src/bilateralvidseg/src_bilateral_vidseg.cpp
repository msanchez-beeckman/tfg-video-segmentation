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
    // std::vector<float> weights;
    // tfg::readWeights(weightFile, weights);
    std::vector<std::vector<float>> labelWeights;
    tfg::readWeightsMultilabel(weightFile, labelWeights);
    const std::vector<float>& weights = labelWeights[0];
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
    // std::vector<cv::Mat> gridMasks(images.size(), cv::Mat(images.size(), CV_8UC1, 255));
    std::vector<cv::Mat> initialMasks(images.size());
    std::vector<cv::Mat> labelMasks(images.size());
    for(unsigned int i = 0; i < initialMasks.size(); i++) {
        initialMasks[i] = 255 * cv::Mat::ones(images[i].size(), CV_8UC1);
        labelMasks[i] = cv::Mat::zeros(images[i].size(), CV_8UC1);
    }
    tfg::Grid bilateralGrid(scales, imagesLuv, initialMasks);
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
    std::vector<cv::Mat> foregroundMasks;
    bilateralGrid.slice(foregroundMasks, threshold);
    std::chrono::steady_clock::time_point flag8 = std::chrono::steady_clock::now();
    std::cout << "Sliced values in " << (std::chrono::duration_cast<std::chrono::microseconds>(flag8-flag7).count())/1000000.0 << " seconds." << std::endl;


    const int numberOfLastLabel = static_cast<int>(labelWeights.size()) - 1;
    std::vector<int> firstLabelsStack = {1};
    std::vector<int> lastLabelsStack = {numberOfLastLabel};
    std::vector<std::vector<float>> labelWeightsNoBg(labelWeights.begin() + 1, labelWeights.end());
    std::vector<std::vector<std::vector<float>>> labelWeightsStack = {labelWeightsNoBg};
    std::vector<std::vector<cv::Mat>> masksStack = {foregroundMasks};
    while(!firstLabelsStack.empty()) {
        const std::vector<cv::Mat>& currentMasks = masksStack.back();
        const int currentFirstLabel = firstLabelsStack.back();
        const int currentLastLabel = lastLabelsStack.back();
        if(currentFirstLabel == currentLastLabel) {
            for(unsigned int f = 0; f < currentMasks.size(); f++) {
                labelMasks[f].setTo(static_cast<unsigned char>(currentFirstLabel), currentMasks[f]);
            }
            std::cout << "Determined pixels for label " << currentFirstLabel << '\n';
            masksStack.pop_back();
            labelWeightsStack.pop_back();
            firstLabelsStack.pop_back();
            lastLabelsStack.pop_back();
            continue;
        }

        const int currentMiddleLabel = currentFirstLabel + (currentLastLabel - currentFirstLabel)/2;
        std::cout << "Grid for labels [" << currentFirstLabel << ", " << currentMiddleLabel << "] vs [" << currentMiddleLabel + 1 << ", " << currentLastLabel << "]" << '\n';

        const std::vector<std::vector<float>>& currentLabelWeights = labelWeightsStack.back();
        std::vector<float> currentWeights;
        tfg::groupLabelWeights(currentLabelWeights, currentWeights);

        tfg::Grid currentGrid(scales, imagesLuv, currentMasks);
        currentGrid.splatMass();
        currentGrid.splatTrackWeights(trackTable, currentWeights, texturelessRadius, 0.0f);
        currentGrid.graphCut(lambda_u, lambda_s, minEdgeCost, W);
        std::vector<cv::Mat> binaryMasks;
        currentGrid.slice(binaryMasks, 0.05f);
        std::cout << "Finished slicing" << '\n';
        std::vector<cv::Mat> invertedBinaryMasks(binaryMasks.size());
        for(unsigned int i = 0; i < binaryMasks.size(); i++) {
            invertedBinaryMasks[i] = cv::Mat::zeros(binaryMasks[i].size(), CV_8UC1);
            cv::bitwise_and(binaryMasks[i], currentMasks[i], binaryMasks[i]);
            cv::bitwise_not(binaryMasks[i], invertedBinaryMasks[i]);
            cv::bitwise_and(invertedBinaryMasks[i], currentMasks[i], invertedBinaryMasks[i]);
        }

        if(currentFirstLabel == 1 && currentLastLabel == 2) {
            cv::imwrite("/home/marco/tmp/tests/firstMask.png", currentMasks[0]);
            cv::imwrite("/home/marco/tmp/tests/binMask.png", binaryMasks[0]);
            cv::imwrite("/home/marco/tmp/tests/invMask.png", invertedBinaryMasks[0]);
        }

        const std::vector<std::vector<float>> leftWeights(labelWeights.begin(), labelWeights.begin() + labelWeights.size()/2);
        const std::vector<std::vector<float>> rightWeights(labelWeights.begin() + labelWeights.size()/2, labelWeights.end());

        firstLabelsStack.pop_back();
        lastLabelsStack.pop_back();
        masksStack.pop_back();
        labelWeightsStack.pop_back();

        firstLabelsStack.push_back(currentFirstLabel);
        lastLabelsStack.push_back(currentMiddleLabel);
        masksStack.push_back(invertedBinaryMasks);
        labelWeightsStack.push_back(leftWeights);

        firstLabelsStack.push_back(currentMiddleLabel + 1);
        lastLabelsStack.push_back(currentLastLabel);
        masksStack.push_back(binaryMasks);
        labelWeightsStack.push_back(rightWeights);
    }

    const int firstNameIndex = parser.get<int>("firstNameIndex");
    const std::string resultsFolder = parser.get<std::string>("outfolder");
    const std::string fileNameOverlaidImages = "resultOverlaid";
    tfg::saveOverlaidMultilabeledImages(images, labelMasks, numberOfLastLabel + 1, resultsFolder, fileNameOverlaidImages, 0.7f, firstNameIndex);






    // Paint only the foreground of the original images
    const std::string fileNameMask = "mask";
    tfg::saveMaskedImages(foregroundMasks, foregroundMasks, resultsFolder, fileNameMask, firstNameIndex);
    const std::string fileNameSegmentation = "foreground";
    tfg::saveMaskedImages(images, foregroundMasks, resultsFolder, fileNameSegmentation, firstNameIndex);
    std::cout << "Saved final results" << std::endl;

    return EXIT_SUCCESS;

}