#include <iostream>
#include <algorithm>
#include <chrono>
#include <opencv4/opencv2/core.hpp>
#include <opencv4/opencv2/imgproc/imgproc.hpp>
#include <opencv4/opencv2/highgui/highgui.hpp>
#include <opencv4/opencv2/ximgproc/slic.hpp>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Sparse>
#include "Region.h"
#include "RegionList.h"
#include "ConsensusVoter.h"
#include "ImageUtils.h"
#include "CmdParser.h"

int main(int argc, char* argv[]) {

    // Parse command line arguments
    std::vector<OptStruct *> options;
    OptStruct opt_outmodel = {"o:", 0, "./results/nlcsegmentation/", nullptr, "Folder where the results of the segmentation should be stored"}; options.push_back(&opt_outmodel);
    OptStruct opt_spsize = {"s:", 0, "16", nullptr, "Superpixel size"}; options.push_back(&opt_spsize);

    std::vector<ParStruct *> parameters;
    ParStruct par_images = {"images", nullptr, "Text file containing the path to the images to be segmented"}; parameters.push_back(&par_images);
    ParStruct par_flows = {"flows", nullptr, "Text file containing the path to the computed flows for each frame"}; parameters.push_back(&par_flows);

    if (!parsecmdline("homography", "Calculating homography between two images", argc, argv, options, parameters))
        return EXIT_FAILURE;

    // cv::Mat image = cv::imread("/home/marco/CLionProjects/tfg_video_segmentation/data/bear/00000.jpg", cv::IMREAD_COLOR);
    // cv::Mat image32;
    // image.convertTo(image32, CV_32FC3);
    // cv::Mat flowu = cv::imread("/home/marco/Projects/TFG/testing/pointtracking/bear/flowu00000.tiff", cv::IMREAD_ANYDEPTH);
    // cv::Mat flowv = cv::imread("/home/marco/Projects/TFG/testing/pointtracking/bear/flowv00000.tiff", cv::IMREAD_ANYDEPTH);

    // auto channels = std::vector<cv::Mat>{image32, flowu, flowv};
    // cv::Mat fiveChannMatrix;
    // cv::merge(channels, fiveChannMatrix);

    // std::cout << "Merged image and flows in a single 5-channel Mat" << std::endl;

    std::vector<cv::Mat> images;
    std::ifstream imageListFile(par_images.value);
    tfg::readImages(imageListFile, images);

    const int SUPERPIXEL_SIZE = std::stoi(opt_spsize.value);
    tfg::RegionList regionList((images[0].cols * images[1].rows)/(SUPERPIXEL_SIZE * SUPERPIXEL_SIZE), images.size());
    tfg::ConsensusVoter consensusVoter((images[0].cols * images[1].rows)/(SUPERPIXEL_SIZE * SUPERPIXEL_SIZE), images.size());

    std::ifstream flowListFile(par_flows.value);

    std::chrono::steady_clock::time_point flag1 = std::chrono::steady_clock::now();
    if(!consensusVoter.initializeMotionSaliencyScores(flowListFile, 0.5f)) {
        std::cout << "No dominant motion has been found in the video" << std::endl;
        return EXIT_FAILURE;
    }
    std::chrono::steady_clock::time_point flag2 = std::chrono::steady_clock::now();
    std::cout << "Motion saliency scores computed in " << (std::chrono::duration_cast<std::chrono::microseconds>(flag2-flag1).count())/1000000.0 << " seconds." << std::endl;

    for(unsigned int f = 0; f < images.size(); f++) {

        std::chrono::steady_clock::time_point flag3 = std::chrono::steady_clock::now();
        std::cout << "Frame " << f << ":" << std::endl;
        cv::Ptr<cv::ximgproc::SuperpixelSLIC> sp = cv::ximgproc::createSuperpixelSLIC(images[f], cv::ximgproc::SLICO, SUPERPIXEL_SIZE);
        sp->iterate();
        std::chrono::steady_clock::time_point flag4 = std::chrono::steady_clock::now();

        std::cout << "Superpixels computed in " << (std::chrono::duration_cast<std::chrono::microseconds>(flag4-flag3).count())/1000000.0 << " seconds." << std::endl;

        const int SUPERPIXELS_IN_FRAME = sp->getNumberOfSuperpixels();
        cv::Mat pixelLabels;
        sp->getLabels(pixelLabels);
        std::vector<tfg::Region> regionsInFrame;
        regionsInFrame.reserve(SUPERPIXELS_IN_FRAME);
        for(int s = 0; s < SUPERPIXELS_IN_FRAME; s++) {
            tfg::Region region(s, f, images[f], pixelLabels);
            regionsInFrame.push_back(region);
        }
        regionList.addNewFrame(regionsInFrame);
        std::chrono::steady_clock::time_point flag5 = std::chrono::steady_clock::now();
        std::cout << "Regions added to RegionList in " << (std::chrono::duration_cast<std::chrono::microseconds>(flag5-flag4).count())/1000000.0 << " seconds." << std::endl;

        consensusVoter.initializeVotesFromSaliencyInFrame(f, pixelLabels, SUPERPIXELS_IN_FRAME);
        std::chrono::steady_clock::time_point flag6 = std::chrono::steady_clock::now();
        std::cout << "Initial region votes established in " << (std::chrono::duration_cast<std::chrono::microseconds>(flag6-flag5).count())/1000000.0 << " seconds." << std::endl;
        std::cout << std::endl;
    }

    std::chrono::steady_clock::time_point flag7 = std::chrono::steady_clock::now();
    const std::vector<float> initialVotes = consensusVoter.getCurrentVotes();
    std::vector<cv::Mat> initialMasks;
    regionList.masksFromVotes(initialVotes, initialMasks, 0.4f);
    std::chrono::steady_clock::time_point flag8 = std::chrono::steady_clock::now();
    std::cout << "Created masks for initial votes in " << (std::chrono::duration_cast<std::chrono::microseconds>(flag8-flag7).count())/1000000.0 << " seconds." << std::endl;

    std::string resultsFolder(opt_outmodel.value);
    std::string fileNameSaliency = "saliency";
    tfg::saveMaskedImages(images, initialMasks, resultsFolder, fileNameSaliency);
    std::chrono::steady_clock::time_point flag9 = std::chrono::steady_clock::now();
    std::cout << "Saved initial results in " << (std::chrono::duration_cast<std::chrono::microseconds>(flag9-flag8).count())/1000000.0 << " seconds." << std::endl;

    regionList.computeDescriptors();
    std::chrono::steady_clock::time_point flag10 = std::chrono::steady_clock::now();
    std::cout << "Grouped descriptors in " << (std::chrono::duration_cast<std::chrono::microseconds>(flag10-flag9).count())/1000000.0 << " seconds." << std::endl;

    Eigen::SparseMatrix<float, Eigen::RowMajor> transM;
    regionList.transitionMatrix(15, 2, 0.1, transM);
    std::chrono::steady_clock::time_point flag11 = std::chrono::steady_clock::now();
    std::cout << "Transition matrix computed in " << (std::chrono::duration_cast<std::chrono::microseconds>(flag11-flag10).count())/1000000.0 << " seconds." << std::endl;

    const std::vector<int> frameBeginningIndices = regionList.getFrameBeginningIndices();
    consensusVoter.reachConsensus(transM, frameBeginningIndices, 20);
    const std::vector<float> finalVotes = consensusVoter.getCurrentVotes();
    std::chrono::steady_clock::time_point flag12 = std::chrono::steady_clock::now();
    std::cout << "Reached consensus in " << (std::chrono::duration_cast<std::chrono::microseconds>(flag12-flag11).count())/1000000.0 << " seconds." << std::endl;

    std::vector<cv::Mat> finalMasks;
    regionList.masksFromVotes(finalVotes, finalMasks, 0.4f);
    std::chrono::steady_clock::time_point flag13 = std::chrono::steady_clock::now();
    std::cout << "Created masks from votes in " << (std::chrono::duration_cast<std::chrono::microseconds>(flag13-flag12).count())/1000000.0 << " seconds." << std::endl;

    std::string fileNameSegmentation = "result";
    tfg::saveMaskedImages(images, finalMasks, resultsFolder, fileNameSegmentation);
    std::chrono::steady_clock::time_point flag14 = std::chrono::steady_clock::now();
    std::cout << "Saved final results in " << (std::chrono::duration_cast<std::chrono::microseconds>(flag14-flag13).count())/1000000.0 << " seconds." << std::endl;

    return EXIT_SUCCESS;
}