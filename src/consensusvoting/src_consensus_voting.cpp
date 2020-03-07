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

    tfg::RegionList regionList(1590, images.size());
    tfg::ConsensusVoter consensusVoter(1590, images.size());

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
        cv::Ptr<cv::ximgproc::SuperpixelSLIC> sp = cv::ximgproc::createSuperpixelSLIC(images[f], cv::ximgproc::SLICO, 16);
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
    regionList.computeDescriptors();
    std::chrono::steady_clock::time_point flag8 = std::chrono::steady_clock::now();
    std::cout << "Grouped descriptors in " << (std::chrono::duration_cast<std::chrono::microseconds>(flag8-flag7).count())/1000000.0 << " seconds." << std::endl;

    Eigen::SparseMatrix<float> transM;
    regionList.transitionMatrix(3, 1, 1, transM);
    std::chrono::steady_clock::time_point flag9 = std::chrono::steady_clock::now();
    std::cout << "Transition matrix computed in " << (std::chrono::duration_cast<std::chrono::microseconds>(flag9-flag8).count())/1000000.0 << " seconds." << std::endl;

    std::vector<float> finalVotes;
    std::vector<int> frameBeginningIndices = regionList.getFrameBeginningIndices();
    consensusVoter.reachConsensus(transM, frameBeginningIndices, 0, finalVotes);
    std::chrono::steady_clock::time_point flag10 = std::chrono::steady_clock::now();
    std::cout << "Reached consensus in " << (std::chrono::duration_cast<std::chrono::microseconds>(flag10-flag9).count())/1000000.0 << " seconds." << std::endl;

    std::vector<cv::Mat> finalMasks;
    regionList.masksFromVotes(finalVotes, finalMasks, 0.1f);
    std::chrono::steady_clock::time_point flag11 = std::chrono::steady_clock::now();
    std::cout << "Created masks from votes in " << (std::chrono::duration_cast<std::chrono::microseconds>(flag11-flag10).count())/1000000.0 << " seconds." << std::endl;

    std::string resultsFolder(opt_outmodel.value);
    std::string fileNameSegmentation = "result";
    tfg::saveMaskedImages(images, finalMasks, resultsFolder, fileNameSegmentation);
    std::chrono::steady_clock::time_point flag12 = std::chrono::steady_clock::now();
    std::cout << "Saved results in " << (std::chrono::duration_cast<std::chrono::microseconds>(flag12-flag11).count())/1000000.0 << " seconds." << std::endl;

    return EXIT_SUCCESS;
}