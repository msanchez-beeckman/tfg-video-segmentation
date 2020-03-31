#include <iostream>
#include <algorithm>
#include <chrono>
#include <opencv4/opencv2/core.hpp>
#include <opencv4/opencv2/ximgproc/slic.hpp>
#include "Region.h"
#include "ConsensusVoter.h"
#include "ImageUtils.h"
#include "CmdParser.h"

int main(int argc, char* argv[]) {

    // Parse command line arguments
    std::vector<OptStruct *> options;
    OptStruct opt_outmodel = {"o:", 0, "./results/nlcsegmentation/", nullptr, "Folder where the results of the segmentation should be stored"}; options.push_back(&opt_outmodel);
    OptStruct opt_spsize = {"s:", 0, "16", nullptr, "Superpixel size"}; options.push_back(&opt_spsize);
    OptStruct opt_F = {"F:", 0, "15", nullptr, "Number of frames for KDTree's window"}; options.push_back(&opt_F);
    OptStruct opt_L = {"L:", 0, "4", nullptr, "Number of nearest neighbours for each frame in KDTree's window"}; options.push_back(&opt_L);
    OptStruct opt_sigma2 = {"S:", 0, "0.1", nullptr, "Value used for the denominator when computing costs between nearest neighbours"}; options.push_back(&opt_sigma2);
    OptStruct opt_T = {"T:", 0, "50", nullptr, "Number of iterations to reach consensus"}; options.push_back(&opt_T);
    OptStruct opt_threshold = {"t:", 0, "0.3", nullptr, "Minimum value of a vote for a superpixel to be considered foreground"}; options.push_back(&opt_threshold);
    OptStruct opt_rmblobs = {"r", 0, nullptr, nullptr, "Remove small non-connected blobs after segmentation"}; options.push_back(&opt_rmblobs);

    std::vector<ParStruct *> parameters;
    ParStruct par_images = {"images", nullptr, "Text file containing the path to the images to be segmented"}; parameters.push_back(&par_images);
    ParStruct par_flows = {"flows", nullptr, "Text file containing the path to the computed flows for each frame"}; parameters.push_back(&par_flows);

    if (!parsecmdline("homography", "Calculating homography between two images", argc, argv, options, parameters))
        return EXIT_FAILURE;

    // Read the images and create a ConsensusVoter object
    std::vector<cv::Mat> images;
    std::ifstream imageListFile(par_images.value);
    tfg::readImages(imageListFile, images);

    const int SUPERPIXEL_SIZE = std::stoi(opt_spsize.value);
    tfg::ConsensusVoter consensusVoter((images[0].cols * images[1].rows)/(SUPERPIXEL_SIZE * SUPERPIXEL_SIZE), images.size());

    // Initialize the votes using motion saliency. If motion saliency does not work, visual saliency is needed.
    std::ifstream flowListFile(par_flows.value);
    std::chrono::steady_clock::time_point flag1 = std::chrono::steady_clock::now();
    if(!consensusVoter.initializeMotionSaliencyScores(flowListFile, 0.5f)) {
        std::cout << "No dominant motion has been found in the video" << std::endl;
        return EXIT_FAILURE;
    }
    std::string resultsFolder(opt_outmodel.value);
    const std::string fileNameCrudeSaliency = "crudeSaliency";
    consensusVoter.saveSaliencies(resultsFolder, fileNameCrudeSaliency);
    std::chrono::steady_clock::time_point flag2 = std::chrono::steady_clock::now();
    std::cout << "Motion saliency scores computed in " << (std::chrono::duration_cast<std::chrono::microseconds>(flag2-flag1).count())/1000000.0 << " seconds." << std::endl;

    // Extract superpixels for each frame, compute their descriptors, and initialize their votes from the saliency scores
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
        consensusVoter.addRegionsByFrame(regionsInFrame);
        std::chrono::steady_clock::time_point flag5 = std::chrono::steady_clock::now();
        std::cout << "Region descriptors computed in " << (std::chrono::duration_cast<std::chrono::microseconds>(flag5-flag4).count())/1000000.0 << " seconds." << std::endl;

        consensusVoter.initializeVotesInFrame(f, pixelLabels, SUPERPIXELS_IN_FRAME);
        std::chrono::steady_clock::time_point flag6 = std::chrono::steady_clock::now();
        std::cout << "Initial region votes established in " << (std::chrono::duration_cast<std::chrono::microseconds>(flag6-flag5).count())/1000000.0 << " seconds." << std::endl;
        std::cout << std::endl;
    }

    // Compute the transition matrix, and reach consensus iteratively multiplying the votes by it
    std::chrono::steady_clock::time_point flag7 = std::chrono::steady_clock::now();
    const int F = std::stoi(opt_F.value);
    const int L = std::stoi(opt_L.value);
    const float sigma2 = std::stof(opt_sigma2.value);
    consensusVoter.computeTransitionMatrix(F, L, sigma2);
    std::chrono::steady_clock::time_point flag8 = std::chrono::steady_clock::now();
    std::cout << "Transition matrix computed in " << (std::chrono::duration_cast<std::chrono::microseconds>(flag8-flag7).count())/1000000.0 << " seconds." << std::endl;

    const int T = std::stoi(opt_T.value);
    consensusVoter.reachConsensus(T);
    std::chrono::steady_clock::time_point flag9 = std::chrono::steady_clock::now();
    std::cout << "Reached consensus in " << (std::chrono::duration_cast<std::chrono::microseconds>(flag9-flag8).count())/1000000.0 << " seconds." << std::endl;

    // Get final segmentation from the final votes
    std::vector<cv::Mat> finalMasks;
    const float THRESHOLD = std::stof(opt_threshold.value);
    const bool REMOVE_BLOBS = opt_rmblobs.flag != 0;
    consensusVoter.getSegmentation(finalMasks, THRESHOLD, REMOVE_BLOBS);
    std::chrono::steady_clock::time_point flag10 = std::chrono::steady_clock::now();
    std::cout << "Created segmentation from votes in " << (std::chrono::duration_cast<std::chrono::microseconds>(flag10-flag9).count())/1000000.0 << " seconds." << std::endl;

    // TEST: reach consensus by batches, so that votes of small blobs are corrected before continuing with the iterations
    // std::chrono::steady_clock::time_point flag9 = std::chrono::steady_clock::now();
    // std::vector<cv::Mat> finalMasks;
    // for(unsigned int i = 0; i < 5; i++) {
    //     consensusVoter.reachConsensus(10);
    //     consensusVoter.getSegmentation(finalMasks, THRESHOLD);
    // }
    // std::chrono::steady_clock::time_point flag10 = std::chrono::steady_clock::now();
    // std::cout << "Reached consensus and segmented video in " << (std::chrono::duration_cast<std::chrono::microseconds>(flag10-flag9).count())/1000000.0 << " seconds." << std::endl;

    // Save results
    const std::string fileNameMask = "mask";
    tfg::saveMaskedImages(finalMasks, finalMasks, resultsFolder, fileNameMask);
    const std::string fileNameSegmentation = "foreground";
    tfg::saveMaskedImages(images, finalMasks, resultsFolder, fileNameSegmentation);
    const std::string fileNameOverlaidImages = "resultOverlaid";
    tfg::saveOverlaidImages(images, finalMasks, resultsFolder, fileNameOverlaidImages);
    std::chrono::steady_clock::time_point flag11 = std::chrono::steady_clock::now();
    std::cout << "Saved final results in " << (std::chrono::duration_cast<std::chrono::microseconds>(flag11-flag10).count())/1000000.0 << " seconds." << std::endl;

    return EXIT_SUCCESS;
}