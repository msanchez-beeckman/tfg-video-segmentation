#include <algorithm>
#include <chrono>
#include <fstream>
#include <iostream>

#include <opencv2/core.hpp>
#include <opencv2/ximgproc/slic.hpp>

#include "ConsensusVoter.h"
#include "ImageUtils.h"
#include "Region.h"


int main(int argc, char* argv[]) {

    // Parse command line arguments
    const cv::String keys =
        "{h help usage ?     |     | Print usage }"
        "{o outfolder        |     | Folder where the results of the track segmentation should be stored }"
        "{d minDomMotion     | 0.5 | Minimum ratio of frames with dominant motion to use motion saliency }"
        "{s spsize           | 16  | Superpixel size }"
        "{O slico            |     | Use SLICO instead of SLIC for superpixel computation }"
        "{F frameWindow      | 15  | Number of frames for KDTree's window }"
        "{L likelyMatches    | 4   | Number of nearest neighbours for each frame in KDTree's window }"
        "{S sigma2           | 0.1 | Value used for the denominator when computing costs between nearest neighbours }"
        "{T iterations       | 50  | Number of iterations to reach consensus }"
        "{t threshold        | 0.3 | Minimum value of a vote for a superpixel to be considered foreground }"
        "{r removeBlobsTh    | 0.1 | Remove small non-connected blobs after segmentation }"
        "{@images            |     | Text file containing the path to the images to be segmented }"
        "{@flows             |     | Text file containing the path to the computed flows for each frame }"
        ;
    
    cv::CommandLineParser parser(argc, argv, keys);
    parser.about("Segmentation by Non-Local Consensus Voting");

    if(parser.has("help") || argc == 1) {
        parser.printMessage();
        return EXIT_SUCCESS;
    }
    if(!parser.check()) {
        parser.printErrors();
        return EXIT_FAILURE;
    }

    // Read the images and create a ConsensusVoter object
    std::vector<cv::Mat> images;
    const std::string imageListFileName = parser.get<std::string>("@images");
    std::ifstream imageListFile(imageListFileName);
    tfg::readImages(imageListFile, images);
    imageListFile.close();

    const int SUPERPIXEL_SIZE = parser.get<int>("spsize");
    tfg::ConsensusVoter consensusVoter((images[0].cols * images[1].rows)/(SUPERPIXEL_SIZE * SUPERPIXEL_SIZE), images.size());

    // Initialize the votes using motion saliency. If motion saliency does not work, visual saliency is needed.
    const std::string flowListFileName = parser.get<std::string>("@flows");
    std::ifstream flowListFile(flowListFileName);
    std::chrono::steady_clock::time_point flag1 = std::chrono::steady_clock::now();
    const float minDominantMotionRatio = parser.get<float>("minDomMotion");
    if(!consensusVoter.initializeMotionSaliencyScores(flowListFile, minDominantMotionRatio)) {
        flowListFile.close();
        std::cout << "No dominant motion has been found in the video" << '\n';
        return EXIT_FAILURE;
    }
    flowListFile.close();
    const std::string resultsFolder = parser.get<std::string>("outfolder");
    const std::string fileNameCrudeSaliency = "crudeSaliency";
    consensusVoter.saveSaliencies(resultsFolder, fileNameCrudeSaliency);
    std::chrono::steady_clock::time_point flag2 = std::chrono::steady_clock::now();
    std::cout << "Motion saliency scores computed in " << (std::chrono::duration_cast<std::chrono::microseconds>(flag2-flag1).count())/1000000.0 << " seconds." << '\n';

    std::vector<cv::Mat> imageMotionInfo;
    consensusVoter.mergeSaliencyWithImages(images, imageMotionInfo);
    const int SLICType = parser.has("slico") ? cv::ximgproc::SLICO : cv::ximgproc::SLIC;
    std::vector<cv::Mat> spContourMasks(images.size());
    
    // Extract superpixels for each frame, compute their descriptors, and initialize their votes from the saliency scores
    for(unsigned int f = 0; f < images.size(); f++) {

        std::chrono::steady_clock::time_point flag3 = std::chrono::steady_clock::now();
        std::cout << "Frame " << f << ":" << '\n';
        // cv::Ptr<cv::ximgproc::SuperpixelSLIC> sp = cv::ximgproc::createSuperpixelSLIC(images[f], cv::ximgproc::SLICO, SUPERPIXEL_SIZE);
        cv::Ptr<cv::ximgproc::SuperpixelSLIC> sp = cv::ximgproc::createSuperpixelSLIC(imageMotionInfo[f], SLICType, SUPERPIXEL_SIZE);
        sp->iterate();
        // sp->enforceLabelConnectivity();
        std::chrono::steady_clock::time_point flag4 = std::chrono::steady_clock::now();

        std::cout << "Superpixels computed in " << (std::chrono::duration_cast<std::chrono::microseconds>(flag4-flag3).count())/1000000.0 << " seconds." << '\n';

        const int SUPERPIXELS_IN_FRAME = sp->getNumberOfSuperpixels();
        cv::Mat pixelLabels;
        sp->getLabels(pixelLabels);
        cv::Mat spContourMask;
        sp->getLabelContourMask(spContourMask);
        spContourMasks[f] = 255 - spContourMask;

        std::vector<tfg::Region> regionsInFrame;
        regionsInFrame.reserve(SUPERPIXELS_IN_FRAME);
        for(int s = 0; s < SUPERPIXELS_IN_FRAME; s++) {
            tfg::Region region(s, f, images[f], pixelLabels);
            regionsInFrame.push_back(region);
        }
        consensusVoter.addRegionsByFrame(regionsInFrame);
        std::chrono::steady_clock::time_point flag5 = std::chrono::steady_clock::now();
        std::cout << "Region descriptors computed in " << (std::chrono::duration_cast<std::chrono::microseconds>(flag5-flag4).count())/1000000.0 << " seconds." << '\n';

        consensusVoter.initializeVotesInFrame(f, pixelLabels, SUPERPIXELS_IN_FRAME);
        std::chrono::steady_clock::time_point flag6 = std::chrono::steady_clock::now();
        std::cout << "Initial region votes established in " << (std::chrono::duration_cast<std::chrono::microseconds>(flag6-flag5).count())/1000000.0 << " seconds." << '\n';
        std::cout << '\n';
    }

    // Compute the transition matrix, and reach consensus iteratively multiplying the votes by it
    std::chrono::steady_clock::time_point flag7 = std::chrono::steady_clock::now();
    const int F = parser.get<int>("F");
    const int L = parser.get<int>("L");
    const float sigma2 = parser.get<float>("sigma2");
    consensusVoter.computeTransitionMatrix(F, L, sigma2);
    std::chrono::steady_clock::time_point flag8 = std::chrono::steady_clock::now();
    std::cout << "Transition matrix computed in " << (std::chrono::duration_cast<std::chrono::microseconds>(flag8-flag7).count())/1000000.0 << " seconds." << '\n';

    const int T = parser.get<int>("T");
    consensusVoter.reachConsensus(T);
    std::chrono::steady_clock::time_point flag9 = std::chrono::steady_clock::now();
    std::cout << "Reached consensus in " << (std::chrono::duration_cast<std::chrono::microseconds>(flag9-flag8).count())/1000000.0 << " seconds." << '\n';

    // Get final segmentation from the final votes
    std::vector<cv::Mat> finalMasks;
    const float THRESHOLD = parser.get<float>("threshold");
    const float REMOVE_BLOBS_TH = parser.get<float>("removeBlobsTh");
    consensusVoter.getSegmentation(finalMasks, THRESHOLD, REMOVE_BLOBS_TH);
    std::chrono::steady_clock::time_point flag10 = std::chrono::steady_clock::now();
    std::cout << "Created segmentation from votes in " << (std::chrono::duration_cast<std::chrono::microseconds>(flag10-flag9).count())/1000000.0 << " seconds." << '\n';

    // TEST: reach consensus by batches, so that votes of small blobs are corrected before continuing with the iterations
    // std::chrono::steady_clock::time_point flag9 = std::chrono::steady_clock::now();
    // std::vector<cv::Mat> finalMasks;
    // for(unsigned int i = 0; i < 5; i++) {
    //     consensusVoter.reachConsensus(10);
    //     consensusVoter.getSegmentation(finalMasks, THRESHOLD);
    // }
    // std::chrono::steady_clock::time_point flag10 = std::chrono::steady_clock::now();
    // std::cout << "Reached consensus and segmented video in " << (std::chrono::duration_cast<std::chrono::microseconds>(flag10-flag9).count())/1000000.0 << " seconds." << '\n';

    // Save results
    const std::string fileNameSpContour = "slic";
    tfg::saveMaskedImages(images, spContourMasks, resultsFolder, fileNameSpContour);
    const std::string fileNameMask = "mask";
    tfg::saveMaskedImages(finalMasks, finalMasks, resultsFolder, fileNameMask);
    const std::string fileNameSegmentation = "foreground";
    tfg::saveMaskedImages(images, finalMasks, resultsFolder, fileNameSegmentation);
    const std::string fileNameOverlaidImages = "resultOverlaid";
    tfg::saveOverlaidImages(images, finalMasks, resultsFolder, fileNameOverlaidImages);
    std::chrono::steady_clock::time_point flag11 = std::chrono::steady_clock::now();
    std::cout << "Saved final results in " << (std::chrono::duration_cast<std::chrono::microseconds>(flag11-flag10).count())/1000000.0 << " seconds." << '\n';

    return EXIT_SUCCESS;
}
