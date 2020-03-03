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
#include "ImageUtils.h"
#include "CmdParser.h"

int main(int argc, char* argv[]) {

    // Parse command line arguments
    std::vector<OptStruct *> options;
    OptStruct opt_outmodel = {"o:", 0, "./results/nlcsegmentation/", nullptr, "Folder where the results of the segmentation should be stored"}; options.push_back(&opt_outmodel);

    std::vector<ParStruct *> parameters;
    ParStruct par_images = {"images", nullptr, "Text file containing the path to the images to be segmented"}; parameters.push_back(&par_images);

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

    for(unsigned int f = 0; f < images.size(); f++) {
        std::cout << "Frame " << f << ":" << std::endl;

        std::chrono::steady_clock::time_point flag1 = std::chrono::steady_clock::now();
        cv::Ptr<cv::ximgproc::SuperpixelSLIC> sp = cv::ximgproc::createSuperpixelSLIC(images[f], cv::ximgproc::SLICO, 16);
        sp->iterate();
        std::chrono::steady_clock::time_point flag2 = std::chrono::steady_clock::now();

        std::cout << "Superpixels computed in " << (std::chrono::duration_cast<std::chrono::microseconds>(flag2-flag1).count())/1000000.0 << " seconds." << std::endl;

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
        std::chrono::steady_clock::time_point flag3 = std::chrono::steady_clock::now();
        std::cout << "Regions added to RegionList in " << (std::chrono::duration_cast<std::chrono::microseconds>(flag3-flag2).count())/1000000.0 << " seconds." << std::endl;
        std::cout << std::endl;
    }

    std::chrono::steady_clock::time_point flag4 = std::chrono::steady_clock::now();
    regionList.computeDescriptors();
    std::chrono::steady_clock::time_point flag5 = std::chrono::steady_clock::now();
    std::cout << "Grouped descriptors in " << (std::chrono::duration_cast<std::chrono::microseconds>(flag5-flag4).count())/1000000.0 << " seconds." << std::endl;

    Eigen::SparseMatrix<float> transM = regionList.transitionMatrix(15, 4, 0.1);
    std::chrono::steady_clock::time_point flag6 = std::chrono::steady_clock::now();
    std::cout << "Transition matrix computed in " << (std::chrono::duration_cast<std::chrono::microseconds>(flag6-flag5).count())/1000000.0 << " seconds." << std::endl;





    // cv::Mat shrunk_flowu = cv::imread("/home/marco/Projects/TFG/testing/pointtracking/u.i0.tiff", cv::IMREAD_ANYDEPTH);
    // cv::Mat shrunk_flowv = cv::imread("/home/marco/Projects/TFG/testing/pointtracking/v.i0.tiff", cv::IMREAD_ANYDEPTH);
    // auto channels = std::vector<cv::Mat>{shrunk_flowu, shrunk_flowv};
    // cv::Mat shrunk_flow;
    // cv::merge(channels, shrunk_flow);

    // cv::Mat restored_flow;
    // cv::resize(shrunk_flow, restored_flow, cv::Size(854, 480));
    // std::vector<cv::Mat> split_flow;
    // cv::split(restored_flow, split_flow);
    // cv::imwrite("/home/marco/Projects/TFG/testing/pointtracking/restored_u.i0.tiff", split_flow[0]);
    // cv::imwrite("/home/marco/Projects/TFG/testing/pointtracking/restored_v.i0.tiff", split_flow[1]);

    return EXIT_SUCCESS;
}