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

    // std::vector<cv::Mat> images;
    // std::ifstream imageListFile(par_images.value);
    // tfg::readImages(imageListFile, images);
    // tfg::RegionList regionList(1590, images.size());

    // for(unsigned int f = 0; f < images.size(); f++) {
    //     std::cout << "Frame " << f << ":" << std::endl;

    //     std::chrono::steady_clock::time_point flag1 = std::chrono::steady_clock::now();
    //     cv::Ptr<cv::ximgproc::SuperpixelSLIC> sp = cv::ximgproc::createSuperpixelSLIC(images[f], cv::ximgproc::SLICO, 16);
    //     sp->iterate();
    //     std::chrono::steady_clock::time_point flag2 = std::chrono::steady_clock::now();

    //     std::cout << "Superpixels computed in " << (std::chrono::duration_cast<std::chrono::microseconds>(flag2-flag1).count())/1000000.0 << " seconds." << std::endl;

    //     const int SUPERPIXELS_IN_FRAME = sp->getNumberOfSuperpixels();
    //     cv::Mat pixelLabels;
    //     sp->getLabels(pixelLabels);
    //     std::vector<tfg::Region> regionsInFrame;
    //     regionsInFrame.reserve(SUPERPIXELS_IN_FRAME);
    //     for(int s = 0; s < SUPERPIXELS_IN_FRAME; s++) {
    //         tfg::Region region(s, f, images[f], pixelLabels);
    //         regionsInFrame.push_back(region);
    //     }
    //     regionList.addNewFrame(regionsInFrame);
    //     std::chrono::steady_clock::time_point flag3 = std::chrono::steady_clock::now();
    //     std::cout << "Regions added to RegionList in " << (std::chrono::duration_cast<std::chrono::microseconds>(flag3-flag2).count())/1000000.0 << " seconds." << std::endl;
    //     std::cout << std::endl;
    // }

    // std::chrono::steady_clock::time_point flag4 = std::chrono::steady_clock::now();
    // regionList.computeDescriptors();
    // std::chrono::steady_clock::time_point flag5 = std::chrono::steady_clock::now();
    // std::cout << "Grouped descriptors in " << (std::chrono::duration_cast<std::chrono::microseconds>(flag5-flag4).count())/1000000.0 << " seconds." << std::endl;

    // Eigen::SparseMatrix<float> transM = regionList.transitionMatrix(15, 4, 0.1);
    // std::chrono::steady_clock::time_point flag6 = std::chrono::steady_clock::now();
    // std::cout << "Transition matrix computed in " << (std::chrono::duration_cast<std::chrono::microseconds>(flag6-flag5).count())/1000000.0 << " seconds." << std::endl;


    // cv::Mat flowu = cv::imread("/home/marco/Projects/TFG/testing/pointtracking/u.i0.tiff", cv::IMREAD_ANYDEPTH);
    // cv::Mat flowv = cv::imread("/home/marco/Projects/TFG/testing/pointtracking/v.i0.tiff", cv::IMREAD_ANYDEPTH);
    cv::Mat flowu = cv::imread("/home/marco/Projects/TFG/testing/pointtracking/bmx_flowu0.tiff", cv::IMREAD_ANYDEPTH);
    cv::Mat flowv = cv::imread("/home/marco/Projects/TFG/testing/pointtracking/bmx_flowv0.tiff", cv::IMREAD_ANYDEPTH);

    cv::Mat magnitude;
    cv::Mat orientation;
    cv::cartToPolar(flowu, flowv, magnitude, orientation);
    magnitude = magnitude.reshape(0, 1);
    std::vector<float> magnitudeData;
    magnitude.row(0).copyTo(magnitudeData);

    // Compute the median of the magnitude of the flow
    std::nth_element(magnitudeData.begin(), magnitudeData.begin() + magnitudeData.size()/2, magnitudeData.end());
    float median;
    if(magnitudeData.size()%2 == 1) {
        median = magnitudeData[magnitudeData.size()/2];
    } else {
        auto it = std::max_element(magnitudeData.begin(), magnitudeData.begin() + magnitudeData.size()/2);
        median = (*it + magnitudeData[magnitudeData.size()/2])/2.0f;
    }

    std::cout << "Median: " << median << std::endl;

    // cv::Mat saliency;
    // cv::Mat kernel = cv::Mat::ones(5, 5, CV_32FC1) / (5 * 5);
    // cv::filter2D(magnitude.mul(magnitude), saliency, -1, kernel);

    // std::cout << saliency - magnitude.mul(magnitude) << std::endl;

    int onlyChannel[] = {0};
    int histSize[] = {10};
    float orientationRange[] = {0, 2*CV_PI};
    const float* ranges[] = {orientationRange};
    cv::Mat orientationHistogram;

    cv::calcHist(&orientation, 1, onlyChannel, cv::Mat(), orientationHistogram, 1, histSize, ranges);
    cv::Scalar sumHist = cv::sum(orientationHistogram);
    orientationHistogram /= sumHist(0) + (sumHist(0) == 0);

    std::cout << orientationHistogram << std::endl;
    double minVal, maxVal;
    int minPos[2], maxPos[2];
    cv::minMaxIdx(orientationHistogram, &minVal, &maxVal, &minPos[0], &maxPos[0]);
    std::cout << "Minimum at (" << minPos[0] << ", " << minPos[1] << ") with value " << minVal << std::endl;
    std::cout << "Maximum at (" << maxPos[0] << ", " << maxPos[1] << ") with value " << maxVal << std::endl;

    float classMark = (2*maxPos[0] + 1)*2*CV_PI / (2 * 10);
    std::cout << "Class mark: " << classMark << std::endl;

    cv::Mat angleDeviation = (orientation - classMark).mul(orientation - classMark);
    std::cout << "Mean angle deviation 1: " << cv::mean(angleDeviation) << std::endl;
    cv::min(angleDeviation, (orientation - classMark + 2 * CV_PI).mul(orientation - classMark + 2 * CV_PI), angleDeviation);
    std::cout << "Mean angle deviation 2: " << cv::mean(angleDeviation) << std::endl;
    cv::min(angleDeviation, (orientation - classMark - 2 * CV_PI).mul(orientation - classMark - 2 * CV_PI), angleDeviation);
    std::cout << "Mean angle deviation 3: " << cv::mean(angleDeviation) << std::endl;

    return EXIT_SUCCESS;
}