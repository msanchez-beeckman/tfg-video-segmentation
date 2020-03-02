#include <iostream>
#include <chrono>
#include <opencv4/opencv2/core.hpp>
#include <opencv4/opencv2/imgproc/imgproc.hpp>
#include <opencv4/opencv2/objdetect/objdetect.hpp>
#include <opencv4/opencv2/highgui/highgui.hpp>
#include <opencv4/opencv2/ximgproc/slic.hpp>
#include <opencv4/opencv2/ml/ml.hpp>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Sparse>
#include "Region.h"
#include "RegionList.h"
#include "ImageUtils.h"
#include "CmdParser.h"

int main(int argc, char* argv[]) {
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
    std::string imageListPath = "/home/marco/CLionProjects/tfg_video_segmentation/data/bear/images.txt";
    std::ifstream imageListFile(imageListPath);
    tfg::readImages(imageListFile, images);
    tfg::RegionList regionList(1590, images.size());

    // /////// Test for frame 0
    // cv::Ptr<cv::ximgproc::SuperpixelSLIC> sp = cv::ximgproc::createSuperpixelSLIC(images[0], cv::ximgproc::SLICO, 16);
    // sp->iterate();
    // int superpixelsInFrame = sp->getNumberOfSuperpixels();
    // std::cout << "Superpixels in frame 0: " << superpixelsInFrame << std::endl;
    // cv::Mat pixelLabels0;
    // sp->getLabels(pixelLabels0);
    // double min, max;
    // cv::minMaxLoc(pixelLabels0, &min, &max);
    // std::cout << "Maximum label in frame 0: " << max << std::endl;

    // std::vector<tfg::Region> regionsInFrame0;
    // for(unsigned int s = 0; s < superpixelsInFrame; s++) {
    //     tfg::Region region(s, 0, images[0], pixelLabels0);
    //     regionsInFrame0.push_back(region);
    // }
    // regionList.addNewFrame(regionsInFrame0);


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


    // cv::Rect boundaries;
    // int patchSize = 15;
    // int xmax = 0;
    // int xmin = labelMask.cols - 1;
    // int ymax = 0;
    // int ymin = labelMask.rows - 1;
    // for(int x = 0; x < labelMask.cols; x++) {
    //     for(int y = 0; y < labelMask.rows; y++) {
    //         unsigned char value = labelMask.at<unsigned char>(y, x);
    //         if(value == 0) continue;
    //         ymin = y < ymin ? y : ymin;
    //         ymax = y > ymax ? y : ymax;
    //         xmin = x < xmin ? x : xmin;
    //         xmax = x > xmax ? x : xmax;
    //     }
    // }

    // boundaries.x = xmin;
    // boundaries.y = ymin;
    // boundaries.width = xmax - xmin + 1;
    // boundaries.height = ymax - ymin + 1;

    // // After delimiting the superpixel, extract a patch around its center
    // // Make sure that the patch is inside the image
    // xmin = static_cast<int>(boundaries.x + (boundaries.width - patchSize)/2);
    // xmin = xmin < 0 ? 0 : xmin;
    // xmax = static_cast<int>(boundaries.x + (boundaries.width + patchSize)/2 + 1);
    // xmax = xmax > labelMask.cols ? labelMask.cols : xmax;

    // ymin = static_cast<int>(boundaries.y + (boundaries.height - patchSize)/2);
    // ymin = ymin < 0 ? 0 : ymin;
    // ymax = static_cast<int>(boundaries.y + (boundaries.height + patchSize)/2 + 1);
    // ymax = ymax > labelMask.rows ? labelMask.rows : ymax;

    // boundaries.x = xmin;
    // boundaries.y = ymin;
    // boundaries.width = xmax - xmin;
    // boundaries.height = ymax - ymin;

    // cv::Mat patch(image, boundaries);

    // // cv::imshow("Patch", patch);
    // // cv::imshow("Masked image", maskedImage);
    // // cv::waitKey(0);

    // std::vector<float> hogDesc;
    // cv::HOGDescriptor hogd(cv::Size(15, 15), cv::Size(15, 15), cv::Size(5, 5), cv::Size(5, 5), 6);
    // hogd.compute(patch, hogDesc);

    // cv::Mat matHOG;
    // matHOG = cv::Mat(54, 1, CV_32FC1, &hogDesc[0]);
    // matHOG = matHOG.reshape(0, 1);
    // std::cout << matHOG << std::endl;

    // float sumaa = 0;
    // for(unsigned int i = 0; i < hogDesc.size(); i++) {
    //     std::cout << hogDesc[i] << std::endl;
    //     sumaa += hogDesc[i];
    // }
    // std::cout << std::endl << sumaa << std::endl;



    // cv::imshow("Test image", maskedImage);
    // cv::waitKey(0);


    //////////// KDTree

    // std::vector<float> test = {0, 1.5f, 0,
    //                            1, 2.1f, 3.0f,
    //                            1, 2, 1.5f,
    //                            2, 0, 0,
    //                            2, 1, 1,
    //                            1, 1.5f, 1,
    //                            0, 2.2f, 1,
    //                            0.5f, 0, 1,
    //                            2, 2, 2,
    //                            1, 1.5f, 2,
    //                            0, 1.9f, 0,
    //                            2, 2, 1.5f};
    // cv::Mat samples(12, 3, CV_32FC1, test.data());
    // std::vector<float> indices = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11};
    // // cv::Mat responses = cv::Mat::ones(12, 1, CV_32FC1);
    // cv::Mat responses(12, 1, CV_32FC1, indices.data());

    // cv::Ptr<cv::ml::KNearest> tree = cv::ml::KNearest::create();
    // tree->train(samples, cv::ml::ROW_SAMPLE, responses);
    // std::cout << "Tree trained" << std::endl;

    // std::vector<float> newPointVec = {0, 0, 0};
    // cv::Mat newPoint(1, 3, CV_32FC1, newPointVec.data());
    // cv::Mat result;
    // cv::Mat neighbourResponses;
    // cv::Mat distances;

    // // Distance is L2 squared
    // tree->findNearest(samples, 3, result, neighbourResponses, distances);

    // std::cout << "NeighbourResponses:" << std::endl;
    // std::cout << neighbourResponses << std::endl << std::endl;

    // std::cout << "Result:" << std::endl;
    // std::cout << result << std::endl << std::endl;

    // std::cout << "Distances:" << std::endl;
    // std::cout << distances << std::endl;

    return EXIT_SUCCESS;
}