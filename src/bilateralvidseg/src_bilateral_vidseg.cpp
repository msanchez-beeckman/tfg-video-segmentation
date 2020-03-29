#include <iostream>
#include <chrono>
#include <opencv4/opencv2/core.hpp>
#include "TrackTable.h"
#include "Grid.h"

int main(int argc, char* argv[]) {
    const int DIMS = 6;
    const std::vector<int> SIZES = {5, 5, 5, 5, 5, 5};
    cv::SparseMat sparseMat(DIMS, SIZES.data(), CV_32FC4);

    std::vector<std::vector<int>> testContainer;
    std::vector<int> index1 = {1, 1, 1, 1, 1, 1}; testContainer.push_back(index1);
    std::vector<int> index2 = {2, 2, 2, 2, 2, 2}; testContainer.push_back(index2);

    cv::Vec4f value; value(0) = 1; value(1) = 2; value(2) = 1; value(3) = 1;
    sparseMat.ref<cv::Vec4f>(testContainer[1].data()) += value;

    const cv::Vec4f& testEntry = sparseMat.ref<cv::Vec4f>(testContainer[1].data());
    std::cout << testEntry << std::endl;

    return 0;

}