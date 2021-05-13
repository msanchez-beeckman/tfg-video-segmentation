
#include <opencv2/video/tracking.hpp>
#include <opencv4/opencv2/core.hpp>
#include <opencv4/opencv2/imgcodecs.hpp>
#include <vector>
#include <chrono>
#include <iostream>

int main(int argc, char* argv[]) {

    // Parse command line arguments
    const cv::String keys =
        "{h help usage ?     |      | Print usage }"
        "{s speed            | 0    | Speed of the algorithm }"
        "{@image1            |      | First image }"
        "{@image2            |      | Second image }"
        "{@flowu             |      | Output flow in the horizontal direction }"
        "{@flowv             |      | Output flow in the vertical direction }"
        ;
    
    cv::CommandLineParser parser(argc, argv, keys);
    parser.about("Compute TV-L1 optical flow");

    if(parser.has("help") || argc == 1) {
        parser.printMessage();
        return EXIT_SUCCESS;
    }
    if(!parser.check()) {
        parser.printErrors();
        return EXIT_FAILURE;
    }

    // Read the images
    const std::string image1FileName = parser.get<std::string>("@image1");
    const std::string image2FileName = parser.get<std::string>("@image2");
    const cv::UMat image1 = cv::imread(image1FileName, cv::IMREAD_GRAYSCALE).getUMat(cv::ACCESS_READ);
    const cv::UMat image2 = cv::imread(image2FileName, cv::IMREAD_GRAYSCALE).getUMat(cv::ACCESS_READ);

    // Preset
    int preset;
    switch (parser.get<int>("speed")) {
        case 0:
            preset = cv::DISOpticalFlow::PRESET_MEDIUM;
            break;
        case 1:
            preset = cv::DISOpticalFlow::PRESET_FAST;
            break;
        case 2:
            preset = cv::DISOpticalFlow::PRESET_ULTRAFAST;
            break;
        default:
            preset = cv::DISOpticalFlow::PRESET_MEDIUM;
    }

    // Compute flow
    cv::UMat opticalFlow(image1.size(), CV_32FC2, cv::ACCESS_RW);
    std::chrono::steady_clock::time_point beginning = std::chrono::steady_clock::now();
    const cv::Ptr<cv::DISOpticalFlow> dis = cv::DISOpticalFlow::create(preset);
    dis->calc(image1, image2, opticalFlow);
    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    std::cout << "Optical flow computed in " << (std::chrono::duration_cast<std::chrono::microseconds>(end-beginning).count())/1000000.0 << " seconds." << '\n';

    std::vector<cv::UMat> channels;
    cv::split(opticalFlow, channels);

    const std::string flowuFileName = parser.get<std::string>("@flowu");
    const std::string flowvFileName = parser.get<std::string>("@flowv");
    cv::imwrite(flowuFileName, channels[0]);
    cv::imwrite(flowvFileName, channels[1]);

    
    return EXIT_SUCCESS;
}
