#include <boost/algorithm/string.hpp>
#include <chrono>
#include "../library/libImage.h"
#include "Track.h"
#include "Homography.h"
#include "ImageUtils.h"

int main(int argc, char* argv[]) {
    std::vector <OptStruct *> options;
    // OptStruct opt_list_points = {"l:", 0, nullptr, nullptr, "File to save list of tracked points"};options.push_back(&opt_list_points);
    OptStruct opt_output = {"o:", 0, "out.txt", nullptr, "File with the weight of each trajectory"}; options.push_back(&opt_output);
    OptStruct opt_images = {"m:", 0, nullptr, nullptr, "File containing image names"}; options.push_back(&opt_images);

    std::vector<ParStruct *> parameters;
    ParStruct pinput = {"input", nullptr, "input file"}; parameters.push_back(&pinput);

    if (!parsecmdline("homography", "Calculating homography between two images", argc, argv, options, parameters))
        return EXIT_FAILURE;


    // Read tracks from text file and put the points
    std::ifstream trackFile(pinput.value);
    std::vector<tfg::Track> tracks;
    tfg::readTracks(trackFile, tracks);
    trackFile.close();

    // homography::pairList<homography::pairList<float>> mappings;
    //std::vector<std::vector<unsigned int>> pointTrajectories;
    std::vector<tfg::Mapping> mappings;
    tfg::getMappings(tracks, mappings);


    const unsigned int NUMBER_OF_FRAMES = mappings.size();
    std::vector<libUSTG::laMatrix> homographies;
    homographies.reserve(NUMBER_OF_FRAMES);

    // Compute first homographies using RANSAC
    for(unsigned int f = 0; f < NUMBER_OF_FRAMES; f++) {
        libUSTG::laMatrix H(3, 3);
        homographies.push_back(H);

        std::vector<float> originX = mappings[f].getOrigin_x();
        std::vector<float> originY = mappings[f].getOrigin_y();
        std::vector<float> destinationX = mappings[f].getDestination_x();
        std::vector<float> destinationY = mappings[f].getDestination_y();

        std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
        libUSTG::compute_ransac_planar_homography_n_points(&(originX[0]), &(originY[0]),
                                                           &(destinationX[0]), &(destinationY[0]),
                                                           originX.size(), 50, 0.1f, homographies[f], nullptr, 1);
        homographies[f] = homographies[f]/homographies[f][2][2];
        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
        std::cout << "(0) Homography " << f << ": " << (std::chrono::duration_cast<std::chrono::microseconds>(end-begin).count())/1000000.0 << " seconds" << std::endl;
        if(f == 0) tfg::printMatrix(homographies[f]);
    }


    // homography::printMappings(mappings);

    // homography::printMatrix(homographies[0]);

    std::vector<float> weights(tracks.size());
    // std::vector<float> residualSq = tfg::getResidualSq(homographies, tracks);
    // tfg::getWeights(residualSq, 4, weights);
    tfg::IRLS(mappings, tracks, homographies, weights);


    // tfg::printVector(weights);
    std::ofstream weightsFile(opt_output.value);
    tfg::writeWeights(weightsFile, weights);



    std::ifstream imagesFile(opt_images.value);
    std::vector<libUSTG::cflimage> iImages;
    tfg::readImages(imagesFile, iImages);
    tfg::paintTracks(tracks, weights, iImages);


    return EXIT_SUCCESS;
}