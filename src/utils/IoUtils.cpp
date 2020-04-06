
#include <sstream>
#include <fstream>
#include "IoUtils.h"

namespace tfg {

    void splitString(const std::string &line, std::vector<std::string> &words, char delimiter) {
        std::istringstream split(line);
        words.clear();
        for(std::string token; std::getline(split, token, delimiter); words.push_back(token));
    }

    void readWeights(std::ifstream &weightFile, std::vector<float> &weights) {
        std::string line;
        std::getline(weightFile, line);
        const int NUMBER_OF_ITEMS = std::stoi(line);
        weights.clear();
        weights.reserve(NUMBER_OF_ITEMS);
        for(unsigned int i = 0; i < NUMBER_OF_ITEMS; i++) {
            std::getline(weightFile, line);
            const float weight = std::stof(line);
            weights.push_back(weight);
        }
    }

    void writeWeights(const std::vector<float> &weights, std::ofstream &weightFile) {
        weightFile << weights.size() << " 2" << std::endl;
        for(unsigned int i = 0; i < weights.size(); i++) {
            weightFile << weights[i] << " " << 1 - weights[i] << std::endl;
        }
    }
}