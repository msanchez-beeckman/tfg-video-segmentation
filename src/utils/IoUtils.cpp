
#include <sstream>
#include <fstream>
// #include <numeric>
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
        // weights.reserve(NUMBER_OF_ITEMS);
        weights.resize(NUMBER_OF_ITEMS);
        for(unsigned int i = 0; i < NUMBER_OF_ITEMS; i++) {
            std::getline(weightFile, line);
            const double weight = std::stod(line);
            // weights.push_back(weight);
            weights[i] = static_cast<float>(weight);
        }
    }

    void writeWeights(const std::vector<float> &weights, std::ofstream &weightFile) {
        weightFile << weights.size() << " 2" << '\n';
        for(unsigned int i = 0; i < weights.size(); i++) {
            weightFile << weights[i] << " " << 1 - weights[i] << '\n';
        }
    }

    void readWeightsMultilabel(std::ifstream &weightFile, std::vector<std::vector<float>> &weights) {
        std::string line;
        std::getline(weightFile, line);
        std::vector<std::string> firstLineWords;
        tfg::splitString(line, firstLineWords, ' ');
        const int NUMBER_OF_ITEMS = std::stoi(firstLineWords[0]);
        const int NUMBER_OF_LABELS = std::stoi(firstLineWords[1]);
        weights.clear();
        weights.resize(NUMBER_OF_LABELS);
        for(int l = 0; l < NUMBER_OF_LABELS; l++) {
            weights[l].resize(NUMBER_OF_ITEMS);
        }
        for(int i = 0; i < NUMBER_OF_ITEMS; i++) {
            std::getline(weightFile, line);
            std::vector<std::string> labelWeights;
            tfg::splitString(line, labelWeights, ' ');
            for(int l = 0; l < NUMBER_OF_LABELS; l++) {
                const double weight = std::stod(labelWeights[l]);
                weights[l][i] = static_cast<float>(weight);
            }
        }
    }

    void groupLabelWeights(const std::vector<std::vector<float>> &weights, std::vector<float> &groupedWeights, int foregroundLabel) {
        const unsigned int NUMBER_OF_LABELS = weights.size();
        const unsigned int NUMBER_OF_ITEMS = weights[0].size();
        groupedWeights.clear();
        groupedWeights.resize(NUMBER_OF_ITEMS);
        for(unsigned int i = 0; i < NUMBER_OF_ITEMS; i++) {
            float totalWeightSum = 0.0f;
            for(unsigned int l = 0; l < NUMBER_OF_LABELS; l++) {
                if(l == foregroundLabel) continue;
                totalWeightSum += weights[l][i];
            }
            groupedWeights[i] = totalWeightSum;
        }
    }
}