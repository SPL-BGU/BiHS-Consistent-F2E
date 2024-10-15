#include <string>
#include <vector>
#include <stdexcept>
#include <iostream>
#include "Driver.h"
#include "ArgParameters.h"
#include "BidirTOH.h"
#include "BidirSTP.h"
#include "BidirMaps.h"
#include "BidirPancake.h"
#include "BidirRoadMap.h"

const std::vector<std::string> SUPPORTED_DOMAINS{"pancake", "toh", "stp", "dao", "road"};

void ParseArguments(ArgParameters &argParameters, std::vector<std::string> clargs) {
    for (int i = 1; i < clargs.size(); ++i) {
        if (clargs[i] == "-d" || clargs[i] == "--domain") {
            ++i;
            if (std::find(SUPPORTED_DOMAINS.begin(), SUPPORTED_DOMAINS.end(), clargs[i]) == SUPPORTED_DOMAINS.end()) {
                throw std::invalid_argument("Unknown domain");
            }
            argParameters.domain = clargs[i];
            std::transform(argParameters.domain.begin(), argParameters.domain.end(), argParameters.domain.begin(),
                           [](unsigned char c) { return std::tolower(c); });
        } else if (clargs[i] == "-a" || clargs[i] == "--algorithms") {
            do {
                ++i;
                argParameters.algs.push_back(clargs[i]);
            } while (i + 1 < clargs.size() && clargs[i + 1][0] != '-');
        } else if (clargs[i] == "-i" || clargs[i] == "--instance") {
            ++i;
            argParameters.instanceId = std::stoi(clargs[i]);

        } else if (clargs[i] == "-n" || clargs[i] == "--number") {
            ++i;
            argParameters.numOfInstances = std::stoi(clargs[i]);

        } else if (clargs[i] == "-h" || clargs[i] == "--heuristic") {
            ++i;
            argParameters.heuristic = clargs[i];
        } else {
            throw std::invalid_argument("Unknown argument identifier encountered: " + clargs[i]);
        }
    }
}

int main(int argc, char *argv[]) {
    std::vector<std::string> clargs(argv, argv + argc);

    ArgParameters argParameters;
    ParseArguments(argParameters, clargs);
    if (argParameters.IsValid()) {
        if (argParameters.domain == "pancake") {
            TestPancake(argParameters);
            return 0;
        } else if (argParameters.domain == "toh") {
            TestTOH(argParameters);
            return 0;
        } else if (argParameters.domain == "stp") {
            TestSTP(argParameters);
            return 0;
        } else if (argParameters.domain == "dao") {
            TestDAO(argParameters);
            return 0;
        } else if (argParameters.domain == "road") {
            TestRoad(argParameters);
            return 0;
        }
        __builtin_unreachable();
    } else {
        return -1;
    }
}
