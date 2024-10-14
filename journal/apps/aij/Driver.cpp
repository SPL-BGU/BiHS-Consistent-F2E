//
// Created by Lior Siag on 06/05/2024.
//

#include <iostream>
#include <vector>
#include <algorithm>
#include "Driver.h"
#include "AijPancake.h"
#include "AijTOH.h"
#include "Utilities.h"
#include "AijSTP.h"
#include "AijMaps.h"
#include "AijRoadmap.h"

const std::vector<std::string> SUPPORTED_DOMAINS{"pancake", "toh", "stp", "dao", "road"};

void ParseArguments(ArgParameters &argParameters, std::vector<std::string> clargs) {
    for (int i = 1; i < clargs.size(); ++i) {
        if (clargs[i] == "-d") {
            ++i;
            if (std::find(SUPPORTED_DOMAINS.begin(), SUPPORTED_DOMAINS.end(), clargs[i]) == SUPPORTED_DOMAINS.end()) {
                throw std::invalid_argument(ERROR_MSG("Unknown domain"));
            }
            argParameters.domain = clargs[i];
        } else if (clargs[i] == "-a") {
            ++i;
            argParameters.alg = clargs[i];

        } else if (clargs[i] == "-i") {
            ++i;
            argParameters.instanceId = std::stoi(clargs[i]);

        } else if (clargs[i] == "-n") {
            ++i;
            argParameters.numOfInstances = std::stoi(clargs[i]);

        } else if (clargs[i] == "-h") {
            ++i;
            argParameters.heuristic = clargs[i];

        } else if (clargs[i] == "-w") {
            do {
                ++i;
                argParameters.weights.push_back(std::stod(clargs[i]));
            } while (i + 1 < clargs.size() && clargs[i + 1][0] != '-');
        } else {
            throw std::invalid_argument(ERROR_MSG("Unknown argument identifier"));
        }
    }
}

int main(int argc, char *argv[]) {
    std::vector<std::string> clargs(argv, argv + argc);
    for (auto &clarg: clargs) {
        std::transform(clarg.begin(), clarg.end(), clarg.begin(), [](unsigned char c) { return std::tolower(c); });
    }

    if (argc == 1 || clargs[1] == "--help") {
        Help();
        return 0;
    }

    if (clargs[1] == "--test") {
        return 0;
    }

    ArgParameters argParameters;
    ParseArguments(argParameters, clargs);
    if (argParameters.domain == "pancake") {
        TestPancake(argParameters);
        return 0;
    } else if (argParameters.domain == "toh") {
        TestToh(argParameters);
        return 0;
    } else if (argParameters.domain == "stp") {
        TestStp(argParameters);
        return 0;
    } else if (argParameters.domain == "dao") {
        TestMaps(argParameters);
        return 0;
    }else if (argParameters.domain == "road") {
        TestRoadmap(argParameters);
        return 0;
    }
    __builtin_unreachable();
}
