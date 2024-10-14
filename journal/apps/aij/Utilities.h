//
// Created by Lior Siag on 16/05/2024.
//

#ifndef HOG2_AIJ_UTILITIES_H
#define HOG2_AIJ_UTILITIES_H

#include <string>
#include <vector>

#define ERROR_MSG(msg) ("[" __FILE__  ":"+std::to_string(__LINE__)+"] " msg)

typedef struct ArgParameters {
    std::string domain;
    std::string alg = "all";
    int instanceId = -1;
    int numOfInstances = 1;
    std::vector<double> weights;
    std::string heuristic;
} ArgParameters;

template <typename T>
int arg_max(std::vector<T> const& vec) {
  return static_cast<int>(std::distance(vec.begin(), max_element(vec.begin(), vec.end())));
}

#endif //HOG2_AIJ_UTILITIES_H
