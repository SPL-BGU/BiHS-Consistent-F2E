#ifndef CONFERENCE_APPS_BIDIRECTIONAL_ARGPARAMETERS_H
#define CONFERENCE_APPS_BIDIRECTIONAL_ARGPARAMETERS_H

#include <utility>
#include <vector>
#include <string>
#include <algorithm>
#include <ostream>

class ArgParameters {
public:
    ArgParameters(std::string domain = "", std::string heuristic = "", const std::vector<std::string> &algs = {},
                  int instanceId = 0, int numOfInstances = 1) : domain(std::move(domain)),
                                                                heuristic(std::move(heuristic)),
                                                                algs(algs), instanceId(instanceId),
                                                                numOfInstances(numOfInstances) {}

    bool IsValid() const {
        if (domain.empty() || heuristic.empty())
            return false;
        if (instanceId < 0 || numOfInstances < 0) {
            return false;
        }
        if (algs.empty()) {
            return false;
        }
        return true;
    }

    bool HasAlgorithm(const std::string &alg) const {
        return std::find(algs.begin(), algs.end(), alg) != algs.end();
    }

    friend std::ostream &operator<<(std::ostream &os, const ArgParameters &parameters) {
        os << "{domain: " << parameters.domain << ", heuristic: " << parameters.heuristic;
        os << ", algs: " << "[";
        for (size_t i = 0; i < parameters.algs.size(); ++i) {
            os << parameters.algs[i];
            if (i != parameters.algs.size() - 1) { os << ", "; }
        }
        os << "], instanceId: " << parameters.instanceId << ", instanceCount: " << parameters.numOfInstances << "}";
        return os;
    }

    std::string domain;
    std::string heuristic;
    std::vector<std::string> algs;
    int instanceId;
    int numOfInstances;
};

#endif //CONFERENCE_APPS_BIDIRECTIONAL_ARGPARAMETERS_H
