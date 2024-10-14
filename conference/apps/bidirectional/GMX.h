//
// Created by Lior Siag on 19/12/2022.
//

#ifndef GMX_H
#define GMX_H

#include <vector>
#include "AStarOpenClosed.h"
#include "Heuristic.h"
#include <climits>
#include <queue>
#include "FPUtil.h"
#include "MaxFlowGraph.h"

template<class state>
class GMX {
public:
    GMX(double epsilon_ = 1) { epsilon = epsilon_; }

    void GenerateBuckets(std::vector<AStarOpenClosedData<state>> &nodes, Heuristic<state> *r_h,
                         state hstate, double solution_cost_, bool forward);

    int GetMinimalExpansions();

    double GetMaxBound(const std::tuple<double, double, double> &gb1, const std::tuple<double, double, double> &gb2);

private:
    std::map<std::tuple<double, double, double>, int> forwardBucketsCounts, backwardBucketsCounts;
    double solution_cost = -1;
    double epsilon;
};

template<class state>
void GMX<state>::GenerateBuckets(std::vector<AStarOpenClosedData<state>> &nodes, Heuristic<state> *r_h,
                                 state hstate, double solution_cost_, bool forward) {
    if (solution_cost > -1 && solution_cost_ != solution_cost)
        assert(!"Different solution costs for different calls");
    solution_cost = solution_cost_;
    for (auto &node: nodes) {
        if (node.where == kClosedList && node.g + node.h < solution_cost) {
            if (forward) {
                forwardBucketsCounts[std::make_tuple(node.g, node.g + node.h,
                                                     node.g - r_h->HCost(node.data, hstate))] += 1;
            } else {
                backwardBucketsCounts[std::make_tuple(node.g, node.g + node.h,
                                                      node.g - r_h->HCost(node.data, hstate))] += 1;
            }
        }
    }
}

template<class state>
int GMX<state>::GetMinimalExpansions() {
    if (forwardBucketsCounts.empty() || backwardBucketsCounts.empty())
        return 0;

    int nodeCount = forwardBucketsCounts.size() + backwardBucketsCounts.size();
    maxflow::Graph<int, int, int> g(nodeCount, 0);
    g.add_node(nodeCount);
    int nodeIdx = 0;

    for (const auto &_: forwardBucketsCounts) {
        g.add_tweights(nodeIdx, _.second, 0);
        nodeIdx += 1;
    }

    for (const auto &_: backwardBucketsCounts) {
        g.add_tweights(nodeIdx, 0, _.second);
        nodeIdx += 1;
    }

    int i = 0;
    for (const auto &fNode: forwardBucketsCounts) {
        int j = forwardBucketsCounts.size();
        for (const auto &bNode: backwardBucketsCounts) {
            if (GetMaxBound(fNode.first, bNode.first) < solution_cost)
                g.add_edge(i, j, INT_MAX, 0);
            j += 1;
        }
        i += 1;
    }
    return g.maxflow();
}

template<class state>
double GMX<state>::GetMaxBound(const std::tuple<double, double, double> &gb1,
                               const std::tuple<double, double, double> &gb2) {
    double gb1rf = 2 * std::get<0>(gb1) - std::get<1>(gb1);
    double gb2rf = 2 * std::get<0>(gb2) - std::get<1>(gb2);
    double gb1rd = 2 * std::get<0>(gb1) - std::get<2>(gb1);
    double gb2rd = 2 * std::get<0>(gb2) - std::get<2>(gb2);


    return std::max({std::get<0>(gb1) + std::get<0>(gb2) + epsilon,
                     std::get<1>(gb1) + std::get<2>(gb2),
                     std::get<2>(gb1) + std::get<1>(gb2),
                     gb1rf + gb2rd,
                     gb1rd + gb2rf});
}

#endif
