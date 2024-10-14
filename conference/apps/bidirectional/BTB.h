#ifndef BTB_H
#define BTB_H

#include <unordered_set>
#include <unordered_map>
#include <iostream>
#include <math.h>
#include <utility>
#include <vector>
#include <queue>
#include "FrontToEnd.h"
#include "BidirErrorBucketBasedList.h"

struct PairsInfo {

    PairsInfo(bool forward_, BucketInfo mostConnected_,
              std::pair <BucketInfo, BucketInfo> smallestPair_,
              std::vector <std::pair<bool, BucketInfo>> vertexCover_) :
            forward(forward_), mostConnected(mostConnected_), smallestPair(smallestPair_), vertexCover(vertexCover_) {}

    bool forward;
    BucketInfo mostConnected;
    std::pair <BucketInfo, BucketInfo> smallestPair;
    std::vector <std::pair<bool, BucketInfo>> vertexCover;
};

struct MustExpandEdge {

    MustExpandEdge(BucketInfo fwBucket_, BucketInfo bwBucket_) : fwBucket(fwBucket_), bwBucket(bwBucket_) {}

    BucketInfo fwBucket;
    BucketInfo bwBucket;
};

struct EdgeCompare {
    // comparator is actually "greater than" so the heap is a min-heap
    bool operator()(const MustExpandEdge &e1, const MustExpandEdge &e2) {
        int e1Min = std::min(e1.fwBucket.nodes, e1.bwBucket.nodes);
        int e2Min = std::min(e2.fwBucket.nodes, e2.bwBucket.nodes);

        if (e1Min == e2Min)
            return std::max(e1.fwBucket.nodes, e1.bwBucket.nodes) > std::max(e2.fwBucket.nodes, e2.bwBucket.nodes);

        return e1Min > e2Min;

    }
};

enum BTBPolicy {
    Alternating, Smallest, MostConnected, VertexCover
};

template<class state, class action, class environment, class priorityQueue = BidirErrorBucketBasedList<state, environment, BucketNodeData<state>>>
class BTB : public FrontToEnd<state, action, environment, priorityQueue> {

    using FrontToEnd<state, action, environment, priorityQueue>::forwardQueue;
    using FrontToEnd<state, action, environment, priorityQueue>::backwardQueue;
    using FrontToEnd<state, action, environment, priorityQueue>::forwardHeuristic;
    using FrontToEnd<state, action, environment, priorityQueue>::backwardHeuristic;
    using FrontToEnd<state, action, environment, priorityQueue>::C;
    using FrontToEnd<state, action, environment, priorityQueue>::epsilon;
    using FrontToEnd<state, action, environment, priorityQueue>::start;
    using FrontToEnd<state, action, environment, priorityQueue>::goal;

    using FrontToEnd<state, action, environment, priorityQueue>::Expand;
    using FrontToEnd<state, action, environment, priorityQueue>::ExpandBucket;
    using FrontToEnd<state, action, environment, priorityQueue>::CheckSolution;

public:
    BTB(BTBPolicy policy_, bool useRC_ = true, double epsilon_ = 1.0)
            : FrontToEnd<state, action, environment, priorityQueue>(epsilon_), policy(policy_) ,useRC(useRC_) {}

    ~BTB() {}

protected:

    bool useRC;
    BTBPolicy policy;

    std::pair<bool, PairsInfo> ComputePairs(bool computeVertexCover = false);

    virtual void RunAlgorithm();

};

template<class state, class action, class environment, class priorityQueue>
void BTB<state, action, environment, priorityQueue>::RunAlgorithm() {
    while (!forwardQueue.IsEmpty() && !backwardQueue.IsEmpty()) {
        auto pairComputation = ComputePairs(policy == BTBPolicy::VertexCover);
        if (pairComputation.first) {
            // check solution after increasing C
            if (CheckSolution()) break;

            // TODO think how we are going to parametrize the tie breaker
        }

        const PairsInfo &pairsInfo = pairComputation.second;

        if (policy == BTBPolicy::Alternating) { // alternate directions
            const BucketInfo &fwInfo = pairsInfo.smallestPair.first;
            const BucketInfo &bwInfo = pairsInfo.smallestPair.second;

            while (!forwardQueue.RemoveIfEmpty(fwInfo.g, fwInfo.h, fwInfo.h_nx) &&
                   !backwardQueue.RemoveIfEmpty(bwInfo.g, bwInfo.h, bwInfo.h_nx)) {
                auto fwPop = forwardQueue.PopBucket(fwInfo.g, fwInfo.h, fwInfo.h_nx);
                Expand(fwPop, fwInfo.g, forwardQueue, backwardQueue,
                       forwardHeuristic, backwardHeuristic, goal, start);

                auto bwPop = backwardQueue.PopBucket(bwInfo.g, bwInfo.h, bwInfo.h_nx);
                Expand(bwPop, bwInfo.g, backwardQueue, forwardQueue,
                       backwardHeuristic, forwardHeuristic, start, goal);
            }

        } else if (policy == BTBPolicy::MostConnected) { // expand the bucket with most connected nodes
            ExpandBucket(pairsInfo.forward, pairsInfo.mostConnected);
        } else if (policy == BTBPolicy::Smallest) { // expand the smallest bucket of the smallest pair
            bool forward = pairsInfo.smallestPair.first.nodes < pairsInfo.smallestPair.second.nodes;
            ExpandBucket(forward, forward ? pairsInfo.smallestPair.first : pairsInfo.smallestPair.second);
        } else if (policy == BTBPolicy::VertexCover) { // expand the bucket with most connected nodes
            auto vertexCover = pairsInfo.vertexCover;
            while (!vertexCover.empty()) {
                auto vertex = vertexCover.back();
                vertexCover.pop_back();
                ExpandBucket(vertex.first, vertex.second);
            }
        } else { exit(0); }

        if (CheckSolution()) break;

    }
}

template<class state, class action, class environment, class priorityQueue>
std::pair<bool, PairsInfo>
BTB<state, action, environment, priorityQueue>::ComputePairs(bool computeVertexCover) {

    auto fwBuckets = forwardQueue.getBucketInfo();
    auto bwBuckets = backwardQueue.getBucketInfo();

    double btbC = DBL_MAX;

    // buckets that can be paired with the most nodes
    std::unordered_map<const BucketInfo, int, BucketHash> fwMostConnectedBuckets;
    std::unordered_map<const BucketInfo, int, BucketHash> bwMostConnectedBuckets;

    // arbitrary lower-bound bucket pair
    std::pair <BucketInfo, BucketInfo> smallestBucketPair;

    // TODO split different techniques into different methods, too much code in here
    std::priority_queue <MustExpandEdge, std::vector<MustExpandEdge>, EdgeCompare> edgeHeap;

    for (const auto &fwInfo: fwBuckets) {
        for (const auto &bwInfo: bwBuckets) {
            double fwKK = fwInfo.h - bwInfo.h_nx;
            double bwKK = bwInfo.h - fwInfo.h_nx;

            double distance = std::max({fwKK, bwKK, epsilon});

            if (useRC) {
                double fwRC = bwInfo.h_nx - fwInfo.h;
                double bwRC = fwInfo.h_nx - bwInfo.h;
                distance = std::max({fwRC, bwRC, distance});
            }

            double bound = fwInfo.g + bwInfo.g + distance;

            if (bound < btbC) { // lower the bound, choose the pair and reset the pairing info
                btbC = bound;
                fwMostConnectedBuckets.clear();
                bwMostConnectedBuckets.clear();
                // heaps have no clear method, so reassign it
                edgeHeap = std::priority_queue < MustExpandEdge, std::vector < MustExpandEdge >, EdgeCompare > ();
                smallestBucketPair = std::make_pair(BucketInfo(), BucketInfo());
            }

            if (bound == btbC) {

                fwMostConnectedBuckets[fwInfo] += bwInfo.nodes;
                bwMostConnectedBuckets[bwInfo] += fwInfo.nodes;

                // select  the pair containing the smallest bucket and, among those, the one with overall fewer states
                int num_nodes = std::min(smallestBucketPair.first.nodes, smallestBucketPair.second.nodes);
                if (fwInfo.nodes < num_nodes || bwInfo.nodes < num_nodes) {
                    smallestBucketPair = std::make_pair(fwInfo, bwInfo);
                } else if (fwInfo.nodes == num_nodes && bwInfo.nodes < smallestBucketPair.second.nodes) {
                    smallestBucketPair.second = bwInfo;
                } else if (bwInfo.nodes == num_nodes && fwInfo.nodes < smallestBucketPair.first.nodes) {
                    smallestBucketPair.first = fwInfo;
                }

                if (computeVertexCover) edgeHeap.push(MustExpandEdge(fwInfo, bwInfo));
            }
        }
    }

    auto conn_comp = [](const std::pair<BucketInfo, int> &p1, const std::pair<BucketInfo, int> &p2) {
        return p1.second < p2.second;
    };

    auto fwMostConnected = std::max_element(fwMostConnectedBuckets.begin(), fwMostConnectedBuckets.end(), conn_comp);
    auto bwMostConnected = std::max_element(bwMostConnectedBuckets.begin(), bwMostConnectedBuckets.end(), conn_comp);

    bool forward = fwMostConnected->second >= bwMostConnected->second;
    auto mostConnected = forward ? fwMostConnected->first : bwMostConnected->first;

    std::vector <std::pair<bool, BucketInfo>> vertexCover;
    std::unordered_set <BucketInfo, BucketHash> seenBuckets;
    if (computeVertexCover) {
        while (!edgeHeap.empty()) {
            MustExpandEdge edge = edgeHeap.top();
            edgeHeap.pop();
            if (seenBuckets.find(edge.fwBucket) != seenBuckets.end() ||
                seenBuckets.find(edge.bwBucket) != seenBuckets.end())
                continue;

            bool fw = edge.fwBucket.nodes <= edge.bwBucket.nodes;
            BucketInfo bucket = fw ? edge.fwBucket : edge.bwBucket;
            vertexCover.push_back(std::make_pair(fw, bucket));
        }
    }

    bool updatedC = btbC > C;
    C = btbC;

    return std::make_pair(updatedC, PairsInfo(forward, mostConnected, smallestBucketPair, vertexCover));
}


#endif
