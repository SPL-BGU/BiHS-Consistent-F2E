#ifndef FRONTTOEND_H
#define FRONTTOEND_H

#include "BidirErrorBucketBasedList.h"
#include "FPUtil.h"
#include <unordered_set>
#include <unordered_map>
#include <iostream>
#include <math.h>
#include <utility>
#include <vector>
#include <queue>


template<class state, class action, class environment, class priorityQueue = BidirErrorBucketBasedList<state, environment, BucketNodeData<state>>>
class FrontToEnd {
public:
    FrontToEnd(double epsilon_ = 1.0) {
        forwardHeuristic = 0;
        backwardHeuristic = 0;
        env = 0;
        ResetNodeCount();
        nodesExpanded = nodesTouched = 0;
        currentCost = DBL_MAX;
        epsilon = epsilon_;
    }

    ~FrontToEnd() {
        forwardQueue.Reset();
        backwardQueue.Reset();
    }

    void GetPath(environment *env, const state &from, const state &to,
                 Heuristic <state> *forward, Heuristic <state> *backward,
                 std::vector <state> &thePath) {
        if (!InitializeSearch(env, from, to, forward, backward, thePath))
            return;

        RunAlgorithm();

//        if (C > currentCost) {
//            std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
//            exit(0);
//        }

        ReconstructSolution(thePath);
    }

    bool InitializeSearch(environment *env, const state &from, const state &to, Heuristic <state> *forward,
                          Heuristic <state> *backward, std::vector <state> &thePath);

    void ExpandBucket(bool forward, const BucketInfo &info);

    virtual const char *GetName() { return "FrontToEnd"; }

    void ResetNodeCount() {
        nodesExpanded = nodesTouched = 0;
        counts.clear();
    }

    uint64_t GetUniqueNodesExpanded() const { return nodesExpanded; }

    uint64_t GetNodesExpanded() const { return nodesExpanded; }

    uint64_t GetNodesTouched() const { return nodesTouched; }

    uint64_t GetNecessaryExpansions() {
        uint64_t necessary = 0;
        for (const auto &count : counts) {
            if (count.first < currentCost)
                necessary += count.second;
        }
        return necessary;
    }

    void Reset() {
        currentCost = DBL_MAX;
        forwardQueue.Reset();
        backwardQueue.Reset();
        ResetNodeCount();
    }

protected:

    bool CheckSolution() { return fgreatereq(C, currentCost); }

    bool ReconstructSolution(std::vector <state> &thePath) {
        std::vector <state> pFor, pBack;
        ExtractPath(backwardQueue, middleNode, pBack);
        ExtractPath(forwardQueue, middleNode, pFor);
        reverse(pFor.begin(), pFor.end());
        thePath = pFor;
        thePath.insert(thePath.end(), pBack.begin() + 1, pBack.end());
    }

    void ExtractPath(const priorityQueue &queue, state &collisionState, std::vector <state> &thePath) {
        thePath.push_back(collisionState);
        auto parent = queue.Lookup(collisionState).parent;
        while (parent != nullptr) {
            thePath.push_back(*parent);
            parent = queue.Lookup(*parent).parent;
        }
    }

    bool Expand(const state *currentState, double g,
                priorityQueue &current, priorityQueue &opposite,
                Heuristic <state> *heuristic, Heuristic <state> *reverseHeuristic,
                const state &target, const state &source);

    priorityQueue forwardQueue, backwardQueue;
    state goal, start;

    uint64_t nodesTouched, nodesExpanded;

    std::map<double, int> counts;

    state middleNode;
    double currentCost;
    double epsilon;

    environment *env;
    Heuristic <state> *forwardHeuristic;
    Heuristic <state> *backwardHeuristic;

    double C = 0.0;

    virtual void RunAlgorithm() = 0;

};

template<class state, class action, class environment, class priorityQueue>
bool FrontToEnd<state, action, environment, priorityQueue>::InitializeSearch(environment *env, const state &from,
                                                                             const state &to,
                                                                             Heuristic <state> *forward,
                                                                             Heuristic <state> *backward,
                                                                             std::vector <state> &thePath) {
    this->env = env;
    forwardHeuristic = forward;
    backwardHeuristic = backward;
    Reset();
    start = from;
    goal = to;
    if (start == goal)
        return false;

    double forwardH = std::max(forwardHeuristic->HCost(start, goal), epsilon);
    double backwardH = std::max(backwardHeuristic->HCost(goal, start), epsilon);

    forwardQueue.setEnvironment(env);
    forwardQueue.AddOpenNode(start, 0, forwardH, 0);
    backwardQueue.setEnvironment(env);
    backwardQueue.AddOpenNode(goal, 0, backwardH, 0);

    C = std::max({forwardH, backwardH, epsilon});

    return true;
}

template<class state, class action, class environment, class priorityQueue>
bool FrontToEnd<state, action, environment, priorityQueue>::Expand(const state *currentState,
                                                                   double g,
                                                                   priorityQueue &current,
                                                                   priorityQueue &opposite,
                                                                   Heuristic <state> *heuristic,
                                                                   Heuristic <state> *reverseHeuristic,
                                                                   const state &target, const state &source) {
    nodesExpanded++;
    counts[C] += 1;

    std::vector <state> neighbors;
    env->GetSuccessors(*currentState, neighbors);

    for (auto &succ : neighbors) {

        nodesTouched++;

        double succG = g + env->GCost(*currentState, succ);

        double h = std::max(heuristic->HCost(succ, target), epsilon);

        // ignore states with greater cost than best solution
        // this can be either g + h
        if (fgreatereq(succG + h, currentCost))
            continue;

        double h_nx = reverseHeuristic->HCost(succ, source);

        // check if there is a collision
        auto collision = opposite.getNodeG(succ, h_nx);
        if (collision.first) {
            auto gValue = collision.second;
            double collisionCost = succG + gValue.second;
            if (fless(collisionCost, currentCost)) {
                currentCost = collisionCost;
                middleNode = succ;

                if (fgreatereq(C, currentCost)) {
                    // add the node so the plan can be extracted
                    current.AddOpenNode(succ, succG, h, h_nx, currentState);
                    break; // step out, don't generate more nodes
                }
            } else if (gValue.first) {
                continue; // if the g value is provably optimal and the collision value is geq, prune the node
            }
        }

        // add it to the open list
        current.AddOpenNode(succ, succG, h, h_nx, currentState);
    }

    return true;
}

template<class state, class action, class environment, class priorityQueue>
void FrontToEnd<state, action, environment, priorityQueue>::ExpandBucket(bool forward, const BucketInfo &info) {
    if (forward) {
        while (!forwardQueue.RemoveIfEmpty(info.g, info.h, info.h_nx)) {
            auto pop = forwardQueue.PopBucket(info.g, info.h, info.h_nx);
            Expand(pop, info.g, forwardQueue, backwardQueue, forwardHeuristic, backwardHeuristic, goal, start);
            if (CheckSolution()) break;
        }
    } else {
        while (!backwardQueue.RemoveIfEmpty(info.g, info.h, info.h_nx)) {
            auto pop = backwardQueue.PopBucket(info.g, info.h, info.h_nx);
            Expand(pop, info.g, backwardQueue, forwardQueue, backwardHeuristic, forwardHeuristic, start, goal);
            if (CheckSolution()) break;
        }
    }
}

#endif
