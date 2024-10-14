//
// Created by Lior Siag on 13/05/2024.
//

#ifndef HOG2_AIJ_BAE_H
#define HOG2_AIJ_BAE_H

#include "AStarOpenClosed.h"
#include "FPUtil.h"
#include "Timer.h"
#include <unordered_map>
#include <cmath>
#include <iostream>
#include "Heuristic.h"
#include "MinCriterion.h"
#include <vector>
#include <algorithm>

template<class state>
struct BAECompare {
    bool operator()(const AStarOpenClosedData<state> &i1, const AStarOpenClosedData<state> &i2) const {
        double p1 = i1.h;
        double p2 = i2.h;
        if (fequal(p1, p2)) {
            return (fless(i1.g, i2.g)); // high g-cost over low
        }
        return (fgreater(p1, p2)); // low priority over high
    }
};

template<class state, class action, class environment, class priorityQueue = AStarOpenClosed<state, BAECompare<state>>>
class BAE {
public:
    BAE(SideCriterion sideCriterion_ = SideCriterion::Alt, double epsilon_ = 1.0, double gcd_ = 1.0) {
        forwardHeuristic = 0;
        backwardHeuristic = 0;
        env = 0;
        ResetNodeCount();
        epsilon = epsilon_;
        gcd = gcd_;
        sideCriterion = sideCriterion_;
    }

    virtual ~BAE() {}

    void GetPath(environment *env, const state &from, const state &to,
                 Heuristic<state> *forward, Heuristic<state> *backward, std::vector<state> &thePath);

    bool InitializeSearch(environment *env, const state &from, const state &to,
                          Heuristic<state> *forward, Heuristic<state> *backward, std::vector<state> &thePath);

    bool DoSingleSearchStep(std::vector<state> &thePath);

    virtual const char *GetName() { return "BAE"; }

    void ResetNodeCount() { nodesExpanded = nodesTouched = uniqueNodesExpanded = 0; }

    inline const int GetNumForwardItems() { return forwardQueue.size(); }

    inline const AStarOpenClosedData<state> &GetForwardItem(unsigned int which) { return forwardQueue.Lookat(which); }

    inline const int GetNumBackwardItems() { return backwardQueue.size(); }

    inline const AStarOpenClosedData<state> &GetBackwardItem(unsigned int which) {
        return backwardQueue.Lookat(which);
    }

    uint64_t GetUniqueNodesExpanded() const { return uniqueNodesExpanded; }

    uint64_t GetNodesExpanded() const { return nodesExpanded; }

    uint64_t GetNodesTouched() const { return nodesTouched; }

    uint64_t GetNecessaryExpansions() {
        uint64_t necessary = 0;
        for (const auto &count: counts) {
            if (count.first < currentCost)
                necessary += count.second;
        }
        return necessary;
    }

private:

    void ExtractPathToGoal(state &node, std::vector<state> &thePath) {
        uint64_t theID;
        backwardQueue.Lookup(env->GetStateHash(node), theID);
        ExtractPathToGoalFromID(theID, thePath);
    }

    void ExtractPathToGoalFromID(uint64_t node, std::vector<state> &thePath) {
        do {
            thePath.push_back(backwardQueue.Lookup(node).data);
            node = backwardQueue.Lookup(node).parentID;
        } while (backwardQueue.Lookup(node).parentID != node);
        thePath.push_back(backwardQueue.Lookup(node).data);
    }

    void ExtractPathToStart(state &node, std::vector<state> &thePath) {
        uint64_t theID;
        forwardQueue.Lookup(env->GetStateHash(node), theID);
        ExtractPathToStartFromID(theID, thePath);
    }

    void ExtractPathToStartFromID(uint64_t node, std::vector<state> &thePath) {
        do {
            thePath.push_back(forwardQueue.Lookup(node).data);
            node = forwardQueue.Lookup(node).parentID;
        } while (forwardQueue.Lookup(node).parentID != node);
        thePath.push_back(forwardQueue.Lookup(node).data);
    }

    void Expand(priorityQueue &current,
                priorityQueue &opposite,
                std::unordered_map<double, int> &currentValues,
                Heuristic<state> *heuristic,
                Heuristic<state> *reverse_heuristic,
                const state &target,
                const state &source);

    double getLowerBound() {
        if (forwardQueue.OpenSize() == 0 || backwardQueue.OpenSize() == 0)
            return DBL_MAX;

        double totalErrorForward = forwardQueue.Lookup(forwardQueue.Peek()).h;
        double totalErrorBackward = backwardQueue.Lookup(backwardQueue.Peek()).h;
        double unroundedLowerBound = (totalErrorForward + totalErrorBackward) / 2;

        // round up to the next multiple of gcd
        return ceil(unroundedLowerBound / gcd) * gcd;
    }

    priorityQueue forwardQueue, backwardQueue;
    state goal, start;
    uint64_t nodesTouched, nodesExpanded, uniqueNodesExpanded;
    state middleNode;
    double currentCost;

    std::vector<state> neighbors;
    environment *env;
    Timer t;
    Heuristic<state> *forwardHeuristic;
    Heuristic<state> *backwardHeuristic;

    double epsilon;
    double gcd;

    SideCriterion sideCriterion;
    bool expandForward;

    std::unordered_map<double, int> counts;

    std::unordered_map<double, int> forwardValues;
    std::unordered_map<double, int> backwardValues;
};

template<class state, class action, class environment, class priorityQueue>
void BAE<state, action, environment, priorityQueue>::GetPath(environment *env, const state &from, const state &to,
                                                             Heuristic<state> *forward, Heuristic<state> *backward,
                                                             std::vector<state> &thePath) {
    if (InitializeSearch(env, from, to, forward, backward, thePath) == false)
        return;
    t.StartTimer();
    while (!DoSingleSearchStep(thePath)) {}
}

template<class state, class action, class environment, class priorityQueue>
bool BAE<state, action, environment, priorityQueue>::InitializeSearch(environment *env, const state &from,
                                                                      const state &to,
                                                                      Heuristic<state> *forward,
                                                                      Heuristic<state> *backward,
                                                                      std::vector<state> &thePath) {
    this->env = env;
    forwardHeuristic = forward;
    backwardHeuristic = backward;
    currentCost = DBL_MAX;
    forwardQueue.Reset();
    backwardQueue.Reset();
    ResetNodeCount();
    thePath.resize(0);
    start = from;
    goal = to;
    if (start == goal)
        return false;

    forwardQueue.AddOpenNode(start, env->GetStateHash(start), 0, forwardHeuristic->HCost(start, goal));
    forwardValues[forwardHeuristic->HCost(start, goal)] = 1;
    backwardQueue.AddOpenNode(goal, env->GetStateHash(goal), 0, backwardHeuristic->HCost(goal, start));
    backwardValues[backwardHeuristic->HCost(goal, start)] = 1;

    expandForward = true;
    return true;
}

template<class state, class action, class environment, class priorityQueue>
bool BAE<state, action, environment, priorityQueue>::DoSingleSearchStep(std::vector<state> &thePath) {
    if ((forwardQueue.OpenSize() == 0 || backwardQueue.OpenSize() == 0) && currentCost == DBL_MAX) {
        std::cerr << " !! Problem with no solution?? Expanded: " << nodesExpanded << std::endl;
        exit(0);
    }

    if (currentCost <= getLowerBound()) {
        std::vector<state> pFor, pBack;
        ExtractPathToGoal(middleNode, pBack);
        ExtractPathToStart(middleNode, pFor);
        std::reverse(pFor.begin(), pFor.end());
        thePath = pFor;
        thePath.insert(thePath.end(), pBack.begin() + 1, pBack.end());
        if (thePath.size() >= 2) {
            // Check if the last element is equal to the second to last element
            if (thePath[thePath.size() - 1] == thePath[thePath.size() - 2]) {
                thePath.pop_back(); // Remove the last element
            }
        }
        return true;
    }

    switch (sideCriterion) {
        case SideCriterion::Alt: {
            if (expandForward) {
                Expand(forwardQueue, backwardQueue, forwardValues, forwardHeuristic, backwardHeuristic, goal, start);
                expandForward = false;
            } else {
                Expand(backwardQueue, forwardQueue, backwardValues, backwardHeuristic, forwardHeuristic, start, goal);
                expandForward = true;
            }
            break;
        }
        case SideCriterion::Cardinality: {
            if (forwardQueue.OpenSize() > backwardQueue.OpenSize())
                Expand(backwardQueue, forwardQueue, backwardValues, backwardHeuristic, forwardHeuristic, start, goal);
            else
                Expand(forwardQueue, backwardQueue, forwardValues, forwardHeuristic, backwardHeuristic, goal, start);
            break;
        }
        case SideCriterion::OptCount: {
            if (forwardValues[forwardQueue.Lookat(forwardQueue.Peek()).h] >
                backwardValues[backwardQueue.Lookat(backwardQueue.Peek()).h])
                Expand(backwardQueue, forwardQueue, backwardValues, backwardHeuristic, forwardHeuristic, start, goal);
            else
                Expand(forwardQueue, backwardQueue, forwardValues, forwardHeuristic, backwardHeuristic, goal, start);
            break;
        }
    }

    return false;
}

template<class state, class action, class environment, class priorityQueue>
void BAE<state, action, environment, priorityQueue>::Expand(priorityQueue &current,
                                                            priorityQueue &opposite,
                                                            std::unordered_map<double, int> &currentValues,
                                                            Heuristic<state> *heuristic,
                                                            Heuristic<state> *reverse_heuristic,
                                                            const state &target,
                                                            const state &source) {
    uint64_t nextID;

    bool success = false;
    while (current.OpenSize() > 0) {
        nextID = current.Close();
        currentValues[current.Lookup(nextID).h] -= 1;
        uint64_t reverseLoc;
        auto loc = opposite.Lookup(env->GetStateHash(current.Lookup(nextID).data), reverseLoc);
        if (loc != kClosedList) {
            success = true;
            break;
        }
        // We are doing this lazily. It isn't clear from the BS* paper what it means to remove the
        // descendants. If you have an explicit graph it is handled differently than when the states
        // only exist in open/closed.
//		// 1. if current closed in opposite direction
//		// 1a. Remove descendents of current in open

    }
    if (!success)
        return;

    // 2. Else expand as usual on current direction
    // 2a. Check for bidirectional solution

    bool foundBetterSolution = false;
    nodesExpanded++;

    counts[getLowerBound()] += 1;

    if (current.Lookup(nextID).reopened == false)
        uniqueNodesExpanded++;

    env->GetSuccessors(current.Lookup(nextID).data, neighbors);
    for (auto &succ: neighbors) {
        nodesTouched++;
        uint64_t childID;
        uint64_t hash = env->GetStateHash(succ);
        auto loc = current.Lookup(hash, childID);
        auto &childData = current.Lookup(childID);
        auto &parentData = current.Lookup(nextID);

        double edgeCost = env->GCost(parentData.data, succ);

        // ignore states with greater cost than best solution
        if (fgreatereq(parentData.g + edgeCost, currentCost))
            continue;

        switch (loc) {
            case kClosedList: // ignore
                if (fless(parentData.g + edgeCost, childData.g)) {
                    std::cerr << "  Expanded with non optimal g??????? " << std::endl;
                    exit(0);
                    childData.h = std::max(childData.h, parentData.h - edgeCost);
                    childData.parentID = nextID;
                    childData.g = parentData.g + edgeCost;
                    current.Reopen(childID);
                }
                break;
            case kOpenList: // update cost if needed
            {
                if (fless(parentData.g + edgeCost, childData.g)) {
                    currentValues[childData.h] -= 1;
                    childData.parentID = nextID;
                    double gDiff = childData.g - (parentData.g + edgeCost);
                    childData.g = parentData.g + edgeCost;
                    childData.h = childData.h - (2 * gDiff); // modify total error accordingly
                    currentValues[childData.h] += 1;
                    current.KeyChanged(childID);
                    uint64_t reverseLoc;
                    auto loc = opposite.Lookup(hash, reverseLoc);
                    if (loc == kOpenList) {
                        if (fless(parentData.g + edgeCost + opposite.Lookup(reverseLoc).g, currentCost)) {
                            foundBetterSolution = true;
                            currentCost = parentData.g + edgeCost + opposite.Lookup(reverseLoc).g;
                            middleNode = succ;
                        }
                    }
                }
            }
                break;
            case kNotFound: {
                double g = parentData.g + edgeCost;
                double h = std::max(heuristic->HCost(succ, target), epsilon);

                // Ignore nodes that don't have lower f-cost than the incumbent solution
                if (!fless(g + h, currentCost))
                    break;

                double totalError = (2 * g) + h - reverse_heuristic->HCost(succ, source);

                current.AddOpenNode(succ, hash, g, totalError, nextID);
                currentValues[totalError] += 1;

                // check for solution
                uint64_t reverseLoc;
                auto loc = opposite.Lookup(hash, reverseLoc);
                if (loc == kOpenList) {
                    if (fless(current.Lookup(nextID).g + edgeCost + opposite.Lookup(reverseLoc).g, currentCost)) {
                        foundBetterSolution = true;
                        currentCost = current.Lookup(nextID).g + edgeCost + opposite.Lookup(reverseLoc).g;
                        middleNode = succ;
                    }
                }
            }
        }
    }
}

#endif //HOG2_AIJ_BAE_H
