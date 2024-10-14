#ifndef BASELINE_H
#define BASELINE_H

#include "MinGBucketOpenClosed.h"
#include "FPUtil.h"
#include <unordered_map>
#include <iostream>

template<class state, class action, class environment, class priorityQueue = MinGBucketOpenClosed<state, environment, BucketNodeData<state>>>
class Baseline {
public:
    Baseline(double epsilon_ = 1.0, double gcd_ = 1.0) {
        forwardHeuristic = 0;
        backwardHeuristic = 0;
        env = 0;
        ResetNodeCount();
        expandForward = true;
        nodesExpanded = nodesTouched = 0;
        currentCost = DBL_MAX;
        epsilon = epsilon_;
        gcd = gcd_;
    }

    ~Baseline() {
        forwardQueue.Reset();
        backwardQueue.Reset();
    }

    void GetPath(environment *env, const state &from, const state &to,
                 Heuristic <state> *forward, Heuristic <state> *backward, std::vector <state> &thePath);

    bool InitializeSearch(environment *env, const state &from, const state &to, Heuristic <state> *forward,
                          Heuristic <state> *backward, std::vector <state> &thePath);

    bool DoSingleSearchStep(std::vector <state> &thePath);

    virtual const char *GetName() { return "Baseline"; }

    void ResetNodeCount() {
        nodesExpanded = nodesTouched = 0;
        counts.clear();
    }

    inline const int GetNumForwardItems() { return forwardQueue.size(); }

    inline const BucketNodeData<state> &GetForwardItem(unsigned int which) { return forwardQueue.Lookat(which); }

    inline const int GetNumBackwardItems() { return backwardQueue.size(); }

    inline const BucketNodeData<state> &GetBackwardItem(unsigned int which) {
        return backwardQueue.Lookat(which);
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

private:

    void ExtractPath(const priorityQueue &queue, state &collisionState, std::vector <state> &thePath) {
        thePath.push_back(collisionState);
        auto parent = queue.Lookup(collisionState).parent;
        while (parent != nullptr) {
            thePath.push_back(*parent);
            parent = queue.Lookup(*parent).parent;
        }

    }

    void Expand(priorityQueue &current, priorityQueue &opposite,
                Heuristic <state> *heuristic, Heuristic <state> *reverseHeuristic,
                const state &target, const state &source);

    bool UpdateC();

    priorityQueue forwardQueue, backwardQueue;
    state goal, start;

    uint64_t nodesTouched, nodesExpanded;

    std::map<double, int> counts;

    state middleNode;
    double currentCost;
    double epsilon;
    double gcd;

    environment *env;
    Heuristic <state> *forwardHeuristic;
    Heuristic <state> *backwardHeuristic;

    bool expandForward = true;

    double C = 0.0;

    bool updateByF = true;
};

template<class state, class action, class environment, class priorityQueue>
void Baseline<state, action, environment, priorityQueue>::GetPath(environment *env, const state &from, const state &to,
                                                                  Heuristic <state> *forward,
                                                                  Heuristic <state> *backward,
                                                                  std::vector <state> &thePath) {
    if (!InitializeSearch(env, from, to, forward, backward, thePath))
        return;

    while (!DoSingleSearchStep(thePath)) {}
}

template<class state, class action, class environment, class priorityQueue>
bool Baseline<state, action, environment, priorityQueue>::InitializeSearch(environment *env, const state &from,
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
    expandForward = true;

    double forwardH = std::max(forwardHeuristic->HCost(start, goal), epsilon);
    double backwardH = std::max(backwardHeuristic->HCost(goal, start), epsilon);

    forwardQueue.setEnvironment(env);
    forwardQueue.AddOpenNode(start, 0, forwardH);
    backwardQueue.setEnvironment(env);
    backwardQueue.AddOpenNode(goal, 0, backwardH);

    C = std::max(forwardH, backwardH);

    return true;
}

template<class state, class action, class environment, class priorityQueue>
bool Baseline<state, action, environment, priorityQueue>::UpdateC() {

    bool updated = false;

    auto fMinf = forwardQueue.getMinF();
    auto fMinb = backwardQueue.getMinF();
    auto fMin = std::max(fMinf, fMinb);

    if (C < fMin) {
        C = fMin;
        updated = true;
    }

    auto minGF = forwardQueue.getMinG(C);
    auto minGB = backwardQueue.getMinG(C);
    auto gBound = minGF + minGB + epsilon;

    while (C < gBound) {
        C += gcd;
        gBound = forwardQueue.getMinG(C) + backwardQueue.getMinG(C) + epsilon;
        updated = true;
    }

    return updated;
}

template<class state, class action, class environment, class priorityQueue>
bool Baseline<state, action, environment, priorityQueue>::DoSingleSearchStep(std::vector <state> &thePath) {

    if (UpdateC()) {
        // TODO think how we are going to parametrize the tie breaker
    }

    if (fgreatereq(C, currentCost)) { // optimal solution found
        std::vector <state> pFor, pBack;
        ExtractPath(backwardQueue, middleNode, pBack);
        ExtractPath(forwardQueue, middleNode, pFor);
        reverse(pFor.begin(), pFor.end());
        thePath = pFor;
        thePath.insert(thePath.end(), pBack.begin() + 1, pBack.end());
        return true;
    }


    if (expandForward) {
        Expand(forwardQueue, backwardQueue, forwardHeuristic, backwardHeuristic, goal, start);
        expandForward = false;
    } else {
        Expand(backwardQueue, forwardQueue, backwardHeuristic, forwardHeuristic, start, goal);
        expandForward = true;
    }
    return false;
}

template<class state, class action, class environment, class priorityQueue>
void Baseline<state, action, environment, priorityQueue>::Expand(priorityQueue &current, priorityQueue &opposite,
                                                                 Heuristic <state> *heuristic,
                                                                 Heuristic <state> *reverseHeuristic,
                                                                 const state &target, const state &source) {
    // keep popping until you get a valid node
    auto nodePair = current.Pop(C);
    while (nodePair.first == nullptr) nodePair = current.Pop(C);

    const auto node = nodePair.first;
    auto nodeG = nodePair.second;
    nodesExpanded++;
    counts[C] += 1;

    std::vector <state> neighbors;
    env->GetSuccessors(*node, neighbors);

    for (auto &succ : neighbors) {

        nodesTouched++;

        double succG = nodeG + env->GCost(*node, succ);

        double h = std::max(heuristic->HCost(succ, target), epsilon);

        // ignore states with greater cost than best solution
        // this can be either g + h
        if (succG + h >= currentCost)
            continue;

        // check if there is a collision
        // TODO: pass a lambda in to avoid the heuristic evaluation
        auto collision = opposite.getNodeG(succ, reverseHeuristic->HCost(succ, source));
        if (collision.first) {
            auto gValue = collision.second;
            double collisionCost = succG + gValue.second;
            if (fless(collisionCost, currentCost)) {
                currentCost = collisionCost;
                middleNode = succ;

                if (fgreatereq(C, currentCost)) {
                    current.AddOpenNode(succ, succG, h, node); // add the node so the plan can be extracted
                    break; // step out, don't generate more nodes
                }
            } else if (gValue.first) {
                continue; // if the g value is provably optimal and the collision value is geq, prune the node
            }
        }

        // add it to the open list
        current.AddOpenNode(succ, succG, h, node);
    }
}

#endif /* Baseline_h */
