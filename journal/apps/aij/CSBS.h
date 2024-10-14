//
// Created by Lior Siag on 13/05/2024.
//

#ifndef HOG2_AIJ_CSBS_H
#define HOG2_AIJ_CSBS_H


#include "AStarOpenClosed.h"
#include "FPUtil.h"
#include "Timer.h"
#include <unordered_map>
#include <cmath>
#include <iostream>
#include "Heuristic.h"
#include <vector>
#include <algorithm>

template<class state>
struct CSBSCompare {
    bool operator()(const AStarOpenClosedData<state> &i1, const AStarOpenClosedData<state> &i2) const {
        double p1 = i1.h;
        double p2 = i2.h;
        if (fequal(p1, p2)) {
            return (fless(i1.g, i2.g)); // high g-cost over low
        }
        return (fgreater(p1, p2)); // low priority over high
    }
};

template<class state, class action, class environment, class priorityQueue = AStarOpenClosed<state, CSBSCompare<state>>>
class CSBS {
public:
    CSBS(const std::vector<double> &lbWeights, bool alternating_ = true, double epsilon_ = 1.0, double gcd_ = 1.0) {
        /*
         * The weights are the weights for the priority function in accordance to the lbc.
         * This means that the first weight is for hD, the second if for HDr, and the last is of epsilon.
         */
        forwardHeuristic = 0;
        backwardHeuristic = 0;
        env = 0;
        ResetNodeCount();
        epsilon = epsilon_;
        gcd = gcd_;
        alternating = alternating_;
        // Check that there are 3 weights for each side and that their sum is 1
        assert(lbWeights.size() == 3 && fequal(lbWeights[0] + lbWeights[1] + lbWeights[2], 1));
        this->lbWeights = lbWeights;
        this->forwardWeights = {lbWeights[0], lbWeights[1]};
        this->backwardWeights = {lbWeights[1], lbWeights[0]};
        this->silent = true;
    }

    virtual ~CSBS() {}

    void SetSilent(bool b) {
        this->silent = b;
    }

    void GetPath(environment *env, const state &from, const state &to,
                 Heuristic<state> *forward, Heuristic<state> *backward, std::vector<state> &thePath);

    bool InitializeSearch(environment *env, const state &from, const state &to,
                          Heuristic<state> *forward, Heuristic<state> *backward, std::vector<state> &thePath);

    bool DoSingleSearchStep(std::vector<state> &thePath);

    std::string GetName() {
        std::string w1 = std::to_string(this->lbWeights[0]);
        w1.erase(w1.find_last_not_of('0') + 1, std::string::npos);
        w1.erase(w1.find_last_not_of('.') + 1, std::string::npos);

        std::string w2 = std::to_string(this->lbWeights[1]);
        w2.erase(w2.find_last_not_of('0') + 1, std::string::npos);
        w2.erase(w2.find_last_not_of('.') + 1, std::string::npos);

        std::string w3 = std::to_string(this->lbWeights[2]);
        w3.erase(w3.find_last_not_of('0') + 1, std::string::npos);
        w3.erase(w3.find_last_not_of('.') + 1, std::string::npos);

        return "CSBS-" + w1 + "-" + w2 + "-" + w3;
    }

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

    void OpenGLDraw() const;

    //	void SetWeight(double w) {weight = w;}
private:

    void Nip(const state &, priorityQueue &reverse);

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

    void OpenGLDraw(const priorityQueue &queue) const;

    void Expand(priorityQueue &current,
                priorityQueue &opposite,
                Heuristic<state> *heuristic,
                Heuristic<state> *reverse_heuristic,
                const state &target,
                const state &source,
                const std::vector<double> &priorityWeights);

    double getLowerBound() {
        if (forwardQueue.OpenSize() == 0 || backwardQueue.OpenSize() == 0)
            return DBL_MAX;

        double forwardPriority = forwardQueue.Lookup(forwardQueue.Peek()).h;
        double backwardPriority = backwardQueue.Lookup(backwardQueue.Peek()).h;
        double unroundedLowerBound = forwardPriority + backwardPriority + this->lbWeights[2] * this->epsilon;

        // Remove an epsilon from the lower bound in a case where we get something bigger than the int of the bound due
        // to error in float representation. E.g., problem 35, GAP-3, with weights:
        // '0.32903704771636516', '0.29775755719222274', '0.37320539509141215'
        unroundedLowerBound -= 0.00000001;
        return ceil(unroundedLowerBound / gcd) * gcd;
    }

    double getPriority(double g, double h, double hr, const std::vector<double> &weights) {
        return g + weights[0] * h - weights[1] * hr;
    }

    priorityQueue forwardQueue, backwardQueue;
    state goal, start;
//	std::unordered_map<std::pair<double, double>, int> dist;
//	std::unordered_map<std::pair<double, double>, int> f, b;
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

    bool alternating;
    bool expandForward;

    std::unordered_map<double, int> counts;
    std::vector<double> lbWeights;
    std::vector<double> forwardWeights;
    std::vector<double> backwardWeights;

    double lastLowerBound;

    bool silent;
};

template<class state, class action, class environment, class priorityQueue>
void CSBS<state, action, environment, priorityQueue>::GetPath(environment *env, const state &from, const state &to,
                                                              Heuristic<state> *forward, Heuristic<state> *backward,
                                                              std::vector<state> &thePath) {
    if (InitializeSearch(env, from, to, forward, backward, thePath) == false)
        return;
    t.StartTimer();
    while (!DoSingleSearchStep(thePath)) {}
}

template<class state, class action, class environment, class priorityQueue>
bool CSBS<state, action, environment, priorityQueue>::InitializeSearch(environment *env, const state &from,
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
    this->lastLowerBound = 0;
    ResetNodeCount();
    thePath.resize(0);
    start = from;
    goal = to;
    if (start == goal)
        return false;

    forwardQueue.AddOpenNode(start, env->GetStateHash(start), 0,
                             this->getPriority(0, forwardHeuristic->HCost(start, goal), 0, this->forwardWeights));
    backwardQueue.AddOpenNode(goal, env->GetStateHash(goal), 0,
                              this->getPriority(0, backwardHeuristic->HCost(goal, start), 0, this->backwardWeights));

    expandForward = true;
    return true;
}

template<class state, class action, class environment, class priorityQueue>
bool CSBS<state, action, environment, priorityQueue>::DoSingleSearchStep(std::vector<state> &thePath) {
    if ((forwardQueue.OpenSize() == 0 || backwardQueue.OpenSize() == 0) && currentCost == DBL_MAX) {
        std::cerr << " !! Problem with no solution?? Expanded: " << nodesExpanded << std::endl;
        exit(0);
    }

    double lb = getLowerBound();
    if (fgreater(lb, this->lastLowerBound)) {
        this->lastLowerBound = lb;
        if (!this->silent) {
            std::cout << "[B] NE=" << this->nodesExpanded << " lb=" << lb << "\n";
        }
    }
    if (currentCost <= lb) {
        std::vector<state> pFor, pBack;
        ExtractPathToGoal(middleNode, pBack);
        ExtractPathToStart(middleNode, pFor);
        std::reverse(pFor.begin(), pFor.end());
        thePath = pFor;
        thePath.insert(thePath.end(), pBack.begin() + 1, pBack.end());

        return true;
    }

    if (alternating) { // original BAE* definition
        if (expandForward) {
            Expand(forwardQueue, backwardQueue, forwardHeuristic, backwardHeuristic, goal, start, forwardWeights);
            expandForward = false;
        } else {
            Expand(backwardQueue, forwardQueue, backwardHeuristic, forwardHeuristic, start, goal, backwardWeights);
            expandForward = true;
        }
    } else { // BS* policy, roughly Pohl's criterion
        if (forwardQueue.OpenSize() > backwardQueue.OpenSize())
            Expand(backwardQueue, forwardQueue, backwardHeuristic, forwardHeuristic, start, goal, backwardWeights);
        else
            Expand(forwardQueue, backwardQueue, forwardHeuristic, backwardHeuristic, goal, start, forwardWeights);
    }

    return false;
}

template<class state, class action, class environment, class priorityQueue>
void CSBS<state, action, environment, priorityQueue>::Expand(priorityQueue &current,
                                                             priorityQueue &opposite,
                                                             Heuristic<state> *heuristic,
                                                             Heuristic<state> *reverse_heuristic,
                                                             const state &target,
                                                             const state &source,
                                                             const std::vector<double> &priorityWeights) {
    uint64_t nextID;
    double lb = DBL_MAX;
    bool success = false;
    while (current.OpenSize() > 0) {
        lb = getLowerBound();
        nextID = current.Close();
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
//		Nip(current.Lookup(nextID).data, opposite);
    }
    if (!success)
        return;

    // 2. Else expand as usual on current direction
    // 2a. Check for bidirectional solution

    bool foundBetterSolution = false;
    nodesExpanded++;

    counts[lb] += 1;


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
                    childData.parentID = nextID;
                    double gDiff = childData.g - (parentData.g + edgeCost);
                    childData.g = parentData.g + edgeCost;
                    childData.h = childData.h - gDiff; // modify priority function
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

                // Ignore nodes that don't have lower f-cost than the incumbant solution
                if (!fless(g + h, currentCost))
                    break;

                double totalError = this->getPriority(g, h, reverse_heuristic->HCost(succ, source), priorityWeights);

                current.AddOpenNode(succ, // This may invalidate our references
                                    hash,
                                    g,
                                    totalError,
                                    nextID);

                // check for solution
                uint64_t reverseLoc;
                auto loc = opposite.Lookup(hash, reverseLoc);
                if (loc == kOpenList) {
                    if (fless(current.Lookup(nextID).g + edgeCost + opposite.Lookup(reverseLoc).g, currentCost)) {
                        foundBetterSolution = true;
//						printf("  Potential solution found, cost: %1.2f + %1.2f = %1.2f\n",
//							   current.Lookup(nextID).g+edgeCost,
//							   opposite.Lookup(reverseLoc).g,
//							   current.Lookup(nextID).g+edgeCost+opposite.Lookup(reverseLoc).g);
//                        std::cout << "  Lower bound: " << getLowerBound() << std::endl;
                        currentCost = current.Lookup(nextID).g + edgeCost + opposite.Lookup(reverseLoc).g;
                        middleNode = succ;
//						PrintOpenStats(f);
//						PrintOpenStats(b);
                    }
                }
            }
        }
    }
}

template<class state, class action, class environment, class priorityQueue>
void CSBS<state, action, environment, priorityQueue>::Nip(const state &s, priorityQueue &queue) {
    assert(!"Not using this code currently - the correct implementation of 'remove' is unclear from BS*");
    // At this point parent has been removed from open
    // Need to find any successors that have a parent id of parent & recursively remove them from open

    std::vector<state> n;

    uint64_t parentID;
    auto loc = queue.Lookup(env->GetStateHash(s), parentID);
    assert(loc == kClosedList);
    env->GetSuccessors(s, n);
    for (auto &succ: n) {
        uint64_t childID;
        uint64_t hash = env->GetStateHash(succ);
        auto loc = queue.Lookup(hash, childID);
        auto &childData = queue.Lookup(childID);
        if (loc == kClosedList && childData.parentID == parentID) {
            Nip(childData.data, queue);
        } else if (loc == kOpenList && (childData.parentID == parentID))// && (childData.data != middleNode))
        {
            if (childData.data == middleNode) {
                std::cout << "Error - removing middle node\n";
                if (&queue == &forwardQueue)
                    std::cout << "In backward search - removing from for\n";
                else
                    std::cout << "In forward search - removing from back\n";
                std::cout << "Parent: " << s << "\n";
                std::cout << "Middle: " << middleNode << "\n";
                std::vector<state> pFor, pBack, final;
                ExtractPathToGoal(middleNode, pBack);
                ExtractPathToStart(middleNode, pFor);
                reverse(pFor.begin(), pFor.end());
                std::cout << "Path forward: \n";

                for (auto &s: pFor)
                    std::cout << s << "\n";
                std::cout << "Path backward: \n";
                for (auto &s: pBack)
                    std::cout << s << "\n";

                exit(0);
            }
            queue.Remove(env->GetStateHash(childData.data));
        }
    }
}

template<class state, class action, class environment, class priorityQueue>
void CSBS<state, action, environment, priorityQueue>::OpenGLDraw() const {
    OpenGLDraw(forwardQueue);
    OpenGLDraw(backwardQueue);
}

template<class state, class action, class environment, class priorityQueue>
void CSBS<state, action, environment, priorityQueue>::OpenGLDraw(const priorityQueue &queue) const {
    double transparency = 0.9;
    if (queue.size() == 0)
        return;
    uint64_t top = -1;
    //	double minf = 1e9, maxf = 0;
    if (queue.OpenSize() > 0) {
        top = queue.Peek();
    }
    for (unsigned int x = 0; x < queue.size(); x++) {
        const AStarOpenClosedData<state> &data = queue.Lookat(x);
        if (x == top) {
            env->SetColor(1.0, 1.0, 0.0, transparency);
            env->OpenGLDraw(data.data);
        }
        if ((data.where == kOpenList) && (data.reopened)) {
            env->SetColor(0.0, 0.5, 0.5, transparency);
            env->OpenGLDraw(data.data);
        } else if (data.where == kOpenList) {
            env->SetColor(0.0, 1.0, 0.0, transparency);
            env->OpenGLDraw(data.data);
        } else if ((data.where == kClosedList) && (data.reopened)) {
            env->SetColor(0.5, 0.0, 0.5, transparency);
            env->OpenGLDraw(data.data);
        } else if (data.where == kClosedList) {
            env->SetColor(1.0, 0.0, 0.0, transparency);
            env->OpenGLDraw(data.data);
        }
    }
}

#endif //HOG2_AIJ_CSBS_H
