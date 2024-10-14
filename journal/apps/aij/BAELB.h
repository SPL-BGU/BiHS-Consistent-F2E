#ifndef BAELB_h
#define BAELB_h

#include "AStarMultiOpenClosed.h"
#include "FPUtil.h"
#include <cmath>
#include <map>
#include <iostream>
#include "Heuristic.h"
#include "MinCriterion.h"

/**
 * An ordering of the different node values we are tracking.
 * This enum must start at 0 and end at length-1 since we rely on the creation of a vector of the enum's size, and
 * populating it based on enum positions.
 */
enum nodeValue {
    ivG,
    ivF,
    ivRF,
    ivD,
    ivRD,
    ivB,
    ivGF,
    ivGD,
    ivFRD,
    ivRFD,
    ivRFRD,
    ivGRF,
    ivGRD,
    ivGB,
    ivGFRD,
    ivGRFD,
    ivGRFRD,
};
const size_t NODEVALUES_NR_ITEMS = 17;

/**
 * An ordering of the different bounds.
 * This enum must start at 0 and end at length-1.
 */
enum BoundValue {
    lbFD,
    lbRFRD,
    lbDF,
    lbRDRF,
    lbG,
    lbB,
    lbFRDRFD,
    lbGFGD,
    lbRFDFRD,
    lbRFRDX2,
    lbGRFGRD,
    lbGDGF,
    lbGRDGRF,
    lbGB,
    lbGFRDGRFD,
    lbGRFDGFRD,
    lbGRFRD

};
const size_t BOUNDVALUES_NR_ITEMS = 17;

/**
 * A vector of bound name and enum pairs for easier printing.
 */
const std::vector<std::pair<std::string, int>> BOUND_NAME_ENUM_PAIRS{
        std::pair<std::string, int>("FD", lbFD),
        std::pair<std::string, int>("RFRD", lbRFRD),
        std::pair<std::string, int>("DF", lbDF),
        std::pair<std::string, int>("RDRF", lbRDRF),
        std::pair<std::string, int>("G", lbG),
        std::pair<std::string, int>("B", lbB),
        std::pair<std::string, int>("FRDRFD", lbFRDRFD),
        std::pair<std::string, int>("GFGD", lbGFGD),
        std::pair<std::string, int>("RFDFRD", lbRFDFRD),
        std::pair<std::string, int>("RFRDX2", lbRFRDX2),
        std::pair<std::string, int>("GRFGRD", lbGRFGRD),
        std::pair<std::string, int>("GDGF", lbGDGF),
        std::pair<std::string, int>("GRDGRF", lbGRDGRF),
        std::pair<std::string, int>("GB", lbGB),
        std::pair<std::string, int>("GFRDGRFD", lbGFRDGRFD),
        std::pair<std::string, int>("GRFDGFRD", lbGRFDGFRD),
        std::pair<std::string, int>("GRFRD", lbGRFRD),
};


/**
 * Node value comparators, each one is responsible for a different value we are tracking.
 * When creating a comparator for this code (and hog2 in general), i1 is the parent and i2 is the child.
 * We return true, if there needs to be a switch between them in HeapifyUp() based on their respective priorities.
 * E.g., GCompare asks does the parent have higher g value than the child, if so, switch them as we want to track the
 * node with minimal G value.
 */


template<typename state>
struct GCompare : NodeCompare<state> {
    bool
    operator()(const AStarMultiOpenClosedData<state> &i1, const AStarMultiOpenClosedData<state> &i2) const override {
        double p1 = i1.g;
        double p2 = i2.g;
        if (fequal(p1, p2)) {
            return (fgreater(i1.h, i2.h)); // Low h over high h
        }
        return fgreater(p1, p2); // Low g over high g
    }
};

template<typename state>
struct FCompare : NodeCompare<state> {
    bool
    operator()(const AStarMultiOpenClosedData<state> &i1, const AStarMultiOpenClosedData<state> &i2) const override {
        double p1 = i1.g + i1.h;
        double p2 = i2.g + i2.h;
        if (fequal(p1, p2)) {
            return (fless(i1.g, i2.g)); // High g-cost over low
        }
        return fgreater(p1, p2); // Low f over high f
    }
};

template<typename state>
struct RFCompare : NodeCompare<state> {
    bool
    operator()(const AStarMultiOpenClosedData<state> &i1, const AStarMultiOpenClosedData<state> &i2) const override {
        double p1 = i1.g - i1.h;
        double p2 = i2.g - i2.h;
        if (fequal(p1, p2)) {
            return (fless(i1.g, i2.g)); // High g-cost over low
        }
        return fgreater(p1, p2); // // Low rf over high
    }
};

template<typename state>
struct DCompare : NodeCompare<state> {
    bool
    operator()(const AStarMultiOpenClosedData<state> &i1, const AStarMultiOpenClosedData<state> &i2) const override {
        double p1 = i1.g - i1.rh;
        double p2 = i2.g - i2.rh;
        if (fequal(p1, p2)) {
            return (fless(i1.g, i2.g)); // High g-cost over low
        }
        return fgreater(p1, p2); // Low d over high d
    }
};

template<typename state>
struct RDCompare : NodeCompare<state> {
    bool
    operator()(const AStarMultiOpenClosedData<state> &i1, const AStarMultiOpenClosedData<state> &i2) const override {
        double p1 = i1.g + i1.rh;
        double p2 = i2.g + i2.rh;
        if (fequal(p1, p2)) {
            return (fless(i1.g, i2.g)); // High g-cost over low
        }
        return fgreater(p1, p2); // // Low rd over high
    }
};

template<class state>
struct BCompare : NodeCompare<state> { // This is BAECompare
    bool operator()(const AStarMultiOpenClosedData<state> &i1, const AStarMultiOpenClosedData<state> &i2) const {
        double p1 = 2 * i1.g + i1.h - i1.rh;
        double p2 = 2 * i2.g + i2.h - i2.rh;
        if (fequal(p1, p2)) {
            return (fless(i1.g, i2.g)); // High g-cost over low
        }
        return (fgreater(p1, p2)); // Low b over high
    }
};

template<typename state>
struct GFCompare : NodeCompare<state> {
    bool
    operator()(const AStarMultiOpenClosedData<state> &i1, const AStarMultiOpenClosedData<state> &i2) const override {
        double p1 = 2 * i1.g + i1.h;
        double p2 = 2 * i2.g + i2.h;
        if (fequal(p1, p2)) {
            return (fless(i1.g, i2.g)); // High g-cost over low
        }
        return fgreater(p1, p2); // Low g+f over high
    }
};

template<typename state>
struct GDCompare : NodeCompare<state> {
    bool
    operator()(const AStarMultiOpenClosedData<state> &i1, const AStarMultiOpenClosedData<state> &i2) const override {
        double p1 = 2 * i1.g - i1.rh;
        double p2 = 2 * i2.g - i2.rh;
        if (fequal(p1, p2)) {
            return (fless(i1.g, i2.g)); // High g-cost over low
        }
        return fgreater(p1, p2); // Low g+d over high
    }
};

template<typename state>
struct FRDCompare : NodeCompare<state> {
    bool
    operator()(const AStarMultiOpenClosedData<state> &i1, const AStarMultiOpenClosedData<state> &i2) const override {
        double p1 = 2 * i1.g + i1.h + i1.rh;
        double p2 = 2 * i2.g + i2.h + i2.rh;
        if (fequal(p1, p2)) {
            return (fless(i1.g, i2.g)); // High g-cost over low
        }
        return fgreater(p1, p2); // // Low f+rd over high
    }
};

template<typename state>
struct RFDCompare : NodeCompare<state> {
    bool
    operator()(const AStarMultiOpenClosedData<state> &i1, const AStarMultiOpenClosedData<state> &i2) const override {
        double p1 = 2 * i1.g - i1.h - i1.rh;
        double p2 = 2 * i2.g - i2.h - i2.rh;
        if (fequal(p1, p2)) {
            return (fless(i1.g, i2.g)); // High g-cost over low
        }
        return fgreater(p1, p2); // // Low rf+d over high
    }
};

template<typename state>
struct RFRDCompare : NodeCompare<state> {
    bool
    operator()(const AStarMultiOpenClosedData<state> &i1, const AStarMultiOpenClosedData<state> &i2) const override {
        double p1 = 2 * i1.g - i1.h + i1.rh;
        double p2 = 2 * i2.g - i2.h + i2.rh;
        if (fequal(p1, p2)) {
            return (fless(i1.g, i2.g)); // High g-cost over low
        }
        return fgreater(p1, p2); // // Low rf+rd over high
    }
};

template<typename state>
struct GRFCompare : NodeCompare<state> {
    bool
    operator()(const AStarMultiOpenClosedData<state> &i1, const AStarMultiOpenClosedData<state> &i2) const override {
        double p1 = 2 * i1.g - i1.h;
        double p2 = 2 * i2.g - i2.h;
        if (fequal(p1, p2)) {
            return (fless(i1.g, i2.g)); // High g-cost over low
        }
        return fgreater(p1, p2); // // Low g+rf over high
    }
};

template<typename state>
struct GRDCompare : NodeCompare<state> {
    bool
    operator()(const AStarMultiOpenClosedData<state> &i1, const AStarMultiOpenClosedData<state> &i2) const override {
        double p1 = 2 * i1.g + i1.rh;
        double p2 = 2 * i2.g + i2.rh;
        if (fequal(p1, p2)) {
            return (fless(i1.g, i2.g)); // High g-cost over low
        }
        return fgreater(p1, p2); // // Low g+rd over high
    }
};

template<typename state>
struct GBCompare : NodeCompare<state> {
    bool
    operator()(const AStarMultiOpenClosedData<state> &i1, const AStarMultiOpenClosedData<state> &i2) const override {
        double p1 = 3 * i1.g + i1.h - i1.rh;
        double p2 = 3 * i2.g + i2.h - i2.rh;
        if (fequal(p1, p2)) {
            return (fless(i1.g, i2.g)); // High g-cost over low
        }
        return fgreater(p1, p2); // // Low g+b over high
    }
};

template<typename state>
struct GFRDCompare : NodeCompare<state> {
    bool
    operator()(const AStarMultiOpenClosedData<state> &i1, const AStarMultiOpenClosedData<state> &i2) const override {
        double p1 = 3 * i1.g + i1.h + i1.rh;
        double p2 = 3 * i2.g + i2.h + i2.rh;
        if (fequal(p1, p2)) {
            return (fless(i1.g, i2.g)); // High g-cost over low
        }
        return fgreater(p1, p2); // // Low g+f+rd over high
    }
};

template<typename state>
struct GRFDCompare : NodeCompare<state> {
    bool
    operator()(const AStarMultiOpenClosedData<state> &i1, const AStarMultiOpenClosedData<state> &i2) const override {
        double p1 = 3 * i1.g - i1.h - i1.rh;
        double p2 = 3 * i2.g - i2.h - i2.rh;
        if (fequal(p1, p2)) {
            return (fless(i1.g, i2.g)); // High g-cost over low
        }
        return fgreater(p1, p2); // // Low g+rf+d over high
    }
};

template<typename state>
struct GRFRDCompare : NodeCompare<state> {
    bool
    operator()(const AStarMultiOpenClosedData<state> &i1, const AStarMultiOpenClosedData<state> &i2) const override {
        double p1 = 3 * i1.g - i1.h + i1.rh;
        double p2 = 3 * i2.g - i2.h + i2.rh;
        if (fequal(p1, p2)) {
            return (fless(i1.g, i2.g)); // High g-cost over low
        }
        return fgreater(p1, p2); // // Low g+rf+rd over high
    }
};


template<class state, class action, class environment, class priorityQueue = AStarMultiOpenClosed<state>>
class BAELB {
public:
    BAELB(nodeValue forwardMainHeapNum_, nodeValue backwardMainHeapNum_,
          SideCriterion sideCriterion_ = SideCriterion::Alt, double epsilon_ = 1.0, double gcd_ = 1.0,
          bool withLog_ = false) {
        forwardHeuristic = 0;
        backwardHeuristic = 0;
        env = 0;
        ResetNodeCount();
        epsilon = epsilon_;
        gcd = gcd_;
        sideCriterion = sideCriterion_;
        withLog = withLog_;

        forwardMainHeapNum = forwardMainHeapNum_;
        backwardMainHeapNum = backwardMainHeapNum_;

        std::vector<NodeCompare<state> *> theComps(NODEVALUES_NR_ITEMS);
        theComps[ivG] = &gCompare;
        theComps[ivF] = &fCompare;
        theComps[ivRF] = &rfCompare;
        theComps[ivD] = &dCompare;
        theComps[ivRD] = &rdCompare;
        theComps[ivB] = &bCompare;
        theComps[ivGF] = &gfCompare;
        theComps[ivGD] = &gdCompare;
        theComps[ivFRD] = &frdCompare;
        theComps[ivRFD] = &rfdCompare;
        theComps[ivRFRD] = &rfrdCompare;
        theComps[ivGRF] = &grfCompare;
        theComps[ivGRD] = &grdCompare;
        theComps[ivGB] = &gbCompare;
        theComps[ivGFRD] = &gfrdCompare;
        theComps[ivGRFD] = &grfdCompare;
        theComps[ivGRFRD] = &grfrdCompare;

        forwardQueue = priorityQueue(theComps, forwardMainHeapNum);
        backwardQueue = priorityQueue(theComps, backwardMainHeapNum);
    }

    BAELB(nodeValue mainHeapNum_, SideCriterion sideCriterion_ = SideCriterion::Alt, double epsilon_ = 1.0,
          double gcd_ = 1.0, bool withLog_ = false) :
            BAELB(mainHeapNum_, mainHeapNum_, sideCriterion_, epsilon_, gcd_, withLog_) {}


    virtual ~BAELB() {}

    void GetPath(environment *env, const state &from, const state &to,
                 Heuristic<state> *forward, Heuristic<state> *backward, std::vector<state> &thePath);

    bool InitializeSearch(environment *theEnv, const state &from, const state &to,
                          Heuristic<state> *forward, Heuristic<state> *backward, std::vector<state> &thePath);

    bool DoSingleSearchStep(std::vector<state> &thePath);

    virtual const char *GetName() { return "BAE-LB"; }

    void ResetNodeCount() {
        nodesExpanded = nodesTouched = uniqueNodesExpanded = 0;
        counts.clear();
    }

    inline const int GetNumForwardItems() { return forwardQueue.size(); }

    inline const AStarMultiOpenClosedData<state> &GetForwardItem(unsigned int which) {
        return forwardQueue.Lookat(which);
    }

    inline const int GetNumBackwardItems() { return backwardQueue.size(); }

    inline const AStarMultiOpenClosedData<state> &GetBackwardItem(unsigned int which) {
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

    void ClearMemory() {
        forwardQueue.Reset();
        backwardQueue.Reset();
        forwardValues.clear();
        backwardValues.clear();
        ResetNodeCount();
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

    void Expand(priorityQueue &current, priorityQueue &opposite, std::unordered_map<double, int> &currentValues,
                nodeValue currHeapNum, Heuristic<state> *heuristic, Heuristic<state> *reverse_heuristic,
                const state &target, const state &source);

    double calcBound(BoundValue bv, const AStarMultiOpenClosedData<state> &forwardNode,
                     const AStarMultiOpenClosedData<state> &backwardNode);

    double getLowerBound(bool log = true, bool forceLog = false);

    double getNodePriority(const AStarMultiOpenClosedData<state> &node, nodeValue priority);

    double getNodePriority(double g, double h, double rh, nodeValue priority);


    priorityQueue forwardQueue, backwardQueue;
    state goal, start;

    uint64_t nodesTouched, nodesExpanded, uniqueNodesExpanded;
    state middleNode;
    double currentCost;

    std::vector<state> neighbors;
    environment *env;
    Heuristic<state> *forwardHeuristic;
    Heuristic<state> *backwardHeuristic;

    double epsilon;
    double gcd;

    SideCriterion sideCriterion;
    bool expandForward;

    bool withLog;

    std::map<double, int> counts;

    GCompare<state> gCompare;
    FCompare<state> fCompare;
    RFCompare<state> rfCompare;
    DCompare<state> dCompare;
    RDCompare<state> rdCompare;
    BCompare<state> bCompare;
    GFCompare<state> gfCompare;
    GDCompare<state> gdCompare;
    FRDCompare<state> frdCompare;
    RFDCompare<state> rfdCompare;
    RFRDCompare<state> rfrdCompare;
    GRFCompare<state> grfCompare;
    GRDCompare<state> grdCompare;
    GBCompare<state> gbCompare;
    GFRDCompare<state> gfrdCompare;
    GRFDCompare<state> grfdCompare;
    GRFRDCompare<state> grfrdCompare;

    std::vector<double> boundKeepers;

    int lastNodeExpansionLogged;

    // maps for the optcount side criterion
    std::unordered_map<double, int> forwardValues;
    std::unordered_map<double, int> backwardValues;

    nodeValue forwardMainHeapNum;
    nodeValue backwardMainHeapNum;
};

template<class state, class action, class environment, class priorityQueue>
double BAELB<state, action, environment, priorityQueue>::getNodePriority(double g, double h, double rh,
                                                                         nodeValue priority) {
    switch (priority) {

        case ivG:
            return g;
        case ivF:
            return g + h;
        case ivRF:
            return g - h;
        case ivD:
            return g - rh;
        case ivRD:
            return g + rh;
        case ivB:
            return 2 * g + h - rh;
        case ivGF:
            return 2 * g + h;
        case ivGD:
            return 2 * g - rh;
        case ivFRD:
            return 2 * g + h + rh;
        case ivRFD:
            return 2 * g - h - rh;
        case ivRFRD:
            return 2 * g - h + rh;;
        case ivGRF:
            return 2 * g - h;
        case ivGRD:
            return 2 * g + rh;
        case ivGB:
            return 3 * g + h - rh;
        case ivGFRD:
            return 3 * g + h + rh;
        case ivGRFD:
            return 3 * g - h - rh;
        case ivGRFRD:
            return 3 * g - h + rh;
    }
}

template<class state, class action, class environment, class priorityQueue>
double BAELB<state, action, environment, priorityQueue>::getNodePriority(const AStarMultiOpenClosedData<state> &node,
                                                                         nodeValue priority) {
    return getNodePriority(node.g, node.h, node.rh, priority);
}

/**
 * Gets the path from the start to goal and puts in a provided reference.
 */
template<class state, class action, class environment, class priorityQueue>
void BAELB<state, action, environment, priorityQueue>::GetPath(environment *env, const state &from, const state &to,
                                                               Heuristic<state> *forward, Heuristic<state> *backward,
                                                               std::vector<state> &thePath) {
    if (InitializeSearch(env, from, to, forward, backward, thePath) == false)
        return;
    while (!DoSingleSearchStep(thePath)) {}
    if (withLog)
        std::cout << "[B] Done\n";
}

/**
 * Initializes the search, resets everything and places start in forwardQueue and goal in backwardQueue.
 */
template<class state, class action, class environment, class priorityQueue>
bool BAELB<state, action, environment, priorityQueue>::InitializeSearch(environment *theEnv, const state &from,
                                                                        const state &to,
                                                                        Heuristic<state> *forward,
                                                                        Heuristic<state> *backward,
                                                                        std::vector<state> &thePath) {
    this->env = theEnv;
    forwardHeuristic = forward;
    backwardHeuristic = backward;
    currentCost = DBL_MAX;
    forwardQueue.Reset();
    backwardQueue.Reset();
    ResetNodeCount();
    thePath.resize(0);
    start = from;
    goal = to;

    boundKeepers.clear();
    boundKeepers.assign(BOUNDVALUES_NR_ITEMS, -DBL_MAX);

    if (start == goal)
        return false;

    forwardQueue.AddOpenNode(start, theEnv->GetStateHash(start), 0, forwardHeuristic->HCost(start, goal), 0);
    forwardValues[getNodePriority(0, forwardHeuristic->HCost(start, goal), 0, forwardMainHeapNum)] = 1;
    backwardQueue.AddOpenNode(goal, theEnv->GetStateHash(goal), 0, backwardHeuristic->HCost(goal, start), 0);
    backwardValues[getNodePriority(0, backwardHeuristic->HCost(start, goal), 0, backwardMainHeapNum)] = 1;

    expandForward = true;
    return true;

}

/**
 * Does a single search step which includes checking for stopping criteria and expanding
 */
template<class state, class action, class environment, class priorityQueue>
bool BAELB<state, action, environment, priorityQueue>::DoSingleSearchStep(std::vector<state> &thePath) {
    if ((forwardQueue.OpenSize() == 0 || backwardQueue.OpenSize() == 0) && currentCost == DBL_MAX) {
        std::cerr << "[ERROR] Problem with no solution? Expanded: " << nodesExpanded << std::endl;
        exit(0);
    }

    // Check if our current solution is lower or equal to the lower bound on C*
    if (currentCost <= getLowerBound()) {
        if (nodesExpanded != lastNodeExpansionLogged) {
            getLowerBound(true, true);
        }
        std::vector<state> pFor, pBack;
        ExtractPathToGoal(middleNode, pBack);
        ExtractPathToStart(middleNode, pFor);
        reverse(pFor.begin(), pFor.end());
        thePath = pFor;
        thePath.insert(thePath.end(), pBack.begin() + 1, pBack.end());

        // If the path is fully on one side
        if (thePath.size() >=2 && thePath[thePath.size() - 1] == thePath[thePath.size() - 2]) {
            thePath.pop_back();
        }

        return true;
    }

    switch (sideCriterion) {
        case SideCriterion::Alt: {
            if (expandForward) {
                Expand(forwardQueue, backwardQueue,
                       forwardValues, forwardMainHeapNum,
                       forwardHeuristic, backwardHeuristic,
                       goal, start);
                expandForward = false;
            } else {
                Expand(backwardQueue, forwardQueue,
                       backwardValues, backwardMainHeapNum,
                       backwardHeuristic, forwardHeuristic,
                       start, goal);
                expandForward = true;
            }
            break;
        }
        case SideCriterion::Cardinality: {
            if (forwardQueue.OpenSize() > backwardQueue.OpenSize()) {
                Expand(backwardQueue, forwardQueue,
                       backwardValues, backwardMainHeapNum,
                       backwardHeuristic, forwardHeuristic,
                       start, goal);
            } else {
                Expand(forwardQueue, backwardQueue,
                       forwardValues, forwardMainHeapNum,
                       forwardHeuristic, backwardHeuristic,
                       goal, start);
            }
            break;
        }
        case SideCriterion::OptCount: {
            if (forwardValues[getNodePriority(forwardQueue.Lookat(forwardQueue.Peek()), forwardMainHeapNum)] >
                backwardValues[getNodePriority(backwardQueue.Lookat(backwardQueue.Peek()), backwardMainHeapNum)]) {
                Expand(backwardQueue, forwardQueue,
                       backwardValues, backwardMainHeapNum,
                       backwardHeuristic, forwardHeuristic,
                       start, goal);
            } else {
                Expand(forwardQueue, backwardQueue,
                       forwardValues, forwardMainHeapNum,
                       forwardHeuristic, backwardHeuristic,
                       goal, start);
            }
            break;
        }
    }

    return false;
}

/**
 * Expand the next node and update or insert its successors to the queue
 */
template<class state, class action, class environment, class priorityQueue>
void BAELB<state, action, environment, priorityQueue>::Expand(priorityQueue &current, priorityQueue &opposite,
                                                              std::unordered_map<double, int> &currentValues,
                                                              nodeValue currHeapNum,
                                                              Heuristic<state> *heuristic,
                                                              Heuristic<state> *reverse_heuristic,
                                                              const state &target, const state &source) {
    //1. if current closed in opposite direction
    // 1a. Remove descendents of current in open

    uint64_t nextID;
    bool success = false;
    while (current.OpenSize() > 0) {
        nextID = current.Close();
        currentValues[getNodePriority(current.Lookup(nextID), currHeapNum)] -= 1;
        uint64_t reverseLoc;
        auto loc = opposite.Lookup(env->GetStateHash(current.Lookup(nextID).data), reverseLoc);
        if (loc != kClosedList) {
            success = true;
            break;
        }
    }
    if (!success)
        return;

    // 2. Else expand as usual on current direction
    // 2a. Check for bidirectional solution

    bool foundBetterSolution = false;
    nodesExpanded++;

    counts[getLowerBound(false)] += 1;

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
            case kClosedList: {
                if (fless(parentData.g + edgeCost, childData.g)) {
                    std::cerr << "[ERROR] Expanded node with non-optimal g" << std::endl;
                    exit(0);
                    // This is written for the case we work with non-consistent heuristics,
                    // but we do not reach it in the current code
                    childData.parentID = nextID;
                    childData.g = parentData.g + edgeCost;
                    childData.h = std::max(childData.h, parentData.h - edgeCost);
                    childData.rh = reverse_heuristic->HCost(succ, source);
                    current.Reopen(childID);
                }
            }
                break;
            case kOpenList: { // Update cost if needed
                if (fless(parentData.g + edgeCost, childData.g)) {
                    currentValues[getNodePriority(childData, currHeapNum)] -= 1;
                    childData.parentID = nextID;
                    childData.g = parentData.g + edgeCost;
                    currentValues[getNodePriority(childData, currHeapNum)] += 1;
                    current.KeyChanged(childID);

                    // Check if we improved the current solution
                    uint64_t reverseLoc;
                    if (opposite.Lookup(hash, reverseLoc) == kOpenList) {
                        if (fless(parentData.g + edgeCost + opposite.Lookup(reverseLoc).g, currentCost)) {
                            foundBetterSolution = true;
                            currentCost = parentData.g + edgeCost + opposite.Lookup(reverseLoc).g;
                            middleNode = succ;
                        }
                    }
                }
            }
                break;
            case kNotFound: { //New Node
                double g = parentData.g + edgeCost;
                double h = std::max(heuristic->HCost(succ, target), epsilon);
                double rh = reverse_heuristic->HCost(succ, source);

                // Ignore nodes that don't have lower f-cost than the incumbant solution
                if (!fless(g + h, currentCost))
                    break;

                current.AddOpenNode(succ, hash, g, h, rh, nextID);
                currentValues[getNodePriority(g, h, rh, currHeapNum)] += 1;
                // Check for solution
                uint64_t reverseLoc;
                if (opposite.Lookup(hash, reverseLoc) == kOpenList) {
                    if (fless(current.Lookup(nextID).g + edgeCost + opposite.Lookup(reverseLoc).g, currentCost)) {
                        foundBetterSolution = true;
                        currentCost = current.Lookup(nextID).g + edgeCost + opposite.Lookup(reverseLoc).g;
                        middleNode = succ;
                    }
                }
            }
                break;
        }
    }
}

/**
 * Calculates a particular bound given the backward and forward node.
 */
template<class state, class action, class environment, class priorityQueue>
double BAELB<state, action, environment, priorityQueue>::calcBound(BoundValue bv,
                                                                   const AStarMultiOpenClosedData<state> &forwardNode,
                                                                   const AStarMultiOpenClosedData<state> &backwardNode) {
    double bound = -1;
    switch (bv) {
        case lbFD: // gF(u) + gB(u) + hF(u) - hF(v)
            bound = forwardNode.g + backwardNode.g + forwardNode.h - backwardNode.rh;
            break;
        case lbRFRD: // gF(u) + gB(v) + hF(v) - hF(u)
            bound = forwardNode.g + backwardNode.g - forwardNode.h + backwardNode.rh;
            break;
        case lbDF: // gF(u) + gB(v) + hB(v) - hB(u)
            bound = forwardNode.g + backwardNode.g - forwardNode.rh + backwardNode.h;
            break;
        case lbRDRF: // gF(u) + gB(v) + hB(u) - hB(v)
            bound = forwardNode.g + backwardNode.g + forwardNode.rh - backwardNode.h;
            break;
        case lbG: // gF(u) + gB(v) + eps
            bound = forwardNode.g + backwardNode.g + epsilon;
            break;
        case lbB: // gF(u) + gB(v) + (hF(u)-hB(v)+hB(v)-hB(u))/2
            bound = forwardNode.g + backwardNode.g +
                    (forwardNode.h - backwardNode.rh + backwardNode.h - forwardNode.rh) / 2;
            break;
        case lbFRDRFD: // gF(u) + gB(v) + (hF(u)-hF(v)+hB(u)-hB(v))/2
            bound = forwardNode.g + backwardNode.g +
                    (forwardNode.h - backwardNode.rh + forwardNode.rh - backwardNode.h) / 2;
            break;
        case lbGFGD: // gF(u) + gB(v) + (hF(u) - hF(v) + eps)/2
            bound = forwardNode.g + backwardNode.g + (forwardNode.h - backwardNode.rh + epsilon) / 2;
            break;
        case lbRFDFRD: // gF(u) + gB(v) + (hF(v) - hF(u) + hB(v) - hB(u))/2
            bound = forwardNode.g + backwardNode.g +
                    (backwardNode.rh - forwardNode.h + backwardNode.h - forwardNode.rh) / 2;
            break;
        case lbRFRDX2: // gF(u) + gB(v) + (hF(v) - hF(u) + hB(u) - hB(v))/2
            bound = forwardNode.g + backwardNode.g +
                    (backwardNode.rh - forwardNode.h + forwardNode.rh - backwardNode.h) / 2;
            break;
        case lbGRFGRD: // gF(u) + gB(v) + (hF(v) - hF(u) + eps)/2
            bound = forwardNode.g + backwardNode.g + (backwardNode.rh - forwardNode.h + epsilon) / 2;
            break;
        case lbGDGF: // gF(u) + gB(v) + (hB(v) - hB(u) + eps)/2
            bound = forwardNode.g + backwardNode.g + (backwardNode.h - forwardNode.rh + epsilon) / 2;
            break;
        case lbGRDGRF: // gF(u) + gB(v) + (hB(u) - hB(v) + eps)/2
            bound = forwardNode.g + backwardNode.g + (forwardNode.rh - backwardNode.h + epsilon) / 2;
            break;
        case lbGB: // gF(u) + gB(v) + (hF(u) - hF(v) + hB(v) - hB(u) + eps)/3
            bound = forwardNode.g + backwardNode.g +
                    (forwardNode.h - backwardNode.rh + backwardNode.h - forwardNode.rh + epsilon) / 3;
            break;
        case lbGFRDGRFD: // gF(u) + gB(v) + (hF(u) - hF(v) + hB(u) - hB(v) + eps)/3
            bound = forwardNode.g + backwardNode.g +
                    (forwardNode.h - backwardNode.rh + forwardNode.rh - backwardNode.h + epsilon) / 3;
            break;
        case lbGRFDGFRD: // gF(u) + gB(v) + (hF(v) - hF(u) + hB(u) - hB(v) + eps)/3
            bound = forwardNode.g + backwardNode.g +
                    (backwardNode.rh - forwardNode.h + backwardNode.h - forwardNode.rh + epsilon) / 3;
            break;
        case lbGRFRD: // gF(u) + gB(v) + (hF(v) - hF(u) + hB(u) - hB(v) + eps)/3
            bound = forwardNode.g + backwardNode.g +
                    (backwardNode.rh - forwardNode.h + forwardNode.rh - backwardNode.h + epsilon) / 3;
            break;
    }
    return ceil(bound / gcd) * gcd;
}

/**
 * Finds the lower bound of the current step. Each heap corresponds to some minimum value
 * and each AStarMultiOpenClosedData contains the g, h, and rh of the state
 */
template<class state, class action, class environment, class priorityQueue>
double BAELB<state, action, environment, priorityQueue>::getLowerBound(bool log, bool forceLog) {
    if (forwardQueue.OpenSize() == 0 || backwardQueue.OpenSize() == 0)
        return DBL_MAX;

    std::vector<AStarMultiOpenClosedData<state>> forwardMinNodes;
    std::vector<AStarMultiOpenClosedData<state>> backwardMinNodes;
    for (int i = 0; i < NODEVALUES_NR_ITEMS; i++) {
        forwardMinNodes.push_back(forwardQueue.Lookup(forwardQueue.Peek(i)));
        backwardMinNodes.push_back(backwardQueue.Lookup(backwardQueue.Peek(i)));
    }
    std::vector<double> cBounds(BOUNDVALUES_NR_ITEMS, 0);

    cBounds[lbFD] = calcBound(lbFD, forwardMinNodes[ivF], backwardMinNodes[ivD]);
    cBounds[lbRFRD] = calcBound(lbRFRD, forwardMinNodes[ivRF], backwardMinNodes[ivRD]);
    cBounds[lbDF] = calcBound(lbDF, forwardMinNodes[ivD], backwardMinNodes[ivF]);
    cBounds[lbRDRF] = calcBound(lbRDRF, forwardMinNodes[ivRD], backwardMinNodes[ivRF]);
    cBounds[lbG] = calcBound(lbG, forwardMinNodes[ivG], backwardMinNodes[ivG]);
    cBounds[lbB] = calcBound(lbB, forwardMinNodes[ivB], backwardMinNodes[ivB]);
    cBounds[lbFRDRFD] = calcBound(lbFRDRFD, forwardMinNodes[ivFRD], backwardMinNodes[ivRFD]);
    cBounds[lbGFGD] = calcBound(lbGFGD, forwardMinNodes[ivGF], backwardMinNodes[ivGD]);
    cBounds[lbRFDFRD] = calcBound(lbRFDFRD, forwardMinNodes[ivRFD], backwardMinNodes[ivFRD]);
    cBounds[lbRFRDX2] = calcBound(lbRFRDX2, forwardMinNodes[ivRFRD], backwardMinNodes[ivRFRD]);
    cBounds[lbGRFGRD] = calcBound(lbGRFGRD, forwardMinNodes[ivGRF], backwardMinNodes[ivGRD]);
    cBounds[lbGDGF] = calcBound(lbGDGF, forwardMinNodes[ivGD], backwardMinNodes[ivGF]);
    cBounds[lbGRDGRF] = calcBound(lbGRDGRF, forwardMinNodes[ivGRD], backwardMinNodes[ivGRF]);
    cBounds[lbGB] = calcBound(lbGB, forwardMinNodes[ivGB], backwardMinNodes[ivGB]);
    cBounds[lbGFRDGRFD] = calcBound(lbGFRDGRFD, forwardMinNodes[ivGFRD], backwardMinNodes[ivGRFD]);
    cBounds[lbGRFDGFRD] = calcBound(lbGRFDGFRD, forwardMinNodes[ivGRFD], backwardMinNodes[ivGFRD]);
    cBounds[lbGRFRD] = calcBound(lbGRFRD, forwardMinNodes[ivGRFRD], backwardMinNodes[ivGRFRD]);

    // If any of the bounds changed, update the keepers and print them, since we set them to -1, they will always be
    // updated and printed before the first expansion, i.e. NE=0
    if (withLog) {
        if (forceLog || ((cBounds != boundKeepers) && log)) {
            std::cout << "[B] NE=" << nodesExpanded << ' ';
            for (auto const &boundPair: BOUND_NAME_ENUM_PAIRS)
                std::cout << boundPair.first << '=' << cBounds[boundPair.second] << ' ';
            std::cout << '\n';
            boundKeepers = cBounds;
            lastNodeExpansionLogged = nodesExpanded;
        }
    }


    return *max_element(cBounds.begin(), cBounds.end());
}

#endif
