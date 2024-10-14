#ifndef DBBS_H
#define DBBS_H

#include "BestBucketBasedList.h"
#include "MinCriterion.h"
#include "FrontToEnd.h"
#include "FPUtil.h"
#include <iostream>
#include <math.h>

template<class state, class action, class environment, MinCriterion criterion, class priorityQueue = BestBucketBasedList<state, environment, BucketNodeData<state>, criterion>>
class DBBS : public FrontToEnd<state, action, environment, priorityQueue> {

    using FrontToEnd<state, action, environment, priorityQueue>::forwardQueue;
    using FrontToEnd<state, action, environment, priorityQueue>::backwardQueue;
    using FrontToEnd<state, action, environment, priorityQueue>::forwardHeuristic;
    using FrontToEnd<state, action, environment, priorityQueue>::backwardHeuristic;
    using FrontToEnd<state, action, environment, priorityQueue>::C;
    using FrontToEnd<state, action, environment, priorityQueue>::currentCost;
    using FrontToEnd<state, action, environment, priorityQueue>::epsilon;
    using FrontToEnd<state, action, environment, priorityQueue>::start;
    using FrontToEnd<state, action, environment, priorityQueue>::goal;
    using FrontToEnd<state, action, environment, priorityQueue>::nodesExpanded;

    using FrontToEnd<state, action, environment, priorityQueue>::Expand;
    using FrontToEnd<state, action, environment, priorityQueue>::ExpandBucket;
    using FrontToEnd<state, action, environment, priorityQueue>::CheckSolution;


public:
    DBBS(SideCriterion side_crit_, bool useB_ = true, double epsilon_ = 1.0, double gcd_ = 1.0)
            : FrontToEnd<state, action, environment, priorityQueue>(epsilon_),
              side_crit(side_crit_), useB(useB_), gcd(gcd_) {}

    ~DBBS() {}

    virtual const char *GetName() { return "DBBS"; }

protected:

    bool UpdateC();

    double GetNextC();

    virtual void RunAlgorithm();

    void ExpandFromBestBucket(priorityQueue &current, priorityQueue &opposite,
                              Heuristic <state> *heuristic,
                              Heuristic <state> *reverseHeuristic,
                              const state &target, const state &source);

    SideCriterion side_crit;
    bool useB;
    double gcd;

    bool expandForward = true;


    // TODO parametrize this
    bool useRC = true;
};

template<class state, class action, class environment, MinCriterion criterion, class priorityQueue>
bool DBBS<state, action, environment, criterion, priorityQueue>::UpdateC() {

    if (forwardQueue.isBestBucketComputed() && backwardQueue.isBestBucketComputed())
        return false; // no need to recompute anything, and no need to rise C

    bool incrementedC = false;

    while (C < currentCost && (!forwardQueue.isBestBucketComputed() || !backwardQueue.isBestBucketComputed())) {

        // initial forward queue limits
        forwardQueue.computeBestBucket(C, C, C, 2.0 * C, DBL_MAX, DBL_MAX);

        double gMinF = forwardQueue.isBestBucketComputed() ? forwardQueue.getMinG() : C;
        double fMinF = forwardQueue.isBestBucketComputed() ? forwardQueue.getMinF() : C;
        double dMinF = forwardQueue.isBestBucketComputed() ? forwardQueue.getMinD() : C;
        double bMinF = forwardQueue.isBestBucketComputed() ? forwardQueue.getMinB() : 2 * C;
        double rfMinF = forwardQueue.isBestBucketComputed() ? forwardQueue.getMinRF() : C;
        double rdMinF = forwardQueue.isBestBucketComputed() ? forwardQueue.getMinRD() : C;

        bool limitsChanged = true;

        // fixpoint computation of limits
        while (limitsChanged) {

            limitsChanged = false;

            backwardQueue.computeBestBucket(C - (gMinF + epsilon), C - dMinF, C - fMinF,
                                            2.0 * C - bMinF, C - rdMinF, C - rfMinF);
            if (!backwardQueue.isBestBucketComputed()) break;

            double gMinB = backwardQueue.isBestBucketComputed() ? backwardQueue.getMinG() : C;
            double fMinB = backwardQueue.isBestBucketComputed() ? backwardQueue.getMinF() : C;
            double dMinB = backwardQueue.isBestBucketComputed() ? backwardQueue.getMinD() : C;
            double bMinB = backwardQueue.isBestBucketComputed() ? backwardQueue.getMinB() : 2 * C;
            double rfMinB = backwardQueue.isBestBucketComputed() ? backwardQueue.getMinRF() : DBL_MAX;
            double rdMinB = backwardQueue.isBestBucketComputed() ? backwardQueue.getMinRD() : DBL_MAX;

            // forward queue limits
            forwardQueue.computeBestBucket(C - (gMinB + epsilon), C - dMinB, C - fMinB,
                                           2.0 * C - bMinB, C - rdMinB, C - rfMinB);

            if (!forwardQueue.isBestBucketComputed()) break;

            double gMinF_new = forwardQueue.isBestBucketComputed() ? forwardQueue.getMinG() : C;
            double fMinF_new = forwardQueue.isBestBucketComputed() ? forwardQueue.getMinF() : C;
            double dMinF_new = forwardQueue.isBestBucketComputed() ? forwardQueue.getMinD() : C;
            double bMinF_new = forwardQueue.isBestBucketComputed() ? forwardQueue.getMinB() : 2 * C;
            double rfMinF_new = forwardQueue.isBestBucketComputed() ? forwardQueue.getMinRF() : DBL_MAX;
            double rdMinF_new = forwardQueue.isBestBucketComputed() ? forwardQueue.getMinRD() : DBL_MAX;

            limitsChanged = gMinF != gMinF_new || fMinF != fMinF_new || dMinF != dMinF_new
                            || bMinF != bMinF_new || rfMinF != rfMinF_new || rdMinF != rdMinF_new;

            gMinF = gMinF_new, fMinF = fMinF_new, dMinF = dMinF_new,
            bMinF = bMinF_new, rfMinF = rfMinF_new, rdMinF = rdMinF_new;
        };

        // if limits don't change and still no expandable bucket is found, increase C
        if (!forwardQueue.isBestBucketComputed() || !backwardQueue.isBestBucketComputed()) {
//            C += gcd;
            C = GetNextC();
            incrementedC = true;
        }
    }

    // if we don't alternate, count nodes on both sides
    if (side_crit == SideCriterion::Cardinality && forwardQueue.isBestBucketComputed() && backwardQueue.isBestBucketComputed()) {
        forwardQueue.countExpandableNodes();
        backwardQueue.countExpandableNodes();
    }

    return incrementedC;
}

template<class state, class action, class environment, MinCriterion criterion, class priorityQueue>
double DBBS<state, action, environment, criterion, priorityQueue>::GetNextC() {

    // TODO figure out if using more bounds may make increasing C slower

    double result = DBL_MAX;

    const NodeValues &forwardValues = forwardQueue.getNodeValues();
    const NodeValues &backwardValues = backwardQueue.getNodeValues();

    // g bound
    const auto &forward_g_values = forwardValues.g_values;
    const auto &backward_g_values = backwardValues.g_values;

    for (const double fw_g_value : forward_g_values)
        for (const double bw_g_value : backward_g_values) {
            double g_bound = fw_g_value + bw_g_value + epsilon;
            if (g_bound > C && g_bound < result)
                result = g_bound;
        }

    const auto &forward_f_values = forwardValues.f_values;
    const auto &forward_d_values = forwardValues.d_values;
    const auto &backward_f_values = backwardValues.f_values;
    const auto &backward_d_values = backwardValues.d_values;

    // forward KK bound
    for (const double fw_f_value : forward_f_values)
        for (const double bw_d_value : backward_d_values) {
            double fw_KK_bound = fw_f_value + bw_d_value;
            if (fw_KK_bound > C && fw_KK_bound < result)
                result = fw_KK_bound;
        }

    // backward KK bound
    for (const double bw_f_value : backward_f_values)
        for (const double fw_d_value : forward_d_values) {
            double bw_KK_bound = bw_f_value + fw_d_value;
            if (bw_KK_bound > C && bw_KK_bound < result)
                result = bw_KK_bound;
        }

    // b bound
    if (useB) {
        const auto &forward_b_values = forwardValues.b_values;
        const auto &backward_b_values = backwardValues.b_values;

        for (const double fw_b_value : forward_b_values)
            for (const double bw_b_value : backward_b_values) {
                double b_bound = gcd * std::ceil(((fw_b_value + bw_b_value) / 2) / gcd);
                if (b_bound > C && b_bound < result)
                    result = b_bound;
            }
    }

    // forward rc bound
    if (useRC) {
        const auto &forward_rf_values = forwardValues.rf_values;
        const auto &backward_rd_values = backwardValues.rd_values;

        for (const double fw_rf_value : forward_rf_values)
            for (const double bw_rd_value : backward_rd_values) {
                double fw_RC_bound = fw_rf_value + bw_rd_value;
                if (fw_RC_bound > C && fw_RC_bound < result)
                    result = fw_RC_bound;
            }
    }

    // backward rc bound
    if (useRC) {
        const auto &forward_rd_values = forwardValues.rd_values;
        const auto &backward_rf_values = backwardValues.rf_values;

        for (const double bw_rf_value : backward_rf_values)
            for (const double fw_rd_value : forward_rd_values) {
                double bw_RC_bound = bw_rf_value + fw_rd_value;
                if (bw_RC_bound > C && bw_RC_bound < result)
                    result = bw_RC_bound;
            }
    }

    return result;
}

template<class state, class action, class environment, MinCriterion criterion, class priorityQueue>
void DBBS<state, action, environment, criterion, priorityQueue>::RunAlgorithm() {
    while (!forwardQueue.IsEmpty() && !backwardQueue.IsEmpty()) {

        if (UpdateC()) {
            // TODO think how we are going to parametrize the tie breaker
            if (CheckSolution()) break; // optimality can be proven after updating C
        }

		// TODO: parametrize better whether we want to alternate or to take a look at the open lists
        if (side_crit == SideCriterion::Alt) { // alternate directions
            if (expandForward) {
                ExpandFromBestBucket(forwardQueue, backwardQueue, forwardHeuristic, backwardHeuristic, goal, start);
                expandForward = false;
            } else {
                ExpandFromBestBucket(backwardQueue, forwardQueue, backwardHeuristic, forwardHeuristic, start, goal);
                expandForward = true;
            }
        } else if (side_crit == SideCriterion::Cardinality){ // choose side with the fewest nodes with minimum g

            double gNodesForward = forwardQueue.getExpandableNodes();
            double gNodesBackward = backwardQueue.getExpandableNodes();

            if (gNodesForward <= gNodesBackward)
                ExpandFromBestBucket(forwardQueue, backwardQueue, forwardHeuristic, backwardHeuristic, goal, start);
            else
                ExpandFromBestBucket(backwardQueue, forwardQueue, backwardHeuristic, forwardHeuristic, start, goal);
        } else if (side_crit == SideCriterion::OptCount){
			double gNodesForward = forwardQueue.getOptCount();
            double gNodesBackward = backwardQueue.getOptCount();
			if (gNodesForward <= gNodesBackward)
                ExpandFromBestBucket(forwardQueue, backwardQueue, forwardHeuristic, backwardHeuristic, goal, start);
            else
                ExpandFromBestBucket(backwardQueue, forwardQueue, backwardHeuristic, forwardHeuristic, start, goal);
			
		}
		else{
			std::cout << "Unsupported Criterion" << std::endl;
            exit(0);
		}

        if (CheckSolution()) break; // a newer collision after expansion may prove optimality
    }
}

template<class state, class action, class environment, MinCriterion criterion, class priorityQueue>
void DBBS<state, action, environment, criterion, priorityQueue>::ExpandFromBestBucket(priorityQueue &current,
                                                                                      priorityQueue &opposite,
                                                                                      Heuristic <state> *heuristic,
                                                                                      Heuristic <state> *reverseHeuristic,
                                                                                      const state &target,
                                                                                      const state &source) {
    auto nodePair = current.Pop();

    // despite apparently having expandable nodes, best candidates may be invalidated entries
    if (nodePair.first == nullptr) return;

    Expand(nodePair.first, nodePair.second, current, opposite, heuristic, reverseHeuristic, target, source);
}

#endif
