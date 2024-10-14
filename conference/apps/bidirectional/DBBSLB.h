#ifndef DBBSLB_H
#define DBBSLB_H

#include "LBBestBucketBasedList.h"
#include "MinCriterion.h"
#include "FrontToEnd.h"
#include "FPUtil.h"
#include <iostream>
#include <math.h>


template<class state, class action, class environment, MinCriterion criterion, class priorityQueue = LBBestBucketBasedList<state, environment, BucketNodeData<state>, criterion>>
class DBBSLB : public FrontToEnd<state, action, environment, priorityQueue> {

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
    DBBSLB(SideCriterion side_crit_, bool useB_ = true, double epsilon_ = 1.0, double gcd_ = 1.0)
            : FrontToEnd<state, action, environment, priorityQueue>(epsilon_),
              side_crit(side_crit_), gcd(gcd_) {}

    ~DBBSLB() {}

    virtual const char *GetName() { return "DBBSLB"; }

protected:

    bool UpdateC();

    double GetNextC();

    virtual void RunAlgorithm();

    void ExpandFromBestBucket(priorityQueue &current, priorityQueue &opposite,
                              Heuristic <state> *heuristic,
                              Heuristic <state> *reverseHeuristic,
                              const state &target, const state &source);

    SideCriterion side_crit;
    double gcd;

    bool expandForward = true;


    // TODO parametrize this
};

template<class state, class action, class environment, MinCriterion criterion, class priorityQueue>
bool DBBSLB<state, action, environment, criterion, priorityQueue>::UpdateC() {

    if (forwardQueue.isBestBucketComputed() && backwardQueue.isBestBucketComputed())
        return false; // no need to recompute anything, and no need to rise C

    bool incrementedC = false;

    while (C < currentCost && (!forwardQueue.isBestBucketComputed() || !backwardQueue.isBestBucketComputed())) {

        // initial forward queue limits
        forwardQueue.computeBestBucket(C, C, C, 2.0 * C, DBL_MAX, DBL_MAX, 2.0 * C, 2.0 * C, 3.0 * C, DBL_MAX, DBL_MAX, DBL_MAX, DBL_MAX, DBL_MAX, DBL_MAX, DBL_MAX, DBL_MAX);

        double gMinF = forwardQueue.isBestBucketComputed() ? forwardQueue.getMinG() : C;
        double fMinF = forwardQueue.isBestBucketComputed() ? forwardQueue.getMinF() : C;
        double dMinF = forwardQueue.isBestBucketComputed() ? forwardQueue.getMinD() : C;
        double bMinF = forwardQueue.isBestBucketComputed() ? forwardQueue.getMinB() : 2 * C;
        double rfMinF = forwardQueue.isBestBucketComputed() ? forwardQueue.getMinRF() : C;
        double rdMinF = forwardQueue.isBestBucketComputed() ? forwardQueue.getMinRD() : C;
		
		double FGMinF = forwardQueue.isBestBucketComputed() ? forwardQueue.getMinFG() : 2 * C;
		double DGMinF = forwardQueue.isBestBucketComputed() ? forwardQueue.getMinDG() : 2 * C;
		double BGMinF = forwardQueue.isBestBucketComputed() ? forwardQueue.getMinBG() : 3 * C;
		double FRDMinF = forwardQueue.isBestBucketComputed() ? forwardQueue.getMinFRD() : 2 * C;
		double RFDMinF = forwardQueue.isBestBucketComputed() ? forwardQueue.getMinRFD() : 2 * C;
		double RFGMinF = forwardQueue.isBestBucketComputed() ? forwardQueue.getMinRFG() : 2 * C;
		double RDGMinF = forwardQueue.isBestBucketComputed() ? forwardQueue.getMinRDG() : 2 * C;
		double RFRDMinF = forwardQueue.isBestBucketComputed() ? forwardQueue.getMinRFRD() : 2 * C;
		double FRDGMinF = forwardQueue.isBestBucketComputed() ? forwardQueue.getMinFRDG() : 3 * C;
		double RFDGMinF = forwardQueue.isBestBucketComputed() ? forwardQueue.getMinRFDG() : 3 * C;
		double RFRDGMinF = forwardQueue.isBestBucketComputed() ? forwardQueue.getMinRFRDG() : 3 * C;
		
        bool limitsChanged = true;

        // fixpoint computation of limits
        while (limitsChanged) {

            limitsChanged = false;

            backwardQueue.computeBestBucket(C - (gMinF + epsilon), C - dMinF, C - fMinF,
                                            2.0 * C - bMinF, C - rdMinF, C - rfMinF, 
											2.0 * C - (DGMinF + epsilon), 2.0 * C - (FGMinF + epsilon), 3.0 * C - (BGMinF + epsilon),
											2.0 * C - RFDMinF, 2.0 * C - FRDMinF, 2.0 * C - (RDGMinF + epsilon), 2.0 * C - (RFGMinF + epsilon),
											2.0 * C - RFRDMinF, 3.0 * C - (RFDGMinF + epsilon), 3.0 * C - (FRDGMinF + epsilon), 3.0 * C - (RFRDGMinF + epsilon)
											);
            if (!backwardQueue.isBestBucketComputed()) break;

            double gMinB = backwardQueue.isBestBucketComputed() ? backwardQueue.getMinG() : C;
            double fMinB = backwardQueue.isBestBucketComputed() ? backwardQueue.getMinF() : C;
            double dMinB = backwardQueue.isBestBucketComputed() ? backwardQueue.getMinD() : C;
            double bMinB = backwardQueue.isBestBucketComputed() ? backwardQueue.getMinB() : 2 * C;
            double rfMinB = backwardQueue.isBestBucketComputed() ? backwardQueue.getMinRF() : DBL_MAX;
            double rdMinB = backwardQueue.isBestBucketComputed() ? backwardQueue.getMinRD() : DBL_MAX;
			
			double FGMinB = backwardQueue.isBestBucketComputed() ? backwardQueue.getMinFG() : 2 * C;
			double DGMinB = backwardQueue.isBestBucketComputed() ? backwardQueue.getMinDG() : 2 * C;
			double BGMinB = backwardQueue.isBestBucketComputed() ? backwardQueue.getMinBG() : 3 * C;
			double FRDMinB = backwardQueue.isBestBucketComputed() ? backwardQueue.getMinFRD() : DBL_MAX;
			double RFDMinB = backwardQueue.isBestBucketComputed() ? backwardQueue.getMinRFD() : DBL_MAX;
			double RFGMinB = backwardQueue.isBestBucketComputed() ? backwardQueue.getMinRFG() : DBL_MAX;
			double RDGMinB = backwardQueue.isBestBucketComputed() ? backwardQueue.getMinRDG() : DBL_MAX;
			double RFRDMinB = backwardQueue.isBestBucketComputed() ? backwardQueue.getMinRFRD() : DBL_MAX;
			double FRDGMinB = backwardQueue.isBestBucketComputed() ? backwardQueue.getMinFRDG() : DBL_MAX;
			double RFDGMinB = backwardQueue.isBestBucketComputed() ? backwardQueue.getMinRFDG() : DBL_MAX;
			double RFRDGMinB = backwardQueue.isBestBucketComputed() ? backwardQueue.getMinRFRDG() : DBL_MAX;

            // forward queue limits
            forwardQueue.computeBestBucket(C - (gMinB + epsilon), C - dMinB, C - fMinB,
                                           2.0 * C - bMinB, C - rdMinB, C - rfMinB,
										   2.0 * C - (DGMinB + epsilon), 2.0 * C - (FGMinB + epsilon), 3.0 * C - (BGMinB + epsilon),
											2.0 * C - RFDMinB, 2.0 * C - FRDMinB, 2.0 * C - (RDGMinB + epsilon), 2.0 * C - (RFGMinB + epsilon),
											2.0 * C - RFRDMinB, 3.0 * C - (RFDGMinB + epsilon), 3.0 * C - (FRDGMinB + epsilon), 3.0 * C - (RFRDGMinB + epsilon)
											);

            if (!forwardQueue.isBestBucketComputed()) break;

            double gMinF_new = forwardQueue.isBestBucketComputed() ? forwardQueue.getMinG() : C;
            double fMinF_new = forwardQueue.isBestBucketComputed() ? forwardQueue.getMinF() : C;
            double dMinF_new = forwardQueue.isBestBucketComputed() ? forwardQueue.getMinD() : C;
            double bMinF_new = forwardQueue.isBestBucketComputed() ? forwardQueue.getMinB() : 2 * C;
            double rfMinF_new = forwardQueue.isBestBucketComputed() ? forwardQueue.getMinRF() : DBL_MAX;
            double rdMinF_new = forwardQueue.isBestBucketComputed() ? forwardQueue.getMinRD() : DBL_MAX;
			
			double FGMinF_new = forwardQueue.isBestBucketComputed() ? forwardQueue.getMinFG() : 2 * C;
			double DGMinF_new = forwardQueue.isBestBucketComputed() ? forwardQueue.getMinDG() : 2 * C;
			double BGMinF_new = forwardQueue.isBestBucketComputed() ? forwardQueue.getMinBG() : 3 * C;
			double FRDMinF_new = forwardQueue.isBestBucketComputed() ? forwardQueue.getMinFRD() : DBL_MAX;
			double RFDMinF_new = forwardQueue.isBestBucketComputed() ? forwardQueue.getMinRFD() : DBL_MAX;
			double RFGMinF_new = forwardQueue.isBestBucketComputed() ? forwardQueue.getMinRFG() : DBL_MAX;
			double RDGMinF_new = forwardQueue.isBestBucketComputed() ? forwardQueue.getMinRDG() : DBL_MAX;
			double RFRDMinF_new = forwardQueue.isBestBucketComputed() ? forwardQueue.getMinRFRD() : DBL_MAX;
			double FRDGMinF_new = forwardQueue.isBestBucketComputed() ? forwardQueue.getMinFRDG() : DBL_MAX;
			double RFDGMinF_new = forwardQueue.isBestBucketComputed() ? forwardQueue.getMinRFDG() : DBL_MAX;
			double RFRDGMinF_new = forwardQueue.isBestBucketComputed() ? forwardQueue.getMinRFRDG() : DBL_MAX;

            limitsChanged = gMinF != gMinF_new || fMinF != fMinF_new || dMinF != dMinF_new
                            || bMinF != bMinF_new || rfMinF != rfMinF_new || rdMinF != rdMinF_new
							|| FGMinF != FGMinF_new || DGMinF != DGMinF_new || BGMinF != BGMinF_new || FRDMinF != FRDMinF_new
							|| RFDMinF != RFDMinF_new || RFGMinF != RFGMinF_new || RDGMinF != RDGMinF_new || RFRDMinF != RFRDMinF_new
							|| FRDGMinF != FRDGMinF_new || RFDGMinF != RFDGMinF_new || RFRDGMinF != RFRDGMinF_new;

            gMinF = gMinF_new, fMinF = fMinF_new, dMinF = dMinF_new,
            bMinF = bMinF_new, rfMinF = rfMinF_new, rdMinF = rdMinF_new;
			
			FGMinF = FGMinF_new, DGMinF = DGMinF_new, BGMinF = BGMinF_new, FRDMinF = FRDMinF_new,
			RFDMinF = RFDMinF_new, RFGMinF = RFGMinF_new, RDGMinF = RDGMinF_new, RFRDMinF = RFRDMinF_new,
		    FRDGMinF = FRDGMinF_new, RFDGMinF = RFDGMinF_new, RFRDGMinF = RFRDGMinF_new;
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
double DBBSLB<state, action, environment, criterion, priorityQueue>::GetNextC() {

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
	const auto &forward_b_values = forwardValues.b_values;
	const auto &backward_b_values = backwardValues.b_values;

	for (const double fw_b_value : forward_b_values)
		for (const double bw_b_value : backward_b_values) {
			double b_bound = gcd * std::ceil(((fw_b_value + bw_b_value) / 2) / gcd);
			if (b_bound > C && b_bound < result)
				result = b_bound;
		}
    
    // forward rc bound

	const auto &forward_rf_values = forwardValues.rf_values;
	const auto &backward_rd_values = backwardValues.rd_values;

	for (const double fw_rf_value : forward_rf_values)
		for (const double bw_rd_value : backward_rd_values) {
			double fw_RC_bound = fw_rf_value + bw_rd_value;
			if (fw_RC_bound > C && fw_RC_bound < result)
				result = fw_RC_bound;
		}
    

    // backward rc bound

	const auto &forward_rd_values = forwardValues.rd_values;
	const auto &backward_rf_values = backwardValues.rf_values;

	for (const double bw_rf_value : backward_rf_values)
		for (const double fw_rd_value : forward_rd_values) {
			double bw_RC_bound = bw_rf_value + fw_rd_value;
			if (bw_RC_bound > C && bw_RC_bound < result)
				result = bw_RC_bound;
		}
    

    // fgdg bound
    const auto &forward_fg_values = forwardValues.fg_values;
    const auto &backward_dg_values = backwardValues.dg_values;

    for (const double fw_fg_value : forward_fg_values)
        for (const double bw_dg_value : backward_dg_values) {
            double fgdg_bound = gcd * std::ceil(((fw_fg_value + bw_dg_value + epsilon) / 2) / gcd);
            if (fgdg_bound > C && fgdg_bound < result)
                result = fgdg_bound;
        }
		
    // dgfg bound
    const auto &forward_dg_values = forwardValues.dg_values;
    const auto &backward_fg_values = backwardValues.fg_values;

    for (const double fw_dg_value : forward_dg_values)
        for (const double bw_fg_value : backward_fg_values) {
            double dgfg_bound = gcd * std::ceil(((fw_dg_value + bw_fg_value + epsilon) / 2) / gcd);
            if (dgfg_bound > C && dgfg_bound < result)
                result = dgfg_bound;
        }
		
	// bgbg bound
    const auto &forward_bg_values = forwardValues.bg_values;
    const auto &backward_bg_values = backwardValues.bg_values;

    for (const double fw_bg_value : forward_bg_values)
        for (const double bw_bg_value : backward_bg_values) {
            double bgbg_bound = gcd * std::ceil(((fw_bg_value + bw_bg_value + epsilon) / 3) / gcd);
            if (bgbg_bound > C && bgbg_bound < result)
                result = bgbg_bound;
        }
		
    // frdrfd bound
    const auto &forward_frd_values = forwardValues.frd_values;
    const auto &backward_rfd_values = backwardValues.rfd_values;

    for (const double fw_frd_value : forward_frd_values)
        for (const double bw_rfd_value : backward_rfd_values) {
            double frdrfd_bound = gcd * std::ceil(((fw_frd_value + bw_rfd_value) / 2) / gcd);
            if (frdrfd_bound > C && frdrfd_bound < result)
                result = frdrfd_bound;
        }	
		
    // rfdfrd bound
    const auto &forward_rfd_values = forwardValues.rfd_values;
    const auto &backward_frd_values = backwardValues.frd_values;

    for (const double fw_rfd_value : forward_rfd_values)
        for (const double bw_frd_value : backward_frd_values) {
            double rfdfrd_bound = gcd * std::ceil(((fw_rfd_value + bw_frd_value) / 2) / gcd);
            if (rfdfrd_bound > C && rfdfrd_bound < result)
                result = rfdfrd_bound;
        }
		
    // rfgrdg bound
    const auto &forward_rfg_values = forwardValues.rfg_values;
    const auto &backward_rdg_values = backwardValues.rdg_values;

    for (const double fw_rfg_value : forward_rfg_values)
        for (const double bw_rdg_value : backward_rdg_values) {
            double rfgrdg_bound = gcd * std::ceil(((fw_rfg_value + bw_rdg_value + epsilon) / 2) / gcd);
            if (rfgrdg_bound > C && rfgrdg_bound < result)
                result = rfgrdg_bound;
        }	

    // rdgrfg bound
    const auto &forward_rdg_values = forwardValues.rdg_values;
    const auto &backward_rfg_values = backwardValues.rfg_values;

    for (const double fw_rdg_value : forward_rdg_values)
        for (const double bw_rfg_value : backward_rfg_values) {
            double rdgrfg_bound = gcd * std::ceil(((fw_rdg_value + bw_rfg_value + epsilon) / 2) / gcd);
            if (rdgrfg_bound > C && rdgrfg_bound < result)
                result = rdgrfg_bound;
        }

    // rfrdrfrd bound
    const auto &forward_rfrd_values = forwardValues.rfrd_values;
    const auto &backward_rfrd_values = backwardValues.rfrd_values;

    for (const double fw_rfrd_value : forward_rfrd_values)
        for (const double bw_rfrd_value : backward_rfrd_values) {
            double rfrdrfrd_bound = gcd * std::ceil(((fw_rfrd_value + bw_rfrd_value) / 2) / gcd);
            if (rfrdrfrd_bound > C && rfrdrfrd_bound < result)
                result = rfrdrfrd_bound;
        }	

    // frdgrfdg bound
    const auto &forward_frdg_values = forwardValues.frdg_values;
    const auto &backward_rfdg_values = backwardValues.rfdg_values;

    for (const double fw_frdg_value : forward_frdg_values)
        for (const double bw_rfdg_value : backward_rfdg_values) {
            double frdgrfdg_bound = gcd * std::ceil(((fw_frdg_value + bw_rfdg_value + epsilon) / 3) / gcd);
            if (frdgrfdg_bound > C && frdgrfdg_bound < result)
                result = frdgrfdg_bound;
        }		
		
    // rfdgfrdg bound
    const auto &forward_rfdg_values = forwardValues.rfdg_values;
    const auto &backward_frdg_values = backwardValues.frdg_values;

    for (const double fw_rfdg_value : forward_rfdg_values)
        for (const double bw_frdg_value : backward_frdg_values) {
            double rfdgfrdg_bound = gcd * std::ceil(((fw_rfdg_value + bw_frdg_value + epsilon) / 3) / gcd);
            if (rfdgfrdg_bound > C && rfdgfrdg_bound < result)
                result = rfdgfrdg_bound;
        }	
		
	    // rfrdgrfrdg bound
    const auto &forward_rfrdg_values = forwardValues.rfrdg_values;
    const auto &backward_rfrdg_values = backwardValues.rfrdg_values;

    for (const double fw_rfrdg_value : forward_rfrdg_values)
        for (const double bw_rfrdg_value : backward_rfrdg_values) {
            double rfrdgrfrdg_bound = gcd * std::ceil(((fw_rfrdg_value + bw_rfrdg_value + epsilon) / 3) / gcd);
            if (rfrdgrfrdg_bound > C && rfrdgrfrdg_bound < result)
                result = rfrdgrfrdg_bound;
        }	

    return result;
}

template<class state, class action, class environment, MinCriterion criterion, class priorityQueue>
void DBBSLB<state, action, environment, criterion, priorityQueue>::RunAlgorithm() {
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
void DBBSLB<state, action, environment, criterion, priorityQueue>::ExpandFromBestBucket(priorityQueue &current,
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
