#ifndef CDBBS_H
#define CDBBS_H

#include <unordered_set>
#include <iostream>
#include <math.h>
#include <utility>
#include <vector>
#include "FrontToEnd.h"
#include "BidirErrorBucketBasedList.h"

struct g_bucket_sorter {
    bool operator()(BucketInfo i, BucketInfo j) const {
        if (i.g != j.g) return i.g < j.g;
        if (i.h != j.h) return i.h < j.h;
        return j.h_nx < i.h_nx; // higher reverse heuristic means lower d, so reverse the condition
    }
};

struct f_bucket_sorter {
    bool operator()(BucketInfo i, BucketInfo j) const {
        double i_f = i.g + i.h;
        double j_f = j.g + j.h;
        if (i_f != j_f) return i_f < j_f;
        if (i.g != j.g) return i.g < j.g;
        return j.h_nx < i.h_nx; // higher reverse heuristic means lower d, so reverse the condition
    }
};

struct d_bucket_sorter {
    bool operator()(BucketInfo i, BucketInfo j) const {
        double i_d = i.g - i.h_nx;
        double j_d = j.g - j.h_nx;
        if (i_d != j_d) return i_d < j_d;
        if (i.g != j.g) return i.g < j.g;
        return i.h < j.h;
    }
};

struct b_bucket_sorter {
    bool operator()(BucketInfo i, BucketInfo j) const {
        double i_b = i.g + i.h + i.g - i.h_nx;
        double j_b = j.g + j.h + j.g - j.h_nx;
        if (i_b != j_b) return i_b < j_b;
        if (i.g != j.g) return i.g < j.g;
        return i.h < j.h;
    }
};

struct rf_bucket_sorter {
    bool operator()(BucketInfo i, BucketInfo j) const {
        double i_rf = i.g - i.h;
        double j_rf = j.g - j.h;
        if (i_rf != j_rf) return i_rf < j_rf;
        if (i.g != j.g) return i.g < j.g;
        return i.h < j.h;
    }
};

struct rc_bucket_sorter {
    bool operator()(BucketInfo i, BucketInfo j) const {
        double i_rc = i.g + i.h_nx;
        double j_rc = j.g + j.h_nx;
        if (i_rc != j_rc) return i_rc < j_rc;
        if (i.g != j.g) return i.g < j.g;
        return i.h < j.h;
    }
};

struct MinBuckets {

    double min_g = 0.0, min_f = 0.0, min_d = 0.0, min_b = 0.0, min_rf = 0.0, min_rc = 0.0;

    std::set <BucketInfo, g_bucket_sorter> g_buckets;
    std::set <BucketInfo, f_bucket_sorter> f_buckets;
    std::set <BucketInfo, d_bucket_sorter> d_buckets;
    std::set <BucketInfo, b_bucket_sorter> b_buckets;
    std::set <BucketInfo, rf_bucket_sorter> rf_buckets;
    std::set <BucketInfo, rc_bucket_sorter> rc_buckets;

    MinBuckets(const std::vector <BucketInfo> &bucketList) :
            g_buckets(std::set<BucketInfo, g_bucket_sorter>(bucketList.begin(), bucketList.end())),
            f_buckets(std::set<BucketInfo, f_bucket_sorter>(bucketList.begin(), bucketList.end())),
            d_buckets(std::set<BucketInfo, d_bucket_sorter>(bucketList.begin(), bucketList.end())),
            b_buckets(std::set<BucketInfo, b_bucket_sorter>(bucketList.begin(), bucketList.end())),
            rf_buckets(std::set<BucketInfo, rf_bucket_sorter>(bucketList.begin(), bucketList.end())),
            rc_buckets(std::set<BucketInfo, rc_bucket_sorter>(bucketList.begin(), bucketList.end())) {
        UpdateMins();
    }

    void UpdateMins() {
        min_g = g_buckets.begin()->g;
        min_f = f_buckets.begin()->g + f_buckets.begin()->h;
        min_d = d_buckets.begin()->g - d_buckets.begin()->h_nx;
        min_b = b_buckets.begin()->g + b_buckets.begin()->h + b_buckets.begin()->g - b_buckets.begin()->h_nx;
        min_rf = rf_buckets.begin()->g - rf_buckets.begin()->h;
        min_rc = rc_buckets.begin()->g + rc_buckets.begin()->h_nx;
    }

    void Clear() {
        min_g = 0.0;
        min_f = 0.0;
        min_d = 0.0;
        min_b = 0.0;
        min_rf = 0.0;
        min_rc = 0.0;
        g_buckets.clear();
        f_buckets.clear();
        d_buckets.clear();
        b_buckets.clear();
        rf_buckets.clear();
        rc_buckets.clear();
    }

    bool IsEmpty() {
        return g_buckets.empty() || f_buckets.empty() || d_buckets.empty() ||
               b_buckets.empty() || rf_buckets.empty() || rc_buckets.empty();
    }

    bool RemoveBucketInfo(const BucketInfo &info) {
        double old_min_g = min_g, old_min_f = min_f, old_min_d = min_d,
                old_min_b = min_b, old_min_rf = min_rf, old_min_rc = min_rc;

        g_buckets.erase(info);
        f_buckets.erase(info);
        d_buckets.erase(info);
        b_buckets.erase(info);
        rf_buckets.erase(info);
        rc_buckets.erase(info);


//        std::cout << "    Remove bucket info: " << (IsEmpty() || old_min_g != min_g || old_min_f != min_f || old_min_d != min_d,
//                old_min_b != min_b || old_min_rf != min_rf || old_min_rc != min_rc) << std::endl;

        return IsEmpty() || old_min_g != min_g || old_min_f != min_f || old_min_d != min_d,
                old_min_b != min_b || old_min_rf != min_rf || old_min_rc != min_rc;
    }

    BucketInfo GetBucket(const MinCriterion &criterion) {
        switch (criterion) {
            case MinG:
                return *(g_buckets.begin());
            case MinF:
                return *(f_buckets.begin());
            case MinD:
                return *(d_buckets.begin());
            case MinB:
                return *(b_buckets.begin());
        }
    }

};

template<class state, class action, class environment, MinCriterion criterion, class priorityQueue = BidirErrorBucketBasedList<state, environment, BucketNodeData<state>>>
class CDBBS : public FrontToEnd<state, action, environment, priorityQueue> {

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
    using FrontToEnd<state, action, environment, priorityQueue>::CheckSolution;

public:
    CDBBS(double epsilon_ = 1.0, double gcd_ = 1.0)
            : FrontToEnd<state, action, environment, priorityQueue>(epsilon_), gcd(gcd_),
              fwBuckets(MinBuckets(forwardQueue.getBucketInfo())),
              bwBuckets(MinBuckets(backwardQueue.getBucketInfo())) {}

    ~CDBBS() {}

    virtual const char *GetName() { return "CDBBS"; }

protected:

    // add thinking using b and rc parameters

    double gcd;

    bool expandForward = true;

    MinBuckets fwBuckets;
    MinBuckets bwBuckets;

    virtual void RunAlgorithm();

    bool UpdateC();

    void DelayBuckets();

};

template<class state, class action, class environment, MinCriterion criterion, class priorityQueue>
void CDBBS<state, action, environment, criterion, priorityQueue>::RunAlgorithm() {

    fwBuckets = MinBuckets(forwardQueue.getBucketInfo());
    bwBuckets = MinBuckets(backwardQueue.getBucketInfo());

//    std::cout << "  Initial C: " << C << std::endl;

    while (!forwardQueue.IsEmpty() && !backwardQueue.IsEmpty()) {

        if (UpdateC()) {
            // TODO think how we are going to parametrize the tie breaker
            if (CheckSolution()) break; // optimality can be proven after updating C
        }

        // alternate directions only, as caching leads to less reliable info
        if (expandForward) {
            const BucketInfo &info = fwBuckets.GetBucket(criterion);
            auto pop = forwardQueue.PopBucket(info.g, info.h, info.h_nx);
            Expand(pop, info.g, forwardQueue, backwardQueue, forwardHeuristic, backwardHeuristic, goal, start);
            expandForward = false;
        } else {
            const BucketInfo &info = bwBuckets.GetBucket(criterion);
            auto pop = backwardQueue.PopBucket(info.g, info.h, info.h_nx);
            Expand(pop, info.g, backwardQueue, forwardQueue, backwardHeuristic, forwardHeuristic, start, goal);
            expandForward = true;
        }

        if (CheckSolution()) break;
    }
}

template<class state, class action, class environment, MinCriterion criterion, class priorityQueue>
bool CDBBS<state, action, environment, criterion, priorityQueue>::UpdateC() {
    double oldC = C;

    while (C < currentCost) {
        bool fwLimitsChanged = false;
        bool bwLimitsChanged = false;

        // find the next fw bucket with nodes
        BucketInfo fwBucket = fwBuckets.GetBucket(criterion);
        // keep trying to get a valid bucket
        while (forwardQueue.RemoveIfEmpty(fwBucket.g, fwBucket.h, fwBucket.h_nx)) {
//            std::cout << "  fw: " << fwBucket.g << " - " << fwBucket.h << " - " << fwBucket.h_nx << std::endl;
//            std::cout << "    empty?: " << fwBuckets.IsEmpty() << std::endl;
//            std::cout << "    size: " << fwBuckets.b_buckets.size() << std::endl;

            fwLimitsChanged |= fwBuckets.RemoveBucketInfo(fwBucket) || fwBuckets.IsEmpty();
            if (!fwBuckets.IsEmpty()) fwBucket = fwBuckets.GetBucket(criterion);
            else break;
        }

        // find the next bw bucket with nodes
        BucketInfo bwBucket = bwBuckets.GetBucket(criterion);
        // keep trying to get a valid bucket
        while (backwardQueue.RemoveIfEmpty(bwBucket.g, bwBucket.h, bwBucket.h_nx)) {
//            std::cout << "  bw: " << bwBucket.g << " - " << bwBucket.h << " - " << bwBucket.h_nx << std::endl;
//            std::cout << "    empty?: " << bwBuckets.IsEmpty() << std::endl;

            bwLimitsChanged |= bwBuckets.RemoveBucketInfo(bwBucket) || bwBuckets.IsEmpty();
            if (!bwBuckets.IsEmpty()) bwBucket = bwBuckets.GetBucket(criterion);
            else break;
        }

        if (!fwLimitsChanged && !bwLimitsChanged) break; // no changes, so no recalculation

        while (true) {
            // limits changed, reload the buckets
            fwBuckets = MinBuckets(forwardQueue.getBucketInfo());
            bwBuckets = MinBuckets(backwardQueue.getBucketInfo());

            // delay buckets
            DelayBuckets();

            if (fwBuckets.IsEmpty() || bwBuckets.IsEmpty()) {


                C += gcd;   // all buckets were delayed: C must be increased
//                std::cout << "  C updated: " << C << " with nodes: " << nodesExpanded << " and solution : "
//                          << currentCost << std::endl;
            } else { break; }// not all were delayed, so keep searching
        }
    }

    return oldC != C;
}

template<class state, class action, class environment, MinCriterion criterion, class priorityQueue>
void CDBBS<state, action, environment, criterion, priorityQueue>::DelayBuckets() {

    while (!fwBuckets.IsEmpty() && !bwBuckets.IsEmpty()) {

        // delay forward buckets
        bool fwUpdated = false;
        std::vector <BucketInfo> fwDelayed;
        for (const auto &fwBucket : fwBuckets.g_buckets) {
            if (fwBucket.g + bwBuckets.min_g + epsilon > C) fwDelayed.push_back(fwBucket); // g bound
            else if (fwBucket.g + fwBucket.h + bwBuckets.min_d > C) fwDelayed.push_back(fwBucket); // fw KK bound
            else if (fwBucket.g - fwBucket.h_nx + bwBuckets.min_f > C) fwDelayed.push_back(fwBucket); // bw KK bound
            else if (2 * fwBucket.g + fwBucket.h - fwBucket.h_nx + bwBuckets.min_b > 2 * C)
                fwDelayed.push_back(fwBucket); // b bound
            else if (fwBucket.g - fwBucket.h + bwBuckets.min_rc > C) fwDelayed.push_back(fwBucket); // rf bound
            else if (fwBucket.g + fwBucket.h_nx + bwBuckets.min_rf > C) fwDelayed.push_back(fwBucket); // rb bound
        }

        for (const auto &fwDelayedBucket : fwDelayed)
            fwUpdated |= fwBuckets.RemoveBucketInfo(fwDelayedBucket);

        // delay forward buckets
        bool bwUpdated = false;
        std::vector <BucketInfo> bwDelayed;
        for (const auto &bwBucket : bwBuckets.g_buckets) {
            if (bwBucket.g + fwBuckets.min_g + epsilon > C) bwDelayed.push_back(bwBucket); // g bound
            else if (bwBucket.g + bwBucket.h + fwBuckets.min_d > C) bwDelayed.push_back(bwBucket); // fw KK bound
            else if (bwBucket.g - bwBucket.h_nx + fwBuckets.min_f > C) bwDelayed.push_back(bwBucket); // bw KK bound
            else if (2 * bwBucket.g + bwBucket.h - bwBucket.h_nx + fwBuckets.min_b > 2 * C)
                bwDelayed.push_back(bwBucket); // b bound
            else if (bwBucket.g - bwBucket.h + fwBuckets.min_rc > C) bwDelayed.push_back(bwBucket); // rf bound
            else if (bwBucket.g + bwBucket.h_nx + fwBuckets.min_rf > C) bwDelayed.push_back(bwBucket); // rb bound
        }

        for (const auto &bwDelayedBucket : bwDelayed)
            bwUpdated |= bwBuckets.RemoveBucketInfo(bwDelayedBucket);

        if (!fwUpdated && !bwUpdated) break;
    }
}


#endif
