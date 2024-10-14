#ifndef BESTBUCKETBASEDLIST_H
#define BESTBUCKETBASEDLIST_H

#include <cassert>
#include <vector>
#include <stdint.h>
#include <limits>
#include <climits>
#include <functional>
#include "BucketBasedList.h"
#include "BidirErrorBucketBasedList.h"
#include "DBBS.h"

template<typename state, class environment, class dataStructure = BucketNodeData<state>, MinCriterion criterion = MinCriterion::MinG>
class BestBucketBasedList : public BidirErrorBucketBasedList<state, environment, dataStructure> {

    using Bucket = std::vector<const state *>;
    using BidirErrorBucketBasedList<state, environment, dataStructure>::fLayers;
    using BidirErrorBucketBasedList<state, environment, dataStructure>::table;


public:

    BestBucketBasedList() : BidirErrorBucketBasedList<state, environment, dataStructure>() {}

    ~BestBucketBasedList() {}

    virtual bool AddOpenNode(const state val, double g, double h, double h_nx, const state *parent = nullptr);

    std::pair<const state *, double> Pop();

    inline double getMinG() { return checkBestBucketAndReturnValue(minG); }

    inline double getMinF() { return checkBestBucketAndReturnValue(minF); }

    inline double getMinD() { return checkBestBucketAndReturnValue(minD); }

    inline double getMinB() { return checkBestBucketAndReturnValue(minB); }

    inline double getMinRF() { return checkBestBucketAndReturnValue(minRF); }

    inline double getMinRD() { return checkBestBucketAndReturnValue(minRD); }

    inline bool isBestBucketComputed() { return bestBucket != nullptr; }

    inline int getExpandableNodes() { return expandableNodes; }
	
	inline int getOptCount() { return m_optCount; }

    void computeBestBucket(double gLim_, double fLim_, double dLim_,
                           double bLim_, double rfLim_, double rdLim_);

    void countExpandableNodes();

private:

    Bucket *bestBucket = nullptr;
	int m_optCount = 0;

    int expandableNodes = INT_MAX;

    double minG = DBL_MAX, minF = DBL_MAX, minD = DBL_MAX, minB = DBL_MAX, minRF = DBL_MAX, minRD = DBL_MAX;
    double gLim = DBL_MAX, fLim = DBL_MAX, dLim = DBL_MAX, bLim = DBL_MAX, rfLim = DBL_MAX, rdLim = DBL_MAX;

    inline double checkBestBucketAndReturnValue(double value) {
        if (bestBucket == nullptr) throw std::runtime_error("No cached best bucket");
        return value;
    }

    inline void invalidateCachedValues() {
        bestBucket = nullptr;
        expandableNodes = INT_MAX;
        minG = DBL_MAX;
        minF = DBL_MAX;
        minD = DBL_MAX;
        minB = DBL_MAX;
        minRF = DBL_MAX;
        minRD = DBL_MAX;
    }


    // TODO parametrize this
    bool useB = true;
    bool useRC = true;
};

template<typename state, class environment, class dataStructure, MinCriterion criterion>
bool BestBucketBasedList<state, environment, dataStructure, criterion>::AddOpenNode(const state val,
                                                                                    const double g,
                                                                                    const double h,
                                                                                    const double h_nx,
                                                                                    const state *parent) {
    bool added = BidirErrorBucketBasedList<state, environment, dataStructure>::AddOpenNode(val, g, h, h_nx, parent);

    const double f = g + h;
    const double d = g - h_nx;

    if (added &&
        (g < minG || f < minF || d < minD
         || (useB && (f + d) < minB)
         || (useRC && (g - h) < minRF) || (useRC && (g + h_nx) < minRD)))
        invalidateCachedValues();

    return added;
}

template<typename state, class environment, class dataStructure, MinCriterion criterion>
void BestBucketBasedList<state, environment, dataStructure, criterion>::computeBestBucket(double gLim_,
                                                                                          double fLim_,
                                                                                          double dLim_,
                                                                                          double bLim_,
                                                                                          double rfLim_,
                                                                                          double rdLim_) {
    invalidateCachedValues();
    gLim = gLim_, fLim = fLim_, dLim = dLim_, bLim = bLim_, rfLim = rfLim_, rdLim = rdLim_;
	
	int gCount = 0, fCount = 0, dCount = 0, bCount = 0, rfCount = 0, rdCount = 0;
	
	Bucket *bBucket = nullptr;
	Bucket *gBucket = nullptr;
	Bucket *fBucket = nullptr;
	Bucket *dBucket = nullptr;
	Bucket *rfBucket = nullptr;
	Bucket *rdBucket = nullptr;

    auto gLayerIt = fLayers.begin();
    while (gLayerIt != fLayers.end()) {
        double g = gLayerIt->first;

        if (g > gLim) break;

        auto &gLayer = gLayerIt->second;

        if (gLayer.size() == 0) { // if the whole g layer is empty, erase it
            gLayerIt = fLayers.erase(gLayerIt);
            continue;
        }

        auto fLayerIt = gLayer.begin();
        while (fLayerIt != gLayer.end()) {
            double h = fLayerIt->first;
            double f = g + h;

            if (f > fLim) break;

            // check its rf value against rfLim
            double rfValue = g - h;
            if (useRC && rfValue > rfLim) {
                fLayerIt++;
                continue; // continue because rf is decreasing
            }

            auto &fLayer = fLayerIt->second;

            if (fLayer.size() == 0) { // if the whole f layer is empty, erase it
                fLayerIt = gLayer.erase(fLayerIt);
                continue;
            }

            auto dLayerIt = fLayer.begin();
            while (dLayerIt != fLayer.end()) {
                double h_nx = dLayerIt->first;
                double d = g - h_nx;

                // deal with bucket - first, check that is not empty
                Bucket &bucket = dLayerIt->second;
                if (bucket.size() == 0) {
                    dLayerIt = fLayer.erase(dLayerIt);
                    continue;
                }

                if (d > dLim) break;

                // check its b value against bLim
                double bValue = f + d;
                if (useB && bValue > bLim) {
                    dLayerIt++;
                    break;
                }

                // check its rd value against rdLim
                double rdValue = g + h_nx;
                if (useRC && rdValue > rdLim) {
                    dLayerIt++;
                    continue; // continue because rd is decreasing
                }

                // pick the bucket as best bucket if it is the best based on the criterion			
				if (g < minG) {
                    minG = g;
					gBucket = &bucket;
                    if (criterion == MinCriterion::MinG) bestBucket = &bucket;
					gCount = bucket.size();
                } else if (g == minG){
					gCount += bucket.size();
				}
					
                if (f < minF) {
                    minF = f;
					fBucket = &bucket;
                    if (criterion == MinCriterion::MinF) bestBucket = &bucket;
					fCount = bucket.size();
                } else if (f == minF){
					fCount += bucket.size();
				}

                if (d < minD) {
                    minD = d;
					dBucket = &bucket;
                    if (criterion == MinCriterion::MinD) bestBucket = &bucket;
					dCount = bucket.size();
                } else if (d == minD){
					dCount += bucket.size();
				}

                if (useB && bValue < minB) {
                    minB = bValue;
					bBucket = &bucket;
                    if (criterion == MinCriterion::MinB) bestBucket = &bucket;
					bCount = bucket.size();
                } else if (useB && bValue == minB){
					bCount += bucket.size();
				}

                if (useRC && rfValue < minRF) {
                    minRF = rfValue;
					rfBucket = &bucket;
					rfCount = bucket.size();
                } else if (useRC && rfValue == minRF){
					rfCount += bucket.size();
				}
				
				if (useRC && rdValue < minRD) {
                    minRD = rdValue;
					rdBucket = &bucket;
					rdCount = bucket.size();
                } else if (useRC && rdValue == minRD){
					rdCount += bucket.size();
				}

                dLayerIt++;
            }
            fLayerIt++;
        }
        gLayerIt++;
    }
	
	int optCount = 0;
	if (criterion == MinCriterion::MinTot){
		optCount = std::min({gCount, fCount, dCount, bCount, rfCount, rdCount});
	} else if (criterion == MinCriterion::MaxTot){
		optCount = std::max({gCount, fCount, dCount, bCount, rfCount, rdCount});
	}
	m_optCount = optCount;
	if (optCount == 0) return;
	if (bCount == optCount) bestBucket = bBucket;
	else if (gCount == optCount) bestBucket = gBucket;
	else if (fCount == optCount) bestBucket = fBucket;
	else if (dCount == optCount) bestBucket = dBucket;
	else if (rfCount == optCount) bestBucket = rfBucket;
	else if (rdCount == optCount) bestBucket = rdBucket;
}

template<typename state, class environment, class dataStructure, MinCriterion criterion>
std::pair<const state *, double> BestBucketBasedList<state, environment, dataStructure, criterion>::Pop() {
    const state *poppedState = nullptr;

    while (poppedState == nullptr) {
        if (!isBestBucketComputed()) {
            return std::make_pair(nullptr, -1); // only pop when a proper bucket is known based on some limits
        }

        poppedState = bestBucket->back();
        bestBucket->pop_back();
        expandableNodes--;
        if (bestBucket->size() == 0)
            invalidateCachedValues(); // whenever a bucket is emptied, the cache must be invalidated
    }

    auto &node = table.at(*poppedState);
    node.bucket_index = -1;
    return std::make_pair(poppedState, node.g);
}


template<typename state, class environment, class dataStructure, MinCriterion criterion>
void BestBucketBasedList<state, environment, dataStructure, criterion>::countExpandableNodes() {

    expandableNodes = 0;
    for (const auto &glayer : fLayers) {
        double g = glayer.first;

        if (g > gLim) break;

        for (const auto &fLayer : glayer.second) {
            double h = fLayer.first;
            double f = g + h;
            double rf = g - h;

            if (f > fLim) break;

            if (useRC && rf > rfLim) continue; // continue because rf is decreasing

            for (const auto &bucket : fLayer.second) {
                double h_nx = bucket.first;
                double d = g - h_nx;
                double rd = g + h_nx;
                double b = f + d;

                if (d > dLim || (useB && b > bLim)) break;

                if (useRC && rd > rdLim) continue; // continue because rd is decreasing

                expandableNodes += bucket.second.size();
            }
        }
    }
}


#endif