#ifndef APPS_AIJ_LBBESTBUCKETBASEDLIST_H
#define APPS_AIJ_LBBESTBUCKETBASEDLIST_H

#include <cassert>
#include <vector>
#include <stdint.h>
#include <limits>
#include <climits>
#include <functional>
#include <initializer_list>
#include <algorithm>
#include "BucketBasedList.h"
#include "BidirErrorBucketBasedList.h"
#include "DBBSLB.h"

template<typename state, class environment, class dataStructure = BucketNodeData<state>, MinCriterion criterion = MinCriterion::MinG>
class LBBestBucketBasedList : public BidirErrorBucketBasedList<state, environment, dataStructure> {

    using Bucket = std::vector<const state *>;
    using BidirErrorBucketBasedList<state, environment, dataStructure>::fLayers;
    using BidirErrorBucketBasedList<state, environment, dataStructure>::table;


public:

    LBBestBucketBasedList() : BidirErrorBucketBasedList<state, environment, dataStructure>() {}

    ~LBBestBucketBasedList() {}

    virtual bool AddOpenNode(const state val, double g, double h, double h_nx, const state *parent = nullptr);

    std::pair<const state *, double> Pop();

    inline double getMinG() { return checkBestBucketAndReturnValue(minG); }

    inline double getMinF() { return checkBestBucketAndReturnValue(minF); }

    inline double getMinD() { return checkBestBucketAndReturnValue(minD); }

    inline double getMinB() { return checkBestBucketAndReturnValue(minB); }

    inline double getMinRF() { return checkBestBucketAndReturnValue(minRF); }

    inline double getMinRD() { return checkBestBucketAndReturnValue(minRD); }

    inline double getMinFG() { return checkBestBucketAndReturnValue(minFG); }

    inline double getMinDG() { return checkBestBucketAndReturnValue(minDG); }

    inline double getMinBG() { return checkBestBucketAndReturnValue(minBG); }

    inline double getMinFRD() { return checkBestBucketAndReturnValue(minFRD); }

    inline double getMinRFD() { return checkBestBucketAndReturnValue(minRFD); }

    inline double getMinRFG() { return checkBestBucketAndReturnValue(minRFG); }

    inline double getMinRDG() { return checkBestBucketAndReturnValue(minRDG); }

    inline double getMinRFRD() { return checkBestBucketAndReturnValue(minRFRD); }

    inline double getMinFRDG() { return checkBestBucketAndReturnValue(minFRDG); }

    inline double getMinRFDG() { return checkBestBucketAndReturnValue(minRFDG); }

    inline double getMinRFRDG() { return checkBestBucketAndReturnValue(minRFRDG); }


    inline bool isBestBucketComputed() { return bestBucket != nullptr; }

    inline int getExpandableNodes() { return expandableNodes; }

    inline int getOptCount() { return m_optCount; }

    void computeBestBucket(double gLim_, double fLim_, double dLim_, double bLim_, double rfLim_, double rdLim_,
                           double fgLim_, double dgLim_, double bgLim_, double frdLim_, double rfdLim_, double rfgLim_,
                           double rdgLim_, double rfrdLim_, double frdgLim_, double rfdgLim_, double rfrdgLim_);

    void countExpandableNodes();

    void setCurrentMaxTotExpansion(MinCriterion minCriterion) {
        maxTotMinCriterion = minCriterion;
    }

private:

    Bucket *bestBucket = nullptr;
    int m_optCount = 0;

    int expandableNodes = INT_MAX;

    double minG = DBL_MAX, minF = DBL_MAX, minD = DBL_MAX, minB = DBL_MAX, minRF = DBL_MAX, minRD = DBL_MAX;
    double minFG = DBL_MAX, minDG = DBL_MAX, minBG = DBL_MAX, minFRD = DBL_MAX, minRFD = DBL_MAX, minRFG = DBL_MAX, minRDG = DBL_MAX, minRFRD = DBL_MAX, minFRDG = DBL_MAX, minRFDG = DBL_MAX, minRFRDG = DBL_MAX;
    double gLim = DBL_MAX, fLim = DBL_MAX, dLim = DBL_MAX, bLim = DBL_MAX, rfLim = DBL_MAX, rdLim = DBL_MAX;
    double fgLim = DBL_MAX, dgLim = DBL_MAX, bgLim = DBL_MAX, frdLim = DBL_MAX, rfdLim = DBL_MAX, rfgLim = DBL_MAX, rdgLim = DBL_MAX, rfrdLim = DBL_MAX, frdgLim = DBL_MAX, rfdgLim = DBL_MAX, rfrdgLim = DBL_MAX;

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

        minFG = DBL_MAX;
        minDG = DBL_MAX;
        minBG = DBL_MAX;
        minFRD = DBL_MAX;
        minRFD = DBL_MAX;

        minRFG = DBL_MAX;
        minRDG = DBL_MAX;
        minRFRD = DBL_MAX;
        minFRDG = DBL_MAX;
        minRFDG = DBL_MAX;
        minRFRDG = DBL_MAX;

    }

    MinCriterion maxTotMinCriterion = MinCriterion::MinB;
};

template<typename state, class environment, class dataStructure, MinCriterion criterion>
bool LBBestBucketBasedList<state, environment, dataStructure, criterion>::AddOpenNode(const state val,
                                                                                      const double g,
                                                                                      const double h,
                                                                                      const double h_nx,
                                                                                      const state *parent) {
    bool added = BidirErrorBucketBasedList<state, environment, dataStructure>::AddOpenNode(val, g, h, h_nx, parent);

    const double f = g + h;
    const double d = g - h_nx;

    if (added &&
        (g < minG || f < minF || d < minD
         || ((f + d) < minB)
         || ((g - h) < minRF) || ((g + h_nx) < minRD)
         || (2 * g + h) < minFG || ((2 * g - h_nx) < minDG)
         || (3 * g + h - h_nx) < minBG || ((2 * g + h + h_nx) < minFRD) || ((2 * g - h - h_nx) < minRFD)
         || (2 * g - h) < minRFG || ((2 * g + h_nx) < minRDG)
         || (2 * g - h + h_nx) < minRFRD || ((3 * g + h + h_nx) < minFRDG)
         || (3 * g - h - h_nx) < minRFDG || ((3 * g - h + h_nx) < minRFRDG)
        ))
        invalidateCachedValues();

    return added;
}

template<typename state, class environment, class dataStructure, MinCriterion criterion>
void LBBestBucketBasedList<state, environment, dataStructure, criterion>::computeBestBucket(double gLim_,
                                                                                            double fLim_,
                                                                                            double dLim_,
                                                                                            double bLim_,
                                                                                            double rfLim_,
                                                                                            double rdLim_,
                                                                                            double fgLim_,
                                                                                            double dgLim_,
                                                                                            double bgLim_,
                                                                                            double frdLim_,
                                                                                            double rfdLim_,
                                                                                            double rfgLim_,
                                                                                            double rdgLim_,
                                                                                            double rfrdLim_,
                                                                                            double frdgLim_,
                                                                                            double rfdgLim_,
                                                                                            double rfrdgLim_
) {
    invalidateCachedValues();
    gLim = gLim_, fLim = fLim_, dLim = dLim_, bLim = bLim_, rfLim = rfLim_, rdLim = rdLim_;
    fgLim = fgLim_, dgLim = dgLim_, bgLim = bgLim_, frdLim = frdLim_, rfdLim = rfdLim_, rfgLim = rfgLim_, rdgLim = rdgLim_, rfrdLim = rfrdLim_, frdgLim = frdgLim_, rfdgLim = rfdgLim_, rfrdgLim = rfrdgLim_;

    int gCount = 0, fCount = 0, dCount = 0, bCount = 0, rfCount = 0, rdCount = 0;
    int fgCount = 0, dgCount = 0, bgCount = 0, frdCount = 0, rfdCount = 0, rfgCount = 0, rdgCount = 0, rfrdCount = 0, frdgCount = 0, rfdgCount = 0, rfrdgCount = 0;

    Bucket *bBucket = nullptr;
    Bucket *gBucket = nullptr;
    Bucket *fBucket = nullptr;
    Bucket *dBucket = nullptr;
    Bucket *rfBucket = nullptr;
    Bucket *rdBucket = nullptr;
    Bucket *fgBucket = nullptr;
    Bucket *dgBucket = nullptr;
    Bucket *bgBucket = nullptr;
    Bucket *frdBucket = nullptr;
    Bucket *rfdBucket = nullptr;
    Bucket *rfgBucket = nullptr;
    Bucket *rdgBucket = nullptr;
    Bucket *rfrdBucket = nullptr;
    Bucket *frdgBucket = nullptr;
    Bucket *rfdgBucket = nullptr;
    Bucket *rfrdgBucket = nullptr;

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

            double fgValue = f + g;
            if (fgValue > fgLim) break;

            // check its rf value against rfLim
            double rfValue = g - h;
            if (rfValue > rfLim) {
                fLayerIt++;
                continue; // continue because rf is decreasing
            }

            double rfgValue = 2 * g - h;
            if (rfgValue > rfgLim) {
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

                double dgValue = d + g;
                if (dgValue > dgLim) {
                    dLayerIt++;
                    break;
                }

                // check its b value against bLim
                double bValue = f + d;
                if (bValue > bLim) {
                    dLayerIt++;
                    break;
                }

                double bgValue = bValue + g;
                if (bgValue > bgLim) {
                    dLayerIt++;
                    break;
                }

                // check its rd value against rdLim
                double rdValue = g + h_nx;
                if (rdValue > rdLim) {
                    dLayerIt++;
                    continue; // continue because rd is decreasing
                }

                double frdValue = f + rdValue;
                if (frdValue > frdLim) {
                    dLayerIt++;
                    continue; // continue because rd is decreasing
                }

                double rfdValue = rfValue + d;
                if (rfdValue > rfdLim) {
                    dLayerIt++;
                    break;
                }

                double rfrdValue = rfValue + rdValue;
                if (rfrdValue > rfrdLim) {
                    dLayerIt++;
                    continue; // continue because rd is decreasing
                }

                double rdgValue = rdValue + g;
                if (rdgValue > rdgLim) {
                    dLayerIt++;
                    continue; // continue because rd is decreasing
                }

                double frdgValue = f + rdgValue;
                if (frdgValue > frdgLim) {
                    dLayerIt++;
                    continue; // continue because rd is decreasing
                }

                double rfdgValue = rfValue + d + g;
                if (rfdgValue > rfdgLim) {
                    dLayerIt++;
                    break;
                }

                double rfrdgValue = rfValue + rdgValue;
                if (rfrdgValue > rfrdgLim) {
                    dLayerIt++;
                    continue; // continue because rd is decreasing
                }

                // pick the bucket as best bucket if it is the best based on the criterion
                if (g < minG) {
                    minG = g;
                    gBucket = &bucket;
                    if (criterion == MinCriterion::MinG ||
                        (criterion == MinCriterion::MaxTot && maxTotMinCriterion == MinCriterion::MinG)) {
                        bestBucket = &bucket;
                    }

                    gCount = bucket.size();
                } else if (g == minG) {
                    gCount += bucket.size();
                }

                if (f < minF) {
                    minF = f;
                    fBucket = &bucket;
                    if (criterion == MinCriterion::MinF ||
                        (criterion == MinCriterion::MaxTot && maxTotMinCriterion == MinCriterion::MinF)) {
                        bestBucket = &bucket;
                    }
                    fCount = bucket.size();
                } else if (f == minF) {
                    fCount += bucket.size();
                }

                if (d < minD) {
                    minD = d;
                    dBucket = &bucket;
                    if (criterion == MinCriterion::MinD ||
                        (criterion == MinCriterion::MaxTot && maxTotMinCriterion == MinCriterion::MinD)) {
                        bestBucket = &bucket;
                    }
                    dCount = bucket.size();
                } else if (d == minD) {
                    dCount += bucket.size();
                }

                if (bValue < minB) {
                    minB = bValue;
                    bBucket = &bucket;
                    if (criterion == MinCriterion::MinB ||
                        (criterion == MinCriterion::MaxTot && maxTotMinCriterion == MinCriterion::MinB)) {
                        bestBucket = &bucket;
                    }
                    bCount = bucket.size();
                } else if (bValue == minB) {
                    bCount += bucket.size();
                }

                if (rfValue < minRF) {
                    minRF = rfValue;
                    rfBucket = &bucket;
                    if (criterion == MinCriterion::MaxTot && maxTotMinCriterion == MinCriterion::MinRF) {
                        bestBucket = &bucket;
                    }
                    rfCount = bucket.size();
                } else if (rfValue == minRF) {
                    rfCount += bucket.size();
                }

                if (rdValue < minRD) {
                    minRD = rdValue;
                    rdBucket = &bucket;
                    if (criterion == MinCriterion::MaxTot && maxTotMinCriterion == MinCriterion::MinRD) {
                        bestBucket = &bucket;
                    }
                    rdCount = bucket.size();
                } else if (rdValue == minRD) {
                    rdCount += bucket.size();
                }

                if (dgValue < minDG) {
                    minDG = dgValue;
                    dgBucket = &bucket;
                    if (criterion == MinCriterion::MaxTot && maxTotMinCriterion == MinCriterion::MinGD) {
                        bestBucket = &bucket;
                    }
                    dgCount = bucket.size();
                } else if (dgValue == minDG) {
                    dgCount += bucket.size();
                }

                if (fgValue < minFG) {
                    minFG = fgValue;
                    fgBucket = &bucket;
                    if (criterion == MinCriterion::MaxTot && maxTotMinCriterion == MinCriterion::MinGF) {
                        bestBucket = &bucket;
                    }
                    fgCount = bucket.size();
                } else if (fgValue == minFG) {
                    fgCount += bucket.size();
                }

                if (bgValue < minBG) {
                    minBG = bgValue;
                    bgBucket = &bucket;
                    if (criterion == MinCriterion::MaxTot && maxTotMinCriterion == MinCriterion::MinGB) {
                        bestBucket = &bucket;
                    }
                    bgCount = bucket.size();
                } else if (bgValue == minBG) {
                    bgCount += bucket.size();
                }

                if (frdValue < minFRD) {
                    minFRD = frdValue;
                    frdBucket = &bucket;
                    if (criterion == MinCriterion::MaxTot && maxTotMinCriterion == MinCriterion::MinFRD) {
                        bestBucket = &bucket;
                    }
                    frdCount = bucket.size();
                } else if (frdValue == minFRD) {
                    frdCount += bucket.size();
                }

                if (rfdValue < minRFD) {
                    minRFD = rfdValue;
                    rfdBucket = &bucket;
                    if (criterion == MinCriterion::MaxTot && maxTotMinCriterion == MinCriterion::MinRFD) {
                        bestBucket = &bucket;
                    }
                    rfdCount = bucket.size();
                } else if (rfdValue == minRFD) {
                    rfdCount += bucket.size();
                }

                if (rfgValue < minRFG) {
                    minRFG = rfgValue;
                    rfgBucket = &bucket;
                    if (criterion == MinCriterion::MaxTot && maxTotMinCriterion == MinCriterion::MinGRF) {
                        bestBucket = &bucket;
                    }
                    rfgCount = bucket.size();
                } else if (rfgValue == minRFG) {
                    rfgCount += bucket.size();
                }

                if (rdgValue < minRDG) {
                    minRDG = rdgValue;
                    rdgBucket = &bucket;
                    if (criterion == MinCriterion::MaxTot && maxTotMinCriterion == MinCriterion::MinGRD) {
                        bestBucket = &bucket;
                    }
                    rdgCount = bucket.size();
                } else if (rdgValue == minRDG) {
                    rdgCount += bucket.size();
                }

                if (rfrdValue < minRFRD) {
                    minRFRD = rfrdValue;
                    rfrdBucket = &bucket;
                    if (criterion == MinCriterion::MaxTot && maxTotMinCriterion == MinCriterion::MinRFRD) {
                        bestBucket = &bucket;
                    }
                    rfrdCount = bucket.size();
                } else if (rfrdValue == minRFRD) {
                    rfrdCount += bucket.size();
                }

                if (frdgValue < minFRDG) {
                    minFRDG = frdgValue;
                    frdgBucket = &bucket;
                    if (criterion == MinCriterion::MaxTot && maxTotMinCriterion == MinCriterion::MinGFRD) {
                        bestBucket = &bucket;
                    }
                    frdgCount = bucket.size();
                } else if (frdgValue == minFRDG) {
                    frdgCount += bucket.size();
                }

                if (rfdgValue < minRFDG) {
                    minRFDG = rfdgValue;
                    rfdgBucket = &bucket;
                    if (criterion == MinCriterion::MaxTot && maxTotMinCriterion == MinCriterion::MinGRFD) {
                        bestBucket = &bucket;
                    }
                    rfdgCount = bucket.size();
                } else if (rfdgValue == minRFDG) {
                    rfdgCount += bucket.size();
                }

                if (rfrdgValue < minRFRDG) {
                    minRFRDG = rfrdgValue;
                    rfrdgBucket = &bucket;
                    if (criterion == MinCriterion::MaxTot && maxTotMinCriterion == MinCriterion::MinGRFRD) {
                        bestBucket = &bucket;
                    }
                    rfrdgCount = bucket.size();
                } else if (rfrdgValue == minRFRDG) {
                    rfrdgCount += bucket.size();
                }

                dLayerIt++;
            }
            fLayerIt++;
        }
        gLayerIt++;
    }

    int optCount = 0;
    if (criterion == MinCriterion::MinTot) {
        optCount = std::min(
                {gCount, fCount, dCount, bCount, rfCount, rdCount, fgCount, dgCount, bgCount, frdCount, rfdCount,
                 rfgCount, rdgCount, rfrdCount, frdgCount, rfdgCount, rfrdgCount});
        m_optCount = optCount;
        if (optCount == 0) return;
        if (bCount == optCount) bestBucket = bBucket;
        else if (gCount == optCount) bestBucket = gBucket;
        else if (fCount == optCount) bestBucket = fBucket;
        else if (dCount == optCount) bestBucket = dBucket;
        else if (rfCount == optCount) bestBucket = rfBucket;
        else if (rdCount == optCount) bestBucket = rdBucket;
        else if (fgCount == optCount) bestBucket = fgBucket;
        else if (dgCount == optCount) bestBucket = dgBucket;
        else if (bgCount == optCount) bestBucket = bgBucket;
        else if (frdCount == optCount) bestBucket = frdBucket;
        else if (rfdCount == optCount) bestBucket = rfdBucket;
        else if (rfgCount == optCount) bestBucket = rfgBucket;
        else if (rdgCount == optCount) bestBucket = rdgBucket;
        else if (rfrdCount == optCount) bestBucket = rfrdBucket;
        else if (frdgCount == optCount) bestBucket = frdgBucket;
        else if (rfdgCount == optCount) bestBucket = rfdgBucket;
        else if (rfrdgCount == optCount) bestBucket = rfrdgBucket;
        return;
    }

    // MinCriterion::MaxTot and all else
    if (bBucket == bestBucket) m_optCount = bCount;
    else if (gBucket == bestBucket) m_optCount = gCount;
    else if (fBucket == bestBucket) m_optCount = fCount;
    else if (dBucket == bestBucket) m_optCount = dCount;
    else if (rfBucket == bestBucket) m_optCount = rfCount;
    else if (rdBucket == bestBucket) m_optCount = rdCount;
    else if (fgBucket == bestBucket) m_optCount = fgCount;
    else if (dgBucket == bestBucket) m_optCount = dgCount;
    else if (bgBucket == bestBucket) m_optCount = bgCount;
    else if (frdBucket == bestBucket) m_optCount = frdCount;
    else if (rfdBucket == bestBucket) m_optCount = rfdCount;
    else if (rfgBucket == bestBucket) m_optCount = rfgCount;
    else if (rdgBucket == bestBucket) m_optCount = rdgCount;
    else if (rfrdBucket == bestBucket) m_optCount = rfrdCount;
    else if (frdgBucket == bestBucket) m_optCount = frdgCount;
    else if (rfdgBucket == bestBucket) m_optCount = rfdgCount;
    else if (rfrdgBucket == bestBucket) m_optCount = rfrdgCount;
}

template<typename state, class environment, class dataStructure, MinCriterion criterion>
std::pair<const state *, double> LBBestBucketBasedList<state, environment, dataStructure, criterion>::Pop() {
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
void LBBestBucketBasedList<state, environment, dataStructure, criterion>::countExpandableNodes() {

    expandableNodes = 0;
    for (const auto &glayer: fLayers) {
        double g = glayer.first;

        if (g > gLim) break;

        for (const auto &fLayer: glayer.second) {
            double h = fLayer.first;
            double f = g + h;
            double rf = g - h;

            if (f > fLim) break;

            if (f + g > fgLim) break;

            if (rf > rfLim) continue; // continue because rf is decreasing

            if (rf + g > rfgLim) continue;


            for (const auto &bucket: fLayer.second) {
                double h_nx = bucket.first;
                double d = g - h_nx;
                double rd = g + h_nx;
                double b = f + d;

                if (d > dLim || (b > bLim)) break;

                if (d + g > dgLim) break;

                if (b + g > bgLim) break;

                if (rd > rdLim) continue; // continue because rd is decreasing

                if (f + rd > frdLim) continue; // continue because rd is decreasing

                if (rf + d > rfdLim) break;

                if (rd + g > rdgLim) continue; // continue because rd is decreasing

                if (rf + rd > rfrdLim) continue; // continue because rd is decreasing

                if (f + rd + g > frdgLim) continue; // continue because rd is decreasing

                if (rf + d + g > rfdgLim) break;

                if (rf + rd + g > rfrdgLim) continue; // continue because rd is decreasing

                expandableNodes += bucket.second.size();
            }
        }
    }
}

#endif //APPS_AIJ_LBBESTBUCKETBASEDLIST_H
