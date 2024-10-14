#ifndef MINGBUCKETOPENCLOSED_H
#define MINGBUCKETOPENCLOSED_H

#include <limits>
#include "BucketBasedList.h"

template<typename state, class environment, class dataStructure = BucketNodeData<state> >
class MinGBucketOpenClosed : public BucketBasedList<state, environment, dataStructure> {

    using Base = BucketBasedList<state, environment, dataStructure>;

public:
    MinGBucketOpenClosed() : Base() {}

    ~MinGBucketOpenClosed() {}

    virtual void AddOpenNode(const state val, double g, double h, const state *parent = nullptr);

    virtual std::pair<const state *, double> Pop(double f);

    double getMinG(double f);


private:

    double minG = std::numeric_limits<double>::max();
    double minGFLayer = std::numeric_limits<double>::max();

    double currentF = 0.0;

};

template<typename state, class environment, class dataStructure>
void MinGBucketOpenClosed<state, environment, dataStructure>::AddOpenNode(const state val, double g, double h,
                                                                          const state *parent) {
    Base::AddOpenNode(val, g, h, parent);

    // update minG and minGFLayer if necessary
    double f = g + h;
    if (g < minG && f <= currentF) {
        minG = g;
        minGFLayer = f;
    }
}

template<typename state, class environment, class dataStructure>
std::pair<const state *, double> MinGBucketOpenClosed<state, environment, dataStructure>::Pop(double f) {

    if (f > currentF) { // if minF has changed, update min g
        currentF = f;
        getMinG(currentF);
    }

    auto &currentFLayer = Base::fLayers[minGFLayer];
    auto &bucket = currentFLayer.begin()->second;
    auto poppedState = bucket.back();
    bucket.pop_back();

    auto &node = Base::table.at(*poppedState);
    node.bucket_index = -1;
    return std::make_pair(poppedState, node.g);
}

/**
  * get the min g value among open nodes such that f(n) <= f
  * in order to get a proper estimate, this method has to find a non-expanded (duplicate) node
  **/
template<typename state, class environment, class dataStructure>
double MinGBucketOpenClosed<state, environment, dataStructure>::getMinG(double f) {

    minG = std::numeric_limits<double>::max();
    minGFLayer = std::numeric_limits<double>::max();

    for (auto it = Base::fLayers.begin(); it != Base::fLayers.end() && it->first <= f;) {
        auto &currentFLayer = it->second;

        bool nodeFound = false;
        while (!nodeFound && !currentFLayer.empty()) {
            auto &bucket = currentFLayer.begin()->second;
            while (!nodeFound && !bucket.empty()) {
                if (bucket.back() != nullptr) { // found a valid node
                    if (minG > currentFLayer.begin()->first) {
                        minG = currentFLayer.begin()->first;
                        minGFLayer = it->first;
                    }
                    nodeFound = true;
                } else {
                    bucket.pop_back(); // discard the expanded node
                }
            }

            if (!nodeFound) {
                currentFLayer.erase(currentFLayer.begin()); // bucket empty - get rid of it
            }
        }

        if (currentFLayer.empty())
            it = Base::fLayers.erase(it); // f layer empty - get rid of it
        else
            it++;
    }

    return minG;
}

#endif