#ifndef BIDIRERRORBUCKETBASEDLIST_H
#define BIDIRERRORBUCKETBASEDLIST_H

#include <cassert>
#include <vector>
#include <map>
#include <set>
#include <unordered_map>
#include <stdint.h>
#include <limits>
#include <climits>
#include <functional>
#include "BucketBasedList.h"
#include "MinCriterion.h"

struct NodeValues {
    std::set<double> g_values;
    std::set<double> f_values;
    std::set<double> d_values;
    std::set<double> b_values;
    std::set<double> rf_values;
    std::set<double> rd_values;

	std::set<double> fg_values;
    std::set<double> dg_values;
    std::set<double> bg_values;
    std::set<double> frd_values;
    std::set<double> rfd_values;
	std::set<double> rfg_values;
    std::set<double> rdg_values;
    std::set<double> rfrd_values;
    std::set<double> frdg_values;
    std::set<double> rfdg_values;
	std::set<double> rfrdg_values;

};

struct BucketInfo {

    BucketInfo() : g(DBL_MAX), h(DBL_MAX), h_nx(DBL_MAX), nodes(INT_MAX) {}

    BucketInfo(double g_, double h_, double h_nx_, int nodes_) : g(g_), h(h_), h_nx(h_nx_), nodes(nodes_) {}

    bool operator==(const BucketInfo &info) const { return g == info.g && h == info.h && h_nx == info.h_nx; }

    double g, h, h_nx;
    int nodes; // the nodes attribute doesn't count for equals nor hash
};

struct BucketHash {

    std::size_t operator()(const BucketInfo &info) const {
        std::size_t hash = 0;
        hash = hash_combine(hash, info.g);
        hash = hash_combine(hash, info.h);
        hash = hash_combine(hash, info.h_nx);
        return hash;
    }

    inline std::size_t hash_combine(std::size_t &seed, const double value) const {
        return std::hash < double > {}(value) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    }
};

template<typename state, class environment, class dataStructure = BucketNodeData<state> >
class BidirErrorBucketBasedList {

public:

    using Bucket = std::vector<const state *>;

    BidirErrorBucketBasedList() : table(10, stateHasher) {}

    ~BidirErrorBucketBasedList() {}

    inline void Reset() {
        table.clear();
        fLayers.clear();
    }

    virtual bool AddOpenNode(const state val, double g, double h, double h_nx, const state *parent = nullptr);

    bool IsEmpty() { return fLayers.size() == 0; }

    bool RemoveIfEmpty(double g, double h, double h_nx);

    const state *PopBucket(double g, double h, double h_nx);

    inline const dataStructure &Lookup(const state &objKey) const { return table.at(objKey); }

    /**
      * get the g value of a node or DBL_MAX if it doesn't exist
      **/
    const std::pair<bool, std::pair<bool, double>> getNodeG(const state &objKey, const double h) {
        // TODO: this should get a function and not h, as h is seldom needed and computing it may be expensive
        auto nodeIt = table.find(objKey);
        if (nodeIt != table.end()) {
            double g = nodeIt->second.g;

            // optimal g if expanded or their g or their f is minimal
            // TODO: implement check for min d or even pareto optimality
            bool optimalG = nodeIt->second.bucket_index == -1;
            // TODO check if optimal g can be proved for nodes in the open list
            // || (isBestBucketComputed() && (g <= getMinG() || (g + h <= getMinF())));
            return std::make_pair(true, std::make_pair(optimalG, nodeIt->second.g));
        } else {
            return std::make_pair(false, std::make_pair(false, std::numeric_limits<double>::max()));
        }
    }

    inline void setEnvironment(environment *env_) { env = env_; }

    std::vector <BucketInfo> getBucketInfo();

    NodeValues getNodeValues();

protected:

    environment *env;

    std::function<size_t(const state &)> stateHasher = [this](const state &x) { return env->GetStateHash(x); };

    std::unordered_map<const state, dataStructure, decltype(stateHasher)> table;

    // fist key is g, second is h, third is h_nx (h_nx is sorted in reverse to traverse by ascending d)
    std::map<double, std::map<double, std::map < double, Bucket, std::greater < double>> >>
    fLayers;

};

template<typename state, class environment, class dataStructure>
bool BidirErrorBucketBasedList<state, environment, dataStructure>::AddOpenNode(const state val,
                                                                               const double g,
                                                                               const double h,
                                                                               const double h_nx,
                                                                               const state *parent) {
    const double f = g + h;
    const double d = g - h_nx;

    auto nodeIt = table.find(val);
    if (nodeIt != table.end()) { // node already exists
        double old_g = nodeIt->second.g;
        if (old_g <= g) {
            return false;    // existing node has no worse g value, don't store
        } else {
            auto bucketIndex = nodeIt->second.bucket_index;
            if (bucketIndex == -1) {
                std::cerr << "  -- Node reopened!!!" << std::endl;
                exit(0);
            }

            // invalidate pointer with higher g value in the open list
            fLayers[old_g][h][h_nx][bucketIndex] = nullptr;

            auto &bucket = fLayers[g][h][h_nx];
            nodeIt->second = dataStructure(g, parent, bucket.size()); // node exists but with worse g value, update
            bucket.push_back(&(nodeIt->first));
        }
    } else {  // node doesn't exist
        auto &bucket = fLayers[g][h][h_nx];
        auto it_pair = table.insert(std::make_pair(val, dataStructure(g, parent, bucket.size())));
        bucket.push_back(&(it_pair.first->first));
    }

    return true;

}

template<typename state, class environment, class dataStructure>
bool BidirErrorBucketBasedList<state, environment, dataStructure>::RemoveIfEmpty(double g,
                                                                                 double h,
                                                                                 double h_nx) {
    Bucket &bucket = fLayers[g][h][h_nx];

    // remove erased entries to make sure that the bucket does not contain only invalid entries
    while (bucket.size() > 0 && bucket.back() == nullptr) {
        bucket.pop_back();
    }

    bool bucketEmptied = false;

    // delete empty dimensions
    if (bucket.size() == 0) {
        bucketEmptied = true;
        auto &fLayer = fLayers[g][h];
        fLayer.erase(h_nx);
        if (fLayer.size() == 0) {
            auto &gLayer = fLayers[g];
            gLayer.erase(h);
            if (gLayer.size() == 0) {
                fLayers.erase(g);
            }
        }
    }

    return bucketEmptied;
}

template<typename state, class environment, class dataStructure>
const state *BidirErrorBucketBasedList<state, environment, dataStructure>::PopBucket(double g,
                                                                                     double h,
                                                                                     double h_nx) {
    // pop state - it has to be a proper bucket, so call RemoveIfInvalid if needed
    Bucket &bucket = fLayers[g][h][h_nx];
    const state *poppedState = bucket.back();
    bucket.pop_back();
    RemoveIfEmpty(g, h, h_nx); // remove if it is empty

    // invalidate node 2 bucket index
    auto &node = table.at(*poppedState);
    node.bucket_index = -1;

    return poppedState;
}

template<typename state, class environment, class dataStructure>
std::vector <BucketInfo> BidirErrorBucketBasedList<state, environment, dataStructure>::getBucketInfo() {

    std::vector <BucketInfo> result;

    for (const auto &glayer : fLayers) {
        double g = glayer.first;
        for (const auto &fLayer : glayer.second) {
            double h = fLayer.first;
            for (const auto &bucket : fLayer.second) {
                double h_nx = bucket.first;
                result.push_back(BucketInfo(g, h, h_nx, bucket.second.size()));

                // TODO check if this optimization is useful enough to be enabled
                // if (!useRC) break; // subsequent buckets will be dominated if RC is not used
            }
        }
    }

    return result;

}

template<typename state, class environment, class dataStructure>
NodeValues BidirErrorBucketBasedList<state, environment, dataStructure>::getNodeValues() {

    NodeValues result;

    for (const auto &glayer : fLayers) {
        double g = glayer.first;
        result.g_values.insert(g);

        for (const auto &fLayer : glayer.second) {
            double h = fLayer.first;
            double f = g + h;
            double rf = g - h;
            result.f_values.insert(f);
			result.fg_values.insert(f + g);
            result.rf_values.insert(rf);

            for (const auto &bucket : fLayer.second) {
                double h_nx = bucket.first;
                double d = g - h_nx;
                double rd = g + h_nx;
                double b = f + d;
                result.d_values.insert(d);
				result.dg_values.insert(d + g);
                result.rd_values.insert(rd);
				result.frd_values.insert(f + rd);
				result.rfd_values.insert(rf + d);
				result.rfg_values.insert(rf + g);
				result.rdg_values.insert(rd + g);
				result.rfrd_values.insert(rf + rd);
				result.frdg_values.insert(f + rd + g);
				result.rfrdg_values.insert(rf + rd + g);
				result.rfdg_values.insert(rf + d + g);
                result.b_values.insert(b);
				result.bg_values.insert(b + g);
            }
        }
    }

    return result;
}

#endif
