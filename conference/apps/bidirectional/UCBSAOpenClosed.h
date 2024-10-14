//
// Created by Lior Siag on 12/11/2022.
//

#ifndef UCBSAOPENCLOSED_H
#define UCBSAOPENCLOSED_H

#include <cassert>
#include <vector>
#include "FPUtil.h"
#include "AStarOpenClosed.h"
#include <ext/hash_map>
#include <stdint.h>
#include <iostream>
#include "BAELB.h"

/**
 * DataLocation enum specifically for ucbs. In this case, both waiting and ready are considered open,
 * but in different stages of the queue.
 */
enum ucbsDataLocation {
    ucbsWaiting,
    ucbsReady,
    ucbsClosed,
    ucbsNotFound
};

enum queueType {
    qTotal = 0,
    qWaiting = 1,
    qReady = 2
};

/**
 * A data object for a domain state for UCBS.
 */
template<typename state>
class UCBSOpenClosedData {
public:
    UCBSOpenClosedData(const state &theData, double gCost, double hCost, double dCost, uint64_t parent,
                       std::vector<uint64_t> openLoc, unsigned int wPos, ucbsDataLocation location)
            : data(theData), g(gCost), h(hCost), d(dCost), parentID(parent), openLocations(openLoc),
              waitingPos(wPos), where(location) { reopened = false; }

    state data;
    double g;
    double h;
    double d;
    uint64_t parentID;
    std::vector<uint64_t> openLocations;
    unsigned int waitingPos;
    bool reopened;
    ucbsDataLocation where;
};

/**
 * Comparator for node values. In order to have a heap which prefers the opposite of what currently is written,
 * we can simply negate the result. This is useful for the ready queue.
 */
template<typename state, class dataStructure = UCBSOpenClosedData<state>>
class UCBSComparator {
public:
    UCBSComparator() {}

    bool
    operator()(nodeValue valueIndex, const dataStructure &i1, const dataStructure &i2) const {
        switch (valueIndex) {
            case ivG:
                return GCompare(i1, i2);
            case ivF:
                return FCompare(i1, i2);
            case ivD:
                return DCompare(i1, i2);
            case ivB:
                return BCompare(i1, i2);
            case ivGF:
                return GFCompare(i1, i2);
            case ivGD:
                return GDCompare(i1, i2);
            case ivGB:
                return GBCompare(i1, i2);
            default:
                assert(false);
                return false;
        }
    }

private:
    bool GCompare(const dataStructure &i1, const dataStructure &i2) const {
        double p1 = i1.g;
        double p2 = i2.g;
        if (fequal(p1, p2)) {
            return (fgreater(i1.h, i2.h)); // Low h over high h
        }
        return fgreater(p1, p2); // Low g over high g
    }

    bool FCompare(const dataStructure &i1, const dataStructure &i2) const {
        double p1 = i1.g + i1.h;
        double p2 = i2.g + i2.h;
        if (fequal(p1, p2)) {
            return (fless(i1.g, i2.g)); // High g-cost over low
        }
        return fgreater(p1, p2); // Low f over high f
    }

    bool DCompare(const dataStructure &i1, const dataStructure &i2) const {
        double p1 = i1.d;
        double p2 = i2.d;
        if (fequal(p1, p2)) {
            return (fless(i1.g, i2.g)); // High g-cost over low
        }
        return fgreater(p1, p2); // Low d over high d
    }

    bool BCompare(const dataStructure &i1, const dataStructure &i2) const {
        double p1 = i1.g + i1.h + i1.d;
        double p2 = i2.g + i2.h + i2.d;
        if (fequal(p1, p2)) {
            return (fless(i1.g, i2.g)); // High g-cost over low
        }
        return (fgreater(p1, p2)); // low b over b
    }

    bool GFCompare(const dataStructure &i1, const dataStructure &i2) const {
        double p1 = 2 * i1.g + i1.h;
        double p2 = 2 * i2.g + i2.h;
        if (fequal(p1, p2)) {
            return (fless(i1.g, i2.g)); // High g-cost over low
        }
        return fgreater(p1, p2); // Low g+f over high
    }

    bool GDCompare(const dataStructure &i1, const dataStructure &i2) const {
        double p1 = i1.g + i1.d;
        double p2 = i2.g + i2.d;
        if (fequal(p1, p2)) {
            return (fless(i1.g, i2.g)); // High g-cost over low
        }
        return fgreater(p1, p2); // Low g+d over high
    }

    bool GBCompare(const dataStructure &i1, const dataStructure &i2) const {
        double p1 = (2 * i1.g) + i1.h + i1.d;
        double p2 = (2 * i2.g) + i2.h + i2.d;
        if (fequal(p1, p2)) {
            return (fless(i1.g, i2.g)); // High g-cost over low
        }
        return fgreater(p1, p2); // // Low g+b over high
    }
};

/**
 * UCBS-All openClosed list.
 * This version takes the minimum values of the queue from the total table, i.e., from all the nodes, waiting and ready.
 */
template<typename state, class dataStructure = UCBSOpenClosedData<state>>
class UCBSAOpenClosed {
public:
    UCBSAOpenClosed(double epsilon_ = 1, double gcd_ = 1);

    ~UCBSAOpenClosed() {};

    void Reset();

    uint64_t
    AddOpenNode(const state &val, uint64_t hash, double g, double h, double d, uint64_t parent = kTAStarNoNode);

    uint64_t AddClosedNode(state &val, uint64_t hash, double g, double h, double d, uint64_t parent = kTAStarNoNode);

    void PassNodeThrough(uint64_t objKey, bool newNode = false);

    void KeyChanged(uint64_t objKey);

    ucbsDataLocation Lookup(uint64_t hashKey, uint64_t &objKey) const;

    inline dataStructure &Lookup(uint64_t objKey) { return elements[objKey]; }

    inline const dataStructure &Lookat(uint64_t objKey) const { return elements[objKey]; }

    uint64_t Peek(queueType qt, unsigned int heapNum) const {
        return theHeaps[qt][heapNum][0];
    }

    uint64_t GetOpenItem(queueType qt, unsigned int heapNum, unsigned int which) {
        return theHeaps[qt][heapNum][which];
    }

    uint64_t Close();

    uint64_t Close(uint64_t objKey);

    void Reopen(uint64_t objKey);

    std::vector<double> GetMinima();

    size_t OpenSize() const { return theHeaps[qTotal][0].size(); }

    size_t ReadySize() const { return theHeaps[qReady][0].size(); }

    size_t ClosedSize() const { return size() - OpenSize(); }

    size_t size() const { return elements.size(); }

    void IncreaseLB();

    bool updateXMins(const std::vector<double> &newXMins);

    std::vector<dataStructure> elements;

private:
    bool HeapifyUp(queueType queueId, unsigned int heapNum, unsigned int index);

    void HeapifyDown(queueType queueId, unsigned int heapNum, unsigned int index);

    void RemoveNodeFromQueue(queueType qt, uint64_t objkey);

    bool CalcBound(nodeValue bound, uint64_t objkey);

    double epsilon;
    double gcd;
    double lowerBound;

    std::vector<std::vector<std::vector<uint64_t>>> theHeaps;

    typedef __gnu_cxx::hash_map<uint64_t, uint64_t, AHash64> IndexTable;
    IndexTable table;

    UCBSComparator<state, dataStructure> comp;

    std::vector<double> xMins;
};

/**
 * Returns location of object as well as object key if found.
 */
template<typename state, class dataStructure>
ucbsDataLocation UCBSAOpenClosed<state, dataStructure>::Lookup(uint64_t hashKey, uint64_t &objKey) const {
    typename IndexTable::const_iterator it;
    it = table.find(hashKey);
    if (it != table.end()) {
        objKey = (*it).second;
        return elements[objKey].where;
    }
    return ucbsNotFound;
}

/**
 * The constructor needs epsilon for bound calculations, and creates the heaps as well.
 * 3 queue types, each contains 7 heaps (7 min values for total, 7 bounds for waiting, and 7 max values for ready).
 */
template<typename state, class dataStructure>
UCBSAOpenClosed<state, dataStructure>::UCBSAOpenClosed(double epsilon_, double gcd_) {
    epsilon = epsilon_;
    gcd = gcd_;

    for (int i = 0; i < 3; i++) {
        theHeaps.push_back(std::vector<std::vector<uint64_t>>());
        for (int j = 0; j < 7; j++) {
            theHeaps[i].push_back(std::vector<uint64_t>());
        }
    }
}

/**
 * Clears the different parts in the data structure.
 */
template<typename state, class dataStructure>
void UCBSAOpenClosed<state, dataStructure>::Reset() {
    table.clear();
    elements.clear();
    for (std::vector<std::vector<uint64_t>> &curHeaps: theHeaps) {
        for (std::vector<uint64_t> &curHeap: curHeaps)
            curHeap.resize(0);
    }
}

/**
 * Add a new node to the open list.
 */
template<typename state, class dataStructure>
uint64_t UCBSAOpenClosed<state, dataStructure>::AddOpenNode(const state &val, uint64_t hash,
                                                            double g, double h, double d, uint64_t parent) {
    // You must check if the node is not already in the queue before adding it
    assert(table.find(hash) == table.end());

    // Generate the new dataNode by giving it its state, the node values, the parent, 3 vectors each the size of the
    // appropriate layer, and initialized with 0 except total which we will heapify. The others are initialized with
    // 0 since they only matter if the node is in that particular spot. And current funnel, and where it is.
    elements.push_back(dataStructure(val, g, h, d, parent,
                                     std::vector<std::vector<uint64_t>>{
                                             std::vector<uint64_t>(theHeaps[qTotal].size(), theHeaps[qTotal][0].size()),
                                             std::vector<uint64_t>(theHeaps[qWaiting].size(), 0),
                                             std::vector<uint64_t>(theHeaps[qReady].size(), 0)
                                     }, 0, ucbsWaiting));

    if (parent == kTAStarNoNode)
        elements.back().parentID = elements.size() - 1;

    table[hash] = elements.size() - 1; // point state hash to element

    for (int i = 0; i < theHeaps[qTotal].size(); i++) {
        HeapifyUp(qTotal, i, elements[val].openLocations[qTotal][i]);
    }

    PassNodeThrough(val, true);

    return elements.size() - 1; //return the index of the new item in the element list
}

/**
 * Add a closed node to the list.
 */
template<typename state, class dataStructure>
uint64_t UCBSAOpenClosed<state, dataStructure>::AddClosedNode(state &val, uint64_t hash, double g, double h, double d,
                                                              uint64_t parent) {
    assert(table.find(hash) == table.end());
    elements.push_back(dataStructure(val, g, h, d, parent,
                                     std::vector<std::vector<uint64_t>>{
                                             std::vector<uint64_t>(theHeaps[qTotal].size(), theHeaps[qTotal][0].size()),
                                             std::vector<uint64_t>(theHeaps[qWaiting].size(), 0),
                                             std::vector<uint64_t>(theHeaps[qReady].size(), 0)
                                     }, 0, ucbsClosed));
    if (parent == kTAStarNoNode)
        elements.back().parentID = elements.size() - 1;
    table[hash] = elements.size() - 1; // hashing to element list location
    return elements.size() - 1;
}

/**
 * Closes the node from ready with min b.
 */
template<typename state, class dataStructure>
uint64_t UCBSAOpenClosed<state, dataStructure>::Close() {
    assert(ReadySize() != 0);
    uint64_t ans = theHeaps[qReady][ivB][0];
    assert(elements[ans].where == qReady && elements[ans].openLocations[qReady][ivB] == 0);
    return Close(ans);
}

/**
 * Move the given item to the closed list and return key.
 */
template<typename state, class dataStructure>
uint64_t UCBSAOpenClosed<state, dataStructure>::Close(uint64_t objKey) {
    assert(ReadySize() != 0);
    // Currently allow only ready nodes to be closed
    assert(elements[objKey].where == ucbsReady);
    RemoveNodeFromQueue(qTotal, objKey);
    RemoveNodeFromQueue(qReady, objKey);

    return objKey;
}

/**
 * One (or more) of the node values changed. In this case, it can only be g, as we are working with static heuristics.
 * If it is waiting, we need to pass it through the funnels again. If it is in ready, we can only lower its bounds,
 * therefore, it will pass through the funnels.
 */
template<typename state, class dataStructure>
void UCBSAOpenClosed<state, dataStructure>::KeyChanged(uint64_t objKey) {
    if (elements[objKey].wehre == ucbsClosed)
        return;

    // Fix its position in total
    for (int i = 0; i < theHeaps[qTotal].size(); i++) {
        if (!HeapifyUp(qTotal, i, elements[objKey].openLocations[qTotal][i])) {
            HeapifyDown(qTotal, i, elements[objKey].openLocations[qTotal][i]);
        }
    }

    PassNodeThrough(objKey, false);
}

/**
 * Gets the current minima from the total open list
 */
template<typename state, class dataStructure>
std::vector<double> UCBSAOpenClosed<state, dataStructure>::GetMinima() {
    auto gMinNode = elements[theHeaps[qTotal][ivG][0]];
    auto fMinNode = elements[theHeaps[qTotal][ivF][0]];
    auto dMinNode = elements[theHeaps[qTotal][ivD][0]];
    auto bMinNode = elements[theHeaps[qTotal][ivB][0]];
    auto gfMinNode = elements[theHeaps[qTotal][ivGF][0]];
    auto gdMinNode = elements[theHeaps[qTotal][ivGD][0]];
    auto gbMinNode = elements[theHeaps[qTotal][ivGB][0]];

    return {gMinNode.g,
            fMinNode.g + fMinNode.h,
            dMinNode.d,
            bMinNode.g + bMinNode.h + bMinNode.d,
            2 * gfMinNode.g + gfMinNode.h,
            gdMinNode.g + gdMinNode.d,
            2 * gbMinNode.g + gbMinNode.h + gbMinNode.d};
}

/**
 * Updates its own xMins based on what it received from the other side, and clears nodes from the ready that will now
 * not pass the corresponding bound. E.g., in the case where dMin increases, we test the fMax heap and see if it still
 * passes the bound.
 */
template<typename state, class dataStructure>
bool UCBSAOpenClosed<state, dataStructure>::updateXMins(const std::vector<double> &newXMins) {
    if (xMins == newXMins)
        return false;
    xMins = newXMins;
    for (int i = 0; i < newXMins.size(); i++) {
        // Once the ready queue empties, or the bound is no longer bigger than lower bound, we can stop transferring
        // nodes back up
        while (!theHeaps[qReady][i].empty() && CalcBound(i, theHeaps[qReady][i][0]) <= lowerBound) {
            PassNodeThrough(theHeaps[qReady][i][0], false);
        }
    }
    return true;
}

/**
 * When we increase LB, we recheck the waiting queue to see if there are new nodes to insert into ready.
 * Since no xMin had changed, and LB had only increased, the nodes in ready do not need to be rechecked.
 */
template<typename state, class dataStructure>
void UCBSAOpenClosed<state, dataStructure>::IncreaseLB() {
    lowerBound += gcd;
    for (int i = 0; i < theHeaps[qWaiting].size(); i++) {
        while (!theHeaps[qWaiting][i].empty() && CalcBound(i, theHeaps[qWaiting][i][0]) <= lowerBound) {
            PassNodeThrough(theHeaps[qWaiting][i][0], false);
        }
    }

}

/**
 * Move the node through the funnels and to the open queue.
 * If a node gets stuck by some bound, we insert it into a funnel, update the location data of said node, and heapify
 * the relevant queue.
 * If it passes to ready, we simply insert it to ready.
 * if newNode is false, it means we are moving a node that was already in place, and we need to remove it beforehand
 */
template<typename state, class dataStructure>
void UCBSAOpenClosed<state, dataStructure>::PassNodeThrough(uint64_t objKey, bool newNode) {
    if (!newNode) {
        RemoveNodeFromQueue(elements[objKey].where, objKey);
    }

    // Go over every bound and see if it stops the current node, insert it into the bound's queue
    for (int i = 0; i < theHeaps[qWaiting].size(); i++) {
        if (CalcBound(i, objKey) < lowerBound) {
            elements[objKey].waitingPos = i;
            elements[objKey].openLocations[qWaiting][i] = theHeaps[qWaiting][i].size();
            theHeaps[qWaiting][i].push_back(objKey);
            HeapifyUp(qWaiting, i, elements[objKey].openLocations[qWaiting][i]);
            return;
        }
    }

    // If it passed all bounds, it enters ready
    elements[objKey].where = qReady;
    for (int i = 0; i < theHeaps[qReady].size(); i++) {
        if (!CalcBound(i, objKey)) {
            elements[objKey].openLocations[qReady][i] = theHeaps[qReady][i].size();
            theHeaps[qReady][i].push_back(objKey);
            HeapifyUp(qReady, i, elements[objKey].openLocations[qReady][i]);
        }
    }
}

/**
 * Move item off the closed list and back to the total heaps and through the funnels
 */
template<typename state, class dataStructure>
void UCBSAOpenClosed<state, dataStructure>::Reopen(uint64_t objKey) {
    assert(elements[objKey].where == ucbsClosed);
    elements[objKey].reopened = true;
    elements[objKey].where = ucbsWaiting;
    for (int i = 0; i < theHeaps.size(); i++) {
        elements[objKey].openLocations[qTotal][i] = theHeaps[qTotal][i].size();
        theHeaps[qTotal][i].push_back(objKey);
        HeapifyUp(qTotal, i, theHeaps[qTotal][i].size() - 1);
    }
    PassNodeThrough(objKey, true);
}

/**
 * Moves a node up the heap. Returns true if the node was moved, false otherwise.
 */
template<typename state, class dataStructure>
bool UCBSAOpenClosed<state, dataStructure>::HeapifyUp(queueType queueId, unsigned int heapNum, unsigned int index) {
    auto &currHeap = theHeaps[queueId][heapNum];
    if (index == 0) return false;
    unsigned int parent = (index - 1) / 2;
    if ((queueId == qReady) !=
        comp(heapNum, elements[currHeap[parent]], elements[currHeap[index]])) {
        unsigned int tmp = currHeap[parent];
        currHeap[parent] = currHeap[index];
        currHeap[index] = tmp;
        elements[currHeap[parent]].openLocations[queueId][heapNum] = parent;
        elements[currHeap[index]].openLocations[queueId][heapNum] = index;
        HeapifyUp(heapNum, parent);
        return true;
    }
    return false;
}

/**
 * Moves a node down the heap.
 */
template<typename state, class dataStructure>
void UCBSAOpenClosed<state, dataStructure>::HeapifyDown(queueType queueId, unsigned int heapNum, unsigned int index) {
    unsigned int child1 = index * 2 + 1;
    unsigned int child2 = index * 2 + 2;
    auto &currHeap = theHeaps[queueId][heapNum];
    unsigned int which;
    unsigned int count = currHeap.size();
    if (child1 >= count)
        return;
    else if (child2 >= count)
        which = child1;
    else if (!((queueId == qReady) && comp(heapNum, elements[currHeap[child1]], elements[currHeap[child2]])))
        which = child1;
    else
        which = child2;

    if (!((queueId == qReady) && comp(heapNum, elements[currHeap[child1]], elements[currHeap[child2]]))) {
        unsigned int tmp = currHeap[which];
        currHeap[which] = currHeap[index];
        currHeap[index] = tmp;
        elements[currHeap[which]].openLocations[heapNum] = which;
        elements[currHeap[index]].openLocations[heapNum] = index;
        HeapifyDown(heapNum, which);
    }
}

/**
 * This is a helper function to remove a node from all connected queues
 * or single queue in the case of qWaiting.
 */
template<typename state, class dataStructure>
void UCBSAOpenClosed<state, dataStructure>::RemoveNodeFromQueue(queueType qt, uint64_t objKey) {
    if (qt == qWaiting) {
        unsigned int currPos = elements[objKey].waitingPos;
        uint64_t index = elements[objKey].openLocations[qt][currPos];
        theHeaps[qt][currPos][index] = theHeaps[qt][currPos][theHeaps[qt][currPos].size() - 1];
        elements[theHeaps[qt][currPos][index]].openLocations[qt][currPos] = index;
        theHeaps[qt][currPos].pop_back();
        if (!HeapifyUp(qt, currPos, index)) {
            HeapifyDown(qt, currPos, index);
        }
    } else {
        std::vector<uint64_t> indexes = elements[objKey].openLocations[qt];
        for (int i = 0; i < theHeaps[qt].size(); i++) {
            theHeaps[qt][i][indexes[i]] = theHeaps[qt][i][theHeaps[qt][i].size() - 1];
            elements[theHeaps[qt][i][indexes[i]]].openLocations[qt][i] = indexes[i];
            theHeaps[qt][i].pop_back();
            if (!HeapifyUp(qt, i, indexes[i])) {
                HeapifyDown(qt, i, indexes[i]);
            }
        }
    }
}

/**
 * Calculate the bound value given a node and which bound to calculate (as written in the nodeValue enum).
 */
template<typename state, class dataStructure>
bool UCBSAOpenClosed<state, dataStructure>::CalcBound(nodeValue bound, uint64_t objKey) {
    dataStructure &node = elements[objKey];
    switch (bound) {
        case ivG:
            return node.g + xMins[ivG] + epsilon;
        case ivF:
            return node.g + node.h + xMins[ivD];
        case ivD:
            return node.d + xMins[ivF];
        case ivB:
            return (node.g + node.h + node.d + xMins[ivB]) / 2;
        case ivGF:
            return (2 * node.g + node.h + xMins[ivGD] + epsilon) / 2;
        case ivGD:
            return (node.g + node.d + xMins[ivGF] + epsilon) / 2;
        case ivGB:
            return (2 * node.g + node.h + node.d + xMins[ivGB] + epsilon) / 3;
        default:
            assert(false);
            return -1;
    }
}
#endif
