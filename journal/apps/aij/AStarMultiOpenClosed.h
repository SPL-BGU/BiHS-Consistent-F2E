#ifndef ASTARMULTIOPENCLOSED_h
#define ASTARMULTIOPENCLOSED_h

#include <cassert>
#include <vector>
#include <ext/hash_map>
#include <cstdint>
#include "AStarOpenClosed.h"
#include <iostream>

template<typename state>
class AStarMultiOpenClosedData {
public:
    AStarMultiOpenClosedData(const state &theData, double gCost, double hCost, double rhCost, uint64_t parent,
                             std::vector<uint64_t> openLocs,
                             dataLocation location)
            : data(theData), g(gCost), h(hCost), rh(rhCost), parentID(parent), openLocations(openLocs),
              where(location) { reopened = false; }

    state data;
    double g;
    double h;
    double rh;
    uint64_t parentID;
    std::vector<uint64_t> openLocations;
    bool reopened;
    dataLocation where;
};

template<typename state>
struct NodeCompare {
    virtual bool
    operator()(const AStarMultiOpenClosedData<state> &i1, const AStarMultiOpenClosedData<state> &i2) const {
        std::cerr << "[ERROR] Used base NodeCompare operator()" << std::endl;
        assert(false);
    };
};

template<typename state, class dataStructure = AStarMultiOpenClosedData<state>>
class AStarMultiOpenClosed {
public:
    AStarMultiOpenClosed();

    AStarMultiOpenClosed(std::vector<NodeCompare<state> *> &theCmpKeys, int mainHeapNum = 0);

    ~AStarMultiOpenClosed();

    void Reset();

    uint64_t
    AddOpenNode(const state &val, uint64_t hash, double g, double h, double rh, uint64_t parent = kTAStarNoNode);

    uint64_t AddClosedNode(state &val, uint64_t hash, double g, double h, double rh, uint64_t parent = kTAStarNoNode);

    void KeyChanged(uint64_t objKey);


    dataLocation Lookup(uint64_t hashKey, uint64_t &objKey) const;

    inline dataStructure &Lookup(uint64_t objKey) { return elements[objKey]; }

    inline const dataStructure &Lookat(uint64_t objKey) const { return elements[objKey]; }

    void Remove(uint64_t hash);

    uint64_t Peek() const { return Peek(mainHeapNum); }

    uint64_t Peek(unsigned int heapNum) const;

    uint64_t GetOpenItem(unsigned int which) { return theHeaps[mainHeapNum][which]; }

    uint64_t GetOpenItem(unsigned int which, unsigned int heapNum) { return theHeaps[heapNum][which]; }

    uint64_t Close(uint64_t objKey);

    uint64_t Close();

    void Reopen(uint64_t objKey);

    size_t OpenSize() const { return theHeaps[mainHeapNum].size(); }

    size_t ClosedSize() const { return size() - OpenSize(); }

    size_t size() const { return elements.size(); }

    std::vector<dataStructure> elements; // All the elements, this is the vector of elements and not the actual heaps

private:
    std::vector<bool> HeapifyUpAll(uint64_t val);

    bool HeapifyUp(unsigned int heapNum, unsigned int index);

    void HeapifyDown(unsigned int heapNum, unsigned int index);

    std::vector<NodeCompare<state> *> cmpKeys; // The comparators vector, all of which inherit from NodeCompare

    std::vector<std::vector<uint64_t>> theHeaps; // A vector containing all the different heaps for each comparator
    typedef __gnu_cxx::hash_map<uint64_t, uint64_t, AHash64> IndexTable;
    IndexTable table;

    int mainHeapNum;
};

/**
 * Notice that while this data structure supports an empty constructor, it is done simply for initialization purposes
 * in cases where AStarMultiOpenClosed constructor is called when it is a property of a class.
 * You should re-initialize it over the default initialization or it would not work.
 */
template<typename state, class dataStructure>
AStarMultiOpenClosed<state, dataStructure>::AStarMultiOpenClosed() {}

template<typename state, class dataStructure>
AStarMultiOpenClosed<state, dataStructure>::AStarMultiOpenClosed(std::vector<NodeCompare<state> *> &theCmpKeys,
                                                                 int mainHeapNum) {
    cmpKeys = theCmpKeys;
    for (int i = 0; i < cmpKeys.size(); i++) {
        theHeaps.push_back(std::vector<uint64_t>());
    }
    this->mainHeapNum = mainHeapNum;

}

template<typename state, class dataStructure>
AStarMultiOpenClosed<state, dataStructure>::~AStarMultiOpenClosed() {}

/**
 * Remove all objects from queue.
 */
template<typename state, class dataStructure>
void AStarMultiOpenClosed<state, dataStructure>::Reset() {
    table.clear();
    elements.clear();
    for (std::vector<uint64_t> &curHeap: theHeaps)
        curHeap.resize(0);
}

/**
 * Add object into open list.
 */
template<typename state, class dataStructure>
uint64_t
AStarMultiOpenClosed<state, dataStructure>::AddOpenNode(const state &val, uint64_t hash, double g, double h,
                                                        double rh, uint64_t parent) {
    // You must check if the node is not already in the queue before adding it
    if (table.find(hash) != table.end()) {
        // return -1;
        assert(false);
    };
    // We add a new item. The item starts as the last item of all heaps, and since all heaps are of the same size
    //  we can simply put a vector the size of the number of heaps, with each element being the size of the heap which is
    //  the place of the new element (before heapifying that is).
    elements.push_back(
            dataStructure(val, g, h, rh, parent, std::vector<uint64_t>(theHeaps.size(), theHeaps[0].size()), kOpenList));
    if (parent == kTAStarNoNode)
        elements.back().parentID = elements.size() - 1;
    // Push the index of the element to the back of each heap
    for (std::vector<uint64_t> &theHeap: theHeaps) {
        theHeap.push_back(elements.size() - 1);
    }
    table[hash] = elements.size() - 1; // hashing to element list location
    HeapifyUpAll(elements.size() - 1);
    return elements.size() - 1; //return the index of the new item in the element list
}


/**
 * Add object into closed list.
 */
template<typename state, class dataStructure>
uint64_t
AStarMultiOpenClosed<state, dataStructure>::AddClosedNode(state &val, uint64_t hash, double g, double h, double rh,
                                                          uint64_t parent) {
    assert(table.find(hash) == table.end());
    elements.push_back(dataStructure(val, g, h, rh, parent, std::vector<uint64_t>(theHeaps.size(), 0), kClosedList));
    if (parent == kTAStarNoNode)
        elements.back().parentID = elements.size() - 1;
    table[hash] = elements.size() - 1; // hashing to element list location
    return elements.size() - 1;
}


/**
 * Indicate that the key for a particular object has changed.
 * Note that an element in a heap cannot need both heapify up and heapify down simultaneously.
 */
template<typename state, class dataStructure>
void AStarMultiOpenClosed<state, dataStructure>::KeyChanged(uint64_t objKey) {
    std::vector<bool> movedInHeapifyUp = HeapifyUpAll(objKey); // First heapify up the element in all heaps
    for (int i = 0; i < movedInHeapifyUp.size(); i++) { // Then for each place it was not moved, try heapifying down
        if (!movedInHeapifyUp[i]) {
            HeapifyDown(i, elements[objKey].openLocations[i]);
        }
    }
}

/**
 * Returns location of object as well as object key.
 */
template<typename state, class dataStructure>
dataLocation AStarMultiOpenClosed<state, dataStructure>::Lookup(uint64_t hashKey, uint64_t &objKey) const {
    typename IndexTable::const_iterator it;
    it = table.find(hashKey);
    if (it != table.end()) {
        objKey = (*it).second;
        return elements[objKey].where;
    }
    return kNotFound;
}

/**
 * Remove item from open/closed given a state hash
 */
template<typename state, class dataStructure>
void AStarMultiOpenClosed<state, dataStructure>::Remove(uint64_t hash) {
    uint64_t index = table[hash];
    for (int i = 0; i < theHeaps.size(); i++) {
        uint64_t openLoc = elements[index].openLocations[i];
        uint64_t swappedItem = theHeaps[i].back();
        theHeaps[i][openLoc] = theHeaps[i].back();
        theHeaps[i].pop_back();
        elements[swappedItem].openLocations[i] = openLoc;
    }
    KeyChanged(index);
    table.erase(table.find(hash));
}

/**
 * Peek at the next item to be expanded in a given heap  (defaults to main heap (id=0)).
 */
template<typename state, class dataStructure>
uint64_t AStarMultiOpenClosed<state, dataStructure>::Peek(unsigned int heapNum) const {
    assert(OpenSize() != 0);
    return theHeaps[heapNum][0];
}

/**
 * Move the best item from the main heap to the closed list and return key.
 */
template<typename state, class dataStructure>
uint64_t AStarMultiOpenClosed<state, dataStructure>::Close() {
    assert(OpenSize() != 0);
    uint64_t ans = theHeaps[mainHeapNum][0];
    assert(elements[ans].openLocations[mainHeapNum] == 0);
    return Close(ans);
}

/**
 * Move the given item to the closed list and return key.
 */
template<typename state, class dataStructure>
uint64_t AStarMultiOpenClosed<state, dataStructure>::Close(uint64_t objKey) {
    assert(OpenSize() != 0);
    std::vector<uint64_t> indexes = elements[objKey].openLocations;
    uint64_t ans = theHeaps[mainHeapNum][indexes[mainHeapNum]];
    assert(ans == objKey);
    elements[ans].where = kClosedList;
    for (int i = 0; i < theHeaps.size(); i++) {
        theHeaps[i][indexes[i]] = theHeaps[i][theHeaps[i].size() - 1];
        elements[theHeaps[i][indexes[i]]].openLocations[i] = indexes[i];
        theHeaps[i].pop_back();
        if (!HeapifyUp(i, indexes[i])) {
            HeapifyDown(i, indexes[i]);
        }
    }
    return ans;
}

/**
 * Move item off the closed list and back onto the open list.
 */
template<typename state, class dataStructure>
void AStarMultiOpenClosed<state, dataStructure>::Reopen(uint64_t objKey) {
    assert(elements[objKey].where == kClosedList);
    elements[objKey].reopened = true;
    elements[objKey].where = kOpenList;
    for (int i = 0; i < theHeaps.size(); i++) {
        elements[objKey].openLocations[i] = theHeaps[i].size();
        theHeaps[i].push_back(objKey);
        HeapifyUp(i, theHeaps[i].size() - 1);
    }
}

/**
 * Heapify up a specified element (by id) in all heaps
 */
template<typename state, class dataStructure>
std::vector<bool> AStarMultiOpenClosed<state, dataStructure>::HeapifyUpAll(uint64_t val) {
    std::vector<bool> elementMoved(theHeaps.size(), false);
    for (int i = 0; i < theHeaps.size(); i++) {
        elementMoved[i] = HeapifyUp(i, elements[val].openLocations[i]);
    }
    return elementMoved;
}

/**
 * Moves a node up the heap. Returns true if the node was moved, false otherwise.
 */
template<typename state, class dataStructure>
bool AStarMultiOpenClosed<state, dataStructure>::HeapifyUp(unsigned int heapNum, unsigned int index) {
    if (index == 0) return false;
    int parent = (index - 1) / 2;
    if ((*cmpKeys[heapNum])(elements[theHeaps[heapNum][parent]], elements[theHeaps[heapNum][index]])) {
        unsigned int tmp = theHeaps[heapNum][parent];
        theHeaps[heapNum][parent] = theHeaps[heapNum][index];
        theHeaps[heapNum][index] = tmp;
        elements[theHeaps[heapNum][parent]].openLocations[heapNum] = parent;
        elements[theHeaps[heapNum][index]].openLocations[heapNum] = index;
        HeapifyUp(heapNum, parent);
        return true;
    }
    return false;
}

/**
 * Moves a node down the heap.
 */
template<typename state, class dataStructure>
void AStarMultiOpenClosed<state, dataStructure>::HeapifyDown(unsigned int heapNum, unsigned int index) {
    unsigned int child1 = index * 2 + 1;
    unsigned int child2 = index * 2 + 2;
    int which;
    unsigned int count = theHeaps[heapNum].size();
    if (child1 >= count)
        return;
    else if (child2 >= count)
        which = child1;
    else if (!((*cmpKeys[heapNum])(elements[theHeaps[heapNum][child1]], elements[theHeaps[heapNum][child2]])))
        which = child1;
    else
        which = child2;

    if (!((*cmpKeys[heapNum])(elements[theHeaps[heapNum][which]], elements[theHeaps[heapNum][index]]))) {
        unsigned int tmp = theHeaps[heapNum][which];
        theHeaps[heapNum][which] = theHeaps[heapNum][index];
        theHeaps[heapNum][index] = tmp;
        elements[theHeaps[heapNum][which]].openLocations[heapNum] = which;
        elements[theHeaps[heapNum][index]].openLocations[heapNum] = index;
        HeapifyDown(heapNum, which);
    }
}

#endif
