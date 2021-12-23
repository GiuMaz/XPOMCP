#pragma once

#include "beliefstate.h"
#include "utils.h"
#include "node.h"
#include <iostream>
#include <iterator>
#include <map>

class VNODE_LAZY;

class QNODE_LAZY
{
public:

    VALUE<int> Value;

    void Initialise();

    auto begin() { return Children.begin(); }
    auto end() { return Children.end(); }

    auto begin() const { return Children.cbegin(); }
    auto end() const { return Children.cend(); }

    VNODE_LAZY*& Child(uint64_t c) {
        if (Children.find(c) == Children.end())
            Children[c] = nullptr;
        return Children[c];
    }

    VNODE_LAZY* Child(uint64_t c) const {
        if (Children.find(c) == Children.end())
            return nullptr;
        return Children.at(c);
    }

    ALPHA& Alpha() { return AlphaData; }
    const ALPHA& Alpha() const { return AlphaData; }

    void DisplayValue(HISTORY& history, int maxDepth, std::ostream& ostr) const;
    void DisplayPolicy(HISTORY& history, int maxDepth, std::ostream& ostr) const;

private:
    std::map<uint64_t, VNODE_LAZY*> Children;
    ALPHA AlphaData;

friend class VNODE;
};

//-----------------------------------------------------------------------------

class VNODE_LAZY : public MEMORY_OBJECT
{
public:

    VALUE<int> Value;

    void Initialise();
    static VNODE_LAZY* Create();
    static void Free(VNODE_LAZY* vnode, const SIMULATOR_LAZY& simulator);
    static void FreeAll();

    QNODE_LAZY& Child(int c) { return Children[c]; }
    const QNODE_LAZY& Child(int c) const { return Children[c]; }
    BELIEF_STATE& Beliefs() { return BeliefState; }
    const BELIEF_STATE& Beliefs() const { return BeliefState; }

    void SetChildren(int count, double value);

    void DisplayValue(HISTORY& history, int maxDepth, std::ostream& ostr) const;
    void DisplayPolicy(HISTORY& history, int maxDepth, std::ostream& ostr) const;

    static int NumChildren;

private:

    std::vector<QNODE_LAZY> Children;
    BELIEF_STATE BeliefState;
    static MEMORY_POOL<VNODE_LAZY> VNodePool;
};

