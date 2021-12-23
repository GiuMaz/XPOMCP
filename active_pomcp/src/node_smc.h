#pragma once

#include "beliefstate.h"
#include "utils.h"
#include <iostream>
#include "node.h"

class HISTORY;
class SIMULATOR;
class SMC_VNODE;

//-----------------------------------------------------------------------------

class SMC_QNODE
{
public:

    VALUE<int> Value;
    VALUE<double> AMAF;

    void Initialise();

    SMC_VNODE*& Child(int c) { return Children[c]; }
    SMC_VNODE* Child(int c) const { return Children[c]; }

    void DisplayValue(HISTORY& history, int maxDepth, std::ostream& ostr) const;
    void DisplayPolicy(HISTORY& history, int maxDepth, std::ostream& ostr) const;

    static int NumChildren;

private:
    std::vector<SMC_VNODE*> Children;

friend class SMC_VNODE;
};

class SMC_VNODE : public MEMORY_OBJECT
{
public:
    VALUE<int> Value;
    VALUE<double> Probability;

    void Initialise();
    static SMC_VNODE* Create();
    static void Free(SMC_VNODE* vnode, const SIMULATOR& simulator);
    static void FreeAll();

    SMC_QNODE& Child(int c) { return Children[c]; }
    const SMC_QNODE& Child(int c) const { return Children[c]; }
    BELIEF_STATE& Beliefs() { return BeliefState; }
    const BELIEF_STATE& Beliefs() const { return BeliefState; }

    void SetChildren(int count, double value);

    void DisplayValue(HISTORY& history, int maxDepth, std::ostream& ostr) const;
    void DisplayPolicy(HISTORY& history, int maxDepth, std::ostream& ostr) const;

    static int NumChildren;

private:
    std::vector<SMC_QNODE> Children;
    BELIEF_STATE BeliefState;
    static MEMORY_POOL<SMC_VNODE> VNodePool;
};

