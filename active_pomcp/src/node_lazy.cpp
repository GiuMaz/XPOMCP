#include "node_lazy.h"
#include "history.h"
#include "utils.h"

using namespace std;

//-----------------------------------------------------------------------------

void QNODE_LAZY::Initialise()
{
    Children.clear();
    AlphaData.AlphaSum.clear();
}

void QNODE_LAZY::DisplayValue(HISTORY& history, int maxDepth, ostream& ostr) const {
    history.Display(ostr);
    ostr << ": " << Value.GetValue() << " (" << Value.GetCount() << ")\n";
    if (history.Size() >= maxDepth)
        return;

    for (const auto & [k, v] : Children) {
        if (v) { // TODO: controllare se può essere nullptr
            history.Back().Observation = k;
            v->DisplayValue(history, maxDepth, ostr);
        }
    }
}

void QNODE_LAZY::DisplayPolicy(HISTORY& history, int maxDepth, ostream& ostr) const
{
    history.Display(ostr);
    ostr << ": " << Value.GetValue() << " (" << Value.GetCount() << ")\n";
    if (history.Size() >= maxDepth)
        return;

    for (const auto & [k, v] : Children) {
        if (v) {
            history.Back().Observation = k;
            v->DisplayPolicy(history, maxDepth, ostr);
        }
    }
}

//-----------------------------------------------------------------------------

MEMORY_POOL<VNODE_LAZY> VNODE_LAZY::VNodePool;

int VNODE_LAZY::NumChildren = 0;

void VNODE_LAZY::Initialise()
{
    assert(NumChildren);
    Children.resize(VNODE_LAZY::NumChildren);
    for (int action = 0; action < VNODE_LAZY::NumChildren; action++)
        Children[action].Initialise();
}

VNODE_LAZY* VNODE_LAZY::Create()
{
    VNODE_LAZY* vnode = VNodePool.Allocate();
    vnode->Initialise();
    return vnode;
}

void VNODE_LAZY::Free(VNODE_LAZY* vnode, const SIMULATOR_LAZY& simulator)
{
    vnode->BeliefState.Free(simulator);
    VNodePool.Free(vnode);
    for (int action = 0; action < VNODE_LAZY::NumChildren; action++) {
        for (auto [k, v] : vnode->Child(action)) {
            if (v)
                Free(v, simulator);
        }
    }
}

void VNODE_LAZY::FreeAll()
{
    VNodePool.DeleteAll();
}

void VNODE_LAZY::SetChildren(int count, double value)
{
    for (int action = 0; action < NumChildren; action++)
    {
        QNODE_LAZY& qnode = Children[action];
        qnode.Value.Set(count, value);
    }
}

void VNODE_LAZY::DisplayValue(HISTORY& history, int maxDepth, ostream& ostr) const
{
    if (history.Size() >= maxDepth)
        return;

    for (int action = 0; action < NumChildren; action++)
    {
        history.Add(action);
        Children[action].DisplayValue(history, maxDepth, ostr);
        history.Pop();
    }
}

void VNODE_LAZY::DisplayPolicy(HISTORY& history, int maxDepth, ostream& ostr) const
{
    if (history.Size() >= maxDepth)
        return;

    double bestq = -Infinity;
    int besta = -1;
    for (int action = 0; action < NumChildren; action++)
    {
        if (Children[action].Value.GetValue() > bestq)
        {
            besta = action;
            bestq = Children[action].Value.GetValue();
        }
    }

    if (besta != -1)
    {
        history.Add(besta);
        Children[besta].DisplayPolicy(history, maxDepth, ostr);
        history.Pop();
    }
}

//-----------------------------------------------------------------------------

