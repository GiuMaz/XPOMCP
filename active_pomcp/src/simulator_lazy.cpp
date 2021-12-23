#include "simulator_lazy.h"

using namespace std;
using namespace UTILS;

SIMULATOR_LAZY::SIMULATOR_LAZY() 
:   Discount(1.0),
    NumActions(0),
    NumObservations(0),
    RewardRange(1.0)
{
}

SIMULATOR_LAZY::SIMULATOR_LAZY(int numActions, observation_t numObservations, double discount)
:   NumActions(numActions),
    NumObservations(numObservations),
    Discount(discount)
{ 
    assert(discount > 0 && discount <= 1);
}

SIMULATOR_LAZY::~SIMULATOR_LAZY() 
{ 
}

void SIMULATOR_LAZY::Validate(const STATE& state) const 
{ 
}

bool SIMULATOR_LAZY::LocalMove(STATE& state, const HISTORY& history,
    observation_t stepObs, const STATUS& status) const
{
    return true;
}

void SIMULATOR_LAZY::GenerateLegal(const STATE& state, const HISTORY& history, 
    std::vector<int>& actions, const STATUS& status) const
{
    for (int a = 0; a < NumActions; ++a)
        actions.push_back(a);
}

void SIMULATOR_LAZY::GeneratePreferred(const STATE& state, const HISTORY& history, 
    std::vector<int>& actions, const STATUS& status) const
{
}

int SIMULATOR_LAZY::SelectRandom(const STATE& state, const HISTORY& history,
    const STATUS& status) const
{
    static vector<int> actions;

    if (Knowledge.RolloutLevel >= KNOWLEDGE::SMART)
    {
        actions.clear();
        GeneratePreferred(state, history, actions, status);
        if (!actions.empty())
            return actions[Random(actions.size())];
    }
        
    if (Knowledge.RolloutLevel >= KNOWLEDGE::LEGAL)
    {
        actions.clear();
        GenerateLegal(state, history, actions, status);
        if (!actions.empty())
            return actions[Random(actions.size())];
    }

    return Random(NumActions);
}

void SIMULATOR_LAZY::Prior(const STATE* state, const HISTORY& history,
    VNODE_LAZY* vnode, const STATUS& status) const
{
    static vector<int> actions;
    
    if (Knowledge.TreeLevel == KNOWLEDGE::PURE || state == 0)
    {
        vnode->SetChildren(0, 0);
        return;
    }
    else
    {
        vnode->SetChildren(+LargeInteger, -Infinity);
    }

    if (Knowledge.TreeLevel >= KNOWLEDGE::LEGAL)
    {
        actions.clear();
        GenerateLegal(*state, history, actions, status);

        for (vector<int>::const_iterator i_action = actions.begin(); i_action != actions.end(); ++i_action)
        {
            int a = *i_action;
            QNODE_LAZY& qnode = vnode->Child(a);
            qnode.Value.Set(0, 0);
        }
    }
    
    if (Knowledge.TreeLevel >= KNOWLEDGE::SMART)
    {
        actions.clear();
        GeneratePreferred(*state, history, actions, status);

        for (vector<int>::const_iterator i_action = actions.begin(); i_action != actions.end(); ++i_action)
        {
            int a = *i_action;
            QNODE_LAZY& qnode = vnode->Child(a);
            qnode.Value.Set(Knowledge.SmartTreeCount, Knowledge.SmartTreeValue);
        }    
    }
}

bool SIMULATOR_LAZY::HasAlpha() const
{
    return false;
}

void SIMULATOR_LAZY::AlphaValue(const QNODE& qnode, double& q, int& n) const
{
}

void SIMULATOR_LAZY::UpdateAlpha(QNODE& qnode, const STATE& state) const
{
}

void SIMULATOR_LAZY::DisplayBeliefs(const BELIEF_STATE& beliefState, 
    ostream& ostr) const
{
}

void SIMULATOR_LAZY::DisplayState(const STATE& state, ostream& ostr) const 
{
}

void SIMULATOR_LAZY::DisplayAction(int action, ostream& ostr) const 
{
    ostr << "Action " << action << endl;
}

void SIMULATOR_LAZY::DisplayObservation(const STATE& state, observation_t observation, ostream& ostr) const
{
    ostr << "Observation " << observation << endl;
}

void SIMULATOR_LAZY::DisplayReward(double reward, std::ostream& ostr) const
{
    ostr << "Reward " << reward << endl;
}

double SIMULATOR_LAZY::GetHorizon(double accuracy, int undiscountedHorizon) const 
{ 
    if (Discount == 1)
        return undiscountedHorizon;
    return log(accuracy) / log(Discount);
}

