#ifndef NETWORK_H
#define NETWORK_H

#include "simulator.h"

class NETWORK_STATE : public STATE
{
public:

    std::vector<bool> Machines;
};

class NETWORK : public SIMULATOR
{
public:

    enum
    {
        E_CYCLE,
        E_3LEGS
    };

    NETWORK(int numMachines, int ntype);

    virtual STATE* Copy(const STATE& state) const;
    virtual void Validate(const STATE& state) const;
    virtual STATE* CreateStartState() const;
    virtual void FreeState(STATE* state) const;
    virtual bool Step(STATE& state, int action, 
        int& observation, double& reward, const BELIEF_STATE& beliefState) const;
        
    virtual void DisplayBeliefs(const BELIEF_STATE& beliefState, 
        std::ostream& ostr) const;
    virtual void DisplayState(const STATE& state, std::ostream& ostr) const;
    virtual void DisplayObservation(const STATE& state, int observation, std::ostream& ostr) const;
    virtual void DisplayAction(int action, std::ostream& ostr) const;
    virtual double JointProb(const STATE& state) const;

private:

    void MakeRingNeighbours();
    void Make3LegsNeighbours();
    
    int NumMachines;
    double FailureProb1, FailureProb2, ObsProb;
    std::vector<std::vector<int> > Neighbours;
    
    mutable MEMORY_POOL<NETWORK_STATE> MemoryPool;
};

#endif // NETWORK_H
