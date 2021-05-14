#ifndef TEST_SIMULATOR_H
#define TEST_SIMULATOR_H

#include "simulator.h"

class TEST_STATE : public STATE
{
public:

    int Depth;
    
    TEST_STATE()
    : Depth(0) { }
};

class TEST_SIMULATOR : public SIMULATOR
{
public:

    TEST_SIMULATOR(int actions, int observations, int maxDepth)
    :   SIMULATOR(actions, observations),
        MaxDepth(maxDepth)
    { }

    virtual STATE* CreateStartState() const;
    virtual bool Step(STATE& state, int action, 
        int& observation, double& reward) const;
    virtual STATE* Copy(const STATE& state) const;
    virtual void FreeState(STATE* state) const;

    double OptimalValue() const;
    double MeanValue() const;

private:
    
    int MaxDepth;
};

#endif // TEST_SIMULATOR_H
