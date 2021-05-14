#include "beliefstate.h"
#include "simulator.h"
#include "utils.h"

using namespace UTILS;

BELIEF_STATE::BELIEF_STATE() // Generates an empty belief state (i.e., an empty vector of state samples)
{
    Samples.clear();
}

void BELIEF_STATE::Free(const SIMULATOR& simulator) // Free the states (samples) of the belief in simulator
{
    for (std::vector<STATE*>::iterator i_state = Samples.begin();
            i_state != Samples.end(); ++i_state)
    {
        simulator.FreeState(*i_state);
    }
    Samples.clear();
}

STATE* BELIEF_STATE::CreateSample(const SIMULATOR& simulator) const // Returns a copy of that randomly chosen i-th state in the beliefstate
{
    int index = Random(Samples.size()); //Randomly selects a sample state
    std::cout << "BELIEF_STATE::CreateSample copy" << std::endl;
    return simulator.Copy(*Samples[index]); // Returns a copy of that state
}

void BELIEF_STATE::AddSample(STATE* state)  // Adds state to the beliefstate
{
    Samples.push_back(state);
}

void BELIEF_STATE::Copy(const BELIEF_STATE& beliefs, const SIMULATOR& simulator) // Makes a copy of all samples in the belief state
{
    for (std::vector<STATE*>::const_iterator i_state = beliefs.Samples.begin();
        i_state != beliefs.Samples.end(); ++i_state)
    {
        std::cout << "BELIEF_STATE::Copy copy" << std::endl;
        AddSample(simulator.Copy(**i_state));
    }
}

void BELIEF_STATE::Move(BELIEF_STATE& beliefs)  // Move all samples into this belief state
{
    for (std::vector<STATE*>::const_iterator i_state = beliefs.Samples.begin();
        i_state != beliefs.Samples.end(); ++i_state)
    {
        AddSample(*i_state);
    }
    beliefs.Samples.clear();
}


