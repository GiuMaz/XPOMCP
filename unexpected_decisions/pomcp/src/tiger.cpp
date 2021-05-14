#include "tiger.h"
#include "utils.h"
#include <fstream>

using namespace std;
using namespace UTILS;

TIGER::TIGER() :
    SIMULATOR(3, 4, 1.0)
{
    // -----------------------------------
    RewardRange = 40.0; // <----- correct is 110 [from -100 to 10]
    // -----------------------------------
}

STATE* TIGER::Copy(const STATE& state) const {
    const auto& tiger_state = safe_cast<const TIGER_STATE&>(state);
    TIGER_STATE* newstate = MemoryPool.Allocate();
    *newstate = tiger_state;
    return newstate;
}

// NOT USED
void TIGER::Validate(const STATE& /*state*/) const { }

STATE* TIGER::CreateStartState() const {
    TIGER_STATE* state = MemoryPool.Allocate();
    state->tiger_on_left = static_cast<bool>(Random(2));
    state->saved_actions.clear();
    return state;
}

void TIGER::FreeState(STATE* state) const // Free memory of state
{
    TIGER_STATE* s = safe_cast<TIGER_STATE*>(state);
    MemoryPool.Free(s);
}


bool TIGER::Step(STATE& state, int action, int& observation, double& reward,
                 const BELIEF_STATE& /*beliefState*/) const {


    TIGER_STATE& tiger_state = safe_cast<TIGER_STATE&>(state);
    reward = 0.0;
    tiger_state.saved_actions.push_back(action);

    if (action == A_LISTEN) // check
    {
        bool true_roar  = Random(100) < 85;
        if (true_roar)
            observation = tiger_state.tiger_on_left ? O_LEFT_ROAR : O_RIGHT_ROAR;
        else
            observation = tiger_state.tiger_on_left ? O_RIGHT_ROAR : O_LEFT_ROAR;

        reward = -1.0;
        return false;
    }

    // Action is a choise
    assert(action == A_LEFT_DOOR || action == A_RIGHT_DOOR);

    // check if the action chose is the same as the position of the tiger
    bool found_a_tiger = tiger_state.tiger_on_left == (action == A_LEFT_DOOR); 

    if (found_a_tiger) {
        reward = -100;
        observation = O_TIGER;
    }
    else { // treasure!
        reward = 10;
        observation = O_TREASURE;
    }
    return true;
}

bool TIGER::LocalMove(STATE& state, const HISTORY& history,
        int stepObservation, const STATUS& status) const {
    // not required
    return true;
}

void TIGER::GenerateLegal(const STATE& /*state*/, const HISTORY& /*history*/,
                          std::vector<int>& legal,
                          const STATUS& /*status*/) const {
    // in this simple game all the actions are always legal
    legal.push_back(A_LISTEN);
    legal.push_back(A_LEFT_DOOR);
    legal.push_back(A_RIGHT_DOOR);
}

void TIGER::GeneratePreferred(const STATE& /*state*/,
                              const HISTORY& /*history*/,
                              std::vector<int>& legal,
                              const STATUS& /*status*/) const {
    // no preferred action, it's a copy of GenerateLegal
    legal.push_back(A_LISTEN);
    legal.push_back(A_LEFT_DOOR);
    legal.push_back(A_RIGHT_DOOR);
}

void TIGER::DisplayBeliefs(const BELIEF_STATE& beliefState,
                           std::ostream& ostr) const {
    cout << "TIGER::DisplayBeliefs start" << endl;
    for (int i = 0; i < beliefState.GetNumSamples(); i++){
        const STATE* s = beliefState.GetSample(i);
        DisplayState(*s,cout);
    }
    cout << "TIGER::DisplayBeliefs end" << endl;
}

void TIGER::DisplayBeliefIds(const BELIEF_STATE& beliefState,
                             std::ostream& ostr) const {
    cout << "TIGER::DisplayBeliefIds: [";
    for (int i = 0; i < beliefState.GetNumSamples(); i++){
        const STATE* s = beliefState.GetSample(i);
        DisplayStateId(*s, cout);cout <<"; ";
    }
    cout << "TIGER::DisplayBeliefs end" << endl;
}

void TIGER::DisplayState(const STATE& state, std::ostream& ostr) const {
    const TIGER_STATE& tiger_state = safe_cast<const TIGER_STATE&>(state);
    ostr << endl;
    ostr << "## STATE ############" << endl;
    ostr << "tiger on left: " << tiger_state.tiger_on_left << endl;
    ostr << "#######################" << endl<< endl;
}

void TIGER::DisplayStateId(const STATE& state, std::ostream& ostr) const {
    const TIGER_STATE& tiger_state = safe_cast<const TIGER_STATE&>(state);
    int id = static_cast<int>(tiger_state.tiger_on_left);
    cout << "State id: " << "(" << id << ")";
}

void TIGER::DisplayObservation(const STATE& state, int observation,
                               std::ostream& ostr) const {
    switch (observation) {
        case O_LEFT_ROAR:
            ostr << "roar from left door" << endl;
            break;
        case O_RIGHT_ROAR:
            ostr << "roar from right door" << endl;
            break;
        case O_TREASURE:
            ostr << "TREASURE!" << endl;
            break;
        case O_TIGER:
            ostr << "TIGER!" << endl;
            break;
    }
}

void TIGER::DisplayAction(int action, std::ostream& ostr) const {
    switch (action) {
        case A_LISTEN:
            ostr << "listen" << endl;
            break;
        case A_LEFT_DOOR:
            ostr << "open left door" << endl;
            break;
        case A_RIGHT_DOOR:
            ostr << "open right door" << endl;
            break;
    }
}

void TIGER::DisplayBeliefDistribution(const BELIEF_STATE& beliefState,
                                      std::ostream& ostr) const {
    std::unordered_map<int, int> dist;
    int id;
    for (int i = 0; i < beliefState.GetNumSamples(); i++){
        const STATE* state = beliefState.GetSample(i);
        const TIGER_STATE& tiger_state = safe_cast<const TIGER_STATE&>(*state);
        
        id = static_cast<int>(tiger_state.tiger_on_left);   

        // If it is not in dist then add it and initialize to 1
        if (dist.find(id) == dist.end())
            dist[id]= 1;
        else
            dist[id]++;
    }
    for (auto it = dist.begin(); it != dist.end(); ++it ){
        ostr << it->first << ":" << it->second << ", ";
    }

}

void TIGER::DisplayStateHist(const STATE& state, const char* fileName) const {

    const TIGER_STATE& tiger_state = safe_cast<const TIGER_STATE&>(state);
    std::ofstream outFile;

    outFile.open(fileName, std::ofstream::out | std::ofstream::app);
    
    string s;
    s="Step";

    for (int i = 0; i < tiger_state.saved_actions.size(); i++)
        s=s + ", " + to_string(i);
    outFile << s << endl;

    s="OnLeft";
    for (int i = 0; i < tiger_state.saved_actions.size(); i++)
        s=s + ", " + to_string(tiger_state.tiger_on_left);
    outFile << s << endl;

    s="Actions";
    for (int i = 0; i < tiger_state.saved_actions.size(); i++)
        s=s + ", " + to_string(tiger_state.saved_actions[i]);
    outFile << s << endl;

    outFile << endl;

    outFile.close();
}
