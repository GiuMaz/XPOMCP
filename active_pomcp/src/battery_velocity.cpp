#include "battery_velocity.h"
#include "utils.h"
#include <fstream>
#include <sstream>
#include <z3.h>
#include <z3++.h>

using namespace std;
using namespace UTILS;

BATTERY_VELOCITY::BATTERY_VELOCITY(std::vector<int> nSubSegs, std::vector<std::vector<double>> subSegLengths, int nEnginePowerValues, int nDifficultyValues, int nVelocityValues)
    :   nSubSegs(nSubSegs),
    subSegLengths(subSegLengths),
    nDifficultyValues(nDifficultyValues),
    nVelocityValues(nVelocityValues),
    nSeg(subSegLengths.size()),
    unif_dist(0.0, 1.0)
{
    NumActions = 1 + nEnginePowerValues; 

    NumObservations = 3;
    collisionPenaltyTime=100;
    RewardRange = 103;
    Discount = 0.95;
}

STATE* BATTERY_VELOCITY::Copy(const STATE& state) const // Makes a copy of the state state
{
    const BATTERY_VELOCITY_STATE& bmState = safe_cast<const BATTERY_VELOCITY_STATE&>(state);
    BATTERY_VELOCITY_STATE* newstate = MemoryPool.Allocate();
    *newstate = bmState;
    return newstate;
}

// NOT USED
void BATTERY_VELOCITY::Validate(const STATE& state) const
{
    const BATTERY_VELOCITY_STATE& bmState = safe_cast<const BATTERY_VELOCITY_STATE&>(state);
}

// Used by standard planner
STATE* BATTERY_VELOCITY::CreateStartState() const
{
    BATTERY_VELOCITY_STATE* bmState = MemoryPool.Allocate();
    bmState->segDifficulties.clear();
    bmState->curSegI = 0;
    bmState->curSubsegJ=0;
    bmState->battery_level=BATTERY_MAX;

    for (int i = 0; i < nSeg; ++i) {
        int rnd= (random_state() % nDifficultyValues);
        bmState->segDifficulties.push_back(rnd); 
    }

    return bmState;
}

void BATTERY_VELOCITY::FreeState(STATE* state) const
{
    BATTERY_VELOCITY_STATE* bmState = safe_cast<BATTERY_VELOCITY_STATE*>(state);
    MemoryPool.Free(bmState);
}


bool BATTERY_VELOCITY::Step(STATE& state, int action,
        observation_t& observation, double& reward) const
{
    BATTERY_VELOCITY_STATE& s = safe_cast<BATTERY_VELOCITY_STATE&>(state);

    if (action == ACTION_RECHARGE) {
        if (s.curSubsegJ == 0) {
            s.battery_level = BATTERY_MAX;
            reward = -3;
            observation = OBS_NONE;
            return false;
        }
        else {
            reward = -100;
            observation = OBS_NONE;
            return false;
        }
    }

    if (s.battery_level == 0) {
        reward = -100;
        observation = OBS_NONE;
        return true;
    }

    double prob_collision;
    if((action==0) && (s.segDifficulties[s.curSegI]==0)) // Low difficulty
        prob_collision=0.0; // 0.0
    if((action==0) && (s.segDifficulties[s.curSegI]==1)) // Medium difficulty
        prob_collision=0.0; // 0.0
    if((action==0) && (s.segDifficulties[s.curSegI]==2)) // High difficulty
        prob_collision=0.028; // 0.028

    if((action==1) && (s.segDifficulties[s.curSegI]==0)) // Low difficulty
        prob_collision=0.0; // 0.0
    if((action==1) && (s.segDifficulties[s.curSegI]==1)) // Medium difficulty
        prob_collision=0.056; // 0.056
    if((action==1) && (s.segDifficulties[s.curSegI]==2)) // High difficulty
        prob_collision=0.11; // 0.11

    if((action==2) && (s.segDifficulties[s.curSegI]==0)) // Low difficulty
        prob_collision=0.0; // 0.0
    if((action==2) && (s.segDifficulties[s.curSegI]==1)) // Medium difficulty
        prob_collision=0.14; // 0.14
    if((action==2) && (s.segDifficulties[s.curSegI]==2)) // High difficulty
        prob_collision=0.25; // 0.25

    // 1  collision, 0 = no collision
    int dp= unif_dist(random_state) < prob_collision ? 1 : 0;

    double move_time =
        (nVelocityValues - action) * subSegLengths[s.curSegI][s.curSubsegJ];
    double collision_time = dp * collisionPenaltyTime;

    reward= move_time - collision_time;

    set_observation(observation, s);

    // Update position, to be ready for next step
    bool is_last = s.curSegI == (nSeg-1) && s.curSubsegJ==(nSubSegs[s.curSegI]-1);

    if (s.curSubsegJ==(nSubSegs[s.curSegI]-1)) {
        s.curSegI += 1;
        s.curSubsegJ = 0;
    }
    else {
        s.curSubsegJ += 1;
    }

    // 80 % of using battery
    s.battery_level =
        unif_dist(random_state) < 0.8 ? s.battery_level - 1 : s.battery_level;

    if (is_last) {
        reward += 100;
        return true; // terminal state
    }
    else
        return false;
}

void BATTERY_VELOCITY::set_observation(SIMULATOR::observation_t &obs,
                                        BATTERY_VELOCITY_STATE &s) const {
    double prob_observe_obstacle;

    if (s.segDifficulties[s.curSegI] == 0) // Low difficulty
        prob_observe_obstacle = 0.44;      // 0.44
    if (s.segDifficulties[s.curSegI] == 1) // Medium difficulty
        prob_observe_obstacle = 0.79;      // 0.79
    if (s.segDifficulties[s.curSegI] == 2) // High difficulty
        prob_observe_obstacle = 0.86;        // 0.86
    int o = unif_dist(random_state) < prob_observe_obstacle ? OBS_CLUTTERED
                                                            : OBS_CLEAR;

    obs = o;
}

void BATTERY_VELOCITY::reset_observation(SIMULATOR::observation_t &obs,
                                          SIMULATOR::observation_t value,
                                          BATTERY_VELOCITY_STATE &s) const {
    obs = value;
}

bool BATTERY_VELOCITY::LocalMove(STATE& state, const HISTORY& history,
        int stepObs, const STATUS& status) const
{
    return true;
}

// Puts in legal a set of legal actions that can be taken from state
void BATTERY_VELOCITY::GenerateLegal(const STATE& state, const HISTORY& history,
        vector<int>& legal, const STATUS& status) const
{
    legal.push_back(0);
    legal.push_back(1);
    legal.push_back(2);
}

void BATTERY_VELOCITY::GeneratePreferred(const STATE& state, const HISTORY& history,
        vector<int>& actions, const STATUS& status) const
{
    actions.push_back(0);
    actions.push_back(1);
    actions.push_back(2);
}

// Display methods -------------------------
void BATTERY_VELOCITY::DisplayBeliefs(const BELIEF_STATE& beliefState,    // Displays all states in beliefState
        std::ostream& ostr) const
{
    cout << "BATTERY_VELOCITY::DisplayBeliefs start" << endl;
    for (int i = 0; i < beliefState.GetNumSamples(); i++){
        const STATE* s = beliefState.GetSample(i);
        DisplayState(*s,cout);
    }
    cout << "BATTERY_VELOCITY::DisplayBeliefs end" << endl;
}

void BATTERY_VELOCITY::DisplayBeliefIds(const BELIEF_STATE& beliefState,    // Displays all states in beliefState
        std::ostream& ostr) const
{
    cout << "BATTERY_VELOCITY::DisplayBeliefIds: [";
    for (int i = 0; i < beliefState.GetNumSamples(); i++){
        const STATE* s = beliefState.GetSample(i);
        DisplayStateId(*s, cout);cout <<"; ";
    }
    cout << "BATTERY_VELOCITY::DisplayBeliefs end" << endl;
}

void BATTERY_VELOCITY::DisplayBeliefDistribution(const BELIEF_STATE& beliefState,
        std::ostream& ostr) const
{
    std::unordered_map<int, int> dist;
    int id;
    for (int i = 0; i < beliefState.GetNumSamples(); i++){
        const STATE* state = beliefState.GetSample(i);
        const BATTERY_VELOCITY_STATE& bmState = safe_cast<const BATTERY_VELOCITY_STATE&>(*state);

        id=0;   
        for (int j = 0; j<nSeg; j++)  // For each segment difficulty
        {
            id+=bmState.segDifficulties[j]*(pow(3,nSeg-j-1));
        }
        // If it is not in dist then add it and initialize to 1
        if (dist.find(id) == dist.end())
            dist[id]= 1;
        else
            dist[id]++;
    }
    for (auto it = dist.begin(); it != dist.end(); ++it ){  // For each state in the belief
        ostr << it->first << ":" << it->second << ", ";
    }
}

void BATTERY_VELOCITY::DisplayState(const STATE& state, std::ostream& ostr) const
{
    const BATTERY_VELOCITY_STATE& bmState = safe_cast<const BATTERY_VELOCITY_STATE&>(state);
    ostr << endl;

    // Display segment difficulties
    ostr << "## STATE ############" << endl;
    ostr << "Difficulties: ";
    for (int i = 0; i < nSeg; i++)
        ostr << i << ":" << bmState.segDifficulties[i] << ", ";
    ostr << endl;

    // Display agent's position
    ostr << "Position: i:" << bmState.curSegI << ", j:" << bmState.curSubsegJ << endl;
    // TODO finisci
}

void BATTERY_VELOCITY::DisplayObservation(const STATE& state, observation_t observation, std::ostream& ostr) const // Prints the observation
{
    if (observation == 0)
        ostr << "Observed cluttered" << endl;
    else
        ostr << "Observed uncluttered" << endl;
}

void BATTERY_VELOCITY::DisplayAction(int action, std::ostream& ostr) const
{
    if (action == 0)  
        ostr << "Action: Low power (0)" << endl;
    if (action == 1) 
        ostr << "Action: Medium power (1)" << endl;
    if (action == 2) 
        ostr << "Action: High power (2)" << endl;
}


void BATTERY_VELOCITY::DisplayStateId(const STATE& state, std::ostream& ostr) const // Displays an id from rock values
{
    const BATTERY_VELOCITY_STATE& bmState = safe_cast<const BATTERY_VELOCITY_STATE&>(state);
    int id=0;
    string s="State id: ";
    for (int i = 0; i<nSeg; i++)  // For each segment difficulty
    {
        id+=bmState.segDifficulties[i]*(pow(nDifficultyValues,nSeg-i-1));
        s=s+to_string(bmState.segDifficulties[i]);
        if(i!=(nSeg-1)){
            s=s+"-";
        }
    }
    cout << s << "(" << id << ")";
}

void BATTERY_VELOCITY::log_problem_info() const {
    // TODO aggiusta
    XES::logger().add_attributes({
            {"problem", "obstacle avoidance"},
            {"RewardRange", RewardRange},
            {"velocity values", nVelocityValues},
            {"difficulty values", nDifficultyValues}
        });

    int i = 1;
    XES::logger().start_list("map");
    for (const auto &seg : subSegLengths) {
        XES::logger().start_list("segment " + std::to_string(i));
        int j = 1;
        for (double subseg : seg) {
            XES::logger().add_attribute({"subseg " + std::to_string(j), subseg});
            ++j;
        }
        XES::logger().end_list();
        ++i;
    }
    XES::logger().end_list();
}

void BATTERY_VELOCITY::log_beliefs(const BELIEF_STATE& beliefState) const {
    static std::unordered_map<int, int> dist;
    dist.clear();
    int id;
    for (int i = 0; i < beliefState.GetNumSamples(); i++){
        const STATE* state = beliefState.GetSample(i);
        const BATTERY_VELOCITY_STATE& bmState = safe_cast<const BATTERY_VELOCITY_STATE&>(*state);

        id=0;   
        for (int j = 0; j<nSeg; j++)  // For each segment difficulty
        {
            id+=bmState.segDifficulties[j]*(pow(3,nSeg-j-1));
        }
        // If it is not in dist then add it and initialize to 1
        if (dist.find(id) == dist.end())
            dist[id]= 1;
        else
            dist[id]++;
    }

    const BATTERY_VELOCITY_STATE& bmState =
        safe_cast<const BATTERY_VELOCITY_STATE&>(*beliefState.GetSample(0));

    XES::logger().add_attribute({"segment", bmState.curSegI });
    XES::logger().add_attribute({"subsegment", bmState.curSubsegJ });
    XES::logger().start_list("belief");
    for (const auto &[k, v] : dist)
        XES::logger().add_attribute({std::to_string(k), v});
    XES::logger().end_list();
}

void BATTERY_VELOCITY::log_state(const STATE& state) const {
    const auto& obs_state = safe_cast<const BATTERY_VELOCITY_STATE&>(state);
    XES::logger().start_list("state");
    XES::logger().add_attribute({"segment", obs_state.curSegI});
    XES::logger().add_attribute({"subsegment", obs_state.curSubsegJ});

    XES::logger().start_list("difficulties");
    for (int i = 0; i < obs_state.segDifficulties.size(); ++i)
        XES::logger().add_attribute({"segment " + std::to_string(i + 1),
                obs_state.segDifficulties[i] });
    XES::logger().end_list();

    XES::logger().end_list();
}

void BATTERY_VELOCITY::log_action(int action) const {
    XES::logger().add_attribute({"action", action});
}

void BATTERY_VELOCITY::log_observation(const STATE& state, observation_t observation) const {
    XES::logger().add_attribute({"observation", observation});
}

void BATTERY_VELOCITY::log_reward(double reward) const {
    XES::logger().add_attribute({"reward", reward});
}

void BATTERY_VELOCITY::set_belief_metainfo(VNODE *v, const SIMULATOR &) const {
    v->Beliefs().set_metainfo(BATTERY_VELOCITY_METAINFO(), *this);
}

