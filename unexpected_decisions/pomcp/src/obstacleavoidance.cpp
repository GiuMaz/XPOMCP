#include "obstacleavoidance.h"
#include "utils.h"

using namespace std;
using namespace UTILS;

OBSTACLEAVOIDANCE::OBSTACLEAVOIDANCE(std::vector<int> nSubSegs, std::vector<std::vector<double>> subSegLengths, int nEnginePowerValues, int nDifficultyValues, int nVelocityValues)
    :   nSubSegs(nSubSegs),
    subSegLengths(subSegLengths),
    nDifficultyValues(nDifficultyValues),
    nVelocityValues(nVelocityValues),
    nSeg(subSegLengths.size())
{
    NumActions = nEnginePowerValues; 

    NumObservations = 4;
    collisionPenaltyTime=100;
    // ---------------------------------------------------
    RewardRange = 90; // <----- correct is 103
    // ---------------------------------------------------
    Discount = 0.95;
    RandomSeed(0);
}

STATE* OBSTACLEAVOIDANCE::Copy(const STATE& state) const // Makes a copy of the state state
{
    const OBSTACLEAVOIDANCE_STATE& bmState = safe_cast<const OBSTACLEAVOIDANCE_STATE&>(state);
    OBSTACLEAVOIDANCE_STATE* newstate = MemoryPool.Allocate();
    *newstate = bmState;
    return newstate;
}

// NOT USED
void OBSTACLEAVOIDANCE::Validate(const STATE& state) const
{
    const OBSTACLEAVOIDANCE_STATE& bmState = safe_cast<const OBSTACLEAVOIDANCE_STATE&>(state);
}

// Used by standard planner
STATE* OBSTACLEAVOIDANCE::CreateStartState() const
{
    OBSTACLEAVOIDANCE_STATE* bmState = MemoryPool.Allocate();
    bmState->dps.clear(); 
    bmState->ps.clear();
    bmState->vs.clear();
    bmState->acs.clear();
    bmState->dt1s.clear();
    bmState->dt2s.clear();
    bmState->dts.clear();
    bmState->ts.clear();
    bmState->ms.clear();
    bmState->os.clear();
    bmState->rs.clear();
    bmState->r_cums.clear();
    bmState->segDifficulties.clear();

    bmState->curSegI = 0;         // Set agent position (segment i)
    bmState->curSubsegJ=0;        // Set agent position (subsegment j)
    bmState->p=0;                 // Set initial number of collisions
    bmState->ps.push_back(0);
    bmState->t=0;                 // Set initial time
    bmState->ts.push_back(0);
    bmState->r_cums.push_back(0); // Set initial cumulative reward

    int rnd;
    for (int i = 0; i < nSeg; ++i) // Randomly set segment difficulties
    {
        rnd=Random(nDifficultyValues);
        bmState->segDifficulties.push_back(rnd); 
    }

    return bmState;
}

STATE* OBSTACLEAVOIDANCE::CreateStartStateFixedValues(std::vector<int> values) const
{

    OBSTACLEAVOIDANCE_STATE* bmState = MemoryPool.Allocate();
    bmState->dps.clear(); 
    bmState->ps.clear();
    bmState->vs.clear();
    bmState->acs.clear();
    bmState->dt1s.clear();
    bmState->dt2s.clear();
    bmState->dts.clear();
    bmState->ts.clear();
    bmState->ms.clear();
    bmState->os.clear();
    bmState->rs.clear();
    bmState->r_cums.clear();
    bmState->segDifficulties.clear();

    bmState->curSegI = 0;         // Set agent position (segment i)
    bmState->curSubsegJ=0;        // Set agent position (subsegment j)
    bmState->p=0;                 // Set initial number of collisions
    bmState->ps.push_back(0);
    bmState->t=0;                 // Set initial time
    bmState->ts.push_back(0);
    bmState->r_cums.push_back(0); // Set initial cumulative reward

    int rnd;
    for (int i = 0; i < nSeg; ++i) // Randomly set segment difficulties
    {
        bmState->segDifficulties.push_back(values[i]); 
    }

    return bmState;
}

STATE* OBSTACLEAVOIDANCE::CreateStartState(std::vector<double*>* stateVarRelationships) const
{
    OBSTACLEAVOIDANCE_STATE* bmState = MemoryPool.Allocate();
    return bmState;
}

STATE* OBSTACLEAVOIDANCE::CreateStartState(vector<STATE*>* allParticles, vector<double> allParticleCumProbs) const
{
    OBSTACLEAVOIDANCE_STATE* bmState = MemoryPool.Allocate();
    bmState->dps.clear(); 
    bmState->ps.clear();
    bmState->vs.clear();
    bmState->acs.clear();
    bmState->dt1s.clear();
    bmState->dt2s.clear();
    bmState->dts.clear();
    bmState->ts.clear();
    bmState->ms.clear();
    bmState->os.clear();
    bmState->rs.clear();
    bmState->r_cums.clear();
    bmState->segDifficulties.clear();

    bmState->curSegI = 0;         // Set agent position (segment i)
    bmState->curSubsegJ=0;        // Set agent position (subsegment j)
    bmState->p=0;                 // Set initial number of collisions
    bmState->ps.push_back(0);
    bmState->t=0;                 // Set initial time
    bmState->ts.push_back(0);
    bmState->r_cums.push_back(0); // Set initial cumulative reward

    // Select a particle from allParticles with probability from allParticleProb
    double rnd=RandomDouble(0,0.9999999999);
    int partI=0; // Particle index
    double cumProbTmp=0.0;
    while(cumProbTmp<=rnd){
        partI++;
        cumProbTmp=allParticleCumProbs[partI];
    }
    partI--;
    const OBSTACLEAVOIDANCE_STATE& bmParticle = safe_cast<const OBSTACLEAVOIDANCE_STATE&>(*((*allParticles)[partI]));

    for (int i = 0; i < nSeg; i++) // Set each segment difficulty to that of the particle
    {
        bmState->segDifficulties.push_back(bmParticle.segDifficulties[i]);
    }
    return bmState;
}

void OBSTACLEAVOIDANCE::FreeState(STATE* state) const
{
    OBSTACLEAVOIDANCE_STATE* bmState = safe_cast<OBSTACLEAVOIDANCE_STATE*>(state);
    MemoryPool.Free(bmState);
}


bool OBSTACLEAVOIDANCE::Step(STATE& state, int action,
        int& observation, double& reward, const BELIEF_STATE& beliefState) const
{
    OBSTACLEAVOIDANCE_STATE& bmState = safe_cast<OBSTACLEAVOIDANCE_STATE&>(state);

    bmState.acs.push_back(action); // In history

    // Compute next indices
    int nextCurSegI;
    int nextCurSubsegJ;
    if((bmState.curSegI==(nSeg-1)) && (bmState.curSubsegJ==(nSubSegs[bmState.curSegI]-1))){ // Last step, end state
        nextCurSegI=bmState.curSegI;
        nextCurSubsegJ=bmState.curSubsegJ+1;
    }
    else{
        if(bmState.curSubsegJ<(nSubSegs[bmState.curSegI]-1)){
            nextCurSegI=bmState.curSegI;
            nextCurSubsegJ=bmState.curSubsegJ+1;
        }  
        else{
            nextCurSegI=bmState.curSegI+1;
            nextCurSubsegJ=0;
        }
    }

    bmState.v=action; // In state <-------- VELOCITY (3)
    bmState.vs.push_back(action); // In history
    double pdp_0, pdp_1; // 0: no collision, 1: yes collision

    // Model
    if((action==0) && (bmState.segDifficulties[bmState.curSegI]==0)){ // Low difficulty
        pdp_1=0.0; // 0.0
    }
    if((action==0) && (bmState.segDifficulties[bmState.curSegI]==1)){ // Medium difficulty
        pdp_1=0.0; // 0.0
    }
    if((action==0) && (bmState.segDifficulties[bmState.curSegI]==2)){ // High difficulty
        pdp_1=0.028; // 0.028
    }

    if((action==1) && (bmState.segDifficulties[bmState.curSegI]==0)){ // Low difficulty
        pdp_1=0.0; // 0.0
    }
    if((action==1) && (bmState.segDifficulties[bmState.curSegI]==1)){ // Medium difficulty
        pdp_1=0.056; // 0.056
    }
    if((action==1) && (bmState.segDifficulties[bmState.curSegI]==2)){ // High difficulty
        pdp_1=0.11; // 0.11
    }

    if((action==2) && (bmState.segDifficulties[bmState.curSegI]==0)){ // Low difficulty
        pdp_1=0.0; // 0.0
    }
    if((action==2) && (bmState.segDifficulties[bmState.curSegI]==1)){ // Medium difficulty
        pdp_1=0.14; // 0.14
    }
    if((action==2) && (bmState.segDifficulties[bmState.curSegI]==2)){ // High difficulty
        pdp_1=0.25; // 0.25
    }

    pdp_0=1-pdp_1;

    double rnd=RandomDouble(0,0.9999999999);
    int dp=-1;
    if(rnd<pdp_0){ // No collision
        dp=0;
    }
    if(rnd>=pdp_0 && rnd<pdp_0+pdp_1){ // Yes collision
        dp=1;
    }
    bmState.dp=dp;
    bmState.dps.push_back(dp); // In history

    bmState.p=bmState.p+dp;
    bmState.ps.push_back(bmState.p);

    // From velocity, length and collisions compute the time to traverse the subsegment
    double dt, dt1, dt2;
    dt1=(nVelocityValues-bmState.v)*subSegLengths[bmState.curSegI][bmState.curSubsegJ];
    bmState.dt1s.push_back(dt1);
    bmState.dt1=dt1;

    dt2=dp*collisionPenaltyTime;
    bmState.dt2s.push_back(dt2);
    bmState.dt2=dt2;

    dt=dt1+dt2;
    bmState.dts.push_back(dt);
    bmState.dt=dt;

    bmState.t=bmState.t+dt1+dt2;

    bmState.ts.push_back(bmState.t);

    double pm_0, pm_1; // 0: low number of times in which angular velocity used, 1: high number of times in which angular velocity used
    // Enrico rettangolo exp 2 autonomous robots (angular velocity)
    /*if(bmState.segDifficulties[bmState.curSegI]==0){ // Low difficulty
      pm_1=0.17; // 0.083
      }
      if(bmState.segDifficulties[bmState.curSegI]==1){ // Medium difficulty
      pm_1=0.24; // 0.23
      }
      if(bmState.segDifficulties[bmState.curSegI]==2){ // High difficulty
      pm_1=0.53; // 0.45
      }*/

    // Enrico ice exp 3 (angular velocity)
    if(bmState.segDifficulties[bmState.curSegI]==0){ // Low difficulty
        pm_1=0.083; // 0.083
    }
    if(bmState.segDifficulties[bmState.curSegI]==1){ // Medium difficulty
        pm_1=0.3; // 0.3
    }
    if(bmState.segDifficulties[bmState.curSegI]==2){ // High difficulty
        pm_1=0.3; // 0.3
    }

    pm_0=1-pm_1;
    rnd=RandomDouble(0,0.9999999999);
    int m=-1;
    if(rnd<pm_0){ // low number of times in which angular velocity used
        m=0;
    }
    else
        m=1;
    bmState.m=m; // In state <------------------------- 
    bmState.ms.push_back(m); // In history

    double po_0, po_1;

    // Model (occupancy) (Autonomous robots)
    if(bmState.segDifficulties[bmState.curSegI]==0){ // Low difficulty
        po_1=0.65; // 0.44
    }
    if(bmState.segDifficulties[bmState.curSegI]==1){ // Medium difficulty
        po_1=0.83; // 0.79
    }
    if(bmState.segDifficulties[bmState.curSegI]==2){ // High difficulty
        po_1=0.93; // 0.86
    }

    po_0=1-po_1;
    rnd=RandomDouble(0,0.9999999999);
    int o=-1;
    if(rnd<po_0){ // low number of times in which angular velocity used
        o=0;
    }
    else
        o=1;
    bmState.o=o; // In state <------------------------- 
    bmState.os.push_back(o); // In history

    observation=m + 2*o;

    // Reward
    reward=-dt;

    bmState.r=reward;
    bmState.rs.push_back(reward);
    bmState.r_cum=bmState.r_cum+reward;
    bmState.r_cums.push_back(bmState.r_cum);

    // Save current distance state-belief (only for data analysis)
    double dist=0; // Total distance real state <-> belief
    int hammingDist=0; // Hamming distance between single state and belief
    int res=0;

    if(beliefState.GetNumSamples()!=0){ // Not fake belief, used to identify simulation steps
        std::unordered_map<int, int> nParticlesPerState;
        int stateId;
        int stateTmp;
        for (int i = 0; i < beliefState.GetNumSamples(); i++){ // Compute map nParticlesPerState: state -> nParticles
            const STATE* state = beliefState.GetSample(i);
            const OBSTACLEAVOIDANCE_STATE& particleState = safe_cast<const OBSTACLEAVOIDANCE_STATE&>(*state);

            // Compute state id
            stateId=0;   
            for (int j = 0; j<nSeg; j++)  // For each segment difficulty
            {
                stateId+=particleState.segDifficulties[j]*(pow(nDifficultyValues,nSeg-j-1));
            }
            // If it is not in nParticlesPerState then add it and initialize to 1
            if (nParticlesPerState.find(stateId) == nParticlesPerState.end())
                nParticlesPerState[stateId]= 1;
            else
                nParticlesPerState[stateId]++;
        }
        // Compute distance as weighted sum of hamming distance between particle state and real state (where weight is belief probability)
        for (auto s = nParticlesPerState.begin(); s != nParticlesPerState.end(); ++s ){  // For each state in the belief
            stateTmp=s->first; // stateId
            hammingDist=0;
            for(int i = nSeg-1; i>=0; i--)// Compute Hamming distance between real state and particle s
            {
                res=stateTmp%nDifficultyValues;
                stateTmp=stateTmp/nDifficultyValues;
                hammingDist=hammingDist+std::abs(bmState.segDifficulties[i]-res); // Hamming distance between real state 
            }
            dist=dist + ((double)s->second/beliefState.GetNumSamples())*hammingDist;
        }
        bmState.dist_state_beliefs.push_back(dist);
    }

    if((bmState.curSegI==(nSeg-1)) && (bmState.curSubsegJ==(nSubSegs[bmState.curSegI])-1)) {
        return true; // True means terminal state
    }
    else{
        // Update position, to be ready for next step
        bmState.curSegI=nextCurSegI;
        bmState.curSubsegJ=nextCurSubsegJ;
        return false; // True means non-terminal state
    }
}

// TO BE IMPLEMENTED
bool OBSTACLEAVOIDANCE::LocalMove(STATE& state, const HISTORY& history,
        int stepObs, const STATUS& status) const
{
    return true;
}



// Puts in legal a set of legal actions that can be taken from state
void OBSTACLEAVOIDANCE::GenerateLegal(const STATE& state, const HISTORY& history,
        vector<int>& legal, const STATUS& status) const
{
    legal.push_back(0);
    legal.push_back(1);
    legal.push_back(2);
}

void OBSTACLEAVOIDANCE::GeneratePreferred(const STATE& state, const HISTORY& history,
        vector<int>& actions, const STATUS& status) const
{
    actions.push_back(0);
    actions.push_back(1);
    actions.push_back(2);
}

// Display methods -------------------------
void OBSTACLEAVOIDANCE::DisplayBeliefs(const BELIEF_STATE& beliefState,    // Displays all states in beliefState
        std::ostream& ostr) const
{
    cout << "OBSTACLEAVOIDANCE::DisplayBeliefs start" << endl;
    for (int i = 0; i < beliefState.GetNumSamples(); i++){
        const STATE* s = beliefState.GetSample(i);
        DisplayState(*s,cout);
    }
    cout << "OBSTACLEAVOIDANCE::DisplayBeliefs end" << endl;
}

void OBSTACLEAVOIDANCE::DisplayBeliefIds(const BELIEF_STATE& beliefState,    // Displays all states in beliefState
        std::ostream& ostr) const
{
    cout << "OBSTACLEAVOIDANCE::DisplayBeliefIds: [";
    for (int i = 0; i < beliefState.GetNumSamples(); i++){
        const STATE* s = beliefState.GetSample(i);
        DisplayStateId(*s, cout);cout <<"; ";
    }
    cout << "OBSTACLEAVOIDANCE::DisplayBeliefs end" << endl;
}

void OBSTACLEAVOIDANCE::DisplayBeliefDistribution(const BELIEF_STATE& beliefState,
        std::ostream& ostr) const
{
    std::unordered_map<int, int> dist;
    int id;
    for (int i = 0; i < beliefState.GetNumSamples(); i++){
        const STATE* state = beliefState.GetSample(i);
        const OBSTACLEAVOIDANCE_STATE& bmState = safe_cast<const OBSTACLEAVOIDANCE_STATE&>(*state);

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

void OBSTACLEAVOIDANCE::DisplayState(const STATE& state, std::ostream& ostr) const
{
    const OBSTACLEAVOIDANCE_STATE& bmState = safe_cast<const OBSTACLEAVOIDANCE_STATE&>(state);
    ostr << endl;

    // Display segment difficulties
    ostr << "## STATE ############" << endl;
    ostr << "Difficulties: ";
    for (int i = 0; i < nSeg; i++)
        ostr << i << ":" << bmState.segDifficulties[i] << ", ";
    ostr << endl;

    // Display agent's position
    ostr << "Position: i:" << bmState.curSegI << ", j:" << bmState.curSubsegJ << endl;

    // 1. Display delta battery voltage in last subsegment
    ostr << "dp: " << bmState.dp << endl;

    // 2. Display voltage
    ostr << "p: " << bmState.p << endl;

    // 3. Display Velocity in the last subsegment
    ostr << "v: " << bmState.v << endl;

    // 5a. Delta time due to normal navigation in last subsegment
    ostr << "dt1: " << bmState.dt1 << endl;

    // 5b. Delta time due to collisions in last subsegment
    ostr << "dt2: " << bmState.dt2 << endl;

    // 5. Delta time in last subsegment
    ostr << "dt: " << bmState.dt << endl;

    // 6. Display time
    ostr << "t: " << bmState.t << endl;

    // 7. Avg ang velocity
    ostr << "m: " << bmState.m << endl;

    // 8. Occupancy
    ostr << "o: " << bmState.o << endl;

    // 9. Reward
    ostr << "r: " << bmState.r << endl;

    // 10. Display cumulative reward
    ostr << "r_cum: " << bmState.r_cum << endl;
    ostr << "#######################" << endl<< endl;
}

void OBSTACLEAVOIDANCE::DisplayObservation(const STATE& state, int observation, std::ostream& ostr) const // Prints the observation
{
    switch (observation)
    {
        case 0:    // observation=E_NONE -> Nothing observed (action!=sampling)
            ostr << "Observed dp=0, dt1=1" << endl;
            break;
        case 1:    // observation=E_NONE -> Nothing observed (action!=sampling)
            ostr << "Observed dp=0, dt1=2" << endl;
            break;
        case 2:    // observation=E_NONE -> Nothing observed (action!=sampling)
            ostr << "Observed dp=0, dt1=3" << endl;
            break;
        case 3:    // observation=E_NONE -> Nothing observed (action!=sampling)
            ostr << "Observed dp=1, dt1=1" << endl;
            break;
        case 4:    // observation=E_NONE -> Nothing observed (action!=sampling)
            ostr << "Observed dp=1, dt1=2" << endl;
            break;
        case 5:    // observation=E_NONE -> Nothing observed (action!=sampling)
            ostr << "Observed dp=1, dt1=3" << endl;
            break;
    }
}

void OBSTACLEAVOIDANCE::DisplayAction(int action, std::ostream& ostr) const // Prints the action performed
{
    if (action == 0)  
        ostr << "Action: Low power (0)" << endl;
    if (action == 1) 
        ostr << "Action: Medium power (1)" << endl;
    if (action == 2) 
        ostr << "Action: High power (2)" << endl;
}


double OBSTACLEAVOIDANCE::JointProb(const STATE& state) const // Displays the STATE (grid with agent and rocks)
{
    // not implemented yet
    return 1.0;
}

void OBSTACLEAVOIDANCE::DisplayStateId(const STATE& state, std::ostream& ostr) const // Displays an id from rock values
{
    const OBSTACLEAVOIDANCE_STATE& bmState = safe_cast<const OBSTACLEAVOIDANCE_STATE&>(state);
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

void OBSTACLEAVOIDANCE::DisplayStateHist(const STATE& state, const char* fileName) const // Displays an id from rock values
{
    const OBSTACLEAVOIDANCE_STATE& bmState = safe_cast<const OBSTACLEAVOIDANCE_STATE&>(state);
    std::ofstream outFile;
    outFile.open(fileName, std::ofstream::out | std::ofstream::app);

    string s;
    s="Step";
    int step=0;
    for (int i = 0; i<nSeg; i++)  // For each segment
    {
        for (int j = 0; j<nSubSegs[i]; j++)  // For each subsegment
        {
            s=s + ", " + to_string(step);
            step=step+1;
        }
    }
    outFile << s << endl;

    s="Segments";
    for (int i = 0; i<nSeg; i++)  // For each segment
    {
        for (int j = 0; j<nSubSegs[i]; j++)  // For each subsegment
        {
            s=s + ", " + to_string(i);
        }
    }
    outFile << s << endl;

    s="SubSegments";
    for (int i = 0; i<nSeg; i++)  // For each segment
    {
        for (int j = 0; j<nSubSegs[i]; j++)  // For each subsegment
        {
            s=s + ", " + to_string(j);
        }
    }
    outFile << s << endl;

    s="Lengths";
    for (int i = 0; i<nSeg; i++)  // For each segment
    {
        for (int j = 0; j<nSubSegs[i]; j++)  // For each subsegment
        {
            s=s + ", " + to_string(subSegLengths[i][j]);
        }
    }
    outFile << s << endl;

    int stateId=0;   
    for (int i = 0; i<nSeg; i++)  // For each segment difficulty
    {
        stateId+=bmState.segDifficulties[i]*(pow(3,nSeg-i-1));
    }
    s="Difficulties ("+ to_string(stateId)+")";
    for (int i = 0; i<nSeg; i++)  // For each segment
    {
        for (int j = 0; j<nSubSegs[i]; j++)  // For each subsegment
        {
            s=s + ", " + to_string(bmState.segDifficulties[i]);
        }
    }
    outFile << s << endl;

    s="dps";
    for (int i = 0; i<bmState.dps.size(); i++)  // For each segment difficulty
    {
        s=s + ", " + to_string(bmState.dps[i]);
    }

    outFile << s << endl;

    // Battery voltage at the end of the last subsegment (2)
    s="p";
    for (int i = 0; i<bmState.ps.size(); i++)  // For each segment difficulty
    {
        s=s + ", " + to_string(bmState.ps[i]);
    }
    outFile << s << endl;

    // Velocity in the last subsegment (3)
    s="v";
    for (int i = 0; i<bmState.vs.size(); i++)  // For each segment difficulty
    {
        s=s + ", " + to_string(bmState.vs[i]);
    }
    outFile << s << endl;

    // Actions (4)
    s="acs";
    for (int i = 0; i<bmState.acs.size(); i++)  // For each segment difficulty
    {
        s=s + ", " + to_string(bmState.acs[i]);
    }
    outFile << s << endl;

    // Delta time in last segment (5a)
    s="dt1s";
    for (int i = 0; i<bmState.dt1s.size(); i++)  // For each segment difficulty
    {
        s=s + ", " + to_string(bmState.dt1s[i]);
    }
    outFile << s << endl;

    // Delta time in last segment (5b)
    s="dt2s";
    for (int i = 0; i<bmState.dt2s.size(); i++)  // For each segment difficulty
    {
        s=s + ", " + to_string(bmState.dt2s[i]);
    }
    outFile << s << endl;

    // Delta time in last segment (5)
    s="dts";
    for (int i = 0; i<bmState.dts.size(); i++)  // For each segment difficulty
    {
        s=s + ", " + to_string(bmState.dts[i]);
    }
    outFile << s << endl;

    // Time from the beginning (cumulative) (6)
    s="ts";
    for (int i = 0; i<bmState.ts.size(); i++)  // For each segment difficulty
    {
        s=s + ", " + to_string(bmState.ts[i]);
    }
    outFile << s << endl;

    // Average angular velocity (m)
    s="ms";
    for (int i = 0; i<bmState.ms.size(); i++)  // For each segment difficulty
    {
        s=s + ", " + to_string(bmState.ms[i]);
    }
    outFile << s << endl;

    // Occupancy (o)
    s="os";
    for (int i = 0; i<bmState.os.size(); i++)  // For each segment difficulty
    {
        s=s + ", " + to_string(bmState.os[i]);
    }
    outFile << s << endl;

    // Reward last subsegment (7)
    s="rs";
    for (int i = 0; i<bmState.rs.size(); i++)  // For each segment difficulty
    {
        s=s + ", " + to_string(bmState.rs[i]);
    }
    outFile << s << endl;

    // Cumulative reward (8)
    s="r_cums";
    for (int i = 0; i<bmState.r_cums.size(); i++)  // For each segment difficulty
    {
        s=s + ", " + to_string(bmState.r_cums[i]);
    }
    outFile << s << endl;

    // Cumulative reward (8)
    s="dist_state_bel";
    for (int i = 0; i<bmState.dist_state_beliefs.size(); i++)  // For each segment difficulty
    {
        s=s + ", " + to_string(bmState.dist_state_beliefs[i]);
    }
    outFile << s << endl<< endl;

    outFile.close();
}


vector<double*>* OBSTACLEAVOIDANCE::CreateStateRelKnowledge(const STATE& state, int nConnComp, double relProbab) const
{
    cout << "In CreateStateRelKnowledge" << endl;
    const OBSTACLEAVOIDANCE_STATE& oaState = safe_cast<const OBSTACLEAVOIDANCE_STATE&>(state);
    // Vector of knowledge
    vector<double*>* stateVarRelationships=new vector<double*>();
    // Vector of all possible edges ()
    vector<double*>* allStateVarRelationships=new vector<double*>();
    for (int i = 0; i<oaState.segDifficulties.size()-1; i++)
    {
        for (int j = i+1; j<oaState.segDifficulties.size(); j++)
        {
            if(oaState.segDifficulties[i]==oaState.segDifficulties[j]){
                allStateVarRelationships->push_back(new double[3]{(double)i, (double)j,relProbab});
            }
        }
    }
    // Check how many connected components there are

    int nTrueConnComp=oaState.segDifficulties.size(); // N edge in stateVarRelationships
    int edgeI=-1;
    vector<vector<int>*>* connComps;
    while(nTrueConnComp>nConnComp){
        edgeI=Random(allStateVarRelationships->size());// Get random edge 
        // If the edge is not in the knowledge
        stateVarRelationships->push_back((*allStateVarRelationships)[edgeI]); // Add the edge to the knowledge
        allStateVarRelationships->erase(allStateVarRelationships->begin()+edgeI);
        // Check how many connected components there are
        connComps=computeConnComp(stateVarRelationships,oaState.segDifficulties.size());
        nTrueConnComp=connComps->size();
    }
    cout << "StateRelKnowledge computed. Connected components:" << endl;
    printConnectedComponents(connComps);
    return stateVarRelationships;
}


bool OBSTACLEAVOIDANCE::LocalMove(STATE& state, const HISTORY& history,
        int stepObs, const STATUS& status, std::vector<double*>* stateVarRelationships) const
{
    // TO BE IMPLEMENTED
    return true;
}

void OBSTACLEAVOIDANCE::PropagateChange(STATE& state, int changedVariable,  
        std::vector<double*>* stateVarRelationships,  vector<int>* alreadyExploredVarIndices) const
{
    // TO BE IMPLEMENTED
}

// Notice: in case of prob=1 changes could be simply propagated in all the connected component
void OBSTACLEAVOIDANCE::PropagateChangeToConnectedComponent(STATE& state, int changedVariable, int newVal, 
        std::vector<double*>* stateVarRelationships) const
{
    // TO BE IMPLEMENTED
}

vector<double*> OBSTACLEAVOIDANCE::FindUnexploredAdjacentVariableIndices(int currentVarIndex, 
        std::vector<double*>* stateVarRelationships, vector<int>* alreadyExploredVarIndices) const
{
    int nRels=stateVarRelationships->size();
    vector<double*> adjIs;
    // TO BE IMPLEMENTED
    return adjIs;
}

double OBSTACLEAVOIDANCE::ComputeParticleProbability(STATE& particle, std::vector<double*>* stateVarRelationships) const {
    OBSTACLEAVOIDANCE_STATE& oastate = safe_cast<OBSTACLEAVOIDANCE_STATE&>(particle);  // State   
    double probab=1;

    int var0, var1, seg0Val, seg1Val;

    double potentialEqualValues, potentialDifferentValues, potential;
    for(std::vector<double*>::iterator it = stateVarRelationships->begin() ; it != stateVarRelationships->end(); ++it){
        var0=(*it)[0]; // First segment
        var1=(*it)[1]; // Second segment
        potentialEqualValues=(*it)[2]/nDifficultyValues;
        potentialDifferentValues=(1.0-(*it)[2])/6;
        // Apply the constraint to the state and compute the related potential
        seg0Val=oastate.segDifficulties[var0];
        seg1Val=oastate.segDifficulties[var1];
        if(seg0Val==seg1Val){   // They are equal
            potential=potentialEqualValues;
        }
        else{   // They are different
            potential=potentialDifferentValues;
        }
        probab=probab*potential;
    }
    return probab;
}
