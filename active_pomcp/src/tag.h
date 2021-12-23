#ifndef TAG_H
#define TAG_H

#include "simulator.h"
#include "coord.h"
#include "grid.h"

class TAG_STATE : public STATE
{
public:

    COORD AgentPos;
    std::vector<COORD> OpponentPos;
    int NumAlive;
};

class TAG : public SIMULATOR
{
public:

    TAG(int numrobots);

    virtual STATE* Copy(const STATE& state) const;
    virtual void Validate(const STATE& state) const;
    virtual STATE* CreateStartState() const;
    virtual void FreeState(STATE* state) const;
    virtual bool Step(STATE& state, int action, 
        observation_t& observation, double& reward) const;
        
    void GeneratePreferred(const STATE& state, const HISTORY& history,
        std::vector<int>& legal, const STATUS& status) const;
    virtual bool LocalMove(STATE& state, const HISTORY& history,
        observation_t stepObs, const STATUS& status) const;

    virtual void DisplayBeliefs(const BELIEF_STATE& beliefState, 
        std::ostream& ostr) const;
    virtual void DisplayState(const STATE& state, std::ostream& ostr) const;
    virtual void DisplayObservation(const STATE& state, observation_t observation, std::ostream& ostr) const;
    virtual void DisplayAction(int action, std::ostream& ostr) const;

    // xes logging
    virtual void log_problem_info() const;
    virtual void log_beliefs(const BELIEF_STATE& beliefState) const;
    virtual void log_state(const STATE& state) const;
    virtual void log_action(int action) const;
    virtual void log_observation(const STATE& state, observation_t observation) const;
    virtual void log_reward(double reward) const;

protected:

    void MoveOpponent(TAG_STATE& tagstate, int opp) const;
    observation_t GetObservation(const TAG_STATE& tagstate, int action) const;
    bool Inside(const COORD& coord) const;
    COORD GetCoord(int index) const;
    int GetIndex(const COORD& coord) const;
    bool IsAlive(const TAG_STATE& tagstate, int opp) const;
    bool IsCorner(const COORD& coord) const;
    COORD GetRandomCorner() const;

    int NumOpponents;
    static const int NumCells;
    
private:

    mutable MEMORY_POOL<TAG_STATE> MemoryPool;
};

#endif
