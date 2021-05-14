#ifndef ROCKSAMPLE_H
#define ROCKSAMPLE_H

#include "simulator.h"
#include "coord.h"
#include "grid.h"

class ROCKSAMPLE_STATE : public STATE
{
public:

    COORD AgentPos;
    struct ENTRY
    {
        bool Valuable;
        bool Collected;
        int Count;    				// Smart knowledge
        int Measured; 				// Smart knowledge
        double LikelihoodValuable;	// Smart knowledge
        double LikelihoodWorthless;	// Smart knowledge
        double ProbValuable;		// Smart knowledge
    };
    std::vector<ENTRY> Rocks;
    int Target; // Smart knowledge
};

class ROCKSAMPLE : public SIMULATOR
{
public:

    ROCKSAMPLE(int size, int rocks);

    virtual STATE* Copy(const STATE& state) const;
    virtual void Validate(const STATE& state) const;
    virtual STATE* CreateStartState() const;
    virtual void FreeState(STATE* state) const;
    virtual bool Step(STATE& state, int action,
        int& observation, double& reward) const;

    void GenerateLegal(const STATE& state, const HISTORY& history,
        std::vector<int>& legal, const STATUS& status) const;
    void GeneratePreferred(const STATE& state, const HISTORY& history,
        std::vector<int>& legal, const STATUS& status) const;
    virtual bool LocalMove(STATE& state, const HISTORY& history,
        int stepObservation, const STATUS& status) const;

    virtual void DisplayBeliefs(const BELIEF_STATE& beliefState,
        std::ostream& ostr) const;
    virtual void DisplayState(const STATE& state, std::ostream& ostr) const;
    virtual void DisplayObservation(const STATE& state, int observation, std::ostream& ostr) const;
    virtual void DisplayAction(int action, std::ostream& ostr) const;

    // xes logging
    virtual void log_problem_info(xes_logger &/*xes*/) const;
    virtual void log_beliefs(const BELIEF_STATE& beliefState,
            xes_logger &/*xes*/) const;
    virtual void log_state(const STATE& state, xes_logger &/*xes*/) const;
    virtual void log_action(int action, xes_logger &/*xes*/) const;
    virtual void log_observation(const STATE& state, int observation,
            xes_logger &/*xes*/) const;
    virtual void log_reward(double reward, xes_logger &/*xes*/) const;

    // shielding
    //virtual int shield_action(const BELIEF_STATE &belief, int action) const {
    //    return action;
    //}
    virtual void pre_shield(const BELIEF_STATE &belief, std::vector<int> &legal_actions) const;
    //virtual void set_belief_metainfo(VNODE *v) const {
    //    v->Beliefs() = BELIEF_META_INFO();
    //}
protected:

    enum
    {
        E_NONE,
        E_GOOD,
        E_BAD
    };

    enum
    {
        E_SAMPLE = 4
    };

    void InitGeneral();
    void Init_4_1();
    void Init_7_8();
    void Init_11_11();
    int GetObservation(const ROCKSAMPLE_STATE& rockstate, int rock) const;
    int SelectTarget(const ROCKSAMPLE_STATE& rockstate) const;

    GRID<int> Grid;
    std::vector<COORD> RockPos;
    int Size, NumRocks;
    COORD StartPos;
    double HalfEfficiencyDistance;
    double SmartMoveProb;
    int UncertaintyCount;

private:

    mutable MEMORY_POOL<ROCKSAMPLE_STATE> MemoryPool;
};

#endif
