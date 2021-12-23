#pragma once

#include "simulator.h"
#include <unordered_map>
#include <vector>
#include <algorithm>
#include <fstream>
#include <array>
#include "hellinger.h"

class BATTERY_VELOCITY_STATE : public STATE
{
    public:
        std::vector<int> segDifficulties;
        int curSegI;
        int curSubsegJ;
        int battery_level;
};

class BATTERY_VELOCITY_METAINFO : public BELIEF_META_INFO
{
    // TODO
public:
    BATTERY_VELOCITY_METAINFO() :
    distr() {
        for (auto &i : distr)
            for (auto &j : i)
                j = 0;
    }

    virtual void update(STATE *s) {
        const BATTERY_VELOCITY_STATE& state = safe_cast<const BATTERY_VELOCITY_STATE&>(*s);
        total++;
        for (int i = 0; i < 8; i++) {  // For each segment difficulty
            distr[i][state.segDifficulties[i]] += 1;
        }
    }

    virtual void clear() {
        total = 0;
        for (auto &i : distr)
            for (auto &j : i)
                j = 0;
    }

    int get_total() const { return total; }
    double get_prob_diff(int segment, int diff) const { 
        return static_cast<double>(distr[segment][diff]) / total;
    }

    virtual BELIEF_META_INFO *clone() const {
        return new BATTERY_VELOCITY_METAINFO(*this);
    }


private:

    std::array<std::array<int, 3>, 8> distr;
    int total = 0;
};

enum {
    ACTION_RECHARGE = 4
};

enum {
    OBS_NONE = 0,
    OBS_CLEAR = 1,
    OBS_CLUTTERED = 2
};

class BATTERY_VELOCITY : public SIMULATOR
{
    public:

        BATTERY_VELOCITY(std::vector<int> nSubSegs, std::vector<std::vector<double>> subSegLengths, int nEnginePowerValues, int nDifficultyValues, int nVelocityValues);

        virtual STATE* Copy(const STATE& state) const;
        virtual void Validate(const STATE& state) const;
        virtual STATE* CreateStartState() const;
        virtual void FreeState(STATE* state) const;
        virtual bool Step(STATE& state, int action, 
                observation_t& observation, double& reward) const;

        void GenerateLegal(const STATE& state, const HISTORY& history,
                std::vector<int>& legal, const STATUS& status) const;
        void GeneratePreferred(const STATE& state, const HISTORY& history,
                std::vector<int>& legal, const STATUS& status) const;
        virtual bool LocalMove(STATE& state, const HISTORY& history,
                int stepObservation, const STATUS& status) const;

        virtual void DisplayBeliefs(const BELIEF_STATE& beliefState,
                std::ostream& ostr) const;
        virtual void DisplayState(const STATE& state, std::ostream& ostr) const;
        virtual void DisplayObservation(const STATE& state, observation_t observation, std::ostream& ostr) const;
        virtual void DisplayAction(int action, std::ostream& ostr) const;
        void DisplayStateId(const STATE& state, std::ostream& ostr) const;
        virtual void DisplayBeliefIds(const BELIEF_STATE& beliefState,
                std::ostream& ostr) const;
        virtual void DisplayBeliefDistribution(const BELIEF_STATE& beliefState,
                std::ostream& ostr) const;
    virtual void log_problem_info() const;
    virtual void log_beliefs(const BELIEF_STATE& beliefState) const;
    virtual void log_state(const STATE& state) const;
    virtual void log_action(int action) const;
    virtual void log_observation(const STATE& state, observation_t observation) const;
    virtual void log_reward(double reward) const;

    virtual void set_belief_metainfo(VNODE *v, const SIMULATOR &s) const;
    virtual void FreeMetainfo(BELIEF_META_INFO *m) const {
        BATTERY_VELOCITY_METAINFO *tm = safe_cast<BATTERY_VELOCITY_METAINFO*>(m);
        delete m;
    }

    virtual void pre_shield(const BELIEF_STATE &belief, std::vector<int> &legal_actions) const {
        assert(false); // TODO
    }

    protected:
        int nSeg;
        std::vector<int> nSubSegs;
        std::vector<std::vector<double>> subSegLengths;
        int nDifficultyValues;
        int nVelocityValues;
        int collisionPenaltyTime;
        static constexpr int BATTERY_MAX = 12;

    private:
        mutable MEMORY_POOL<BATTERY_VELOCITY_STATE> MemoryPool;

        void set_observation(SIMULATOR::observation_t &obs, BATTERY_VELOCITY_STATE &s) const;
        void reset_observation(SIMULATOR::observation_t &obs,
                               SIMULATOR::observation_t value,
                               BATTERY_VELOCITY_STATE &s) const;

        mutable std::uniform_real_distribution<> unif_dist;
};

