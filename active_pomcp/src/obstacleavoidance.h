#ifndef OBSTACLEAVOIDANCE_H
#define OBSTACLEAVOIDANCE_H

#include "simulator.h"
#include <unordered_map>
#include <vector>
#include <algorithm>
#include <fstream>
#include <array>
#include "hellinger.h"

class OBSTACLEAVOIDANCE_STATE : public STATE
{
    public:

        std::vector<int> segDifficulties;
        int curSegI;
        int curSubsegJ;
        int dp;
        int p;
        int v;
        double dt;
        double dt1;
        double dt2;
        double t;
        int o;
        double r;
        double r_total;

        std::vector<int> dps;
        std::vector<int> ps;
        std::vector<int> vs;
        std::vector<int> acs;
        std::vector<double> dts;
        std::vector<double> dt1s;
        std::vector<double> dt2s;
        std::vector<double> ts;
        std::vector<int> os;
        std::vector<double> rs;
        std::vector<double> r_totals;
        std::vector<double> dist_state_beliefs;
};

class OBSTACLEAVOIDANCE_METAINFO : public BELIEF_META_INFO
{
public:
    OBSTACLEAVOIDANCE_METAINFO() :
    distr() {
        for (auto &i : distr)
            for (auto &j : i)
                j = 0;
    }

    virtual void update(STATE *s) {
        const OBSTACLEAVOIDANCE_STATE& state = safe_cast<const OBSTACLEAVOIDANCE_STATE&>(*s);
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
        return new OBSTACLEAVOIDANCE_METAINFO(*this);
    }


private:

    std::array<std::array<int, 3>, 8> distr;
    int total = 0;
};

class OBSTACLEAVOIDANCE : public SIMULATOR
{
    public:

        OBSTACLEAVOIDANCE(std::vector<int> nSubSegs, std::vector<std::vector<double>> subSegLengths, int nEnginePowerValues, int nDifficultyValues, int nVelocityValues);

        OBSTACLEAVOIDANCE(std::vector<int> nSubSegs, std::vector<std::vector<double>> subSegLengths, int nEnginePowerValues, int nDifficultyValues, int nVelocityValues, std::string shield_file);

        virtual STATE* Copy(const STATE& state) const;
        virtual void Validate(const STATE& state) const;
        virtual STATE* CreateStartState() const;
        virtual void FreeState(STATE* state) const;
        virtual bool Step(STATE& state, int action, 
                observation_t& observation, double& reward) const;
        virtual bool Step(const VNODE *const mcts_root, STATE &state, int action, 
            observation_t& observation, double& reward, const BOUNDS &s) const;

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
        virtual double JointProb(const STATE& state) const;
        void DisplayStateId(const STATE& state, std::ostream& ostr) const;
        void DisplayStateHist(const STATE& state, const char* fileName) const;
        virtual void DisplayBeliefIds(const BELIEF_STATE& beliefState,
                std::ostream& ostr) const;
        virtual void DisplayBeliefDistribution(const BELIEF_STATE& beliefState,
                std::ostream& ostr) const;
        STATE* CreateStartState(std::vector<double*>* stateVarRelationships) const;
        STATE* CreateStartState(std::vector<STATE*>* allParticles, std::vector<double> allParticleCumProbs) const;
        STATE* CreateStartStateFixedValues(std::vector<int> values) const;
        virtual bool LocalMove(STATE& state, const HISTORY& history,
                int stepObservation, const STATUS& status, std::vector<double*>* stateVarRelationships) const;

        virtual double ComputeParticleProbability(STATE& particle, std::vector<double*>* stateVarRelationships) const;

    virtual void log_problem_info() const;
    virtual void log_beliefs(const BELIEF_STATE& beliefState) const;
    virtual void log_state(const STATE& state) const;
    virtual void log_action(int action) const;
    virtual void log_observation(const STATE& state, observation_t observation) const;
    virtual void log_reward(double reward) const;

    virtual void set_belief_metainfo(VNODE *v, const SIMULATOR &s) const;
    virtual void FreeMetainfo(BELIEF_META_INFO *m) const {
        OBSTACLEAVOIDANCE_METAINFO *tm = safe_cast<OBSTACLEAVOIDANCE_METAINFO*>(m);
        delete m;
    }

    virtual void pre_shield(const BELIEF_STATE &belief, std::vector<int> &legal_actions) const;

    void set_visual(std::vector<std::vector<std::pair<double,double>>> v) {
        visual = v;
    }

    protected:
        int nSeg;
        std::vector<int> nSubSegs;
        std::vector<std::vector<double>> subSegLengths;
        int nDifficultyValues;
        int nVelocityValues;
        int collisionPenaltyTime;

    private:
        mutable MEMORY_POOL<OBSTACLEAVOIDANCE_STATE> MemoryPool;

        void set_observation(SIMULATOR::observation_t &obs, OBSTACLEAVOIDANCE_STATE &s) const;
        void reset_observation(SIMULATOR::observation_t &obs,
                               SIMULATOR::observation_t value,
                               OBSTACLEAVOIDANCE_STATE &s) const;
        hellinger_shield<3> speed_2_points;
        double shield_x1, shield_x2, shield_x3, shield_x4;
        mutable std::uniform_real_distribution<> unif_dist;
        std::vector<std::vector<std::pair<double,double>>> visual;
};

#endif
