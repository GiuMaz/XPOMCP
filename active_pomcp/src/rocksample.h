#ifndef ROCKSAMPLE_H
#define ROCKSAMPLE_H

#include "simulator.h"
#include "coord.h"
#include "grid.h"
#include "hellinger.h"
#include "json.hpp"
#include <string>

struct ROCKSAMPLE_STATE : public STATE
{
    COORD AgentPos;
    struct ENTRY
    {
        bool Valuable;
        bool Collected;
        // Smart knowledge
        int Count;
        int Measured;
        double LikelihoodValuable;
        double LikelihoodWorthless;
        double ProbValuable;
    };

    std::vector<ENTRY> Rocks;
    // Smart knowledge
    int Target;

};

static
void to_json(nlohmann::json& j, const ROCKSAMPLE_STATE::ENTRY& p) {
    j = nlohmann::json{
        {"Valuable", p.Valuable},
        {"Collected", p.Collected},
        {"Count", p.Count},
        {"Measured", p.Measured},
        {"LikelihoodValuable", p.LikelihoodValuable},
        {"LikelihoodWorthless", p.LikelihoodWorthless},
        {"ProbValuable", p.ProbValuable},
    };
}

static
void from_json(const nlohmann::json &j, ROCKSAMPLE_STATE::ENTRY &p) {
    j.at("Valuable").get_to(p.Valuable);
    j.at("Collected").get_to(p.Collected);
    j.at("Count").get_to(p.Count);
    j.at("Measured").get_to(p.Measured);
    j.at("LikelihoodValuable").get_to(p.LikelihoodValuable);
    j.at("LikelihoodWorthless").get_to(p.LikelihoodWorthless);
    j.at("ProbValuable").get_to(p.ProbValuable);
}

static
void to_json(nlohmann::json& j, const ROCKSAMPLE_STATE& p) {
    j = nlohmann::json{{"Rocks", p.Rocks}, {"Target", p.Target}};
}

static
void from_json(const nlohmann::json& j, ROCKSAMPLE_STATE& p) {
    j.at("Rocks").get_to(p.Rocks);
    j.at("Target").get_to(p.Target);
}

class ROCKSAMPLE_METAINFO : public BELIEF_META_INFO
{
public:
    ROCKSAMPLE_METAINFO(int NumRocks): distr(NumRocks, 0), collected_() {}

    virtual void update(STATE *s) {
        const ROCKSAMPLE_STATE& state = safe_cast<const ROCKSAMPLE_STATE&>(*s);
        ++total;

        for (size_t i = 0; i < state.Rocks.size(); i++) {
            if (state.Rocks[i].Valuable)
                distr[i] += 1;
        }
        x_ = state.AgentPos.X;
        y_ = state.AgentPos.Y;

        if (collected_.empty()) {
            for (size_t i = 0; i < state.Rocks.size(); i++) {
                collected_.push_back(state.Rocks[i].Collected? 1 : 0);
            }
        }
    }

    virtual void clear() {
        total = 0;
        x_ = 0;
        y_ = 0;
        for (auto &i : distr)
            i = 0;
        collected_.clear();
    }

    int get_total() const { return total; }
    double get_prob_valuable(int rock) const { 
        return static_cast<double>(distr[rock]) / total;
    }

    virtual BELIEF_META_INFO *clone() const {
        return new ROCKSAMPLE_METAINFO(*this);
    }

    const std::vector<int> &collected() const { return collected_; }
    bool collected(int rock) const { return collected_[rock] == 1; }
    int x() const { return x_; }
    int y() const { return y_; }

private:
    std::vector<int> distr;
    std::vector<int> collected_;
    int total = 0;
    int x_ = 0, y_ = 0;
};



class ROCKSAMPLE : public SIMULATOR
{
public:

    ROCKSAMPLE(int size, int rocks);
    ROCKSAMPLE(int size, int rocks, std::string file_name);
    ROCKSAMPLE(int size, int rocks, int x, int y,
               const std::vector<double> &belief);

    virtual STATE* Copy(const STATE& state) const;
    virtual void Validate(const STATE& state) const;
    virtual STATE* CreateStartState() const;
    virtual void FreeState(STATE* state) const;
    virtual bool Step(STATE& state, int action,
        observation_t& observation, double& reward) const;
    virtual bool Step(const VNODE *const mcts_root, STATE &state, int action, 
        observation_t& observation, double& reward, double min, double max) const;

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

    // xes logging
    virtual void log_problem_info() const;
    virtual void log_beliefs(const BELIEF_STATE& beliefState) const;
    virtual void log_state(const STATE& state) const;
    virtual void log_action(int action) const;
    virtual void log_observation(const STATE& state, observation_t observation) const;
    virtual void log_reward(double reward) const;

    virtual void set_belief_metainfo(VNODE *v, const SIMULATOR &s) const;
    virtual void FreeMetainfo(BELIEF_META_INFO *m) const {
        ROCKSAMPLE_METAINFO *tm = safe_cast<ROCKSAMPLE_METAINFO*>(m);
        delete m;
    }
    virtual void pre_shield(const BELIEF_STATE &belief, std::vector<int> &legal_actions) const;

    virtual Classification check_rule(const BELIEF_META_INFO &m, int a, double t) const;

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
    void Init_15_15();
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

    hellinger_shield<1> sampling_points;
    double sample_shield_tr;
    mutable std::uniform_real_distribution<> unif_dist;
    bool has_fixed_belief;
    std::vector<double> fixed_belief;
};

#endif
