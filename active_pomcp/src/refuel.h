#pragma once

#include "simulator.h"
#include "coord.h"
#include "grid.h"
#include "json.hpp"
#include <string>

struct REFUEL_STATE : public STATE
{
    COORD AgentPos;
    int fuel;
};

/*
struct REFUEL_OBS {
    bool can_refuel; // aat station and fuel < cap
    bool fuel_empty;
    bool north_enabled;
    bool south_enabled;
    bool east_enabled;
    bool west_enabled;

    REFUEL_OBS() = default;
    REFUEL_OBS(int o) { from_int(o); }

    int to_int() {
        return can_refuel
            + (fuel_empty << 1)
            + (north_enabled << 2)
            + (south_enabled << 3)
            + (east_enabled << 4)
            + (west_enabled << 5);
    }

    void from_int(int o) {
        can_refuel    = o % 2; o /= 2;
        fuel_empty    = o % 2; o /= 2;
        north_enabled = o % 2; o /= 2;
        south_enabled = o % 2; o /= 2;
        east_enabled  = o % 2; o /= 2;
        west_enabled  = o % 2;
    }
};
*/
struct REFUEL_OBS {
    bool can_refuel; // aat station and fuel < cap
    bool fuel_empty;

    REFUEL_OBS() = default;
    REFUEL_OBS(int o) { from_int(o); }

    uint64_t to_int() {
        uint64_t res = 0;
        res = can_refuel ? 1 : 0;
        res += fuel_empty ? 2 : 0 ;
        return res;
    }

    // magari esplorare più cose possiili senza limiti
    void from_int(int o) {
        can_refuel = o % 2; o /= 2;
        fuel_empty = o % 2;
    }
};

//static
//void to_json(nlohmann::json& j, const ROCKSAMPLE_STATE::ENTRY& p) {
//    j = nlohmann::json{
//        {"Valuable", p.Valuable},
//        {"Collected", p.Collected},
//        {"Count", p.Count},
//        {"Measured", p.Measured},
//        {"LikelihoodValuable", p.LikelihoodValuable},
//        {"LikelihoodWorthless", p.LikelihoodWorthless},
//        {"ProbValuable", p.ProbValuable},
//    };
//}
//
//static
//void from_json(const nlohmann::json &j, ROCKSAMPLE_STATE::ENTRY &p) {
//    j.at("Valuable").get_to(p.Valuable);
//    j.at("Collected").get_to(p.Collected);
//    j.at("Count").get_to(p.Count);
//    j.at("Measured").get_to(p.Measured);
//    j.at("LikelihoodValuable").get_to(p.LikelihoodValuable);
//    j.at("LikelihoodWorthless").get_to(p.LikelihoodWorthless);
//    j.at("ProbValuable").get_to(p.ProbValuable);
//}
//
//static
//void to_json(nlohmann::json& j, const ROCKSAMPLE_STATE& p) {
//    j = nlohmann::json{{"Rocks", p.Rocks}, {"Target", p.Target}};
//}
//
//static
//void from_json(const nlohmann::json& j, ROCKSAMPLE_STATE& p) {
//    j.at("Rocks").get_to(p.Rocks);
//    j.at("Target").get_to(p.Target);
//}

class REFUEL_METAINFO : public BELIEF_META_INFO
{
    /* TODO
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
    */
};



class REFUEL : public SIMULATOR
{
public:

    REFUEL(int size, int energy);

    virtual bool Step(STATE& state, int action,
        observation_t& observation, double& reward) const;

    virtual STATE* Copy(const STATE& state) const;
    virtual void Validate(const STATE& state) const;
    virtual STATE* CreateStartState() const;
    virtual void FreeState(STATE* state) const;
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
    /*
    virtual void log_problem_info() const;
    virtual void log_beliefs(const BELIEF_STATE& beliefState) const;
    virtual void log_state(const STATE& state) const;
    virtual void log_action(int action) const;
    virtual void log_observation(const STATE& state, observation_t observation) const;
    virtual void log_reward(double reward) const;

    virtual void set_belief_metainfo(VNODE *v, const SIMULATOR &s) const;
    virtual void FreeMetainfo(BELIEF_META_INFO *m) const {
        REFUEL_METAINFO *tm = safe_cast<REFUEL_METAINFO*>(m);
        delete m;
    }
    virtual void pre_shield(const BELIEF_STATE &belief, std::vector<int> &legal_actions) const;

    virtual Classification check_rule(const BELIEF_META_INFO &m, int a, double t) const;
*/

protected:
    enum
    {
        E_REFUEL = 4
    };

    static constexpr int resolution = 2;
    static constexpr double slipery = 0.3;

    GRID<int> Grid;
    int size, energy;

    std::vector<COORD> stations;
    std::vector<COORD> obstacles;
    COORD start, target;

    bool can_refuel(const REFUEL_STATE &s) const {
        return s.fuel < energy && find(stations.begin(), stations.end(),
                                       s.AgentPos) != stations.end();
    }

private:
    mutable MEMORY_POOL<REFUEL_STATE> MemoryPool;
};

