#ifndef TIGER_H
#define TIGER_H

#include "simulator.h"
#include <unordered_map>
#include <vector>
#include <random>
#include <algorithm>
#include "hellinger.h"

class TIGER_STATE : public STATE
{
public:
    bool tiger_on_left;
    std::vector<int> saved_actions;
};

class TIGER_METAINFO : public BELIEF_META_INFO
{
public:
    virtual void update(STATE *s) {
        auto tiger_state = safe_cast<TIGER_STATE *>(s);
        total++;
        if (tiger_state->tiger_on_left)
            on_left++;
    }

    virtual void clear() {
        total = 0;
        on_left = 0;
    }

    int get_on_left() const { return on_left; }
    int get_total() const { return total; }
    double perc_on_left() const { 
        if (total == 0)
            return 0.0;
        else
            return static_cast<double>(on_left) / total;
    }

    virtual std::unique_ptr<BELIEF_META_INFO> clone() const {
        return std::make_unique<TIGER_METAINFO>(*this);
    }

private:
    int on_left = 0;
    int total = 0;
};

class TIGER : public SIMULATOR
{
public:
    TIGER();
    TIGER(const std::string &shield_file);

    virtual STATE* Copy(const STATE& state) const;
    virtual void Validate(const STATE& state) const;
    virtual STATE* CreateStartState() const;
    virtual void FreeState(STATE* state) const;
    virtual bool Step(STATE& state, int action, int& observation,
                      double& reward) const;
    void GenerateLegal(const STATE& state, const HISTORY& history,
        std::vector<int>& legal, const STATUS& status) const;
    void GeneratePreferred(const STATE& state, const HISTORY& history,
        std::vector<int>& legal, const STATUS& status) const;
    virtual bool LocalMove(STATE& state, const HISTORY& history,
        int stepObservation, const STATUS& status) const;

    virtual void DisplayBeliefs(const BELIEF_STATE& beliefState,
        std::ostream& ostr) const;
    virtual void DisplayState(const STATE& state, std::ostream& ostr) const;
    virtual void DisplayObservation(const STATE& state, int observation,
                                    std::ostream& ostr) const;
    virtual void DisplayAction(int action, std::ostream& ostr) const;

    virtual void log_problem_info(xes_logger &xes) const;
    virtual void log_beliefs(const BELIEF_STATE& beliefState,
            xes_logger &xes) const;
    virtual void log_state(const STATE& state, xes_logger &xes) const;
    virtual void log_action(int action, xes_logger &xes) const;
    virtual void log_observation(const STATE& state, int observation,
            xes_logger &xes) const;
    virtual void log_reward(double reward, xes_logger &xes) const;
    
    virtual void set_belief_metainfo(VNODE *v) const;

    // shielding
    virtual void pre_shield(const BELIEF_STATE &belief, std::vector<int> &legal_actions) const;
    
protected:
    std::vector<int> saved_actions;

    enum
    {
        A_LISTEN,
        A_LEFT_DOOR,
        A_RIGHT_DOOR,
    };

    enum
    {
        O_LEFT_ROAR,
        O_RIGHT_ROAR,
        O_TREASURE,
        O_TIGER
    };

private:
    mutable MEMORY_POOL<TIGER_STATE> MemoryPool;

    // shield
    double tr_open, tr_listen;
    hellinger_shield<2> open_left_points;
    hellinger_shield<2> open_right_points;
    hellinger_shield<2> listen_points;
};

#endif // TIGER_H
