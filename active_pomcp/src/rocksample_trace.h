#pragma once
#include "json.hpp"
#include "rocksample.h"
#include "trace.h"

class ROCKSAMPLE_STEP_INFO {
};

// only store metadata
class ROCKSAMPLE_STEP {
public:
    using step_info_t = ROCKSAMPLE_STEP_INFO;

    void save_belief(const BELIEF_STATE &b) {
        const auto &info =
            safe_cast<const ROCKSAMPLE_METAINFO &>(b.get_metainfo());

        const auto *state =
            safe_cast<const ROCKSAMPLE_STATE *>(b.GetSample(0));

        for (int i = 0; i < state->Rocks.size(); i++)
            rock_probs_.push_back(info.get_prob_valuable(i));

        x_ = state->AgentPos.X;
        y_ = state->AgentPos.Y;
    }

    void build_from_json(std::vector<double> r, int x, int y) {
        rock_probs_ = r;
        x_ = x;
        y_ = y;
    }

    void save_action(int a) {
        action_ = a;
    }

    void save_action_rank(const VNODE * const a) {
        action_rank_.clear();
        for (int i = 0; i < a->NumChildren; i++) {
            action_rank_.push_back({a->Child(i).Value.GetValue()});
        }
    }

    void save_action_rank(std::vector<double> r) {
        action_rank_ = r;
    }

    int x() const { return x_; }
    int y() const { return y_; }
    int action() const { return action_; }
    const std::vector<double> &rocks() const { return rock_probs_; }
    const std::vector<double> &action_rank() const { return action_rank_; }

    friend void to_json(nlohmann::json& j, const ROCKSAMPLE_STEP& p);
    friend void from_json(const nlohmann::json &j, ROCKSAMPLE_STEP &p);
  private:
    // metainfo
    std::vector<double> rock_probs_;
    std::vector<double> action_rank_;
    int x_, y_, action_;
};
void to_json(nlohmann::json& j, const ROCKSAMPLE_STEP& p);
void from_json(const nlohmann::json &j, ROCKSAMPLE_STEP &p);

// store the complete particle distribution
class ROCKSAMPLE_COMPLETE_STEP {
public:
    using step_info_t = ROCKSAMPLE_STEP_INFO;

    void save_belief(const BELIEF_STATE &b) {
        const auto &info =
            safe_cast<const ROCKSAMPLE_METAINFO &>(b.get_metainfo());

        const auto *state =
            safe_cast<const ROCKSAMPLE_STATE *>(b.GetSample(0));

        for (int i = 0; i < state->Rocks.size(); i++)
            rock_probs_.push_back(info.get_prob_valuable(i));

        x_ = state->AgentPos.X;
        y_ = state->AgentPos.Y;

        for (int i = 0; i < b.GetNumSamples(); i++) {
            const auto *state =
                safe_cast<const ROCKSAMPLE_STATE *>(b.GetSample(i));
            particles_.emplace_back(*state);
        }
    }

    void build_from_json(std::vector<double> r, int x, int y) {
        rock_probs_ = r;
        x_ = x;
        y_ = y;
    }

    void save_action(int a) {
        action_ = a;
    }

    void save_action_rank(const VNODE * const a) {
        action_rank_.clear();
        for (int i = 0; i < a->NumChildren; i++) {
            action_rank_.push_back({a->Child(i).Value.GetValue()});
        }
    }

    void save_action_rank(std::vector<double> r) {
        action_rank_ = r;
    }

    int x() const { return x_; }
    int y() const { return y_; }
    int action() const { return action_; }
    const std::vector<double> &rocks() const { return rock_probs_; }
    const std::vector<double> &action_rank() const { return action_rank_; }

    friend void to_json(nlohmann::json& j, const ROCKSAMPLE_COMPLETE_STEP& p);
    friend void from_json(const nlohmann::json &j, ROCKSAMPLE_COMPLETE_STEP &p);
  private:
    // metainfo
    std::vector<double> rock_probs_;
    std::vector<double> action_rank_;
    int x_, y_, action_;
    // complete belief
    std::vector<ROCKSAMPLE_STATE> particles_;
};
void to_json(nlohmann::json& j, const ROCKSAMPLE_COMPLETE_STEP& p);
void from_json(const nlohmann::json &j, ROCKSAMPLE_COMPLETE_STEP &p);

class ROCKSAMPLE_TRACE_INFO {
};

class ROCKSAMPLE_RUN_INFO {
};


#if 1
using ROCKSAMPLE_RUN = RUN<ROCKSAMPLE_RUN_INFO, ROCKSAMPLE_STEP>;
using ROCKSAMPLE_TRACE = TRACE<ROCKSAMPLE_TRACE_INFO, ROCKSAMPLE_RUN>;
#else
using ROCKSAMPLE_RUN = RUN<ROCKSAMPLE_RUN_INFO, ROCKSAMPLE_COMPLETE_STEP>;
using ROCKSAMPLE_TRACE = TRACE<ROCKSAMPLE_TRACE_INFO, ROCKSAMPLE_RUN>;
#endif


