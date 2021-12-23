#pragma once
#include "json.hpp"
#include "obstacleavoidance.h"
#include "trace.h"

class OBSTACLEAVOIDANCE_STEP_INFO {
};

// only store metadata
class OBSTACLEAVOIDANCE_STEP {
public:
    using step_info_t = OBSTACLEAVOIDANCE_STEP_INFO;

    void save_belief(const BELIEF_STATE &b) {
        const auto &info =
            safe_cast<const OBSTACLEAVOIDANCE_METAINFO &>(b.get_metainfo());

        const auto *state =
            safe_cast<const OBSTACLEAVOIDANCE_STATE *>(b.GetSample(0));

        for (int seg = 0; seg < 8; seg++)
            for (int diff = 0; diff < 3; diff++)
                distr_[seg][diff] = info.get_prob_diff(seg, diff);

        seg_ = state->curSegI;
        subseg_ = state->curSubsegJ;
    }

    void build_from_json(std::vector<double> r, int x, int y) {
        // DISABLED
        assert(false);
    }

    void save_action(int a) {
        action_ = a;
    }

    void save_action_rank(const VNODE * const a) {
        // DISABLED
        /*
        action_rank_.clear();
        for (int i = 0; i < a->NumChildren; i++) {
            action_rank_.push_back({a->Child(i).Value.GetValue()});
        }
        */
    }

    void save_action_rank(std::vector<double> r) {
        // DISABLED
        //action_rank_ = r;
    }

    int seg() const { return seg_; }
    int subseg() const { return subseg_; }
    int action() const { return action_; }
    const std::array<std::array<double, 3>, 8> &distr() const { return distr_; }
    double seg_diff(int seg, int diff) const { return distr_[seg][diff]; }
    const std::array<double, 3> &seg_diff_distr(int seg) const {
        return distr_[seg];
    }

    const std::vector<double> &action_rank() const { return action_rank_; }

    friend void to_json(nlohmann::json& j, const OBSTACLEAVOIDANCE_STEP& p);
    friend void from_json(const nlohmann::json &j, OBSTACLEAVOIDANCE_STEP &p);
  private:
    // metainfo
    std::array<std::array<double, 3>, 8> distr_;
    std::vector<double> action_rank_;
    int seg_, subseg_, action_;
};

void to_json(nlohmann::json& j, const OBSTACLEAVOIDANCE_STEP& p);
void from_json(const nlohmann::json &j, OBSTACLEAVOIDANCE_STEP &p);

class OBSTACLEAVOIDANCE_TRACE_INFO {
};

class OBSTACLEAVOIDANCE_RUN_INFO {
};


using OBSTACLEAVOIDANCE_RUN = RUN<OBSTACLEAVOIDANCE_RUN_INFO, OBSTACLEAVOIDANCE_STEP>;
using OBSTACLEAVOIDANCE_TRACE = TRACE<OBSTACLEAVOIDANCE_TRACE_INFO, OBSTACLEAVOIDANCE_RUN>;




