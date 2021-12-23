#include "obstacleavoidance_trace.h"

void to_json(nlohmann::json& j, const OBSTACLEAVOIDANCE_STEP& p) {
    std::vector<OBSTACLEAVOIDANCE_STATE> particles;
    j = nlohmann::json{
        {"distr_", p.distr_},
        {"action_rank_", p.action_rank_},
        {"seg_,", p.seg_},
        {"subseg_,", p.subseg_},
        {"action_", p.action_}
    };
}

void from_json(const nlohmann::json &j, OBSTACLEAVOIDANCE_STEP &p) {
    //j.at("Valuable").get_to(p.Valuable);
    j.at("distr_").get_to(p.distr_);
    j.at("action_rank_").get_to(p.action_rank_);
    j.at("seg_,").get_to(p.seg_);
    j.at("subseg_,").get_to(p.subseg_);
    j.at("action_").get_to(p.action_);
}
