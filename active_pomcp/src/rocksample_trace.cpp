#include "rocksample_trace.h"

void to_json(nlohmann::json& j, const ROCKSAMPLE_STEP& p) {
    // complete belief
    std::vector<ROCKSAMPLE_STATE> particles;
    j = nlohmann::json{
        {"rock_probs_", p.rock_probs_},
        {"action_rank_", p.action_rank_},
        {"x_,", p.x_},
        {"y_,", p.y_},
        {"action_", p.action_}
    };
}

void from_json(const nlohmann::json &j, ROCKSAMPLE_STEP &p) {
    //j.at("Valuable").get_to(p.Valuable);
    j.at("rock_probs_").get_to(p.rock_probs_);
    j.at("action_rank_").get_to(p.action_rank_);
    j.at("x_,").get_to(p.x_);
    j.at("y_,").get_to(p.y_);
    j.at("action_").get_to(p.action_);
}

void to_json(nlohmann::json& j, const ROCKSAMPLE_COMPLETE_STEP& p) {
    // complete belief
    std::vector<ROCKSAMPLE_STATE> particles;
    j = nlohmann::json{
        {"rock_probs_", p.rock_probs_},
        {"action_rank_", p.action_rank_},
        {"x_,", p.x_},
        {"y_,", p.y_},
        {"action_", p.action_},
        {"particles_", p.particles_}
    };
}

void from_json(const nlohmann::json &j, ROCKSAMPLE_COMPLETE_STEP &p) {
    //j.at("Valuable").get_to(p.Valuable);
    j.at("rock_probs_").get_to(p.rock_probs_);
    j.at("action_rank_").get_to(p.action_rank_);
    j.at("x_,").get_to(p.x_);
    j.at("y_,").get_to(p.y_);
    j.at("action_").get_to(p.action_);
    j.at("particles_").get_to(p.particles_);
}
