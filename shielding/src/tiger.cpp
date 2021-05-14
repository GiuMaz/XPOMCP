#include "tiger.h"
#include "utils.h"
#include <fstream>
#include <sstream>
#include <array>

using namespace std;
using namespace UTILS;

TIGER::TIGER() :
    SIMULATOR(3, 4, 1.0),
    tr_open(0.92),
    tr_listen(0.87)
{
    RewardRange = 110.0; // from -100 to 10
}

TIGER::TIGER(const std::string &shield_file):
    SIMULATOR(3, 4, 1.0)
{
    RewardRange = 110.0; // from -100 to 10

    // shield rule
    ifstream iff(shield_file);
    iff >> tr_open >> tr_listen;

    if (iff.eof())
        return;

    // threshold for comparison of hellinger distances
    double threshold;
    iff >> threshold;
    open_left_points.set_threshold(threshold);
    open_right_points.set_threshold(threshold);
    listen_points.set_threshold(threshold);

    // collect points for comparison
    std::string line;
    std::getline(iff, line); // discard threhsold line
    std::array<double, 2> p;

    // open points
    std::vector<std::array<double, 2>> left, right;
    std::getline(iff, line);
    std::stringstream ssopen(line);
    while (ssopen >> p[0] >> p[1]) {
        if (p[0] < p[1])
            left.emplace_back(p);
        else
            right.emplace_back(p);
    }
    open_left_points.set_points(left);
    open_right_points.set_points(right);

    std::vector<std::array<double, 2>> points;
    // listen points
    std::getline(iff, line);
    std::stringstream sslisten(line);
    while (sslisten >> p[0] >> p[1])
        points.emplace_back(p);
}

STATE* TIGER::Copy(const STATE& state) const {
    const auto& tiger_state = safe_cast<const TIGER_STATE&>(state);
    TIGER_STATE* newstate = MemoryPool.Allocate();
    *newstate = tiger_state;
    return newstate;
}

// NOT USED
void TIGER::Validate(const STATE& /*state*/) const { }

STATE* TIGER::CreateStartState() const {
    TIGER_STATE* state = MemoryPool.Allocate();
    state->tiger_on_left = static_cast<bool>(random_state()%2);
    state->saved_actions.clear();
    return state;
}

void TIGER::set_belief_metainfo(VNODE *v) const {
    v->Beliefs().set_metainfo(TIGER_METAINFO());
}

void TIGER::FreeState(STATE* state) const // Free memory of state
{
    TIGER_STATE* s = safe_cast<TIGER_STATE*>(state);
    MemoryPool.Free(s);
}


bool TIGER::Step(STATE& state, int action, int& observation, double& reward) const {


    TIGER_STATE& tiger_state = safe_cast<TIGER_STATE&>(state);
    reward = 0.0;
    tiger_state.saved_actions.push_back(action);

    if (action == A_LISTEN) // check
    {
        bool true_roar  = (random_state()%100) < 85;
        if (true_roar)
            observation = tiger_state.tiger_on_left ? O_LEFT_ROAR : O_RIGHT_ROAR;
        else
            observation = tiger_state.tiger_on_left ? O_RIGHT_ROAR : O_LEFT_ROAR;

        reward = -1.0;
        return false;
    }

    // Action is a choise
    assert(action == A_LEFT_DOOR || action == A_RIGHT_DOOR);

    // check if the action chose is the same as the position of the tiger
    bool found_a_tiger = tiger_state.tiger_on_left == (action == A_LEFT_DOOR); 

    if (found_a_tiger) {
        reward = -100;
        observation = O_TIGER;
    }
    else { // treasure!
        reward = 10;
        observation = O_TREASURE;
    }
    return true;
}

bool TIGER::LocalMove(STATE& state, const HISTORY& history,
        int stepObservation, const STATUS& status) const {
    // not required
    return true;
}

void TIGER::GenerateLegal(const STATE& /*state*/, const HISTORY& /*history*/,
                          std::vector<int>& legal,
                          const STATUS& /*status*/) const {
    // in this simple game all the actions are always legal
    legal.push_back(A_LISTEN);
    legal.push_back(A_LEFT_DOOR);
    legal.push_back(A_RIGHT_DOOR);
}

void TIGER::GeneratePreferred(const STATE& /*state*/,
                              const HISTORY& /*history*/,
                              std::vector<int>& legal,
                              const STATUS& /*status*/) const {
    // no preferred action, it's a copy of GenerateLegal
    legal.push_back(A_LISTEN);
    legal.push_back(A_LEFT_DOOR);
    legal.push_back(A_RIGHT_DOOR);
}

void TIGER::DisplayBeliefs(const BELIEF_STATE& beliefState,
                           std::ostream& ostr) const {
    ostr << "TIGER::DisplayBeliefs start" << endl;
    for (int i = 0; i < beliefState.GetNumSamples(); i++){
        const STATE* s = beliefState.GetSample(i);
        DisplayState(*s,ostr);
    }
    ostr << "TIGER::DisplayBeliefs end" << endl;
}

void TIGER::DisplayState(const STATE& state, std::ostream& ostr) const {
    const TIGER_STATE& tiger_state = safe_cast<const TIGER_STATE&>(state);
    ostr << endl;
    ostr << "## STATE ############" << endl;
    ostr << "tiger on left: " << tiger_state.tiger_on_left << endl;
    ostr << "#######################" << endl<< endl;
}

void TIGER::DisplayObservation(const STATE& state, int observation,
                               std::ostream& ostr) const {
    switch (observation) {
        case O_LEFT_ROAR:
            ostr << "roar from left door" << endl;
            break;
        case O_RIGHT_ROAR:
            ostr << "roar from right door" << endl;
            break;
        case O_TREASURE:
            ostr << "foundt a treasure!" << endl;
            break;
        case O_TIGER:
            ostr << "eaten by a tiger!" << endl;
            break;
    }
}

void TIGER::DisplayAction(int action, std::ostream& ostr) const {
    switch (action) {
        case A_LISTEN:
            ostr << "listen" << endl;
            break;
        case A_LEFT_DOOR:
            ostr << "open left door" << endl;
            break;
        case A_RIGHT_DOOR:
            ostr << "open right door" << endl;
            break;
    }
}

// xes
void TIGER::log_beliefs(const BELIEF_STATE& beliefState,
        xes_logger & xes) const {
    int left = 0, right = 0;

    for (int i = 0; i < beliefState.GetNumSamples(); i++){
        const STATE* s = beliefState.GetSample(i);
        const TIGER_STATE *ts = safe_cast<const TIGER_STATE *>(s);
        if (ts->tiger_on_left)
            ++left;
        else
            ++right;
    }
    xes.start_list("belief");
    xes.add_attribute({"tiger left", left});
    xes.add_attribute({"tiger right", right});
    xes.end_list();
}

void TIGER::log_state(const STATE& state, xes_logger & xes) const {
    const TIGER_STATE& tiger_state = safe_cast<const TIGER_STATE&>(state);
    xes.start_list("state");
    xes.add_attribute({"tiger on left", tiger_state.tiger_on_left});
    xes.end_list();
}

void TIGER::log_action(int action, xes_logger & xes) const {
    switch (action) {
        case A_LISTEN:
            xes.add_attribute({"action", "listen"});
            break;
        case A_LEFT_DOOR:
            xes.add_attribute({"action", "open left"});
            break;
        case A_RIGHT_DOOR:
            xes.add_attribute({"action", "open right"});
            break;
    }
}

void TIGER::log_observation(const STATE& state, int observation, xes_logger & xes) const {
    switch (observation) {
        case O_LEFT_ROAR:
            xes.add_attribute({"observation", "roar left"});
            break;
        case O_RIGHT_ROAR:
            xes.add_attribute({"observation", "roar right"});
            break;
        case O_TREASURE:
            xes.add_attribute({"observation", "treasure"});
            break;
        case O_TIGER:
            xes.add_attribute({"observation", "tiger"});
            break;
    }
}

void TIGER::log_reward(double reward, xes_logger & xes) const {
    xes.add_attribute({"reward", reward});
}

void TIGER::log_problem_info(xes_logger &xes) const {
    xes.add_attributes({
            {"problem", "tiger"},
            {"RewardRange", RewardRange},
            {"shield open", tr_open},
            {"shield listen", tr_listen},
            {"complex shielding", complex_shield}
        });
}

void TIGER::pre_shield(const BELIEF_STATE &belief, std::vector<int> &legal_actions) const{
    legal_actions.clear();
    // listen is always legal

    const auto& meta =
        dynamic_cast<const TIGER_METAINFO&>(belief.get_metainfo());

    std::array<double, 2> point = {meta.perc_on_left(), 1.0 - meta.perc_on_left()};

    if (meta.perc_on_left() <= tr_listen &&
            meta.perc_on_left() >= (1.0 - tr_listen)) {
        legal_actions.push_back(A_LISTEN);
    }
    else if (complex_shield && listen_points.is_in_threshold(point)) {
        legal_actions.push_back(A_LISTEN);
    }

    if (meta.perc_on_left() <= (1.0 - tr_open)) {
        legal_actions.push_back(A_LEFT_DOOR);
    }
    else if (complex_shield && open_left_points.is_in_threshold(point)) {
        legal_actions.push_back(A_LEFT_DOOR);
    }

    if (meta.perc_on_left() >= tr_open) {
        legal_actions.push_back(A_RIGHT_DOOR);
    }
    else if (complex_shield && open_right_points.is_in_threshold(point)) {
        legal_actions.push_back(A_RIGHT_DOOR);
    }

    // default
    if (legal_actions.empty())
        legal_actions.push_back(A_LISTEN);
}

