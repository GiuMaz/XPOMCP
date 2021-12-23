#include "rocksample.h"
#include <limits>
#include <sstream>

#include <unordered_map>

#include "utils.h"

using namespace std;
using namespace UTILS;

ROCKSAMPLE::ROCKSAMPLE(int size, int rocks)
:   Grid(size, size),
    Size(size),
    NumRocks(rocks),
    SmartMoveProb(0.95),
    UncertaintyCount(0),
    has_fixed_belief(false)
{
    NumActions = NumRocks + 5;
    NumObservations = 3;
    RewardRange = 20;
    Discount = 0.95;

    if (size == 4 && rocks == 1)
        Init_4_1();
    else if (size == 7 && rocks == 8)
        Init_7_8();
    else if (size == 11 && rocks == 11)
        Init_11_11();
    else if (size == 15 && rocks == 15)
        Init_15_15();
    else
        InitGeneral();
}

ROCKSAMPLE::ROCKSAMPLE(int size, int rocks, int x, int y,
           const std::vector<double> &belief)
:   Grid(size, size),
    Size(size),
    NumRocks(rocks),
    SmartMoveProb(0.95),
    UncertaintyCount(0) ,
    has_fixed_belief(true),
    fixed_belief(belief)
{
    NumActions = NumRocks + 5;
    NumObservations = 3;
    RewardRange = 20;
    Discount = 0.95;

    if (size == 4 && rocks == 1)
        Init_4_1();
    else if (size == 7 && rocks == 8)
        Init_7_8();
    else if (size == 11 && rocks == 11)
        Init_11_11();
    else if (size == 15 && rocks == 15)
        Init_15_15();
    else
        InitGeneral();

    StartPos = COORD(x, y);
}

ROCKSAMPLE::ROCKSAMPLE(int size, int rocks, std::string shield_file)
:   Grid(size, size),
    Size(size),
    NumRocks(rocks),
    SmartMoveProb(0.95),
    UncertaintyCount(0),
    has_fixed_belief(false)
{
    NumActions = NumRocks + 5;
    NumObservations = 3;
    RewardRange = 20;
    Discount = 0.95;

    if (size == 4 && rocks == 1)
        Init_4_1();
    else if (size == 7 && rocks == 8)
        Init_7_8();
    else if (size == 11 && rocks == 11)
        Init_11_11();
    else if (size == 15 && rocks == 15)
        Init_15_15();
    else
        InitGeneral();

    ifstream iff(shield_file);
    iff >> sample_shield_tr;
    if (iff.eof())
        return;

    double threshold;
    iff >> threshold;
    sampling_points.set_threshold(threshold);

    std::string line;
    std::getline(iff, line); // discard threshold line
    std::array<double, 1> p;
    std::vector<std::array<double, 1>> points;

    std::getline(iff, line);
    std::stringstream ssopen(line);
    while (ssopen >> p[0])
        points.emplace_back(p);
    sampling_points.set_points(points);

}

void ROCKSAMPLE::InitGeneral()
{
    HalfEfficiencyDistance = 20; // 20 IS STANDARD
    //HalfEfficiencyDistance = 2; // 2 IS NOISY
    StartPos = COORD(0, Size / 2);
    //RandomSeed(0);
    Grid.SetAllValues(-1);
    for (int i = 0; i < NumRocks; ++i)
    {
        COORD pos;
        do {
            pos = COORD(random_state() % Size, random_state() % Size);
        } while (Grid(pos) >= 0);
        Grid(pos) = i;
        RockPos.push_back(pos);
    }
}

void ROCKSAMPLE::Init_4_1()
{
    // Equivalent to RockSample_7_8.pomdpx
    cout << "Using special layout for rocksample(7, 8)" << endl;

    COORD rocks[] =
    {
        COORD(3, 2),
    };

    HalfEfficiencyDistance = 20;
    //HalfEfficiencyDistance = 2; // 2 IS NOISY
    StartPos = COORD(0, 0);
    Grid.SetAllValues(-1);

    for (int i = 0; i < NumRocks; ++i)
    {
        Grid(rocks[i]) = i;
        RockPos.push_back(rocks[i]);
    }
}

void ROCKSAMPLE::Init_7_8()
{
    // Equivalent to RockSample_7_8.pomdpx
    cout << "Using special layout for rocksample(7, 8)" << endl;

    COORD rocks[] =
    {
        COORD(2, 0),
        COORD(0, 1),
        COORD(3, 1),
        COORD(6, 3),
        COORD(2, 4),
        COORD(3, 4),
        COORD(5, 5),
        COORD(1, 6)
    };

    HalfEfficiencyDistance = 20;
    StartPos = COORD(0, 3);
    Grid.SetAllValues(-1);
    for (int i = 0; i < NumRocks; ++i)
    {
        Grid(rocks[i]) = i;
        RockPos.push_back(rocks[i]);
    }
}

void ROCKSAMPLE::Init_11_11()
{
    // Equivalent to RockSample_11_11.pomdp(x)
    cout << "Using special layout for rocksample(11, 11)" << endl;

    COORD rocks[] =
    {
        COORD(0, 3),
        COORD(0, 7),
        COORD(1, 8),
        COORD(2, 4),
        COORD(3, 3),
        COORD(3, 8),
        COORD(4, 3),
        COORD(5, 8),
        COORD(6, 1),
        COORD(9, 3),
        COORD(9, 9)
    };

    HalfEfficiencyDistance = 20;
    //HalfEfficiencyDistance = 2;
    StartPos = COORD(0, 5);
    Grid.SetAllValues(-1);
    for (int i = 0; i < NumRocks; ++i)
    {
        Grid(rocks[i]) = i;
        RockPos.push_back(rocks[i]);
    }
}

void ROCKSAMPLE::Init_15_15()
{
    cout << "Using special layout for rocksample(15, 15)" << endl;

    COORD rocks[] =
    {
        COORD(0, 4),
        COORD(0, 11),
        COORD(1, 8),
        COORD(2, 4),
        COORD(2, 12),
        COORD(3, 9),
        COORD(4, 3),
        COORD(5, 14),
        COORD(6, 0),
        COORD(6, 8),
        COORD(9, 3),
        COORD(10, 2),
        COORD(11, 7),
        COORD(12, 12),
        COORD(14, 9)
    };

    HalfEfficiencyDistance = 20;
    //HalfEfficiencyDistance = 2;
    StartPos = COORD(0, 7);
    Grid.SetAllValues(-1);
    for (int i = 0; i < NumRocks; ++i) {
        Grid(rocks[i]) = i;
        RockPos.push_back(rocks[i]);
    }
}

STATE* ROCKSAMPLE::Copy(const STATE& state) const
{
    const ROCKSAMPLE_STATE& rockstate = safe_cast<const ROCKSAMPLE_STATE&>(state);
    ROCKSAMPLE_STATE* newstate = MemoryPool.Allocate();
    *newstate = rockstate;
    return newstate;
}

void ROCKSAMPLE::Validate(const STATE& state) const
{
    const ROCKSAMPLE_STATE& rockstate = safe_cast<const ROCKSAMPLE_STATE&>(state);
    assert(Grid.Inside(rockstate.AgentPos));
}

STATE* ROCKSAMPLE::CreateStartState() const
{
    ROCKSAMPLE_STATE* rockstate = MemoryPool.Allocate();
    rockstate->AgentPos = StartPos;
    rockstate->Rocks.clear();

    if (has_fixed_belief) {
        for (int i = 0; i < NumRocks; i++)
        {
            ROCKSAMPLE_STATE::ENTRY entry;
            entry.Collected = false;
            entry.Valuable = unif_dist(random_state) <= fixed_belief[i];
            entry.Count = 0;
            entry.Measured = 0;
            entry.ProbValuable = 0.5;
            entry.LikelihoodValuable = 1.0;
            entry.LikelihoodWorthless = 1.0;
            rockstate->Rocks.push_back(entry);
        }
        rockstate->Target = SelectTarget(*rockstate);
    }
    else {
        for (int i = 0; i < NumRocks; i++)
        {
            ROCKSAMPLE_STATE::ENTRY entry;
            entry.Collected = false;
            entry.Valuable = unif_dist(random_state) <= 0.5;
            entry.Count = 0;
            entry.Measured = 0;
            entry.ProbValuable = 0.5;
            entry.LikelihoodValuable = 1.0;
            entry.LikelihoodWorthless = 1.0;
            rockstate->Rocks.push_back(entry);
        }
        rockstate->Target = SelectTarget(*rockstate);
    }
    return rockstate;
}

void ROCKSAMPLE::FreeState(STATE* state) const
{
    ROCKSAMPLE_STATE* rockstate = safe_cast<ROCKSAMPLE_STATE*>(state);
    MemoryPool.Free(rockstate);
}

// return true if the state is final
bool ROCKSAMPLE::Step(STATE& state, int action,
    observation_t& observation, double& reward) const
{
    ROCKSAMPLE_STATE& rockstate = safe_cast<ROCKSAMPLE_STATE&>(state);
    reward = 0;
    observation = E_NONE;

    if (action < E_SAMPLE) // move
    {
        switch (action)
        {
            case COORD::E_EAST:
                if (rockstate.AgentPos.X + 1 < Size)
                {
                    rockstate.AgentPos.X++;
                    break;
                }
                else
                {
                    reward = +10;
                    return true;
                }

            case COORD::E_NORTH:
                if (rockstate.AgentPos.Y + 1 < Size)
                    rockstate.AgentPos.Y++;
                else
                    reward = -100;
                break;

            case COORD::E_SOUTH:
                if (rockstate.AgentPos.Y - 1 >= 0)
                    rockstate.AgentPos.Y--;
                else
                    reward = -100;
                break;

            case COORD::E_WEST:
                if (rockstate.AgentPos.X - 1 >= 0)
                    rockstate.AgentPos.X--;
                else
                    reward = -100;
                break;
        }
    }

    if (action == E_SAMPLE) // sample
    {
        int rock = Grid(rockstate.AgentPos);
        if (rock >= 0 && !rockstate.Rocks[rock].Collected)
        {
            rockstate.Rocks[rock].Collected = true;
            if (rockstate.Rocks[rock].Valuable)
                reward = +10;
            else
                reward = -10;
        }
        else
        {
            reward = -100;
        }
    }

    if (action > E_SAMPLE) // check
    {
        int rock = action - E_SAMPLE - 1;
        assert(rock < NumRocks);
        observation = GetObservation(rockstate, rock);
        rockstate.Rocks[rock].Measured++;

        double distance = COORD::EuclideanDistance(rockstate.AgentPos, RockPos[rock]);
        double efficiency = (1 + pow(2, -distance / HalfEfficiencyDistance)) * 0.5;

        if (observation == E_GOOD)
        {
            rockstate.Rocks[rock].Count++;
            rockstate.Rocks[rock].LikelihoodValuable *= efficiency;
            rockstate.Rocks[rock].LikelihoodWorthless *= 1.0 - efficiency;

        }
        else
        {
            rockstate.Rocks[rock].Count--;
            rockstate.Rocks[rock].LikelihoodWorthless *= efficiency;
            rockstate.Rocks[rock].LikelihoodValuable *= 1.0 - efficiency;
        }
        double denom = (0.5 * rockstate.Rocks[rock].LikelihoodValuable) +
            (0.5 * rockstate.Rocks[rock].LikelihoodWorthless);
        rockstate.Rocks[rock].ProbValuable = (0.5 * rockstate.Rocks[rock].LikelihoodValuable) / denom;
    }

    if (rockstate.Target < 0 || rockstate.AgentPos == RockPos[rockstate.Target])
        rockstate.Target = SelectTarget(rockstate);

    assert(reward != -100);
    return false;
}

/*
 * explore unlikely scenarios
 */
bool ROCKSAMPLE::Step(const VNODE *const mcts_root, STATE &state, int action, 
        observation_t& observation, double& reward, double min, double max) const
{
    // consider only the check actions
    if (action <= E_SAMPLE)
        return Step(state, action, observation, reward);

    const auto &qnode = mcts_root->Child(action);

    for (int i = 0; i < NumObservations; i++) {
        const VNODE *vnode = qnode.Child(i);

        if (!vnode || vnode->Beliefs().Empty())
            continue;

        const auto &meta = dynamic_cast<const ROCKSAMPLE_METAINFO &>(
            vnode->Beliefs().get_metainfo());
        int rock = action - E_SAMPLE - 1;
        double prob = meta.get_prob_valuable(rock);
        if (prob > min && prob < max) {
            std::cout << "ROCK " << rock << " PROB " << prob << " FORCE OBSERVATION " << i << std::endl;
            reward = 0.0;
            observation = i;
            return false;
        }
    }

    // default behavior
    return Step(state, action, observation, reward);
}

bool ROCKSAMPLE::LocalMove(STATE& state, const HISTORY& history,
    int stepObs, const STATUS& status) const
{
    ROCKSAMPLE_STATE& rockstate = safe_cast<ROCKSAMPLE_STATE&>(state);
    int rock = random_state() % NumRocks;
    rockstate.Rocks[rock].Valuable = !rockstate.Rocks[rock].Valuable;

    if (history.Back().Action > E_SAMPLE) // check rock
    {
        rock = history.Back().Action - E_SAMPLE - 1;
        int realObs = history.Back().Observation;

        // Condition new state on real observation
        int newObs = GetObservation(rockstate, rock);
        if (newObs != realObs)
            return false;

        // Update counts to be consistent with real observation
        if (realObs == E_GOOD && stepObs == E_BAD)
            rockstate.Rocks[rock].Count += 2;
        if (realObs == E_BAD && stepObs == E_GOOD)
            rockstate.Rocks[rock].Count -= 2;
    }
    return true;
}

void ROCKSAMPLE::GenerateLegal(const STATE& state, const HISTORY& history,
    vector<int>& legal, const STATUS& status) const
{

    const ROCKSAMPLE_STATE& rockstate =
        safe_cast<const ROCKSAMPLE_STATE&>(state);

    if (rockstate.AgentPos.Y + 1 < Size)
        legal.push_back(COORD::E_NORTH);

    legal.push_back(COORD::E_EAST);

    if (rockstate.AgentPos.Y - 1 >= 0)
        legal.push_back(COORD::E_SOUTH);

    if (rockstate.AgentPos.X - 1 >= 0)
        legal.push_back(COORD::E_WEST);

    int rock = Grid(rockstate.AgentPos);
    if (rock >= 0 && !rockstate.Rocks[rock].Collected)
        legal.push_back(E_SAMPLE);

    for (rock = 0; rock < NumRocks; ++rock)
        if (!rockstate.Rocks[rock].Collected)
            legal.push_back(rock + 1 + E_SAMPLE);
}

void ROCKSAMPLE::GeneratePreferred(const STATE& state, const HISTORY& history,
    vector<int>& actions, const STATUS& status) const
{
    static const bool UseBlindPolicy = false;

    if (UseBlindPolicy) {
        actions.push_back(COORD::E_EAST);
        return;
    }

    const auto& rockstate = safe_cast<const ROCKSAMPLE_STATE&>(state);

    // Sample rocks with more +ve than -ve observations
    int rock = Grid(rockstate.AgentPos);
    if (rock >= 0 && !rockstate.Rocks[rock].Collected) {
        int total = 0;
        for (int t = 0; t < history.Size(); ++t) {
            if (history[t].Action == rock + 1 + E_SAMPLE)
            {
                if (history[t].Observation == E_GOOD)
                    total++;
                if (history[t].Observation == E_BAD)
                    total--;
            }
        }
        if (total > 0) {
            actions.push_back(E_SAMPLE);
            return;
        }
    }

    // processes the rocks
    bool all_bad = true;
    bool north_interesting = false;
    bool south_interesting = false;
    bool west_interesting  = false;
    bool east_interesting  = false;

    for (int rock = 0; rock < NumRocks; ++rock) {
        const ROCKSAMPLE_STATE::ENTRY& entry = rockstate.Rocks[rock];
        if (!entry.Collected) {
            int total = 0;
            for (int t = 0; t < history.Size(); ++t) {
                if (history[t].Action == rock + 1 + E_SAMPLE) {
                    if (history[t].Observation == E_GOOD)
                        total++;
                    if (history[t].Observation == E_BAD)
                        total--;
                }
            }

            if (total >= 0) {
                all_bad = false;

                if (RockPos[rock].Y > rockstate.AgentPos.Y)
                    north_interesting = true;
                if (RockPos[rock].Y < rockstate.AgentPos.Y)
                    south_interesting = true;
                if (RockPos[rock].X < rockstate.AgentPos.X)
                    west_interesting = true;
                if (RockPos[rock].X > rockstate.AgentPos.X)
                    east_interesting = true;
            }
        }
    }

    // if all remaining rocks seem bad, then head east
    if (all_bad) {
        actions.push_back(COORD::E_EAST);
        return;
    }

    // generate a random legal move, with the exceptions that:
    //   a) there is no point measuring a rock that is already collected
    //   b) there is no point measuring a rock too often
    //   c) there is no point measuring a rock which is clearly bad or good
    //   d) we never sample a rock (since we need to be sure)
    //   e) we never move in a direction that doesn't take us closer to
    //      either the edge of the map or an interesting rock
    if (rockstate.AgentPos.Y + 1 < Size && north_interesting)
        actions.push_back(COORD::E_NORTH);

    if (east_interesting)
        actions.push_back(COORD::E_EAST);

    if (rockstate.AgentPos.Y - 1 >= 0 && south_interesting)
        actions.push_back(COORD::E_SOUTH);

    if (rockstate.AgentPos.X - 1 >= 0 && west_interesting)
        actions.push_back(COORD::E_WEST);

    for (rock = 0; rock < NumRocks; ++rock) {
        if (!rockstate.Rocks[rock].Collected &&
            rockstate.Rocks[rock].ProbValuable != 0.0 &&
            rockstate.Rocks[rock].ProbValuable != 1.0 &&
            rockstate.Rocks[rock].Measured < 5 &&
            std::abs(rockstate.Rocks[rock].Count) < 2
            ) {
            actions.push_back(rock + 1 + E_SAMPLE);
        }
    }
}

int ROCKSAMPLE::GetObservation(const ROCKSAMPLE_STATE& rockstate, int rock) const
{
    double distance = COORD::EuclideanDistance(rockstate.AgentPos, RockPos[rock]);
    double efficiency = (1.0 + pow(2, -distance/HalfEfficiencyDistance)) * 0.5;

    if (unif_dist(random_state) < efficiency)
        return rockstate.Rocks[rock].Valuable ? E_GOOD : E_BAD;
    else
        return rockstate.Rocks[rock].Valuable ? E_BAD : E_GOOD;
}

int ROCKSAMPLE::SelectTarget(const ROCKSAMPLE_STATE& rockstate) const
{
    int bestDist = Size * 2;
    int bestRock = -1;
    for (int rock = 0; rock < NumRocks; ++rock)
    {
        if (!rockstate.Rocks[rock].Collected
            && rockstate.Rocks[rock].Count >= UncertaintyCount)
        {
            int dist = COORD::ManhattanDistance(rockstate.AgentPos, RockPos[rock]);
            if (dist < bestDist)
                bestDist = dist;
        }
    }
    return bestRock;
}

void ROCKSAMPLE::DisplayBeliefs(const BELIEF_STATE& beliefState,
    std::ostream& ostr) const
{
}

void ROCKSAMPLE::DisplayState(const STATE& state, std::ostream& ostr) const
{
    const ROCKSAMPLE_STATE& rockstate = safe_cast<const ROCKSAMPLE_STATE&>(state);
    ostr << endl;
    for (int x = 0; x < Size + 2; x++)
        ostr << "# ";
    ostr << endl;
    for (int y = Size - 1; y >= 0; y--)
    {
        ostr << "# ";
        for (int x = 0; x < Size; x++)
        {
            COORD pos(x, y);
            int rock = Grid(pos);
            const ROCKSAMPLE_STATE::ENTRY& entry = rockstate.Rocks[rock];
            if (rockstate.AgentPos == COORD(x, y))
                ostr << "* ";
            else if (rock >= 0 && !entry.Collected)
                ostr << rock << (entry.Valuable ? "$" : "X");
            else
                ostr << ". ";
        }
        ostr << "#" << endl;
    }
    for (int x = 0; x < Size + 2; x++)
        ostr << "# ";
    ostr << endl;
}

void ROCKSAMPLE::DisplayObservation(const STATE& state, observation_t observation, std::ostream& ostr) const
{
    switch (observation)
    {
    case E_NONE:
        break;
    case E_GOOD:
        ostr << "Observed good" << endl;
        break;
    case E_BAD:
        ostr << "Observed bad" << endl;
        break;
    }
}

void ROCKSAMPLE::DisplayAction(int action, std::ostream& ostr) const
{
    if (action < E_SAMPLE)
        ostr << COORD::CompassString[action] << endl;
    if (action == E_SAMPLE)
        ostr << "Sample" << endl;
    if (action > E_SAMPLE)
        ostr << "Check " << action - E_SAMPLE << endl;
}

void ROCKSAMPLE::log_problem_info() const {
    XES::logger().add_attributes({
            {"problem", "rocksample"},
            {"RewardRange", RewardRange},
            {"HalfEfficiencyDistance", HalfEfficiencyDistance},
            {"Size", Size},
            {"NumRocks", NumRocks}
        });

    XES::logger().start_list("rocks");
    for (int i = 0; i < RockPos.size(); i++) {
        XES::logger().start_list("rock");
        XES::logger().add_attributes({
                {"number", i},
                {"coord x", RockPos[i].X},
                {"coord y", RockPos[i].Y},
                });
        XES::logger().end_list();
    }
    XES::logger().end_list(); // end rocks
}

void ROCKSAMPLE::log_beliefs(const BELIEF_STATE& beliefState) const {
    std::unordered_map<int, int> dist;
    int id;
    for (int i = 0; i < beliefState.GetNumSamples(); i++){
        const STATE* state = beliefState.GetSample(i);
        const auto& rockstate = safe_cast<const ROCKSAMPLE_STATE&>(*state);
        
        // Compute state id
        id=0;   
        for (int j = 0; j<rockstate.Rocks.size(); j++) {
            id += rockstate.Rocks[j].Valuable ? pow2(j) : 0;
        }
        // If it is not in dist then add it and initialize to 1
        if (dist.find(id) == dist.end())
            dist[id]= 1;
        else
            dist[id]++;
    }

    const STATE* state = beliefState.GetSample(0);
    const auto& rs = safe_cast<const ROCKSAMPLE_STATE&>(*state);
    XES::logger().add_attribute({"coord x", rs.AgentPos.X });
    XES::logger().add_attribute({"coord y", rs.AgentPos.Y });
    XES::logger().start_list("belief");
    for (const auto &[k, v] : dist)
        XES::logger().add_attribute({std::to_string(k), v});
    XES::logger().end_list();
}

void ROCKSAMPLE::log_state(const STATE& state) const {
    const ROCKSAMPLE_STATE& rs = safe_cast<const ROCKSAMPLE_STATE&>(state);
    XES::logger().start_list("state");
    XES::logger().add_attribute({"coord x", rs.AgentPos.X });
    XES::logger().add_attribute({"coord y", rs.AgentPos.Y });
    XES::logger().start_list("rocks");
    int i = 0;
    for (const auto &r : rs.Rocks) {
        XES::logger().start_list("rock");
        XES::logger().add_attributes({
                {"number", i},
                {"valuable", r.Valuable},
                {"collected", r.Collected}
                });
        XES::logger().end_list();
        ++i;
    }
    XES::logger().end_list(); // end rocks
    XES::logger().end_list(); // end state
}

void ROCKSAMPLE::log_action(int action) const {
    switch (action) {
        case COORD::E_EAST:
            XES::logger().add_attribute({"action", "east"});
            return;

        case COORD::E_NORTH:
            XES::logger().add_attribute({"action", "north"});
            return;

        case COORD::E_SOUTH:
            XES::logger().add_attribute({"action", "south"});
            return;

        case COORD::E_WEST:
            XES::logger().add_attribute({"action", "west"});
            return;
        case E_SAMPLE:
            XES::logger().add_attribute({"action", "sample"});
            return;
        default:
            int rock = action - E_SAMPLE - 1;
            XES::logger().add_attribute({"action", "check " + std::to_string(rock)});
            return;
    }
}

void ROCKSAMPLE::log_observation(const STATE& state, observation_t observation) const {
    switch (observation) {
        case E_NONE:
            XES::logger().add_attribute({"observation", "none"});
            return;
        case E_GOOD:
            XES::logger().add_attribute({"observation", "good"});
            return;
        case E_BAD:
            XES::logger().add_attribute({"observation", "bad"});
            return;
    }
}

void ROCKSAMPLE::log_reward(double reward) const {
    XES::logger().add_attribute({"reward", reward});
}

void ROCKSAMPLE::set_belief_metainfo(VNODE *v, const SIMULATOR &) const {
    v->Beliefs().set_metainfo(ROCKSAMPLE_METAINFO(NumRocks), *this);
}

void ROCKSAMPLE::pre_shield(const BELIEF_STATE &belief, std::vector<int> &legal_actions) const {
    const STATE* state = belief.GetSample(0);
    const auto& rs = safe_cast<const ROCKSAMPLE_STATE&>(*state);
    const auto& meta =
        dynamic_cast<const ROCKSAMPLE_METAINFO&>(belief.get_metainfo());

    // moving is always legal
    if (rs.AgentPos.Y - 1 >= 0)
        legal_actions.push_back(COORD::E_SOUTH);
    if (rs.AgentPos.Y + 1 < Size)
        legal_actions.push_back(COORD::E_NORTH);
    if (rs.AgentPos.X - 1 >= 0)
        legal_actions.push_back(COORD::E_WEST);
    legal_actions.push_back(COORD::E_EAST);
               
    // checks are always legal
    for (int rock = 0; rock < NumRocks; ++rock) {
        int action = E_SAMPLE + 1 + rock;
        legal_actions.push_back(action);
    }

    // only sample if safe
    int rock = Grid(rs.AgentPos);
    if (rock >= 0 && !rs.Rocks[rock].Collected) {
        double p = meta.get_prob_valuable(rock);
        if (p >= sample_shield_tr)
            legal_actions.push_back(E_SAMPLE);
        else if (sampling_points.is_in_threshold({p}))
            legal_actions.push_back(E_SAMPLE);
    }
}

Classification ROCKSAMPLE::check_rule(const BELIEF_META_INFO &m, int a, double t) const {
    const ROCKSAMPLE_METAINFO &meta = safe_cast<const ROCKSAMPLE_METAINFO &>(m);

    int rock = -1;
    COORD coord(meta.x(), meta.y());
    for (int i = 0; i < NumRocks; ++i)
    {
        if (coord == RockPos[i]) {
            rock = i;
            break;
        }
    }

    if (a == E_SAMPLE) {
        if ( rock >= 0 && !meta.collected(rock) && meta.get_prob_valuable(rock) >= t)
            return TRUE_POSITIVE; // the rule correctly describes the action
        else
            return FALSE_NEGATIVE; // the rule should have described this action
    }
    else {
        if (rock == -1 || meta.collected(rock) || meta.get_prob_valuable(rock) < t)
            return TRUE_NEGATIVE; // the rule correctly avoid sampling
        else
            return FALSE_POSITIVE; // the rule falsely describe this action
    }
}

