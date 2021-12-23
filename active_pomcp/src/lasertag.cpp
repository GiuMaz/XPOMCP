#include "lasertag.h"
#include <limits>

using namespace std;
using namespace UTILS;

double gausscdf(double x, double mean, double sigma) {
    return 0.5 * (1 + erf((x - mean) / (sqrt(2) * sigma)));
}

LASERTAG::LASERTAG():
    noise_sigma_(2.5),
    unit_size_(1.0)
{
    NumActions = 5;
    NumObservations = (1ll << 48);
    RewardRange = 10;
    Discount = 0.95;

    for (int i = 0; i < NBEAMS; i++)
        SetReading(same_loc_obs_, 101, i);

    int total = 0;
    while (total < 8) {
        int obst = Random(NumCells);
        if (!obstacles[obst]) {
            obstacles[obst] = true;
            total++;
        }
    }
}

void LASERTAG::SetReading(observation_t &obs, observation_t reading, int dir) const {
  // Clear bits
  obs &= ~(((1ll << BITS_PER_READING) - 1) << (dir * BITS_PER_READING));
  // Set bits
  obs |= reading << (dir * BITS_PER_READING);
}

double LASERTAG::LaserRange(int stateid, int dir) const {
    COORD rob = GetCoord(stateid % NumCells);
    COORD opp = GetCoord(stateid / NumCells);
    int d = 1;
    while (true) {
        COORD coord = rob + COORD::Compass[dir] * d;
        if (!Inside(coord) || obstacles[GetIndex(coord)] || coord == opp)
            return COORD::EuclideanDistance(rob, coord);
        d++;
    }
}

double LASERTAG::LaserRange(COORD rob, COORD opp, int dir) const {
    int d = 1;
    while (true) {
        COORD coord = rob + COORD::Compass[dir] * d;
        if (!Inside(coord) || obstacles[GetIndex(coord)] || coord == opp)
            return COORD::EuclideanDistance(rob, coord);
        d++;
    }
}

STATE* LASERTAG::Copy(const STATE& state) const
{
    const LASERTAG_STATE& tagstate = safe_cast<const LASERTAG_STATE&>(state);
    LASERTAG_STATE* newstate = MemoryPool.Allocate();
    *newstate = tagstate;
    return newstate; 
}

void LASERTAG::Validate(const STATE& state) const
{
    const LASERTAG_STATE& tagstate = safe_cast<const LASERTAG_STATE&>(state);
    //std::cout << "======================" << tagstate.AgentPos.X << " " << tagstate.AgentPos.Y << std::endl;
    assert(Inside(tagstate.AgentPos));
}

STATE* LASERTAG::CreateStartState() const
{
    LASERTAG_STATE* tagstate = MemoryPool.Allocate();
    tagstate->AgentPos = GetCoord(Random(NumCells));
    tagstate->OpponentPos = GetCoord(Random(NumCells));
    return tagstate;
}

void LASERTAG::FreeState(STATE* state) const
{
    LASERTAG_STATE* tagstate = safe_cast<LASERTAG_STATE*>(state);
    MemoryPool.Free(tagstate);
}

bool LASERTAG::Step(STATE& state, int action, observation_t& observation, double& reward) const
{
    LASERTAG_STATE& tagstate = safe_cast<LASERTAG_STATE&>(state);
    bool terminal = false;

    // LASERTAG action
    if (action == 4) // LASERTAG
    {
        if (tagstate.OpponentPos == tagstate.AgentPos) {
            reward = 10;
            terminal = true;
        }
        else
            reward = -10;
    }

    // Move opponents
    MoveOpponent(tagstate);

    // Move action
    if (action < 4) {
        reward = -1;
        COORD nextpos = tagstate.AgentPos + COORD::Compass[action];
        if (Inside(nextpos) && !obstacles[GetIndex(nextpos)])
            tagstate.AgentPos = nextpos;
    }
    
    if (terminal || tagstate.AgentPos == tagstate.OpponentPos)
        observation = same_loc_obs_;
    else
        get_obs(tagstate, observation);

    //std::cout << observation << std::endl;
    return terminal;
}

inline int LASERTAG::GetObservation(const LASERTAG_STATE& tagstate, int action) const
{
    SIMULATOR_LAZY::observation_t observation;
    get_obs(tagstate, observation);
    return observation;
}

inline bool LASERTAG::Inside(const COORD& coord) const {
        return coord.X >= 0 && coord.X < 11 && coord.Y >= 0 && coord.Y < 7;
}

inline COORD LASERTAG::GetCoord(int index) const {
    assert(index >= 0 && index < 11*7);
    return COORD(index / 7, index % 7);
}

inline int LASERTAG::GetIndex(const COORD& coord) const {
    assert(coord.X >= 0 && coord.X < 11 && coord.Y >= 0 && coord.Y < 7);
    return coord.X * 7 + coord.Y;
}

inline bool LASERTAG::IsCorner(const COORD& coord) const
{
    return (coord.X == 0 && coord.Y == 0 ||
            coord.X == 0 && coord.Y == 6 ||
            coord.X == 10 && coord.Y == 0 ||
            coord.X == 10 && coord.Y == 6);
}

inline COORD LASERTAG::GetRandomCorner() const
{
    switch(Random(4)) {
        case 0: return COORD(0, 0);
        case 1: return COORD(0, 6);
        case 2: return COORD(10, 0);
        default: return COORD(10, 6);
    }
}
    
void LASERTAG::MoveOpponent(LASERTAG_STATE& tagstate) const
{
    const COORD& agent = tagstate.AgentPos;
    COORD& opponent = tagstate.OpponentPos;
    
    static vector<int> actions;
    actions.clear();

    if (opponent.X >= agent.X)
        actions.push_back(COORD::E_EAST);
    if (opponent.Y == agent.Y && opponent.X > agent.X)
        actions.push_back(COORD::E_EAST);

    if (opponent.Y >= agent.Y)
        actions.push_back(COORD::E_NORTH);
    if (opponent.X == agent.X && opponent.Y > agent.Y)
        actions.push_back(COORD::E_NORTH);

    if (opponent.X <= agent.X)
        actions.push_back(COORD::E_WEST);
    if (opponent.Y == agent.Y && opponent.X < agent.X)
        actions.push_back(COORD::E_WEST);

    if (opponent.Y <= agent.Y)
        actions.push_back(COORD::E_SOUTH);
    if (opponent.X == agent.X && opponent.Y < agent.Y)
        actions.push_back(COORD::E_SOUTH);
    
    assert(!actions.empty());
    if (Bernoulli(0.8))
    {
        int d = actions[Random(actions.size())];
        COORD nextpos = opponent + COORD::Compass[d];
        if (Inside(nextpos) && !obstacles[GetIndex(nextpos)])
            opponent = nextpos;
    }
}

bool LASERTAG::LocalMove(STATE& state, const HISTORY& history,
    observation_t stepObs, const STATUS& status) const
{
    LASERTAG_STATE& tagstate = safe_cast<LASERTAG_STATE&>(state);

    tagstate.OpponentPos = GetCoord(Random(NumCells));

    int realObs = history.Back().Observation;
    if (realObs != same_loc_obs_)
        tagstate.AgentPos = GetCoord(realObs);
    int simObs = GetObservation(tagstate, history.Back().Action);

    return simObs == realObs;
}

void LASERTAG::GenerateLegal(const STATE& state, const HISTORY& history,
    vector<int>& actions, const STATUS& status) const
{
    // If history is empty then we don't know where we are yet
    if (history.Size() == 0)
        return;

    const LASERTAG_STATE& tagstate = safe_cast<const LASERTAG_STATE&>(state);

    // If we just saw an opponent and we are in a corner then LASERTAG
    if (history.Back().Observation == same_loc_obs_) {
        actions.push_back(4);
    }
    
    // Don't double back and don't go into walls
    for (int d = 0; d < 4; ++d) {
        auto nextpos = tagstate.AgentPos + COORD::Compass[d];
        if (Inside(nextpos) && !obstacles[GetIndex(nextpos)])
            actions.push_back(d);
    }
}

void LASERTAG::GeneratePreferred(const STATE& state, const HISTORY& history,
    vector<int>& actions, const STATUS& status) const
{
    const LASERTAG_STATE& tagstate = safe_cast<const LASERTAG_STATE&>(state);
    
    // If history is empty then we don't know where we are yet
    if (history.Size() == 0)
        return;

    // If we just saw an opponent and we are in a corner then LASERTAG
    if (history.Back().Observation == same_loc_obs_ /*&& IsCorner(tagstate.AgentPos)*/)
    {
        actions.push_back(4);
        return;
    }
    
    // Don't double back and don't go into walls
    for (int d = 0; d < 4; ++d) {
        if (history.Back().Action == COORD::Opposite(d))
            continue;

        auto nextpos = tagstate.AgentPos + COORD::Compass[d];
        if (Inside(nextpos) && !obstacles[GetIndex(nextpos)])
            actions.push_back(d);
    }
}

void LASERTAG::DisplayBeliefs(const BELIEF_STATE& beliefState, 
    std::ostream& ostr) const
{
}

void LASERTAG::DisplayState(const STATE& state, std::ostream& ostr) const
{
    const LASERTAG_STATE& tagstate = safe_cast<const LASERTAG_STATE&>(state);
    GRID<char> cgrid(11, 7);

    cgrid.SetAllValues('.');

    if (tagstate.OpponentPos == tagstate.AgentPos) {
        cgrid(tagstate.OpponentPos) = '*';
    }
    else {
        cgrid(tagstate.OpponentPos) = 'T';
        cgrid(tagstate.AgentPos) = 'A';
    }
     

    for (int y = 0; y < 7; y++) {
        for (int x = 0; x < 11; x++) {
            if (obstacles[GetIndex(COORD(x, y))])
                cgrid(COORD(x, y)) = '#';
        }
    }

    for (int y = 6; y >= 0; y--)
    {
        for (int x = 0; x < 11; x++)
        {
            COORD pos(x, y);
            if (Inside(pos))
            {
                ostr << cgrid(pos) << ' ';
            }
            else
            {
                ostr << "  ";
            }
        }
        ostr << endl;
    }
}

void LASERTAG::DisplayObservation(const STATE& state, observation_t observation, std::ostream& ostr) const
{
    // TODO: arricchisci stampa
    ostr << "Observation " << observation << endl;
}

void LASERTAG::DisplayAction(int action, std::ostream& ostr) const
{
    if (action < 4)
        ostr << COORD::CompassString[action] << endl;
    else
        ostr << "LASERTAG" << endl;
}

void LASERTAG::log_problem_info() const {
    XES::logger().add_attributes({
            {"problem", "LASERTAG"},
            {"RewardRange", RewardRange},
            {"NumCells", NumCells},
        });
}

void LASERTAG::log_beliefs(const BELIEF_STATE& beliefState) const {
    std::unordered_map<int, int> dist;
    int id;
    for (int i = 0; i < beliefState.GetNumSamples(); i++){
        const STATE* state = beliefState.GetSample(i);
        const auto& tagstate = safe_cast<const LASERTAG_STATE&>(*state);
        
        // Compute state id
        id=0;
        if (tagstate.OpponentPos == COORD::Null) {
            id *= (NumCells+1);
            id += NumCells;
        }
        else {
            id *= (NumCells+1);
            assert(GetIndex(tagstate.OpponentPos) < NumCells);
            id += GetIndex(tagstate.OpponentPos);
        }
        // If it is not in dist then add it and initialize to 1
        if (dist.find(id) == dist.end())
            dist[id]= 1;
        else
            dist[id]++;
    }

    const STATE* state = beliefState.GetSample(0);
    const auto& ts = safe_cast<const LASERTAG_STATE&>(*state);
    XES::logger().add_attribute({"agent x", ts.AgentPos.X });
    XES::logger().add_attribute({"agent y", ts.AgentPos.Y });
    XES::logger().add_attribute({"agent index", GetIndex(ts.AgentPos)});
    XES::logger().start_list("belief");
    for (const auto &[k, v] : dist)
        XES::logger().add_attribute({std::to_string(k), v});
    XES::logger().end_list();
}

void LASERTAG::log_state(const STATE& state) const {
    const auto& ts = safe_cast<const LASERTAG_STATE&>(state);
    XES::logger().start_list("state");
    XES::logger().add_attribute({"agent x", ts.AgentPos.X });
    XES::logger().add_attribute({"agent y", ts.AgentPos.Y });

    XES::logger().add_attribute({"opponent x", ts.OpponentPos.X});
    XES::logger().add_attribute({"opponent y", ts.OpponentPos.Y});

    XES::logger().end_list(); // end state
}

void LASERTAG::log_action(int action) const {
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
        case 4:
            XES::logger().add_attribute({"action", "LASERTAG"});
            return;
    }
}

void LASERTAG::log_observation(const STATE& state, observation_t observation) const {
    XES::logger().add_attribute({"observation", observation});
}

void LASERTAG::log_reward(double reward) const {
    XES::logger().add_attribute({"reward", reward});
}

void LASERTAG::get_obs(const LASERTAG_STATE &state, observation_t &observation) const {
    static std::array<std::vector<double>,NBEAMS> distribution;

    for (int d = 0; d < NBEAMS; d++) {
        distribution[d].clear();

        double dist = LaserRange(state.AgentPos,state.OpponentPos, d);
        for (int reading = 0; reading < (dist/unit_size_); reading++) {
            double min_noise = reading * unit_size_ - dist;
            double max_noise = min(dist, (reading + 1) * unit_size_) - dist;
            double prob = 2 * (gausscdf(max_noise, 0, noise_sigma_) -
                    (reading > 0 ? gausscdf(min_noise, 0, noise_sigma_) : 0));
            distribution[d].push_back(prob);
        }
    }

    observation = 0;
    for (int dir = 0; dir < NBEAMS; dir++) {
        double mass = RandomDouble(0.0, 1.0);
        int reading = 0;
        for (; reading < distribution[dir].size(); reading++) {
            mass -= distribution[dir][reading];
            if (mass < 1e-8) break;
        }
        SetReading(observation, reading, dir);
    }
}
