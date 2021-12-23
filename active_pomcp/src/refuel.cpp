#include "refuel.h"

REFUEL::REFUEL(int s, int e)
    : Grid(s, s), size(s), energy(e),
      stations({{0, 0}, {s / 3, s / 3}, {2 * (s / 3) - 1, 2 * (s / 3) - 1}}),
      obstacles({{s - 2, s - 2}}), start({0, 0}), target({s - 1, s - 1}) {
        NumActions = 5;
        //NumObservations = (1<<7)-1;
        NumObservations = 4;
        RewardRange = 20;
        Discount = 0.95;
      }

bool REFUEL::Step(STATE& state, int action,
    observation_t& observation, double& reward) const {
    auto & rs = safe_cast<REFUEL_STATE&>(state);

    bool crash = false;
    if (action < E_REFUEL) {
        bool slip = UTILS::Bernoulli(slipery);

        if (action == COORD::E_NORTH) {
            rs.AgentPos.Y -= slip ? 2 : 1;
            if(rs.AgentPos.Y < 0)
               rs.AgentPos.Y  = 0;
        }
        else if (action == COORD::E_SOUTH) {
            rs.AgentPos.Y += slip ? 2 : 1;
            if(rs.AgentPos.Y > (size-1))
               rs.AgentPos.Y  = size-1;
        }
        else if (action == COORD::E_EAST) {
            rs.AgentPos.X += slip ? 2 : 1;
            if(rs.AgentPos.X > (size-1))
               rs.AgentPos.X  = size-1;
        }
        else if (action == COORD::E_WEST) {
            rs.AgentPos.X -= slip ? 2 : 1;
            if(rs.AgentPos.X < 0)
               rs.AgentPos.X  = 0;
        }

        // FUEL
        if (UTILS::Bernoulli(0.7)) {
            rs.fuel = std::max(0, rs.fuel-1);
        }

        // REWARD
        if (std::find(obstacles.begin(), obstacles.end(), rs.AgentPos) !=
            obstacles.end()) { // CRASH!
            reward = -100;
            crash = true;
        }
        else if (rs.AgentPos == target)
            reward = 100;
        else if (rs.fuel == 0 && !can_refuel(rs))
            reward = -100;
        else
            reward = -1;

    }

    else if (action == E_REFUEL) {
        if (can_refuel(rs)) {
            reward = -3;
            rs.fuel = energy;
        }
        else
            reward = -100;
    }

    REFUEL_OBS obs;
    obs.can_refuel = can_refuel(rs);
    obs.fuel_empty = (rs.fuel == 0);
    //obs.north_enabled = rs.AgentPos.Y > 0;
    //obs.south_enabled = rs.AgentPos.Y < size;
    //obs.east_enabled = rs.AgentPos.X < size;
    //obs.west_enabled = rs.AgentPos.X > 0;
    observation = obs.to_int();

    return rs.AgentPos == target || (rs.fuel == 0 && !can_refuel(rs)) || crash;
}

STATE* REFUEL::Copy(const STATE& state) const
{
    const REFUEL_STATE& rs = safe_cast<const REFUEL_STATE&>(state);
    REFUEL_STATE* newstate = MemoryPool.Allocate();
    *newstate = rs;
    return newstate;
}

void REFUEL::Validate(const STATE& state) const
{
    const REFUEL_STATE& rs = safe_cast<const REFUEL_STATE&>(state);
    assert(Grid.Inside(rs.AgentPos));
    assert(rs.fuel >= 0 && rs.fuel <= energy);
}

void REFUEL::FreeState(STATE* state) const
{
    REFUEL_STATE* rs = safe_cast<REFUEL_STATE*>(state);
    MemoryPool.Free(rs);
}

STATE* REFUEL::CreateStartState() const
{
    REFUEL_STATE* rs = MemoryPool.Allocate();
    rs->AgentPos = COORD(0, 0);
    rs->fuel = energy;
    return rs;
}

void REFUEL::GenerateLegal(const STATE& state, const HISTORY& history,
    std::vector<int>& legal, const STATUS& status) const
{
    const REFUEL_STATE& rs =
        safe_cast<const REFUEL_STATE&>(state);

    if (rs.AgentPos.Y != 0)
        legal.push_back(COORD::E_NORTH);
    if (rs.AgentPos.Y != (size-1))
        legal.push_back(COORD::E_SOUTH);
    if (rs.AgentPos.X != 0)
        legal.push_back(COORD::E_WEST);
    if (rs.AgentPos.X != (size-1))
        legal.push_back(COORD::E_EAST);

    if (can_refuel(rs))
        legal.push_back(E_REFUEL);
}

void REFUEL::GeneratePreferred(const STATE& state, const HISTORY& history,
    std::vector<int>& actions, const STATUS& status) const {
    // TODO
    GenerateLegal(state, history, actions, status);
}

bool REFUEL::LocalMove(STATE& state, const HISTORY& history,
    int stepObs, const STATUS& status) const
{
    // TODO
    /*
    REFUEL_STATE& rockstate = safe_cast<REFUEL_STATE&>(state);
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
    */
    return false;
}

void REFUEL::DisplayBeliefs(const BELIEF_STATE& beliefState,
    std::ostream& ostr) const {
}

void REFUEL::DisplayState(const STATE& state, std::ostream& ostr) const {
    const REFUEL_STATE& rs = safe_cast<const REFUEL_STATE&>(state);
    ostr << "Position: (" << rs.AgentPos.X << ", " << rs.AgentPos.Y
         << ") Fuel: " << rs.fuel << std::endl;
}

void REFUEL::DisplayObservation(const STATE& state, observation_t observation, std::ostream& ostr) const {
    REFUEL_OBS o(observation);
    ostr << "can refuel: " << o.can_refuel << " fuel empty: " << o.fuel_empty << std::endl;
}
/*
void REFUEL::DisplayObservation(const STATE& state, observation_t observation, std::ostream& ostr) const {
    REFUEL_OBS o(observation);
    ostr << "can refuel: " << o.can_refuel << " fuel empty: " << o.fuel_empty
        << " directions: [N:" << o.north_enabled << ", S:" << o.south_enabled <<
        ", E:" << o.east_enabled << ", W:" << o.west_enabled << "]" << std::endl;
}
*/

void REFUEL::DisplayAction(int action, std::ostream& ostr) const {
    if (action < E_REFUEL)
        ostr << COORD::CompassString[action] << std::endl;
    else if (action == E_REFUEL)
        ostr << "REFUEL" << std::endl;
}
