#include "rocksample.h"

#include <unordered_map>

#include "utils.h"

using namespace std;
using namespace UTILS;

ROCKSAMPLE::ROCKSAMPLE(int size, int rocks)
:   Grid(size, size),
    Size(size),
    NumRocks(rocks),
    SmartMoveProb(0.95),
    UncertaintyCount(0)
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
    else
        InitGeneral();
}

void ROCKSAMPLE::InitGeneral()
{
    HalfEfficiencyDistance = 20;
    StartPos = COORD(0, Size / 2);
    RandomSeed(0);
    Grid.SetAllValues(-1);
    for (int i = 0; i < NumRocks; ++i)
    {
        COORD pos;
        do
        {
            pos = COORD(Random(Size), Random(Size));
        }
        while (Grid(pos) >= 0);
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
    StartPos = COORD(0, 5);
    Grid.SetAllValues(-1);
    for (int i = 0; i < NumRocks; ++i)
    {
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
    for (int i = 0; i < NumRocks; i++)
    {
        ROCKSAMPLE_STATE::ENTRY entry;
        entry.Collected = false;
        entry.Valuable = Bernoulli(0.5);
        entry.Count = 0;
        entry.Measured = 0;
        entry.ProbValuable = 0.5;
        entry.LikelihoodValuable = 1.0;
        entry.LikelihoodWorthless = 1.0;
        rockstate->Rocks.push_back(entry);
    }
    rockstate->Target = SelectTarget(*rockstate);
    return rockstate;
}

void ROCKSAMPLE::FreeState(STATE* state) const
{
    ROCKSAMPLE_STATE* rockstate = safe_cast<ROCKSAMPLE_STATE*>(state);
    MemoryPool.Free(rockstate);
}

// return true if the state is final
bool ROCKSAMPLE::Step(STATE& state, int action,
    int& observation, double& reward) const
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

bool ROCKSAMPLE::LocalMove(STATE& state, const HISTORY& history,
    int stepObs, const STATUS& status) const
{
    ROCKSAMPLE_STATE& rockstate = safe_cast<ROCKSAMPLE_STATE&>(state);
    int rock = Random(NumRocks);
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

	if (UseBlindPolicy)
	{
		actions.push_back(COORD::E_EAST);
		return;
	}

	const ROCKSAMPLE_STATE& rockstate =
	        safe_cast<const ROCKSAMPLE_STATE&>(state);

	// Sample rocks with more +ve than -ve observations
	int rock = Grid(rockstate.AgentPos);
	if (rock >= 0 && !rockstate.Rocks[rock].Collected)
	{
		int total = 0;
		for (int t = 0; t < history.Size(); ++t)
		{
			if (history[t].Action == rock + 1 + E_SAMPLE)
			{
				if (history[t].Observation == E_GOOD)
					total++;
				if (history[t].Observation == E_BAD)
					total--;
			}
		}
		if (total > 0)
		{
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

	for (int rock = 0; rock < NumRocks; ++rock)
	{
		const ROCKSAMPLE_STATE::ENTRY& entry = rockstate.Rocks[rock];
		if (!entry.Collected)
		{
			int total = 0;
			for (int t = 0; t < history.Size(); ++t)
			{
				if (history[t].Action == rock + 1 + E_SAMPLE)
				{
					if (history[t].Observation == E_GOOD)
						total++;
					if (history[t].Observation == E_BAD)
						total--;
				}
			}

			if (total >= 0)
			{
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
	if (all_bad)
	{
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


	for (rock = 0; rock < NumRocks; ++rock)
	{
		if (!rockstate.Rocks[rock].Collected    &&
			rockstate.Rocks[rock].ProbValuable != 0.0 &&
			rockstate.Rocks[rock].ProbValuable != 1.0 &&
			rockstate.Rocks[rock].Measured < 5  &&
			std::abs(rockstate.Rocks[rock].Count) < 2)
		{
			actions.push_back(rock + 1 + E_SAMPLE);
		}
	}
}

int ROCKSAMPLE::GetObservation(const ROCKSAMPLE_STATE& rockstate, int rock) const
{
    double distance = COORD::EuclideanDistance(rockstate.AgentPos, RockPos[rock]);
    double efficiency = (1 + pow(2, -distance / HalfEfficiencyDistance)) * 0.5;

    if (Bernoulli(efficiency))
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

void ROCKSAMPLE::DisplayObservation(const STATE& state, int observation, std::ostream& ostr) const
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

void ROCKSAMPLE::log_problem_info(xes_logger &xes) const {
    xes.add_attributes({
            {"problem", "rocksample"},
            {"RewardRange", RewardRange}
            // TODO: extra info...
        });
}

void ROCKSAMPLE::log_beliefs(const BELIEF_STATE& beliefState, xes_logger &xes) const {
    std::unordered_map<int, int> dist;
    int id;
    for (int i = 0; i < beliefState.GetNumSamples(); i++){
        const STATE* state = beliefState.GetSample(i);
        const auto& rockstate = safe_cast<const ROCKSAMPLE_STATE&>(*state);
        
        // Compute state id
        id=0;   
        for (int j = 0; j<rockstate.Rocks.size(); j++) 
        {
            id+=rockstate.Rocks[j].Valuable*(pow(2,j));
        }
        // If it is not in dist then add it and initialize to 1
        if (dist.find(id) == dist.end())
            dist[id]= 1;
        else
            dist[id]++;
    }

    const STATE* state = beliefState.GetSample(0);
    const auto& rs = safe_cast<const ROCKSAMPLE_STATE&>(*state);
    xes.add_attribute({"coord x", rs.AgentPos.X });
    xes.add_attribute({"coord y", rs.AgentPos.Y });
    xes.start_list("belief");
    for (const auto &[k, v] : dist)
        xes.add_attribute({std::to_string(k), v});
    xes.end_list();
}

void ROCKSAMPLE::log_state(const STATE& state, xes_logger &xes) const {
    const ROCKSAMPLE_STATE& rs = safe_cast<const ROCKSAMPLE_STATE&>(state);
    xes.start_list("state");
    xes.add_attribute({"coord x", rs.AgentPos.X });
    xes.add_attribute({"coord y", rs.AgentPos.Y });
    xes.start_list("rocks");
    int i = 0;
    for (const auto &r : rs.Rocks) {
        xes.start_list("rock");
        xes.add_attributes({
                {"number", i},
                {"valuable", r.Valuable},
                {"collected", r.Collected}
                });
        xes.end_list();
        ++i;
    }
    xes.end_list(); // end rocks
    xes.end_list(); // end state
}

void ROCKSAMPLE::log_action(int action, xes_logger &xes) const {
    switch (action) {
        case COORD::E_EAST:
            xes.add_attribute({"action", "east"});
            return;

        case COORD::E_NORTH:
            xes.add_attribute({"action", "north"});
            return;

        case COORD::E_SOUTH:
            xes.add_attribute({"action", "south"});
            return;

        case COORD::E_WEST:
            xes.add_attribute({"action", "west"});
            return;
        case E_SAMPLE:
            xes.add_attribute({"action", "sample"});
            return;
        default:
            int rock = action - E_SAMPLE - 1;
            xes.add_attribute({"action", "check " + std::to_string(rock)});
            return;
    }
}

void ROCKSAMPLE::log_observation(const STATE& state, int observation, xes_logger &xes) const {
    switch (observation) {
        case E_NONE:
            xes.add_attribute({"observation", "none"});
            return;
        case E_GOOD:
            xes.add_attribute({"observation", "good"});
            return;
        case E_BAD:
            xes.add_attribute({"observation", "bad"});
            return;
    }
}

void ROCKSAMPLE::log_reward(double reward, xes_logger &xes) const {
    xes.add_attribute({"reward", reward});
}

void ROCKSAMPLE::pre_shield(const BELIEF_STATE &belief, std::vector<int> &legal_actions) const {
    const STATE* state = belief.GetSample(0);
    const auto& rs = safe_cast<const ROCKSAMPLE_STATE&>(*state);

    // forbidden position (3, 3)
    if (rs.AgentPos.X != 2 || rs.AgentPos.Y != 3)
        legal_actions.push_back(COORD::E_EAST);
    if (rs.AgentPos.X != 3 || rs.AgentPos.Y != 2)
        legal_actions.push_back(COORD::E_NORTH);
    if (rs.AgentPos.X != 3 || rs.AgentPos.Y != 4)
        legal_actions.push_back(COORD::E_SOUTH);
    if (rs.AgentPos.X != 4 || rs.AgentPos.Y != 2)
        legal_actions.push_back(COORD::E_WEST);
               
    // sample is always legal
    legal_actions.push_back(E_SAMPLE);

    // sample is always legal
    for (int rock = 0; rock < NumRocks; ++rock) {
        int action = E_SAMPLE + 1 + rock;
        legal_actions.push_back(action);
    }
}
