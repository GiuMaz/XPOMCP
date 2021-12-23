#pragma onces

#include "simulator_lazy.h"
#include "coord.h"
#include "grid.h"

class LASERTAG_STATE : public STATE
{
public:
    COORD AgentPos;
    COORD OpponentPos;
};

class LASERTAG : public SIMULATOR_LAZY
{
public:
    LASERTAG();

    virtual STATE* Copy(const STATE& state) const;
    virtual void Validate(const STATE& state) const;
    virtual STATE* CreateStartState() const;
    virtual void FreeState(STATE* state) const;
    virtual bool Step(STATE &state, int action, observation_t &observation,
                      double &reward) const;

    void GenerateLegal(const STATE& state, const HISTORY& history,
            std::vector<int>& legal, const STATUS& status) const;
    void GeneratePreferred(const STATE& state, const HISTORY& history,
        std::vector<int>& legal, const STATUS& status) const;
    virtual bool LocalMove(STATE& state, const HISTORY& history,
        observation_t stepObs, const STATUS& status) const;

    virtual void DisplayBeliefs(const BELIEF_STATE& beliefState, 
        std::ostream& ostr) const;
    virtual void DisplayState(const STATE& state, std::ostream& ostr) const;
    virtual void DisplayObservation(const STATE& state, observation_t observation, std::ostream& ostr) const;
    virtual void DisplayAction(int action, std::ostream& ostr) const;

    // xes logging
    virtual void log_problem_info() const;
    virtual void log_beliefs(const BELIEF_STATE& beliefState) const;
    virtual void log_state(const STATE& state) const;
    virtual void log_action(int action) const;
    virtual void log_observation(const STATE& state, observation_t observation) const;
    virtual void log_reward(double reward) const;

private:

    void MoveOpponent(LASERTAG_STATE& tagstate) const;
    int GetObservation(const LASERTAG_STATE& tagstate, int action) const;
    bool Inside(const COORD& coord) const;
    COORD GetCoord(int index) const;
    int GetIndex(const COORD& coord) const;
    bool IsCorner(const COORD& coord) const;
    COORD GetRandomCorner() const;
    double LaserRange(int state, int dir) const;
    double LaserRange(COORD rob, COORD opp, int dir) const;
    void SetReading(observation_t &obs, observation_t reading, int dir) const;
    void get_obs(const LASERTAG_STATE &state, observation_t &observation) const;


    static constexpr int NumCells = 7 * 11;
    static constexpr int NBEAMS = 8;
    static constexpr int BITS_PER_READING = 7;
    observation_t same_loc_obs_;

    double noise_sigma_;
    double unit_size_;
    //std::vector<std::vector<std::vector<double> > > reading_distributions_;

    // 7x11 map
    // Y
    // |         *
    // |   *
    // | *
    // |          *
    // |    *     *
    // | *     *
    // |___________X
    std::array<bool, NumCells> obstacles;

    mutable MEMORY_POOL<LASERTAG_STATE> MemoryPool;
};
