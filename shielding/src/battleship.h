#ifndef BATTLESHIP_H
#define BATTLESHIP_H

#include "simulator.h"
#include "grid.h"
#include <list>

struct SHIP
{
    COORD Position;
    int Direction;
    int Length;
};

class BATTLESHIP_STATE : public STATE
{
public:

    struct CELL
    {
        bool Occupied;
        bool Visited;
        bool Diagonal;
    };

    GRID<CELL> Cells;
    std::vector<SHIP> Ships;
    int NumRemaining;
};

class BATTLESHIP : public SIMULATOR
{
public:

    BATTLESHIP(int xsize = 10, int ysize = 10, int maxlength = 4);

    virtual STATE* Copy(const STATE& state) const;
    virtual void Validate(const STATE& state) const;
    virtual STATE* CreateStartState() const;
    virtual void FreeState(STATE* state) const;
    virtual bool Step(STATE& state, int action, 
        int& observation, double& reward) const;
        
    void GenerateLegal(const STATE& state, const HISTORY& history,
        std::vector<int>& legal, const STATUS& status) const;
    virtual bool LocalMove(STATE& state, const HISTORY& history,
        int stepObs, const STATUS& status) const;

    virtual void DisplayBeliefs(const BELIEF_STATE& beliefState, 
        std::ostream& ostr) const;
    virtual void DisplayState(const STATE& state, std::ostream& ostr) const;
    virtual void DisplayObservation(const STATE& state, int observation, std::ostream& ostr) const;
    virtual void DisplayAction(int action, std::ostream& ostr) const;

private:

    bool Collision(const BATTLESHIP_STATE& bsstate, const SHIP& ship) const;
    void MarkShip(BATTLESHIP_STATE& bsstate, const SHIP& ship) const;
    void UnmarkShip(BATTLESHIP_STATE& bsstate, const SHIP& ship) const;
    void UpdateLegal(BATTLESHIP_STATE& bsstate) const;
    bool MoveShips(BATTLESHIP_STATE& bsstate) const;
    bool SwitchTwoShips(BATTLESHIP_STATE& bsstate) const;
    bool SwitchThreeShips(BATTLESHIP_STATE& bsstate) const;
    
    int XSize, YSize;
    int MaxLength, TotalRemaining;
   
    mutable MEMORY_POOL<BATTLESHIP_STATE> MemoryPool;
};

#endif // BATTLESHIP_H
