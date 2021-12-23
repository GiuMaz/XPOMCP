#pragma once

#include "simulator_lazy.h"
#include "node_lazy.h"
#include "statistic.h"
#include "mcts.h"

class MCTS_LAZY
{
public:

    using PARAMS = MCTS::PARAMS;

    MCTS_LAZY(const SIMULATOR_LAZY& simulator, const PARAMS& params);
    ~MCTS_LAZY();

    int SelectAction();
    bool Update(int action, SIMULATOR_LAZY::observation_t observation, double reward);

    void UCTSearch();
    void RolloutSearch();

    double Rollout(STATE& state);

    const BELIEF_STATE& BeliefState() const { return Root->Beliefs(); }
    const HISTORY& GetHistory() const { return History; }
    const SIMULATOR_LAZY::STATUS& GetStatus() const { return Status; }
    void ClearStatistics();
    void DisplayStatistics(std::ostream& ostr) const;
    void DisplayValue(int depth, std::ostream& ostr) const;
    void DisplayPolicy(int depth, std::ostream& ostr) const;

    const VNODE_LAZY * const GetRoot() const { return Root; }

    static void UnitTest();
    static void InitFastUCB(double exploration);

private:

    const SIMULATOR_LAZY& Simulator;
    int TreeDepth, PeakTreeDepth;
    PARAMS Params;
    VNODE_LAZY* Root;
    HISTORY History;
    SIMULATOR_LAZY::STATUS Status;

    STATISTIC StatTreeDepth;
    STATISTIC StatRolloutDepth;
    STATISTIC StatTotalReward;

    std::vector<int> legal_actions;

    int GreedyUCB(VNODE_LAZY* vnode, bool ucb) const;
    int SelectRandom() const;
    double SimulateV(STATE& state, VNODE_LAZY* vnode);
    double SimulateQ(STATE& state, QNODE_LAZY& qnode, int action);
    void AddRave(VNODE_LAZY* vnode, double totalReward);
    VNODE_LAZY* ExpandNode(const STATE* state);
    void AddSample(VNODE_LAZY* node, const STATE& state);
    void AddTransforms(VNODE_LAZY* root, BELIEF_STATE& beliefs);
    STATE* CreateTransform() const;
    void Resample(BELIEF_STATE& beliefs);

    // Fast lookup table for UCB
    static const int UCB_N = 10000, UCB_n = 100;
    static double UCB[UCB_N][UCB_n];
    static bool InitialisedFastUCB;

    double FastUCB(int N, int n, double logN) const;

};

