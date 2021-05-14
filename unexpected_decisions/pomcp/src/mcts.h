#ifndef MCTS_H
#define MCTS_H

#include "simulator.h"
#include "node.h"
#include "statistic.h"
#include <fstream>
#include <sys/stat.h>
using namespace std;

class MCTS
{
public:

    struct PARAMS
    {
        PARAMS();

        int Verbose;
        int MaxDepth;       // Max depth of the tree
        int NumSimulations; // Number of simulations in the simulator (MCTS)
        int NumStartStates; 
        bool UseTransforms;
        int NumTransforms;
        int MaxAttempts;
        int ExpandCount;
        double ExplorationConstant;
        bool UseRave;
        double RaveDiscount;
        double RaveConstant;
        bool DisableTree;
    };
    

    MCTS(const SIMULATOR& simulator, const PARAMS& params);
    MCTS(const SIMULATOR& simulator, const STATE* state, const PARAMS& params);
    MCTS(const SIMULATOR& simulator, std::vector<double*>* stateVarRelationships, const PARAMS& params);
    MCTS(const SIMULATOR& simulator, std::vector<double*>* stateVarRelationships, int nAllParticles, const PARAMS& params);
    ~MCTS();

    int SelectAction();
    bool Update(int action, int observation, double reward);

    void UCTSearch();
    void RolloutSearch();

    double Rollout(STATE& state);

    const BELIEF_STATE& BeliefState() const { return Root->Beliefs(); }
    const HISTORY& GetHistory() const { return History; }
    const SIMULATOR::STATUS& GetStatus() const { return Status; }
    void ClearStatistics();
    void DisplayStatistics(std::ostream& ostr) const;
    void DisplayValue(int depth, std::ostream& ostr) const;
    void DisplayPolicy(int depth, std::ostream& ostr) const;

    static void UnitTest();
    static void InitFastUCB(double exploration);
    
    bool Update(int action, int observation, double reward, std::vector<double*>* stateVarRelationships);
    void AddTransforms(VNODE* root, BELIEF_STATE& beliefs, std::vector<double*>* stateVarRelationships);
    STATE* CreateTransform(std::vector<double*>* stateVarRelationships) const;
    
private:

    const SIMULATOR& Simulator;
    int TreeDepth, PeakTreeDepth;
    PARAMS Params;
    VNODE* Root;
    HISTORY History;
    SIMULATOR::STATUS Status;

    STATISTIC StatTreeDepth;
    STATISTIC StatRolloutDepth;
    STATISTIC StatTotalReward;
    std::ofstream outFileBeliefPerStep;
    std::ofstream outFileUCT_tmp;
    
    

    int GreedyUCB(VNODE* vnode, bool ucb) const;
    int SelectRandom() const;
    double SimulateV(STATE& state, VNODE* vnode);
    double SimulateQ(STATE& state, QNODE& qnode, int action);
    void AddRave(VNODE* vnode, double totalReward);
    VNODE* ExpandNode(const STATE* state);
    void AddSample(VNODE* node, const STATE& state);
    void AddTransforms(VNODE* root, BELIEF_STATE& beliefs);
    STATE* CreateTransform() const;
    void Resample(BELIEF_STATE& beliefs);

    static const int UCB_N = 10000, UCB_n = 100;
    static double UCB[UCB_N][UCB_n];
    static bool InitialisedFastUCB;

    double FastUCB(int N, int n, double logN) const;

    static void UnitTestGreedy();
    static void UnitTestUCB();
    static void UnitTestRollout();
    static void UnitTestSearch(int depth);
    
    
    
};

#endif // MCTS_H
