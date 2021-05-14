#ifndef SIMULATOR_H
#define SIMULATOR_H

#include "history.h"
#include "node.h"
#include "utils.h"
#include <iostream>
#include <math.h>
#include <vector>

class BELIEF_STATE;

class STATE : public MEMORY_OBJECT
{
};

class SIMULATOR
{
public:

    struct KNOWLEDGE
    {
        enum
        {
            PURE,
            LEGAL,
            SMART,
            NUM_LEVELS
        };

        KNOWLEDGE();
        
        int RolloutLevel;
        int TreeLevel;
        int SmartTreeCount;
        double SmartTreeValue;
        
        int Level(int phase) const
        {
            assert(phase < STATUS::NUM_PHASES);
            if (phase == STATUS::TREE)
                return TreeLevel;
            else
                return RolloutLevel;
        }
        int relKnowLevel;
        STATE* realState;
    };

    struct STATUS   
    {
        STATUS();
        
        enum
        {
            TREE,
            ROLLOUT,
            NUM_PHASES
        };
        
        enum
        {
            CONSISTENT,
            INCONSISTENT,
            RESAMPLED,
            OUT_OF_PARTICLES
        };
        
        int Phase;
        int Particles;
    };

    SIMULATOR();
    SIMULATOR(int numActions, int numObservations, double discount = 1.0);    
    virtual ~SIMULATOR();

    virtual STATE* CreateStartState() const = 0;

    virtual void FreeState(STATE* state) const = 0;

    virtual bool Step(STATE& state, int action, 
        int& observation, double& reward, const BELIEF_STATE& beliefState) const = 0;
    
        
    virtual STATE* Copy(const STATE& state) const = 0;
    
    virtual void Validate(const STATE& state) const;

    virtual bool LocalMove(STATE& state, const HISTORY& history,
        int stepObs, const STATUS& status) const;

    void Prior(const STATE* state, const HISTORY& history, VNODE* vnode,
        const STATUS& status) const;

    int SelectRandom(const STATE& state, const HISTORY& history,
        const STATUS& status) const;

    virtual void GenerateLegal(const STATE& state, const HISTORY& history, 
        std::vector<int>& actions, const STATUS& status) const;

    virtual void GeneratePreferred(const STATE& state, const HISTORY& history, 
        std::vector<int>& actions, const STATUS& status) const;

    virtual bool HasAlpha() const;
    virtual void AlphaValue(const QNODE& qnode, double& q, int& n) const;
    virtual void UpdateAlpha(QNODE& qnode, const STATE& state) const;

    virtual void DisplayBeliefs(const BELIEF_STATE& beliefState, 
        std::ostream& ostr) const;
    virtual void DisplayState(const STATE& state, std::ostream& ostr) const;
    virtual void DisplayAction(int action, std::ostream& ostr) const;
    virtual void DisplayObservation(const STATE& state, int observation, std::ostream& ostr) const;
    virtual void DisplayReward(double reward, std::ostream& ostr) const;
    
    virtual double JointProb(const STATE& state) const;
    virtual void DisplayStateId(const STATE& state, std::ostream& ostr) const;
    virtual void DisplayStateHist(const STATE& state, const char* fileName) const;
    virtual void Path2Csv(const STATE& state) const;
    virtual int GetRelKnowLevel() const;
    virtual void DisplayBeliefIds(const BELIEF_STATE& beliefState, 
        std::ostream& ostr) const;
    virtual void DisplayBeliefDistribution(const BELIEF_STATE& beliefState, 
        std::ostream& ostr) const;
    virtual STATE* CreateStartState(std::vector<double*>* stateVarRelationships) const;
    virtual STATE* CreateStartState(std::vector<STATE*>* allParticles, std::vector<double> allParticleCumProbs) const;
    virtual STATE* CreateStartStateFixedValues(std::vector<int> values) const;
    
    virtual std::vector<double*>* CreateStateRelKnowledge(const STATE& state, int nConnComp, double relProbab) const;
    virtual bool LocalMove(STATE& state, const HISTORY& history,
        int stepObs, const STATUS& status, std::vector<double*>* stateVarRelationships) const;
    
    virtual void PropagateChange(STATE& state, int changedVariableIndex, std::vector<double*>* stateVarRelationships, 
        std::vector<int>* alreadyExploredVarIndices) const;
    virtual void PropagateChangeToConnectedComponent(STATE& state, int changedVariableIndex, int newVal, std::vector<double*>* stateVarRelationships) const;
    virtual std::vector<double*> FindUnexploredAdjacentVariableIndices(int currentVarIndex, 
        std::vector<double*>* stateVarRelationships, std::vector<int>* alreadyExploredVarIndices) const;
    virtual double ComputeParticleProbability(STATE& particle, std::vector<double*>* stateVarRelationships) const;

    void SetKnowledge(const KNOWLEDGE& knowledge) { Knowledge = knowledge; }
    int GetNumActions() const { return NumActions; }
    int GetNumObservations() const { return NumObservations; }
    bool IsEpisodic() const { return false; }
    double GetDiscount() const { return Discount; }
    double GetRewardRange() const { return RewardRange; }
    double GetHorizon(double accuracy, int undiscountedHorizon = 100) const;
    
protected:

    int NumActions, NumObservations;
    double Discount, RewardRange;
    KNOWLEDGE Knowledge;
};

#endif // SIMULATOR_H
