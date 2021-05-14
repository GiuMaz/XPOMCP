#ifndef OBSTACLEAVOIDANCE_H
#define OBSTACLEAVOIDANCE_H

#include "simulator.h"
#include <unordered_map>
#include <vector>
#include <algorithm>
#include <fstream>

class OBSTACLEAVOIDANCE_STATE : public STATE
{
    public:

        std::vector<int> segDifficulties;
        int curSegI;
        int curSubsegJ;
        int dp;
        int p;
        int v;
        double dt;
        double dt1;
        double dt2;
        double t;
        int m;
        int o;
        double r;
        double r_cum;

        std::vector<int> dps;
        std::vector<int> ps;
        std::vector<int> vs;
        std::vector<int> acs;
        std::vector<double> dts;
        std::vector<double> dt1s;
        std::vector<double> dt2s;
        std::vector<double> ts;
        std::vector<int> ms;
        std::vector<int> os;
        std::vector<double> rs;
        std::vector<double> r_cums;
        std::vector<double> dist_state_beliefs;
};

class OBSTACLEAVOIDANCE : public SIMULATOR
{
    public:

        OBSTACLEAVOIDANCE(std::vector<int> nSubSegs, std::vector<std::vector<double>> subSegLengths, int nEnginePowerValues, int nDifficultyValues, int nVelocityValues);

        virtual STATE* Copy(const STATE& state) const;
        virtual void Validate(const STATE& state) const;
        virtual STATE* CreateStartState() const;
        virtual void FreeState(STATE* state) const;
        virtual bool Step(STATE& state, int action, 
                int& observation, double& reward, const BELIEF_STATE& beliefState) const;

        void GenerateLegal(const STATE& state, const HISTORY& history,
                std::vector<int>& legal, const STATUS& status) const;
        void GeneratePreferred(const STATE& state, const HISTORY& history,
                std::vector<int>& legal, const STATUS& status) const;
        virtual bool LocalMove(STATE& state, const HISTORY& history,
                int stepObservation, const STATUS& status) const;

        virtual void DisplayBeliefs(const BELIEF_STATE& beliefState,
                std::ostream& ostr) const;
        virtual void DisplayState(const STATE& state, std::ostream& ostr) const;
        virtual void DisplayObservation(const STATE& state, int observation, std::ostream& ostr) const;
        virtual void DisplayAction(int action, std::ostream& ostr) const;
        virtual double JointProb(const STATE& state) const;
        void DisplayStateId(const STATE& state, std::ostream& ostr) const;
        void DisplayStateHist(const STATE& state, const char* fileName) const;
        virtual void DisplayBeliefIds(const BELIEF_STATE& beliefState,
                std::ostream& ostr) const;
        virtual void DisplayBeliefDistribution(const BELIEF_STATE& beliefState,
                std::ostream& ostr) const;
        std::vector<double*>* CreateStateRelKnowledge(const STATE& state, int nConnComp, double relProbab) const;
        STATE* CreateStartState(std::vector<double*>* stateVarRelationships) const;
        STATE* CreateStartState(std::vector<STATE*>* allParticles, std::vector<double> allParticleCumProbs) const;
        STATE* CreateStartStateFixedValues(std::vector<int> values) const;
        virtual bool LocalMove(STATE& state, const HISTORY& history,
                int stepObservation, const STATUS& status, std::vector<double*>* stateVarRelationships) const;

        virtual void PropagateChange(STATE& state, int changedVariableIndex, std::vector<double*>* stateVarRelationships, std::vector<int>* alreadyExploredVarIndices) const;
        virtual void PropagateChangeToConnectedComponent(STATE& state, int changedVariable, int newVal, std::vector<double*>* stateVarRelationships) const;

        virtual std::vector<double*> FindUnexploredAdjacentVariableIndices(int currentVarIndex, std::vector<double*>* stateVarRelationships, std::vector<int>* alreadyExploredVarIndices) const;
        virtual double ComputeParticleProbability(STATE& particle, std::vector<double*>* stateVarRelationships) const;

    protected:
        int nSeg;
        std::vector<int> nSubSegs;
        std::vector<std::vector<double>> subSegLengths;
        int nDifficultyValues;
        int nVelocityValues;
        int collisionPenaltyTime;

    private:
        mutable MEMORY_POOL<OBSTACLEAVOIDANCE_STATE> MemoryPool;
};

#endif
