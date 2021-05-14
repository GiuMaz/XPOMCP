#ifndef EXPERIMENT_H
#define EXPERIMENT_H

#include "mcts.h"
#include "simulator.h"
#include "statistic.h"
#include "XES_logger.h"
#include <fstream>

//----------------------------------------------------------------------------

struct RESULTS
{
    void Clear();

    STATISTIC Time;
    STATISTIC Reward;
    STATISTIC DiscountedReturn;
    STATISTIC UndiscountedReturn;
};

inline void RESULTS::Clear()
{
    Time.Clear();
    Reward.Clear();
    DiscountedReturn.Clear();
    UndiscountedReturn.Clear();
}

//----------------------------------------------------------------------------

class EXPERIMENT
{
public:

    struct PARAMS
    {
        PARAMS();
        
        int NumRuns;
        int NumSteps;
        int SimSteps;
        double TimeOut;
        int MinDoubles, MaxDoubles;
        int TransformDoubles;
        int TransformAttempts;
        double Accuracy;
        int UndiscountedHorizon;
        bool AutoExploration;
    };

    EXPERIMENT(const SIMULATOR& real, const SIMULATOR& simulator, 
        const std::string& outputFile, 
        EXPERIMENT::PARAMS& expParams, MCTS::PARAMS& searchParams,
        std::string xes_file = "log.xes");

    void set_fixed_seed(int s) {
        use_fixed_seed = true;
        fixed_seed = s;
    }

    void Run();
    void MultiRun();
    void DiscountedReturn();
    void AverageReward();

private:
    const SIMULATOR& Real;
    const SIMULATOR& Simulator;
    EXPERIMENT::PARAMS& ExpParams;
    MCTS::PARAMS& SearchParams;
    RESULTS Results;
    xes_logger logger;

    bool use_fixed_seed = false;
    int fixed_seed = -1;

    int real_seed;
    std::ofstream OutputFile;
};

//----------------------------------------------------------------------------

#endif // EXPERIMENT_H
