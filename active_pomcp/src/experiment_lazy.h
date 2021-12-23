#pragma once

#include "mcts.h"
#include "mcts_lazy.h"
#include "simulator.h"
#include "simulator_lazy.h"
#include "statistic.h"
#include "XES_logger.h"
#include "experiment.h"
#include <fstream>


class EXPERIMENT_LAZY
{
public:

    using PARAMS = EXPERIMENT::PARAMS;

    EXPERIMENT_LAZY(const SIMULATOR_LAZY& real, const SIMULATOR_LAZY& simulator, 
        const std::string& outputFile, 
        EXPERIMENT_LAZY::PARAMS& expParams, MCTS_LAZY::PARAMS& searchParams);

    void set_fixed_seed(int s) {
        use_fixed_seed = true;
        fixed_seed = s;
    }

    void Run();
    void MultiRun();
    void DiscountedReturn();
    void AverageReward();

private:
    const SIMULATOR_LAZY& Real;
    const SIMULATOR_LAZY& Simulator;
    EXPERIMENT_LAZY::PARAMS& ExpParams;
    MCTS_LAZY::PARAMS& SearchParams;
    RESULTS Results;

    bool use_fixed_seed = false;
    int fixed_seed = -1;
    int real_seed;
    std::ofstream OutputFile;
};

//----------------------------------------------------------------------------
