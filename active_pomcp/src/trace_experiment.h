#pragma once

#include "XES_logger.h"
#include "experiment.h"
#include "mcts.h"
#include "simulator.h"
#include "statistic.h"
#include "trace.h"
#include <fstream>

//----------------------------------------------------------------------------

/*
 * Run an experiment that build a specific trace.
 * To be used for rule generation.
 */
template<typename T>
class TRACE_EXPERIMENT
{
public:
    using trace_t = T;

    struct PARAMS
    {
        PARAMS();
        
        int NumRuns;
        int NumSteps;
        int SimSteps;
        double TimeOut;
        int PowParticles;
        int TransformDoubles;
        int TransformAttempts;
        double Accuracy;
        int UndiscountedHorizon;
        bool AutoExploration;
    };

    TRACE_EXPERIMENT(const SIMULATOR &real, const SIMULATOR &simulator,
                    TRACE_EXPERIMENT::PARAMS &expParams,
                    MCTS::PARAMS &searchParams);

    void set_fixed_seed(int s) {
        use_fixed_seed = true;
        fixed_seed = s;
    }

    void BuildTrace(trace_t & trace);

private:
    const SIMULATOR& Real;
    const SIMULATOR& Simulator;
    TRACE_EXPERIMENT::PARAMS& ExpParams;
    MCTS::PARAMS& SearchParams;
    RESULTS Results;

    bool use_fixed_seed = false;
    int fixed_seed = -1;
    int real_seed;

    void Run(trace_t &trace);
    void MultiRun(trace_t &trace);
};

template<typename T>
TRACE_EXPERIMENT<T>::PARAMS::PARAMS()
:   NumRuns(1000),
    NumSteps(100000),
    SimSteps(1000),
    TimeOut(3600),
    PowParticles(15),
    TransformDoubles(-4),
    TransformAttempts(1000),
    Accuracy(0.01),
    UndiscountedHorizon(1000),
    AutoExploration(true) {
}

template<typename T>
TRACE_EXPERIMENT<T>::TRACE_EXPERIMENT(const SIMULATOR &real,
                                 const SIMULATOR &simulator,
                                 TRACE_EXPERIMENT::PARAMS &expParams,
                                 MCTS::PARAMS &searchParams)
    : Real(real), Simulator(simulator),
      ExpParams(expParams), SearchParams(searchParams) {
    if (ExpParams.AutoExploration) {
        if (SearchParams.UseRave)
            SearchParams.ExplorationConstant = 0;
        else
            SearchParams.ExplorationConstant = simulator.GetRewardRange();
    }
    MCTS::InitFastUCB(SearchParams.ExplorationConstant);
}

template<typename T>
void TRACE_EXPERIMENT<T>::Run(trace_t &trace)
{
    Timer timer;

    MCTS mcts(Simulator, SearchParams);

    double undiscountedReturn = 0.0;
    double discountedReturn = 0.0;
    double discount = 1.0;
    bool terminal = false;
    bool outOfParticles = false;
    int t;

    if (use_fixed_seed) {
        Real.set_seed(real_seed);
        real_seed++;
    }
    STATE* state = Real.CreateStartState();

    if (SearchParams.Verbose >= 1)
        Real.DisplayState(*state, std::cout);

    for (t = 0; t < ExpParams.NumSteps; t++)
    {
        if (XES::enabled())
            XES::logger().start_event();

        SIMULATOR::observation_t observation;
        double reward;

        trace.start_step();
        trace.save_belief(mcts.BeliefState());

        if (XES::enabled()) {
            Simulator.log_beliefs(mcts.BeliefState());
        }

        // selecte best action using thes simulator
        int action = mcts.SelectAction();

        trace.save_action(action);
        trace.save_action_rank(mcts.GetRoot());
        trace.end_step();

        // apply best action to the real world
        terminal = Real.Step(*state, action, observation, reward);


        Results.Reward.Add(reward);
        undiscountedReturn += reward;
        discountedReturn += reward * discount;
        discount *= Real.GetDiscount();

        if (SearchParams.Verbose >= 1)
        {
            Real.DisplayAction(action, std::cout);
            Real.DisplayState(*state, std::cout);
            Real.DisplayObservation(*state, observation, std::cout);
            Real.DisplayReward(reward, std::cout);
        }

        if (XES::enabled()) {
            Real.log_action(action);
            Real.log_observation(*state, observation);
            Real.log_reward(reward);

            XES::logger().end_event();
        }

        if (terminal)
        {
            std::cout << "Terminated" << std::endl;
            break;
        }
        outOfParticles = !mcts.Update(action, observation, reward);
        if (outOfParticles)
            break;

        if (timer.elapsed() > ExpParams.TimeOut)
        {
            std::cout << "Timed out after " << t << " steps in "
                << Results.Time.GetTotal() << "seconds" << std::endl;
            break;
        }
    }

    if (outOfParticles)
    {
        std::cout << "Out of particles, finishing episode with SelectRandom"
                  << std::endl;
        HISTORY history = mcts.GetHistory();
        while (++t < ExpParams.NumSteps) {
            SIMULATOR::observation_t observation;
            double reward;

            // This passes real state into simulator!
            // SelectRandom must only use fully observable state
            // to avoid "cheating"
            int action = Simulator.SelectRandom(*state, history, mcts.GetStatus());
            terminal = Real.Step(*state, action, observation, reward);

            Results.Reward.Add(reward);
            undiscountedReturn += reward;
            discountedReturn += reward * discount;
            discount *= Real.GetDiscount();

            if (SearchParams.Verbose >= 1)
            {
                Real.DisplayAction(action, std::cout);
                Real.DisplayState(*state, std::cout);
                Real.DisplayObservation(*state, observation, std::cout);
                Real.DisplayReward(reward, std::cout);
            }

            if (terminal)
            {
                std::cout << "Terminated" << std::endl;
                break;
            }

            history.Add(action, observation);
        }
    }

    Results.Time.Add(timer.elapsed());
    Results.UndiscountedReturn.Add(undiscountedReturn);
    Results.DiscountedReturn.Add(discountedReturn);
    std::cout << "Discounted return = " << discountedReturn
        << ", average = " << Results.DiscountedReturn.GetMean() << std::endl;
    std::cout << "Undiscounted return = " << undiscountedReturn
        << ", average = " << Results.UndiscountedReturn.GetMean() << std::endl;

    if (XES::enabled())
        XES::logger().add_attributes(
            {{"discounted return", discountedReturn},
             {"undiscounted return", undiscountedReturn}});
}

template<typename T>
void TRACE_EXPERIMENT<T>::MultiRun(trace_t &trace)
{
    for (int n = 0; n < ExpParams.NumRuns; n++)
    {
        std::cout << "Starting run " << n + 1 << " with "
             << SearchParams.NumSimulations << " simulations... " << std::endl;

        if (XES::enabled())
            XES::logger().start_trace();

        if (XES::enabled())
            XES::logger().add_attributes(
                {{"run", n + 1}, {"simulations", SearchParams.NumSimulations}});

        trace.start_run();
        Run(trace);
        trace.end_run();

        if (XES::enabled())
            XES::logger().end_trace();

        if (Results.Time.GetTotal() > ExpParams.TimeOut)
        {
            std::cout << "Timed out after " << n << " runs in "
                << Results.Time.GetTotal() << "seconds" << std::endl;
            break;
        }
    }
}

template<typename T>
void TRACE_EXPERIMENT<T>::BuildTrace(trace_t &trace)
{
    SearchParams.MaxDepth = Simulator.GetHorizon(ExpParams.Accuracy, ExpParams.UndiscountedHorizon);
    ExpParams.SimSteps = Simulator.GetHorizon(ExpParams.Accuracy, ExpParams.UndiscountedHorizon);
    //ExpParams.NumSteps = Real.GetHorizon(ExpParams.Accuracy, ExpParams.UndiscountedHorizon);

    if (XES::enabled()) {
        XES::logger().add_attributes({
            {"MaxDepth", SearchParams.MaxDepth},
            {"SimSteps", ExpParams.SimSteps},
            {"NumSteps", ExpParams.NumSteps},
            {"shield", SearchParams.use_shield},
        });
        Simulator.log_problem_info();
    }

    if (use_fixed_seed) {
        UTILS::RandomSeed(fixed_seed);
        Simulator.set_seed(fixed_seed);
        real_seed = fixed_seed + 1;
    }

    SearchParams.NumSimulations = 1 << ExpParams.PowParticles;
    SearchParams.NumStartStates = 1 << ExpParams.PowParticles;

    if (ExpParams.PowParticles + ExpParams.TransformDoubles >= 0)
        SearchParams.NumTransforms =
            1 << (ExpParams.PowParticles + ExpParams.TransformDoubles);
    else
        SearchParams.NumTransforms = 1;
    SearchParams.MaxAttempts = SearchParams.NumTransforms * ExpParams.TransformAttempts;

    Results.Clear();
    MultiRun(trace);

    if (XES::enabled()) {
        XES::logger().add_attributes(
            {{"average undiscounted return",
              Results.UndiscountedReturn.GetMean()},
             {"average undiscounted return std",
              Results.UndiscountedReturn.GetStdErr()},
             {"average discounted return", Results.DiscountedReturn.GetMean()},
             {"average discounted return std",
              Results.DiscountedReturn.GetStdErr()},
             {"average time", Results.Time.GetMean()},
             {"average time std", Results.Time.GetStdDev()},
             {"total time", Results.Time.GetTotal()}});
    }

    std::cout << "Simulations = " << SearchParams.NumSimulations << std::endl
         << "Runs = " << Results.Time.GetCount() << std::endl
         << "Undiscounted return = " << Results.UndiscountedReturn.GetMean()
         << " +- " << Results.UndiscountedReturn.GetStdErr() << std::endl
         << "Discounted return = " << Results.DiscountedReturn.GetMean()
         << " +- " << Results.DiscountedReturn.GetStdErr() << std::endl
         << "Time = " << Results.Time.GetMean() << std::endl;
}

//----------------------------------------------------------------------------

