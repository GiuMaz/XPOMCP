#include "experiment.h"
#include "boost/timer.hpp"

using namespace std;

EXPERIMENT::PARAMS::PARAMS()
:   NumRuns(1000),
    NumSteps(100000),
    SimSteps(1000),
    TimeOut(3600),
    MinDoubles(0),
    MaxDoubles(20),
    TransformDoubles(-4),
    TransformAttempts(1000),
    Accuracy(0.01),
    UndiscountedHorizon(1000),
    AutoExploration(true)
{
}

EXPERIMENT::EXPERIMENT(const SIMULATOR& real,
    const SIMULATOR& simulator, const string& outputFile,
    EXPERIMENT::PARAMS& expParams, MCTS::PARAMS& searchParams,
    std::string file)
:   Real(real),
    Simulator(simulator),
    ExpParams(expParams),
    SearchParams(searchParams),
    OutputFile(outputFile.c_str()),
    logger(file)
{
    if (ExpParams.AutoExploration)
    {
        if (SearchParams.UseRave)
            SearchParams.ExplorationConstant = 0;
        else
            SearchParams.ExplorationConstant = simulator.GetRewardRange();
    }
    MCTS::InitFastUCB(SearchParams.ExplorationConstant);
}

void EXPERIMENT::Run()
{
    boost::timer timer;

    MCTS mcts(Simulator, SearchParams, logger);

    double undiscountedReturn = 0.0;
    double discountedReturn = 0.0;
    double discount = 1.0;
    bool terminal = false;
    bool outOfParticles = false;
    int t;

    if (fixed_seed) {
        Real.set_seed(real_seed);
        real_seed++;
    }
    STATE* state = Real.CreateStartState();

    if (SearchParams.Verbose >= 1)
        Real.DisplayState(*state, cout);

    for (t = 0; t < ExpParams.NumSteps; t++)
    {
        logger.start_event();

        int observation;
        double reward;
        int action = mcts.SelectAction();
        terminal = Real.Step(*state, action, observation, reward);

        Results.Reward.Add(reward);
        undiscountedReturn += reward;
        discountedReturn += reward * discount;
        discount *= Real.GetDiscount();

        if (SearchParams.Verbose >= 1)
        {
            Real.DisplayAction(action, cout);
            Real.DisplayState(*state, cout);
            Real.DisplayObservation(*state, observation, cout);
            Real.DisplayReward(reward, cout);
        }
        Real.log_action(action, logger);
        Real.log_observation(*state, observation, logger);
        Real.log_reward(reward, logger);

        logger.end_event();

        if (terminal)
        {
            cout << "Terminated" << endl;
            break;
        }
        outOfParticles = !mcts.Update(action, observation, reward);
        if (outOfParticles)
            break;

        if (timer.elapsed() > ExpParams.TimeOut)
        {
            cout << "Timed out after " << t << " steps in "
                << Results.Time.GetTotal() << "seconds" << endl;
            break;
        }
    }

    if (outOfParticles)
    {
        cout << "Out of particles, finishing episode with SelectRandom" << endl;
        HISTORY history = mcts.GetHistory();
        while (++t < ExpParams.NumSteps)
        {
            int observation;
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
                Real.DisplayAction(action, cout);
                Real.DisplayState(*state, cout);
                Real.DisplayObservation(*state, observation, cout);
                Real.DisplayReward(reward, cout);
            }

            if (terminal)
            {
                cout << "Terminated" << endl;
                break;
            }

            history.Add(action, observation);
        }
    }

    Results.Time.Add(timer.elapsed());
    Results.UndiscountedReturn.Add(undiscountedReturn);
    Results.DiscountedReturn.Add(discountedReturn);
    cout << "Discounted return = " << discountedReturn
        << ", average = " << Results.DiscountedReturn.GetMean() << endl;
    cout << "Undiscounted return = " << undiscountedReturn
        << ", average = " << Results.UndiscountedReturn.GetMean() << endl;

    logger.add_attributes({
        {"discounted return", discountedReturn},
        {"undiscounted return", undiscountedReturn}
    });
}

void EXPERIMENT::MultiRun()
{
    for (int n = 0; n < ExpParams.NumRuns; n++)
    {
        cout << "Starting run " << n + 1 << " with "
             << SearchParams.NumSimulations << " simulations... " << endl;

        logger.start_trace();

        logger.add_attributes({
                {"run", n+1},
                {"simulations", SearchParams.NumSimulations}
            });
        Run();
        logger.end_trace();
        if (Results.Time.GetTotal() > ExpParams.TimeOut)
        {
            cout << "Timed out after " << n << " runs in "
                << Results.Time.GetTotal() << "seconds" << endl;
            break;
        }
    }
}

void EXPERIMENT::DiscountedReturn()
{
    cout << "Main runs" << endl;
    OutputFile << "Simulations\tRuns\tUndiscounted return\tUndiscounted error\tDiscounted return\tDiscounted error\tTime\n";

    SearchParams.MaxDepth = Simulator.GetHorizon(ExpParams.Accuracy, ExpParams.UndiscountedHorizon);
    ExpParams.SimSteps = Simulator.GetHorizon(ExpParams.Accuracy, ExpParams.UndiscountedHorizon);
    ExpParams.NumSteps = Real.GetHorizon(ExpParams.Accuracy, ExpParams.UndiscountedHorizon);

    logger.add_attributes({
            {"MaxDepth", SearchParams.MaxDepth},
            {"SimSteps", ExpParams.SimSteps},
            {"NumSteps", ExpParams.NumSteps},
            {"shield", SearchParams.use_shield},
            });
    Simulator.log_problem_info(logger);

    for (int i = ExpParams.MinDoubles; i <= ExpParams.MaxDoubles; i++)
    {
        if (use_fixed_seed) {
            UTILS::RandomSeed(fixed_seed);
            Simulator.set_seed(fixed_seed);
            real_seed = fixed_seed + 1;
        }

        SearchParams.NumSimulations = 1 << i;
        SearchParams.NumStartStates = 1 << i;
        if (i + ExpParams.TransformDoubles >= 0)
            SearchParams.NumTransforms = 1 << (i + ExpParams.TransformDoubles);
        else
            SearchParams.NumTransforms = 1;
        SearchParams.MaxAttempts = SearchParams.NumTransforms * ExpParams.TransformAttempts;

        Results.Clear();
        MultiRun();

        logger.add_attributes(
            {{"average undiscounted return",
              Results.UndiscountedReturn.GetMean()},
             {"average undiscounted return std",
              Results.UndiscountedReturn.GetStdErr()},
             {"average discounted return", Results.DiscountedReturn.GetMean()},
             {"average discounted return std",
              Results.DiscountedReturn.GetStdErr()},
             {"average time", Results.Time.GetMean()},
             {"average time std", Results.Time.GetStdDev()},
             {"total time", Results.Time.GetTotal()}
             });

        cout << "Simulations = " << SearchParams.NumSimulations << endl
            << "Runs = " << Results.Time.GetCount() << endl
            << "Undiscounted return = " << Results.UndiscountedReturn.GetMean()
            << " +- " << Results.UndiscountedReturn.GetStdErr() << endl
            << "Discounted return = " << Results.DiscountedReturn.GetMean()
            << " +- " << Results.DiscountedReturn.GetStdErr() << endl
            << "Time = " << Results.Time.GetMean() << endl;
        OutputFile << SearchParams.NumSimulations << "\t"
            << Results.Time.GetCount() << "\t"
            << Results.UndiscountedReturn.GetMean() << "\t"
            << Results.UndiscountedReturn.GetStdErr() << "\t"
            << Results.DiscountedReturn.GetMean() << "\t"
            << Results.DiscountedReturn.GetStdErr() << "\t"
            << Results.Time.GetMean() << endl;
    }
}

void EXPERIMENT::AverageReward()
{
    cout << "Main runs" << endl;
    OutputFile << "Simulations\tSteps\tAverage reward\tAverage time\n";

    SearchParams.MaxDepth = Simulator.GetHorizon(ExpParams.Accuracy, ExpParams.UndiscountedHorizon);
    ExpParams.SimSteps = Simulator.GetHorizon(ExpParams.Accuracy, ExpParams.UndiscountedHorizon);

    for (int i = ExpParams.MinDoubles; i <= ExpParams.MaxDoubles; i++)
    {
        SearchParams.NumSimulations = 1 << i;
        SearchParams.NumStartStates = 1 << i;
        if (i + ExpParams.TransformDoubles >= 0)
            SearchParams.NumTransforms = 1 << (i + ExpParams.TransformDoubles);
        else
            SearchParams.NumTransforms = 1;
        SearchParams.MaxAttempts = SearchParams.NumTransforms * ExpParams.TransformAttempts;

        Results.Clear();
        Run();

        cout << "Simulations = " << SearchParams.NumSimulations << endl
            << "Steps = " << Results.Reward.GetCount() << endl
            << "Average reward = " << Results.Reward.GetMean()
            << " +- " << Results.Reward.GetStdErr() << endl
            << "Average time = " << Results.Time.GetMean() / Results.Reward.GetCount() << endl;
        OutputFile << SearchParams.NumSimulations << "\t"
            << Results.Reward.GetCount() << "\t"
            << Results.Reward.GetMean() << "\t"
            << Results.Reward.GetStdErr() << "\t"
            << Results.Time.GetMean() / Results.Reward.GetCount() << endl;
    }
}

//----------------------------------------------------------------------------
