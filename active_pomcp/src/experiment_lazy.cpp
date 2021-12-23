#include "experiment_lazy.h"

using namespace std;

EXPERIMENT_LAZY::EXPERIMENT_LAZY(const SIMULATOR_LAZY& real,
    const SIMULATOR_LAZY& simulator, const string& outputFile,
    EXPERIMENT_LAZY::PARAMS& expParams, MCTS_LAZY::PARAMS& searchParams)
:   Real(real),
    Simulator(simulator),
    ExpParams(expParams),
    SearchParams(searchParams),
    OutputFile(outputFile.c_str())
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

void EXPERIMENT_LAZY::Run()
{
    Timer timer;

    MCTS_LAZY mcts(Simulator, SearchParams);

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
        Real.DisplayState(*state, cout);

    for (t = 0; t < ExpParams.NumSteps; t++)
    {
        if (XES::enabled())
            XES::logger().start_event();

        SIMULATOR_LAZY::observation_t observation;
        double reward;

        if (XES::enabled()) {
            Simulator.log_beliefs(mcts.BeliefState());
        }

        //cout << "===\n particles" << mcts.BeliefState().GetNumSamples() << endl;

        int action = mcts.SelectAction();

        //int count = 0;
        //for (const auto &q : mcts.GetRoot()->Child(action))
        //    count++;
        //cout << "observations: " << count << "\n===" << endl;

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

        if (XES::enabled()) {
            Real.log_action(action);
            Real.log_observation(*state, observation);
            Real.log_reward(reward);
        }

        if (XES::enabled())
            XES::logger().end_event();

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
            SIMULATOR_LAZY::observation_t observation;
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

    if (XES::enabled())
        XES::logger().add_attributes(
            {{"discounted return", discountedReturn},
             {"undiscounted return", undiscountedReturn}});
}

void EXPERIMENT_LAZY::MultiRun()
{
    for (int n = 0; n < ExpParams.NumRuns; n++)
    {
        cout << "Starting run " << n + 1 << " with "
             << SearchParams.NumSimulations << " simulations... " << endl;

        if (XES::enabled())
            XES::logger().start_trace();

        if (XES::enabled())
            XES::logger().add_attributes(
                {{"run", n + 1}, {"simulations", SearchParams.NumSimulations}});

        Run();

        if (XES::enabled())
            XES::logger().end_trace();

        if (Results.Time.GetTotal() > ExpParams.TimeOut)
        {
            cout << "Timed out after " << n << " runs in "
                << Results.Time.GetTotal() << "seconds" << endl;
            break;
        }
    }
}

void EXPERIMENT_LAZY::DiscountedReturn()
{
    cout << "Main runs" << endl;
    OutputFile << "Simulations\tRuns\tUndiscounted return\tUndiscounted error\tDiscounted return\tDiscounted error\tTime\n";

    SearchParams.MaxDepth = Simulator.GetHorizon(ExpParams.Accuracy, ExpParams.UndiscountedHorizon);
    ExpParams.SimSteps = Simulator.GetHorizon(ExpParams.Accuracy, ExpParams.UndiscountedHorizon);
    ExpParams.NumSteps = Real.GetHorizon(ExpParams.Accuracy, ExpParams.UndiscountedHorizon);

    if (XES::enabled()) {
        XES::logger().add_attributes({
            {"MaxDepth", SearchParams.MaxDepth},
            {"SimSteps", ExpParams.SimSteps},
            {"NumSteps", ExpParams.NumSteps},
            {"shield", SearchParams.use_shield},
        });
        Simulator.log_problem_info();
    }

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
                    {"total time", Results.Time.GetTotal()}
                    });
        }

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

void EXPERIMENT_LAZY::AverageReward()
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

