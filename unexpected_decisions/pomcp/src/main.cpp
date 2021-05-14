#include "mcts.h"
#include "network.h"
#include "obstacleavoidance.h"
#include "tiger.h"
#include "testsimulator.h"
#include "experiment.h"
#include <boost/program_options.hpp>

using namespace std;
using namespace boost::program_options;

void UnitTests()
{
    cout << "Testing UTILS" << endl;
    UTILS::UnitTest();
    cout << "Testing MCTS" << endl;
    MCTS::UnitTest();
}

void disableBufferedIO(void)
{
    setbuf(stdout, NULL);
    setbuf(stdin, NULL);
    setbuf(stderr, NULL);
    setvbuf(stdout, NULL, _IONBF, 0);
    setvbuf(stdin, NULL, _IONBF, 0);
    setvbuf(stderr, NULL, _IONBF, 0);
}

int main(int argc, char* argv[])
{
    
    MCTS::PARAMS searchParams;          // Monte-Carlo Tree Search parameters
    EXPERIMENT::PARAMS expParams;       // Experiment Parameters
    SIMULATOR::KNOWLEDGE knowledge;     // Simulator's knowledge
    string problem, outputfile, policy; // 
    int size, number, treeknowledge = 1, rolloutknowledge = 1, smarttreecount = 10;
    double smarttreevalue = 1.0;

    int random_seed = -1;

    options_description desc("Allowed options");
    desc.add_options()
        ("help", "produce help message")
        ("test", "run unit tests")
        ("problem", value<string>(&problem), "problem to run")
        ("outputfile", value<string>(&outputfile)->default_value("output.txt"), "summary output file")
        ("policy", value<string>(&policy), "policy file (explicit POMDPs only)")
        ("size", value<int>(&size), "size of problem (problem specific)")
        ("number", value<int>(&number), "number of elements in problem (problem specific)")
        ("timeout", value<double>(&expParams.TimeOut), "timeout (seconds)")
        ("mindoubles", value<int>(&expParams.MinDoubles), "minimum power of two simulations")
        ("maxdoubles", value<int>(&expParams.MaxDoubles), "maximum power of two simulations") // Max power of numSim
        ("runs", value<int>(&expParams.NumRuns), "number of runs")
        ("testing", value<int>(&expParams.Testing), "testing experiment")
        ("accuracy", value<double>(&expParams.Accuracy), "accuracy level used to determine horizon")
        ("horizon", value<int>(&expParams.UndiscountedHorizon), "horizon to use when not discounting")
        ("numsteps", value<int>(&expParams.NumSteps), "number of steps to run when using average reward")
        ("verbose", value<int>(&searchParams.Verbose), "verbosity level")
        ("autoexploration", value<bool>(&expParams.AutoExploration), "Automatically assign UCB exploration constant")
        ("exploration", value<double>(&searchParams.ExplorationConstant), "Manual value for UCB exploration constant")
        ("usetransforms", value<bool>(&searchParams.UseTransforms), "Use transforms")
        ("transformdoubles", value<int>(&expParams.TransformDoubles), "Relative power of two for transforms compared to simulations")
        ("transformattempts", value<int>(&expParams.TransformAttempts), "Number of attempts for each transform")
        ("userave", value<bool>(&searchParams.UseRave), "RAVE")
        ("ravediscount", value<double>(&searchParams.RaveDiscount), "RAVE discount factor")
        ("raveconstant", value<double>(&searchParams.RaveConstant), "RAVE bias constant")
        ("treeknowledge", value<int>(&knowledge.TreeLevel), "Knowledge level in tree (0=Pure, 1=Legal, 2=Smart)")
        ("rolloutknowledge", value<int>(&knowledge.RolloutLevel), "Knowledge level in rollouts (0=Pure, 1=Legal, 2=Smart)")
        ("smarttreecount", value<int>(&knowledge.SmartTreeCount), "Prior count for preferred actions during smart tree search")
        ("smarttreevalue", value<double>(&knowledge.SmartTreeValue), "Prior value for preferred actions during smart tree search")
        ("relknowlevel", value<int>(&knowledge.relKnowLevel), "Level of knowledge about state-variable relationships: 0=no, 1=MRF, 2=Oracle")
        ("disabletree", value<bool>(&searchParams.DisableTree), "Use 1-ply rollout action selection")
        ("seed", value<int>(&random_seed), "set random seed (-1 to initialize using time, >= 0 to use a fixed integer as the initial seed)")
        ;

    variables_map vm;
    store(parse_command_line(argc, argv, desc), vm);
    notify(vm);

    if (vm.count("help"))
    {
        cout << desc << "\n";
        return 1;
    }

   
    if (vm.count("problem") == 0)
    {
        cout << "No problem specified" << endl;
        return 1;
    }

    if (vm.count("test"))
    {
        cout << "Running unit tests" << endl;
        UnitTests();
        return 0;
    }

    SIMULATOR* real = 0;
    SIMULATOR* simulator = 0;

    if (problem == "obstacleavoidance") {

        std::vector<int> nSubSegs={3,5,2,3,2,5,4,11};
        std::vector<std::vector<double>> subSegLengths = {
            {0.9, 0.9, 1.0},
            {1.0, 1.0, 1.2, 0.9, 1.15},
            {1.1, 1.1},
            {0.9, 0.9, 1.0},
            {0.6, 0.6},
            {1.4, 1.0, 0.9, 0.9, 0.95},
            {1.0, 0.9, 0.9, 0.9},
            {1.0, 1.4, 1.2, 1.2, 1.2, 1.2, 1.2, 1.2, 1.2, 1.2, 1.2}};
        int nEnginePowerValues=3;
        int nDifficultyValues=3;
        int nVelocityValues=3;

        real =
            new OBSTACLEAVOIDANCE(nSubSegs, subSegLengths, nEnginePowerValues,
                                  nDifficultyValues, nVelocityValues);
        simulator =
            new OBSTACLEAVOIDANCE(nSubSegs, subSegLengths, nEnginePowerValues,
                                  nDifficultyValues, nVelocityValues);
    }
    else if (problem == "tiger")
    {
        if (random_seed >= 0) UTILS::RandomSeed(random_seed);
        real = new TIGER();
        simulator = new TIGER();
    }
    else if (problem == "test")
    {
        int actions=3;
        int observations=3; 
        int maxDepth=3;
        real = new TEST_SIMULATOR(actions,observations,maxDepth);
        simulator = new TEST_SIMULATOR(actions,observations,maxDepth);
    }
    else
    {
        cout << "Unknown problem" << endl;
        exit(1);
    }

    simulator->SetKnowledge(knowledge);

    EXPERIMENT experiment(*real, *simulator, outputfile, expParams, searchParams);
    experiment.DiscountedReturn();

    delete real;
    delete simulator;
    return 0;
}
