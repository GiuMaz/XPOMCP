#include "battleship.h"
#include "experiment.h"
#include "experiment_lazy.h"
#include "mcts.h"
#include "network.h"
#include "obstacleavoidance.h"
#include "pocman.h"
#include "rocksample.h"
#include "tag.h"
#include "lasertag.h"
#include "tiger.h"

#include <boost/program_options.hpp>
#include <memory>

using namespace std;
using namespace boost::program_options;

void UnitTests() {
    cout << "Testing UTILS" << endl;
    UTILS::UnitTest();
    cout << "Testing COORD" << endl;
    COORD::UnitTest();
    cout << "Testing MCTS" << endl;
    MCTS::UnitTest();
}

void disableBufferedIO() {
    setbuf(stdout, nullptr);
    setbuf(stdin, nullptr);
    setbuf(stderr, nullptr);
    setvbuf(stdout, nullptr, _IONBF, 0);
    setvbuf(stdin, nullptr, _IONBF, 0);
    setvbuf(stderr, nullptr, _IONBF, 0);
}

int main(int argc, char *argv[]) {
    MCTS::PARAMS searchParams;
    EXPERIMENT::PARAMS expParams;
    SIMULATOR::KNOWLEDGE knowledge;
    string problem, outputfile, policy;
    int size, number, treeknowledge = 0, rolloutknowledge = 1,
                      smarttreecount = 10;
    double smarttreevalue = 1.0;
    int random_seed = 12345678;
    std::string shield_file;

    double W = 0.0;
    bool complex_shield = false, xes_log = true;

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
        ("maxdoubles",value<int>(&expParams.MaxDoubles), "maximum power of two simulations")
        ("runs", value<int>(&expParams.NumRuns), "number of runs")
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
        ("disabletree", value<bool>(&searchParams.DisableTree), "Use 1-ply rollout action selection")
        ("seed", value<int>(&random_seed), "set random seed (-1 to initialize using time, >= 0 to use a fixed integer as the initial seed)")
        ("useshield", value<bool>(&searchParams.use_shield), "Use preshield (if supported)")
        ("complexshield", value<bool>(&complex_shield), "Use complex shield (if supported)")
        ("shieldfile", value<std::string>(&shield_file), "Specify file with shield parameters")
        ("setW", value<double>(&W), "Fix the reward range (testing purpouse)")
        ("xes", value<bool>(&xes_log)->default_value(true), "Enable XES log");

    variables_map vm;
    store(parse_command_line(argc, argv, desc), vm);
    notify(vm);

    if (vm.count("help") != 0) {
        cout << desc << "\n";
        return 1;
    }

    if (vm.count("problem") == 0) {
        cout << "No problem specified" << endl;
        return 1;
    }

    if (vm.count("test") != 0) {
        cout << "Running unit tests" << endl;
        UnitTests();
        return 0;
    }

    if (problem == "lasertag") {
        auto real = std::make_unique<LASERTAG>();
        auto simulator = std::make_unique<LASERTAG>();
        simulator->SetKnowledge(knowledge);

        if (vm.count("setW") != 0) {
            real->SetRewardRange(W);
            simulator->SetRewardRange(W);
        }

        if (vm.count("complexshield") != 0) {
            simulator->set_complex_shield(true);
        }

        EXPERIMENT_LAZY experiment(*real, *simulator, outputfile, expParams,
                              searchParams);

        if (vm.count("seed") != 0) {
            experiment.set_fixed_seed(random_seed);
        }

        experiment.DiscountedReturn();
        return 0;
    }

    std::unique_ptr<SIMULATOR> real = nullptr;
    std::unique_ptr<SIMULATOR> simulator = nullptr;

    XES::init(xes_log, "log.xes");

    if (problem == "battleship") {
        real = std::make_unique<BATTLESHIP>(size, size, number);
        simulator = std::make_unique<BATTLESHIP>(size, size, number);
    } else if (problem == "pocman") {
        real = std::make_unique<FULL_POCMAN>();
        simulator = std::make_unique<FULL_POCMAN>();
    } else if (problem == "network") {
        real = std::make_unique<NETWORK>(size, number);
        simulator = std::make_unique<NETWORK>(size, number);
    } else if (problem == "rocksample") {
        real = std::make_unique<ROCKSAMPLE>(size, number);
        if (searchParams.use_shield)
            simulator = std::make_unique<ROCKSAMPLE>(size, number, shield_file);
        else
            simulator = std::make_unique<ROCKSAMPLE>(size, number);
    } else if (problem == "tag") {
        real = std::make_unique<TAG>(number);
        simulator = std::make_unique<TAG>(number);
    //} else if (problem == "tiger") {
    //    real = std::make_unique<TIGER>(); // Real simulator (environment)
    //    if (searchParams.use_shield)
    //        simulator = std::make_unique<TIGER>(shield_file);
    //    else
    //        simulator = std::make_unique<TIGER>();
    } else if (problem == "obstacleavoidance") {
        /* Model 1 (Rectangle for Autonomous robots) */
        //std::vector<int> nSubSegs = {3, 5, 3, 5};
        //std::vector<std::vector<double>> subSegLengths = {
        //    {1.0, 1.0, 1.0},
        //    {1.0, 1.0, 1.0, 1.0, 1.0},
        //    {1.0, 1.0, 1.0},
        //    {1.0, 1.0, 1.0, 1.0, 1.0}};
        //std::vector<std::vector<std::pair<double,double>>> visual {
        //    {{0, 0}, {1, 0}, {2,0}},
        //    {{3, 0}, {3, 1}, {3, 2}, {3, 3}, {3, 4}},
        //    {{3, 5}, {2, 5}, {1, 5}},
        //    {{0, 5}, {0, 4}, {0, 3}, {0, 2}, {0, 1}}
        //};
        //int nEnginePowerValues = 3;
        //int nDifficultyValues = 3;
        //int nVelocityValues = 3;

        /* ICE */
        // std::vector<int> nSubSegs = {3, 5, 2, 3, 2, 5, 4, 11};
        // std::vector<std::vector<double>> subSegLengths = {
        //     {0.9, 0.9, 1.0},
        //     {1.0, 1.0, 1.2, 0.9, 1.15},
        //     {0.6, 0.6},
        //     {0.9, 0.9, 1.0},
        //     {1.1, 1.0},
        //     {1.4, 1.0, 0.9, 0.9, 0.95},
        //     {1.0, 0.9, 0.9, 0.9},
        //     {1.0, 1.4, 1.2, 1.2, 1.2, 1.2, 1.2, 1.2, 1.2, 1.2, 1.2}
        // };

        // std::vector<std::vector<std::pair<double,double>>> visual {
        //     {{0.0, 13.2}, {0.9, 13.2}, {1.8, 13.2}},
        //     {{2.8, 13.2}, {2.8, 12.2}, {2.8, 11.2}, {2.8, 10.0}, {2.8, 9.1}, },
        //     {{2.8, 7.95}, {2.2, 7.95}},
        //     {{1.6, 7.95}, {1.6, 7.05}, {1.6, 6.15}},
        //     {{1.6, 5.15}, {2.7, 5.15}},
        //     {{3.7, 5.15}, {3.7, 3.75}, {3.7, 2.75}, {3.7, 1.85}, {3.7, 0.95}},
        //     {{3.7, 0.0}, {2.7, 0.0}, {1.8, 0.0}, {0.9, 0.0}} ,
        //     {{0.0, 0.0}, {0.0, 1.0}, {0.0, 2.4}, {0.0, 3.6}, {0.0, 4.8}, {0.0, 6.0}, {0.0, 7.2}, {0.0, 8.4}, {0.0, 9.6}, {0.0, 10.8}, {0.0, 12.0}}
        // };
        // int nEnginePowerValues = 3;
        // int nDifficultyValues = 3;
        // int nVelocityValues = 3;

        /* ICE SMALL */
        std::vector<int> nSubSegs = {3, 5, 2, 3, 2, 5, 4, 12};
        std::vector<std::vector<double>> subSegLengths = {
            {0.9, 0.9, 1.0},
            {1.0, 1.0, 1.2, 0.9, 1.15},
            {0.6, 0.6},
            {0.9, 0.9, 1.0},
            {1.1, 1.0},
            {1.4, 1.0, 0.9, 0.9, 0.95},
            {1.0, 0.9, 0.9, 0.9},
            {1.0, 1.4, 1.2, 1.0, 1.2, 1.2, 1.2, 1.2, 1.2, 1.1, 1.1, 0.2}
        };

        std::vector<std::vector<std::pair<double,double>>> visual {
            {{0.0, 13.2}, {0.9, 13.2}, {1.8, 13.2}},
            {{2.8, 13.2}, {2.8, 12.2}, {2.8, 11.2}, {2.8, 10.0}, {2.8, 9.1}, },
            {{2.8, 7.95}, {2.2, 7.95}},
            {{1.6, 7.95}, {1.6, 7.05}, {1.6, 6.15}},
            {{1.6, 5.15}, {2.7, 5.15}},
            {{3.7, 5.15}, {3.7, 3.75}, {3.7, 2.75}, {3.7, 1.85}, {3.7, 0.95}},
            {{3.7, 0.0}, {2.7, 0.0}, {1.8, 0.0}, {0.9, 0.0}} ,
            {{0.0, 0.0}, {0.0, 1.0}, {0.0, 2.4}, {0.0, 3.6}, {0.0, 4.8}, {0.0, 6.0}, {0.0, 7.2}, {0.0, 8.4}, {0.0, 9.6}, {0.0, 10.7}, {0.0, 11.8}, {0.0, 12.0}}
        };
        int nEnginePowerValues = 3;
        int nDifficultyValues = 3;
        int nVelocityValues = 3;

        real = std::make_unique<OBSTACLEAVOIDANCE>(
            nSubSegs, subSegLengths, nEnginePowerValues, nDifficultyValues,
            nVelocityValues);

        if (searchParams.use_shield)
            simulator = std::make_unique<OBSTACLEAVOIDANCE>(
                nSubSegs, subSegLengths, nEnginePowerValues, nDifficultyValues,
                nVelocityValues, shield_file);
        else
            simulator = std::make_unique<OBSTACLEAVOIDANCE>(
                nSubSegs, subSegLengths, nEnginePowerValues, nDifficultyValues,
                nVelocityValues);
        dynamic_cast<OBSTACLEAVOIDANCE &>(*real).set_visual(visual);
        dynamic_cast<OBSTACLEAVOIDANCE &>(*simulator).set_visual(visual);
    } else {
        cout << "Unknown problem" << endl;
        exit(1);
    }

    if (vm.count("setW") != 0) {
        real->SetRewardRange(W);
        simulator->SetRewardRange(W);
    }

    if (vm.count("complexshield") != 0) {
        simulator->set_complex_shield(true);
    }

    simulator->SetKnowledge(knowledge);

    EXPERIMENT experiment(*real, *simulator, outputfile, expParams,
                          searchParams);


    if (vm.count("seed") != 0) {
        experiment.set_fixed_seed(random_seed);
    }

    experiment.DiscountedReturn();

    return 0;
}

