#include "creator_experiment.h"
#include "json.hpp"
#include "mcts.h"
#include "obstacleavoidance.h"
#include "obstacleavoidance_trace.h"
#include "trace_experiment.h"
#include "active_obstacle.h"

#include <algorithm>
#include <boost/program_options.hpp>
#include <cmath>
#include <exception>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <string_view>
#include <type_traits>
#include <z3++.h>
#include <z3.h>

using namespace boost::program_options;
using json = nlohmann::json;

z3::expr to_real(z3::context &c, double d) {
    return c.real_val(std::to_string(d).c_str());
}

double to_double(z3::expr d) {
    return std::stod(d.get_decimal_string(6));
}
class XPOMCP_SOLVER {
public:

    struct SOFT_LIT {
        SOFT_LIT(z3::expr l, int r, int s) : lit(l), run(r), step(s) {}

        z3::expr lit;
        int run;
        int step;
    };

    XPOMCP_SOLVER() : c(), solver(c) {
        // z3::set_param("auto-config", "false");
        // try {
        //     z3::params p(c);
        //     p.set("optsmt_engine", "lra");
        //     solver.set(p);
        //     std::cout << "OK" << std::endl;
        // }
        // catch (z3::exception &e) {
        //     std::cout << e.msg() << std::endl;
        // }
    }

    z3::expr real_const(const char *s) { return c.real_const(s); }
    z3::expr int_const(const char *s) { return c.int_const(s); }
    z3::expr bool_const(const char *s) { return c.bool_const(s); }
    z3::expr prob_const(const char *s) { 
        z3::expr ex = c.real_const(s);
        solver.add(ex >= 0 && ex <= 1);
        return ex;
    }

    const z3::context &context() const { return c; }
    z3::context &context() { return c; }

    void add_soft_constraint(int run, int step, z3::expr formula) {
        std::string name =
            "b_" + std::to_string(run) + "_" + std::to_string(step);
        z3::expr l = bool_const(name.c_str());
        soft_constrains.push_back({l, run, step});
        formula = l || formula;
        solver.add(formula);
    }

    /*
     * In the rule, lower bounds must be as higher as possible and
     * upper bound as lower as possible
     */
    Solution build_rule(std::vector<z3::expr> lower_bound,
            std::vector<z3::expr> upper_bound) {

        int low_threshold = 0;
        int total_soft_constr = soft_constrains.size();
        int high_threshold = total_soft_constr;
        int final_threshold = -1;

        std::vector<int> coeff(soft_constrains.size(), 1);
        z3::expr_vector lits(c);
        for (const auto &l : soft_constrains)
            lits.push_back(l.lit);

        //z3::model best_model(c);
        while (low_threshold <= high_threshold) {
            solver.push();
            int threshold = (low_threshold + high_threshold) / 2;
            solver.add(z3::pble(lits, coeff.data(), threshold));

            auto result = solver.check();
            if (result == z3::sat) {
                final_threshold = threshold;
                //best_model = solver.get_model();
                high_threshold = threshold - 1;
            }
            else {
                low_threshold = threshold + 1;
            }

            solver.pop();
        }

        // build extreme rules

        // tight rule: lower bounds must be as higher as possible and
        // upper bound as lower as possible
        Solution sol;

        for (const auto & l : lower_bound)
        {
            Models result(context());
            {
                solver.push();
                solver.add(z3::pble(lits, coeff.data(), final_threshold));
                solver.maximize(l);
                solver.check();

                result.tight_model = solver.get_model();

                solver.pop();
            }
            {
                solver.push();
                solver.add(z3::pble(lits, coeff.data(), final_threshold + 1));
                solver.minimize(l);
                solver.check();

                result.loose_model = solver.get_model();

                solver.pop();
            }
            sol.lower_bound.emplace_back(result);
        }

        for (const auto & u : upper_bound)
        {
            Models result(context());
            {
                solver.push();
                solver.add(z3::pble(lits, coeff.data(), final_threshold));
                solver.minimize(u);
                solver.check();

                result.tight_model = solver.get_model();

                solver.pop();
            }
            {
                solver.push();
                solver.add(z3::pble(lits, coeff.data(), final_threshold + 1));
                solver.maximize(u);
                solver.check();

                result.loose_model = solver.get_model();

                solver.pop();
            }
            sol.upper_bound.emplace_back(result);
        }
        // loose rule

        return sol;
    }

    const std::vector<SOFT_LIT> &unsatisfiable_steps() {
        return anomalies;
    }

private:
    z3::context c;
    z3::optimize solver;

    std::vector<SOFT_LIT> soft_constrains;
    std::vector<SOFT_LIT> anomalies;
};

BOUNDS build_rule(const OBSTACLEAVOIDANCE_TRACE &vr_trace) {
    // build rule
    XPOMCP_SOLVER xpomcp;

    auto &c = xpomcp.context();

    z3::expr x_1 = xpomcp.prob_const("x_1");
    z3::expr x_2 = xpomcp.prob_const("x_2");
    z3::expr x_3 = xpomcp.prob_const("x_3");
    z3::expr x_4 = xpomcp.prob_const("x_4");
    
    std::vector<std::pair<int, int>> soft_run_step;

    int nrun = 0, nstep = 0;
    for (const auto &run : vr_trace.runs()) {
        nstep = 0;

        for (const auto & step : run.steps()) {

            int seg = step.seg();

            double p_0 = step.seg_diff(seg, 0);
            double p_1 = step.seg_diff(seg, 1);
            double p_2 = step.seg_diff(seg, 2);

            z3::expr formula(c);

            formula = to_real(c,p_0) >= x_1 ||
                to_real(c, p_2) <= x_2 ||
                (to_real(c, p_0) >= x_3 && to_real(c, p_1) >= x_4);

            // sample
            if (step.action() != 2)
                formula = !formula;

            xpomcp.add_soft_constraint(nrun, nstep, formula);

            nstep += 1;
        }
        nrun += 1;
    }

    auto solution = xpomcp.build_rule({x_1, x_3, x_4}, {x_2});

    BOUNDS b;
    b.x1_loose = to_double(solution.lower_bound[0].loose_model.eval(x_1));
    b.x1_strict = to_double(solution.lower_bound[0].tight_model.eval(x_1));

    b.x2_loose = to_double(solution.upper_bound[0].tight_model.eval(x_2));
    b.x2_strict = to_double(solution.upper_bound[0].loose_model.eval(x_2));

    b.x3_loose = to_double(solution.lower_bound[1].loose_model.eval(x_3));
    b.x3_strict = to_double(solution.lower_bound[1].tight_model.eval(x_3));

    b.x4_loose = to_double(solution.lower_bound[2].loose_model.eval(x_4));
    b.x4_strict = to_double(solution.lower_bound[2].tight_model.eval(x_4));

    std::cout << "FAST: "
        "p0 >= [" << b.x1_loose << ", " << b.x1_strict << "]" <<
        " OR p2 <= [" << b.x2_loose << ", " << b.x2_strict << "]" <<
        " OR (p0 >= [" << b.x3_loose << ", " << b.x3_strict << "]" <<
        " AND p1 >= [" << b.x4_loose << ", " << b.x4_strict << "])" << std::endl;

    std::cout << "fail to satisfy " << xpomcp.unsatisfiable_steps().size()
              << " steps" << std::endl;

    return b;
}

int main(int argc, char *argv[]) {

    MCTS::PARAMS searchParams;
    TRACE_EXPERIMENT<OBSTACLEAVOIDANCE_TRACE>::PARAMS expParams;
    SIMULATOR::KNOWLEDGE knowledge;
    std::string outputfile, policy, initfile;
    int size, number;
    int treeknowledge = 0, rolloutknowledge = 1, smarttreecount = 10;
    double smarttreevalue = 1.0;
    int random_seed = 654321;

    double W = 0.0;
    bool xes_log = true;

    int initrun = 5, activeiter = 5, activerun = 5, passiveiter = 5, passiverun = 5;

    options_description desc("Allowed options");
    desc.add_options()
        ("help", "produce help message")
        ("outputfile", value<std::string>(&outputfile)->default_value("output.txt"),
        "summary output file")
        ("policy", value<std::string>(&policy), "policy file (explicit POMDPs only)")
        ("size", value<int>(&size), "size of problem (problem specific)")
        ("number", value<int>(&number),
         "number of elements in problem (problem specific)")
        ("timeout", value<double>(&expParams.TimeOut), "timeout (seconds)")
        ("powparticles",value<int>(&expParams.PowParticles),
         "power of two simulations")
        ("accuracy", value<double>(&expParams.Accuracy),
         "accuracy level used to determine horizon")
        ("horizon", value<int>(&expParams.UndiscountedHorizon),
         "horizon to use when not discounting")
        ("numsteps", value<int>(&expParams.NumSteps),
         "number of steps to run when using average reward")
        ("initrun", value<int>(&initrun),
         "")
        ("activeiter", value<int>(&activeiter),
         "")
        ("activerun", value<int>(&activerun),
         "")
        ("passiveiter", value<int>(&passiveiter),
         "")
        ("passiverun", value<int>(&passiverun),
         "")
        ("verbose", value<int>(&searchParams.Verbose), "verbosity level")
        ("autoexploration", value<bool>(&expParams.AutoExploration),
         "Automatically assign UCB exploration constant")
        ("exploration", value<double>(&searchParams.ExplorationConstant),
         "Manual value for UCB exploration constant")
        ("usetransforms", value<bool>(&searchParams.UseTransforms),
         "Use transforms")
        ("transformdoubles", value<int>(&expParams.TransformDoubles),
         "Relative power of two for transforms compared to simulations")
        ("transformattempts", value<int>(&expParams.TransformAttempts),
         "Number of attempts for each transform")
        ("userave", value<bool>(&searchParams.UseRave), "RAVE")
        ("ravediscount", value<double>(&searchParams.RaveDiscount),
         "RAVE discount factor")
        ("raveconstant", value<double>(&searchParams.RaveConstant),
         "RAVE bias constant")
        ("treeknowledge", value<int>(&knowledge.TreeLevel),
         "Knowledge level in tree (0=Pure, 1=Legal, 2=Smart)")
        ("rolloutknowledge", value<int>(&knowledge.RolloutLevel),
         "Knowledge level in rollouts (0=Pure, 1=Legal, 2=Smart)")
        ("smarttreecount", value<int>(&knowledge.SmartTreeCount),
         "Prior count for preferred actions during smart tree search")
        ("smarttreevalue", value<double>(&knowledge.SmartTreeValue),
         "Prior value for preferred actions during smart tree search")
        ("disabletree", value<bool>(&searchParams.DisableTree),
         "Use 1-ply rollout action selection")
        ("seed", value<int>(&random_seed),
         "set random seed (-1 to initialize using time, >= 0 to use a fixed "
         "integer as the initial seed)")
        ("setW", value<double>(&W), "Fix the reward range (testing purpouse)")
        ("initfile", value<std::string>(&initfile), "use json file to initialize")
        ("xes", value<bool>(&xes_log)->default_value(true), "Enable XES log");

    variables_map vm;
    store(parse_command_line(argc, argv, desc), vm);
    notify(vm);

    if (vm.count("help") != 0) {
        std::cout << desc << "\n";
        return 1;
    }

    std::unique_ptr<SIMULATOR> real = nullptr;
    std::unique_ptr<SIMULATOR> simulator = nullptr;

    XES::init(xes_log, "log.xes");

    std::vector<int> nSubSegs = {3, 5, 2, 3, 2, 5, 4, 11};
    std::vector<std::vector<double>> subSegLengths = {
        {0.9, 0.9, 1.0},
        {1.0, 1.0, 1.2, 0.9, 1.15},
        {0.6, 0.6},
        {0.9, 0.9, 1.0},
        {1.1, 1.0},
        {1.4, 1.0, 0.9, 0.9, 0.95},
        {1.0, 0.9, 0.9, 0.9},
        {1.0, 1.4, 1.2, 1.2, 1.2, 1.2, 1.2, 1.2, 1.2, 1.2, 1.2}
    };

    std::vector<std::vector<std::pair<double,double>>> visual {
        {{0.0, 13.2}, {0.9, 13.2}, {1.8, 13.2}},
            {{2.8, 13.2}, {2.8, 12.2}, {2.8, 11.2}, {2.8, 10.0}, {2.8, 9.1}, },
            {{2.8, 7.95}, {2.2, 7.95}},
            {{1.6, 7.95}, {1.6, 7.05}, {1.6, 6.15}},
            {{1.6, 5.15}, {2.7, 5.15}},
            {{3.7, 5.15}, {3.7, 3.75}, {3.7, 2.75}, {3.7, 1.85}, {3.7, 0.95}},
            {{3.7, 0.0}, {2.7, 0.0}, {1.8, 0.0}, {0.9, 0.0}} ,
            {{0.0, 0.0}, {0.0, 1.0}, {0.0, 2.4}, {0.0, 3.6}, {0.0, 4.8},
                {0.0, 6.0}, {0.0, 7.2}, {0.0, 8.4}, {0.0, 9.6}, {0.0, 10.8},
                {0.0, 12.0}}
    };
    int nEnginePowerValues = 3;
    int nDifficultyValues = 3;
    int nVelocityValues = 3;

    real = std::make_unique<OBSTACLEAVOIDANCE>(
        nSubSegs, subSegLengths, nEnginePowerValues, nDifficultyValues,
        nVelocityValues);

    simulator = std::make_unique<OBSTACLEAVOIDANCE>(
        nSubSegs, subSegLengths, nEnginePowerValues, nDifficultyValues,
        nVelocityValues);

    // ----- STANDARD START -----

    if (vm.count("setW") != 0) {
        real->SetRewardRange(W);
        simulator->SetRewardRange(W);
    }

    // initialize using five runs
    expParams.NumRuns = initrun;

    simulator->SetKnowledge(knowledge);

    TRACE_EXPERIMENT<OBSTACLEAVOIDANCE_TRACE> experiment(
        *real, *simulator, expParams, searchParams);

    if (vm.count("seed") != 0) {
        experiment.set_fixed_seed(random_seed);
        UTILS::RandomSeed(random_seed);
    }

    std::unique_ptr<OBSTACLEAVOIDANCE_TRACE> trace =
        std::make_unique<OBSTACLEAVOIDANCE_TRACE>();

    if (vm.count("initfile") == 0) {
        experiment.BuildTrace(*trace);
        std::ofstream f("initfile.json");
        f << std::setw(4) << trace->serialize() << std::endl;
    } else {
        std::ifstream f(initfile);
        json j;
        f >> j;
        trace->deserialize(j);
    }

    BOUNDS solution = build_rule(*trace);

    for (int i = 0; i < activeiter; i++) {
        expParams.NumRuns =  activerun;

        auto real2 = std::make_unique<OBSTACLEAVOIDANCE>(
                nSubSegs, subSegLengths, nEnginePowerValues, nDifficultyValues,
                nVelocityValues); 
        auto simulator2 = std::make_unique<OBSTACLEAVOIDANCE>(
                nSubSegs, subSegLengths, nEnginePowerValues, nDifficultyValues,
                nVelocityValues);

        ACTIVE_OBSTACLE<OBSTACLEAVOIDANCE_TRACE> experiment_creator(
            *real2, *simulator2, expParams, searchParams, solution);

        if (vm.count("seed") != 0) {
            experiment_creator.set_fixed_seed(++random_seed);
            UTILS::RandomSeed(random_seed);
        }

        experiment_creator.BuildTrace(*trace);

        solution = build_rule(*trace);

        std::ofstream f("initfile2.json");
        f << std::setw(4) << trace->serialize() << std::endl;
    }

    for (int i = 0; i < passiveiter; i++) {
        expParams.NumRuns =  passiverun;

        auto real2 = std::make_unique<OBSTACLEAVOIDANCE>(
                nSubSegs, subSegLengths, nEnginePowerValues, nDifficultyValues,
                nVelocityValues);

        auto simulator2 = std::make_unique<OBSTACLEAVOIDANCE>(
                nSubSegs, subSegLengths, nEnginePowerValues, nDifficultyValues,
                nVelocityValues);

        TRACE_EXPERIMENT<OBSTACLEAVOIDANCE_TRACE> experiment(
            *real2, *simulator2, expParams, searchParams);

        if (vm.count("seed") != 0) {
            experiment.set_fixed_seed(++random_seed);
            UTILS::RandomSeed(random_seed);
        }

        experiment.BuildTrace(*trace);

        auto t3 = build_rule(*trace);

        std::ofstream f("initfile2.json");
        f << std::setw(4) << trace->serialize() << std::endl;
    }
    return 0;
}


