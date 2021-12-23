#include "battleship.h"
#include "trace_experiment.h"
#include "mcts.h"
#include "network.h"
#include "obstacleavoidance.h"
#include "pocman.h"
#include "rocksample.h"
#include "tag.h"
#include "tiger.h"
#include "creator_experiment.h"
#include "json.hpp"
#include "rocksample_trace.h"

#include <algorithm>
#include <sstream>
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
#include <z3.h>
#include <z3++.h>

using namespace boost::program_options;
using json = nlohmann::json;

std::map<int, std::vector<COORD>> rocks = {
    {11,
     {COORD(0, 3), COORD(0, 7), COORD(1, 8), COORD(2, 4), COORD(3, 3),
      COORD(3, 8), COORD(4, 3), COORD(5, 8), COORD(6, 1), COORD(9, 3),
      COORD(9, 9)}},
    {15,
     {COORD(0, 4), COORD(0, 11), COORD(1, 8), COORD(2, 4), COORD(2, 12),
      COORD(3, 9), COORD(4, 3), COORD(5, 14), COORD(6, 0), COORD(6, 8),
      COORD(9, 3), COORD(10, 2), COORD(11, 7), COORD(12, 12), COORD(14, 9)}}};

z3::expr to_real(z3::context &c, double d) {
    return c.real_val(std::to_string(d).c_str());
}

double to_double(z3::expr d) {
    return std::stod(d.get_decimal_string(6));
}

// return -1 if no rock in position x,y
int get_rock(int x, int y, int r) {
    int rock_pos = 0;
    for (; rock_pos < rocks[r].size(); rock_pos++) {
        if (rocks[r][rock_pos].X == x &&
            rocks[r][rock_pos].Y == y) {
            return rock_pos;
        }
    }
    return -1;
}

class XPOMCP_SOLVER {
public:
    struct Models {
        Models(z3::context &c) : tight_model(c), loose_model(c) {}
        z3::model tight_model;
        z3::model loose_model;
    };

    struct SOFT_LIT {
        SOFT_LIT(z3::expr l, int r, int s) : lit(l), run(r), step(s) {}

        z3::expr lit;
        int run;
        int step;
    };

    XPOMCP_SOLVER() : c(), solver(c) {
        //z3::set_param("auto-config", "false");
        //try {
        //    z3::params p(c);
        //    p.set("optsmt_engine", "lra");
        //    solver.set(p);
        //    std::cout << "OK" << std::endl;
        //}
        //catch (z3::exception &e) {
        //    std::cout << e.msg() << std::endl;
        //}
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
    Models build_rule(std::vector<z3::expr> lower_bound,
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
        Models result(context());

        // tight rule: lower bounds must be as higher as possible and
        // upper bound as lower as possible
        {
            solver.push();
            solver.add(z3::pble(lits, coeff.data(), final_threshold));

            z3::expr maximization = to_real(c, 0.0);
            for (const auto &e : lower_bound)
                maximization = maximization + e;
            for (const auto &e : upper_bound)
                maximization = maximization - e;

            //std::cout << maximization << std::endl;
            solver.maximize(maximization);
            //std::cout << solver << std::endl;
            solver.check();
            result.tight_model = solver.get_model();

            std::cout << result.tight_model.eval(lower_bound[0]) << std::endl;

            for (const auto &l : soft_constrains) {
                if (!result.tight_model.eval(l.lit).is_true()) continue;
                anomalies.emplace_back(l);
            }

            solver.pop();
        }

        // loose rule
        {
            solver.push();
            //solver.add(z3::pble(lits, coeff.data(), final_threshold));
            solver.add(z3::pble(lits, coeff.data(), final_threshold + 1));

            z3::expr maximization = to_real(c, 0.0);
            for (const auto &e : lower_bound)
                maximization = maximization - e;
            for (const auto &e : upper_bound)
                maximization = maximization + e;

            //std::cout << maximization << std::endl;
            solver.maximize(maximization);
            //std::cout << solver << std::endl;
            solver.check();
            result.loose_model = solver.get_model();

            std::cout << result.loose_model.eval(lower_bound[0]) << std::endl;

            solver.pop();
        }

        return result;
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

std::pair<double, double> build_rule(const ROCKSAMPLE_TRACE &rs_trace, int rocknum) {
    // build rule
    XPOMCP_SOLVER xpomcp;

    auto &c = xpomcp.context();

    z3::expr x_1 = xpomcp.prob_const("x_1");
    
    std::vector<std::pair<int, int>> soft_run_step;

    int nrun = 0, nstep = 0;
    for (const auto &run : rs_trace.runs()) {
        nstep = 0;
        std::vector<bool> sampled_rock(rocknum, false);

        for (const auto & step : run.steps()) {

            int r = get_rock(step.x(), step.y(), rocknum);
            double prob = r == -1 ? 0.0 : step.rocks()[r];

            z3::expr formula(c);

            formula = (r >= 0) && !sampled_rock[r] && to_real(c, prob) >= x_1;

            /*
            if (r >=0) {
                std::cout << "rock " << r << " sampled " << sampled_rock[r]
                          << " prob val " << prob << " " << to_real(c, prob)
                          << " action" << step.action() << std::endl;
            }
            */

            // sample
            if (step.action() == 4) {
                sampled_rock[r] = true;
            }
            else {
                formula = !formula;
            }

            xpomcp.add_soft_constraint(nrun, nstep, formula);

            nstep += 1;
        }
        nrun += 1;
    }

    auto models = xpomcp.build_rule({x_1}, {});

    double lw = to_double(models.loose_model.eval(x_1));
    double up = to_double(models.tight_model.eval(x_1));

    std::cout << "sample when confidence is >= [" << (lw * 100.0) << ", " <<
        (up * 100.0) << "] %" << std::endl;

    std::cout << "fail to satisfy " << xpomcp.unsatisfiable_steps().size()
              << " steps" << std::endl;

    return {lw, up};
}

int main(int argc, char *argv[]) {

    MCTS::PARAMS searchParams;
    TRACE_EXPERIMENT<ROCKSAMPLE_TRACE>::PARAMS expParams;
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

    real      = std::make_unique<ROCKSAMPLE>(size, number);
    simulator = std::make_unique<ROCKSAMPLE>(size, number);

    // ----- STANDARD START -----

    if (vm.count("setW") != 0) {
        real->SetRewardRange(W);
        simulator->SetRewardRange(W);
    }

    // initialize using five runs
    expParams.NumRuns = initrun;

    simulator->SetKnowledge(knowledge);

    TRACE_EXPERIMENT<ROCKSAMPLE_TRACE> experiment(*real, *simulator, expParams,
                                                  searchParams);

    if (vm.count("seed") != 0) {
        experiment.set_fixed_seed(random_seed);
    }

    std::unique_ptr<ROCKSAMPLE_TRACE> trace = std::make_unique<ROCKSAMPLE_TRACE>();
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

    auto models = build_rule(*trace, number);

    // ----- RANDOM STARTING STATES -----

    //for (int i = 0; i < 100; i++) {
    //    int x = UTILS::Random(11);
    //    int y = UTILS::Random(11);
    //    std::vector<double> belief(11);
    //    for (double &d : belief) {
    //        d = UTILS::RandomDouble(0.0, 1.0);
    //    }

    //    real = std::make_unique<ROCKSAMPLE>(size, number);
    //    simulator = std::make_unique<ROCKSAMPLE>(size, number, x, y, belief);

    //    expParams.NumRuns = 1;
    //    expParams.NumSteps = 1;
    //    simulator->SetKnowledge(knowledge);
    //    TRACE_EXPERIMENT<ROCKSAMPLE_TRACE> experiment(*real, *simulator,
    //                                                  expParams, searchParams);

    //    experiment.BuildTrace(*trace);

    //}

    //{
    //    double t = build_rule(*trace);
    //    std::ofstream f("initfile_random.json");
    //    f << std::setw(4) << trace->serialize() << std::endl;
    //}

    // ----- FORCE RARE BELIEFS -----

//#if 0
    double lw = models.first;
    double up = models.second;
    for (int i = 0; i < activeiter; i++) {
        expParams.NumRuns =  activerun;

        std::unique_ptr<SIMULATOR> real2 =
            std::make_unique<ROCKSAMPLE>(size, number);
        std::unique_ptr<SIMULATOR> simulator2 =
            std::make_unique<ROCKSAMPLE>(size, number);

        CREATOR_EXPERIMENT<ROCKSAMPLE_TRACE> experiment_creator(
            *real2, *simulator2, expParams, searchParams, lw, up);

        if (vm.count("seed") != 0) {
            experiment_creator.set_fixed_seed(++random_seed);
        }


        /*
        if (vm.count("random_seed") != 0) {
            experiment_creator.set_fixed_seed(random_seed);
        }
        */

        experiment_creator.BuildTrace(*trace);
        /*
        std::cout << "precision " << experiment_creator.rule_precision()
                  << " recall " << experiment_creator.rule_recall()
                  << std::endl;
        */

        auto t3 = build_rule(*trace, number);
        lw = t3.first;
        up = t3.second;

        std::ofstream f("initfile2.json");
        f << std::setw(4) << trace->serialize() << std::endl;
    }

    for (int i = 0; i < passiveiter; i++) {
        expParams.NumRuns =  passiverun;

        std::unique_ptr<SIMULATOR> real2 =
            std::make_unique<ROCKSAMPLE>(size, number);
        std::unique_ptr<SIMULATOR> simulator2 =
            std::make_unique<ROCKSAMPLE>(size, number);

        TRACE_EXPERIMENT<ROCKSAMPLE_TRACE> experiment(*real2, *simulator2,
                                                      expParams, searchParams);

        if (vm.count("seed") != 0) {
            experiment.set_fixed_seed(++random_seed);
        }

        /*
        if (vm.count("random_seed") != 0) {
            experiment_creator.set_fixed_seed(random_seed);
        }
        */

        experiment.BuildTrace(*trace);
        /*
        std::cout << "precision " << experiment_creator.rule_precision()
                  << " recall " << experiment_creator.rule_recall()
                  << std::endl;
        */

        auto t3 = build_rule(*trace, number);
        lw = t3.first;
        up = t3.second;

        std::ofstream f("initfile2.json");
        f << std::setw(4) << trace->serialize() << std::endl;
    }
//#endif
    // ----- EXPLORE UNSATISFIABLE STEPS -----

    /*
    for (const auto & u : uns_steps) {
        int x = u.x;
        int y = u.y;
        std::vector<double> belief(11);
        for (int i = 0; i < 11; i++) {
            belief[i] = u.rocks[i];
        }

        real = std::make_unique<ROCKSAMPLE>(size, number);
        simulator = std::make_unique<ROCKSAMPLE>(size, number, x, y, belief);

        expParams.NumRuns = 1;
        expParams.NumSteps = 1;
        simulator->SetKnowledge(knowledge);
        TRACE_EXPERIMENT<ROCKSAMPLE_TRACE> experiment(*real, *simulator,
                                                      expParams, searchParams);

        experiment.BuildTrace(*trace);
    }

    {
        double t = build_rule(*trace);
        std::ofstream f("initfile_informative.json");
        f << std::setw(4) << trace->serialize() << std::endl;
    }
    */
    return 0;
}

