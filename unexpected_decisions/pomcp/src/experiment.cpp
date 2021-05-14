#include "experiment.h"
#include "boost/timer.hpp"
#include <math.h>
#include "utils.h"

using namespace std;
using namespace UTILS;

EXPERIMENT::PARAMS::PARAMS()
:   NumRuns(1000),
    Testing(0),
    NumSteps(100000),
    SimSteps(1000),
    TimeOut(10000000), // 3600
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
    EXPERIMENT::PARAMS& expParams, MCTS::PARAMS& searchParams)
:   Real(real),
    Simulator(simulator),
    OutputFile(outputFile),
    ExpParams(expParams),
    SearchParams(searchParams)
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

void EXPERIMENT::Run(int i) // A single run is performed here
{
    boost::timer timer;

    if(ExpParams.Testing==1){ // Testing 1: difficulty sequence fixed given the run, observation fixed given the step
        cout << "Setting RandomSeed to " << i << endl;
        RandomSeed(i);
    }
    STATE* state;
    if(ExpParams.Testing==0 || ExpParams.Testing==1){ // Fixed state values
        state = Real.CreateStartState(); // Create start state i the real environment
    }
    if(ExpParams.Testing==2){ // Fixed state values
        std::vector<int> stateValues; //
        stateValues.clear();
        stateValues.push_back(2);
        stateValues.push_back(2);
        stateValues.push_back(2);
        stateValues.push_back(2);
        stateValues.push_back(2);
        stateValues.push_back(2);
        stateValues.push_back(2);
        stateValues.push_back(2);
        state = Real.CreateStartStateFixedValues(stateValues); 
    }

    if (SearchParams.Verbose >= 1){
        cout << "Displaying real simulator state";
        Real.DisplayState(*state, cout);
        cout << "Real state id: "; Real.DisplayStateId(*state,cout); cout << endl;
                
        outFilePolicyPerStep.open("policyPerStep.txt");
    }
    

    // STANDARD
    MCTS mcts(Simulator, SearchParams);
    
    // RELATIONSHIPS 
    int nConnComp=3;      // Number of relationships
    double relProbab=0.9; // Notice: it works only with probab=1.0 at the moment
    vector<double*>* stateVarRelationships;
    if(Simulator.GetRelKnowLevel()==1){
        stateVarRelationships=Simulator.CreateStateRelKnowledge(*state, nConnComp, relProbab);
        cout << "stateVarRelationships:" << endl;
        for(std::vector<double*>::iterator it = stateVarRelationships->begin() ; it != stateVarRelationships->end(); ++it){
            cout << (*it)[0] << ", " << (*it)[1] << ", " << (*it)[2] <<  endl;
        }
    }
    int nSamples=10000;
    
    double undiscountedReturn = 0.0;    // Initialize variables
    double discountedReturn = 0.0;
    double discount = 1.0;
    bool terminal = false;
    bool outOfParticles = false;
    int t;
    
    for (t = 0; t < ExpParams.NumSteps; t++) // for each step (i.e., moves in the real environment)
    {
        cout << endl << "--------------------------------------------------- Step " << t << " in the real world" << endl;
        int observation;
        double reward;
         
        cout << "Start simulations to generate local policy (i.e., generate MCTS)" << endl;
        int action = mcts.SelectAction();
        cout << "End simulations to generate local policy" << endl<< endl;
        cout << "Perform action in the real environment" << endl;
        
        if (SearchParams.Verbose >= 1)
        {
            outFilePolicyPerStep << "Step " << t << endl;
            mcts.DisplayStatistics(outFilePolicyPerStep);
            mcts.DisplayPolicy(6, outFilePolicyPerStep);
            mcts.DisplayValue(6, outFilePolicyPerStep);
            outFilePolicyPerStep << endl << endl;
        }

        if(ExpParams.Testing==1){
            cout << "Setting RandomSeed to " << t << endl;
            RandomSeed(t);
        }
        terminal = Real.Step(*state, action, observation, reward, mcts.BeliefState());
        
        
        Results.Reward.Add(reward);                 // Add the reward to the results
        undiscountedReturn += reward;               // Add reward to undiscountedReturn
        discountedReturn += reward * discount;      // Add discounted reward to discountedReturn
        discount *= Real.GetDiscount();             // Update discount

        if (SearchParams.Verbose >= 1)              // Display
        {
            Real.DisplayAction(action, cout);
            cout << "New real state" << endl;
            Real.DisplayState(*state, cout);
            Real.DisplayObservation(*state, observation, cout);
            Real.DisplayReward(reward, cout);
        }

        if (terminal)                               // If terminal then finish
        {
            cout << "Terminated" << endl;
            // Save last belief
            //Simulator.DisplayBeliefDistribution(mcts.BeliefState(), mcts.outFileBeliefPerStep);
            //mcts.outFileBeliefPerStep << endl;
            break;
        }
        
        if(Simulator.GetRelKnowLevel()==1){ // Relationship knowledge
            outOfParticles = !mcts.Update(action, observation, reward, stateVarRelationships);
        }
        else{
            outOfParticles = !mcts.Update(action, observation, reward); // Update the mtcs with current action, observation and reward 
        }
        
        if (outOfParticles)
            break;

        if (timer.elapsed() > ExpParams.TimeOut)
        {
            cout << "Timed out after " << t << " steps in "
                << Results.Time.GetTotal() << "seconds" << endl;
            break;
        }
    } // End for

    if (outOfParticles) // Manage the outOfParticlesCase if needed
    {
        cout << "Out of particles, finishing episode with SelectRandom" << endl;
        HISTORY history = mcts.GetHistory();
        while (++t < ExpParams.NumSteps)
        {
            int observation;
            double reward;

            int action = Simulator.SelectRandom(*state, history, mcts.GetStatus());
            terminal = Real.Step(*state, action, observation, reward, mcts.BeliefState());

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
    if (SearchParams.Verbose >= 1)
    {
        outFilePerfPerRun << undiscountedReturn << ", " << discountedReturn << ", "  << timer.elapsed() << endl;
        Real.DisplayStateHist(*state, "stateEvolution.csv");
        outFilePolicyPerStep.close();
    }
}


void EXPERIMENT::MultiRun()
{
    int status;
    string runFolder="Run_";
    const char* folder;
    for (int n = 0; n < ExpParams.NumRuns; n++) // Loop on n runs
    {
        cout << endl << "-------------------------------------------------------------- EXPERIMENT::MultiRun: Starting run " << n + 1 << " with "
            << SearchParams.NumSimulations << " simulations... " << endl;
        if (SearchParams.Verbose >= 1){
            folder=(runFolder+to_string(n)).c_str();
            status=mkdir(folder, 0777);
            status=chdir(folder);
            outFilePerfPerRun << SearchParams.NumSimulations << ", " << n << ", ";
        }
        Run(n);
        if (Results.Time.GetTotal() > ExpParams.TimeOut)
        {
            cout << "Timed out after " << n << " runs in "
                << Results.Time.GetTotal() << "seconds" << endl;
            break;
        }
        if (SearchParams.Verbose >= 1){
            status=chdir("..");
        }
    }
}

void EXPERIMENT::DiscountedReturn() // Simulation starts here 
{
    cout << "EXPERIMENT::DiscountedReturn: Main runs" << endl;
    OutputFile << "Simulations\tRuns\tUndiscounted return\tUndiscounted error\tDiscounted return\tDiscounted error\tTime\n";

    SearchParams.MaxDepth = Simulator.GetHorizon(ExpParams.Accuracy, ExpParams.UndiscountedHorizon);
    ExpParams.SimSteps = Simulator.GetHorizon(ExpParams.Accuracy, ExpParams.UndiscountedHorizon);
    if(ExpParams.NumSteps==100000)
        ExpParams.NumSteps = Real.GetHorizon(ExpParams.Accuracy, ExpParams.UndiscountedHorizon);
    
    string expFolderName="myExp";
    
    int status;
    if (SearchParams.Verbose >= 1){
        
        status=mkdir(expFolderName.c_str(), 0777);
        status=chdir(expFolderName.c_str());
    }
    string nSimPow="nSimPow_";
    string sPerfPerRun="perfPerRun_nSim_";
    string sCsv=".csv";
    string sPerfPerNSim="perfPerNSim.csv";
    outFilePerfPerNSim.open(sPerfPerNSim);
    outFilePerfPerNSim << "NSim, NRuns, UndiscountedReturn, UndiscountedError, DiscountedReturn, DiscountedError, TimePomcp" << endl;
            
    
    for (int i = ExpParams.MinDoubles; i <= ExpParams.MaxDoubles; i++)
    {
        
        if (SearchParams.Verbose >= 1){
            
            status=mkdir((nSimPow+to_string(i)).c_str(), 0777);
            status=chdir((nSimPow+to_string(i)).c_str());
            
            outFilePerfPerRun.open(sPerfPerRun+to_string(i)+sCsv);
            outFilePerfPerRun << "NSim, Run, UndiscountedReturn, DiscountedReturn, Time" << endl;
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
        
        if (SearchParams.Verbose >= 1){
            outFilePerfPerRun.close();
            status=chdir("..");
            outFilePerfPerNSim << SearchParams.NumSimulations << ", "
            << Results.Time.GetCount() << ", "
            << Results.UndiscountedReturn.GetMean() << ", "
            << Results.UndiscountedReturn.GetStdErr() << ", "
            << Results.DiscountedReturn.GetMean() << ", "
            << Results.DiscountedReturn.GetStdErr() << ", "
            << Results.Time.GetMean() << endl;
        }
    }
    if (SearchParams.Verbose >= 1){
        outFilePerfPerNSim.close();
        status=chdir("..");
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
        Run(0);

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
