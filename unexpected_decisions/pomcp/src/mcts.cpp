#include "mcts.h"
#include "testsimulator.h"
#include <math.h>

#include <algorithm>

using namespace std;
using namespace UTILS;

//-----------------------------------------------------------------------------

MCTS::PARAMS::PARAMS()
:   Verbose(0),
    MaxDepth(100),
    NumSimulations(1000),
    NumStartStates(1000),
    UseTransforms(true),
    NumTransforms(0),
    MaxAttempts(0),
    ExpandCount(1),
    ExplorationConstant(1),
    UseRave(false),
    RaveDiscount(1.0),
    RaveConstant(0.01),
    DisableTree(false)
{
}


// STANDARD
MCTS::MCTS(const SIMULATOR& simulator, const PARAMS& params)
:   Simulator(simulator),   // Gets the simulator 
    Params(params),         // Gets the parameters
    TreeDepth(0)            // Sets tree depth=0
{
    cout << "Generating new MCTS#########################" << endl;
    VNODE::NumChildren = Simulator.GetNumActions();         // Set number of actions
    QNODE::NumChildren = Simulator.GetNumObservations();    // Set number of obs

    Root = ExpandNode(Simulator.CreateStartState()); // Creates a start state (e.g., in rocksample with StartPos and Rocks (positions already in the simulator grid, values set here))
                                                     // End its (action) children QNODES and puts knowledge in them 
    for (int i = 0; i < Params.NumStartStates; i++) {
        Root->Beliefs().AddSample(Simulator.CreateStartState()); // Add states samples to root beliefs
    }
    if (Params.Verbose >= 1){
        outFileBeliefPerStep.open("beliefsPerStep.csv");
    }
}

MCTS::MCTS(const SIMULATOR& simulator, const STATE* realState, const PARAMS& params)
:   Simulator(simulator),   // Gets the simulator 
    Params(params),         // Gets the parameters
    TreeDepth(0)            // Sets tree depth=0
{
    cout << "Generating new MCTS#########################" << endl;
    VNODE::NumChildren = Simulator.GetNumActions();         // Set number of actions
    QNODE::NumChildren = Simulator.GetNumObservations();    // Set number of obs

    Root = ExpandNode(Simulator.Copy(*realState)); // Creates a start state (e.g., in rocksample with StartPos and Rocks (positions already in the simulator grid, values set here))
                                                     // End its (action) children QNODES and puts knowledge in them 
    for (int i = 0; i < Params.NumStartStates; i++) 
        Root->Beliefs().AddSample(Simulator.Copy(*realState)); // Add states samples to root beliefs 
    // Simulator.Copy(
    if (Params.Verbose >= 1){
        outFileBeliefPerStep.open("beliefsPerStep.csv");
        //outFileUCT_tmp.open("uct_tmp.txt");
    }
}

MCTS::MCTS(const SIMULATOR& simulator, vector<double*>* stateVarRelationships, const PARAMS& params)
:   Simulator(simulator),   // Gets the simulator 
    Params(params),         // Gets the parameters
    TreeDepth(0)            // Sets tree depth=0
{
    VNODE::NumChildren = Simulator.GetNumActions();         // Set number of actions
    QNODE::NumChildren = Simulator.GetNumObservations();    // Set number of obs
    
    Root = ExpandNode(Simulator.CreateStartState(stateVarRelationships));
    for (int i = 0; i < Params.NumStartStates; i++) 
        Root->Beliefs().AddSample(Simulator.CreateStartState(stateVarRelationships));
    if (Params.Verbose >= 1){
        outFileBeliefPerStep.open("beliefsPerStep.csv");
    }
}

MCTS::MCTS(const SIMULATOR& simulator, vector<double*>* stateVarRelationships, int nAllParticles, const PARAMS& params)
:   Simulator(simulator),   // Gets the simulator 
    Params(params),         // Gets the parameters
    TreeDepth(0)            // Sets tree depth=0
{
    VNODE::NumChildren = Simulator.GetNumActions();         // Set number of actions
    QNODE::NumChildren = Simulator.GetNumObservations();    // Set number of obs
    
    vector<STATE*>* allParticles=new vector<STATE*>(nAllParticles);
    vector<double> allParticleProb(nAllParticles);
    STATE* particle;
    double prob;
    for (int i = 0; i < nAllParticles; i++){
        particle=Simulator.CreateStartState(); // Creates a random particle
        (*allParticles)[i]=particle; // Add particle to allParticles
        prob=Simulator.ComputeParticleProbability(*particle,stateVarRelationships); // Compute particle probability
        allParticleProb[i]=prob;
    }
    
    // Compute cumulative probability
    double sumAllParticleProb=0.0;
    for(int i=0; i<nAllParticles; i++){
        sumAllParticleProb+=allParticleProb[i];
    }
    vector<double> allParticleCumProbs(nAllParticles+1);
    allParticleCumProbs[0]=0.0;
    for(unsigned long i=0; i<allParticleProb.size(); i++){
        allParticleCumProbs[i+1]=allParticleCumProbs[i]+allParticleProb[i]/sumAllParticleProb;
    }
    Root = ExpandNode(Simulator.CreateStartState(allParticles,allParticleCumProbs));
    for (int i = 0; i < Params.NumStartStates; i++) 
        Root->Beliefs().AddSample(Simulator.CreateStartState(allParticles,allParticleCumProbs));

    if (Params.Verbose >= 1){
        outFileBeliefPerStep.open("beliefsPerStep.csv");
    }
    
}

MCTS::~MCTS()
{
    VNODE::Free(Root, Simulator);
    VNODE::FreeAll();
    outFileBeliefPerStep.close();
}

// Updates the tree removing the current root, moving it to the node hao. Updates the history.
bool MCTS::Update(int action, int observation, double reward)
{
    if (Params.Verbose >= 1)
        cout << "In MCTS::Update" << endl;
    History.Add(action, observation);
    BELIEF_STATE beliefs;

    QNODE& qnode = Root->Child(action);         // action QNODE 
    VNODE* vnode = qnode.Child(observation);    // history VNODE
    if (vnode)
    {
        if (Params.Verbose >= 1)
            cout << "Matched " << vnode->Beliefs().GetNumSamples() << " states" << endl;
        beliefs.Copy(vnode->Beliefs(), Simulator);
    }
    else
    {
        if (Params.Verbose >= 1)
            cout << "No matching node found" << endl;
    }

    if (Params.UseTransforms)
        AddTransforms(Root, beliefs);

    if (beliefs.Empty() && (!vnode || vnode->Beliefs().Empty()))
        return false;

    if (Params.Verbose >= 2)
        Simulator.DisplayBeliefs(beliefs, cout);

    // Find a state to initialise prior (only requires fully observed state)
    const STATE* state = 0;
    if (vnode && !vnode->Beliefs().Empty())
        state = vnode->Beliefs().GetSample(0);
    else    
        state = beliefs.GetSample(0);

    // Delete old tree and create new root
    VNODE::Free(Root, Simulator);
    VNODE* newRoot = ExpandNode(state);
    newRoot->Beliefs() = beliefs;   
    Root = newRoot;
    return true;
}

bool MCTS::Update(int action, int observation, double reward, std::vector<double*>* stateVarRelationships)
{
    if (Params.Verbose >= 1)
        cout << "In MCTS::Update" << endl;
    int nRels=stateVarRelationships->size();
    History.Add(action, observation);
    BELIEF_STATE beliefs;

    QNODE& qnode = Root->Child(action);
    VNODE* vnode = qnode.Child(observation);
    if (vnode)
    {
        if (Params.Verbose >= 1)
            cout << "Matched " << vnode->Beliefs().GetNumSamples() << " states" << endl;
        beliefs.Copy(vnode->Beliefs(), Simulator);
    }
    else
    {
        if (Params.Verbose >= 1)
            cout << "No matching node found" << endl;
    }

    if (Params.UseTransforms)
        AddTransforms(Root, beliefs, stateVarRelationships);
    if (beliefs.Empty() && (!vnode || vnode->Beliefs().Empty()))
        return false;

    if (Params.Verbose >= 2)
        Simulator.DisplayBeliefs(beliefs, cout);

    const STATE* state = 0;
    if (vnode && !vnode->Beliefs().Empty())
        state = vnode->Beliefs().GetSample(0);
    else    
        state = beliefs.GetSample(0);

    // Delete old tree and create new root
    VNODE::Free(Root, Simulator);
    VNODE* newRoot = ExpandNode(state);
    newRoot->Beliefs() = beliefs;   
    Root = newRoot;
    return true;
}

int MCTS::SelectAction()
{
    if (Params.DisableTree)
        RolloutSearch();
    else
        UCTSearch();
    return GreedyUCB(Root, false);
}

void MCTS::RolloutSearch()
{
	std::vector<double> totals(Simulator.GetNumActions(), 0.0);
	int historyDepth = History.Size();
	std::vector<int> legal;
	assert(BeliefState().GetNumSamples() > 0);
	Simulator.GenerateLegal(*BeliefState().GetSample(0), GetHistory(), legal, GetStatus());
	random_shuffle(legal.begin(), legal.end());

	for (int i = 0; i < Params.NumSimulations; i++)
	{
		int action = legal[i % legal.size()];
		STATE* state = Root->Beliefs().CreateSample(Simulator);
		Simulator.Validate(*state);

		int observation;
		double immediateReward, delayedReward, totalReward;

                const BELIEF_STATE& beliefsTmp=BELIEF_STATE();
		bool terminal = Simulator.Step(*state, action, observation, immediateReward, beliefsTmp);

		VNODE*& vnode = Root->Child(action).Child(observation);
		if (!vnode && !terminal)
		{
			vnode = ExpandNode(state);
			AddSample(vnode, *state);
		}
		History.Add(action, observation);

		delayedReward = Rollout(*state); 
		totalReward = immediateReward + Simulator.GetDiscount() * delayedReward;
		Root->Child(action).Value.Add(totalReward);

		Simulator.FreeState(state);
		History.Truncate(historyDepth);
	}
}

void MCTS::UCTSearch()
{
    ClearStatistics();
    int historyDepth = History.Size();
    
    if(Params.Verbose >= 1){
        cout << "MCTS::UCTSearch >> Number of samples in the belief state: " << Root->Beliefs().GetNumSamples()<< endl;
        cout << "MCTS::UCTSearch >> Belief distribution: ";
        Simulator.DisplayBeliefDistribution(Root->Beliefs(), cout);
        cout << endl;
        Simulator.DisplayBeliefDistribution(Root->Beliefs(), outFileBeliefPerStep);
        outFileBeliefPerStep << endl;
        
    }
    if (Params.Verbose >= 2)
    {
        cout << "MCTS::UCTSearch >> Root belief states"<< endl;
        cout << "------ BELIEF STATE -------" <<  endl;
        for (int i = 0; i < Root->Beliefs().GetNumSamples(); i++) // For each simulation
        {
            const STATE* state = Root->Beliefs().GetSample(i);
            cout << "State " << i << ":"<< endl;
            Simulator.DisplayState(*state, cout);
            Simulator.DisplayStateId(*state, cout);
            cout << endl;
        }
    }

    for (int n = 0; n < Params.NumSimulations; n++)
    {
        STATE* state = Root->Beliefs().CreateSample(Simulator);
        Simulator.Validate(*state);
        if (Params.Verbose >= 2)
        {
            cout << "Sim " << n << ", "; Simulator.DisplayStateId(*state, cout); cout << endl;
        }
        
        Status.Phase = SIMULATOR::STATUS::TREE;
        if (Params.Verbose >= 2)
        {
            cout << "Starting simulation " << n << endl;
            Simulator.DisplayState(*state, cout);
        }
        TreeDepth = 0;
        PeakTreeDepth = 0;
        double totalReward = SimulateV(*state, Root);
        StatTotalReward.Add(totalReward);
        StatTreeDepth.Add(PeakTreeDepth);

        if (Params.Verbose >= 2)
            cout << "Total reward = " << totalReward << endl;
        if (Params.Verbose >= 3)
            DisplayValue(4, cout);

        Simulator.FreeState(state);
        History.Truncate(historyDepth);
    }
    
    DisplayStatistics(cout);
}

double MCTS::SimulateV(STATE& state, VNODE* vnode)
{
    int action = GreedyUCB(vnode, true);
    PeakTreeDepth = TreeDepth;
    if (TreeDepth >= Params.MaxDepth)
        return 0;

    if (TreeDepth == 1)
        AddSample(vnode, state);

    QNODE& qnode = vnode->Child(action);
    double totalReward = SimulateQ(state, qnode, action);
    vnode->Value.Add(totalReward);
    AddRave(vnode, totalReward);
    return totalReward;
}

double MCTS::SimulateQ(STATE& state, QNODE& qnode, int action)
{
    int observation;
    double immediateReward, delayedReward = 0;

    if (Simulator.HasAlpha())
        Simulator.UpdateAlpha(qnode, state);
    const BELIEF_STATE& beliefsTmp=BELIEF_STATE();
    bool terminal = Simulator.Step(state, action, observation, immediateReward, beliefsTmp);
    assert(observation >= 0 && observation < Simulator.GetNumObservations());
    History.Add(action, observation);

    if (Params.Verbose >= 3)
    {
        Simulator.DisplayAction(action, cout);
        Simulator.DisplayObservation(state, observation, cout);
        Simulator.DisplayReward(immediateReward, cout);
        Simulator.DisplayState(state, cout);
    }
    VNODE*& vnode = qnode.Child(observation);
    
    if (!vnode && !terminal && qnode.Value.GetCount() >= Params.ExpandCount)
    {
        vnode = ExpandNode(&state);
    }
    if (!terminal)
    {
        TreeDepth++;
        if (vnode)
            delayedReward = SimulateV(state, vnode);
        else
            delayedReward = Rollout(state);
        TreeDepth--;
    }

    double totalReward = immediateReward + Simulator.GetDiscount() * delayedReward;
    qnode.Value.Add(totalReward);
    return totalReward;
}

void MCTS::AddRave(VNODE* vnode, double totalReward)
{
    double totalDiscount = 1.0;
    for (int t = TreeDepth; t < History.Size(); ++t)
    {
        QNODE& qnode = vnode->Child(History[t].Action);
        qnode.AMAF.Add(totalReward, totalDiscount);
        totalDiscount *= Params.RaveDiscount;
    }
}

VNODE* MCTS::ExpandNode(const STATE* state)
{
    VNODE* vnode = VNODE::Create();
    vnode->Value.Set(0, 0);
    Simulator.Prior(state, History, vnode, Status);

    if (Params.Verbose >= 2)
    {
        cout << "Expanding node: ";
        History.Display(cout);
        cout << endl;
    }

    return vnode;
}

void MCTS::AddSample(VNODE* node, const STATE& state)
{
    STATE* sample = Simulator.Copy(state);
    node->Beliefs().AddSample(sample);
    if (Params.Verbose >= 2)
    {
        cout << "Adding sample:" << endl;
        Simulator.DisplayState(*sample, cout);
    }
}

int MCTS::GreedyUCB(VNODE* vnode, bool ucb) const
{
    static vector<int> besta;
    besta.clear();
    double bestq = -Infinity;
    int N = vnode->Value.GetCount();
    double logN = log(N + 1);
    bool hasalpha = Simulator.HasAlpha();
    int tmp;

    for (int action = 0; action < Simulator.GetNumActions(); action++)
    {
        double q, alphaq;
        int n, alphan;

        QNODE& qnode = vnode->Child(action);
        q = qnode.Value.GetValue(); tmp=q;
        n = qnode.Value.GetCount();

        if (Params.UseRave && qnode.AMAF.GetCount() > 0)
        {
            double n2 = qnode.AMAF.GetCount();
            double beta = n2 / (n + n2 + Params.RaveConstant * n * n2);
            q = (1.0 - beta) * q + beta * qnode.AMAF.GetValue();
        }

        if (hasalpha && n > 0)
        {
            Simulator.AlphaValue(qnode, alphaq, alphan);
            q = (n * q + alphan * alphaq) / (n + alphan);
        }

        if (ucb){
            q += FastUCB(N, n, logN);
        }

        if (q >= bestq)
        {
            if (q > bestq)
                besta.clear();
            bestq = q;
            besta.push_back(action);
        }
    }

    assert(!besta.empty());
    int selectedAction = besta[Random(besta.size())];
    return selectedAction;
}

double MCTS::Rollout(STATE& state)
{
    Status.Phase = SIMULATOR::STATUS::ROLLOUT;
    if (Params.Verbose >= 3)
        cout << "Starting rollout" << endl;

    double totalReward = 0.0;
    double discount = 1.0;
    bool terminal = false;
    int numSteps;
    for (numSteps = 0; numSteps + TreeDepth < Params.MaxDepth && !terminal; ++numSteps)
    {
        int observation;
        double reward;
        int action = Simulator.SelectRandom(state, History, Status);
        
        const BELIEF_STATE& beliefsTmp=BELIEF_STATE();
        terminal = Simulator.Step(state, action, observation, reward, beliefsTmp);
        History.Add(action, observation);

        if (Params.Verbose >= 4)
        {
            Simulator.DisplayAction(action, cout);
            Simulator.DisplayObservation(state, observation, cout);
            Simulator.DisplayReward(reward, cout);
            Simulator.DisplayState(state, cout);
        }

        totalReward += reward * discount;
        discount *= Simulator.GetDiscount();
    }

    StatRolloutDepth.Add(numSteps);
    if (Params.Verbose >= 3)
        cout << "Ending rollout after " << numSteps
            << " steps, with total reward " << totalReward << endl;
    return totalReward;
}

void MCTS::AddTransforms(VNODE* root, BELIEF_STATE& beliefs)
{
    
    int attempts = 0, added = 0;

    while (added < Params.NumTransforms && attempts < Params.MaxAttempts)
    {
        STATE* transform = CreateTransform();
        if (transform)
        {
            beliefs.AddSample(transform);
            added++;
        }
        attempts++;
    }

    if (Params.Verbose >= 1)
    {
        cout << "Created " << added << " local transformations out of "
            << attempts << " attempts" << endl;
    }
}

void MCTS::AddTransforms(VNODE* root, BELIEF_STATE& beliefs, std::vector<double*>* stateVarRelationships)
{
    if (Params.Verbose >= 2){
        cout << "In MCTS::AddTransforms" << endl;
        cout << "Params.NumTransforms: " << Params.NumTransforms << "Params.MaxAttempts: " << Params.MaxAttempts << endl;
    }
    int attempts = 0, added = 0;

    while (added < Params.NumTransforms && attempts < Params.MaxAttempts)
    {
        STATE* transform = CreateTransform(stateVarRelationships);
        if (transform)
        {
            beliefs.AddSample(transform);
            added++;
        }
        attempts++;
    }

    if (Params.Verbose >= 1)
    {
        cout << "Created " << added << " local transformations out of "
            << attempts << " attempts" << endl;
    }
}

STATE* MCTS::CreateTransform() const
{
    int stepObs;
    double stepReward;

    STATE* state = Root->Beliefs().CreateSample(Simulator);
    const BELIEF_STATE& beliefsTmp=BELIEF_STATE();
    Simulator.Step(*state, History.Back().Action, stepObs, stepReward, beliefsTmp);
    if (Simulator.LocalMove(*state, History, stepObs, Status))
        return state;
    Simulator.FreeState(state);
    return 0;
}

STATE* MCTS::CreateTransform(std::vector<double*>* stateVarRelationships) const
{
    int stepObs;
    double stepReward;

    STATE* state = Root->Beliefs().CreateSample(Simulator);
    const BELIEF_STATE& beliefsTmp=BELIEF_STATE();
    Simulator.Step(*state, History.Back().Action, stepObs, stepReward, beliefsTmp);
    if (Params.Verbose >= 2){
        cout << "Performing local move to state " << endl;
        Simulator.DisplayState(*state,cout);
    }
    if (Simulator.LocalMove(*state, History, stepObs, Status, stateVarRelationships))
        return state;
    Simulator.FreeState(state);
    return 0;
}

double MCTS::UCB[UCB_N][UCB_n];
bool MCTS::InitialisedFastUCB = true;

void MCTS::InitFastUCB(double exploration)
{
    cout << "Initialising fast UCB table... ";
    for (int N = 0; N < UCB_N; ++N)
        for (int n = 0; n < UCB_n; ++n)
            if (n == 0)
                UCB[N][n] = Infinity;
            else
                UCB[N][n] = exploration * sqrt(log(N + 1) / n);
    cout << "done" << endl;
    InitialisedFastUCB = true;
}

inline double MCTS::FastUCB(int N, int n, double logN) const
{
    if (InitialisedFastUCB && N < UCB_N && n < UCB_n)
        return UCB[N][n];

    if (n == 0)
        return Infinity;
    else
        return Params.ExplorationConstant * sqrt(logN / n);
}

void MCTS::ClearStatistics()
{
    StatTreeDepth.Clear();
    StatRolloutDepth.Clear();
    StatTotalReward.Clear();
}

void MCTS::DisplayStatistics(ostream& ostr) const
{
    if (Params.Verbose >= 1)
    {
        StatTreeDepth.Print("Tree depth", ostr);
        StatRolloutDepth.Print("Rollout depth", ostr);
        StatTotalReward.Print("Total reward", ostr);
    }

    if (Params.Verbose >= 2)
    {
        ostr << "Policy after " << Params.NumSimulations << " simulations" << endl;
        DisplayPolicy(6, ostr);
        ostr << "Values after " << Params.NumSimulations << " simulations" << endl;
        DisplayValue(6, ostr);
    }
}

void MCTS::DisplayValue(int depth, ostream& ostr) const
{
    HISTORY history;
    ostr << "MCTS Values:" << endl;
    Root->DisplayValue(history, depth, ostr);
}

void MCTS::DisplayPolicy(int depth, ostream& ostr) const
{
    HISTORY history;
    ostr << "MCTS Policy:" << endl;
    Root->DisplayPolicy(history, depth, ostr);
}

//-----------------------------------------------------------------------------

void MCTS::UnitTest()
{
    UnitTestGreedy();
    UnitTestUCB();
    UnitTestRollout();
    for (int depth = 1; depth <= 3; ++depth)
        UnitTestSearch(depth);
}

void MCTS::UnitTestGreedy()
{
    TEST_SIMULATOR testSimulator(5, 5, 0);
    PARAMS params;
    MCTS mcts(testSimulator, params);
    int numAct = testSimulator.GetNumActions();
    int numObs = testSimulator.GetNumObservations();

    VNODE* vnode = mcts.ExpandNode(testSimulator.CreateStartState());
    vnode->Value.Set(1, 0);
    vnode->Child(0).Value.Set(0, 1);
    for (int action = 1; action < numAct; action++)
        vnode->Child(action).Value.Set(0, 0);
    assert(mcts.GreedyUCB(vnode, false) == 0);
}

void MCTS::UnitTestUCB()
{
    TEST_SIMULATOR testSimulator(5, 5, 0);
    PARAMS params;
    MCTS mcts(testSimulator, params);
    int numAct = testSimulator.GetNumActions();
    int numObs = testSimulator.GetNumObservations();

    // With equal value, action with lowest count is selected
    VNODE* vnode1 = mcts.ExpandNode(testSimulator.CreateStartState());
    vnode1->Value.Set(1, 0);
    for (int action = 0; action < numAct; action++)
        if (action == 3)
            vnode1->Child(action).Value.Set(99, 0);
        else
            vnode1->Child(action).Value.Set(100 + action, 0);
    assert(mcts.GreedyUCB(vnode1, true) == 3);

    // With high counts, action with highest value is selected
    VNODE* vnode2 = mcts.ExpandNode(testSimulator.CreateStartState());
    vnode2->Value.Set(1, 0);
    for (int action = 0; action < numAct; action++)
        if (action == 3)
            vnode2->Child(action).Value.Set(99 + numObs, 1);
        else
            vnode2->Child(action).Value.Set(100 + numAct - action, 0);
    assert(mcts.GreedyUCB(vnode2, true) == 3);

    // Action with low value and low count beats actions with high counts
    VNODE* vnode3 = mcts.ExpandNode(testSimulator.CreateStartState());
    vnode3->Value.Set(1, 0);
    for (int action = 0; action < numAct; action++)
        if (action == 3)
            vnode3->Child(action).Value.Set(1, 1);
        else
            vnode3->Child(action).Value.Set(100 + action, 1);
    assert(mcts.GreedyUCB(vnode3, true) == 3);

    // Actions with zero count is always selected
    VNODE* vnode4 = mcts.ExpandNode(testSimulator.CreateStartState());
    vnode4->Value.Set(1, 0);
    for (int action = 0; action < numAct; action++)
        if (action == 3)
            vnode4->Child(action).Value.Set(0, 0);
        else
            vnode4->Child(action).Value.Set(1, 1);
    assert(mcts.GreedyUCB(vnode4, true) == 3);
}

void MCTS::UnitTestRollout()
{
    TEST_SIMULATOR testSimulator(2, 2, 0);
    PARAMS params;
    params.NumSimulations = 1000;
    params.MaxDepth = 10;
    MCTS mcts(testSimulator, params);
    double totalReward;
    for (int n = 0; n < mcts.Params.NumSimulations; ++n)
    {
        STATE* state = testSimulator.CreateStartState();
        mcts.TreeDepth = 0;
        totalReward += mcts.Rollout(*state);
    }
    double rootValue = totalReward / mcts.Params.NumSimulations;
    double meanValue = testSimulator.MeanValue();
    assert(fabs(meanValue - rootValue) < 0.1);
}

void MCTS::UnitTestSearch(int depth)
{
    TEST_SIMULATOR testSimulator(3, 2, depth);
    PARAMS params;
    params.MaxDepth = depth + 1;
    params.NumSimulations = pow(10, depth + 1);
    MCTS mcts(testSimulator, params);
    mcts.UCTSearch();
    double rootValue = mcts.Root->Value.GetValue();
    double optimalValue = testSimulator.OptimalValue();
    assert(fabs(optimalValue - rootValue) < 0.1);
}

//-----------------------------------------------------------------------------
