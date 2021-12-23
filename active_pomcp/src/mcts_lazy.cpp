#include "mcts_lazy.h"
#include "testsimulator.h"
#include <cmath>

#include <algorithm>

using namespace std;
using namespace UTILS;

//-----------------------------------------------------------------------------

MCTS_LAZY::MCTS_LAZY(const SIMULATOR_LAZY& simulator, const PARAMS& params)
:   Simulator(simulator),
    Params(params),
    TreeDepth(0),
    PeakTreeDepth(0)
{
    VNODE_LAZY::NumChildren = Simulator.GetNumActions();
    //QNODE_LAZY::NumChildren = Simulator.GetNumObservations();

    Root = ExpandNode(Simulator.CreateStartState());

    for (int i = 0; i < Params.NumStartStates; i++) {
        Root->Beliefs().AddSample(Simulator.CreateStartState());
    }
}

MCTS_LAZY::~MCTS_LAZY()
{
    VNODE_LAZY::Free(Root, Simulator);
    VNODE_LAZY::FreeAll();
}

bool MCTS_LAZY::Update(int action, SIMULATOR_LAZY::observation_t observation, double reward)
{
    History.Add(action, observation);
    BELIEF_STATE beliefs;

    // Find matching vnode from the rest of the tree
    QNODE_LAZY& qnode = Root->Child(action);
    VNODE_LAZY* vnode = qnode.Child(observation);
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

    // Generate transformed states to avoid particle deprivation
    if (Params.UseTransforms)
        AddTransforms(Root, beliefs);

    // If we still have no particles, fail
    if (beliefs.Empty() && (!vnode || vnode->Beliefs().Empty()))
        return false;

    if (Params.Verbose >= 1)
        Simulator.DisplayBeliefs(beliefs, cout);

    // Find a state to initialise prior (only requires fully observed state)
    const STATE* state = 0;
    if (vnode && !vnode->Beliefs().Empty())
        state = vnode->Beliefs().GetSample(0);
    else
        state = beliefs.GetSample(0);

    // Delete old tree and create new root
    VNODE_LAZY::Free(Root, Simulator);
    VNODE_LAZY* newRoot = ExpandNode(state);
    //std::swap(newRoot->Beliefs(), beliefs);
    newRoot->Beliefs().Move(beliefs);
    Root = newRoot;
    return true;
}

int MCTS_LAZY::SelectAction()
{
    if (Params.DisableTree)
        RolloutSearch();
    else
        UCTSearch();

    /*
    if constexpr (false)
        return Simulator.shield_action(Root->Beliefs(), GreedyUCB(Root, false));
    else
    */
    return GreedyUCB(Root, false);
}

void MCTS_LAZY::RolloutSearch()
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

        SIMULATOR_LAZY::observation_t observation;
		double immediateReward, delayedReward, totalReward;
		bool terminal = Simulator.Step(*state, action, observation, immediateReward);

		VNODE_LAZY*& vnode = Root->Child(action).Child(observation);
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

void MCTS_LAZY::UCTSearch()
{
    ClearStatistics();
    int historyDepth = History.Size();

    // compute legal action in root
    legal_actions.clear();

    if (Params.use_shield) {
        // initialize legal actions
        Simulator.pre_shield(Root->Beliefs(), legal_actions);
    }

    for (int n = 0; n < Params.NumSimulations; n++)
    {
        STATE* state = Root->Beliefs().CreateSample(Simulator);
        Simulator.Validate(*state);
        Status.Phase = SIMULATOR::STATUS::TREE;
        if (Params.Verbose >= 2)
        {
            cout << "Starting simulation" << endl;
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

double MCTS_LAZY::SimulateV(STATE& state, VNODE_LAZY* vnode)
{
    int action = GreedyUCB(vnode, true);

    PeakTreeDepth = TreeDepth;
    if (TreeDepth >= Params.MaxDepth) // search horizon reached
        return 0;

    if (TreeDepth == 1)
        AddSample(vnode, state);

    QNODE_LAZY& qnode = vnode->Child(action);
    double totalReward = SimulateQ(state, qnode, action);
    vnode->Value.Add(totalReward);
    AddRave(vnode, totalReward);
    return totalReward;
}

double MCTS_LAZY::SimulateQ(STATE& state, QNODE_LAZY& qnode, int action)
{
    SIMULATOR::observation_t observation;
    double immediateReward, delayedReward = 0;

    /* DISABLED
    if (Simulator.HasAlpha())
        Simulator.UpdateAlpha(qnode, state);
    */

    bool terminal = Simulator.Step(state, action, observation, immediateReward);
    //assert(observation >= 0 && observation < Simulator.GetNumObservations());
    History.Add(action, observation);

    if (Params.Verbose >= 3)
    {
        Simulator.DisplayAction(action, cout);
        Simulator.DisplayObservation(state, observation, cout);
        Simulator.DisplayReward(immediateReward, cout);
        Simulator.DisplayState(state, cout);
    }

    VNODE_LAZY*& vnode = qnode.Child(observation);
    if (!vnode && !terminal && qnode.Value.GetCount() >= Params.ExpandCount)
        vnode = ExpandNode(&state);

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

void MCTS_LAZY::AddRave(VNODE_LAZY* vnode, double totalReward)
{
    double totalDiscount = 1.0;
    for (int t = TreeDepth; t < History.Size(); ++t)
    {
        QNODE_LAZY& qnode = vnode->Child(History[t].Action);
        //qnode.AMAF.Add(totalReward, totalDiscount);
        totalDiscount *= Params.RaveDiscount;
    }
}

VNODE_LAZY* MCTS_LAZY::ExpandNode(const STATE* state)
{
    VNODE_LAZY* vnode = VNODE_LAZY::Create();
    Simulator.set_belief_metainfo(vnode);
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

void MCTS_LAZY::AddSample(VNODE_LAZY* node, const STATE& state)
{
    STATE* sample = Simulator.Copy(state);
    node->Beliefs().AddSample(sample);
    if (Params.Verbose >= 2)
    {
        cout << "Adding sample:" << endl;
        Simulator.DisplayState(*sample, cout);
    }
}

int MCTS_LAZY::GreedyUCB(VNODE_LAZY* vnode, bool ucb) const
{
    static vector<int> besta;
    besta.clear();
    double bestq = -Infinity;
    int N = vnode->Value.GetCount();
    double logN = log(N + 1);
    bool hasalpha = Simulator.HasAlpha();

    // PRE-SHIELDING
    if (Params.use_shield && TreeDepth == 0) {
        for (int action : legal_actions)
        {
            double q, alphaq;
            int n, alphan;

            QNODE_LAZY& qnode = vnode->Child(action);
            q = qnode.Value.GetValue();
            n = qnode.Value.GetCount();

            /*
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
            */

            if (ucb)
                q += FastUCB(N, n, logN);

            if (q >= bestq)
            {
                if (q > bestq)
                    besta.clear();
                bestq = q;
                besta.push_back(action);
            }
        }
    }
    else {
        for (int action = 0; action < Simulator.GetNumActions(); action++)
        {
            double q, alphaq;
            int n, alphan;

            QNODE_LAZY& qnode = vnode->Child(action);
            q = qnode.Value.GetValue();
            n = qnode.Value.GetCount();

            /*
            if (Params.UseRave && qnode.AMAF.GetCount() > 0)
            {
                double n2 = qnode.AMAF.GetCount();
                double beta = n2 / (n + n2 + Params.RaveConstant * n * n2);
                q = (1.0 - beta) * q + beta * qnode.AMAF.GetValue();
            }
            */

            /*
            if (hasalpha && n > 0)
            {
                Simulator.AlphaValue(qnode, alphaq, alphan);
                q = (n * q + alphan * alphaq) / (n + alphan);
            }
            */

            if (ucb)
                q += FastUCB(N, n, logN);

            if (q >= bestq)
            {
                if (q > bestq)
                    besta.clear();
                bestq = q;
                besta.push_back(action);
            }
        }
    }

    assert(!besta.empty());
    return besta[Random(besta.size())];
}

double MCTS_LAZY::Rollout(STATE& state)
{
    Status.Phase = SIMULATOR_LAZY::STATUS::ROLLOUT;
    if (Params.Verbose >= 3)
        cout << "Starting rollout" << endl;

    double totalReward = 0.0;
    double discount = 1.0;
    bool terminal = false;
    int numSteps;
    for (numSteps = 0; numSteps + TreeDepth < Params.MaxDepth && !terminal; ++numSteps)
    {
        SIMULATOR_LAZY::observation_t observation;
        double reward;

        int action = Simulator.SelectRandom(state, History, Status);
        terminal = Simulator.Step(state, action, observation, reward);
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

void MCTS_LAZY::AddTransforms(VNODE_LAZY* root, BELIEF_STATE& beliefs)
{
    int attempts = 0, added = 0;

    // Local transformations of state that are consistent with history
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

STATE* MCTS_LAZY::CreateTransform() const
{
    SIMULATOR_LAZY::observation_t stepObs;
    double stepReward;

    STATE* state = Root->Beliefs().CreateSample(Simulator);
    Simulator.Step(*state, History.Back().Action, stepObs, stepReward);
    if (Simulator.LocalMove(*state, History, stepObs, Status))
        return state;
    Simulator.FreeState(state);
    return 0;
}

double MCTS_LAZY::UCB[UCB_N][UCB_n];
bool MCTS_LAZY::InitialisedFastUCB = true;

void MCTS_LAZY::InitFastUCB(double exploration)
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

inline double MCTS_LAZY::FastUCB(int N, int n, double logN) const
{
    if (InitialisedFastUCB && N < UCB_N && n < UCB_n)
        return UCB[N][n];

    if (n == 0)
        return Infinity;
    else
        return Params.ExplorationConstant * sqrt(logN / n);
}

void MCTS_LAZY::ClearStatistics()
{
    StatTreeDepth.Clear();
    StatRolloutDepth.Clear();
    StatTotalReward.Clear();
}

void MCTS_LAZY::DisplayStatistics(ostream& ostr) const
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

void MCTS_LAZY::DisplayValue(int depth, ostream& ostr) const
{
    HISTORY history;
    ostr << "MCTS_LAZY Values:" << endl;
    Root->DisplayValue(history, depth, ostr);
}

void MCTS_LAZY::DisplayPolicy(int depth, ostream& ostr) const
{
    HISTORY history;
    ostr << "MCTS_LAZY Policy:" << endl;
    Root->DisplayPolicy(history, depth, ostr);
}

