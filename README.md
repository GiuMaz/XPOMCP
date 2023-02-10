Explainable POMCP
-----------------

XPOMCP is an explainability methodology for POMCP-generated policies.
It builds a compact representation of a policy using two elements:
 - a high level description provided by a human expert (using logical formulas)
 - the analysis of execution traces generated with a POMCP agent.

The result can be used to identify unexpected decisions (see folder unexpected\_decisions) to shield the execution of a POMCP agent (see folder shielding).

The XPOMCP methodology was first presented in "Policy Interpretation for Partially Observable Monte-Carlo Planning: a Rule-based Approach" by Giulio Mazzi, Alberto Castellini, and Alessandro Farinelli.
The shielding mechanism was presented in "Rule-based Shielding for Partially Observable Monte-Carlo Planning" by Giulio Mazzi, Alberto Castellini, and Alessandro Farinelli.

Our POMCP implementation is based on the original code from David Silver and Joel Veness. XPOMCP is implemented in python and uses the z3 SMT-solver.
