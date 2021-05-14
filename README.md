Explainable POMCP
-----------------

XPOMCP is an explainability methodology for POMCP generated policies.
It build a compact representation of a policy using two elements:
 - an high level description provided by an human expert (using logical formulas)
 - the analysis of execution traces generated with a POMCP agent.

The result can be used to identify unexpected decisions (see folder unexpected\_decisions) of to shield the execution of a POMCP agent (see folder shielding).

The XPOMCP methodology was presented in "Policy Interpretation for Partially Observable Monte-Carlo Planning: a Rule-based Approach" by Giulio Mazzi, Alberto Castellini and Alessandro Farinelli.
The shielding mechanism was presented in "Rule-based Shielding for Partially Observable Monte-Carlo Planning" by Giulio Mazzi, Alberto Castellini and Alessandro Farinelli.
