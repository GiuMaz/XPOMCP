# Copyright 2021 Giulio Mazzi
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
# 
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import sys, csv, os
import math, random
import z3

#########
# UTILS #
#########

def Hellinger_distance(P, Q):
    """
    Hellinger_distance between two probability distribution.
    """
    dist = 0.0
    for p, q in zip(P, Q):
        dist += (math.sqrt(p) - math.sqrt(q)) ** 2

    dist = math.sqrt(dist)
    dist /= math.sqrt(2)

    return dist

def to_real(x):
    """
    Convert Z3 Fractional numbers into floating points
    """
    return float(x.numerator_as_long()/x.denominator_as_long())


#############################
# VELOCITY REGULATION RULES #
#############################

class DummyVar:
    """
    Class that represent a dummy variable introduced by the MAX-SMT step.
    It contains the literal (a Boolean variable) that identify the dummy
    variable inside the SMT problem and the the information related to wich
    rule, run and step is codified by the variable.
    """
    def __init__(self, literal, rule, run, step):
        self.literal = literal
        self.rule = rule
        self.run = run
        self.step = step

class SpeedRule:
    """
    Class that represent a rule based on speed: given a set of constraints, the
    robot must go to a certain speed (or to one of the speeds, if multiple speed
    are specified) when the constraints hold, and to a different speed
    otherwhise
    """
    def __init__(self, speeds, constraints):
        self.speeds = speeds
        self.constraints = constraints

class SpeedRuleConstraints:
    """
    Constraint for a speed rule. It takes two (possibly empty) lists:
      - the first one specify a lower bound on certain states
      - the second one specify an upper bound on certain states

    for example:
    SpeedRuleConstraints([0],[2]) specify that
      - the probability of state 0 must be >= than a threshold 
      - the probability of state 2 must be <= than another threshold
    SpeedRuleConstraints([0,1],[]) specify that:
      - the probability of state 0 must be >= than a threshold
      - the probability of state 1 must be >= than another threshold
    """
    def __init__(self, greater_equal, lower_equal):
        self.greater_equal = greater_equal
        self.lower_equal = lower_equal

class RuleSynth:
    """
    Synthetize rules from runs of an POMCP algorithm
    """
    def __init__(self, folder, threshold, rules):
        self.folder = folder
        self.threshold = threshold
        self.rules = rules

        self.segments_in_runs = []
        self.actions_in_runs = []
        self.belief_in_runs = []
        self.run_folders = []
        self.parse_folder(self.folder)

        self.solver = z3.Optimize()
        self.thresholds  = [[] for i in range(len(rules))]
        self.soft_constr = [[] for i in range(len(rules))]

    def parse_folder(self, folder):
        """
        Parse folder and build information from runs
        """
        print('Import experiments')
        for subdir, dirs, files in os.walk(runs_folder):
            dirs.sort()
            if sorted(files) != ['beliefsPerStep.csv', 'policyPerStep.txt', 'stateEvolution.csv']:
                # not a run, skip
                continue

            self.run_folders.append(subdir)
            with open(os.path.join(subdir, 'stateEvolution.csv')) as csv_file:
                csv_reader = csv.reader(csv_file, delimiter=',')
                for i, row in enumerate(csv_reader):
                    if i == 1:
                        self.segments_in_runs.append([int(x) for x in row[1:]])
                    if i == 8:
                        self.actions_in_runs.append([int(x) for x in row[1:]])

            # collect the belief at each step
            belief_map = []
            with open(os.path.join(subdir, 'beliefsPerStep.csv')) as bps:
                csv_reader = csv.reader(bps, delimiter=',')
                for row in csv_reader:
                    belief_map.append({});
                    for entry in row:
                        if entry and not entry.isspace():
                            state, particles = entry.split(':')
                            belief_map[-1][int(state)] = int(particles)

            # compute the local belief (diff function)
            self.belief_in_runs.append([])
            for num, step in enumerate(belief_map):
                self.belief_in_runs[-1].append({0:0, 1:0, 2:0})
                for belief, particles in step.items():
                    self.belief_in_runs[-1][-1][(belief//(3**(8-self.segments_in_runs[-1][num]-1)))%3] += particles
                total = self.belief_in_runs[-1][-1][0] + self.belief_in_runs[-1][-1][1] + self.belief_in_runs[-1][-1][2]
                self.belief_in_runs[-1][-1][0] /= total
                self.belief_in_runs[-1][-1][1] /= total
                self.belief_in_runs[-1][-1][2] /= total

    def find_max_satisfiable_rule(self, rule_num):
        """
        Build a model that satisfies as many soft clauses as possible using MAX-SMT
        """
        print('Find maximum number of satisfiable step in rule {}'.format(rule_num))
        rule = self.rules[rule_num]

        # enforce probability axioms
        for c in range(len(rule.constraints)): # constraint in rule
            self.thresholds[rule_num].append([None, None, None])
            for s in range(3): # state in constraint
                t = z3.Real('t_r{}_c{}_state{}'.format(rule_num, c, s))
                self.thresholds[rule_num][c][s] = t
                # each threshold is a probability and must have a value
                # bethween 0 and 1
                self.solver.add(0.0 < t)
                self.solver.add(t <= 1.0)
            # the sum of the probability on the three states must be 1
            prob_sum = z3.Sum(self.thresholds[rule_num][c])
            self.solver.add(prob_sum == 1.0)

        # hard constraint, they must be be specified by hand in this version
        # e.g: x_1 >= 0.9
        self.solver.add(self.thresholds[rule_num][0][0] >= 0.9)

        # build soft clauses
        for run in range(len(self.belief_in_runs)):
            t = self.thresholds[rule_num]
            for bel, belief in enumerate(self.belief_in_runs[run]):
                # generate boolean var for soft constraints
                soft = z3.Bool('b_{}_{}_{}'.format(rule_num, run, bel))
                self.soft_constr[rule_num].append(DummyVar(soft, rule_num, run, bel))

                # add the rule
                subrules = []
                for c in range(len(rule.constraints)):
                    subrule = []
                    for i in rule.constraints[c].greater_equal:
                        subrule.append(belief[i] >= t[c][i])
                    for i in rule.constraints[c].lower_equal:
                        subrule.append(belief[i] <= t[c][i])
                    subrules.append(z3.And(subrule))

                formula = z3.Or(subrules)

                if self.actions_in_runs[run][bel] in rule.speeds:
                    self.solver.add(z3.Or(soft, formula))
                else:
                    self.solver.add(z3.Or(soft, z3.Not(formula)))

        # solve MAX-SMT problem
        low_threshold = 0
        total_soft_constr = len(self.soft_constr[rule_num])
        high_threshold = len(self.soft_constr[rule_num])
        final_threshold = -1
        best_model = []
        while low_threshold <= high_threshold:
            self.solver.push()

            threshold = (low_threshold + high_threshold) // 2

            self.solver.add(z3.PbLe([(soft.literal, 1) for soft in self.soft_constr[rule_num]], threshold))
            result = self.solver.check()
            if result == z3.sat:
                final_threshold = threshold
                best_model = self.solver.model()
                high_threshold = threshold - 1
            else:
                low_threshold = threshold + 1
            self.solver.pop()

        print('fail to satisfy {} steps out of {}'.format(final_threshold, total_soft_constr))
        # return a model that satisfy all the hard clauses and the maximum number of soft clauses
        return best_model

    def print_rule_result(self, rule_num, model):
        """
        pretty printing of rules, give a certain model
        """
        rule = self.rules[rule_num]
        print('rule: go at speed {} if: '.format(rule.speeds[0] if len(rule.speeds) == 1 else rule.speeds), end = '')
        for i, constraint in enumerate(rule.constraints):
            if i > 0:
                print('OR ', end='')

            if len(constraint.greater_equal) + len(constraint.lower_equal) == 1:
                for c in constraint.greater_equal:
                    print('P_{} >= {:.3f} '.format(c, to_real(model[self.thresholds[rule_num][i][c]])), end='')
                for c in constraint.lower_equal:
                    print('P_{} <= {:.3f} '.format(c, to_real(model[self.thresholds[rule_num][i][c]])), end='')
            elif len(constraint.greater_equal) != 0:
                print('(P_{} >= {:.3f}'.format(constraint.greater_equal[0], to_real(model[self.thresholds[rule_num][i][0]])), end='')
                for c in constraint.greater_equal[1:]:
                    print(' AND P_{} >= {:.3f}'.format(c, to_real(model[self.thresholds[rule_num][i][c]])), end='')
                for c in constraint.lower_equal:
                    print(' AND P_{} <= {:.3f}'.format(c, to_real(model[self.thresholds[rule_num][i][c]])), end='')
                print(') ',end='')
            else:
                print('(P_{} <= {:.3f} '.format(constraint.lower_equal[0], to_real(model[self.thresholds[rule_num][i][0]])), end='')
                for c in constraint.lower_equal[1:]:
                    print(' AND P_{} <= {:.3f}'.format(c, to_real(model[self.thresholds[rule_num][i][c]])), end='')
                print(') ',end='')
        print()

    def synthetize_rule(self, rule_num, model):
        """
        Synthetize a rule as close as possible to the trace.
        Print all the unstatisfiable steps and highlight anomalies.
        """
        self.solver.push()

        # fix dummy variables
        for soft in self.soft_constr[rule_num]:
            if model[soft.literal] == True:
                self.solver.add(soft.literal)
            elif model[soft.literal] == False:  
                self.solver.add(z3.Not(soft.literal))

        # try to optimize intervals
        interval_cost = z3.Real('interval_cost')
        cost = []
        for j, const in enumerate(self.rules[rule_num].constraints):
            for k in const.greater_equal:
                cost.append(self.thresholds[rule_num][j][k])
            for k in const.lower_equal:
                cost.append(-self.thresholds[rule_num][j][k])
        total_cost = z3.Sum(cost)
        self.solver.add(interval_cost == total_cost)
        self.solver.minimize(interval_cost)

        # check if SAT or UNSAT
        print('Check Formulas')
        result = self.solver.check()

        m = self.solver.model()
        # remove intervall optimization requirements
        self.solver.pop()

        # exit if unsat
        if result != z3.sat:
            print("no rule for speed 2")
            return

        # print results
        self.print_rule_result(rule_num, m)

        # generate 1000 random points inside the rule
        rule_points = []
        generated_points = 0
        while generated_points < 1000:
            point = [ 0.0, 0.0, 0.0 ]
            point[0] = random.uniform(0.0, 1.0)
            point[1] = random.uniform(0.0, 1.0 - point[0])
            point[2] = 1.0 - point[0] - point[1]

            satisfy_a_constraint = False
            for i, constraint in enumerate(self.rules[rule_num].constraints):
                is_ok = True
                for c in constraint.lower_equal:
                    threshold = to_real(m[self.thresholds[rule_num][i][c]])
                    if point[c] > threshold:
                        is_ok = False
                        break
                if not is_ok:
                    continue

                for c in constraint.greater_equal:
                    threshold = to_real(m[self.thresholds[rule_num][i][c]])
                    if point[c] < threshold:
                        is_ok = False
                        break
                if not is_ok:
                    continue

                satisfy_a_constraint = True
                break

            if satisfy_a_constraint:
                rule_points.append(point)
                generated_points += 1

        # Hellinger distance of unsatisfiable steps
        failed_rules = []
        Hellinger_min = []
        for num, soft in enumerate(self.soft_constr[rule_num]):
            if m[soft.literal] == False or not (self.actions_in_runs[soft.run][soft.step] in self.rules[rule_num].speeds) :
                continue
            failed_rules.append(num)
            P = [ self.belief_in_runs[soft.run][soft.step][0], self.belief_in_runs[soft.run][soft.step][1], self.belief_in_runs[soft.run][soft.step][2] ]
            hel_dst = [Hellinger_distance(P, Q) for Q in rule_points]
            Hellinger_min.append(min(hel_dst))

        # print unsatisfiable steps in decreasing order of hellinger distance
        print('Unsatisfiable steps:')
        anomaly_positions = []
        for soft, hel, pos in [[self.soft_constr[rule_num][x], h, self.soft_constr[rule_num][x].run*35 + self.soft_constr[rule_num][x].step] for h, x in sorted(zip(Hellinger_min, failed_rules), key=lambda pair: pair[0], reverse = True)]:
            if hel > self.threshold:
                print('ANOMALY: ', end='')
            print('run {} step {}: action {} with belief P_0 = {:.3f} P_1 = {:.3f} P_2 = {:.3f} --- Hellinger = {}'.format(
                self.run_folders[soft.run], soft.step, self.actions_in_runs[soft.run][soft.step], self.belief_in_runs[soft.run][soft.step][0],
                self.belief_in_runs[soft.run][soft.step][1], self.belief_in_runs[soft.run][soft.step][2], hel))
            if hel > self.threshold:
                anomaly_positions.append(pos)

    def synthetize_rules(self):
        """
        synthetize each rule
        """
        for rule in range(len(self.rules)):
            self.solver.push()
            model = self.find_max_satisfiable_rule(rule)
            self.synthetize_rule(rule, model)
            self.solver.pop()


########
# MAIN #
########

if __name__ == "__main__":
    # parse input files
    if len(sys.argv) != 2:
        print ('usage: rule_synthesis <runs_folder>')
        exit()

    runs_folder = str(sys.argv[1])

    rs = RuleSynth(
            folder=runs_folder,
            threshold=0.10,
            rules=[
                SpeedRule(
                    speeds=[2],
                    constraints = [
                        SpeedRuleConstraints(greater_equal=[0],    lower_equal=[]),
                        SpeedRuleConstraints(greater_equal=[],     lower_equal=[2]),
                        SpeedRuleConstraints(greater_equal=[0, 1], lower_equal=[]),
                        ]
                    )
                ]
            )
    rs.synthetize_rules()

