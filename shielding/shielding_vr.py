import sys, csv, os
import math, random
import z3
import xml.etree.ElementTree as ET

#######
# XES #
#######
xes_ns = { 'xes': 'rttp://www.w3.org/2001/XMLSchema' }

def node_from_key(root, key):
    for atr in root:
        if 'key' in atr.attrib and atr.attrib['key'] == key:
            return atr
    return None

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
    def __init__(self, xes_log, threshold, rules):
        self.xes_log = xes_log
        self.xes_tree = ET.parse(xes_log)

        self.threshold = threshold
        self.rules = rules

        self.segments_in_runs = []
        self.actions_in_runs = []
        self.belief_in_runs = []
        self.run_folders = []
        self.build_from_xes()

        self.solver = z3.Optimize()
        self.thresholds  = [[] for i in range(len(rules))]
        self.soft_constr = [[] for i in range(len(rules))]

    def build_from_xes(self):
        """
        Parse xes log and build data from traces
        """
        log = self.xes_tree.getroot()

        for trace in log.findall('xes:trace', xes_ns):

            self.run_folders.append('run {}'.format(
                int(node_from_key(trace, 'run').attrib['value'])))

            # each xes trace is a POMCP's run
            self.segments_in_runs.append([])
            self.actions_in_runs.append([])
            self.belief_in_runs.append([])

            for event in trace.findall('xes:event', xes_ns):
                # attributes
                segment = int(node_from_key(event,'segment').attrib['value'])
                self.segments_in_runs[-1].append(segment)

                action = int(node_from_key(event,'action').attrib['value'])
                self.actions_in_runs[-1].append(action)

                # belief
                self.belief_in_runs[-1].append({0:0, 1:0, 2:0})
                for i in node_from_key(event, 'belief'):
                    state = i.attrib['key']
                    particles = int(i.attrib['value'])
                    local_difficulty = (int(state) // (3 ** (7 - segment))) % 3
                    self.belief_in_runs[-1][-1][local_difficulty] += particles

                total = self.belief_in_runs[-1][-1][0] + self.belief_in_runs[-1][-1][1] + self.belief_in_runs[-1][-1][2]
                self.belief_in_runs[-1][-1][0] /= total
                self.belief_in_runs[-1][-1][1] /= total
                self.belief_in_runs[-1][-1][2] /= total

    def find_max_satisfiable_rule(self, rule_num):
        """
        Build a model that satisfies as many soft clauses as possible using MAX-SMT
        """
        rule = self.rules[rule_num]

        # enforce probability axioms
        for c in range(len(rule.constraints)):
            self.thresholds[rule_num].append([None, None, None])
            for s in range(3):
                t = z3.Real('t_r{}_c{}_state{}'.format(rule_num, c, s))
                self.thresholds[rule_num][c][s] = t
                self.solver.add(0.0 < t)
                self.solver.add(t <= 1.0)
            prob_sum = z3.Sum(self.thresholds[rule_num][c])
            self.solver.add(prob_sum == 1.0)
        
        if rule_num == 0: 
            self.solver.add(self.thresholds[0][0][0] >= 0.90)

        for run in range(len(self.belief_in_runs)):
            t = self.thresholds[rule_num]
            for bel, belief in enumerate(self.belief_in_runs[run]):
                soft = z3.Bool('b_{}_{}_{}'.format(rule_num, run, bel))
                self.soft_constr[rule_num].append(DummyVar(soft, rule_num, run, bel))

                subrules = []
                for c in range(len(rule.constraints)):
                    subrule = []
                    for i in rule.constraints[c].greater_equal:
                        subrule.append(belief[i] >= t[c][i])
                    for i in rule.constraints[c].lower_equal:
                        subrule.append(belief[i] <= t[c][i])
                    subrules.append(z3.And(subrule))

                formula = z3.Or(subrules)

                
                if self.actions_in_runs[run][bel] not in rule.speeds:
                    formula = z3.Not(formula) 

                self.solver.add(z3.Or(soft, formula))
                

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

        for soft in self.soft_constr[rule_num]:
            if model[soft.literal] == True:
                self.solver.add(soft.literal)
            elif model[soft.literal] == False:  
                self.solver.add(z3.Not(soft.literal))

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

        result = self.solver.check()

        m = self.solver.model()

        if result != z3.sat:
            print("unsatisiable")
            return

        print('{} {} {} {}'.format(
            to_real(m[self.thresholds[0][0][0]]),
            to_real(m[self.thresholds[0][1][2]]),
            to_real(m[self.thresholds[0][2][0]]),
            to_real(m[self.thresholds[0][2][1]])))
        print('0.1')

        self.solver.pop()

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

        useful_point = set()
        test_counter = 0
        while test_counter < 1000:
            point = [ 0.0, 0.0, 0.0 ]
            point[0] = random.uniform(0.0, 1.0)
            point[1] = random.uniform(0.0, 1.0 - point[0])
            point[2] = 1.0 - point[0] - point[1]

            fails_all_constraints = True
            for i, constraint in enumerate(self.rules[rule_num].constraints):
                is_ok = True
                for c in constraint.lower_equal:
                    threshold = to_real(m[self.thresholds[rule_num][i][c]])
                    if point[c] <= threshold:
                        is_ok = False
                        break
                if not is_ok:
                    fails_all_constraints = False
                    break

                for c in constraint.greater_equal:
                    threshold = to_real(m[self.thresholds[rule_num][i][c]])
                    if point[c] >= threshold:
                        is_ok = False
                        break
                if not is_ok:
                    fails_all_constraints = False
                    break

            if fails_all_constraints:
                test_counter += 1
                hel_dst = [[Hellinger_distance(point, Q),
                    (Q[0], Q[1], Q[2])] for Q in rule_points]
                useful_point.add(min(hel_dst)[1])

        for p in useful_point:
            print('{} {} {}'.format(p[0], p[1], p[2]), end=' ')
        print()

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
        print ('usage: shielding_vr <log.xes>')
        exit()

    xes_log = str(sys.argv[1])

    rs = RuleSynth(
            xes_log=xes_log,
            threshold=0.1,
            rules=[
                SpeedRule(
                    speeds=[2],
                    constraints = [
                        SpeedRuleConstraints(greater_equal=[0],    lower_equal=[]),
                        SpeedRuleConstraints(greater_equal=[],     lower_equal=[2]),
                        SpeedRuleConstraints(greater_equal=[0, 1],  lower_equal=[])
                        ]
                    ),
                ]
            )
    rs.synthetize_rules()

