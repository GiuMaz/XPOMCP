import sys, csv, os
import math, random
import z3
import xml.etree.ElementTree as ET

random.seed(123)

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

#########
# TIGER #
#########

class RuleSynth:
    """
    Synthetize rules from runs of an POMCP algorithm
    """
    def __init__(self, xes_log, threshold):
        self.xes_log = xes_log
        self.xes_tree = ET.parse(xes_log)

        self.threshold = threshold
        self.actions_in_runs = []
        self.belief_in_runs = []
        self.run_folders = []
        self.build_from_xes()

        self.solver = z3.Optimize()
        self.threshold_open = None
        self.threshold_listen = None
        self.soft_constr_open = []
        self.soft_constr_listen = []

    def build_from_xes(self):
        """
        Parse xes log and build data from traces
        """
        log = self.xes_tree.getroot()

        for trace in log.findall('xes:trace', xes_ns):
            self.run_folders.append('run {}'.format(
                int(node_from_key(trace, 'run').attrib['value'])))

            self.actions_in_runs.append([])
            self.belief_in_runs.append([])

            for event in trace.findall('xes:event', xes_ns):
                action = 0
                if node_from_key(event,'action').attrib['value'] == 'listen':
                    action = 0
                elif node_from_key(event,'action').attrib['value'] == 'open left':
                    action = 1
                else:
                    action = 2
                self.actions_in_runs[-1].append(action)

                belief = node_from_key(event, 'belief')
                left  = int(node_from_key(belief, 'tiger left').attrib['value'])
                right = int(node_from_key(belief, 'tiger right').attrib['value'])
                tot = left + right
                self.belief_in_runs[-1].append({0: (left / tot), 1: (right / tot)})

    def find_max_satisfiable_rule(self):
        """
        Build a model that satisfies as many soft clauses as possible using MAX-SMT
        """
        self.threshold_open = z3.Real('t')
        self.threshold_listen = z3.Real('u')
        t = self.threshold_open
        u = self.threshold_listen

        self.solver.add(0.0 < t)
        self.solver.add(t <= 1.0)
        self.solver.add(0.0 < u)
        self.solver.add(u <= 1.0)

        for run in range(len(self.belief_in_runs)):
            for bel, belief in enumerate(self.belief_in_runs[run]):
                soft = z3.Bool('b_{}_{}'.format(run, bel))
                self.soft_constr_open.append(DummyVar(soft, run, bel, 1))

                formula = z3.If(belief[0] > belief[1], belief[0] >= t, belief[1] >= t)
                if self.actions_in_runs[run][bel] != 0 :
                    self.solver.add(z3.Or(soft, formula))
                else:
                    self.solver.add(z3.Or(soft, z3.Not(formula)))

                soft = z3.Bool('c_{}_{}'.format(run, bel))
                self.soft_constr_listen.append(DummyVar(soft, run, bel, 2))
                formula = z3.If(belief[0] > belief[1], belief[0] <= u, belief[1] <= u)
                if self.actions_in_runs[run][bel] == 0 :
                    self.solver.add(z3.Or(soft, formula))
                else:
                    self.solver.add(z3.Or(soft, z3.Not(formula)))

        self.solver.add(self.threshold_open > 0.9)

        low_threshold = 0
        total_soft_constr = len(self.soft_constr_open) + len(self.soft_constr_listen)
        high_threshold = len(self.soft_constr_open) + len(self.soft_constr_listen)
        final_threshold = -1
        best_model = []
        while low_threshold <= high_threshold:
            self.solver.push()

            threshold = (low_threshold + high_threshold) // 2
            self.solver.add(z3.PbLe([(soft.literal, 1) for soft in (self.soft_constr_open+self.soft_constr_listen)], threshold))
            result = self.solver.check()
            if result == z3.sat:
                final_threshold = threshold
                best_model = self.solver.model()
                high_threshold = threshold - 1
            else:
                low_threshold = threshold + 1
            self.solver.pop()

        return best_model

    def synthetize_rule(self, model):
        self.solver.push()

        for soft in self.soft_constr_open:
            if model[soft.literal] == True:
                self.solver.add(soft.literal)
            elif model[soft.literal] == False:  
                self.solver.add(z3.Not(soft.literal))

        for soft in self.soft_constr_listen:
            if model[soft.literal] == True:
                self.solver.add(soft.literal)
            elif model[soft.literal] == False:  
                self.solver.add(z3.Not(soft.literal))

        self.solver.maximize(self.threshold_open - self.threshold_listen)

        result = self.solver.check()

        m = self.solver.model()
        self.solver.pop()

        if result != z3.sat:
            return

        print('{} {}'.format(to_real(model[self.threshold_open]), to_real(model[self.threshold_listen])))
        print('0.1')
        open_points = []
        threshold = to_real(m[self.threshold_open])
        while len(open_points) < 1000:
            point = [ 0.0, 0.0 ]
            point[0] = random.uniform(0.0, 1.0)
            point[1] = 1.0 - point[0]
            if point[0] >= threshold or point[1] >= threshold:
                open_points.append(point)

        useful_point = set()
        test_counter = 0
        while test_counter < 1000:
            point = [ 0.0, 0.0 ]
            point[0] = random.uniform(0.0, 1.0)
            point[1] = 1.0 - point[0]
            if point[0] < threshold and point[1] < threshold:
                hel_dst = [[Hellinger_distance(point, Q), (Q[0], Q[1])] for Q in open_points]
                useful_point.add(min(hel_dst)[1])
                test_counter+=1

        for p in useful_point:
            print('{} {} '.format(p[0], p[1]), end='')
        print()

        listen_points = []
        threshold = to_real(m[self.threshold_listen])
        while len(listen_points) < 1000:
            point = [ 0.0, 0.0 ]
            point[0] = random.uniform(0.00, 1.00)
            point[1] = 1.0 - point[0]
            if point[0] <= threshold and point[1] <= threshold:
                listen_points.append(point)

        useful_point = set()
        test_counter = 0
        while test_counter < 1000:
            point = [ 0.0, 0.0 ]
            point[0] = random.uniform(0.0, 1.0)
            point[1] = 1.0 - point[0]
            if point[0] > threshold or point[1] > threshold:
                hel_dst = [[Hellinger_distance(point, Q), (Q[0], Q[1])] for Q in listen_points]
                useful_point.add(min(hel_dst)[1])
                test_counter+=1
        for p in useful_point:
            print('{} {} '.format(p[0], p[1]), end='')
        print()

    def synthetize_rules(self):
        self.solver.push()
        model = self.find_max_satisfiable_rule()
        self.synthetize_rule(model)
        self.solver.pop()


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print ('usage: shielding_tiger <log.xes>')
        exit()

    xes_log = str(sys.argv[1])

    rs = RuleSynth(
            xes_log=xes_log,
            threshold=0.1
            )
    rs.synthetize_rules()

