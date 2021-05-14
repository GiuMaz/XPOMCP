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

def value_from_key(root, key):
    for atr in root:
        if 'key' in atr.attrib and atr.attrib['key'] == key:
            return atr.attrib['value']
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

def manhattan_distance(x1, y1, x2, y2):
    return abs(x1 - x2) + abs(y1 - y2)

def euclidean_distance(x1, y1, x2, y2):
    return math.sqrt((x1 - x2)**2 + (y1 - y2)**2)

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

class Rock:
    def __init__(self, x, y, num):
        self.x = x
        self.y = y
        self.num = num

class RuleSynth:
    """
    Synthetize rules from runs of an POMCP algorithm
    """
    def __init__(self, xes_log, threshold):
        self.xes_log = xes_log
        self.xes_tree = ET.parse(xes_log)

        self.sample_confidence = 1.0

        self.threshold = threshold
        self.rules = 3

        self.rocknum = -1
        self.size = -1
        self.rocks = []

        self.beliefs = []
        self.actions = []
        self.positions = []
        self.runs = []
        self.steps = []

        self.collected = []

        self.build_from_xes()

        self.solver = z3.Optimize()
        self.thresholds  = [[] for i in range(self.rules)]
        self.soft_constr = [[] for i in range(self.rules)]

    def is_rock_pos(self, x, y):
        for r in self.rocks:
            if r.x == x and r.y == y:
                return True
        return False

    def rock_number(self, x, y):
        """
        return the number of the rock in position (x, y), or -1 if there is
        no rock in (x, y)
        """
        for r in self.rocks:
            if r.x == x and r.y == y:
                return r.num
        return -1

    def build_from_xes(self):
        """
        Parse xes log and build data from traces
        """
        log = self.xes_tree.getroot()
        self.size = int(value_from_key(log,'Size'))
        self.rocknum = int(value_from_key(log,'NumRocks'))
        for rock in node_from_key(log, 'rocks'):
            x = int(value_from_key(rock, 'coord x'))
            y = int(value_from_key(rock, 'coord y'))
            num = int(value_from_key(rock, 'number'))
            self.rocks.append(Rock(x=x, y=y, num=num))

        for trace in log.findall('xes:trace', xes_ns):
            run = int(value_from_key(trace, 'run'))
            step = 1
            for event in trace.findall('xes:event', xes_ns):
                if step == 1:
                    self.collected.append([0 for i in range(0, self.rocknum)])
                else:
                    collected = [i for i in self.collected[-1]]
                    if self.actions[-1] == 'sample':
                        pos = self.positions[-1]
                        num = self.rock_number(pos[0], pos[1])
                        if num != -1:
                            collected[num] = 1
                    self.collected.append(collected)

                self.runs.append(run)
                self.steps.append(step)
                step +=1

                x = int(value_from_key(event,'coord x'))
                y = int(value_from_key(event,'coord y'))
                self.positions.append([x, y])

                action = value_from_key(event,'action')
                self.actions.append(action)

                # belief
                self.beliefs.append({})
                total = {}
                for i in range(0, self.rocknum):
                    self.beliefs[-1][i] = 0
                    total[i] = 0

                for i in node_from_key(event, 'belief'):
                    state = int(i.attrib['key'])
                    particles = int(i.attrib['value'])
                    for j in range(0, self.rocknum):
                        total[j] += particles
                        if (state // (2**j)) % 2 == 1:
                            self.beliefs[-1][j] += particles

                for j in range(0, self.rocknum):
                    self.beliefs[-1][j] /= total[j]

    def build_sample_rule(self):
        """
        Build a rule for sampling
        """
        # enforce probability axioms
        t = z3.Real('t_sample')
        self.thresholds[0].append(t)
        self.solver.add(0.0 <= t)
        self.solver.add(t <= 1.0)

        # hard constraint, they must be be specified by hand in this version
        # e.g: x_1 >= 0.9
        #self.solver.add(t > 0.6)

        # build soft clauses
        for i in range(0, len(self.beliefs)):
            bel = self.beliefs[i]
            act = self.actions[i]
            pos = self.positions[i]
            run = self.runs[i]
            step = self.steps[i]
            collected = self.collected[i]

            # generate boolean var for soft constraints 
            soft = z3.Bool('b_sample_{}'.format(i))
            self.soft_constr[0].append(DummyVar(soft, 0, run, step))

            # add the rule
            subrules = []
            for r in self.rocks:
                sub = z3.And(
                        pos[0] == r.x,
                        pos[1] == r.y,
                        collected[r.num] == 0,
                        bel[r.num] >= t
                        )
                subrules.append(sub)

            formula = z3.Or(subrules)

            if act != 'sample':
                formula = z3.Not(formula) 

            self.solver.add(z3.Or(soft, formula))

        # solve MAX-SMT problem
        low_threshold = 0
        high_threshold = len(self.soft_constr[0])
        final_threshold = -1
        best_model = []

        while low_threshold <= high_threshold:
            self.solver.push()

            threshold = (low_threshold + high_threshold) // 2
            #Pble pseudo boolean less equal 
            self.solver.add(z3.PbLe([(soft.literal, 1)
                for soft in self.soft_constr[0]], threshold))
            result = self.solver.check()
            if result == z3.sat:
                final_threshold = threshold
                best_model = self.solver.model()
                high_threshold = threshold - 1
            else:
                low_threshold = threshold + 1
            self.solver.pop()

        # build tight bounds
        model = best_model

        # fix dummy variables
        for soft in self.soft_constr[0]:
            if model[soft.literal] == True:
                self.solver.add(soft.literal)
            elif model[soft.literal] == False:  
                self.solver.add(z3.Not(soft.literal))
        self.solver.maximize(t)

        # check if SAT or UNSAT
        result = self.solver.check()
        if result != z3.sat:
            print("unsatisfiable")
            return

        model = self.solver.model()

        # generate 1000 random points inside the rule
        rule_points = [[to_real(model[t])]]

        print('sample if in position and confidence >= {}'.format(to_real(model[t])))
        self.sample_confidence = to_real(model[t])
        print('fail to satisfy {} out of {} steps'.format(final_threshold, len(self.steps)))

        ## Hellinger distance of unsatisfiable steps
        failed_rules = []
        Hellinger_min = []
        for num, soft in enumerate(self.soft_constr[0]):
            if model[soft.literal] == False or self.actions[num] != 'sample' :
                continue
            failed_rules.append(num)
            pos = self.positions[num]
            rock = self.rock_number(pos[0], pos[1])
            P = [self.beliefs[num][rock]]
            hel_dst = [Hellinger_distance(P, Q) for Q in rule_points]
            Hellinger_min.append(min(hel_dst))

        # print unsatisfiable steps in decreasing order of hellinger distance
        print('Unsatisfiable steps:')
        for x, soft, hel in [[x, self.soft_constr[0][x], h] for h, x in sorted(zip(Hellinger_min, failed_rules), key=lambda pair: pair[0], reverse = True)]:
            if hel > self.threshold:
                print('ANOMALY: ', end='')
            pos = self.positions[x]
            rock = self.rock_number(pos[0], pos[1])
            print('run {} step {}: action {} with belief of valuable rock = {:.3f} --- Hellinger = {}'.format(
                soft.run, soft.step, self.actions[x],
                self.beliefs[x][rock],
                hel)
                )

        ## Hellinger distance of unsatisfiable steps
        for num, soft in enumerate(self.soft_constr[0]):
            if model[soft.literal] == False or self.actions[num] == 'sample' :
                continue
            pos = self.positions[num]
            rock = self.rock_number(pos[0], pos[1])
            print('run {} step {}: action {} with belief of valuable rock = {:.3f}'.format(soft.run, soft.step, self.actions[num], self.beliefs[num][rock]))

    def build_check_rule(self):
        """
        Build a rule for check 1..n
        """
        u1 = z3.Real('u1_sample')
        self.thresholds[1].append(u1)
        self.solver.add(0.0 <= u1)
        self.solver.add(u1 <= 1.0)
        v1 = z3.Real('v1_sample')
        self.thresholds[1].append(v1)
        self.solver.add(0.0 <= v1)
        self.solver.add(v1 <= 1.0)
        m1 = z3.Real('m1_sample')
        self.thresholds[1].append(m1)
        self.solver.add(m1 >= 0.0)
        n1 = z3.Real('n1_sample')
        self.thresholds[1].append(n1)
        self.solver.add(n1 >= 0.0)

        self.solver.add(v1 < u1)
        self.solver.add(n1 <= m1)

        #u2 = z3.Real('u2_sample')
        #self.thresholds[1].append(u2)
        #self.solver.add(0.0 <= u2)
        #self.solver.add(u2 <= 1.0)
        #v2 = z3.Real('v2_sample')
        #self.thresholds[1].append(v2)
        #self.solver.add(0.0 <= v2)
        #self.solver.add(v2 <= 1.0)
        #m2 = z3.Real('m2_sample')
        #self.thresholds[1].append(m2)
        #self.solver.add(m2 >= 0.0)
        #n2 = z3.Real('n2_sample')
        #self.thresholds[1].append(n2)
        #self.solver.add(n2 >= 0.0)

        #self.solver.add(v2 < u2)
        #self.solver.add(n2 + 1 <= m2)

        #self.solver.add(m1 <= n2)

        #self.solver.add(m2 <= 8)

        ## hard constraint, they must be be specified by hand in this version
        ## e.g: x_1 >= 0.9

        ## build soft clauses
        for i in range(0, len(self.beliefs)):
            bel = self.beliefs[i]
            act = self.actions[i]
            pos = self.positions[i]
            run = self.runs[i]
            step = self.steps[i]
            collected = self.collected[i]

            # generate boolean var for soft constraints 
            soft = z3.Bool('b_check_{}'.format(i))
            self.soft_constr[1].append(DummyVar(soft, 1, run, step))

            if act == 'north' or act == 'south' or act == 'east' or act == 'west' or act == 'sample':
                self.solver.add(z3.Or(soft, True))
                continue

            c = int(act.split()[1])
            r = self.rocks[c]
            # add the rule
            formula = z3.Or(
                    z3.And(
                        euclidean_distance(pos[0], pos[1], r.x, r.y) >= n1,
                        euclidean_distance(pos[0], pos[1], r.x, r.y) <= m1,
                        bel[c] <= u1,
                        bel[c] >= v1
                        ),
                    #z3.And(
                    #    euclidean_distance(pos[0], pos[1], r.x, r.y) >= n2,
                    #    euclidean_distance(pos[0], pos[1], r.x, r.y) <= m2,
                    #    bel[c] <= u2,
                    #    bel[c] >= v2
                    #    )
                    )

            self.solver.add(z3.Or(soft, formula))

        # solve MAX-SMT problem
        low_threshold = 0
        high_threshold = len(self.soft_constr[1])
        final_threshold = -1
        best_model = []

        while low_threshold <= high_threshold:
            self.solver.push()

            threshold = (low_threshold + high_threshold) // 2
            #Pble pseudo boolean less equal 
            self.solver.add(z3.PbLe([(soft.literal, 1)
                for soft in self.soft_constr[1]], threshold))
            result = self.solver.check()
            if result == z3.sat:
                final_threshold = threshold
                best_model = self.solver.model()
                high_threshold = threshold - 1
            else:
                low_threshold = threshold + 1
            self.solver.pop()

        ## build tight bounds
        model = best_model

        # fix dummy variables
        for soft in self.soft_constr[1]:
            if model[soft.literal] == True:
                self.solver.add(soft.literal)
            elif model[soft.literal] == False:  
                self.solver.add(z3.Not(soft.literal))

        self.solver.minimize(z3.Sum(u1, m1, -v1, -n1))
        #self.solver.minimize(z3.Sum(u1, m1, -v1, -n1, u2, m2, -v2, -n2))

        # check if SAT or UNSAT
        result = self.solver.check()
        if result != z3.sat:
            print("unsatisfiable")
            return

        model = self.solver.model()

        ## generate 1000 random points inside the rule
        #rule_points = [[to_real(model[t])]]

        print('check when: distance [{:.3f}, {:.3f}] and belief of valuable rock in [{:.3f}, {:.3f}]'.format(to_real(model[n1]), to_real(model[m1]), to_real(model[v1]), to_real(model[u1])))
        #print('check when: distance [{:.3f}, {:.3f}] and belief of valuable rock in [{:.3f}, {:.3f}] OR distance [{:.3f}, {:.3f}] and belief of valuable rock in [{:.3f}, {:.3f}]'.format(to_real(model[n1]), to_real(model[m1]), to_real(model[v1]), to_real(model[u1]), to_real(model[n2]), to_real(model[m2]), to_real(model[v2]), to_real(model[u2])))
        print('fail to satisfy {} out of {} steps'.format(final_threshold, len(self.steps)))

        ### Hellinger distance of unsatisfiable steps
        #failed_rules = []
        #Hellinger_min = []
        #for num, soft in enumerate(self.soft_constr[1]):
        #    if model[soft.literal] == False or self.actions[num] != 'sample' :
        #        continue
        #    failed_rules.append(num)
        #    pos = self.positions[num]
        #    rock = self.rock_number(pos[0], pos[1])
        #    P = [self.beliefs[num][rock]]
        #    hel_dst = [Hellinger_distance(P, Q) for Q in rule_points]
        #    Hellinger_min.append(min(hel_dst))

        ## print unsatisfiable steps in decreasing order of hellinger distance
        #print('Unsatisfiable steps:')
        #for x, soft, hel in [[x, self.soft_constr[1][x], h] for h, x in sorted(zip(Hellinger_min, failed_rules), key=lambda pair: pair[0], reverse = True)]:
        #    if hel > self.threshold:
        #        print('ANOMALY: ', end='')
        #    pos = self.positions[x]
        #    rock = self.rock_number(pos[0], pos[1])
        #    print('run {} step {}: action {} with belief of valuable rock = {:.3f} --- Hellinger = {}'.format(
        #        soft.run, soft.step, self.actions[x],
        #        self.beliefs[x][rock],
        #        hel)
        #        )

        ### Hellinger distance of unsatisfiable steps
        #for num, soft in enumerate(self.soft_constr[1]):
        #    if model[soft.literal] == False or self.actions[num] == 'sample' :
        #        continue
        #    pos = self.positions[num]
        #    rock = self.rock_number(pos[0], pos[1])
        #    print('run {} step {}: do not sample with belief of valuable rock = {:.3f}'.format(soft.run, soft.step, self.beliefs[num][rock]))

    def build_north_south_rule(self):
        """
        Build a rule for north or south
        """
        c = z3.Real('valuable_confidence')
        self.thresholds[2].append(c)
        self.solver.add(c >= 0.0)
        self.solver.add(c <= 1.0)
        self.solver.add(c >= self.sample_confidence)

        ## build soft clauses
        for i in range(0, len(self.beliefs)):
            bel = self.beliefs[i]
            act = self.actions[i]
            pos = self.positions[i]
            run = self.runs[i]
            step = self.steps[i]
            collected = self.collected[i]

            # generate boolean var for soft constraints 
            soft = z3.Bool('b_north_south_{}'.format(i))
            self.soft_constr[2].append(DummyVar(soft, 2, run, step))

            if act != 'north' and act != 'south':
                self.solver.add(z3.Or(soft, True))
                continue

            if act == 'north':
                subrules = []
                for r in self.rocks:
                    sub = z3.And(
                            pos[1] < r.y,
                            collected[r.num] == 0,
                            bel[r.num] >= c
                            )
                    subrules.append(sub)
                formula = z3.Or(subrules)
                self.solver.add(z3.Or(soft, formula))
            else:
                subrules = []
                for r in self.rocks:
                    sub = z3.And(
                            pos[1] > r.y,
                            collected[r.num] == 0,
                            bel[r.num] >= c
                            )
                    subrules.append(sub)
                formula = z3.Or(subrules)
                self.solver.add(z3.Or(soft, formula))

        # solve MAX-SMT problem
        low_threshold = 0
        high_threshold = len(self.soft_constr[2])
        final_threshold = -1
        best_model = []

        while low_threshold <= high_threshold:
            self.solver.push()

            threshold = (low_threshold + high_threshold) // 2
            #Pble pseudo boolean less equal 
            self.solver.add(z3.PbLe([(soft.literal, 1)
                for soft in self.soft_constr[1]], threshold))
            result = self.solver.check()
            if result == z3.sat:
                final_threshold = threshold
                best_model = self.solver.model()
                high_threshold = threshold - 1
            else:
                low_threshold = threshold + 1
            self.solver.pop()

        ## build tight bounds
        model = best_model

        # fix dummy variables
        for soft in self.soft_constr[2]:
            if model[soft.literal] == True:
                self.solver.add(soft.literal)
            elif model[soft.literal] == False:  
                self.solver.add(z3.Not(soft.literal))

        #self.solver.minimize(z3.Sum(u1, m1, -v1, -n1, u2, m2, -v2, -n2))

        # check if SAT or UNSAT
        result = self.solver.check()
        if result != z3.sat:
            print("unsatisfiable")
            return

        model = self.solver.model()

        ## generate 1000 random points inside the rule
        #rule_points = [[to_real(model[t])]]

        print('move north or south if confidence of treasure is >={:.3f}'.format(to_real(model[c])))
        print('fail to satisfy {} out of {} steps'.format(final_threshold, len(self.steps)))

        ### Hellinger distance of unsatisfiable steps
        #failed_rules = []
        #Hellinger_min = []
        #for num, soft in enumerate(self.soft_constr[1]):
        #    if model[soft.literal] == False or self.actions[num] != 'sample' :
        #        continue
        #    failed_rules.append(num)
        #    pos = self.positions[num]
        #    rock = self.rock_number(pos[0], pos[1])
        #    P = [self.beliefs[num][rock]]
        #    hel_dst = [Hellinger_distance(P, Q) for Q in rule_points]
        #    Hellinger_min.append(min(hel_dst))

        ## print unsatisfiable steps in decreasing order of hellinger distance
        #print('Unsatisfiable steps:')
        #for x, soft, hel in [[x, self.soft_constr[1][x], h] for h, x in sorted(zip(Hellinger_min, failed_rules), key=lambda pair: pair[0], reverse = True)]:
        #    if hel > self.threshold:
        #        print('ANOMALY: ', end='')
        #    pos = self.positions[x]
        #    rock = self.rock_number(pos[0], pos[1])
        #    print('run {} step {}: action {} with belief of valuable rock = {:.3f} --- Hellinger = {}'.format(
        #        soft.run, soft.step, self.actions[x],
        #        self.beliefs[x][rock],
        #        hel)
        #        )

        ### Hellinger distance of unsatisfiable steps
        #for num, soft in enumerate(self.soft_constr[1]):
        #    if model[soft.literal] == False or self.actions[num] == 'sample' :
        #        continue
        #    pos = self.positions[num]
        #    rock = self.rock_number(pos[0], pos[1])
        #    print('run {} step {}: do not sample with belief of valuable rock = {:.3f}'.format(soft.run, soft.step, self.beliefs[num][rock]))
    def synthetize_rules(self):
        """
        synthetize each rule
        """
        # sample rule
        self.solver.push()
        self.build_sample_rule()
        self.solver.pop()

        self.solver.push()
        self.build_check_rule()
        self.solver.pop()

        self.solver.push()
        self.build_north_south_rule()
        self.solver.pop()

########
# MAIN #
########

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print ('usage: xpomcp <log.xes>')
        exit()

    xes_log = str(sys.argv[1])

    rs = RuleSynth(
            xes_log=xes_log,
            threshold=0.1
            )
    rs.synthetize_rules()

