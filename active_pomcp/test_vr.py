import sys, csv, os
import math, random
import z3
import xml.etree.ElementTree as ET

#######
# XES #
#######
xes_ns = {'xes': 'rttp://www.w3.org/2001/XMLSchema'}

def node_from_key(root, key):
    return root.find('./*[@key="{}"]'.format(key))

def xes_attribute(root, key):
    atr = node_from_key(root, key)
    if atr.tag == '{rttp://www.w3.org/2001/XMLSchema}string':
        return atr.attrib['value']
    elif atr.tag == '{rttp://www.w3.org/2001/XMLSchema}int':
        return int(atr.attrib['value'])
    elif atr.tag == '{rttp://www.w3.org/2001/XMLSchema}float':
        return float(atr.attrib['value'])
    elif atr.tag == '{rttp://www.w3.org/2001/XMLSchema}boolean':
        return bool(atr.attrib['value'])
    return None

if __name__ == "__main__":
    # parse input files
    if len(sys.argv) != 6:
        print ('usage: xpomcp <log.xes> <x1> <x2> <x3> <x4>')
        exit()

    # optional limit on the number of runs
    max_runs = 100
    runs = 0

    xes_log = str(sys.argv[1])
    x1 = float(sys.argv[2])
    x2 = float(sys.argv[3])
    x3 = float(sys.argv[4])
    x4 = float(sys.argv[5])

    belief_in_runs  = []


    # statistics
    tp, fp, fn, tn = 0, 0, 0, 0
    for ev, trace in ET.iterparse(xes_log):
        if trace.tag == '{rttp://www.w3.org/2001/XMLSchema}trace':
            if max_runs and runs >= max_runs:
                break
            runs+=1

            belief_in_runs.append([])
            for event in trace.findall('xes:event', xes_ns):
                # attributes
                segment = int(node_from_key(event,'segment').attrib['value'])

                action = int(node_from_key(event,'action').attrib['value'])

                # belief
                belief_in_runs[-1].append({0:0, 1:0, 2:0})
                for i in node_from_key(event, 'belief'):
                    state = i.attrib['key']
                    particles = int(i.attrib['value'])
                    local_difficulty = (int(state) // (3 ** (7 - segment))) % 3
                    belief_in_runs[-1][-1][local_difficulty] += particles

                total = belief_in_runs[-1][-1][0] + belief_in_runs[-1][-1][1] + belief_in_runs[-1][-1][2]
                belief_in_runs[-1][-1][0] /= total
                belief_in_runs[-1][-1][1] /= total
                belief_in_runs[-1][-1][2] /= total
                
                p0 = belief_in_runs[-1][-1][0]
                p1 = belief_in_runs[-1][-1][1]
                p2 = belief_in_runs[-1][-1][2]

                rule_action = action == 2
                rule_belief = (p0 >= x1 or p2 <= x2 or (p0 >= x3 and p1 >= x4) )

                if (rule_belief and rule_action):
                    tp += 1
                elif (not rule_belief and not rule_action):
                    tn += 1
                elif (rule_belief and not rule_action):
                    fp += 1
                elif (not rule_belief and rule_action):
                    fn += 1

    precision = tp / (tp + fp)
    recall = tp / (tp + fn)
    f1 = (2*tp) / (2*tp + fp + fn)
    print(f1)

