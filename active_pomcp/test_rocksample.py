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

########
# MAIN #
########

if __name__ == "__main__":
    # parse input files
    if len(sys.argv) != 3:
        print ('usage: xpomcp <log.xes> <limit>')
        exit()

    xes_log = str(sys.argv[1])
    upper_limit = float(sys.argv[2])

    # optional limit on the number of runs
    max_runs = None
    runs = 0

    # statistics
    tp, fp, fn, tn = 0, 0, 0, 0

    # collect problem info
    map_size = None
    num_rocks = None
    rocks = None
    for _, element in ET.iterparse(xes_log):
        if (element.tag == '{rttp://www.w3.org/2001/XMLSchema}list' and
                element.attrib['key'] == 'rocks'):
            rocks = []
            for r in element.findall('./*[@key="rock"]'):
                x = xes_attribute(r, 'coord x')
                y = xes_attribute(r, 'coord y')

                rocks.append([x, y])
        elif (element.tag == '{rttp://www.w3.org/2001/XMLSchema}int' and
                element.attrib['key'] == 'Size'):
            map_size = int(element.attrib['value'])

        elif (element.tag == '{rttp://www.w3.org/2001/XMLSchema}int' and
                element.attrib['key'] == 'NumRocks'):
            num_rocks = int(element.attrib['value'])

        if map_size is not None and num_rocks is not None and rocks is not None:
            break

    for ev, trace in ET.iterparse(xes_log):
        if trace.tag == '{rttp://www.w3.org/2001/XMLSchema}trace':
            if max_runs and runs >= max_runs:
                break
            runs+=1

            sampled = [0] * num_rocks

            for event in trace.findall('xes:event', xes_ns):
                x = xes_attribute(event,'coord x')
                y = xes_attribute(event,'coord y')

                num_rock = -1
                for i, r in enumerate(rocks):
                    if r[0] == x and r[1] == y:
                        num_rock = i
                        break

                if num_rock == -1 or sampled[num_rock] == 1:
                    continue

                action = xes_attribute(event,'action')
                is_sample = action == 'sample'

                # belief
                beliefs = [0] * num_rocks
                total = 0
                for i in node_from_key(event, 'belief'):
                    state = int(i.attrib['key'])
                    particles = int(i.attrib['value'])
                    for i in range(0, num_rocks):
                        if (state // (2**i)) % 2 == 1:
                            beliefs[i] += particles
                    total += particles

                beliefs = [ b / total for b in beliefs]

                if (beliefs[num_rock] >= upper_limit and is_sample):
                    tp += 1
                elif (beliefs[num_rock] < upper_limit and not is_sample):
                    tn += 1
                elif (beliefs[num_rock] >= upper_limit and not is_sample):
                    fp += 1
                elif (beliefs[num_rock] < upper_limit and is_sample):
                    fn += 1
                sampled[num_rock] = 1
            trace.clear()

    precision = tp / (tp + fp)
    recall = tp / (tp + fn)
    f1 = (2*tp) / (2*tp + fp + fn)

    print('precision',precision, 'recall', recall, 'f1', f1)


