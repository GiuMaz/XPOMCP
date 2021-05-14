This file contains a modified version of pomcp (written in c++) and an implementation of the shielding generation algorithm described in the algorithm.
It also contains a xes log file that can be used to test the shield generation program (`dataset_tiger_60.xes`) and a shield file that can be used to test the POMCP with a shield (`tiger_shield_60.txt`).

# POMCP
This folder contains a modification of the POMCP algorithm written by David Silver and Joel Veness.
We use this algorithm to generate the traces of the tiger and the velocity regulation problems.

To compile the code, run the commands:
```
mkdir build && cd build
cmake ..
make
```

## TIGER
To generate a trace of tiger with 1000 runs, run the command:
```
./pomcp --problem tiger --runs=1000 --mindoubles=15 --maxdoubles=15 --verbose=0 --usetransforms=0 --seed=1234 --useshield=0 --setW=110
```
by changing `--runs` it is possible to modify the number of runs in a trace, and `--seed` is used to fix the seed of the random number generator.
It is possible to introduce errors using `--setW` (the c value in the paper).
The trace is automatically stored in the log.xes file.

## VELOCITY REGULATION
To generate 100 traces of velocity regulation (that is named obstacle avoidance in the code), run the command:
```
/pomcp --problem obstacleavoidance --runs=100 --mindoubles=15 --maxdoubles=15 --verbose=0 --usetransforms=0 --seed=1234 --useshield=0 --setW=103
```
by changing `--runs` it is possible to modify the number of runs in a trace, and `--seed` is used to fix the seed of the random number generator.
The trace is automatically stored in the log.xes file.
It is possible to introduce errors using `--setW` (the c value of the paper).


## SHIELDED POMCP
To run the pomcp using a shield, use the command:
```
./pomcp --problem tiger --runs=1000 --mindoubles=15 --maxdoubles=15 --verbose=0 --usetransforms=0 --seed=1234 --useshield=1 --complexshield=1 --setW=60 --shieldfile ./tiger_shield_60.txt
```
by changing `--runs` it is possible to modify the number of runs in a trace, and `--seed` is used to fix the seed of the random number generator.
`--useshield` and `--complexshield` must be set at 1 to reproduce the results of the article.
The shield must be stored in a file. This supplementary material has one shield already generated. For generating more shield use `./run_tiger.sh` and `run_vr.sh`.

# SHIELD GENERATION
Implementation of the shielding methodology for the tiger and the velocity regulation domains.
The code is written in python and requires python3 and virtualenv to be used.
It was tested on Ubuntu 18.04.

To run the code, create a virtual environment in the folder that contains this README.md using the commands:
```
virtualenv -p python3 shielding
source shielding/bin/activate
pip install -r requirements.txt
```
To try it on an example trace, use the command:
```
python shielding_tiger.py dataset_tiger_60.xes
```
It generates a shield for the tiger problem in the case of c=60

