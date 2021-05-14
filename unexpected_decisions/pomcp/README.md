POMCP
-----
Modification of the POMCP algorithm written by David Silver and Joel Veness.
We use this algorithm to generate the traces of the tiger and the velocity regulation problems.

To compile the code, run the commands:
```
mkdir build && cd build
cmake ..
make
```

TIGER
-----
To generate 1000 traces of tiger, run the command:

```
./pomcp --problem tiger --runs=1000 --mindoubles=15 --maxdoubles=15 --verbose=1 --numsteps=100 --relknowlevel=0 --usetransforms=0 --testing=0 --seed=1
```
by changing `--runs` it is possible to modify the number of runs in a trace, and `--seed` is used to fix the seed of the random number generator.
The trace is automatically stored into the `myExp` folder.
To generate our dataset of 50 tiger traces, we use all the seed in the interval [1, 50].
The `RewardRange` parameter is a fixed value specified at line 12 in the file `src/tiger.cpp`. The expected values is 110, by reducing it some errors occour.

VELOCITY REGULATION
-------------------
To generate 100 traces of velocity regulation (that is named obstacle avoidance in the code), run the command:

```
/pomcp --problem obstacleavoidance --runs=100 --mindoubles=15 --maxdoubles=15 --verbose=1 --numsteps=100 --relknowlevel=0 --usetransforms=0 --testing=1
```

by changing `--runs` it is possible to modify the number of runs in a trace, the seed is fixed in the code when using `--testing=1`.
The trace is automatically stored into the `myExp` folder.
The `RewardRange` parameter is a fixed value specified at line 19 in the file `src/obstacleavoidance.cpp`. The expected values is 103, by reducing it some errors occour.

