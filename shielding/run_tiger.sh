#!/bin/bash
LOG=/dev/null
POMCP=./build/pomcp

echo "" > $LOG

Ws="110 80 60 40"
RUNS=1000
TRAINING_SEED=5
TESTING_SEED=1234

# generate training dataset
for i in ${Ws}; do
    echo generate dataset W=$i;
    $POMCP --problem tiger --runs=$RUNS --mindoubles=15 --maxdoubles=15 --verbose=0 --usetransforms=0 --seed=$TRAINING_SEED --useshield=0 --setW=$i >> $LOG;
    mv log.xes dataset_tiger_${i}.xes
done

# train
for i in ${Ws}; do
    echo train shield with W=$i;
    python3 shielding_tiger.py dataset_tiger_${i}.xes > tiger_shield_${i}.txt
done

# testing
for i in ${Ws}; do
    echo test W=$i without shield;
    $POMCP --problem tiger --runs=$RUNS --mindoubles=15 --maxdoubles=15 --verbose=0 --usetransforms=0 --seed=$TESTING_SEED --useshield=0 --setW=$i >> $LOG;
    mv log.xes tiger_${i}_no_shield.xes
done

for i in ${Ws}; do
    for j in ${Ws}; do
        echo test W=$i with shield $j;
        $POMCP --problem tiger --runs=$RUNS --mindoubles=15 --maxdoubles=15 --verbose=0 --usetransforms=0 --seed=$TESTING_SEED --useshield=1 --complexshield=1 --setW=$i --shieldfile tiger_shield_${j}.txt >> $LOG
        mv log.xes tiger_${i}_shield_${j}.xes
    done;
done
