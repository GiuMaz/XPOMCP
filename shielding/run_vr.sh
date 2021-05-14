#!/bin/bash
LOG=/dev/null

echo "" > $LOG

Ws="103 90 70 50"
RUNS=100
TRAINING_SEED=1
TESTING_SEED=1234

# generate training dataset
for i in ${Ws}; do
    echo generate dataset W=$i;
    ./pomcp --problem obstacleavoidance --runs=$RUNS --mindoubles=15 --maxdoubles=15 --verbose=0 --usetransforms=0 --seed=$TRAINING_SEED --useshield=0 --setW=$i >> $LOG;
    mv log.xes dataset_vr_${i}.xes
done

# train
for i in $Ws; do
    echo train shield with W=$i;
    python3 xpomcp_vr.py dataset_vr_${i}.xes > vr_shield_${i}.txt
done

# testing
for i in $Ws; do
    echo test W=$i without shield;
    ./pomcp --problem obstacleavoidance --runs=$RUNS --mindoubles=15 --maxdoubles=15 --verbose=0 --usetransforms=0 --seed=$TESTING_SEED --useshield=0 --setW=$i >> $LOG;
    mv log.xes vr_${i}_no_shield.xes
done

for i in $Ws; do
    for j in $Ws; do
        echo test W=$i with shield $j;
        ./pomcp --problem obstacleavoidance --runs=$RUNS --mindoubles=15 --maxdoubles=15 --verbose=0 --usetransforms=0 --seed=$TESTING_SEED --useshield=1 --complexshield=1 --setW=$i --shieldfile vr_shield_${j}.txt >> $LOG
        mv log.xes vr_${i}_shield_${j}.xes
    done;
done
