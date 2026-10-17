#!/bin/bash
set -e
S=/d/cftuv-angular-gates-20261017
R4=/d/CFTUV-export-angular-20261016/artifacts/export_angular_checkpoint_20261016/angular-r4
mkdir -p $R4/scripts
cd $S
cp cand/light-0cb3.json $R4/profile-candidate-311-light-0cb3.json
cp cand/full-0cb3.json $R4/profile-candidate-311-full-0cb3.json
cp cand/full-0cb3.json.receipt.json $R4/profile-candidate-311-full-0cb3.json.receipt.json
cp cand/profile-test-CPython-3.11-light-6d580b97.json cand/profile-test-CPython-3.11-full-6d580b97.json $R4/
cp replay-6d58/replay.json $R4/replay-6d58.json
cp replay-6d58-elapsed.txt replay-6d58-elapsed-warm.txt $R4/
cp pymut-results.json $R4/shim-mutants.json
cp timing/bind-cost-f3.json $R4/bind-cost-f3.json
for t in t3 t4 d3; do cp timing/$t/controller.json $R4/timing-$t-controller.json; cp timing/$t-summary.txt $R4/timing-$t-summary.txt; done
for t in g3w0 g3w12; do tail -30 ab/$t.log > $R4/treeab-$t-tail.log; done
cp ab/g3w12-stages.txt $R4/treeab-g3w12-stages.txt
for f in py_mutants.py bind_once_extra.py replay_elapsed_warm.py cands.sh ab_run3.sh timing_run3.sh timing_run5.sh timing_diag3.sh edit_pool_loader.py copy_evidence_r4.sh; do cp scripts/$f $R4/scripts/; done
du -sh $R4
