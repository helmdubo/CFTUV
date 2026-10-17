#!/bin/bash
set -e
S=/d/cftuv-angular-gates-20261017
R3=/d/CFTUV-export-angular-20261016/artifacts/export_angular_checkpoint_20261016/angular-r3
cd $S
bash scripts/copy_evidence.sh > /dev/null
cp ab/full-summary.json $R3/treeab-full-summary.json
cp ab/full.log $R3/treeab-full.log
cp timing/t1/controller.json $R3/timing-t1-controller.json
cp timing/d1/controller.json $R3/timing-d1-controller.json
cp timing/t1-summary.txt $R3/timing-t1-summary.txt
cp timing/d1-summary.txt $R3/timing-d1-summary.txt
cp timing/bind-cost.json $R3/bind-cost.json
cp replay/final-82bf-elapsed.txt $R3/replay-final-82bf-elapsed.txt
for f in ab_summary.py timing_summary.py timing_run.sh timing_diag.sh bind_cost.py replay_elapsed.py patch_operator2.py copy_evidence2.sh; do cp scripts/$f $R3/scripts/; done
cp operator/operator_controller.py $R3/operator/
du -sh $R3
cd /d/CFTUV-export-angular-20261016
git status --short
