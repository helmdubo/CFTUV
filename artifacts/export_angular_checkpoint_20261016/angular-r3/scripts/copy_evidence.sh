#!/bin/bash
set -e
cd /d/CFTUV-export-angular-20261016
R3=artifacts/export_angular_checkpoint_20261016/angular-r3
S=/d/cftuv-angular-gates-20261017
mkdir -p $R3/abhook $R3/operator $R3/scripts
cp $S/abhook/angular_pin_hook.py $S/abhook/sitecustomize.py $R3/abhook/
cp $S/operator/operator_controller.py $S/operator/operator_runner.py $R3/operator/
for f in angular_tree_ab.py ab_run.sh make_profiles.py cand_values.py ab_angular_counts.py patch_operator.py add_angular_files.py edit_angular_op.py edit_ast.py edit_shim.py ast_hash.py mutants.py run_mutants.py recheck_survivors.py replay.py build_wheel.ps1 summarize_mutants.py copy_evidence.sh; do
  cp $S/scripts/$f $R3/scripts/
done
cp $S/cand/light-ff5e.json $R3/profile-candidate-311-light-ff5e.json
cp $S/cand/full-ff5e.json $R3/profile-candidate-311-full-ff5e.json
cp $S/cand/full-ff5e.json.receipt.json $R3/profile-candidate-311-full-ff5e.json.receipt.json
cp $S/cand/profile-test-CPython-3.11-light-82bff418.json $S/cand/profile-test-CPython-3.11-full-82bff418.json $R3/
cp $S/replay/final-82bf-launch/summary.json $R3/replay-final-82bf-summary.json
cp $S/replay/final-82bf/replay.json $R3/replay-final-82bf.json
if [ -f $S/ab/full.json ]; then cp $S/ab/full.log $R3/treeab-full.log; fi
ls $R3
du -sh $R3
