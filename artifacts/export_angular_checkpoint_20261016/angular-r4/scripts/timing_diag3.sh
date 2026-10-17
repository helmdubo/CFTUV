#!/bin/bash
# Diagnostic: the same branch tree, product default (ANGULAR PYTHON, no hook) vs ANGULAR NATIVE (hook). Not the main comparison.
S=D:/cftuv-angular-gates-20261017
TAG=$1; CASE=$2
cd $S/operator
python -B operator_controller.py \
  --base-tree D:/CFTUV-export-angular-20261016 --base-native-site $S/wheel-final3/site --base-build-id 6d580b9752342d6320b3c3ef1d7825d237a608d018dbe007ec844d3cf097eb9e \
  --branch-tree D:/CFTUV-export-angular-20261016 --branch-native-site $S/wheel-final3/site --branch-build-id 6d580b9752342d6320b3c3ef1d7825d237a608d018dbe007ec844d3cf097eb9e \
  --branch-env CFTUV_ANGULAR_BACKEND=NATIVE \
  --branch-hook-profiles "$S/cand/profile-test-CPython-3.11-light-6d580b97.json;$S/cand/profile-test-CPython-3.11-full-6d580b97.json" \
  --reps 3 --case $CASE --output $S/timing/$TAG > $S/timing-$TAG.log 2>&1
echo "exit $?"
tail -8 $S/timing-$TAG.log
