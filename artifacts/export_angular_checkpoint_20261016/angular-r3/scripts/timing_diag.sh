#!/bin/bash
# Diagnostic: the same branch tree, product default (ANGULAR PYTHON, no hook) vs ANGULAR NATIVE (hook). Not the main comparison.
S=D:/cftuv-angular-gates-20261017
TAG=$1; CASE=$2
cd $S/operator
python -B operator_controller.py \
  --base-tree D:/CFTUV-export-angular-20261016 --base-native-site $S/wheel-final2/site --base-build-id 82bff41899305761fdc7b76fd757954b1fc003b8a96c53668552e5d4e3bf986d \
  --branch-tree D:/CFTUV-export-angular-20261016 --branch-native-site $S/wheel-final2/site --branch-build-id 82bff41899305761fdc7b76fd757954b1fc003b8a96c53668552e5d4e3bf986d \
  --branch-env CFTUV_ANGULAR_BACKEND=NATIVE \
  --branch-hook-profiles "$S/cand/profile-test-CPython-3.11-light-82bff418.json;$S/cand/profile-test-CPython-3.11-full-82bff418.json" \
  --reps 3 --case $CASE --output $S/timing/$TAG > $S/timing-$TAG.log 2>&1
echo "exit $?"
tail -8 $S/timing-$TAG.log
