#!/bin/bash
S=D:/cftuv-angular-gates-20261017
TAG=$1
cd $S/operator
python -B operator_controller.py \
  --base-tree E:/GITHUB/CFTUV --base-native-site C:/Users/helmd/.cftuv-native/r3-native --base-build-id ab93d1820a2bd69601cfd57da607a7e6b99387cc56812077b9e018c58c9e25e9 \
  --branch-tree D:/CFTUV-export-angular-20261016 --branch-native-site $S/wheel-final3/site --branch-build-id 6d580b9752342d6320b3c3ef1d7825d237a608d018dbe007ec844d3cf097eb9e \
  --branch-env CFTUV_ANGULAR_BACKEND=NATIVE \
  --branch-hook-profiles "$S/cand/profile-test-CPython-3.11-light-6d580b97.json;$S/cand/profile-test-CPython-3.11-full-6d580b97.json" \
  --reps 5 --case both --output $S/timing/$TAG > $S/timing-$TAG.log 2>&1
echo "exit $?"
tail -20 $S/timing-$TAG.log
