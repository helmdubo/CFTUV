#!/bin/bash
S=D:/cftuv-angular-gates-20261017
TAG=$1
cd $S/operator
python -B operator_controller.py \
  --base-tree E:/GITHUB/CFTUV --base-native-site C:/Users/helmd/.cftuv-native/r3-native --base-build-id ab93d1820a2bd69601cfd57da607a7e6b99387cc56812077b9e018c58c9e25e9 \
  --branch-tree D:/CFTUV-export-angular-20261016 --branch-native-site $S/wheel-final2/site --branch-build-id 82bff41899305761fdc7b76fd757954b1fc003b8a96c53668552e5d4e3bf986d \
  --branch-env CFTUV_ANGULAR_BACKEND=NATIVE \
  --branch-hook-profiles "$S/cand/profile-test-CPython-3.11-light-82bff418.json;$S/cand/profile-test-CPython-3.11-full-82bff418.json" \
  --reps 3 --case both --output $S/timing/$TAG > $S/timing-$TAG.log 2>&1
echo "exit $?"
tail -20 $S/timing-$TAG.log
