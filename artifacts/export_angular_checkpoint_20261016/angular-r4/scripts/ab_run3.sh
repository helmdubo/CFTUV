#!/bin/bash
# usage: ab_run.sh TAG WORKERS PRESSES CASE... ; arm A PYTHON vs arm B NATIVE, both with the TEST-ONLY pin hook.
S=D:/cftuv-angular-gates-20261017
T=D:/CFTUV-export-angular-20261016
TAG=$1; WORKERS=$2; PRESSES=$3; shift 3
CASES=()
for c in "$@"; do CASES+=(--case "$c"); done
cd $S
python -B scripts/angular_tree_ab.py --tool-tree $T \
  --profiles "$S/cand/profile-test-CPython-3.11-light-6d580b97.json;$S/cand/profile-test-CPython-3.11-full-6d580b97.json" \
  --hook-log $S/ab/$TAG-hook.jsonl --tmp $S/ab/$TAG-tmp -- \
  --base $T --branch $T --base-env CFTUV_ANGULAR_BACKEND=PYTHON --branch-env CFTUV_ANGULAR_BACKEND=NATIVE \
  --scene C:/Users/helmd/AppData/Local/Temp/cftuv_field_parity_20261008/frozen/testscene.blend \
  --scene cover008=C:/Users/helmd/AppData/Local/Temp/cftuv_cover008_frozen_v2/buildings2_2.blend \
  --expect-scene-sha 750baefc50ddbccad2ff090dae135c55e7b0d1584ca62697261a32b249e15fb4 \
  --expect-scene-sha cover008=a044b5aa8d5156bcf64167d7181557fb563b2d1472d8f837b4c6a51ce414ab32 \
  "${CASES[@]}" --workers $WORKERS --presses $PRESSES \
  --native-path $S/wheel-final3/site --expect-native-build-id 6d580b9752342d6320b3c3ef1d7825d237a608d018dbe007ec844d3cf097eb9e \
  --keys-may-differ --work-dir $S/ab/$TAG-work --out $S/ab/$TAG.json > $S/ab/$TAG.log 2>&1
echo "exit $?"
tail -25 $S/ab/$TAG.log
