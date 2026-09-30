#!/usr/bin/env bash
# Ступенчатая добавка шимов на patch 7 d2, N проходов (шум машины ±10 %, поэтому повторы).
# bash artifacts/numeric_repr/run_proto_chain.sh <passes>
B=artifacts/numeric_repr/baseline_e548fb5.json
OUT=artifacts/numeric_repr/proto_results/chain
mkdir -p $OUT
for pass in $(seq 1 ${1:-2}); do
  for sh in none sign sign,compare sign,compare,mul sign,compare,mul,addsub sign,compare,mul,addsub,radical sign,compare,mul,addsub,radical,floorcache; do
    tag=$(echo $sh | tr ',' '+')
    python artifacts/numeric_repr/proto_run.py 7 2 $sh $B $OUT/p7_d2_pass${pass}_${tag}.json > /dev/null 2>&1
  done
done
echo CHAIN_DONE > $OUT/DONE
