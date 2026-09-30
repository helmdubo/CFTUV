#!/usr/bin/env bash
# Матрица прототипов: одиночные домены d2, последовательно (без конкуренции за ядра).
# Запуск из корня worktree: bash artifacts/numeric_repr/run_proto_matrix.sh
B=artifacts/numeric_repr/baseline_e548fb5.json
OUT=artifacts/numeric_repr/proto_results
ALL=sign,compare,mul,addsub,radical,floorcache
for patch in 7 6 1; do
  for sh in none $ALL; do
    tag=$(echo $sh | tr ',' '+')
    python artifacts/numeric_repr/proto_run.py $patch 2 $sh $B $OUT/p${patch}_d2_${tag}.json > /dev/null 2>&1
  done
done
# ступенчатая добавка шимов только на patch 7 (самый дешёвый из трёх)
for sh in sign sign,compare sign,compare,mul sign,compare,mul,addsub sign,compare,mul,addsub,radical; do
  tag=$(echo $sh | tr ',' '+')
  python artifacts/numeric_repr/proto_run.py 7 2 $sh $B $OUT/p7_d2_${tag}.json > /dev/null 2>&1
done
echo MATRIX_DONE > $OUT/DONE
