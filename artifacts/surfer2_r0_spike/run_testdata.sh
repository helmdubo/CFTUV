#!/bin/bash
# Внешняя замена цели runtests_cc: она НЕ собирается с gtest 1.14
# (INSTANTIATE_TEST_CASE_P с висячей запятой -> "Empty arguments are not allowed").
# Код Surfer2 не правим, поэтому делаем ровно то же, что делал бы их тест:
# гоняем surfer на каждом .graphml из test-data и требуем rc=0.
S="$1"; D="$2"
pass=0; fail=0
: > /tmp/testdata_failures.txt
while IFS= read -r f; do
  if timeout 300 "$S" "$f" /dev/null >/dev/null 2>&1; then
    pass=$((pass+1))
  else
    rc=$?
    fail=$((fail+1))
    echo "rc=$rc $f" >> /tmp/testdata_failures.txt
  fi
done < <(find "$D" -name '*.graphml' | sort)
echo "PASS=$pass FAIL=$fail"
cat /tmp/testdata_failures.txt
