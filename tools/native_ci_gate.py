"""Вердикт `native-gate` — единственной проверки `native.yml`, которую можно сделать обязательной (задача `native-gate`).

Проходит ТОЛЬКО если `changes` отработала И (нативных путей коммит не трогал ИЛИ каждая тяжёлая задача (`rust`, `wheel`, `corpus`, `differential`) закончилась `success`). Всё остальное — провал
с названной причиной: `changes` упала либо отменена, её выход не `true`/`false`, тяжёлая задача упала, отменена либо ПРОПУЩЕНА при `native=true` (пропущенная задача не доказывает ничего).

    python tools/native_ci_gate.py     # среда: CHANGES_RESULT, NATIVE, RUST_RESULT, WHEEL_RESULT, CORPUS_RESULT, DIFFERENTIAL_RESULT (`needs.<задача>.result`, `needs.changes.outputs.native`)

Тяжёлые задачи в вердикт входят по именам `HEAVY_JOBS`; совещательная `interpreters-advisory` не входит намеренно (красная ножка там — сведение, не блок).
"""

from __future__ import annotations

import os
import sys

HEAVY_JOBS = ("rust", "wheel", "corpus", "differential")


def decide(changes: str, native: str, results: dict) -> tuple:
    """`(прошло, причина)` по результату `changes`, её выходу `native` и результатам тяжёлых задач `{имя: результат}`."""

    if changes != "success":
        return False, f"the `changes` job did not succeed (result {changes!r}): it is not known whether the native acceptance is required"
    if native == "false":
        return True, "no native-relevant path changed: the heavy jobs were not required (and were skipped)"
    if native != "true":
        return False, f"the `changes` job gave an unknown output native={native!r} (expected `true` or `false`)"
    bad = {name: results.get(name, "") for name in HEAVY_JOBS if results.get(name, "") != "success"}
    if bad:
        return False, "a native-relevant change, and the heavy jobs that are not `success`: " + ", ".join(f"{name}={result or 'missing'}" for name, result in bad.items())
    return True, "a native-relevant change, and every heavy job succeeded: " + ", ".join(f"{name}={results[name]}" for name in HEAVY_JOBS)


def main(environment: dict | None = None) -> int:
    environment = dict(os.environ if environment is None else environment)
    results = {name: environment.get(f"{name.upper()}_RESULT", "") for name in HEAVY_JOBS}
    changes, native = environment.get("CHANGES_RESULT", ""), environment.get("NATIVE", "")
    passed, why = decide(changes, native, results)
    print(f"changes={changes or 'missing'} native={native or 'missing'} " + " ".join(f"{name}={result or 'missing'}" for name, result in results.items()))
    print(("NATIVE_GATE_PASSED " if passed else "NATIVE_GATE_FAILED ") + why)
    return 0 if passed else 1


if __name__ == "__main__":
    sys.exit(main())
