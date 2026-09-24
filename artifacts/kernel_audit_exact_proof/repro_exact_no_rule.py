"""Воспроизведение: EXACT сосуществует с NO_RULE на большинстве корпуса.

Запуск из корня репозитория:

    PYTHONPATH=kernel/src:kernel/tests python3 \
        artifacts/kernel_audit_exact_proof/repro_exact_no_rule.py

Сам счёт волнового фронта SymPy не использует, но фасад пакета
`cftuv_envelope/__init__.py` импортирует его на уровне модуля
(`source_grid.py`), поэтому для запуска нужен установленный sympy+mpmath
ЛИБО заглушка на PYTHONPATH (см. P1 TEST-COLLECTION-BOOT в ROADMAP).

Выход — JSON в stdout: по каждой фигуре обоих корпусов outcome и ненулевые
NO_RULE-счётчики, плюс сводка. Число в сводке — довод, по которому жёсткий
инвариант «EXACT ⇒ NO_RULE = 0» отвергнут (DECISIONS, 2026-08-02).
"""

from __future__ import annotations

import json
import sys

from wavefront_cases import named_corpus, partial_source_corpus
from cftuv_envelope.wavefront.skeleton import (
    CandidateRefusal,
    build_skeleton,
    refusal_counter,
)


def main() -> None:
    no_rule = [
        refusal_counter(member)
        for member in CandidateRefusal
        if member.value.startswith("NO_RULE")
    ]
    cases = {}
    exact_with_no_rule = 0
    for name, polygon in list(named_corpus()) + list(partial_source_corpus()):
        skeleton = build_skeleton(polygon)
        nonzero = {
            counter: skeleton.counter(counter)
            for counter in no_rule
            if skeleton.counter(counter)
        }
        cases[name] = {"outcome": skeleton.outcome.value, "no_rule": nonzero}
        if nonzero and skeleton.outcome.value == "EXACT":
            exact_with_no_rule += 1
    json.dump(
        {
            "total_cases": len(cases),
            "exact_with_nonzero_no_rule": exact_with_no_rule,
            "cases": cases,
        },
        sys.stdout,
        ensure_ascii=False,
        indent=1,
        sort_keys=True,
    )
    sys.stdout.write("\n")


if __name__ == "__main__":
    main()
