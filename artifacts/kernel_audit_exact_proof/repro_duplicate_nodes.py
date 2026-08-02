"""Воспроизведение: два узла скелета на одну точную пару (время, точка).

Запуск из корня репозитория:

    PYTHONPATH=kernel/src:kernel/tests python3 \
        artifacts/kernel_audit_exact_proof/repro_duplicate_nodes.py

Требование к окружению то же, что у repro_exact_no_rule.py (sympy для
фасада пакета, не для самого счёта).

Идентичность узла берётся канонической записью `node_record` — той же, что
входит в семантический дайджест. Две записи с одинаковыми (time, point) —
нарушение объявленной идентичности `SkeletonNodeV1` («сошедшиеся в одной
точке дают ОДИН узел»). Найденные фигуры — контрпримеры для ворот P0-2
WF-SAME-TIME-CLOSURE-GATE (ROADMAP, п. 0-bis).
"""

from __future__ import annotations

import json
import sys
from collections import Counter

from wavefront_cases import named_corpus, partial_source_corpus
from cftuv_envelope.wavefront.digest import node_record
from cftuv_envelope.wavefront.skeleton import build_skeleton


def main() -> None:
    findings = {}
    for name, polygon in list(named_corpus()) + list(partial_source_corpus()):
        skeleton = build_skeleton(polygon)
        keys: Counter = Counter()
        kinds: dict = {}
        for node in skeleton.nodes:
            record = node_record(node)
            key = json.dumps(
                [
                    record["time_dividend"],
                    record["time_divisor"],
                    record["point_x"],
                    record["point_y"],
                ]
            )
            keys[key] += 1
            kinds.setdefault(key, []).append(record["kind"])
        duplicated = {
            key: kinds[key] for key, count in keys.items() if count > 1
        }
        if duplicated:
            findings[name] = {
                "outcome": skeleton.outcome.value,
                "duplicate_key_count": len(duplicated),
                "kinds": sorted(duplicated.values()),
            }
    json.dump(
        {"cases_with_duplicates": len(findings), "cases": findings},
        sys.stdout,
        ensure_ascii=False,
        indent=1,
        sort_keys=True,
    )
    sys.stdout.write("\n")


if __name__ == "__main__":
    main()
