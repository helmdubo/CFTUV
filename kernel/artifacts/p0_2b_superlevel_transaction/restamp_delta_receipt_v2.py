"""Пересъёмка ожиданий `new` в delta_receipt_v2 после P0-2B-FINISH.

Блок `old` НЕ трогается: это история, доснятая на приёмке P0-2b, и переписывать
её нечем. Пересъёмке подлежит только `new` — то, что стек даёт СЕГОДНЯ.

Закон, по которому пересъёмка законна (серия S): **счётчики считают содержимое
ответа, не поиск.** Семь счётчиков из восьми разошедшихся мерили машинерию
поиска — сколько кандидатов фильтр рассмотрел, сколько протухло, сколько дублей
сняла дедупликация; транзакция пересевает кандидатов иначе, и число рассмотренных
меняется, а ответ — нет. Восьмой, `refused_no_rule_triple_always_concurrent`,
мерит содержимое: он становится записью долга. Его сдвиг закрыт отдельной
распиской Q5 (`kernel/artifacts/p0_2_superlevel_transaction/
Q5_OBLIGATION_DIRECTION_TABLE_V1.json`): во всех четырёх случаях долгов стало
СТРОГО меньше, и строго меньше именно НЕДОКАЗАННОГО; ни одного случая в
обратную сторону на всех 63.

Что при этом НЕ сдвинулось и потому доказывает, что пересъёмка не прячет
регрессию: `outcome`, `semantic_digest` и `node_records_sha256` совпали с
дооснасткой на ВСЕХ 63 случаях.

Запуск (из корня репозитория):

    PYTHONPATH=kernel/src:kernel/tests python3 \
        artifacts/p0_2b_superlevel_transaction/restamp_delta_receipt_v2.py
"""

from __future__ import annotations

import collections
import json
from pathlib import Path

from cftuv_envelope.wavefront.digest import semantic_digest
from cftuv_envelope.wavefront.skeleton import build_skeleton
from wavefront_cases import named_corpus, partial_source_corpus


RECEIPT = Path(__file__).with_name("delta_receipt_v2.json")

SEARCH_MACHINERY = (
    "coincident_split_targets",
    "discarded_stale_candidates",
    "refused_filter_event_in_the_past",
    "refused_filter_point_outside_front",
    "refused_filter_span_does_not_collapse",
    "refused_filter_triple_never_concurrent",
    "split_candidates_examined",
)
ANSWER_CONTENT = ("refused_no_rule_triple_always_concurrent",)


def _cases():
    for name, polygon in named_corpus():
        yield f"named::{name}", polygon
    for name, polygon in partial_source_corpus():
        yield f"partial_source::{name}", polygon


def main() -> None:
    receipt = json.loads(RECEIPT.read_text(encoding="utf-8"))
    by_id = {case["case_id"]: case for case in receipt["cases"]}
    moved, guarded = {}, []
    for case_id, polygon in _cases():
        expected = by_id[case_id]["new"]
        skeleton = build_skeleton(polygon)
        counters = dict(skeleton.counters)
        outcome = skeleton.outcome.value
        digest = semantic_digest(skeleton)
        # Оси, которые пересъёмке НЕ подлежат: если поехали они, это регрессия,
        # а не смена единицы измерения.
        if outcome != expected["outcome"] or digest != expected["semantic_digest"]:
            guarded.append(case_id)
            continue
        changed = {
            name: [expected["legacy_counters"][name], counters.get(name, 0)]
            for name in expected["legacy_counters"]
            if counters.get(name, 0) != expected["legacy_counters"][name]
        }
        proof_counts = dict(collections.Counter(
            f"{item.cause.value}|{item.disposition.value}"
            for item in skeleton.proof_obligations
        ))
        status = skeleton.proof_status.value
        if not changed and proof_counts == expected["proof_counts"] and (
            status == expected["proof_status"]
        ):
            continue
        moved[case_id] = {
            "legacy_counters": changed,
            "proof_counts": [expected["proof_counts"], proof_counts],
            "proof_status": [expected["proof_status"], status],
        }
        expected["legacy_counters"] = {
            name: counters.get(name, 0) for name in expected["legacy_counters"]
        }
        expected["proof_counts"] = proof_counts
        expected["proof_status"] = status
    if guarded:
        raise SystemExit(
            "исход или дайджест сдвинулись — это регрессия, а не пересъёмка: "
            + ", ".join(guarded)
        )
    receipt["restamp_p0_2b_finish"] = {
        "schema": "P0_2B_FINISH_COUNTER_RESTAMP_V1",
        "law": "счётчики считают содержимое ответа, не поиск",
        "old_block_untouched": True,
        "axes_that_did_not_move": [
            "outcome", "semantic_digest", "node_records_sha256",
        ],
        "search_machinery_counters": list(SEARCH_MACHINERY),
        "answer_content_counters": list(ANSWER_CONTENT),
        "answer_content_receipt": (
            "kernel/artifacts/p0_2_superlevel_transaction/"
            "Q5_OBLIGATION_DIRECTION_TABLE_V1.json"
        ),
        "cases_restamped": len(moved),
        "detail": moved,
    }
    RECEIPT.write_text(
        json.dumps(receipt, indent=2, sort_keys=True, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    print(f"пересняты ожидания у {len(moved)} случаев из 63")


if __name__ == "__main__":
    main()
