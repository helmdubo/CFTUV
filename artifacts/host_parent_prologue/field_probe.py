"""Сверка пролога кнопки и кодировщика ключа с прежними на ЗАПИСАННОМ полевом слепке `building` (122 домена); печатает одну строку JSON.

Отдельный процесс, потому что харнесс `artifacts/perf_prepare_diag/env.py` дополняет заглушку `mathutils.Vector` на весь процесс
(унарный минус, равенство, хэш): в общем процессе хостовой сюиты это была бы невидимая правка окружения соседних тестов. Тот же приём,
что у `artifacts/host_prologue_index/field_probe.py`. Запускает `tests/test_envelope_stage_inputs_lean.py`; руками:

    python artifacts/host_parent_prologue/field_probe.py

Сверяется:

* облегчённый пролог кнопки (`stage_production_inputs`) с полным (`stage_domain_inputs`) на выделении слепка (458 рёбер), на каждом ребре
  отдельно (частичное выделение цепочки дополняется до целой) и на случайных подмножествах: четыре величины и записи профиля (счётчики,
  квитанции, имена стадий в порядке записи);
* ключ содержимого каждого домена (`domain_content_key`, таблица типов и планы записи) с ключом на прежнем рекурсивном кодировщике
  (`tests/content_key_legacy.py`): плотности 0..4, допуск растяжения умолчания и иной, ключ полосы с досягаемостью умолчания и иной.
"""

from __future__ import annotations

import json
import random
import sys
from fractions import Fraction
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "artifacts" / "perf_prepare_diag"))

import env  # noqa: E402,F401  (пути дерева и заглушки bpy/mathutils)
import big_scene  # noqa: E402

from cftuv.envelope_content_key import domain_content_key  # noqa: E402
from cftuv.envelope_debug_profile import EnvelopeDebugProfileBuilderV1  # noqa: E402
from cftuv.envelope_export_input import build_host_export_input  # noqa: E402
from cftuv.envelope_metric_export import band_key_of  # noqa: E402
from cftuv.envelope_topology_export import (  # noqa: E402
    build_envelope_topology_export,
    stage_domain_inputs,
    stage_production_inputs,
)
from content_key_legacy import legacy_domain_content_key  # noqa: E402


def records(profile):
    snapshot = profile.snapshot()
    return (
        tuple(snapshot.counters),
        tuple(snapshot.receipts),
        tuple((item.stage, item.patch_domain_id) for item in snapshot.timings),
    )


def outcome(function, bundle, selection, topology):
    profile = EnvelopeDebugProfileBuilderV1("building", "PRODUCTION")
    try:
        value = function(bundle, selection, profile=profile, topology_export=topology)
    except Exception as exc:  # noqa: BLE001 - сравнивается и исключение
        return ("raised", type(exc).__name__, str(exc)), records(profile)
    return ("ok", value), records(profile)


def full_four(bundle, selection, *, profile, topology_export):
    return stage_domain_inputs(bundle, selection, profile=profile, topology_export=topology_export)[1:]


def lean(bundle, selection, *, profile, topology_export):
    return stage_production_inputs(bundle, selection, profile=profile, topology_export=topology_export)


def main() -> None:
    payload, _bm, bundle, _seconds = big_scene.build()
    selected = frozenset(int(value) for value in payload["raw"]["selected_edges"])
    topology = build_envelope_topology_export(bundle)
    mismatches: list[str] = []

    edges = sorted({int(edge) for record in topology.host_chains for edge in record.canonical_edge_ids})
    rng = random.Random(5)
    selections = [selected, frozenset(edges)]
    selections += [frozenset({edge}) for edge in rng.sample(edges, min(12, len(edges)))]
    selections += [frozenset(rng.sample(edges, rng.randint(1, len(edges)))) for _ in range(8)]
    answered = 0
    profile_records_equal = True
    for selection in selections:
        left, right = outcome(lean, bundle, selection, topology), outcome(full_four, bundle, selection, topology)
        if left[0] != right[0]:
            mismatches.append(f"answer of a selection of {len(selection)} edges")
        if left[1] != right[1]:
            profile_records_equal = False
            mismatches.append(f"profile records of a selection of {len(selection)} edges")
        answered += left[0][0] == "ok"

    banded = topology.with_chart_band(None, selected)
    _scene, revision, patch_ids, request_id, by_domain = stage_domain_inputs(bundle, selected, topology_export=banded)
    keys_compared = 0
    for density in ("0", "1", "2", "3", "4"):
        for budget in (None, Fraction(1, 4)):
            narrowed = banded.with_developable_stretch_budget(budget)
            for patch in patch_ids:
                export = build_host_export_input(narrowed, patch, alpha=0.25, request_id=request_id, density=density)
                own = frozenset(by_domain[narrowed.patch_domain_id_by_patch[patch]])
                band = band_key_of(narrowed, patch)
                if domain_content_key(export, own, band) != legacy_domain_content_key(export, own, band):
                    mismatches.append(f"key of patch {patch} density {density} budget {budget}")
                keys_compared += 1
    for reach in (Fraction(1, 4), Fraction(3, 2)):
        wide = topology.with_chart_band(reach, selected)
        for patch in patch_ids:
            export = build_host_export_input(wide, patch, alpha=0.25, request_id=request_id, density="1")
            own = frozenset(by_domain[wide.patch_domain_id_by_patch[patch]])
            band = band_key_of(wide, patch)
            if domain_content_key(export, own, band) != legacy_domain_content_key(export, own, band):
                mismatches.append(f"key of patch {patch} reach {reach}")
            keys_compared += 1
    print(
        json.dumps(
            {
                "domains": len(patch_ids),
                "lean_selections": len(selections),
                "lean_answered": answered,
                "lean_profile_records_equal": profile_records_equal,
                "keys_compared": keys_compared,
                "selected_edges": len(selected),
                "mismatches": mismatches,
            }
        )
    )


main()
