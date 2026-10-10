"""Сверка индекса с прежним фильтром на ЗАПИСАННОМ полевом слепке `building` (122 домена); печатает одну строку JSON.

Отдельный процесс, потому что харнесс `artifacts/perf_prepare_diag/env.py` дополняет заглушку `mathutils.Vector` на весь процесс
(унарный минус, равенство, хэш): в общем процессе хостовой сюиты это была бы невидимая правка окружения соседних тестов. Тот же
приём, что у `tests/test_field_release_matrix.py`. Запускает `tests/test_surface_index.py`; руками:

    python artifacts/host_prologue_index/field_probe.py

На каждом домене слепка сверяются: вид пакета (граф, пять кортежей поверхности и кольцо соседей, по тождеству записей и порядку),
вход выгрузки домена (`HostExportInputV1`, равенство значений), ключ содержимого с ключом полосы. Один вид на ВСЕ патчи сразу.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "artifacts" / "perf_prepare_diag"))

import env  # noqa: E402,F401  (пути дерева и заглушки bpy/mathutils)
import big_scene  # noqa: E402

from cftuv.envelope_content_key import domain_content_key  # noqa: E402
from cftuv.envelope_export_input import build_host_export_input  # noqa: E402
from cftuv.envelope_metric_export import band_key_of  # noqa: E402
from cftuv.envelope_topology_export import build_analysis_bundle_id_view, build_envelope_topology_export  # noqa: E402
from prologue_index_legacy import (  # noqa: E402
    legacy_band_key_of,
    legacy_host_export_input,
    legacy_view,
    view_parts,
)


def main() -> None:
    payload, _bm, bundle, _seconds = big_scene.build()
    selected = frozenset(int(value) for value in payload["raw"]["selected_edges"])
    topology = build_envelope_topology_export(bundle)
    banded = topology.with_chart_band(None, selected)
    mismatches: list[str] = []
    patches = sorted(int(patch) for patch in bundle.patch_graph.nodes)
    views = inputs = ring_faces = 0
    for patch in patches:
        new = build_analysis_bundle_id_view(bundle, frozenset({patch}))
        old = legacy_view(bundle, frozenset({patch}))
        if view_parts(new) != view_parts(old) or new.patch_surface != old.patch_surface:
            mismatches.append(f"view {patch}")
        else:
            views += 1
        ring_faces += bool(new.patch_surface.neighbour_faces)
        own = frozenset(
            int(edge) for record in topology.host_chains if record.patch_id == patch for edge in record.canonical_edge_ids
        ) & selected
        fresh = build_host_export_input(topology, patch, alpha=0.25, request_id="request", density="1")
        legacy = legacy_host_export_input(topology, patch, alpha=0.25, request_id="request", density="1")
        if view_parts(build_analysis_bundle_id_view(fresh.bundle, frozenset({patch}))) != view_parts(
            legacy_view(fresh.bundle, frozenset({patch}))
        ):
            mismatches.append(f"light view {patch}")
        key_new = domain_content_key(fresh, own, band_key_of(banded, patch))
        key_old = domain_content_key(legacy, own, legacy_band_key_of(banded, patch))
        if fresh != legacy or key_new != key_old or band_key_of(banded, patch) != legacy_band_key_of(banded, patch):
            mismatches.append(f"input {patch}")
        else:
            inputs += 1
    whole_new = build_analysis_bundle_id_view(bundle, frozenset(patches))
    whole_old = legacy_view(bundle, frozenset(patches))
    if view_parts(whole_new) != view_parts(whole_old):
        mismatches.append("view of every patch")
    print(
        json.dumps(
            {
                "domains": len(patches),
                "views": views,
                "inputs": inputs,
                "ring_faces": ring_faces,
                "selected_edges": len(selected),
                "mismatches": mismatches,
            }
        )
    )


main()
