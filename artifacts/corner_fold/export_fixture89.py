"""Выгрузка домена patch 89 `building` в малую ядерную фикстуру закона МИТРЫ НА ИЗЛОМЕ (`CORNER_MITER_ON_FOLD_V1`).

Запуск из корня репозитория:
  python artifacts/corner_fold/export_fixture89.py

Снимок и запросы строятся ровно тем маршрутом, что и кнопка (`artifacts/faces_chain_refusal/export_fixture17.py`):
`build_envelope_analysis_snapshot` по ОДНОМУ патчу и `build_envelope_decal_request` с `density=d`, alpha 0.25.
Снимок от плотности не зависит и один; запросов три — плотности 1, 2, 4 (те же, что у ворот).
Патч 89 — домен-развёртка: у вершин 40 и 46 кольцо-1 сложено на 1.86 градуса, у вершины 121 лежит треугольник
шириной 0.02 м под прямым углом к соседям, вершина 34 — изгиб около 178 градусов (веер).
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "perf_prepare_diag"))

import env  # noqa: E402,F401

FIXTURE = Path(__file__).resolve().parents[2] / "kernel" / "fixtures" / "building_patch89_fold_miter_v1"
PATCH_ID = 89
DENSITIES = (1, 2, 4)
ALPHA = 0.25


def main():
    import big_scene
    from cftuv.envelope_request_export import (
        _typed_value,
        build_envelope_analysis_snapshot,
        build_envelope_decal_request,
    )
    from cftuv.envelope_topology_export import stage_domain_inputs
    from cftuv_envelope import AnalysisSnapshotCodecV1, DecalRequestCodecV1

    _, bundle, selected, _ = big_scene.survey()
    _, revision, _, request_id, by_domain = stage_domain_inputs(bundle, selected)
    domain_id = _typed_value("patch-domain", revision, PATCH_ID)
    snapshot = build_envelope_analysis_snapshot(
        bundle, included_patch_ids=frozenset({PATCH_ID})
    )
    FIXTURE.mkdir(parents=True, exist_ok=True)
    (FIXTURE / "analysis_snapshot.json").write_bytes(AnalysisSnapshotCodecV1.dumps(snapshot))
    for density in DENSITIES:
        request = build_envelope_decal_request(
            snapshot,
            frozenset(by_domain[domain_id]),
            ALPHA,
            decal_request_id_value=request_id,
            density=density,
        )
        (FIXTURE / f"decal_request_density{density}.json").write_bytes(
            DecalRequestCodecV1.dumps(request)
        )
    manifest = {
        "fixture_id": "building_patch89_fold_miter_v1",
        "object_name": "building",
        "patch_id": PATCH_ID,
        "alpha": str(ALPHA),
        "densities": list(DENSITIES),
        "patch_domain_ids": [domain_id],
        "why": (
            "домен-развёртка со сложенной окрестностью вогнутых углов: вершины 40 и 46 (кольцо-1 сложено на 1.86 градуса, "
            "изгиб 35.9 и 90 градусов) и вершина 121 (треугольник под прямым углом к соседям, изгиб 89.7) берут митру "
            "со швом, вершина 34 (изгиб около 178 градусов) остаётся веером"
        ),
    }
    (FIXTURE / "manifest.json").write_text(
        json.dumps(manifest, ensure_ascii=False, indent=1), encoding="utf-8"
    )
    for item in sorted(FIXTURE.iterdir()):
        print(item.name, item.stat().st_size)


if __name__ == "__main__":
    main()
