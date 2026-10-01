"""Выгрузка домена patch 17 `building` в малую ядерную фикстуру.

Запуск из корня репозитория:
  python artifacts/faces_chain_refusal/export_fixture17.py

Снимок и запросы строятся ровно тем маршрутом, что и кнопка (`run17.py`):
`build_envelope_analysis_snapshot` по ОДНОМУ патчу и `build_envelope_decal_request`
с `density=d`, alpha 0.45. Снимок от плотности не зависит и один; запросов три —
плотности 2, 3, 4, на которых домен отказывал `FACE_CHAIN_DOES_NOT_CLOSE` до
ветки `crowded` (плотности 0 и 1 дают EXACT и до неё).
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "perf_prepare_diag"))

import env  # noqa: E402,F401

FIXTURE = Path(__file__).resolve().parents[2] / "kernel" / "fixtures" / "building_patch17_crowded_v1"
PATCH_ID = 17
DENSITIES = (2, 3, 4)


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
            0.45,
            decal_request_id_value=request_id,
            density=density,
        )
        (FIXTURE / f"decal_request_density{density}.json").write_bytes(
            DecalRequestCodecV1.dumps(request)
        )
    manifest = {
        "fixture_id": "building_patch17_crowded_v1",
        "object_name": "building",
        "patch_id": PATCH_ID,
        "alpha": "0.45",
        "densities": list(DENSITIES),
        "patch_domain_ids": [domain_id],
        "why": (
            "острый выпуклый угол внешней петли рядом с дырой, зажатой в клин: "
            "две дуги между парой рёбер, ветка crowded сборщика граней"
        ),
    }
    (FIXTURE / "manifest.json").write_text(
        json.dumps(manifest, ensure_ascii=False, indent=1), encoding="utf-8"
    )
    for item in sorted(FIXTURE.iterdir()):
        print(item.name, item.stat().st_size)


if __name__ == "__main__":
    main()
