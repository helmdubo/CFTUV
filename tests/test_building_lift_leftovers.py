"""Два остатка `building` (LEFTOVER A/B, 2026-10-03) на настоящем слепке сцены, маршрутом кнопки.

A. Патч 89 (ступенька 1.6 см, вершина `building:34` с веером 360.167°) раньше отказывал
`DEVELOPABLE_CHART_SELF_OVERLAP`. Третье предложение развёртки (запас угла, `_cone_relief`) даёт карту в
бюджете растяжения: домен EXACT. Материализацию патча 89 останавливал другой закон — нормаль смещения
вершины (`SOURCE_VERTEX_ANGLE_WEIGHTED_NORMAL_V1`): в углу `building:34` у треугольника-иголки `building:513`
(угол 0.17°) вес нулевой, нормаль вершины перпендикулярна его нормали с косинусом -3.1e-4. Допуск глубины
(`SURFACE_OFFSET_OPPOSITION_DEPTH_V1`, 0.1 мм на смещении 0.02 м; здесь глубина ~8 мкм) пропускает его с
записью: патч 89 строится, а худший треугольник и глубина названы счётчиком и диагностикой.

B. Патчи 1 и 11: вершина `src:` не вставала в позицию хоста (`SOURCE_VERTEX_LIFT_REFUSED_BY_FACE_ORIENTATION`),
потому что её хост-позиция поворачивала замыкающее ребро поперёк иголки шириной 6 мкм, и сварка с соседом
давала `ADAPTER_WELD_POSITION_MISMATCH` (2 вершины, 0.196 мм). Узлы иголки следуют за вершиной
(`SOURCE_VERTEX_LIFT_NODES_FOLLOWED`): вершина лежит в позиции хоста, перевёрнутых граней нет.

Прогон — `artifacts/materialize_sweep/sweep.py` в отдельном процессе (то же, что делает кнопка: снимок,
запрос, очередь, затем `materialize_domain` продуктовым законом топологии), около 15 секунд.
"""

from __future__ import annotations

import json
import os
import subprocess
import sys
from functools import lru_cache
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
SWEEP = ROOT / "artifacts" / "materialize_sweep" / "sweep.py"
PRODUCT_LAW = "PLANAR_POLYGONS_V1"
LIFT_REFUSED = "MATERIALIZE_SOURCE_VERTICES_LIFT_REFUSED_BY_FACE_ORIENTATION"
LIFTED = "MATERIALIZE_SOURCE_VERTICES_LIFTED_AT_HOST"
FOLLOWED = "MATERIALIZE_NODES_FOLLOWED_SOURCE_LIFT"
FLIPPED_BY_LIFT = "MATERIALIZE_TRIANGLES_FLIPPED_BY_SOURCE_LIFT"
FLIPPED_VS_SOURCE = "MATERIALIZE_TRIANGLES_FLIPPED_VS_SOURCE"
OPPOSITION_TOLERATED = "MATERIALIZE_OFFSET_OPPOSITIONS_TOLERATED"
OPPOSITION_WORST_DEPTH = "MATERIALIZE_OFFSET_OPPOSITION_WORST_DEPTH_NANOMETRES"


@lru_cache(maxsize=1)
def _rows(tmp_name: str = "building_leftovers_sweep.json") -> dict:
    import tempfile

    out = Path(tempfile.gettempdir()) / f"cftuv_{os.getpid()}_{tmp_name}"
    environment = dict(os.environ, PYTHONSAFEPATH="1")
    finished = subprocess.run(
        [
            sys.executable,
            str(SWEEP),
            "run",
            "--workers",
            "4",
            "--densities",
            "2",
            "--only",
            "1,11,89",
            "--topology",
            PRODUCT_LAW,
            "--out",
            str(out),
        ],
        capture_output=True,
        text=True,
        cwd=str(ROOT),
        env=environment,
        timeout=420,
    )
    if finished.returncode != 0:
        pytest.fail(f"SWEEP_DID_NOT_FINISH: {finished.returncode}\n{finished.stderr[-3000:]}")
    try:
        record = json.loads(out.read_text(encoding="utf-8"))
    finally:
        out.unlink(missing_ok=True)
    return record["runs"]["2"]["domains"]


@pytest.mark.parametrize("patch_id", ("1", "11"))
def test_a_thin_face_no_longer_costs_a_source_vertex_its_host_position(patch_id):
    row = _rows()[patch_id]
    counters, topology = row["counters"], row["topology_counters"]

    assert row["materialization"] == "MATERIALIZED"
    assert counters[LIFT_REFUSED] == 0
    assert counters[FOLLOWED] > 0
    assert counters[LIFTED] > 0
    assert counters[FLIPPED_VS_SOURCE] == 0
    assert topology.get(FLIPPED_BY_LIFT, 0) == 0
    named = [line for line in row["diagnostics"] if line.startswith("SOURCE_VERTEX_LIFT_NODES_FOLLOWED")]
    assert len(named) == 1 and "moved rigidly" in named[0]
    assert not any(line.startswith("SOURCE_VERTEX_LIFT_REFUSED") for line in row["diagnostics"])


def test_building_patch_89_builds_with_the_offset_opposition_tolerated_and_recorded():
    row = _rows()["89"]
    counters = row["counters"]

    assert row["prepare_outcome"] == "EXACT" and row["coverage_outcome"] == "EXACT"
    assert row["planarity"] == "DevelopableUnfoldCertificateV1"
    assert row["materialization"] == "MATERIALIZED"
    assert counters[OPPOSITION_TOLERATED] == 1
    # Глубина на опорном смещении 0.02 м: микроны, не больше допуска 0.1 мм (100000 нм).
    assert 0 < counters[OPPOSITION_WORST_DEPTH] <= 100_000
    assert counters[LIFT_REFUSED] == 0 and counters[FLIPPED_VS_SOURCE] == 0
    named = [line for line in row["diagnostics"] if line.startswith("SURFACE_OFFSET_OPPOSITION_TOLERATED")]
    assert len(named) == 1 and "building:513" in named[0] and "building:34" in named[0]


def test_the_opposition_reference_offset_is_the_host_default():
    """Допуск глубины судится на смещении, которое ядро не знает: опорное равно умолчанию хоста."""

    kernel_source = str(ROOT / "kernel" / "src")
    if kernel_source not in sys.path:
        sys.path.insert(0, kernel_source)
    from cftuv.envelope_production_mesh import DEFAULT_DECAL_OFFSET
    from cftuv_envelope.materialize.offset_normal import OFFSET_REFERENCE_METRES

    assert float(OFFSET_REFERENCE_METRES) == DEFAULT_DECAL_OFFSET
