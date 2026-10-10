"""Каждый отказ ступени метрики ядра выходит из хоста ПОД СВОИМ ИМЕНЕМ: полнота зеркала исходов исполняема.

Полевой случай `cover.008` (патчи 18 и 285): `_host_outcome_for` сводил восемнадцать имён ядра (`SOURCE_SNAP_*`, `NEAR_PLANAR_PROJECTION_*`,
`NEAR_PLANAR_REDUCED_FRAME_REQUIRES_SOURCE_SNAP`) к `ENVELOPE_DEBUG_PIPELINE_STAGE_FAILED`: поле читало «этап конвейера упал» там, где ядро
точно назвало причину. Зеркало закрыто двумя замками, которые не зависят от памяти автора:

1. ЗАМКНУТАЯ КЛАССИФИКАЦИЯ. Каждый член `NamedOutcome` либо зеркалируется хостом (то же имя), либо назван здесь с причиной, по которой он не
   отказ ступени метрики. Новый исход ядра без хост-имени и без записи здесь красный.
2. ЧТЕНИЕ ИСТОЧНИКА. Член, не зеркалируемый хостом, не вправе упоминаться (`NamedOutcome.X`) в модуле ядра вне разрешённых: иначе ядро начало бы
   бросать отказ, которого хост не называет. Снятый с производства член не упоминается нигде.
"""

from __future__ import annotations

import ast
import types
from pathlib import Path

import pytest

import cftuv_envelope as kernel
from cftuv.envelope_host_outcomes import (
    METRIC_STAGE_OUTCOMES,
    EnvelopeDebugHostOutcome,
    host_outcome_for,
    refusal_text,
)
from cftuv.envelope_request_export import EnvelopeHostAdapterError, _rational_affine_metric
from cftuv_envelope.outcomes import NamedOutcome

KERNEL_ROOT = Path(__file__).resolve().parents[1] / "kernel" / "src" / "cftuv_envelope"

#: Не отказ ступени метрики: причина, по которой хост имени не заводит. Ключ - имя ядра.
NOT_A_METRIC_REFUSAL = {
    # Объявлены ядром, нигде им не бросаются.
    "DECAL_ANALYSIS_SCHEMA_UNSUPPORTED": "UNREFERENCED",
    "BARRIER_SPLIT_REQUIRED": "UNREFERENCED",
    "BARRIER_BYPASS_UNSUPPORTED": "UNREFERENCED",
    "SHARED_ENVELOPE_MIXED_ALPHA_UNPROVEN": "UNREFERENCED",
    "PENDING_EXACT_EVALUATION": "UNREFERENCED",
    "APPROXIMATE_MATERIALIZATION_PENDING": "UNREFERENCED",
    "JUNCTION_ROUTE_PAIRING_REQUIRED": "UNREFERENCED",
    # Снят с производства (P0-4-HARDENING): исход, который нельзя выпустить ни на каком входе.
    "NEAR_PLANAR_PROJECTION_CYCLIC_ORDER_CHANGED": "RETIRED",
    # Другая ступень: проверка снапшота и счёт плотности, не метрика.
    "OWNERSHIP_PARTITION_UNPROVEN": "OTHER_STAGE",
    "ANGULAR_PROFILE_SELECTION_UNCERTAIN": "OTHER_STAGE",
    # Диагностики материализатора: идут в батч, домен они не отказывают.
    "FACE_BEYOND_CHART_REACH": "MATERIALIZER_DIAGNOSTIC",
    "PERIODIC_CUT_SEAM_RESIDUAL": "MATERIALIZER_DIAGNOSTIC",
    "CHART_REACH_TIGHTENED_FOR_SEAM": "MATERIALIZER_DIAGNOSTIC",
    "PERIODIC_CUT_BISECTOR_DEVIATION": "MATERIALIZER_DIAGNOSTIC",
    "NEAR_PLANAR_LIFT_ON_CERTIFIED_PLANE": "MATERIALIZER_DIAGNOSTIC",
    "NEAR_PLANAR_LIFT_ONTO_SOURCE_TRIANGLES": "MATERIALIZER_DIAGNOSTIC",
    "DEVELOPABLE_LIFT_ONTO_UNFOLDED_SOURCE_TRIANGLES": "MATERIALIZER_DIAGNOSTIC",
    "DEVELOPABLE_OFFSET_MIN_GAP_COSINE": "MATERIALIZER_DIAGNOSTIC",
    "SURFACE_OFFSET_OPPOSITION_TOLERATED": "MATERIALIZER_DIAGNOSTIC",
    "SOURCE_EDGES_LIFTED_ONTO_SURFACE": "MATERIALIZER_DIAGNOSTIC",
    "U_RESTARTS_AT_DOMAIN_BORDER": "MATERIALIZER_DIAGNOSTIC",
    "U_RESTARTS_AT_CLOSED_FLOW_OPENING": "MATERIALIZER_DIAGNOSTIC",
    "CORNER_JOIN_SAME_PCHAIN_V1": "MATERIALIZER_DIAGNOSTIC",
    "CORNER_MITER_ON_FOLD_V1": "MATERIALIZER_DIAGNOSTIC",
    "DEGRADED_MITER_CORNER_IN_GEOMETRY": "MATERIALIZER_DIAGNOSTIC",
    "SOURCE_VERTEX_LIFTED_AT_HOST_POSITION_V1": "MATERIALIZER_DIAGNOSTIC",
    "SOURCE_VERTEX_DISPLACED_BY_LATTICE": "MATERIALIZER_DIAGNOSTIC",
    "SOURCE_VERTEX_LIFT_REFUSED_BY_FACE_ORIENTATION": "MATERIALIZER_DIAGNOSTIC",
    "SOURCE_VERTEX_LIFT_NODES_FOLLOWED": "MATERIALIZER_DIAGNOSTIC",
    "SOURCE_VERTEX_STATIONED_ON_CHORD_V1": "MATERIALIZER_DIAGNOSTIC",
    "SOURCE_VERTEX_CHORD_STATION_SKIPPED": "MATERIALIZER_DIAGNOSTIC",
}

#: Модули ядра, где вправе упоминаться не зеркалируемый член (метрика и снап их не упоминают).
UNMIRRORED_MAY_BE_REFERENCED_IN = {"validation.py": "OTHER_STAGE", "_density_policy.py": "OTHER_STAGE"}

#: Отказы ступени метрики, которых хост не называл до среза COVER008-A; теперь каждый стоит в `METRIC_STAGE_OUTCOMES`.
MIRRORED_BY_THIS_SLICE = (
    "NEAR_PLANAR_REDUCED_FRAME_REQUIRES_SOURCE_SNAP",
    "SOURCE_SNAP_VERTEX_INJECTIVITY_VIOLATED",
    "SOURCE_SNAP_NONZERO_EDGE_COLLAPSED",
    "SOURCE_SNAP_NEW_NONADJACENT_EDGE_INTERSECTION",
    "SOURCE_SNAP_INTENDED_RIGHT_CORNER_DEGENERATED",
    "NEAR_PLANAR_PROJECTION_BOUNDARY_INJECTIVITY_VIOLATED",
    "NEAR_PLANAR_PROJECTION_NONZERO_BOUNDARY_EDGE_COLLAPSED",
    "NEAR_PLANAR_PROJECTION_NEW_NONADJACENT_EDGE_INTERSECTION",
    "NEAR_PLANAR_PROJECTION_NEW_COLLINEAR_EDGE_OVERLAP",
    "NEAR_PLANAR_PROJECTION_LOOP_ORIENTATION_CHANGED",
    "NEAR_PLANAR_PROJECTION_BOUNDARY_COMPONENT_COUNT_CHANGED",
    "NEAR_PLANAR_PROJECTION_OUTER_HOLE_NESTING_CHANGED",
    "NEAR_PLANAR_PROJECTION_SOURCE_ANCHOR_IDENTITY_CHANGED",
    "NEAR_PLANAR_PROJECTION_RESOLVED_PLANE_BASIS_UNAVAILABLE",
    "NEAR_PLANAR_PROJECTION_FAN_IDENTITY_CHANGED",
    "NEAR_PLANAR_PROJECTION_VERTEX_INJECTIVITY_VIOLATED",
    "NEAR_PLANAR_PROJECTION_FACE_POLYGON_NOT_SIMPLE",
    "NEAR_PLANAR_PROJECTION_INTERIOR_OVERLAP",
)


def _host_names() -> set[str]:
    return {item.value for item in EnvelopeDebugHostOutcome}


def test_every_kernel_outcome_is_mirrored_by_the_host_or_named_as_not_a_metric_refusal():
    unclassified = sorted(
        item.value for item in NamedOutcome if item.value not in _host_names() and item.value not in NOT_A_METRIC_REFUSAL
    )
    assert not unclassified, (
        f"исход ядра без имени хоста и без записи 'не отказ метрики': {unclassified}. Заведите член в EnvelopeDebugHostOutcome "
        "(и в METRIC_STAGE_OUTCOMES, если это отказ метрики) либо назовите причину в NOT_A_METRIC_REFUSAL."
    )
    assert set(NOT_A_METRIC_REFUSAL) <= {item.value for item in NamedOutcome}, "запись о несуществующем исходе ядра"
    assert not set(NOT_A_METRIC_REFUSAL) & _host_names(), "исход получил имя хоста, но остался в списке 'не отказ метрики'"


def test_every_metric_outcome_has_a_host_name_that_is_the_same_word():
    """Исход, который хост зеркалирует, возвращается из `host_outcome_for` ТЕМ ЖЕ словом, а не запасным."""

    for item in NamedOutcome:
        if item.value in NOT_A_METRIC_REFUSAL:
            continue
        host = host_outcome_for(item)
        assert host.value == item.value, item
        assert host is not EnvelopeDebugHostOutcome.ENVELOPE_DEBUG_PIPELINE_STAGE_FAILED, item
    for name in MIRRORED_BY_THIS_SLICE:
        assert EnvelopeDebugHostOutcome[name].value == name == NamedOutcome[name].value
        assert EnvelopeDebugHostOutcome[name] in METRIC_STAGE_OUTCOMES, name


def test_the_two_source_preflight_names_are_host_names_on_the_metric_stage_and_not_kernel_outcomes():
    for name in ("SOURCE_T_VERTEX", "SOURCE_FACE_SELF_INTERSECTION"):
        assert EnvelopeDebugHostOutcome[name] in METRIC_STAGE_OUTCOMES
        assert name not in {item.value for item in NamedOutcome}


def _references() -> dict[str, set[str]]:
    """`{член NamedOutcome: модули ядра, где он упомянут как NamedOutcome.X}` (кроме самого `outcomes.py`)."""

    found: dict[str, set[str]] = {}
    for path in sorted(KERNEL_ROOT.rglob("*.py")):
        relative = path.relative_to(KERNEL_ROOT).as_posix()
        if relative == "outcomes.py":
            continue
        for node in ast.walk(ast.parse(path.read_text(encoding="utf-8"))):
            if isinstance(node, ast.Attribute) and isinstance(node.value, ast.Name) and node.value.id == "NamedOutcome":
                found.setdefault(node.attr, set()).add(relative)
    return found


def test_an_unmirrored_outcome_is_not_raised_by_the_metric_stage_and_a_retired_one_by_nobody():
    references = _references()
    stray = sorted(
        (name, sorted(files))
        for name, files in references.items()
        if name not in _host_names()
        and name in NOT_A_METRIC_REFUSAL
        and NOT_A_METRIC_REFUSAL[name] != "MATERIALIZER_DIAGNOSTIC"
        and any(file not in UNMIRRORED_MAY_BE_REFERENCED_IN for file in files)
    )
    assert not stray, f"не зеркалируемый исход упомянут вне разрешённых модулей: {stray}"
    diagnostics_elsewhere = sorted(
        name
        for name, kind in NOT_A_METRIC_REFUSAL.items()
        if kind == "MATERIALIZER_DIAGNOSTIC"
        and any(not file.startswith("materialize/") for file in references.get(name, ()))
    )
    assert not diagnostics_elsewhere, f"диагностика материализатора упомянута вне materialize/: {diagnostics_elsewhere}"
    for name, kind in NOT_A_METRIC_REFUSAL.items():
        if kind in {"UNREFERENCED", "RETIRED"}:
            assert name not in references, f"{name} ({kind}) снова упоминается: {sorted(references[name])}"


def _metric_that_refuses_with(outcome):
    """`_rational_affine_metric` на ядре, чья метрика бросает отказ `outcome`: то, что хост делает с исходом, пришедшим из ядра."""

    def refuse(**_arguments):
        raise kernel.PlanarMetricAdmissionError(outcome, "probe text")

    stand_in = types.SimpleNamespace(
        build_rational_affine_planar_metric=refuse,
        PlanarMetricAdmissionError=kernel.PlanarMetricAdmissionError,
        PlanarityAdmissionLawV1=kernel.PlanarityAdmissionLawV1,
        GridSnappingLawV1=kernel.GridSnappingLawV1,
        LineageId=kernel.LineageId,
    )
    surface = types.SimpleNamespace(source_faces=(), surface_triangles=())
    with pytest.raises(EnvelopeHostAdapterError) as raised:
        _rational_affine_metric(
            stand_in,
            source_revision=kernel.SourceRevision("revision"),
            patch_domain_id=kernel.PatchDomainId("domain"),
            owner_patch_id=kernel.PatchId("patch"),
            source_vertices=(),
            surface_ir=surface,
            chains=((), ()),
            budget=None,
        )
    return raised.value


@pytest.mark.parametrize("name", MIRRORED_BY_THIS_SLICE)
def test_the_metric_export_carries_each_mirrored_outcome_under_its_own_name(name):
    error = _metric_that_refuses_with(NamedOutcome[name])
    assert error.outcome is EnvelopeDebugHostOutcome[name]
    assert str(error) == "probe text"
    assert error.patch_domain_id == "domain"


def test_an_outcome_without_a_host_name_keeps_its_kernel_name_in_the_text_instead_of_vanishing():
    error = _metric_that_refuses_with(NamedOutcome.BARRIER_SPLIT_REQUIRED)
    assert error.outcome is EnvelopeDebugHostOutcome.ENVELOPE_DEBUG_PIPELINE_STAGE_FAILED
    assert str(error) == "BARRIER_SPLIT_REQUIRED: probe text"
    assert refusal_text(EnvelopeDebugHostOutcome.GRID_WINDOW_CLOSED, NamedOutcome.GRID_WINDOW_CLOSED, "x") == "x"
