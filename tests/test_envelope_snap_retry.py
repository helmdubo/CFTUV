"""Повторная попытка домена под законом масштаба, сохраняющим плоскость патча (`SOURCE_SNAP_PLANE_PRESERVED_RETRY_V1`).

Что держат тесты: повтор идёт ТОЛЬКО после отказа из замкнутого множества и только для него; построенный домен не получает повтор
и остаётся тем же объектом; исход повтора записан вместе с именем первоначального отказа; повторный отказ называется своим исходом, а
первоначальный остаётся причиной; закон входит в ключи кэшей метрики, геометрии, подготовки и результата, а умолчание не меняет ни одного ключа.
Настоящий прогон (`cover.008`, 5 доменов) - полевая приёмка `tools/blender_field_case_cover008.py`.
"""

from __future__ import annotations

import ast
import dataclasses
import inspect
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest

ROOT = Path(__file__).resolve().parents[1]
KERNEL_SRC = ROOT / "kernel" / "src"
if str(KERNEL_SRC) not in sys.path:
    sys.path.insert(0, str(KERNEL_SRC))

from cftuv import envelope_debug_session as session  # noqa: E402
from cftuv import envelope_snap_retry as retry  # noqa: E402
from cftuv.envelope_debug_profile import EnvelopeDebugProfileBuilderV1  # noqa: E402
from cftuv.envelope_production_export import MATERIALIZED, ProductionDomainResultV1, _RunInputsV1  # noqa: E402
from cftuv.envelope_topology_export import (  # noqa: E402
    GRID_SCALE_LAW_PLANE_PRESERVING,
    EnvelopeTopologyExportV1,
    metric_law_key,
)

LOTTERY = {
    "COVERAGE_IS_NOT_EXACT": [
        "preparation:PLAN_IS_NOT_COMPILED: DENSITY_RATIONAL_AUTHORITY_EXHAUSTED",
        "preparation:DOMAIN_GEOMETRY_REFUSED: PLANAR_OWNER_INTERIOR_DIRECTION_REQUIRED: ordered support normals do not realize the certified owner-sector turn",
    ],
    "SURFACE_OFFSET_NORMAL_OPPOSES_TRIANGLE": ["the offset normal of vertex v opposes the normal of the triangle"],
}
#: Остальные отказы `cover.008`: не лотерея привязки, повтор их не лечит и не получает.
FOREIGN = (
    ("COVERAGE_IS_NOT_EXACT", "preparation:DOMAIN_GEOMETRY_REFUSED: REFERENCE_CERTIFIED_PREDICATE_UNDECIDABLE: direction binding certificate is not proven"),
    ("COVERAGE_IS_NOT_EXACT", "preparation:BRIDGE_DID_NOT_MAP: sparse-patch-domain-region:x: SOURCE_EDGE_INSIDE_A_LINE_CLASS_IS_UNDETERMINED"),
    ("NO_GRID_SCALE_RESTORES_RELATIONS", "ни один масштаб окна не восстановил задуманные отношения"),
    ("ENVELOPE_DEBUG_PIPELINE_STAGE_FAILED", "source snap did not preserve the embedding: SourceSnapEmbeddingCertificateV1(...)"),
    ("COVERAGE_IS_NOT_EXACT", "preparation:PLAN_IS_NOT_COMPILED: DENSITY_RATIONAL_AUTHORITY_EXHAUSTED_FOR_ANOTHER_REASON"),
)


def _result(patch, outcome, detail="", diagnostics=()):
    built = outcome == MATERIALIZED
    return ProductionDomainResultV1(
        patch_id=patch, domain_id=f"domain{patch}", outcome=outcome, batch=object() if built else None, detail=detail, diagnostics=tuple(diagnostics)
    )


def test_the_closed_set_names_exactly_the_three_observed_refusals():
    assert retry.SNAP_LOTTERY_REFUSALS == {
        "DENSITY_RATIONAL_AUTHORITY_EXHAUSTED",
        "PLANAR_OWNER_INTERIOR_DIRECTION_REQUIRED",
        "SURFACE_OFFSET_NORMAL_OPPOSES_TRIANGLE",
    }
    assert isinstance(retry.SNAP_LOTTERY_REFUSALS, frozenset)


@pytest.mark.parametrize("outcome,detail", [(outcome, detail) for outcome, details in LOTTERY.items() for detail in details])
def test_a_lottery_refusal_is_found_in_the_outcome_or_in_the_named_reason(outcome, detail):
    name = retry.lottery_refusal(_result(1, outcome, detail))

    assert name in retry.SNAP_LOTTERY_REFUSALS


@pytest.mark.parametrize("outcome,detail", FOREIGN)
def test_a_foreign_refusal_gets_no_retry(outcome, detail):
    assert retry.lottery_refusal(_result(1, outcome, detail)) is None


def test_a_built_domain_never_gets_a_retry_even_when_its_text_names_a_lottery_refusal():
    built = _result(1, MATERIALIZED, "preparation:PLAN_IS_NOT_COMPILED: DENSITY_RATIONAL_AUTHORITY_EXHAUSTED")

    assert retry.lottery_refusal(built) is None


def test_the_reason_text_is_read_in_the_format_the_kernel_writes_it():
    """Связь с ядром: текст отказа подготовки строит `admit_domain`; формат не может уйти, не уронив этот тест."""

    from cftuv_envelope.materialize.admit import admit_domain

    for outcome, detail in (("PLAN_IS_NOT_COMPILED", "DENSITY_RATIONAL_AUTHORITY_EXHAUSTED"), ("DOMAIN_GEOMETRY_REFUSED", "PLANAR_OWNER_INTERIOR_DIRECTION_REQUIRED: ordered support normals")):
        prepared = SimpleNamespace(outcome=SimpleNamespace(value=outcome), detail=detail)
        refused = admit_domain(prepared, None, None)
        result = _result(1, refused.outcome.value, refused.detail)

        assert refused.outcome.value == "COVERAGE_IS_NOT_EXACT"
        assert retry.lottery_refusal(result) == detail.split(":")[0]


def test_the_law_string_of_the_host_is_the_kernel_law():
    from cftuv_envelope.contracts.metric import GridScaleLawV1

    assert GRID_SCALE_LAW_PLANE_PRESERVING == GridScaleLawV1.PLANE_PRESERVING_V1.value


# --------------------------------------------------------------------------
# Запись исхода
# --------------------------------------------------------------------------


def test_a_retry_that_builds_the_domain_records_the_outcome_and_the_original_refusal():
    original = _result(7, "COVERAGE_IS_NOT_EXACT", LOTTERY["COVERAGE_IS_NOT_EXACT"][0])
    built = _result(7, MATERIALIZED, "", diagnostics=("SOME_DIAGNOSTIC: x",))

    merged = retry.merged_result(original, built, "DENSITY_RATIONAL_AUTHORITY_EXHAUSTED")

    assert merged.outcome == MATERIALIZED and merged.is_materialized
    first = merged.diagnostics[0]
    assert first.startswith(retry.SNAP_RETRY_OUTCOME + ":") and "DENSITY_RATIONAL_AUTHORITY_EXHAUSTED" in first and "COVERAGE_IS_NOT_EXACT" in first
    assert merged.diagnostics[1:] == ("SOME_DIAGNOSTIC: x",)


def test_a_retry_that_refuses_again_reports_its_own_refusal_and_keeps_the_original_as_the_cause():
    original = _result(7, "COVERAGE_IS_NOT_EXACT", LOTTERY["COVERAGE_IS_NOT_EXACT"][0])
    again = _result(7, "SURFACE_OFFSET_NORMAL_OPPOSES_TRIANGLE", "the offset normal opposes")

    merged = retry.merged_result(original, again, "DENSITY_RATIONAL_AUTHORITY_EXHAUSTED")

    assert merged.outcome == "SURFACE_OFFSET_NORMAL_OPPOSES_TRIANGLE" and not merged.is_materialized
    assert "the offset normal opposes" in merged.detail
    assert "cause DENSITY_RATIONAL_AUTHORITY_EXHAUSTED" in merged.detail and original.detail in merged.detail
    assert any(line.startswith(retry.SNAP_RETRY_OUTCOME) for line in merged.diagnostics)


def test_a_retry_that_raised_leaves_the_original_refusal_standing_with_the_exception_in_its_reason():
    original = _result(7, "COVERAGE_IS_NOT_EXACT", LOTTERY["COVERAGE_IS_NOT_EXACT"][0])
    raised = _result(7, "PRODUCTION_DOMAIN_RAISED", "ValueError: boom")

    merged = retry.merged_result(original, raised, "DENSITY_RATIONAL_AUTHORITY_EXHAUSTED")

    assert merged.outcome == "COVERAGE_IS_NOT_EXACT" and "ValueError: boom" in merged.detail


# --------------------------------------------------------------------------
# Повтор по прогону
# --------------------------------------------------------------------------


def _run(controller, export):
    return _RunInputsV1(
        controller=controller,
        analysis_bundle=None,
        topology_export=export,
        revision="rev",
        patch_ids=(1, 2, 3, 4),
        selected_by_domain={},
        alpha=0.25,
        alpha_text="0.25",
        request_id="request",
        density=2,
        uv_policy_id="uv",
        topology_law="law",
        hooks=None,
        profile=EnvelopeDebugProfileBuilderV1("source", "PRODUCTION"),
    )


class _Controller:
    def __init__(self):
        self.hooked = []

    def worker_export_hooks(self, export, profile):
        self.hooked.append(export)
        return ("hooks", export)


def test_the_retry_reruns_only_the_lottery_domains_on_the_marked_export_and_leaves_the_rest_untouched():
    export = EnvelopeTopologyExportV1("rev", None, (), {1: "domain1", 2: "domain2", 3: "domain3", 4: "domain4"})
    controller = _Controller()
    run = _run(controller, export)
    built, lottery, foreign, second = (
        _result(1, MATERIALIZED),
        _result(2, "COVERAGE_IS_NOT_EXACT", LOTTERY["COVERAGE_IS_NOT_EXACT"][0]),
        _result(3, "NO_GRID_SCALE_RESTORES_RELATIONS", "none"),
        _result(4, "SURFACE_OFFSET_NORMAL_OPPOSES_TRIANGLE", "opposes"),
    )
    entries = [SimpleNamespace(patch_id=index, needs_work=True, prepared=None, carried=None) for index in (1, 2, 3, 4)]
    seen = {}

    def scan(retried_run):
        seen["export"], seen["patches"], seen["hooks"] = retried_run.topology_export, retried_run.patch_ids, retried_run.hooks
        seen["lists"] = (retried_run.relabeled, retried_run.registered)
        return [SimpleNamespace(patch_id=patch, needs_work=True, prepared=None, carried=None) for patch in retried_run.patch_ids]

    def dispatch(retried_run, ready, cold, pool):
        seen["cold"] = [item.patch_id for item in cold]
        seen["pool"] = pool
        return {"domain2": _result(2, MATERIALIZED), "domain4": _result(4, "SURFACE_OFFSET_NORMAL_OPPOSES_TRIANGLE", "still opposes")}, {}

    def collect(retry_entries, done, refused):
        return [done[f"domain{entry.patch_id}"] for entry in retry_entries]

    results = retry.retry_snap_lottery(run, entries, [built, lottery, foreign, second], "POOL", scan=scan, dispatch=dispatch, collect=collect)

    assert seen["patches"] == (2, 4) and seen["cold"] == [2, 4] and seen["pool"] == "POOL"
    assert seen["export"].grid_scale_law == GRID_SCALE_LAW_PLANE_PRESERVING and export.grid_scale_law is None
    assert controller.hooked == [seen["export"]] and seen["hooks"] == ("hooks", seen["export"])
    assert seen["lists"][0] is not run.relabeled and seen["lists"][1] is not run.registered
    assert results[0] is built and results[2] is foreign  # построенный и чужой отказ - те же объекты
    assert results[1].is_materialized and results[1].diagnostics[0].startswith(retry.SNAP_RETRY_OUTCOME)
    assert results[3].outcome == "SURFACE_OFFSET_NORMAL_OPPOSES_TRIANGLE" and "cause SURFACE_OFFSET_NORMAL_OPPOSES_TRIANGLE" in results[3].detail
    counters = {item.name: item.value for item in run.profile.snapshot().counters if item.patch_domain_id is None}
    assert (counters[retry.SNAP_RETRY_ATTEMPTED], counters[retry.SNAP_RETRY_RECOVERED], counters[retry.SNAP_RETRY_REFUSED_AGAIN]) == (2, 1, 1)


def test_a_run_without_a_lottery_refusal_does_no_work_and_returns_the_same_results():
    run = _run(_Controller(), EnvelopeTopologyExportV1("rev", None, (), {1: "domain1"}))
    results = [_result(1, MATERIALIZED), _result(2, "NO_GRID_SCALE_RESTORES_RELATIONS", "none")]

    def forbidden(*_args, **_kwargs):
        raise AssertionError("the retry must not scan or dispatch")

    entries = [SimpleNamespace(patch_id=1), SimpleNamespace(patch_id=2)]
    assert retry.retry_snap_lottery(run, entries, results, None, scan=forbidden, dispatch=forbidden, collect=forbidden) is results


# --------------------------------------------------------------------------
# Ключи кэшей
# --------------------------------------------------------------------------


def _request():
    from decimal import Decimal

    import cftuv_envelope as kernel
    from cftuv.envelope_request_policy import build_envelope_request_contract, envelope_angular_policy

    return build_envelope_request_contract(
        kernel, kernel.DecalRequestId("request"), frozenset(), Decimal("0.25"), envelope_angular_policy(kernel, None, None, None)
    )


def test_the_default_law_changes_no_key_and_the_retry_law_changes_every_one():
    controller = session.EnvelopeDebugSessionController()
    request = _request()
    args = ("rev", "domain0", frozenset({2}), request)

    default_key = controller._preparation_key(*args, "PYTHON")
    retried_key = controller._preparation_key(*args, "PYTHON", GRID_SCALE_LAW_PLANE_PRESERVING)

    assert len(default_key) == 5 and default_key == controller._preparation_key(*args, "PYTHON", None)
    assert retried_key[:5] == default_key and retried_key != default_key
    assert controller.production_result_key(*args, "0.25", skeleton_id="PYTHON") == (default_key, "0.25")
    assert controller.production_result_key(*args, "0.25", skeleton_id="PYTHON", grid_scale_law=GRID_SCALE_LAW_PLANE_PRESERVING) == (retried_key, "0.25")
    export = EnvelopeTopologyExportV1("rev", None, (), {0: "domain0"})
    assert metric_law_key(export) == () and metric_law_key(export.with_grid_scale_retry()) == (("grid_scale_law", GRID_SCALE_LAW_PLANE_PRESERVING),)


def test_a_preparation_of_the_retry_never_lands_under_the_key_of_the_ordinary_one_and_back():
    controller = session.EnvelopeDebugSessionController()
    request = _request()
    args = ("rev", "domain0", frozenset({2}), request)
    ordinary, retried = object(), object()

    assert controller.get_conveyor_preparation(*args, lambda: ordinary, skeleton_id="PYTHON") is ordinary
    assert controller.peek_conveyor_preparation(*args, skeleton_id="PYTHON", grid_scale_law=GRID_SCALE_LAW_PLANE_PRESERVING) is None
    assert controller.get_conveyor_preparation(*args, lambda: retried, skeleton_id="PYTHON", grid_scale_law=GRID_SCALE_LAW_PLANE_PRESERVING) is retried
    assert controller.peek_conveyor_preparation(*args, skeleton_id="PYTHON") is ordinary
    assert controller.peek_conveyor_preparation(*args, skeleton_id="PYTHON", grid_scale_law=GRID_SCALE_LAW_PLANE_PRESERVING) is retried


def test_the_metric_and_the_geometry_of_the_retry_are_separate_cache_records():
    controller = session.EnvelopeDebugSessionController()
    export = EnvelopeTopologyExportV1("rev", None, (), {0: "domain0"})
    marked = export.with_grid_scale_retry()

    def metric(law):
        return SimpleNamespace(
            source_revision_value="rev", patch_id=0, patch_domain_id="domain0", snapshot=object(), developable_stretch_budget=None, band_key=None, grid_scale_law=law
        )

    ordinary = controller.get_patch_metric(export, 0, build=lambda: metric(None))
    assert not controller.has_patch_metric(marked, 0) and controller.has_patch_metric(export, 0)
    retried = controller.get_patch_metric(marked, 0, build=lambda: metric(GRID_SCALE_LAW_PLANE_PRESERVING))
    assert retried is not ordinary and controller.has_patch_metric(marked, 0)
    assert controller.get_patch_metric(export, 0) is ordinary and controller.get_patch_metric(marked, 0) is retried
    assert controller.get_domain_geometry(ordinary) is not controller.get_domain_geometry(retried)
    assert controller.get_domain_geometry(retried).grid_scale_law == GRID_SCALE_LAW_PLANE_PRESERVING


def test_the_binding_and_the_scan_key_of_the_run_carry_the_law():
    from cftuv.envelope_production_export import _binding
    from cftuv.envelope_scan_memo import scan_key

    export = EnvelopeTopologyExportV1("rev", None, (), {1: "domain1"})
    ordinary = _run(_Controller(), export)
    marked = dataclasses.replace(ordinary, topology_export=export.with_grid_scale_retry())

    assert ordinary.grid_scale_law is None and marked.grid_scale_law == GRID_SCALE_LAW_PLANE_PRESERVING
    assert scan_key(ordinary) != scan_key(marked)
    assert _binding(ordinary, 1, "domain1", frozenset({2})) != _binding(marked, 1, "domain1", frozenset({2}))


# --------------------------------------------------------------------------
# Стены
# --------------------------------------------------------------------------


def test_only_the_retry_module_orders_the_plane_preserving_law():
    """Закон не становится умолчанием по недосмотру: экспорт с законом строит только повтор (и тесты)."""

    callers = []
    for path in sorted((ROOT / "cftuv").glob("*.py")):
        for node in ast.walk(ast.parse(path.read_text(encoding="utf-8"), filename=str(path))):
            if isinstance(node, ast.Attribute) and node.attr == "with_grid_scale_retry":
                callers.append(path.name)
            if isinstance(node, ast.Constant) and node.value == GRID_SCALE_LAW_PLANE_PRESERVING and path.name != "envelope_topology_export.py":
                callers.append(path.name + ":" + str(node.value))

    assert callers == ["envelope_snap_retry.py"], callers


def test_the_kernel_default_of_every_builder_is_the_first_law():
    from cftuv_envelope import planar_metric, source_grid
    from cftuv_envelope.contracts.metric import GridScaleLawV1

    for function in (
        planar_metric.build_rational_affine_planar_metric,
        planar_metric.build_embedding_certified_rational_affine_planar_metric,
        planar_metric._build_planar_family_metric,
        planar_metric._developable_rung,
        source_grid.resolve_source_grid,
        source_grid.select_grid_scale,
    ):
        assert inspect.signature(function).parameters[
            "scale_law" if function in (source_grid.resolve_source_grid, source_grid.select_grid_scale) else "grid_scale_law"
        ].default is GridScaleLawV1.FIRST_ANGLE_RESTORING_V1, function.__name__
