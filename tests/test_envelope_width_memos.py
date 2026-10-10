"""Память шага ширины на хосте: пролог прогона и записи сборки входов не меняют ответа (хост без Blender).

Утверждений пять, и каждое стоит на сверке «с памятью и без»:

1. ОТВЕТ ТОТ ЖЕ. Серия ширин (вверх, вниз, повтор, недопустимые) на двух сессиях, с памятью и без: результаты доменов
   (батчи, исходы, счётчики, дайджесты) и записи профиля прогона (счётчики, квитанции доменов) равны; различаются только
   счётчики самой памяти.
2. ПАМЯТЬ РАБОТАЕТ. После первого нажатия пролог берётся из памяти, а вход каждого домена - из записи сборки, с запросом,
   равным собранному заново.
3. КЛЮЧ ПОЛОН. Другое выделение, другой пакет анализа и другая ревизия пролога не принимают записей; память ограничена.
4. НЕДОПУСТИМАЯ ШИРИНА идёт прежним путём (запись не отдаётся), а домен с полосовой картой записи не получает.
5. ПРОЛОГ ПИШЕТ В ПРОФИЛЬ ТО ЖЕ: попадание повторяет счётчики и квитанции, которые записал счёт.
"""

from __future__ import annotations

import sys
from decimal import Decimal
from pathlib import Path

import pytest

KERNEL_SRC = Path(__file__).resolve().parents[1] / "kernel" / "src"
if str(KERNEL_SRC) not in sys.path:
    sys.path.insert(0, str(KERNEL_SRC))

from cftuv import envelope_production_export as production  # noqa: E402
from cftuv import envelope_scan_memo as scan_memo  # noqa: E402
from cftuv.envelope_debug_profile import EnvelopeDebugProfileBuilderV1  # noqa: E402
from cftuv.envelope_debug_session import EnvelopeDebugSessionController  # noqa: E402
from cftuv.envelope_domain_pool import shutdown_domain_pool  # noqa: E402
from cftuv.envelope_production_export import run_production  # noqa: E402
from cftuv.envelope_topology_export import (  # noqa: E402
    STAGE_INPUTS_MEMO_LIMIT,
    StageInputsMemoV1,
    stage_domain_inputs,
)
from envelope_fixture_bundles import quad_row_bundle  # noqa: E402

ROW = 5
WIDTHS = (0.25, 0.26, 0.24, 0.25, 0.3, 0.2, 0.26, -1.0, float("nan"), 0.27)
MEMO_COUNTERS = {production.PRODUCTION_STAGE_INPUTS_MEMO_HIT, production.PRODUCTION_SCAN_RECORDS_REUSED}


@pytest.fixture(scope="module", autouse=True)
def _no_pool_outlives_the_module():
    yield
    shutdown_domain_pool()


def session(*, memo: bool) -> EnvelopeDebugSessionController:
    controller = EnvelopeDebugSessionController()
    controller.stage_inputs_memo.enabled = memo
    controller.scan_memo.enabled = memo
    return controller


def press(bundle, controller, alpha, selected=frozenset(range(ROW))):
    return run_production(
        controller,
        bundle,
        selected,
        alpha,
        source_object_key="object",
        source_data_key="mesh",
        density=None,
        workers=0,
    )


def facts(run):
    """Записи профиля прогона без секунд и без счётчиков самой памяти."""

    profile = run.profile
    counters = tuple(
        (item.name, item.patch_domain_id, item.value)
        for item in profile.counters
        if item.name not in MEMO_COUNTERS and "PRODUCTION_CLIP_MEMO" not in item.name
    )
    return counters, tuple(profile.receipts)


def answer(run):
    return tuple(run.results)


def test_a_series_of_widths_gives_the_same_answer_with_and_without_the_memos():
    bundle = quad_row_bundle(ROW)
    with_memo, without = session(memo=True), session(memo=False)
    for width in WIDTHS:
        left, right = press(bundle, with_memo, width), press(bundle, without, width)
        assert answer(left) == answer(right), width
        assert facts(left) == facts(right), width
        assert [item.outcome for item in left.results] == [item.outcome for item in right.results]
    assert with_memo.stage_inputs_memo.hits > 0 and with_memo.stage_inputs_memo.misses == 1
    assert len(with_memo.scan_memo) == ROW
    assert len(without.scan_memo) == 0 and len(without.stage_inputs_memo) == 0


def test_after_the_first_press_the_prologue_and_every_domain_input_come_from_the_memos():
    bundle = quad_row_bundle(ROW)
    controller = session(memo=True)
    first = press(bundle, controller, 0.25)
    assert first.counter(production.PRODUCTION_STAGE_INPUTS_MEMO_HIT) == 0
    assert first.counter(production.PRODUCTION_SCAN_RECORDS_REUSED) == 0
    # Первое нажатие холодное: метрик в кэше нет, домены идут воркеру выгрузкой, а вход (снапшот с запросом) родитель строит в `_adopt_cold`
    # и ЗАПИСЫВАЕТ там же: первый шаг ширины после кнопки берёт его из записи, а не собирает снова.
    second = press(bundle, controller, 0.26)
    assert second.counter(production.PRODUCTION_STAGE_INPUTS_MEMO_HIT) == 1
    assert second.counter(production.PRODUCTION_SCAN_RECORDS_REUSED) == ROW, "the cold press recorded the inputs it built"
    third = press(bundle, controller, 0.27)
    assert third.counter(production.PRODUCTION_STAGE_INPUTS_MEMO_HIT) == 1
    assert third.counter(production.PRODUCTION_SCAN_RECORDS_REUSED) == ROW
    assert third.counter(production.PRODUCTION_PREPARATION_BUILDS) == 0
    assert [item.is_materialized for item in third.results] == [True] * ROW


def test_the_request_of_a_record_equals_the_request_built_anew(monkeypatch):
    from cftuv import envelope_request_export as export

    captured = []
    real = export.build_envelope_decal_request

    def spy(snapshot, selected, alpha, **kwargs):
        request = real(snapshot, selected, alpha, **kwargs)
        captured.append((snapshot, selected, kwargs, request))
        return request

    monkeypatch.setattr(export, "build_envelope_decal_request", spy)
    bundle = quad_row_bundle(ROW)
    controller = session(memo=True)
    press(bundle, controller, 0.25)  # холодное: входы строит `_adopt_cold` и записывает их
    assert len(captured) == ROW
    (records,) = controller.scan_memo._runs.values()
    assert len(records) == ROW
    by_snapshot = {id(item[0]): item for item in captured}
    for record in records.values():
        snapshot, selected, kwargs, built = by_snapshot[id(record.snapshot)]
        assert record.request == built
        for width in (0.26, 0.2, 0.2501):
            alpha = scan_memo.request_alpha(width)
            assert alpha == Decimal(str(float(width)))
            assert scan_memo.request_at(record, alpha) == real(snapshot, selected, width, **kwargs)
        # Ключ подготовки записи — тот же, что строит контроллер по запросу.
        key = EnvelopeDebugSessionController._preparation_key
        assert record.prep_key == key(record.prep_key[0], record.prep_key[1], record.selected, built)
        assert record.prep_key == key(record.prep_key[0], record.prep_key[1], record.selected, scan_memo.request_at(record, Decimal("0.4")))
    captured.clear()
    first_step = press(bundle, controller, 0.26)
    assert not captured, "the first width step after the cold press builds no request: every input comes from its record"
    assert first_step.counter(production.PRODUCTION_SCAN_RECORDS_REUSED) == ROW


def test_an_invalid_width_never_gets_a_record_and_takes_the_old_path():
    assert scan_memo.request_alpha(-0.1) is None
    assert scan_memo.request_alpha(float("nan")) is None
    assert scan_memo.request_alpha(float("inf")) is None
    assert scan_memo.request_alpha(0.0) == Decimal("0.0")
    bundle = quad_row_bundle(ROW)
    with_memo, without = session(memo=True), session(memo=False)
    press(bundle, with_memo, 0.25)
    bad = press(bundle, with_memo, -1.0)
    same = press(bundle, without, -1.0)
    assert answer(bad) == answer(same)
    assert bad.counter(production.PRODUCTION_SCAN_RECORDS_REUSED) == 0
    assert not any(item.is_materialized for item in bad.results)


def test_a_snapshot_with_a_band_chart_is_never_recorded():
    class Descriptor:
        def __init__(self, certificate):
            self.planarity_certificate = certificate

    class Snapshot:
        def __init__(self, *certificates):
            self.surface_metric_descriptors = tuple(Descriptor(item) for item in certificates)

    from cftuv.envelope_chart_band import band_certificate_type

    assert not scan_memo.carries_band(Snapshot(object(), None))
    assert scan_memo.carries_band(Snapshot(object(), object.__new__(band_certificate_type())))


def test_the_prologue_memo_is_keyed_by_the_selection_and_the_very_same_bundle_and_is_bounded():
    bundle = quad_row_bundle(ROW)
    controller = session(memo=True)
    export = controller.get_topology_export(bundle, "object", "mesh")
    memo = StageInputsMemoV1()
    first = stage_domain_inputs(bundle, frozenset(range(ROW)), topology_export=export, memo=memo)
    assert (memo.misses, memo.hits, memo.last) == (1, 0, "MISS")
    again = stage_domain_inputs(bundle, frozenset(range(ROW)), topology_export=export, memo=memo)
    assert (memo.misses, memo.hits, memo.last) == (1, 1, "HIT")
    assert again[:4] == first[:4] and again[4] == first[4]
    assert again[4] is not first[4], "the per-domain selection is handed out as a copy"
    again[4][next(iter(again[4]))].add(10**6)
    assert stage_domain_inputs(bundle, frozenset(range(ROW)), topology_export=export, memo=memo)[4] == first[4]
    stage_domain_inputs(bundle, frozenset(range(ROW - 1)), topology_export=export, memo=memo)
    assert memo.last == "MISS", "another selection is another prologue"
    other = quad_row_bundle(ROW)
    other_export = session(memo=True).get_topology_export(other, "object", "mesh")
    stage_domain_inputs(other, frozenset(range(ROW)), topology_export=other_export, memo=memo)
    assert memo.last == "MISS", "a rebuilt bundle of the same revision does not borrow the record of another object"
    for number in range(2, 2 + STAGE_INPUTS_MEMO_LIMIT + 2):
        stage_domain_inputs(bundle, frozenset(range(number)), topology_export=export, memo=memo)
    assert len(memo) == STAGE_INPUTS_MEMO_LIMIT
    memo.clear()
    assert len(memo) == 0


def test_a_hit_writes_into_the_profile_what_the_computation_wrote():
    bundle = quad_row_bundle(ROW)
    controller = session(memo=True)
    export = controller.get_topology_export(bundle, "object", "mesh")
    memo = StageInputsMemoV1()
    selected = frozenset(range(ROW))
    direct = EnvelopeDebugProfileBuilderV1("row", "PRODUCTION")
    stage_domain_inputs(bundle, selected, profile=direct, topology_export=export)
    profiles = []
    for _ in range(2):
        profile = EnvelopeDebugProfileBuilderV1("row", "PRODUCTION")
        stage_domain_inputs(bundle, selected, profile=profile, topology_export=export, memo=memo)
        profiles.append(profile.snapshot())
    assert (memo.misses, memo.hits) == (1, 1)
    reference = direct.snapshot()
    for snapshot in profiles:
        assert snapshot.counters == reference.counters
        assert snapshot.receipts == reference.receipts
        assert snapshot.counters and snapshot.receipts


def test_a_changed_revision_drops_the_memos():
    bundle = quad_row_bundle(ROW)
    controller = session(memo=True)
    press(bundle, controller, 0.25)
    press(bundle, controller, 0.26)
    assert len(controller.stage_inputs_memo) == 1 and len(controller.scan_memo) == ROW
    controller._drop_revision_scoped()
    assert len(controller.stage_inputs_memo) == 0 and len(controller.scan_memo) == 0


def test_every_materialized_domain_carries_the_recorded_step_facts_and_the_json_receipt_names_them(tmp_path):
    import json

    from cftuv_envelope.materialize.structure import batch_structure

    bundle = quad_row_bundle(ROW)
    controller = session(memo=True)
    run = press(bundle, controller, 0.25)
    materialized = [item for item in run.results if item.is_materialized]
    assert len(materialized) == ROW
    counted = sum(
        run.counter(name)
        for name in (
            production.PRODUCTION_ALPHA_INTERVALS_CERTIFIED,
            production.PRODUCTION_ALPHA_INTERVALS_AT_EVENT,
            production.PRODUCTION_ALPHA_INTERVALS_NOT_CERTIFIED,
        )
    )
    assert counted == ROW
    for item in materialized:
        assert item.alpha_interval is not None and item.alpha_interval.alpha == 0.25
        assert item.structure_digest == batch_structure(item.batch).digest
    summary = json.loads(production.export_production_json(run.results, tmp_path, label="step").read_text(encoding="utf-8"))
    for row in summary["domains"]:
        assert row["structure_digest"] and row["alpha_interval"]["law"] == "COVERAGE_AND_CLIP_CROSSINGS_V1"
        assert row["alpha_interval"]["uncertified"], "the record names what the interval does not certify"
