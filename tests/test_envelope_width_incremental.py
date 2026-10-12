"""WIDTH_REFRESH_INCREMENTAL_V1: инкрементальный точный такт перетаскивания ширины и политика его сверки (без Blender).

Что держат тесты (каждое утверждение названо и проверено числами, а не словом):

1. СДВИГ СЕРТИФИКАТОМ. Результат домена, сдвинутый его аффинным сертификатом на новую ширину (`rebase_result`), равен точному lean-прогону на этой ширине: позиции и UV вида - с отклонением не больше
   `EXACT_TOLERANCE`, грани, швы, ссылки, владельцы и подпись структуры те же; на нулевом сдвиге вид - тот же объект. Копия помечена (lean, размещение `certificate`), у неё нет батча, и она не ложится ни в какой кэш.
2. ПОДМНОЖЕСТВО. `run_production(served=...)` не сканирует, не пересылает и не кэширует обслуженные домены, всегда отправляет остальные в пул (даже когда байтов подготовок мало) и считает их ТЕМ ЖЕ путём, что lean-такт:
   посчитанные домены побитово равны полному lean-прогону на той же ширине. Класс прогона и счётчики названы; обслуженный домен чужой ревизии не подменяется (считается как обычно); без `lean` подмножество отказано.
3. РЕШЕНИЕ ТИКА (`plan_tick`). Обслуживается ровно то, что двигает кадр модели; остальным домены названа причина; полный такт (сверка) назван причиной: нет записи сверки, число тактов, путь ширины, нет модели, чужая
   модель, мало обслуженных, переключатель выключен. Запись сверки пишет кнопка и полный такт, инкрементальный такт её только считает.
4. ОБРАЗЕЦ ТАКТА. Образец хранит ПРЕЖНИЕ результаты обслуженных доменов (следующий такт сдвигает их, а не копии), сверка предсказания (`deviation`) в инкрементальном такте не идёт и журнал доверия не меняется,
   модель строится из составного образца и равна точному прогону на ширине такта, а цепочка тактов не копит ошибку.
5. ЗАПРЕТ. Инкрементальный такт не ложится в зафиксированный ответ: подтверждение (COMMIT) на той же ширине побитово равно прямому прогону после любого числа тактов.
"""

from __future__ import annotations

import dataclasses
import re
import sys
from pathlib import Path

from types import SimpleNamespace

import numpy as np
import pytest

KERNEL_SRC = Path(__file__).resolve().parents[1] / "kernel" / "src"
if str(KERNEL_SRC) not in sys.path:
    sys.path.insert(0, str(KERNEL_SRC))

from cftuv import envelope_production_mesh as production_mesh  # noqa: E402
from cftuv import envelope_width_incremental as incremental  # noqa: E402
from cftuv import envelope_width_mesh_preview as preview  # noqa: E402
from cftuv import envelope_width_preview_model as pm  # noqa: E402
from cftuv import envelope_width_rebase as rebase  # noqa: E402
from cftuv.envelope_debug_session import EnvelopeDebugSessionController  # noqa: E402
from cftuv.envelope_domain_pool import shutdown_domain_pool  # noqa: E402
from cftuv.envelope_production_export import (  # noqa: E402
    PLACEMENT_CACHED,
    PRODUCTION_REFRESH_INCREMENTAL_RECOMPUTED,
    PRODUCTION_REFRESH_INCREMENTAL_SERVED,
    PRODUCTION_RESULT_CACHE_HIT,
    PRODUCTION_RESULT_CACHE_MISS,
    REFRESH_COMMIT,
    REFRESH_INCREMENTAL,
    REFRESH_LEAN,
    LeanResultError,
    production_timing_text,
    run_production,
)
from cftuv.envelope_production_result import PLACEMENT_REBASED  # noqa: E402
from content_fixtures import InProcessPool, with_revision  # noqa: E402
from envelope_fixture_bundles import quad_row_bundle  # noqa: E402

ROW = 5
EDGES = frozenset(range(ROW))
ALPHA = 0.25
OFFSET = 0.001
KEY = ("incremental-row",)
#: Допуск равенства сдвинутого результата и точного прогона (метры и единицы UV): округление `binary64` и разность ширин.
EXACT_TOLERANCE = 1e-9


@pytest.fixture(scope="module", autouse=True)
def _no_pool_outlives_the_module():
    yield
    shutdown_domain_pool()


@pytest.fixture(autouse=True)
def _no_distance_limit(monkeypatch):
    """Большинство тестов делают большие прыжки ширины (8 раз, 8 %), чтобы вывести домен за интервал или досягаемость: правило пути ширины от сверки в них выключено, а своё правило держит его тест."""

    monkeypatch.setattr(incremental, "INCREMENTAL_VERIFY_AFTER_DISTANCE", 1e9)


@pytest.fixture(scope="module")
def clipped():
    """Последний патч слегка вышел из плоскости (near-planar с резкой): интервал его домена кончается на ширине 1.9943, у остальных - на 2.0."""

    return with_revision(quad_row_bundle(ROW, lifted_corner=0.3), "clipped")


@pytest.fixture(scope="module")
def unfolded():
    """Последний патч вышел из плоскости сильно: его домен - развёртка (приближённый кадр: досягаемость 6 % ширины сертификата), его сосед по стене тоже приближён."""

    return with_revision(quad_row_bundle(ROW, lifted_corner=1.0), "unfolded")


@pytest.fixture(scope="module")
def flat():
    return with_revision(quad_row_bundle(ROW), "flat")


def _press(controller, bundle, alpha, *, lean=False, served=None, pool=None):
    return run_production(
        controller,
        bundle,
        EDGES,
        alpha,
        source_object_key="object",
        source_data_key="mesh",
        density=None,
        workers=2 if pool is not None else 0,
        domain_pool=pool,
        lean=lean,
        served=served,
    )


class _Session:
    """Сессия после кнопки (то, что кладёт `execute` оператора): образец с сертификатами на экране, запись сверки, модель строит первый кадр."""

    def __init__(self, bundle, alpha=ALPHA):
        self.bundle = bundle
        self.controller = EnvelopeDebugSessionController()
        self.run = _press(self.controller, bundle, alpha)
        arrays = production_mesh.build_mesh_arrays(self.run.results, OFFSET)
        self.sample = preview.build_sample(self.run.results, arrays, KEY, alpha)
        preview.note_button_display(self.controller, self.sample)
        assert preview.ensure_model(self.controller)

    def plan(self, alpha, grade=REFRESH_INCREMENTAL):
        c = self.controller
        return preview.plan_tick(grade, c.width_displayed, c.width_model, c.width_incremental, alpha)

    def tick(self, alpha, *, pool=None, plan=None):
        """Тик, как его считает поток `compute`: решение, прогон подмножества, хвост (`finish_live_run`); результат применён к сессии так, как применяет `lean_exact_now`."""

        c = self.controller
        plan = self.plan(alpha) if plan is None else plan
        run = _press(c, self.bundle, alpha, lean=True, served=plan.served, pool=pool)
        live = preview.finish_live_run(
            run, alpha=alpha, offset=OFFSET, key=KEY, displayed=c.width_displayed, previous=c.width_model, trust=c.width_trust, lean=True, plan=plan
        )
        preview.note_exact_display(c, live)
        return live


#: Числа материализатора, что меняются непрерывно с шириной и решений не несут (`S6_PARAMETRIC_DRAG_SPIKE` §1, класс 7): нанометры, милли-доли, предельные величины. Остальные счётчики - РЕШЕНИЯ (тесселяция, силуэт, резка): обслуженный домен
#: несёт счётчики прежнего прогона, и расхождение решений с точным прогоном - это перелом структуры, который такт не видит.
_RECORDED_NUMBERS = re.compile(r"NANOMETRES|MILLI|MILLICELLS|EXACT_WORK|_MAX_|DEPTH|_MIN_|SECONDS|BYTES")


def _decisions(counters):
    return tuple(item for item in counters if not _RECORDED_NUMBERS.search(item[0]))


def _view_difference(first, second):
    position = float(np.max(np.abs(np.asarray(first.vertices.positions) - np.asarray(second.vertices.positions)), initial=0.0))
    uv = float(np.max(np.abs(np.asarray(first.uvs) - np.asarray(second.uvs)), initial=0.0))
    return position, uv


# ==========================================================================
# 1. Сдвиг сертификатом
# ==========================================================================


@pytest.mark.parametrize("alpha", [0.2501, 0.26, 0.5, 1.0, 1.5])
def test_a_rebased_result_equals_the_exact_lean_result_of_the_new_width(clipped, alpha):
    session = _Session(clipped)
    exact = {item.patch_id: item for item in _press(session.controller, clipped, alpha, lean=True).results}
    moved = 0
    for origin in session.run.results:
        interval = origin.alpha_interval
        if not (interval.low < alpha < interval.high):
            continue  # за интервалом сдвигать нельзя: это событие ядра, домен считается заново
        shifted, why = rebase.rebase_result(origin, str(float(alpha)))
        assert why == "" and shifted is not None
        twin = exact[origin.patch_id]
        position, uv = _view_difference(shifted.view, twin.view)
        assert position <= EXACT_TOLERANCE and uv <= EXACT_TOLERANCE, (origin.patch_id, position, uv)
        for name in ("faces", "seams", "owners", "boundary_chains"):
            assert getattr(shifted.view, name) == getattr(twin.view, name), name
        assert shifted.view.vertices.refs == twin.view.vertices.refs and shifted.view.vertices.normals == twin.view.vertices.normals
        assert shifted.structure_digest == twin.structure_digest and _decisions(shifted.counters) == _decisions(twin.counters) and shifted.outcome == twin.outcome
        moved += 1
    assert moved >= ROW - 1


def test_a_zero_shift_gives_the_same_view_objects_and_a_certificate_of_the_same_width(flat):
    session = _Session(flat)
    origin = session.run.results[0]
    shifted, why = rebase.rebase_result(origin, str(float(ALPHA)))
    assert why == "" and shifted.view is origin.view and shifted.parametric.alpha == origin.parametric.alpha


def test_a_rebased_result_is_a_lean_result_that_says_where_it_came_from_and_cannot_be_cached(flat):
    session = _Session(flat)
    controller = session.controller
    origin = session.run.results[1]
    shifted, _why = rebase.rebase_result(origin, "0.26")

    assert shifted.is_materialized and shifted.lean and shifted.placement == PLACEMENT_REBASED and shifted.step_path == "REBASED"
    assert shifted.parametric.alpha == pm.Fraction("0.26") and shifted.parametric.position_slopes is origin.parametric.position_slopes
    for name in ("batch", "vertex_normals", "content_digest"):
        with pytest.raises(LeanResultError):
            getattr(shifted, name)
    with pytest.raises(LeanResultError):
        controller.remember_production_result(("rebased",), shifted)
    assert (shifted.patch_id, shifted.domain_id, shifted.counters, shifted.alpha_interval) == (origin.patch_id, origin.domain_id, origin.counters, origin.alpha_interval)
    assert origin.placement != PLACEMENT_REBASED and not origin.lean  # исходный результат не тронут


def test_a_result_that_cannot_be_moved_is_refused_by_name(flat):
    session = _Session(flat)
    origin = session.run.results[0]
    no_numbers = dataclasses.replace(origin.parametric, markers=("PARAM_NON_AFFINE_CLIP_NODE",))
    assert rebase.rebase_result(origin.with_changes(parametric=no_numbers), "0.26") == (None, rebase.REBASE_NO_CERTIFICATE)
    assert rebase.rebase_result(origin.with_changes(parametric=None), "0.26") == (None, rebase.REBASE_NO_CERTIFICATE)
    wrong_shape = dataclasses.replace(origin.parametric, vertex_count=origin.parametric.vertex_count + 1)
    assert rebase.rebase_result(origin.with_changes(parametric=wrong_shape), "0.26") == (None, rebase.REBASE_SHAPE_MISMATCH)
    assert rebase.rebase_result(origin, "0.0") == (None, rebase.REBASE_ALPHA_NOT_POSITIVE_FINITE)
    refusal = session.run.results[0].with_changes(outcome="SOME_REFUSAL")
    assert rebase.rebase_result(refusal, "0.26") == (None, rebase.REBASE_NOT_MATERIALIZED)


# ==========================================================================
# 2. Подмножество
# ==========================================================================


def test_the_subset_run_computes_only_the_domains_the_certificate_does_not_serve_and_they_equal_the_full_lean_run(clipped):
    session = _Session(clipped)
    alpha = 1.999  # за интервалом домена 4 (1.9943), внутри интервалов остальных
    plan = session.plan(alpha)
    assert plan.incremental and sorted(plan.served) == [0, 1, 2, 3] and plan.held == {4: pm.DOMAIN_HELD_OUTSIDE}

    pool = InProcessPool()
    subset = _press(session.controller, clipped, alpha, lean=True, served=plan.served, pool=pool)
    full = _press(session.controller, clipped, alpha, lean=True)

    assert subset.refresh == REFRESH_INCREMENTAL and full.refresh == REFRESH_LEAN
    assert pool.kinds == ["production"], "в пул ушёл один домен: подмножество не пересылает обслуженные"
    assert subset.counter(PRODUCTION_REFRESH_INCREMENTAL_SERVED) == 4 and subset.counter(PRODUCTION_REFRESH_INCREMENTAL_RECOMPUTED) == 1
    assert subset.counter(PRODUCTION_RESULT_CACHE_HIT) == 0 and subset.counter(PRODUCTION_RESULT_CACHE_MISS) == 1  # обслуженные - не попадания кэша
    assert "WIDTH_REFRESH_INCREMENTAL_V1 4 served by the certificate, 1 recomputed" in production_timing_text(subset)
    by_patch = {item.patch_id: item for item in subset.results}
    for twin in full.results:
        item = by_patch[twin.patch_id]
        if twin.patch_id in plan.served:
            assert item.placement == PLACEMENT_REBASED and item.placement != PLACEMENT_CACHED
            position, uv = _view_difference(item.view, twin.view)
            assert position <= EXACT_TOLERANCE and uv <= EXACT_TOLERANCE
        else:
            assert item.placement != PLACEMENT_REBASED
            assert item.view == twin.view  # посчитанный домен побитово равен полному lean-прогону
            assert (item.outcome, item.detail, item.counters, item.diagnostics, item.structure_digest, item.alpha_interval) == (
                twin.outcome, twin.detail, twin.counters, twin.diagnostics, twin.structure_digest, twin.alpha_interval
            )


def test_a_small_subset_goes_to_the_pool_where_a_small_full_batch_stays_in_the_parent(clipped):
    """Мало байт подготовок значит мало доменов, а не то, что пересылка не окупится: подмножество всегда в пул, а полная партия ниже порога остаётся в родителе (как было)."""

    session = _Session(clipped)
    plan = session.plan(1.999)
    pool_full = InProcessPool()
    _press(session.controller, clipped, 1.999, lean=True, pool=pool_full)
    pool_subset = InProcessPool()
    _press(session.controller, clipped, 1.999, lean=True, served=plan.served, pool=pool_subset)
    assert pool_full.kinds == [] and pool_subset.kinds == ["production"]


def test_without_a_pool_the_subset_is_computed_in_the_parent_with_the_same_answer(clipped):
    session = _Session(clipped)
    plan = session.plan(1.999)
    parent = _press(session.controller, clipped, 1.999, lean=True, served=plan.served)
    pooled = _press(session.controller, clipped, 1.999, lean=True, served=plan.served, pool=InProcessPool())
    assert [item.view for item in parent.results if item.placement != PLACEMENT_REBASED] == [item.view for item in pooled.results if item.placement != PLACEMENT_REBASED]
    assert [item.placement for item in parent.results].count(PLACEMENT_REBASED) == 4


def test_a_served_set_belongs_to_a_lean_run_only(clipped):
    session = _Session(clipped)
    plan = session.plan(1.999)
    with pytest.raises(ValueError, match="lean run"):
        _press(session.controller, clipped, 1.999, served=plan.served)


def test_a_served_result_of_another_revision_is_not_substituted_the_domain_is_computed(clipped):
    session = _Session(clipped)
    plan = session.plan(1.999)
    foreign = dict(plan.served)
    foreign[0] = foreign[0].with_changes(domain_id="patch-domain:another-revision")
    run = _press(session.controller, clipped, 1.999, lean=True, served=foreign)
    by_patch = {item.patch_id: item for item in run.results}
    assert by_patch[0].placement != PLACEMENT_REBASED and by_patch[0].domain_id != "patch-domain:another-revision"
    assert run.counter(PRODUCTION_REFRESH_INCREMENTAL_SERVED) == 3 and run.counter(PRODUCTION_REFRESH_INCREMENTAL_RECOMPUTED) == 2


# ==========================================================================
# 3. Решение тика
# ==========================================================================


def test_the_plan_serves_what_the_model_frame_moves_and_names_why_the_rest_is_recomputed(unfolded):
    session = _Session(unfolded)
    near = session.plan(0.2501)
    far = session.plan(0.27)  # 8 % от ширины сертификата: за досягаемостью приближённых доменов (6 %)

    assert near.incremental and sorted(near.served) == [0, 1, 2, 3, 4] and near.held == {}
    assert far.incremental and sorted(far.served) == [0, 1, 2] and far.held == {3: pm.DOMAIN_HELD_DISPLAY_REACH, 4: pm.DOMAIN_HELD_DISPLAY_REACH}
    model = session.controller.width_model
    assert [bool(item) for item in pm.served_mask(model, 0.27)] == [True, True, True, False, False]
    assert far.originals == {patch: session.run.results[patch] for patch in far.served}  # сдвигаются прежние результаты, не копии
    assert all(item.placement == PLACEMENT_REBASED for item in far.served.values())


def test_a_tick_that_did_not_order_incrementality_has_no_plan(flat):
    session = _Session(flat)
    assert session.plan(0.26, grade=REFRESH_LEAN) is None and session.plan(0.26, grade=REFRESH_COMMIT) is None


@pytest.mark.parametrize(
    "mutate, expected",
    [
        (lambda c: setattr(c, "width_incremental", None), incremental.REASON_NO_BASE),
        (lambda c: setattr(c, "width_incremental", incremental.IncrementalStateV1(incremental.INCREMENTAL_VERIFY_AFTER_TICKS, ALPHA)), incremental.REASON_PERIOD_TICKS),
        (lambda c: setattr(c, "width_model", None), incremental.REASON_NO_MODEL),
        (lambda c: setattr(c, "width_displayed", dataclasses.replace(c.width_displayed)), incremental.REASON_MODEL_FOREIGN),
        (lambda c: setattr(c, "width_displayed", dataclasses.replace(c.width_displayed, origins={})), incremental.REASON_MODEL_FOREIGN),
    ],
)
def test_a_tick_is_a_full_lean_tick_under_a_named_reason_when_the_verification_is_due_or_there_is_nothing_to_serve(flat, mutate, expected):
    session = _Session(flat)
    mutate(session.controller)
    plan = session.plan(0.2501)
    assert not plan.incremental and plan.mode == REFRESH_LEAN and plan.reason == expected and plan.served == {} and plan.held == {}


def test_a_sample_without_results_to_move_is_a_full_tick_by_name(flat):
    session = _Session(flat)
    displayed = dataclasses.replace(session.sample, origins={})
    model = pm.build_model(displayed)
    session.controller.width_displayed, session.controller.width_model = displayed, model
    assert session.plan(0.2501).reason == incremental.REASON_NO_ORIGINS


def test_the_path_of_the_width_from_the_verification_is_a_reason_of_its_own(flat, monkeypatch):
    monkeypatch.setattr(incremental, "INCREMENTAL_VERIFY_AFTER_DISTANCE", 0.02)
    session = _Session(flat)
    assert session.plan(ALPHA * (1.0 + 0.5 * incremental.INCREMENTAL_VERIFY_AFTER_DISTANCE)).incremental
    for alpha in (ALPHA * (1.0 + 2.0 * incremental.INCREMENTAL_VERIFY_AFTER_DISTANCE), ALPHA * (1.0 - 2.0 * incremental.INCREMENTAL_VERIFY_AFTER_DISTANCE)):
        plan = session.plan(alpha)
        assert not plan.incremental and plan.reason == incremental.REASON_PERIOD_DISTANCE


def test_a_tick_that_serves_too_few_domains_is_a_full_tick_that_verifies_for_free(unfolded, monkeypatch):
    session = _Session(unfolded)
    monkeypatch.setattr(incremental, "INCREMENTAL_MIN_SERVED_SHARE", 0.7)
    monkeypatch.setattr(preview, "INCREMENTAL_MIN_SERVED_SHARE", 0.7)
    assert session.plan(0.2501).incremental  # пять из пяти
    plan = session.plan(0.27)  # три из пяти < 0.7
    assert not plan.incremental and plan.reason == incremental.REASON_FEW_SERVED and plan.domains == ROW


def test_the_switch_turns_the_incremental_tick_off_by_name(flat, monkeypatch):
    session = _Session(flat)
    monkeypatch.setenv(incremental.ENVIRONMENT_SWITCH, "0")
    assert session.plan(0.2501).reason == incremental.REASON_SWITCHED_OFF


def test_the_verification_record_is_written_by_the_button_and_the_full_ticks_and_only_counted_by_the_incremental_ones(flat):
    session = _Session(flat)
    controller = session.controller
    assert controller.width_incremental == incremental.IncrementalStateV1(0, ALPHA)  # кнопка
    session.tick(0.2501)
    session.tick(0.2502)
    assert controller.width_incremental == incremental.IncrementalStateV1(2, ALPHA)
    full = session.tick(0.2503, plan=incremental.full_plan(incremental.REASON_PERIOD_TICKS))  # сверка
    assert not full.plan.incremental and controller.width_incremental == incremental.IncrementalStateV1(0, 0.2503)
    preview.drop_model(controller, preview.HISTORY_STEP)
    assert controller.width_incremental is None  # образца нет - сверять не с чем


def test_the_ticks_count_up_to_the_period_and_the_next_one_is_the_verification(flat):
    session = _Session(flat)
    reasons = []
    for step in range(1, incremental.INCREMENTAL_VERIFY_AFTER_TICKS + 2):
        plan = session.plan(ALPHA * (1.0 + 0.001 * step))
        reasons.append((plan.incremental, plan.reason))
        session.tick(ALPHA * (1.0 + 0.001 * step), plan=plan)
    assert reasons == [(True, "")] * incremental.INCREMENTAL_VERIFY_AFTER_TICKS + [(False, incremental.REASON_PERIOD_TICKS)]
    assert session.controller.width_incremental.ticks == 0
    log = preview.log_of(session.controller)
    assert log.incremental_ticks == 0  # журнал считает кадры, легшие в меш (`lean_exact_now`), а тут меш не пишется


# ==========================================================================
# 4. Образец такта
# ==========================================================================


def test_the_sample_of_an_incremental_tick_keeps_the_previous_results_of_the_served_domains_and_judges_nothing(clipped):
    session = _Session(clipped)
    controller = session.controller
    before_trust, before_model = controller.width_trust, controller.width_model
    live = session.tick(1.999)

    assert live.plan.incremental and live.check is None and live.trust_events == () and live.trust is before_trust
    assert sorted(live.sample.origins) == [0, 1, 2, 3, 4]
    for patch in range(4):
        assert live.sample.origins[patch] is session.run.results[patch]  # прежний результат на своей ширине, не сдвинутая копия
    assert live.sample.origins[4] is not session.run.results[4] and live.sample.origins[4].placement != PLACEMENT_REBASED
    assert controller.width_displayed is live.sample and controller.width_model is live.model and live.model is not before_model
    assert controller.width_incremental == incremental.IncrementalStateV1(1, ALPHA)


def test_the_composite_sample_equals_the_exact_lean_sample_of_the_width_and_the_model_frame_predicts_the_next_one(clipped):
    session = _Session(clipped)
    alpha = 1.999
    live = session.tick(alpha)
    full_run = _press(session.controller, clipped, alpha, lean=True)
    full_arrays = production_mesh.build_mesh_arrays(full_run.results, OFFSET, reports=False)
    full = preview.build_sample(full_run.results, full_arrays, KEY, alpha)

    assert live.sample.layout_digest == full.layout_digest  # раскладка меша та же: кадр ляжет одной записью
    assert np.max(np.abs(live.sample.positions - full.positions)) <= EXACT_TOLERANCE and np.max(np.abs(live.sample.uvs - full.uvs)) <= EXACT_TOLERANCE
    assert [item.token for item in live.sample.domains] == [item.token for item in full.domains]
    assert [item.structure for item in live.sample.domains] == [item.structure for item in full.domains]
    model_frame = pm.positions_and_uvs(live.model, alpha * 0.9995)
    nearby = _press(session.controller, clipped, alpha * 0.9995, lean=True)
    nearby_arrays = production_mesh.build_mesh_arrays(nearby.results, OFFSET, reports=False)
    assert np.max(np.abs(model_frame[0] - np.asarray(nearby_arrays.positions))) <= 1e-6  # домены, которые модель двигает, лежат на точном; придержанный остаётся на геометрии образца


def test_a_chain_of_incremental_ticks_does_not_accumulate_the_error_and_always_moves_the_original_results(flat):
    session = _Session(flat)
    origins = dict(session.sample.origins)
    widths = [ALPHA * (1.0 + 0.002 * step) for step in range(1, 4)]
    for alpha in widths:
        live = session.tick(alpha)
        assert live.plan.incremental
        assert all(live.sample.origins[patch] is origins[patch] for patch in origins)  # всегда одни и те же исходные результаты
        exact = _press(session.controller, session.bundle, alpha, lean=True)
        exact_arrays = production_mesh.build_mesh_arrays(exact.results, OFFSET, reports=False)
        assert np.max(np.abs(live.sample.positions - np.asarray(exact_arrays.positions))) <= EXACT_TOLERANCE
        assert np.max(np.abs(live.sample.uvs - np.asarray(exact_arrays.uvs))) <= EXACT_TOLERANCE
    assert session.controller.width_incremental.ticks == len(widths)


def test_the_arrays_of_an_incremental_tick_differ_from_the_full_tick_only_in_rounding_and_the_labels_of_the_owners(clipped):
    session = _Session(clipped)
    alpha = 1.999
    plan = session.plan(alpha)
    run = _press(session.controller, clipped, alpha, lean=True, served=plan.served)
    arrays = production_mesh.build_mesh_arrays(run.results, OFFSET, reports=False)
    full = production_mesh.build_mesh_arrays(_press(session.controller, clipped, alpha, lean=True).results, OFFSET, reports=False)
    assert arrays.faces == full.faces and arrays.seam_edges == full.seam_edges and arrays.face_domain == full.face_domain
    assert arrays.domain_tokens == full.domain_tokens and arrays.domain_vertex_index == full.domain_vertex_index
    assert np.max(np.abs(np.asarray(arrays.positions) - np.asarray(full.positions))) <= EXACT_TOLERANCE


# ==========================================================================
# 5. Подтверждение
# ==========================================================================


def test_the_confirm_after_incremental_ticks_is_the_direct_commit_run_and_carries_no_trace_of_them(clipped):
    session = _Session(clipped)
    for alpha in (0.2501, 0.2502, 1.999):
        session.tick(alpha, pool=InProcessPool())
    cached = session.controller.production_result_count
    confirm = _press(session.controller, clipped, 0.3)  # COMMIT: ни lean, ни served
    direct = _press(EnvelopeDebugSessionController(), clipped, 0.3)

    assert confirm.refresh == REFRESH_COMMIT and not any(item.lean or item.placement == PLACEMENT_REBASED for item in confirm.results)
    assert production_mesh.build_mesh_arrays(confirm.results, OFFSET).digest == production_mesh.build_mesh_arrays(direct.results, OFFSET).digest
    assert [item.content_digest for item in confirm.results] == [item.content_digest for item in direct.results]
    assert session.controller.production_result_count >= cached  # кэш растёт только зафиксированными результатами


# ==========================================================================
# 6. Заказ тика инструментом (`request_drag_tick`): класс заказа, решение, строки
# ==========================================================================

from test_envelope_refresh_lean import _drag_world  # noqa: E402
from cftuv import envelope_width_live as live  # noqa: E402


def _after_the_button(world):
    """Мир после кнопки: образец с сертификатами на экране, запись сверки (кнопка её пишет) и модель (её строит первый кадр)."""

    controller = world.controller
    arrays = production_mesh.build_mesh_arrays(world.run.results, 0.001)
    sample = preview.build_sample(world.run.results, arrays, preview.sample_key(world.record, 0.001), 0.25)
    preview.note_button_display(controller, sample)
    assert preview.ensure_model(controller)
    return controller


def test_the_tool_orders_the_drag_tick_as_incremental_and_the_tick_after_the_button_serves_every_domain(monkeypatch):
    calls: list = []
    world = _drag_world(monkeypatch, calls=calls)
    controller = _after_the_button(world)
    assert live.request_drag_tick(world.context, controller, 0.2505)
    assert controller.width_live._pending.payload.grade == REFRESH_INCREMENTAL and live.REFRESH_INCREMENTAL in live.DRAG_GRADES
    live.settle_width_live(controller)

    assert len(calls) == 1
    tick = calls[0]
    assert tick.plan.incremental and sorted(tick.plan.served) == list(range(ROW)) and tick.plan.held == {}
    assert tick.run.refresh == REFRESH_INCREMENTAL and tick.run.counter(PRODUCTION_REFRESH_INCREMENTAL_SERVED) == ROW
    assert tick.arrays.reports is False and tick.check is None  # lean-массивы; сверка предсказания не идёт
    assert world.written == []  # зафиксированной записи нет


def test_the_first_tick_without_a_verification_record_is_a_full_lean_tick_under_a_named_reason(monkeypatch):
    calls: list = []
    world = _drag_world(monkeypatch, calls=calls)  # образец положен без кнопки: записи сверки нет
    controller = world.controller
    assert controller.width_incremental is None
    assert live.request_drag_tick(world.context, controller, 0.2505)
    live.settle_width_live(controller)

    tick = calls[0]
    assert not tick.plan.incremental and tick.plan.reason == incremental.REASON_NO_BASE
    assert tick.run.refresh == REFRESH_LEAN and tick.run.counter(PRODUCTION_REFRESH_INCREMENTAL_SERVED) is None
    assert isinstance(tick.check, (pm.DeviationV1, type(None)))  # полный такт сверяет прежнюю модель, как lean-такт


def test_the_switch_returns_the_drag_tick_to_the_plain_lean_tick(monkeypatch):
    calls: list = []
    world = _drag_world(monkeypatch, calls=calls)
    controller = _after_the_button(world)
    monkeypatch.setenv(incremental.ENVIRONMENT_SWITCH, "0")
    assert live.request_drag_tick(world.context, controller, 0.2505)
    live.settle_width_live(controller)
    assert calls[0].plan.reason == incremental.REASON_SWITCHED_OFF and calls[0].run.refresh == REFRESH_LEAN


def test_the_confirm_after_incremental_ticks_is_the_commit_and_the_ticks_wrote_nothing_committed(monkeypatch):
    calls: list = []
    world = _drag_world(monkeypatch, calls=calls)
    controller = _after_the_button(world)
    for width in (0.2505, 0.251):
        assert live.request_drag_tick(world.context, controller, width)
        live.settle_width_live(controller)
    assert len(calls) == 2 and all(item.plan.incremental for item in calls)

    preview.finish_drag(controller, restore=False)
    world.drag(0.3)  # подтверждение: запись ширины -> COMMIT
    live.settle_width_live(controller)
    assert len(calls) == 2 and len(world.written) == 1 and world.written[0][3] == 0.3
    assert not any(item.lean or item.placement == PLACEMENT_REBASED for item in world.written[0][0])


def test_an_incremental_frame_is_laid_down_like_a_lean_frame_and_named_in_the_state_the_log_and_the_status(monkeypatch):
    from test_envelope_refresh_lean import _exact
    from test_envelope_width_preview_model import _owned

    world = _owned(monkeypatch)
    controller = world.controller
    controller.width_incremental = incremental.IncrementalStateV1(1, 0.25)
    plan = incremental.IncrementalPlanV1(REFRESH_INCREMENTAL, "", {0: object()}, {0: object()}, {1: pm.DOMAIN_HELD_OUTSIDE}, 2, 0.004)
    results = [SimpleNamespace(patch_id=0, placement=PLACEMENT_REBASED), SimpleNamespace(patch_id=1, placement="worker")]
    frame = _exact(0.27)
    frame = SimpleNamespace(**{**vars(frame), "plan": plan, "run": SimpleNamespace(results=results)})

    assert preview.lean_exact_now(controller, frame) == ""

    state = controller.width_mesh_preview
    assert state.outcome == REFRESH_INCREMENTAL and (state.live, state.held, state.held_patches) == (1, 1, (1,)) and controller.width_displayed is frame.sample
    assert controller.width_incremental == incremental.IncrementalStateV1(2, 0.25)  # такт добавлен к счёту, ширина сверки прежняя
    log = preview.log_of(controller)
    assert (log.incremental_ticks, log.incremental_served, log.incremental_recomputed, log.incremental_full) == (1, 1, 1, {})
    lines = preview.status_lines(controller)
    assert lines[0].startswith("Mesh WIDTH_REFRESH_INCREMENTAL_V1: width 0.27, 1 domains moved by the affine certificate (structure verified afterwards), 1 recomputed exactly (patch 1)")
    assert "checked by the next full tick" in lines[0] and any("Incremental ticks WIDTH_REFRESH_INCREMENTAL_V1: 1 incremental" in line for line in lines)

    # Полный такт под названной причиной (сверка): счёт начат заново, причина - в журнале.
    full = SimpleNamespace(**{**vars(_exact(0.28)), "plan": incremental.full_plan(incremental.REASON_PERIOD_TICKS)})
    assert preview.lean_exact_now(controller, full) == ""
    assert controller.width_incremental == incremental.IncrementalStateV1(0, 0.28) and controller.width_mesh_preview.outcome == REFRESH_LEAN
    assert preview.log_of(controller).incremental_full == {incremental.REASON_PERIOD_TICKS: 1}
