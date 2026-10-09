"""Продуктовый путь на хранилище по содержимому: правка меша пересчитывает только домены, чьё содержимое изменилось.

Кэши ревизии сбрасываются с её сменой, а ревизия — хэш всего меша, поэтому прежде любая правка делала холодными
ВСЕ домены. Теперь домен с тем же содержимым (ключ `envelope_content_key`) берёт подготовку и результат из
хранилища, а результат переносится на новую ревизию (`envelope_content_store`), в воркере пула, параллельно.

Что держат тесты:

1. Считаются только затронутые домены (счётчики сборок и пула), остальные приходят из хранилища.
2. Ответ равен ответу холодного прогона на правленом меше — меш писателя без порядка граней и номеров владельцев
   (`content_equivalence`: их выводит ядро хэшем от ревизии).
3. Возврат к прежней ревизии не считает ничего; новый alpha после правки берёт подготовки.
4. Сдвиг номеров патчей (шов на другом конце меша) — полное переиспользование с новыми номерами и id доменов.
5. Перенос, которого не вышло, назван, и домен считается заново; ключ, который нечем построить, — тоже назван.
6. Хранилище ограничено, `clear()` его сбрасывает, смена ревизии — нет.
"""

from __future__ import annotations

import sys
from fractions import Fraction
from pathlib import Path

import pytest

KERNEL_SRC = Path(__file__).resolve().parents[1] / "kernel" / "src"
if str(KERNEL_SRC) not in sys.path:
    sys.path.insert(0, str(KERNEL_SRC))

from cftuv import envelope_domain_pool as pool_module  # noqa: E402
from cftuv import envelope_production_export as production  # noqa: E402
from cftuv.envelope_content_key import ContentKeyUnsupported  # noqa: E402
from cftuv.envelope_debug_session import EnvelopeDebugSessionController  # noqa: E402
from cftuv.envelope_production_export import PLACEMENT_CACHED, run_production  # noqa: E402
from cftuv.envelope_production_mesh import build_mesh_arrays  # noqa: E402
from content_equivalence import mesh_projection, result_projection  # noqa: E402
from content_fixtures import InProcessPool, moved_vertex, renumbered, with_revision  # noqa: E402
from envelope_fixture_bundles import quad_row_bundle  # noqa: E402

ROW = 5
EDGES = frozenset(range(ROW))
ALPHA = 0.25
#: Правый верхний угол ряда принадлежит только последнему квадрату; вершина 2 — угол квадратов 1 и 2.
LAST_CORNER = 2 * ROW + 1
SHARED_CORNER = 2
#: Куда уходит правая верхняя вершина ряда: меш остаётся тем же рядом, но последний квадрат уже не плоский.
LIFT = (10.0, 2.0, 0.4)
COUNTERS = (
    production.PRODUCTION_CONTENT_KEYED,
    production.PRODUCTION_CONTENT_REGISTERED,
    production.PRODUCTION_CONTENT_RESULT_REUSED,
    production.PRODUCTION_CONTENT_PREPARATION_REUSED,
    production.PRODUCTION_CONTENT_RELABELED,
    production.PRODUCTION_CONTENT_RELABEL_FAILED,
    production.PRODUCTION_CONTENT_UNKEYED,
    production.PRODUCTION_PREPARATION_BUILDS,
)


@pytest.fixture
def pool(monkeypatch):
    """Пул «в процессе» и порог пересылки в ноль: малая партия иначе остаётся в родителе."""

    from cftuv import envelope_queue_pool

    fake = InProcessPool()
    monkeypatch.setattr(pool_module, "get_domain_pool", lambda workers, external_python="": fake)
    monkeypatch.setattr(envelope_queue_pool, "COVERAGE_POOL_MIN_BYTES", 0)
    return fake


@pytest.fixture(scope="module")
def row():
    return with_revision(quad_row_bundle(ROW), "row")


def _press(bundle, controller, *, alpha=ALPHA, workers=2, edges=EDGES, reach=None):
    return run_production(
        controller,
        bundle,
        edges,
        alpha,
        source_object_key="object",
        source_data_key="mesh",
        density=None,
        workers=workers,
        chart_reach_cap=reach,
    )


def _numbers(run):
    return {name.removeprefix("PRODUCTION_"): run.counter(name) for name in COUNTERS}


def _mesh(run):
    return mesh_projection(build_mesh_arrays(run.results, 0.001))


def _cold(bundle, *, alpha=ALPHA, edges=EDGES, reach=None):
    """Тот же меш на пустой сессии без пула: эталон ответа."""

    return _press(bundle, EnvelopeDebugSessionController(), alpha=alpha, workers=0, edges=edges, reach=reach)


def _same_answer(run, reference):
    assert [result_projection(item) for item in run.results] == [
        result_projection(item) for item in reference.results
    ]
    assert _mesh(run) == _mesh(reference)


# --------------------------------------------------------------------------
# 1-2. Считаются только затронутые домены; ответ равен холодному прогону
# --------------------------------------------------------------------------


def test_a_vertex_edit_computes_only_the_domain_that_owns_the_vertex(pool, row):
    controller = EnvelopeDebugSessionController()
    first = _press(row, controller)
    assert _numbers(first)["CONTENT_KEYED"] == ROW and not first.counter(production.PRODUCTION_CONTENT_RESULT_REUSED)
    pool.kinds.clear()
    edited = moved_vertex(row, LAST_CORNER, LIFT)
    assert edited.source_revision != row.source_revision

    run = _press(edited, controller)

    numbers = _numbers(run)
    assert numbers["CONTENT_RESULT_REUSED"] == ROW - 1 and numbers["CONTENT_RELABELED"] == ROW - 1
    assert numbers["PREPARATION_BUILDS"] == 1 and numbers["CONTENT_RELABEL_FAILED"] == 0
    assert (run.counter(production.PRODUCTION_RESULT_CACHE_HIT), run.counter(production.PRODUCTION_RESULT_CACHE_MISS)) == (ROW - 1, 1)
    assert [item.placement for item in run.results[: ROW - 1]] == [PLACEMENT_CACHED] * (ROW - 1)
    assert run.results[ROW - 1].placement != PLACEMENT_CACHED
    # Один домен холодный, четыре идут воркерам только на перенос (работа идёт параллельно, а не в родителе).
    assert sorted(pool.kinds) == ["cold"] + ["production"] * (ROW - 1)
    assert run.cold
    assert all(item.batch.source_revision.value.startswith("host-source:" + edited.source_revision.digest) for item in run.results)
    _same_answer(run, _cold(edited))


@pytest.mark.parametrize("move", ((10.0, 2.5, 0.0), LIFT), ids=("in-plane", "lifted"))
def test_the_edit_of_a_vertex_that_is_no_seam_of_a_neighbour_keeps_the_weld_of_a_cold_run(pool, row, move):
    """Красный контроль сварки: сосед по шву пересчитан, домен по ту сторону шва взят из хранилища.

    Правый верхний угол ряда принадлежит только последнему квадрату (на шве его нет), а общий шов с четвёртым
    квадратом сваривается по вершинам, чьи позиции в двух доменах побитово равны. Если бы нетронутый домен
    был устаревшим, счётчики сварки и сам сваренный меш разошлись бы с холодным прогоном.
    """

    controller = EnvelopeDebugSessionController()
    _press(row, controller)
    edited = moved_vertex(row, LAST_CORNER, move)

    run = _press(edited, controller)
    cold = _cold(edited)

    assert _numbers(run)["CONTENT_RESULT_REUSED"] == ROW - 1
    arrays, reference = build_mesh_arrays(run.results, 0.001), build_mesh_arrays(cold.results, 0.001)
    assert arrays.weld_counters == reference.weld_counters
    assert arrays.weld_counters and dict(arrays.weld_counters)["ADAPTER_WELD_GROUPS"] > 0
    assert arrays.seam_counters == reference.seam_counters
    assert [(patch, outcome) for patch, outcome, _detail in arrays.warnings] == [
        (patch, outcome) for patch, outcome, _detail in reference.warnings
    ]
    assert dict(arrays.weld_counters)["ADAPTER_WELD_POSITION_MISMATCH"] == 0
    _same_answer(run, cold)


def test_the_registered_counter_tells_the_keyed_from_the_stored(row, monkeypatch):
    from cftuv.envelope_debug_profile import EnvelopeDebugProfileBuilderV1
    from cftuv.envelope_debug_session import evaluate_envelope_debug_staged

    first = _press(row, EnvelopeDebugSessionController(), workers=0)
    assert _numbers(first)["CONTENT_KEYED"] == _numbers(first)["CONTENT_REGISTERED"] == ROW

    # Снапшот домена взят из кэша ревизии (его построила кнопка отладки, а не выгрузка под записью токенов):
    # домен ключён, но переносить его результат потом было бы нечем.
    controller = EnvelopeDebugSessionController()
    evaluate_envelope_debug_staged(
        row, EDGES, ALPHA, profile=EnvelopeDebugProfileBuilderV1("row", "QUEUE"), controller=controller,
        source_object_key="object", source_data_key="mesh", engine="QUEUE", density=None, workers=0,
    )
    monkeypatch.setattr(controller, "has_patch_metric", lambda *_a, **_k: False)

    run = _press(row, controller, workers=0)

    assert _numbers(run)["CONTENT_KEYED"] == ROW and _numbers(run)["CONTENT_REGISTERED"] == 0
    assert len(controller.content_store) == 0


def test_a_constant_of_the_request_policy_changed_between_presses_is_a_miss(pool, row, monkeypatch):
    """Подмена константы политики запроса БЕЗ `clear()`: на другой ревизии ключ её видит, ничего не берётся устаревшим.

    На ТОЙ ЖЕ ревизии кэши ревизии (прежние, ключ — запрос) подмены не видят: так было до хранилища и остаётся
    (записано в DECISIONS); хранилище же ключится подписью политики и умолчанием допуска растяжения.
    """

    from cftuv import envelope_request_policy as policy

    controller = EnvelopeDebugSessionController()
    _press(row, controller)
    monkeypatch.setattr(policy, "DEFAULT_ENVELOPE_STRETCH_BUDGET", Fraction(1, 4))
    pool.kinds.clear()

    run = _press(moved_vertex(row, LAST_CORNER, LIFT), controller)

    assert _numbers(run)["CONTENT_RESULT_REUSED"] == 0 and _numbers(run)["PREPARATION_BUILDS"] == ROW
    assert pool.kinds == ["cold"] * ROW


def test_a_vertex_of_two_domains_computes_both(pool, row):
    controller = EnvelopeDebugSessionController()
    _press(row, controller)
    pool.kinds.clear()

    run = _press(moved_vertex(row, SHARED_CORNER, (4.0, 0.0, 0.5)), controller)

    numbers = _numbers(run)
    assert numbers["PREPARATION_BUILDS"] == 2 and numbers["CONTENT_RESULT_REUSED"] == ROW - 2
    _same_answer(run, _cold(moved_vertex(row, SHARED_CORNER, (4.0, 0.0, 0.5))))


def test_the_workers_are_not_needed_for_the_move(row):
    """Без пула (ноль воркеров) то же: перенос делает родитель тем же кодом, ответ тот же."""

    controller = EnvelopeDebugSessionController()
    _press(row, controller, workers=0)
    edited = moved_vertex(row, LAST_CORNER, LIFT)

    run = _press(edited, controller, workers=0)

    numbers = _numbers(run)
    assert numbers["CONTENT_RESULT_REUSED"] == ROW - 1 and numbers["PREPARATION_BUILDS"] == 1
    _same_answer(run, _cold(edited))


# --------------------------------------------------------------------------
# 3. Возврат, повтор, новый alpha
# --------------------------------------------------------------------------


def test_the_return_to_a_previous_revision_computes_nothing(pool, row):
    controller = EnvelopeDebugSessionController()
    first = _press(row, controller)
    _press(moved_vertex(row, LAST_CORNER, LIFT), controller)
    pool.kinds.clear()

    back = _press(row, controller)

    assert pool.kinds == [] and not back.cold
    assert _numbers(back)["CONTENT_RESULT_REUSED"] == ROW and _numbers(back)["PREPARATION_BUILDS"] == 0
    assert [item.placement for item in back.results] == [PLACEMENT_CACHED] * ROW
    # Результаты возвращены без переноса: они лежат в идентичностях той самой ревизии.
    assert _numbers(back)["CONTENT_RELABELED"] == 0
    assert [item.content_digest for item in back.results] == [item.content_digest for item in first.results]


def test_the_press_after_an_edit_is_warm_and_builds_no_key(pool, row, monkeypatch):
    from cftuv import envelope_content_key

    controller = EnvelopeDebugSessionController()
    _press(row, controller)
    edited = moved_vertex(row, LAST_CORNER, LIFT)
    _press(edited, controller)
    pool.kinds.clear()

    def forbidden(*_args, **_kwargs):
        raise AssertionError("the key of a bound domain must not be built again")

    monkeypatch.setattr(envelope_content_key, "domain_content_key", forbidden)
    again = _press(edited, controller)

    assert pool.kinds == [] and not again.cold
    assert (again.counter(production.PRODUCTION_RESULT_CACHE_HIT), again.counter(production.PRODUCTION_RESULT_CACHE_MISS)) == (ROW, 0)


def test_a_selection_change_elsewhere_after_an_edit_moves_nothing(pool, row):
    """Смена выделения меняет id запроса прогона, но не требует переноса: он метка вычисления, как был всегда."""

    controller = EnvelopeDebugSessionController()
    _press(row, controller)
    edited = moved_vertex(row, LAST_CORNER, LIFT)
    first = _press(edited, controller)
    pool.kinds.clear()

    again = _press(edited, controller, edges=EDGES | {ROW + 2})  # верхнее ребро патча 2: выделение ТОЛЬКО его домена

    assert again.revision == first.revision
    assert _numbers(again)["CONTENT_RELABELED"] == 0 and _numbers(again)["CONTENT_RELABEL_FAILED"] == 0
    assert (again.counter(production.PRODUCTION_RESULT_CACHE_HIT), again.counter(production.PRODUCTION_RESULT_CACHE_MISS)) == (ROW - 1, 1)
    assert pool.kinds == ["cold"]
    reference = _cold(edited, edges=EDGES | {ROW + 2})
    # Результаты, взятые без переноса, несут id запроса СВОЕГО вычисления (как всегда было у кэша): сравнивается всё остальное.
    def without_request(run):
        views = [result_projection(item) for item in run.results]
        for view in views:
            if "batch" in view:
                view["batch"] = {name: value for name, value in view["batch"].items() if name != "decal_request_id"}
        return views

    assert without_request(again) == without_request(reference)
    assert _mesh(again) == _mesh(reference)


def test_another_alpha_after_an_edit_takes_the_preparations_from_the_store(pool, row):
    controller = EnvelopeDebugSessionController()
    _press(row, controller)
    edited = moved_vertex(row, LAST_CORNER, LIFT)
    _press(edited, controller)
    pool.kinds.clear()

    wider = _press(edited, controller, alpha=0.4)

    numbers = _numbers(wider)
    # Четыре подготовки идут из хранилища, пятая (правленый домен) лежит в кэше ревизии: считать их заново нечем.
    assert numbers["CONTENT_PREPARATION_REUSED"] == ROW - 1 and numbers["PREPARATION_BUILDS"] == 0
    assert wider.counter(production.PRODUCTION_PREPARATION_REUSED) == ROW
    assert numbers["CONTENT_RELABELED"] == ROW - 1 and numbers["CONTENT_RELABEL_FAILED"] == 0
    assert not wider.cold and pool.kinds == ["production"] * ROW
    _same_answer(wider, _cold(edited, alpha=0.4))


def test_another_law_after_an_edit_is_another_result_never_a_stale_one(pool, row, monkeypatch):
    """Закон топологии и закон подъёма — часть слота результата: после правки они не отдают прежний ответ."""

    laws = sorted(production.PRODUCTION_TOPOLOGY_LAWS - {production.PRODUCTION_TOPOLOGY_LAW})
    controller = EnvelopeDebugSessionController()
    _press(row, controller)
    edited = moved_vertex(row, LAST_CORNER, LIFT)
    _press(edited, controller)
    pool.kinds.clear()

    other = run_production(
        controller, edited, EDGES, ALPHA, source_object_key="object", source_data_key="mesh",
        density=None, workers=2, topology_law=laws[0],
    )

    assert {item.decal_topology_law for item in other.results} == {laws[0]}
    assert other.counter(production.PRODUCTION_RESULT_CACHE_MISS) == ROW
    assert _numbers(other)["CONTENT_PREPARATION_REUSED"] == ROW - 1 and _numbers(other)["PREPARATION_BUILDS"] == 0
    reference = run_production(
        EnvelopeDebugSessionController(), edited, EDGES, ALPHA, source_object_key="object",
        source_data_key="mesh", density=None, workers=0, topology_law=laws[0],
    )
    _same_answer(other, reference)

    from cftuv.surface_ir import HostNearPlanarLiftPolicy

    monkeypatch.setattr(production, "HOST_NEAR_PLANAR_LIFT_POLICY", HostNearPlanarLiftPolicy.CERTIFIED_PLANE_V1)
    lifted = _press(edited, controller)
    assert lifted.counter(production.PRODUCTION_RESULT_CACHE_MISS) == ROW


def test_another_object_with_the_same_content_takes_everything_from_the_store(pool, row):
    controller = EnvelopeDebugSessionController()
    _press(row, controller)
    pool.kinds.clear()
    copy = with_revision(row, "the-copy", row.source_revision.digest)

    run = _press(copy, controller)

    assert _numbers(run)["CONTENT_RESULT_REUSED"] == ROW and _numbers(run)["CONTENT_RELABELED"] == ROW
    assert not run.cold
    assert {item.batch.source_revision.value for item in run.results} == {"host-source:%s:the-copy" % row.source_revision.digest}
    _same_answer(run, _cold(copy))


# --------------------------------------------------------------------------
# 3а. Полосовая карта (BAND-CHART C1): досягаемость, выбор цепей и отказ по alpha
# --------------------------------------------------------------------------


def test_another_reach_cap_at_the_same_revision_is_a_miss_never_the_bound_answer(pool, row):
    """Красный контроль: привязка ключа к ревизии несёт ключ полосы, а не только выделение, плотность и допуск.

    Привязка и результат по ключу содержимого появляются у домена, взятого из хранилища при ДРУГОЙ ревизии (холодный
    домен лежит в кэше ревизии под ключом запроса, а тот досягаемость видит), поэтому нажатие идёт после правки меша.
    """

    controller = EnvelopeDebugSessionController()
    _press(row, controller)
    edited = moved_vertex(row, LAST_CORNER, LIFT)
    first = _press(edited, controller)
    assert _numbers(first)["CONTENT_RESULT_REUSED"] == ROW - 1
    pool.kinds.clear()

    wide = _press(edited, controller, reach=Fraction(3, 4))

    assert wide.revision == first.revision
    assert (wide.counter(production.PRODUCTION_RESULT_CACHE_HIT), wide.counter(production.PRODUCTION_RESULT_CACHE_MISS)) == (0, ROW)
    assert _numbers(wide)["CONTENT_RESULT_REUSED"] == 0 and pool.kinds == ["cold"] * ROW
    pool.kinds.clear()

    # Вернулись к умолчанию: ответ лежит под своей привязкой и своим ключом, ничего не считается.
    again = _press(edited, controller)
    assert (again.counter(production.PRODUCTION_RESULT_CACHE_HIT), again.counter(production.PRODUCTION_RESULT_CACHE_MISS)) == (ROW, 0)
    assert pool.kinds == []
    _same_answer(wide, _cold(edited, reach=Fraction(3, 4)))


def test_another_reach_cap_after_an_edit_recomputes_every_domain(pool, row):
    controller = EnvelopeDebugSessionController()
    _press(row, controller)
    edited = moved_vertex(row, LAST_CORNER, LIFT)
    pool.kinds.clear()

    run = _press(edited, controller, reach=Fraction(3, 4))

    assert _numbers(run)["CONTENT_RESULT_REUSED"] == 0 and _numbers(run)["CONTENT_PREPARATION_REUSED"] == 0
    assert pool.kinds == ["cold"] * ROW
    _same_answer(run, _cold(edited, reach=Fraction(3, 4)))


def test_the_key_of_a_domain_carries_the_band_key_of_its_patch(row, monkeypatch):
    """Ключ содержимого строится с ключом полосы ЭТОГО патча (досягаемость и выбранные рёбра патча)."""

    from cftuv import envelope_content_key

    seen = []
    real = envelope_content_key.domain_content_key

    def spy(export, selected, band_key=None, backend="PYTHON", skeleton_backend="PYTHON"):
        seen.append(band_key)
        return real(export, selected, band_key, backend, skeleton_backend)

    monkeypatch.setattr(envelope_content_key, "domain_content_key", spy)
    reach = Fraction(3, 4)
    _press(row, EnvelopeDebugSessionController(), reach=reach, workers=0)

    assert len(seen) == ROW
    assert sorted(tuple(band[1]) for band in seen) == [(patch,) for patch in range(ROW)]
    assert all(band[0] == reach for band in seen)


def _band_double(monkeypatch, cap):
    """Домены ряда притворяются доменами с картой-полосой досягаемости `cap`: отказ по alpha даёт строитель запроса."""

    from decimal import Decimal

    from cftuv import envelope_chart_band as chart_band
    from cftuv import envelope_request_export as request_export

    def refuse(_snapshot, alpha_decimal):
        if Decimal(alpha_decimal) > Decimal(str(cap)):
            raise request_export.EnvelopeHostAdapterError(
                request_export.EnvelopeDebugHostOutcome.REQUEST_ALPHA_EXCEEDS_CHART_REACH,
                f"alpha={float(alpha_decimal):.6g} m is beyond the chart reach cap {cap} m",
            )

    monkeypatch.setattr(chart_band, "refuse_alpha_beyond_reach", refuse)
    monkeypatch.setattr(request_export, "refuse_alpha_beyond_reach", refuse)


def test_an_alpha_beyond_the_reach_of_a_band_is_refused_for_a_stored_preparation_as_for_a_cold_domain(pool, row, monkeypatch):
    """Красный контроль: подготовка из хранилища не обходит отказ запроса по alpha выше досягаемости полосы."""

    _band_double(monkeypatch, 0.3)
    controller = EnvelopeDebugSessionController()
    _press(row, controller)
    edited = moved_vertex(row, LAST_CORNER, LIFT)
    _press(edited, controller)
    pool.kinds.clear()

    beyond = _press(edited, controller, alpha=0.4)
    sent_to_workers = list(pool.kinds)

    reference = _cold(edited, alpha=0.4)
    assert {item.outcome for item in reference.results} == {"REQUEST_ALPHA_EXCEEDS_CHART_REACH"}
    assert [(item.patch_id, item.domain_id, item.outcome, item.detail) for item in beyond.results] == [
        (item.patch_id, item.domain_id, item.outcome, item.detail) for item in reference.results
    ]
    assert sent_to_workers == [] and beyond.counter(production.PRODUCTION_REFUSED) == ROW
    # Отказ назван и следа в хранилище не оставляет: alpha в пределах досягаемости снова строит меш.
    inside = _press(edited, controller, alpha=0.25)
    assert all(item.is_materialized for item in inside.results)
    _same_answer(inside, _cold(edited, alpha=0.25))


# --------------------------------------------------------------------------
# 4. Сдвиг номеров патчей
# --------------------------------------------------------------------------


def test_a_shift_of_patch_numbers_reuses_every_domain_under_its_new_number(pool, row):
    controller = EnvelopeDebugSessionController()
    _press(row, controller)
    pool.kinds.clear()
    shifted = renumbered(row, {patch: patch + 3 for patch in range(ROW)})

    run = _press(shifted, controller)

    numbers = _numbers(run)
    assert numbers["CONTENT_RESULT_REUSED"] == ROW and numbers["PREPARATION_BUILDS"] == 0
    assert [item.patch_id for item in run.results] == [patch + 3 for patch in range(ROW)]
    assert all(item.domain_id == item.batch.patch_domain_id.value for item in run.results)
    _same_answer(run, _cold(shifted))


# --------------------------------------------------------------------------
# 5. Названные отказы
# --------------------------------------------------------------------------


def test_a_result_that_cannot_be_moved_is_named_and_its_domain_is_computed_anew(pool, row, capsys):
    controller = EnvelopeDebugSessionController()
    _press(row, controller)
    store = controller.content_store
    slot = production.result_slot(
        str(float(ALPHA)), production.PRODUCTION_UV_POLICY, production.PRODUCTION_TOPOLOGY_LAW,
        production.HOST_NEAR_PLANAR_LIFT_POLICY.value,
    )
    # Запись без токенов цепочек: неучтённая идентичность хоста в результате.
    key, entry = next(iter(store._entries.items()))
    stored = entry.results[slot]
    entry.results[slot] = _with_cut_record(stored)
    edited = moved_vertex(row, LAST_CORNER, LIFT)
    capsys.readouterr()

    run = _press(edited, controller)

    assert _numbers(run)["CONTENT_RELABEL_FAILED"] >= 1
    assert production.PRODUCTION_CONTENT_RELABEL_FAILED in capsys.readouterr().out
    assert all(item.is_materialized for item in run.results)
    _same_answer(run, _cold(edited))


def _with_cut_record(result):
    import dataclasses

    labels = result.labels
    cut = dataclasses.replace(labels, tokens=tuple(item for item in labels.tokens if item.kind != "chain-use"))
    return dataclasses.replace(result, labels=cut)


def test_a_domain_whose_input_cannot_be_keyed_is_computed_as_before_and_named(pool, row, monkeypatch):
    from cftuv import envelope_content_key

    def refuse(*_args, **_kwargs):
        raise ContentKeyUnsupported("a value the encoder does not know")

    monkeypatch.setattr(envelope_content_key, "domain_content_key", refuse)
    controller = EnvelopeDebugSessionController()

    run = _press(row, controller)

    assert _numbers(run)["CONTENT_UNKEYED"] == ROW and _numbers(run)["CONTENT_KEYED"] == 0
    assert len(controller.content_store) == 0
    _same_answer(run, _cold(row))


def test_a_press_fills_the_store_with_every_domain_it_materialized(pool, row):
    from cftuv.envelope_production_export import OUTCOME_DOMAIN_RAISED

    controller = EnvelopeDebugSessionController()
    run = _press(row, controller)

    assert all(item.outcome != OUTCOME_DOMAIN_RAISED for item in run.results)
    assert len(controller.content_store) == ROW and controller.content_store.result_count == ROW


# --------------------------------------------------------------------------
# 6. Жизнь хранилища
# --------------------------------------------------------------------------


def test_an_edit_keeps_the_store_and_a_reset_clears_it(pool, row):
    controller = EnvelopeDebugSessionController()
    _press(row, controller)
    _press(moved_vertex(row, LAST_CORNER, LIFT), controller)

    assert len(controller.content_store) == ROW + 1
    assert controller.invalidation_count >= 1

    controller.clear()

    assert len(controller.content_store) == 0 and controller.content_store.result_count == 0
    cold = _press(row, controller)
    assert cold.cold and _numbers(cold)["CONTENT_RESULT_REUSED"] == 0


def test_the_pickles_of_held_preparations_survive_an_edit(pool, row):
    controller = EnvelopeDebugSessionController()
    _press(row, controller)
    _press(row, controller, alpha=0.4)  # новый alpha: подготовки ушли воркерам пиклом
    held = len(controller.preparation_blobs)
    assert held >= ROW

    _press(moved_vertex(row, LAST_CORNER, LIFT), controller)

    assert len(controller.preparation_blobs) >= held


def test_the_store_is_bounded(pool, row, monkeypatch):
    from cftuv import envelope_content_store

    monkeypatch.setattr(envelope_content_store, "CONTENT_STORE_ENTRY_LIMIT", 3)
    monkeypatch.setattr(envelope_content_store, "CONTENT_STORE_RESULT_LIMIT", 3)
    controller = EnvelopeDebugSessionController()

    run = _press(row, controller)

    assert len(controller.content_store) == 3 and controller.content_store.result_count <= 3
    assert all(item.is_materialized for item in run.results)
    # Вытесненные домены просто считаются заново, а не ломают прогон.
    edited = _press(moved_vertex(row, LAST_CORNER, LIFT), controller)
    _same_answer(edited, _cold(moved_vertex(row, LAST_CORNER, LIFT)))


def test_a_preparation_of_the_debug_button_is_not_stored(pool, row):
    from cftuv.envelope_debug_profile import EnvelopeDebugProfileBuilderV1
    from cftuv.envelope_debug_session import evaluate_envelope_debug_staged

    controller = EnvelopeDebugSessionController()
    evaluate_envelope_debug_staged(
        row,
        EDGES,
        ALPHA,
        profile=EnvelopeDebugProfileBuilderV1("row", "QUEUE"),
        controller=controller,
        source_object_key="object",
        source_data_key="mesh",
        engine="QUEUE",
        density=None,
        workers=0,
    )

    run = _press(row, controller)

    # Подготовки кнопки отладки без записи токенов хоста: результат на них честно посчитан, но переносить его нечем.
    assert not run.cold and len(controller.content_store) == 0
    assert run.counter(production.PRODUCTION_PREPARATION_REUSED) == ROW
