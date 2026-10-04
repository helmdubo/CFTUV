"""Домен не отказывает из-за стыка потока: перекладина с вершиной резки, сдвинутой привязкой, и снятие стыка плана.

Полевая беда (`sagging_wall`, патч 0, домен `...85cef2`, ширина декали от 0.381): `STATION_VALUE_CONFLICT: vertex
clip:22 in region 5 has two (s, r) answers` — кнопка отказала патчу, который на ширине до 0.38 строился. Причина не
в стыке закона `CORNER_JOIN_SAME_PCHAIN_V1`, а в перекладине JOIN плана: ребро между полосами двух кусков стены лежит на
биссектрисе ТОЧНО, пока вершина `src:` стоит где стояла; привязка `src:` к углу карты перед резкой (`clip_snap`)
сдвигает её на 1.4 ячейки (допуск назван и посчитан), и новая вершина резки на перекладине получает от двух пробегов
разные `(s, r)` (`s` 1.4787 против 1.2568, `r` 0.380920 против 0.380923). Перекладинный закон
(`RUNG_STATION_FROM_CHAIN_VERTEX_V1`) требует точного равенства и отказывает; снятие стыка (`junction_to_withdraw`)
знало только стыки без записи угла, а этот стык — JOIN плана.

Исправление двухслойное. (1) `RUNG_CHORD_STATION_V1`: единственный ответ для вершины на ОБЩЕМ ребре двух граней
потока — линейный вдоль ребра между фактами его концов; на биссектрисе он равен перекладинному закону точно, при
сдвинутом конце расходится с аффинной картой пробега на тот же сдвиг, что назван счётчиком привязки. (2) Снятие
стыка достаёт и стык JOIN плана, последним средством: угол и митра остаются, снимается непрерывность `u`.

| что проверяется                                                                      | тест |
|--------------------------------------------------------------------------------------|------|
| поле: патч 0 строится на каждой alpha, где он отказывал; закон посчитан                | `test_the_field_domain_materializes_...` |
| КРАСНЫЙ КОНТРОЛЬ: без обоих слоёв (как `8531c1a`) возвращается тот же отказ поля       | `test_without_both_layers_the_field_refusal_returns` |
| второй слой один: домен строится, снято больше стыков, регионов больше (швы названы); первый слой возвращает непрерывность | `test_the_withdrawal_of_a_plan_join_alone_still_builds_the_domain` |
| закон ребра там, где отвечал перекладинный закон, даёт ТОТ ЖЕ батч побитово              | `test_the_chord_law_gives_the_batch_of_the_rung_law_where_the_rung_law_answers` |
| вершина стыка плана, снятого до таблицы, остаётся названным углом                      | `test_a_plan_join_withdrawn_before_the_table_stays_a_named_corner` |
| порядок снятия: сперва стык без записи угла, затем стык плана                           | `test_the_junction_without_a_record_is_withdrawn_before_the_plan_junction` |
| интерполяция по ребру: точна, а вне ребра, вне отрезка и без концов ответа нет          | `test_the_chord_station_is_the_exact_interpolation_of_the_ends` |
| ребро новой вершины берётся из контура грани: ближайшие вершины исходного контура       | `test_the_chord_of_a_new_vertex_is_the_edge_between_the_nearest_original_vertices` |
"""

from __future__ import annotations

import dataclasses
from fractions import Fraction
from functools import lru_cache
from pathlib import Path
from types import SimpleNamespace

import pytest

import cftuv_envelope as kernel
import cftuv_envelope.materialize.assemble as assemble
import cftuv_envelope.materialize.clip as clip_module
import cftuv_envelope.materialize.domain as domain_module
from cftuv_envelope.contracts.geometry_batch import DecalTopologyLawV1
from cftuv_envelope.contracts.metric import NearPlanarLiftLawV1
from cftuv_envelope.exact_sqrt_sum import SqrtSumV1
from cftuv_envelope.ids import PolicyId
from cftuv_envelope.materialize.admit import MaterializationOutcome, materialization_request
from cftuv_envelope.materialize.clip import _chords_of
from cftuv_envelope.materialize.domain import materialize_domain
from cftuv_envelope.materialize.stations import (
    SKIP_JOIN_WITHDRAWN_AT_STATION_CONFLICT,
    chain_station_table,
    junction_to_withdraw,
)
from cftuv_envelope.wavefront import conveyor_coverage, prepare_conveyor

import materialize_factories as factories

FIXTURE = Path(__file__).resolve().parents[1] / "fixtures" / "sagging_wall_rung_chord_v1"
EQUIVALENCE = Path(__file__).resolve().parents[1] / "fixtures" / "wall_noise_top_rung_clip_v1"
UV = PolicyId("UV_DIRECT_STRIP_V1")
ALPHAS = ("0.4", "0.6")


@lru_cache(maxsize=None)
def _domain(alpha: str):
    snapshot = kernel.AnalysisSnapshotCodecV1.loads((FIXTURE / "analysis_snapshot.json").read_bytes())
    request = kernel.DecalRequestCodecV1.loads((FIXTURE / f"decal_request_alpha_{alpha}.json").read_bytes())
    prepared = prepare_conveyor(snapshot, request)
    assert prepared.outcome.value == "EXACT", prepared.detail
    coverage = conveyor_coverage(prepared, alpha)
    assert coverage.outcome.value == "EXACT", coverage.detail
    return prepared, coverage


def _materialize(alpha: str):
    """Домен так, как его строит кнопка: законы продукта (`SOURCE_FACES_CLIPPED_V1`, `PLANAR_POLYGONS_V1`)."""

    prepared, coverage = _domain(alpha)
    result = materialize_domain(
        prepared,
        coverage,
        request=materialization_request(prepared, uv_policy_id=UV),
        near_planar_lift_law=NearPlanarLiftLawV1.SOURCE_FACES_CLIPPED_V1,
        decal_topology_law=DecalTopologyLawV1.PLANAR_POLYGONS_V1,
    )
    return result, dict(result.counters)


def _without_the_chord_law(monkeypatch):
    monkeypatch.setattr(assemble, "_chord_station", lambda *args, **kwargs: None)


def _without_the_plan_withdrawal(monkeypatch):
    """Снятие стыка таким, каким оно было на `8531c1a`: только стыки без записи угла."""

    real = domain_module.junction_to_withdraw
    monkeypatch.setattr(
        domain_module,
        "junction_to_withdraw",
        lambda table, runs: real(dataclasses.replace(table, plan_joins=()), runs),
    )


@pytest.mark.parametrize("alpha", ALPHAS)
def test_the_field_domain_materializes_at_every_alpha_that_used_to_refuse(alpha):
    result, counters = _materialize(alpha)

    assert result.outcome is MaterializationOutcome.MATERIALIZED, result.detail
    assert result.batch is not None
    # Закон назван и посчитан: вершины резки на общем ребре перекладины получили станцию по ребру.
    assert counters["MATERIALIZE_RUNG_CHORD_STATIONS"] >= 1
    # Остался ровно один снятый стык — узел события скелета (три пробега одного потока в одной вершине).
    assert counters["STATION_SKIP_JOIN_WITHDRAWN_AT_STATION_CONFLICT"] == 1


@pytest.mark.parametrize("alpha", ALPHAS)
def test_without_both_layers_the_field_refusal_returns(alpha, monkeypatch):
    """Красный контроль: оба слоя выключены — тот же отказ, что у владельца (`8531c1a`), с вершиной `clip:`."""

    _without_the_chord_law(monkeypatch)
    _without_the_plan_withdrawal(monkeypatch)

    result, _counters = _materialize(alpha)

    assert result.outcome is MaterializationOutcome.BATCH_DID_NOT_VALIDATE
    assert result.detail.startswith("STATION_VALUE_CONFLICT: vertex clip:"), result.detail
    assert "in region 5 has two (s, r) answers" in result.detail


@pytest.mark.parametrize("alpha", ALPHAS)
def test_the_withdrawal_of_a_plan_join_alone_still_builds_the_domain(alpha, monkeypatch):
    """Второй слой без первого: домен строится, снято больше стыков, регионов больше (швы названы пропуском)."""

    full, full_counters = _materialize(alpha)
    _without_the_chord_law(monkeypatch)

    result, counters = _materialize(alpha)

    assert result.outcome is MaterializationOutcome.MATERIALIZED, result.detail
    assert counters["MATERIALIZE_RUNG_CHORD_STATIONS"] == 0
    assert counters["STATION_SKIP_JOIN_WITHDRAWN_AT_STATION_CONFLICT"] > full_counters["STATION_SKIP_JOIN_WITHDRAWN_AT_STATION_CONFLICT"]
    assert counters["MATERIALIZE_REGIONS"] > full_counters["MATERIALIZE_REGIONS"]
    # Первый слой возвращает непрерывность: стыки без записи угла, снятые прежним кодом из-за вершин на перекладине, остались потоком.
    assert counters["STATION_SAME_PCHAIN_JOINS"] < full_counters["STATION_SAME_PCHAIN_JOINS"]
    # Снятые стыки названы пропуском в строке диагностики батча, а не потеряны.
    assert any("CORNER_JOIN_SAME_PCHAIN_V1" in line for line in result.diagnostics)
    assert full.outcome is MaterializationOutcome.MATERIALIZED


def _materialized_equivalence_domain():
    snapshot = kernel.AnalysisSnapshotCodecV1.loads((EQUIVALENCE / "analysis_snapshot.json").read_bytes())
    request = kernel.DecalRequestCodecV1.loads((EQUIVALENCE / "decal_request.json").read_bytes())
    prepared = prepare_conveyor(snapshot, request)
    assert prepared.outcome.value == "EXACT", prepared.detail
    coverage = conveyor_coverage(prepared, "0.5")
    result = materialize_domain(
        prepared,
        coverage,
        request=materialization_request(prepared, uv_policy_id=UV),
        near_planar_lift_law=NearPlanarLiftLawV1.SOURCE_FACES_CLIPPED_V1,
        decal_topology_law=DecalTopologyLawV1.PLANAR_POLYGONS_V1,
    )
    assert result.outcome is MaterializationOutcome.MATERIALIZED, result.detail
    return result, dict(result.counters)


def test_the_chord_law_gives_the_batch_of_the_rung_law_where_the_rung_law_answers(monkeypatch):
    """Закон ребра ничего не меняет там, где перекладинный закон отвечал: ответ тот же, батч тот же побитово.

    Вершины резки на перекладине, лежащей на биссектрисе ТОЧНО, получают станцию вершины цепи от перекладинного закона.
    Здесь он выключен ТОЛЬКО на стадии резки, и те же вершины получают интерполяцию по ребру: дайджест полного батча
    обязан совпасть. Расходятся они лишь там, где перекладина сдвинута привязкой (поле `sagging_wall`), а там
    перекладинный закон отказывал.
    """

    reference, reference_counters = _materialized_equivalence_domain()
    assert reference_counters["MATERIALIZE_RUNG_CHORD_STATIONS"] == 0
    real_values, real_rung = assemble.station_values, assemble._rung_station

    def clip_stage_without_the_rung_law(*args, chords=None, **kwargs):
        if chords is None:
            return real_values(*args, chords=chords, **kwargs)
        assemble._rung_station = lambda *_args: None
        try:
            return real_values(*args, chords=chords, **kwargs)
        finally:
            assemble._rung_station = real_rung

    monkeypatch.setattr(clip_module, "station_values", clip_stage_without_the_rung_law)

    chord, chord_counters = _materialized_equivalence_domain()

    assert chord_counters["MATERIALIZE_RUNG_CHORD_STATIONS"] >= 1, "the fixture must have clip vertices on a rung"
    assert chord.content_digest == reference.content_digest
    assert chord.batch.semantic_digest == reference.batch.semantic_digest


def test_a_plan_join_withdrawn_before_the_table_stays_a_named_corner():
    prepared, _coverage = _domain("0.4")
    whole = chain_station_table(prepared, factories.budget())
    assert whole.plan_joins, "the fixture must keep a JOIN of the plan that became a flow"
    vertex, first, second = whole.plan_joins[0]

    held = chain_station_table(prepared, factories.budget(), frozenset({vertex}))

    assert len(held.plan_joins) == len(whole.plan_joins) - 1
    assert all(item[0] != vertex for item in held.plan_joins)
    assert sum(1 for _where, name in held.skips if name == SKIP_JOIN_WITHDRAWN_AT_STATION_CONFLICT) == 1
    # Вхождения больше не в одном потоке: у каждого свой кадр либо поток короче.
    assert held.flow_of_run != whole.flow_of_run
    assert len(held.joins) == len(whole.joins) - 1


def _table_of(same_chain=(), plan=()):
    runs = {name: SimpleNamespace(chain_use_id=name) for name in ("a", "b", "c", "d")}
    return SimpleNamespace(runs=runs, same_chain_joins=tuple(same_chain), plan_joins=tuple(plan))


def test_the_junction_without_a_record_is_withdrawn_before_the_plan_junction():
    # Прежний порядок цел: новый пробег `b` сначала входящий, затем исходящий стык, и стыки без записи идут первыми.
    both = _table_of(same_chain=[("v-chain", "b", "c", "CONVEX")], plan=[("v-plan", "a", "b")])
    assert junction_to_withdraw(both, ("b",)) == "v-chain"
    # Только стык плана: он снимается последним средством.
    assert junction_to_withdraw(_table_of(plan=[("v-plan", "a", "b")]), ("b", "c")) == "v-plan"
    # Входящий раньше исходящего, новый пробег раньше прежних: порядок один и тот же у обоих списков.
    plan = _table_of(plan=[("v-1", "a", "b"), ("v-2", "b", "c")])
    assert junction_to_withdraw(plan, ("b",)) == "v-1"
    assert junction_to_withdraw(plan, ("c", "a")) == "v-2"
    # Нет стыка у пробегов конфликта — снимать нечего, и конфликт остаётся конфликтом.
    assert junction_to_withdraw(_table_of(plan=[("v-1", "a", "b")]), ("c", "d")) is None
    assert junction_to_withdraw(_table_of(), ("a",)) is None


def _point(x, y):
    return (SqrtSumV1.rational(x), SqrtSumV1.rational(y))


def _fact(s, r):
    return (SqrtSumV1.rational(s), SqrtSumV1.rational(r))


def test_the_chord_station_is_the_exact_interpolation_of_the_ends():
    chord = ("u", "v", _point(0, 0), _point(4, 0))
    anchors = {(5, "u"): _fact(1, 0), (5, "v"): _fact(1, 2)}
    budget = factories.budget()

    # Середина ребра: `s` постоянна (перекладина), `r` — среднее концов.
    assert assemble._chord_station(chord, _point(1, 0), 5, anchors, budget) == _fact(1, Fraction(1, 2))
    # Концы ребра — собственные факты концов.
    assert assemble._chord_station(chord, _point(0, 0), 5, anchors, budget) == _fact(1, 0)
    assert assemble._chord_station(chord, _point(4, 0), 5, anchors, budget) == _fact(1, 2)
    # Ребро вдоль второй оси и концы с разной `s`: интерполяция по обеим координатам фактов.
    vertical = ("u", "v", _point(0, 0), _point(0, 6))
    sloped = {(5, "u"): _fact(0, 1), (5, "v"): _fact(3, 4)}
    assert assemble._chord_station(vertical, _point(0, 2), 5, sloped, budget) == _fact(1, 2)
    # Точка вне прямой ребра (вторая координата точная), вне отрезка, регион без фактов концов, нулевое ребро — ответа нет.
    assert assemble._chord_station(chord, _point(1, 1), 5, anchors, budget) is None
    assert assemble._chord_station(chord, _point(5, 0), 5, anchors, budget) is None
    assert assemble._chord_station(chord, _point(-1, 0), 5, anchors, budget) is None
    assert assemble._chord_station(chord, _point(1, 0), 6, anchors, budget) is None
    assert assemble._chord_station(("u", "v", _point(0, 0), _point(0, 0)), _point(0, 0), 5, anchors, budget) is None


def test_the_chord_of_a_new_vertex_is_the_edge_between_the_nearest_original_vertices():
    original = [("a", _point(0, 0)), ("b", _point(4, 0)), ("c", _point(4, 4)), ("d", _point(0, 4))]
    refined = [
        ("a", _point(0, 0)),
        ("clip:0", _point(1, 0)),
        ("clip:1", _point(2, 0)),
        ("b", _point(4, 0)),
        ("c", _point(4, 4)),
        ("d", _point(0, 4)),
        ("clip:2", _point(0, 2)),
    ]
    (found,) = _chords_of([original], [refined])

    assert set(found) == {"clip:0", "clip:1", "clip:2"}
    assert found["clip:0"][:2] == ("a", "b") and found["clip:1"][:2] == ("a", "b")
    # Ребро замыкает контур: после последней исходной вершины идёт первая.
    assert found["clip:2"][:2] == ("d", "a")
    assert found["clip:0"][2:] == (_point(0, 0), _point(4, 0))
    # Грань без новых вершин и контур без исходных вершин (нечего брать концом) — пустые записи.
    assert _chords_of([original], [list(original)]) == [{}]
    assert _chords_of([[("z", _point(9, 9))]], [[("clip:0", _point(1, 1))]]) == [{}]
