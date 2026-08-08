"""Ворота ленивой гидратации: ленивый ответ ПОБИТОВО равен плотному.

Почему это ворота, а не разовый прогон. Ленивая гидратация снимает цену
плотного пересъёма фронта на каждом exact-time пакете, и снимает она её
единственным законным способом — не считая ДВАЖДЫ то, что уже посчитано.
Утверждение «память возвращает ровно то, что вернул бы пересчёт» доказывается
двумя способами, и оба обязаны быть здесь.

ПЕРВЫЙ — ПО ПОСТРОЕНИЮ. Ключ памяти (`PositionMemoV1`) — это РОВНО те
аргументы, от которых зависит `_hydrate_position`: две несущие прямые в
порядке (prev, next), закон скольжения и момент `t`. Ни одна строка формулы не
читает ни рантаймовый id, ни соседей, ни состояние строителя; `prime_universe`
держится самой памятью и проверяется `admits`. Значит попадание в память и
пересчёт — одно значение, а не два близких.

ВТОРОЙ — ПРОГОНОМ, и он здесь главный. Доказательство по построению держится
на утверждении о чистоте, а утверждения в этом репозитории имеют привычку
расходиться с реальностью (`AGENTS.md`). Поэтому весь корпус, весь корпус
весов и стен, все полевые фикстуры, фигура с длинной цепочкой смертей и
ОТКАЗНЫЕ домены гоняются ДВАЖДЫ — `dense_hydration=True` и `False` — и
сравниваются по всем наблюдаемым осям: исход, semantic_digest, узлы, уровни,
счётчики, статус доказательства и его ДОЛГИ, грани с точными площадями,
покрытие на нескольких alpha.

ОТКАЗНЫЙ ПУТЬ ЦЕННЕЕ УСПЕШНОГО. Там, где фронт не закрылся, равенство ленивого
и плотного обязано держаться тем более: иначе отказ был бы артефактом памяти,
а не свойством входа. Поэтому корпус здесь НЕ фильтруется по исходу.
"""

from __future__ import annotations

from fractions import Fraction
import json
from pathlib import Path

import pytest

import cftuv_envelope as kernel
from cftuv_envelope.wavefront import prepare_conveyor
from cftuv_envelope.wavefront.coverage import coverage_at
from cftuv_envelope.wavefront.digest import node_record, semantic_digest
from cftuv_envelope.wavefront.faces import build_faces
from cftuv_envelope.wavefront.skeleton import SplitSearch, build_skeleton
from cftuv_envelope.wavefront.exact_candidate_view import PositionMemoV1

from wavefront_cases import named_corpus, partial_source_corpus, star
from weighted_wall_differential_cases import weighted_wall_differential_corpus


#: Alpha, на которых сверяется покрытие. Ноль и единица — границы, дробь между
#: ними — общий случай, крупная — фронт, прошедший фигуру насквозь.
_ALPHAS = (Fraction(0), Fraction(1, 3), Fraction(1), Fraction(4))

_FIELD_CASES = (
    "building_all_seams_patch_001_lost_resolved_v1",
    "building_all_seams_patch_006_lost_resolved_v1",
    "building_all_seams_patch_011_lost_resolved_v1",
    "building_all_seams_patch_105_lost_resolved_v1",
)


def _sqrt_sum_record(value) -> list:
    return [
        [radicand, [coefficient.numerator, coefficient.denominator]]
        for radicand, coefficient in value.terms
    ]


def _point_record(point) -> list:
    return [_sqrt_sum_record(point[0]), _sqrt_sum_record(point[1])]


def _obligation_record(obligation) -> dict:
    """Долг доказательства целиком, а не только его число.

    Статус — величина агрегированная, и совпасть он может при РАЗНЫХ долгах.
    Ворота, сверяющие только статус, пропустили бы подмену одного долга другим,
    и подмена эта была бы молчаливой.
    """

    return {
        name: (
            value.value if hasattr(value, "value")
            else repr(value)
        )
        for name, value in sorted(vars(obligation).items())
    } if hasattr(obligation, "__dict__") else {
        name: (
            getattr(obligation, name).value
            if hasattr(getattr(obligation, name), "value")
            else repr(getattr(obligation, name))
        )
        for name in sorted(obligation.__slots__)
    }


def _skeleton_axes(skeleton) -> dict:
    return {
        "outcome": skeleton.outcome.value,
        "semantic_digest": semantic_digest(skeleton),
        "levels": skeleton.levels,
        "counters": [list(item) for item in skeleton.counters],
        "proof_status": skeleton.proof_status.value,
        "proof_obligations": [
            _obligation_record(item) for item in skeleton.proof_obligations
        ],
        "nodes": sorted(
            (
                json.dumps(node_record(node), sort_keys=True)
                for node in skeleton.nodes
            )
        ),
    }


def _partition_axes(partition) -> dict:
    return {
        "outcome": partition.outcome.value,
        "detail": partition.detail,
        "doubled_area": _sqrt_sum_record(partition.doubled_area),
        "polygon_doubled_area": partition.polygon_doubled_area,
        "area_defect": _sqrt_sum_record(partition.area_defect),
        "faces": [
            {
                "owner": repr(face.owner),
                "source_start": list(face.source_start),
                "source_end": list(face.source_end),
                "line": repr(face.line),
                "doubled_area": _sqrt_sum_record(face.doubled_area),
                "points": [_point_record(point) for point in face.points],
            }
            for face in partition.faces
        ],
    }


def _coverage_axes(partition) -> dict:
    rows = {}
    for alpha in _ALPHAS:
        covered = coverage_at(partition, alpha)
        rows[str(alpha)] = {
            "outcome": covered.outcome.value,
            "detail": covered.detail,
            "doubled_area": _sqrt_sum_record(covered.doubled_area),
            "polygon_doubled_area": covered.polygon_doubled_area,
            "faces": [
                {
                    "owner": repr(face.owner),
                    "doubled_area": _sqrt_sum_record(face.doubled_area),
                    "points": [_point_record(point) for point in face.points],
                }
                for face in covered.faces
            ],
        }
    return rows


def _observable(polygon, *, dense: bool, search: SplitSearch,
                coverage: bool) -> dict:
    """Всё, что о фигуре можно наблюдать снаружи, одним словарём."""

    skeleton = build_skeleton(
        polygon, split_search=search, dense_hydration=dense
    )
    axes = {"skeleton": _skeleton_axes(skeleton)}
    partition = build_faces(polygon, skeleton)
    axes["faces"] = _partition_axes(partition)
    if coverage:
        axes["coverage"] = _coverage_axes(partition)
    return axes


def _assert_shadow(label, polygon, *, coverage: bool = True) -> str:
    """Ленивый и плотный ответы совпадают на ОБОИХ режимах поиска разрезов.

    `coverage=False` — только для полевых многоугольников, и это не послабление
    ворот, а снятие ЛИШНЕЙ работы. `coverage_at` — чистая функция пары
    (разбиение, alpha): гидратации она не видит вовсе, в её подпись строитель не
    входит. Разбиения здесь уже сверены ЦЕЛИКОМ — каждая грань, каждая её
    точка, каждая точная площадь и невязка площади, — поэтому равенство
    покрытий из них СЛЕДУЕТ, а не проверяется заново. Считать его на полевом
    домене при произвольной alpha стоит дорого и, что важнее, БЕЗ БЮДЖЕТА:
    знак `a*x + b*y - c - alpha*sqrt(q)` умеет уходить в факторизацию, и
    ворота стали бы ждать её без потолка. Настоящее полевое покрытие
    сравнивается там, где у него есть и alpha домена, и бюджет:
    `artifacts/lazy_frozen_hydration/ab_field.py` и полевая релиз-матрица.
    """

    outcomes = []
    for search in SplitSearch:
        lazy = _observable(
            polygon, dense=False, search=search, coverage=coverage
        )
        dense = _observable(
            polygon, dense=True, search=search, coverage=coverage
        )
        assert lazy == dense, (label, search.value)
        outcomes.append(lazy["skeleton"]["outcome"])
    return outcomes[0]


# --------------------------------------------------------------------------
# Корпус 63: именованные фигуры плюс частичный источник.
# --------------------------------------------------------------------------

_NAMED = tuple(named_corpus()) + tuple(partial_source_corpus())


def test_named_corpus_is_sixty_three_cases():
    """Размер корпуса — часть ворот: молчаливо усохший корпус не ворота."""

    assert len(_NAMED) == 63


@pytest.mark.parametrize("name,polygon", _NAMED, ids=[n for n, _ in _NAMED])
def test_lazy_matches_dense_on_the_named_corpus(name, polygon):
    _assert_shadow(name, polygon)


# --------------------------------------------------------------------------
# Корпус весов и стен. Здесь живут ОТКАЗНЫЕ домены и длинные цепочки смертей:
# именно на нём измерено, что недостающие вхождения лежат в 6-7 шагах от
# события, то есть радиусом снимок не сужается.
# --------------------------------------------------------------------------

_WEIGHTED = tuple(
    (case.name, case.polygon) for case in weighted_wall_differential_corpus()
)


def test_weighted_wall_corpus_is_twenty_three_cases():
    assert len(_WEIGHTED) == 23


@pytest.mark.parametrize(
    "name,polygon", _WEIGHTED, ids=[n for n, _ in _WEIGHTED]
)
def test_lazy_matches_dense_on_the_weighted_wall_corpus(name, polygon):
    _assert_shadow(name, polygon)


def test_the_weighted_wall_corpus_actually_contains_refusals():
    """Ворота отказного пути обязаны ИМЕТЬ отказ, иначе они его не проверяют.

    Проверка, у которой нельзя построить нарушение, не проверяет ничего. Здесь
    строится наоборот: утверждается, что среди прогнанных случаев есть хотя бы
    один НЕ-EXACT, — иначе сверка отказного пути была бы словом.
    """

    outcomes = {
        name: build_skeleton(polygon).outcome.value
        for name, polygon in _WEIGHTED
    }
    assert any(value != "EXACT" for value in outcomes.values()), outcomes


# --------------------------------------------------------------------------
# Длинная цепочка смертей. Звёздчатые контуры в общем положении — вход, на
# котором фронт умирает не парой, а цепью, и именно они опровергли фиксированный
# радиус K1/K2.
# --------------------------------------------------------------------------

_LONG_CHAIN = tuple(
    (f"star_15_seed_{seed}", star(15, seed))
    for seed in range(4)
)


@pytest.mark.parametrize(
    "name,polygon",
    [row for row in _LONG_CHAIN if row[1] is not None],
    ids=[name for name, polygon in _LONG_CHAIN if polygon is not None],
)
def test_lazy_matches_dense_on_long_death_chains(name, polygon):
    _assert_shadow(name, polygon)


# --------------------------------------------------------------------------
# Поле. Форма ворот та же, что у уже существующей полевой тени
# (`test_wavefront_superlevel_transaction.py`): конвейер поднимается ОДИН раз —
# он даёт многоугольник региона и стоит дорого, — а сверяются оба режима на
# этом многоугольнике. Гонять `prepare_conveyor` дважды значило бы удвоить
# самую дорогую часть ворот, ничего не добавив к сверяемым осям: гидратация
# живёт в марше фронта, а не в мосте законов прихода.
# --------------------------------------------------------------------------


def _field_inputs(case_name):
    root = (
        Path(__file__).parents[1]
        / "fixtures"
        / "sem_clb_02_lost_domains_v1"
        / "cases"
        / case_name
    )
    snapshot = kernel.AnalysisSnapshotCodecV1.loads(
        (root / "analysis_snapshot.json").read_bytes()
    )
    request = kernel.DecalRequestCodecV1.loads(
        (root / "decal_request.json").read_bytes()
    )
    (domain,) = snapshot.patch_domains
    return snapshot, request, domain.patch_domain_id


@pytest.mark.parametrize("case_name", _FIELD_CASES)
def test_lazy_matches_dense_on_the_field_fixtures(case_name):
    snapshot, request, domain_id = _field_inputs(case_name)
    prepared = prepare_conveyor(
        snapshot, request, patch_domain_id=domain_id
    )
    (region,) = prepared.regions
    polygon = region.bridge.polygon
    assert polygon is not None, (case_name, prepared.detail)
    _assert_shadow(case_name, polygon, coverage=False)


# --------------------------------------------------------------------------
# Сама память: утверждения о ней, у которых МОЖНО построить нарушение.
# --------------------------------------------------------------------------


def test_the_memo_refuses_a_foreign_prime_universe():
    """Чужой базис примитивных скоростей память не обслуживает.

    Иначе утверждение «`prime_universe` держится памятью, а не ключом» было бы
    прозой: подсунуть чужой базис и получить старый ответ никто бы не заметил.
    """

    memo = PositionMemoV1((2, 3))
    assert memo.admits((2, 3))
    assert not memo.admits((2, 3, 5))
    assert not memo.admits(())


def test_the_memo_is_bounded_by_the_superlevel():
    """Память ограничена уровнем: `clear` действительно чистит.

    Ограничение — про ПАМЯТЬ, а не про правильность: значение не зависит от
    возраста записи. Но незачищаемая память у длинного марша копила бы словарь
    на весь домен, и это надо было бы заметить не профилем, а падением.
    """

    memo = PositionMemoV1((2,))
    memo.entries[("k",)] = "v"
    assert memo.entries
    memo.clear()
    assert not memo.entries


def test_dense_mode_really_switches_the_memo_off():
    """Плотный режим — не декорация: памяти у него нет вовсе.

    Без этой проверки теневая сверка могла бы сравнивать ленивый путь сам с
    собой и зеленеть по построению.
    """

    from cftuv_envelope.wavefront import skeleton as skeleton_module

    polygon = dict(named_corpus())["axis_square"]
    dense = skeleton_module._Builder(polygon, dense_hydration=True)
    lazy = skeleton_module._Builder(polygon, dense_hydration=False)
    assert dense._position_memo is None
    assert lazy._position_memo is not None
    assert dense._candidate_view().position_memo is None
    assert lazy._candidate_view().position_memo is not None


def test_the_lazy_run_spends_strictly_fewer_position_hydrations():
    """Ленивый режим ДЕШЕВЛЕ плотного — числом, а не обещанием.

    Ворота про цену стоят рядом с воротами про ответ намеренно: карточка
    закрывается только тогда, когда ОБА утверждения проверяются прогоном.
    """

    from cftuv_envelope.exact_sqrt_sum import exact_work_budget

    polygon = dict(named_corpus())["cross"]
    dense_budget = exact_work_budget(stage="TEST", domain_id="dense")
    lazy_budget = exact_work_budget(stage="TEST", domain_id="lazy")
    build_skeleton(
        polygon, work_budget=dense_budget, dense_hydration=True
    )
    build_skeleton(
        polygon, work_budget=lazy_budget, dense_hydration=False
    )
    assert lazy_budget.exact_position_hydrations > 0
    assert (
        lazy_budget.exact_position_hydrations
        < dense_budget.exact_position_hydrations
    )
