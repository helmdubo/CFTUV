"""Ветка `crowded` сборщика граней: две дуги между парой рёбер, выбор по границе 4.

Вход — домен patch 17 здания `building` (`kernel/fixtures/building_patch17_crowded_v1`,
выгружен `artifacts/faces_chain_refusal/export_fixture17.py`): в остром выпуклом
углу (~35 градусов) внешней петли зажата четырёхугольная дыра, и на плотностях
Fan Density 2, 3, 4 между рёбрами `L0e2` и `L0e3` рождаются ДВЕ дуги. До ветки
домен отказывал `FACES_DID_NOT_ASSEMBLE` / `FACE_CHAIN_DOES_NOT_CLOSE`
(DECISIONS 2026-10-01); плотности 0 и 1 собирались и до неё.

Что проверяется, по столбцу на утверждение:

| утверждение | тест |
|---|---|
| домен собирается на d2, d3, d4, все три границы держатся | `test_the_wedged_hole_domain_assembles` |
| граница 4 держится на ИТОГОВОМ разбиении, проверенная ГЕОМЕТРИЕЙ, а не ключами | `test_every_unpaired_segment_lies_on_the_polygon_boundary` |
| на фикстуре различающиеся сегменты путей все парные (иначе парность не различает) | `test_on_the_fixture_every_segment_that_differs_between_path_variants_is_paired` |
| ядро отвергает сегмент без пары вне границы, невидимый парности и площади | `test_the_boundary_half_rejects_an_unpaired_segment_that_pairing_and_area_cannot_see` |
| предикат границы точен и спрашивает вложенность, не только прямую | `test_the_boundary_predicate_is_exact_and_asks_for_containment_not_just_collinearity` |
| поздний отказ после грани ветки называет собственную причину | `test_a_late_edge_refusal_after_a_branch_face_keeps_its_own_reason` |
| та же проверка заявляет нарушение на трёх неверных комбинациях | `test_the_geometric_check_rejects_every_wrong_combination` |
| парность, граница области и площадь НЕЗАВИСИМО выбирают одну и ту же комбинацию из четырёх | `test_pairing_boundary_and_area_each_pick_the_same_single_combination` |
| несколько допустимых комбинаций — именованная неоднозначность с числами | `test_several_admissible_combinations_are_named_ambiguous` |
| потолок перебора — тоже неоднозначность, а не «первая найденная» | `test_every_cap_turns_the_branch_into_a_named_ambiguity` |
| ни одной допустимой — прежний отказ с прежним текстом | `test_no_admissible_combination_keeps_the_previous_refusal` |
| счётчики ветки пусты там, где ветка не входилась | `test_counters_are_empty_when_the_branch_is_not_entered` |
| ветка не входится на корпусе | `test_the_branch_is_never_entered_on_the_named_corpus` |
| каждый точный знак ветки идёт под бюджетом транзакции | `test_every_exact_sign_of_the_branch_runs_under_the_named_budget` |
"""

from __future__ import annotations

import json
from collections import Counter
from dataclasses import replace
from functools import cache
from fractions import Fraction
from itertools import product
from pathlib import Path

import pytest

from cftuv_envelope import AnalysisSnapshotCodecV1, DecalRequestCodecV1
from cftuv_envelope.wavefront import conveyor as conveyor_module
from cftuv_envelope.wavefront import faces as faces_module
from cftuv_envelope.wavefront import prepare_conveyor
from cftuv_envelope.wavefront.conveyor import ConveyorOutcome
from cftuv_envelope.exact_sqrt_sum import exact_work_budget
from cftuv_envelope.wavefront.faces import (
    CrowdedChainsV1,
    FaceOutcome,
    FaceV1,
    RegionBoundaryV1,
    _CrowdedFace,
    _Variant,
    build_faces_traced,
    contour_crossings,
    head_tail_paths,
    orientation,
    pairing_defects,
    participant_seats,
    settle_crowded,
    unpaired_off_boundary,
)
from cftuv_envelope.wavefront import build_skeleton
from cftuv_envelope.wavefront.polygon import PolygonV1
from cftuv_envelope.wavefront.sqrt_sum import SqrtSumV1

from wavefront_cases import named_corpus

FIXTURE = Path(__file__).resolve().parents[1] / "fixtures" / "building_patch17_crowded_v1"
DENSITIES = (2, 3, 4)
#: Ребро, у которого правило смежности отказывало: `L0e2` (его лицо — первое у ветки).
FIRST_CROWDED_EDGE = (673740, 137852, 0, 262144)


@cache
def _snapshot():
    return AnalysisSnapshotCodecV1.loads((FIXTURE / "analysis_snapshot.json").read_bytes())


@cache
def _prepared(density: int):
    request = DecalRequestCodecV1.loads(
        (FIXTURE / f"decal_request_density{density}.json").read_bytes()
    )
    manifest = json.loads((FIXTURE / "manifest.json").read_text(encoding="utf-8"))
    (domain,) = [
        item
        for item in _snapshot().patch_domains
        if item.patch_domain_id.value == manifest["patch_domain_ids"][0]
    ]
    return prepare_conveyor(
        _snapshot(), request, patch_domain_id=domain.patch_domain_id
    )


def _region(density: int):
    (region,) = _prepared(density).regions
    return region


def _rebuild(density: int):
    """Грани того же скелета заново — без подготовки домена (она в десятки раз дороже)."""

    region = _region(density)
    return build_faces_traced(region.bridge.polygon, region.skeleton)


# --------------------------------------------------------------------------
# 1. Домен собирается, и все объявленные границы держатся
# --------------------------------------------------------------------------


@pytest.mark.parametrize("density", DENSITIES)
def test_the_wedged_hole_domain_assembles(density):
    """Патч, отказывавший на d2..d4, теперь EXACT, и его границы 1–3 проверены.

    Границы спрашиваются у САМОГО разбиения, а не пересказываются из исхода:
    контур каждой грани прост, площадь каждой строго положительна, сумма равна
    площади многоугольника ТОЧНО. Счётчики ветки — измеренные числа: две грани,
    по два пути, четыре комбинации, выбрана одна.
    """

    prepared = _prepared(density)
    region = _region(density)
    partition = region.partition
    assert prepared.outcome is ConveyorOutcome.EXACT, prepared.detail
    assert partition.outcome is FaceOutcome.EXACT
    assert partition.every_contour_is_simple
    assert partition.every_face_is_positive
    assert partition.area_reproduces_polygon
    assert partition.area_defect.is_zero
    assert all(contour_crossings(face.points) == () for face in partition.faces)
    assert [item for item in prepared.counters if "CROWDED" in item[0]] == [
        ("CONVEYOR_CROWDED_CHAIN_ATTEMPTS", 2),
        ("CONVEYOR_CROWDED_CHAIN_PATHS", 4),
        ("CONVEYOR_CROWDED_CHAIN_COMBINATIONS", 4),
        ("CONVEYOR_CROWDED_CHAIN_SELECTED", 1),
    ]


@pytest.mark.parametrize("density", DENSITIES)
def test_the_selected_chains_are_the_ones_pairing_and_area_agree_on(density):
    """Пересборка тех же узлов даёт то же разбиение: выбор детерминирован."""

    again, trace = _rebuild(density)
    assert again.outcome is FaceOutcome.EXACT
    assert again.faces == _region(density).partition.faces
    assert (trace.faces, trace.paths, trace.combinations, trace.selected) == (2, 4, 4, 1)


# --------------------------------------------------------------------------
# 2. Граница 4, проверенная геометрией на итоговом разбиении
# --------------------------------------------------------------------------


def _rational_point(point):
    return (SqrtSumV1.rational(point[0]), SqrtSumV1.rational(point[1]))


def _dot_sign(origin, toward, point):
    """Знак `(point - origin) . (toward - origin)`: на какой стороне от `origin`."""

    value = SqrtSumV1.zero()
    for axis in (0, 1):
        value = value + (point[axis] - origin[axis]) * (toward[axis] - origin[axis])
    return value.sign()


def _lies_on_boundary(polygon, first, second) -> bool:
    """Отрезок целиком лежит на одном ребре границы области. Точно, без допусков."""

    for loop in polygon.loops:
        points = loop.points
        for index in range(len(points)):
            start = _rational_point(points[index])
            end = _rational_point(points[(index + 1) % len(points)])
            if start == end:
                continue
            if orientation(start, end, first) or orientation(start, end, second):
                continue
            if all(
                _dot_sign(start, end, item) >= 0 and _dot_sign(end, start, item) >= 0
                for item in (first, second)
            ):
                return True
    return False


def _boundary_violations(polygon, faces) -> list[str]:
    """Нарушения границы 4 у набора граней, найденные ГЕОМЕТРИЕЙ, а не ключами продукта.

    Парность в продукте считается по каноническим ключам точек и признакам «по ту
    сторону есть грань». Здесь тот же вопрос задан иначе: сегменты берутся прямо
    из контуров граней, у каждого считается число граней, где он стоит в ту же
    сторону и в обратную, а «нет пары» проверяется СВОЙСТВОМ ПОЛИГОНА — отрезок
    лежит на ребре границы области. Верно тогда и только тогда:

    * встречный стоит ровно в одной грани, сам стоит ровно в одной — сегмент
      внутренний и границы не касается;
    * встречного нет вовсе — сегмент лежит на границе области (опорное ребро
      либо дуга вдоль стены).
    """

    directed: Counter = Counter()
    ends: dict = {}
    for face in faces:
        points = face.points
        for index in range(len(points)):
            first = points[index]
            second = points[(index + 1) % len(points)]
            key = (
                (first[0].terms, first[1].terms),
                (second[0].terms, second[1].terms),
            )
            if key[0] == key[1]:
                continue
            directed[key] += 1
            ends[key] = (first, second)
    violations: list[str] = []
    for (first, second), count in directed.items():
        if count != 1:
            violations.append(f"сегмент в {count} гранях в одну сторону")
            continue
        reverse = directed[(second, first)]
        on_boundary = _lies_on_boundary(polygon, *ends[(first, second)])
        if reverse and on_boundary:
            violations.append("парный сегмент лежит на границе области")
        elif not reverse and not on_boundary:
            violations.append("сегмент без пары вне границы области")
    return violations


@pytest.mark.parametrize("density", DENSITIES)
def test_every_unpaired_segment_lies_on_the_polygon_boundary(density):
    """ГРАНИЦА 4 на итоговом разбиении: нарушений нет, и спрошено геометрией."""

    region = _region(density)
    assert _boundary_violations(region.bridge.polygon, region.partition.faces) == []


@pytest.mark.parametrize("density", DENSITIES)
def test_the_geometric_check_rejects_every_wrong_combination(density, monkeypatch):
    """Контроль самой проверки: на трёх отвергнутых комбинациях она ЗАЯВЛЯЕТ нарушение.

    Проверка, которая молчит на любом входе, ничего не доказывает. Здесь те же
    грани области, но ветка `crowded` собрана неверными путями (три комбинации из
    четырёх), и геометрическая проверка обязана найти нарушения; на выбранной —
    ни одного.
    """

    captured = {}
    original = faces_module.settle_crowded

    def spy(pending, *rest):
        captured["pending"] = pending
        return original(pending, *rest)

    monkeypatch.setattr(faces_module, "settle_crowded", spy)
    region = _region(density)
    partition, _ = _rebuild(density)
    pending = captured["pending"]
    keys = {item.key for item in pending}
    fixed = [face for face in partition.faces if face.owner not in keys]
    verdicts = []
    for combination in product(*(item.variants for item in pending)):
        faces = fixed + [variant.face for variant in combination]
        verdicts.append(len(_boundary_violations(region.bridge.polygon, faces)))
    assert len(verdicts) == 4
    assert sorted(verdicts)[0] == 0
    assert sum(1 for item in verdicts if item == 0) == 1
    assert all(item > 0 for item in sorted(verdicts)[1:])


# --------------------------------------------------------------------------
# 3. Два независимых критерия выбирают одну и ту же комбинацию
# --------------------------------------------------------------------------


@pytest.mark.parametrize("density", DENSITIES)
def test_pairing_boundary_and_area_each_pick_the_same_single_combination(
    density, monkeypatch
):
    """Из четырёх комбинаций путей парность, граница области и площадь оставляют ОДНУ и ту же.

    Критерии независимы: парность читает сегменты и признак стены, граница
    области — ГЕОМЕТРИЮ сегментов без пары (точные знаки), площадь — сумму
    `SqrtSumV1`. Совпадение трёх слепых друг к другу проверок на ровно одной из
    четырёх — и есть довод, что выбор не подогнан под ответ. Числа дефектов у
    отвергнутых комбинаций ненулевые (в замере d2: 12, 24, 20).
    """

    pending, fixed_total, fixed_segments, polygon_area, boundary = _capture_settle(
        density, monkeypatch
    )
    target = SqrtSumV1.rational(polygon_area)
    rows = []
    for combination in product(*(item.variants for item in pending)):
        every = fixed_segments + [s for v in combination for s in v.segments]
        total = fixed_total
        for variant in combination:
            total = total + variant.face.doubled_area
        rows.append(
            (
                pairing_defects(every),
                unpaired_off_boundary(every, boundary),
                (total - target).is_zero,
            )
        )
    assert len(rows) == 4
    by_pairing = [index for index, (defects, _, _) in enumerate(rows) if defects == 0]
    by_boundary = [index for index, (_, off, _) in enumerate(rows) if off == 0]
    by_area = [index for index, (_, _, exact) in enumerate(rows) if exact]
    assert by_pairing == by_boundary == by_area
    assert len(by_pairing) == 1
    assert all(
        defects > 0 and off > 0 for defects, off, exact in rows if not exact
    )


# --------------------------------------------------------------------------
# 4. Неоднозначность, потолки и прежний отказ
# --------------------------------------------------------------------------


def test_several_admissible_combinations_are_named_ambiguous(monkeypatch):
    """Два одинаково допустимых пути — `FACE_CHAIN_AMBIGUOUS`, а не первый попавшийся.

    Неоднозначность внесена настоящей данностью, а не ярлыком: у каждой грани
    ветки её верный путь выдан ДВАЖДЫ (два вхождения одного и того же контура, как
    дали бы два совпавших по геометрии пути). У каждой грани тогда три варианта,
    комбинаций девять, допустимых четыре (верный путь каждой грани в любой из двух
    копий), и сборщик обязан сказать это числом, а не выбрать одну.
    """

    chosen = list(_region(2).partition.faces)
    original = faces_module.crowded_face

    def with_the_chosen_twice(*args, **kwargs):
        found = original(*args, **kwargs)
        if found is None:
            return None
        picked = tuple(
            variant
            for variant in found.variants
            if any(variant.face == face for face in chosen)
        )
        assert len(picked) == 1
        return replace(found, variants=found.variants + picked)

    monkeypatch.setattr(faces_module, "crowded_face", with_the_chosen_twice)
    partition, trace = _rebuild(2)
    assert partition.outcome is FaceOutcome.FACE_CHAIN_AMBIGUOUS
    assert partition.faces == ()
    assert (
        "комбинаций 9, по парности 4, по границе области 4, по площади 4, "
        "допустимых 4"
    ) in partition.detail
    assert (trace.faces, trace.combinations, trace.selected) == (2, 9, 0)


@pytest.mark.parametrize(
    ("cap", "value", "needle"),
    [
        ("CROWDED_PATH_CAP", 1, "оборван потолком"),
        ("CROWDED_STEP_CAP", 3, "оборван потолком"),
        ("CROWDED_COMBINATION_CAP", 2, "оборван потолком"),
    ],
)
def test_every_cap_turns_the_branch_into_a_named_ambiguity(
    cap, value, needle, monkeypatch
):
    """Перебор, упёршийся в потолок, не вправе утверждать «ровно одна».

    Каждый из трёх потолков ловится своим сжатием: путей — до одного (у граней
    их два), шагов — до трёх, комбинаций — до двух (их четыре). Ответ один:
    `FACE_CHAIN_AMBIGUOUS` и число в `detail`.
    """

    monkeypatch.setattr(faces_module, cap, value)
    partition, trace = _rebuild(2)
    assert partition.outcome is FaceOutcome.FACE_CHAIN_AMBIGUOUS
    assert needle in partition.detail
    assert partition.faces == ()
    assert trace.selected == 0


def test_no_admissible_combination_keeps_the_previous_refusal(monkeypatch):
    """Ни одной допустимой — `FACE_CHAIN_DOES_NOT_CLOSE` с ПРЕЖНИМ текстом отказа.

    Граница 4 сломана намеренно (каждый сегмент объявлен дефектным): допустимых
    комбинаций нет, и отказ обязан начинаться с той же строки, что и до ветки, —
    ребро и число участников в трёх точках, — а перебор лишь дописывает числа в
    хвост.
    """

    monkeypatch.setattr(faces_module, "pairing_defects", lambda segments: 1)
    partition, trace = _rebuild(2)
    assert partition.outcome is FaceOutcome.FACE_CHAIN_DOES_NOT_CLOSE
    # 11, а не 13: угол выше 90 градусов на d2 держит ОДИН канонический луч, а не
    # два лифтованных (FAN-CANONICAL-COUNT), и участников в точках стало на два меньше.
    assert partition.detail.startswith(
        f"ребро {FIRST_CROWDED_EDGE[:2]} -> {FIRST_CROWDED_EDGE[2:]}: "
        "1 участников в трёх и более точках из 11: "
    )
    assert "допустимых 0" in partition.detail
    assert partition.faces == ()
    assert (trace.faces, trace.combinations, trace.selected) == (2, 4, 0)


# --------------------------------------------------------------------------
# 5. Счётчики и звенья по отдельности
# --------------------------------------------------------------------------


def test_counters_are_empty_when_the_branch_is_not_entered():
    """Ветка не входилась — счётчиков нет вовсе, а не четыре нуля.

    Структурные счётчики подготовки заморожены тестами и воротами равенства ответа
    домена; нули на каждом из доменов, где ветка молчит, сдвинули бы их все.
    """

    assert faces_module.CrowdedChainsV1().counters() == ()
    assert conveyor_module._crowded_counters(()) == ()


def test_the_branch_is_never_entered_on_the_named_corpus():
    """На корпусе фигур ветка молчит: ни одной грани, ни одного пути, ни счётчика.

    Ветка заменила отказ, а не правило сборки, и пока ни одна грань не отказала,
    результат побитово тот, что и был. Фигур корпуса отказ не касался никогда
    (`test_wavefront_faces.py`, замороженный ноль); здесь то же утверждение про
    САМУ ветку: след пуст.
    """

    for name, polygon in named_corpus():
        skeleton = build_skeleton(polygon)
        partition, trace = build_faces_traced(polygon, skeleton)
        assert trace == CrowdedChainsV1(), name
        assert partition.outcome is FaceOutcome.EXACT, name


def test_every_exact_sign_of_the_branch_runs_under_the_named_budget(monkeypatch):
    """Точные знаки ветки (контуры путей, площади путей) платят из бюджета домена.

    Ветка зовёт `contour_crossings` и `sign` для каждого пути ДО выбора, и каждый
    такой вызов обязан нести бюджет транзакции: безбюджетный знак при сопряжении
    уходит в факторизацию мимо потолка. Знаки перехвачены на самом классе, чтобы
    увидеть и те, что идут через `orientation`.
    """

    region = _region(2)
    budget = exact_work_budget(stage="GUARD", domain_id="patch-17")
    seen: list = []
    original = SqrtSumV1.sign

    def spy(self, **kwargs):
        seen.append(kwargs.get("budget"))
        return original(self, **kwargs)

    monkeypatch.setattr(SqrtSumV1, "sign", spy)
    partition, trace = build_faces_traced(
        region.bridge.polygon, region.skeleton, budget
    )
    assert partition.outcome is FaceOutcome.EXACT
    assert trace.faces == 2
    assert seen, "ветка не спросила ни одного знака"
    assert all(item is budget for item in seen)


def test_head_tail_paths_walks_both_ways_round_a_cycle_and_respects_the_caps(monkeypatch):
    """Цикл из четырёх точек даёт два пути head -> tail; потолки обрывают перебор.

    Точки 0..3 по кольцу a(0-1), b(1-2), c(2-3), d(3-0); голова — точка 0 (там
    `f`), хвосты — 1 и 3 (там `p`). Из головы в хвост через ВСЕ точки ровно два
    обхода — по часовой и против.
    """

    partners = [
        {"a", "d", "f"},
        {"a", "b", "p"},
        {"b", "c"},
        {"c", "d", "p"},
    ]
    shared, crowded = participant_seats(partners)
    assert crowded == []
    paths, exhausted = head_tail_paths(partners, shared, "p", "f")
    assert sorted(paths) == [(0, 1, 2, 3), (0, 3, 2, 1)]
    assert not exhausted
    monkeypatch.setattr(faces_module, "CROWDED_PATH_CAP", 1)
    paths, exhausted = head_tail_paths(partners, shared, "p", "f")
    assert len(paths) == 1 and exhausted
    monkeypatch.setattr(faces_module, "CROWDED_PATH_CAP", 16)
    monkeypatch.setattr(faces_module, "CROWDED_STEP_CAP", 2)
    paths, exhausted = head_tail_paths(partners, shared, "p", "f")
    assert exhausted


def test_pairing_defects_names_each_way_a_segment_can_break_the_boundary():
    """Граница 4 на минимальных входах: каждое нарушение даёт ненулевое число."""

    assert pairing_defects([("A", "B", True), ("B", "A", True)]) == 0
    assert pairing_defects([("A", "B", False)]) == 0  # на границе области
    assert pairing_defects([("A", "B", True)]) == 1  # парный обязан быть
    assert pairing_defects([("A", "B", False), ("B", "A", True)]) == 1  # лишний парный
    assert pairing_defects(
        [("A", "B", True), ("A", "B", True), ("B", "A", True)]
    ) == 3  # один сегмент в двух гранях в одну сторону


# --------------------------------------------------------------------------
# 6. Вторая половина границы 4 исполняет ЯДРО, а не тест
# --------------------------------------------------------------------------


def _capture_settle(density, monkeypatch):
    """Аргументы, с которыми ядро зовёт `settle_crowded` на фикстуре."""

    captured = {}
    original = faces_module.settle_crowded

    def spy(*args):
        captured["args"] = args
        return original(*args)

    monkeypatch.setattr(faces_module, "settle_crowded", spy)
    _rebuild(density)
    return captured["args"]


@pytest.mark.parametrize("density", DENSITIES)
def test_on_the_fixture_every_segment_that_differs_between_path_variants_is_paired(
    density, monkeypatch
):
    """ПОСЫЛКА выбора по парности: различаются только сегменты, у которых пара ЕСТЬ.

    Парность различает пути лишь там, где у сегмента обязана быть парная грань.
    Если бы между путями одной грани различались сегменты без пары (дуга вдоль
    стены), парность была бы к ним слепа, и выбор держался бы на площади и на
    второй половине границы. На фикстуре это не так: все различающиеся сегменты
    `mirrored=True`, а сегменты без пары у всех путей грани общие. Утверждение
    заморожено, потому что от него зависит, на чём стоит выбор.
    """

    pending = _capture_settle(density, monkeypatch)[0]
    assert len(pending) == 2
    for item in pending:
        assert len(item.variants) == 2
        sets = [set(variant.segments) for variant in item.variants]
        differing = set.union(*sets) - set.intersection(*sets)
        assert differing
        assert all(mirrored for _, _, mirrored in differing)


def _key(x, y):
    return (SqrtSumV1.rational(x).terms, SqrtSumV1.rational(y).terms)


def _wall_arc_face(arc_end):
    """Синтетическая грань ветки, у которой участник зажатия — СТЕНА `x = 8`.

    Квадрат 8 x 8, опорное ребро — нижнее. Дуга между опорным ребром и стеной не
    имеет парной грани (у стены грани нет), поэтому её сегмент объявлен без
    пары. Две копии грани различаются ТОЛЬКО концом этой дуги; площадь у них
    одна (`32`, а остальное в `fixed_total` дополняет область до `128`).
    """

    area = SqrtSumV1.rational(32)
    owner = (0, 0, 8, 0)
    face = FaceV1(owner, (0, 0), (8, 0), (), area)
    return face, (_key(0, 0), _key(8, 0), False), (_key(8, 0), arc_end, False)


def _synthetic_pending(*arc_ends):
    variants = []
    for arc_end in arc_ends:
        face, support, arc = _wall_arc_face(arc_end)
        variants.append(_Variant(face, (support, arc)))
    return [_CrowdedFace(face.owner, "синтетика", 2, False, tuple(variants))]


def _settle_square(arc_ends):
    polygon = PolygonV1.build(((0, 0), (8, 0), (8, 8), (0, 8)))
    return settle_crowded(
        _synthetic_pending(*arc_ends),
        SqrtSumV1.rational(96),
        [],
        128,
        RegionBoundaryV1.of(polygon),
    )


def test_the_boundary_half_rejects_an_unpaired_segment_that_pairing_and_area_cannot_see(
    monkeypatch,
):
    """Путь с «дугой вдоль стены», режущей область насквозь, отвергает ГЕОМЕТРИЯ.

    Два варианта одной грани различаются концом дуги без пары: у верного он на
    стене `x = 8` (`(8, 3)`), у неверного внутри области (`(5, 3)`) при том же
    признаке «стена». Парность у обоих нулевая (пары не требуется и нет), площадь
    у обоих сходится. БЕЗ второй половины границы выбора нет — ровно та дыра,
    о которой сказал аудит; С ней остаётся верный. Контроль: с выключенной
    геометрией отвечает `FACE_CHAIN_AMBIGUOUS`, то есть различает варианты именно
    она.
    """

    right, wrong = _key(8, 3), _key(5, 3)
    pending = _synthetic_pending(wrong, right)
    for variant in pending[0].variants:
        assert pairing_defects(list(variant.segments)) == 0
        total = variant.face.doubled_area + SqrtSumV1.rational(96)
        assert (total - SqrtSumV1.rational(128)).is_zero

    chosen, refusal, tried = _settle_square((wrong, right))
    assert refusal is None and tried == 2
    assert chosen[0].segments[1][1] == right

    with monkeypatch.context() as patched:
        patched.setattr(faces_module, "unpaired_off_boundary", lambda *_: 0)
        chosen, refusal, tried = _settle_square((wrong, right))
    assert chosen is None
    assert refusal[0] is FaceOutcome.FACE_CHAIN_AMBIGUOUS
    assert "допустимых 2" in refusal[1]


def test_when_every_variant_leaves_the_boundary_the_previous_refusal_stays():
    """Все пути с сегментом без пары вне границы — прежний `FACE_CHAIN_DOES_NOT_CLOSE`."""

    chosen, refusal, tried = _settle_square((_key(5, 3), _key(4, 4)))
    assert chosen is None and tried == 2
    assert refusal[0] is FaceOutcome.FACE_CHAIN_DOES_NOT_CLOSE
    assert "синтетика" in refusal[1]
    assert "по границе области 0" in refusal[1]
    assert "допустимых 0" in refusal[1]


def test_the_boundary_predicate_is_exact_and_asks_for_containment_not_just_collinearity():
    """Предикат границы: точные знаки на корнях, вложенность в ребро, а не только прямая.

    Гипотенуза `(8, 0) -> (0, 8)` лежит на `x + y = 8`. Точки `(4 + sqrt2,
    4 - sqrt2)` и `(4 - sqrt2, 4 + sqrt2)` ей принадлежат ТОЧНО; сдвиг на
    `10^-12` — уже нет, и допуска, который его пропустил бы, нет. Отрезок на той
    же прямой, но за концом ребра, на границе не лежит.
    """

    polygon = PolygonV1.build(((0, 0), (8, 0), (0, 8)))
    boundary = RegionBoundaryV1.of(polygon)
    root = SqrtSumV1.radical(1, 2)
    four = SqrtSumV1.rational(4)

    def point(x, y):
        return (x.terms, y.terms)

    first = point(four + root, four - root)
    second = point(four - root, four + root)
    assert boundary.holds(first, second)
    assert boundary.holds(first, _key(8, 0))
    assert not boundary.holds(first, _key(9, -1))
    shift = SqrtSumV1.rational(Fraction(1, 10**12))
    assert not boundary.holds(first, point(four - root, four + root + shift))
    assert not boundary.holds(_key(1, 1), _key(2, 2))
    assert boundary.holds(_key(0, 0), _key(4, 0))
    assert boundary.holds(_key(4, 0), _key(0, 0))


def test_unpaired_off_boundary_counts_only_segments_without_a_partner():
    """Парный сегмент не спрашивается: геометрию платит лишь сегмент без встречного."""

    polygon = PolygonV1.build(((0, 0), (8, 0), (8, 8), (0, 8)))
    boundary = RegionBoundaryV1.of(polygon)
    inside = (_key(2, 2), _key(5, 3), True)
    back = (_key(5, 3), _key(2, 2), True)
    edge = (_key(0, 0), _key(8, 0), False)
    assert unpaired_off_boundary([inside, back, edge], boundary) == 0
    assert unpaired_off_boundary([inside, edge], boundary) == 1
    assert unpaired_off_boundary([(_key(2, 2), _key(5, 3), False)], boundary) == 1


# --------------------------------------------------------------------------
# 7. Поздний отказ после грани ветки называет собственную причину
# --------------------------------------------------------------------------

#: Подвижное ребро ПОСЛЕ обеих граней ветки: порядок рёбер — порядок скелета.
LATE_EDGE = (262144, 0, -60628, 59544)


@pytest.mark.parametrize("mode", ["no_nodes", "one_node_lost"])
def test_a_late_edge_refusal_after_a_branch_face_keeps_its_own_reason(mode, monkeypatch):
    """Отказ ребра ПОСЛЕ грани ветки: ответ прежний, `detail` дополнен его причиной.

    До правки `detail` был строкой первой грани ветки, и отказ позднего ребра
    (`FACE_HAS_NO_SKELETON_NODE` либо его собственная цепочка) прятался за ней.
    Исход остаётся `FACE_CHAIN_DOES_NOT_CLOSE` и текст начинается с грани ветки,
    как раньше, но дальше названы исход и причина самого позднего ребра.
    Ребро портится в самом наборе узлов: нет ни одного либо потерян один.
    """

    region = _region(2)
    original = faces_module._nodes_by_owner
    seen = {}

    def doctored(skeleton):
        by_key, error = original(skeleton)
        if mode == "no_nodes":
            by_key.pop(LATE_EDGE)
        else:
            by_key[LATE_EDGE] = by_key[LATE_EDGE][:-1]
            seen["nodes"] = tuple(by_key[LATE_EDGE])
        return by_key, error

    monkeypatch.setattr(faces_module, "_nodes_by_owner", doctored)
    partition, trace = _rebuild(2)
    assert partition.outcome is FaceOutcome.FACE_CHAIN_DOES_NOT_CLOSE
    assert partition.faces == ()
    assert (trace.faces, trace.paths, trace.combinations, trace.selected) == (2, 4, 0, 0)
    branch = (
        f"ребро {FIRST_CROWDED_EDGE[:2]} -> {FIRST_CROWDED_EDGE[2:]}: "
        "1 участников в трёх и более точках из 11: "
    )
    assert partition.detail.startswith(branch)
    span = f"ребро {LATE_EDGE[:2]} -> {LATE_EDGE[2:]}"
    if mode == "no_nodes":
        late = f"(FACE_HAS_NO_SKELETON_NODE): {span} без узлов"
    else:
        previous, following = faces_module.edge_neighbours(region.bridge.polygon)[
            LATE_EDGE
        ]
        chain, why = faces_module.face_chain(
            LATE_EDGE, seen["nodes"], previous, following
        )
        assert chain is None
        late = f"(FACE_CHAIN_DOES_NOT_CLOSE): {span}: {why}"
    assert partition.detail.endswith(f"; затем отказ другого ребра {late}")
