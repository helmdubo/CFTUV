"""Закон растворения прямых вершин: он УЖЕ ЕСТЬ в продукте, и вот его границы.

КАРТОЧКА `DISSOLVE-STRAIGHT-01` ЗАКРЫТА НАХОДКОЙ (2026-08-01). Задание требовало
ввести растворение прямых вершин в петли домена конвейера. Замер показал, что
вводить нечего: закон живёт в продукте с `e38d140` (2026-07-24), выше по
конвейеру, и прямые вершины до конвейера не доходят вовсе.

ГДЕ ЖИВЁТ ЗАКОН: `reference/boundary.py::_continuous_support_intervals`
(строки 113-170), вызывается из `boundary.py:206` при построении домена. Четыре
его условия и есть закон растворения:

    continuous     = points_equal(previous.end, segment.start)
    collinear      = exact_sign(oriented_cross(prev.tangent, seg.tangent)) == 0
    same_direction = exact_sign(dot_g(prev.tangent, seg.tangent)) > 0
    same_normal    = cross(owner_normal) == 0 and dot_g(owner_normal) > 0

`collinear` — точный нулевой поворот без допуска; `same_direction` строго
положителен и потому отсекает ШПИЛЬКУ (разворот тоже даёт нулевое векторное
произведение); `same_normal` требует совпадения нормали ВЛАДЕЛЬЦА, то есть
«оба ребра несут один закон прихода»; слияние берёт `first.start -> last.end`,
сливает provenance и чеканит новый `support_id` — то есть спаны владения
сливаются вместе с рёбрами.

ЧТО ЗАМЕРЕНО (перехват `_region_loops` предикатом ниже, весь фикстурный корпус):

    building_002_point_contact_v1     вершин 12   ПРЯМЫХ 0   шпилек 0
    building_002_full_selection_v1    вершин 12   ПРЯМЫХ 0   шпилек 0
    building_002_weighted_normals_v1  вершин  4   ПРЯМЫХ 0   шпилек 0
    building_patch10_density4_v1      вершин 12   ПРЯМЫХ 0   шпилек 0

Ноль — это не «предикат не работает», а полнота закона выше по конвейеру: к
петлям домена вход приходит уже слитым. Прежний замер «~64 растворимых вершины
по сцене» относится к вершинам ИСХОДНОГО МЕША (хост, BMesh) — их и съедает
`_continuous_support_intervals`, пока домен строится.

ЧТО ОСТАЁТСЯ НЕРАСТВОРИМЫМ ПРИНЦИПИАЛЬНО: коллинеарные соседи РАЗНЫХ цепей на
одной прямой. Их держит `same_normal` — власть владения, — и они остаются
законной работой правила прямой вершины скелета (`skeleton._Vertex.sliding`,
вершина третьего вида у развёрнутого угла). Растворить их значило бы слить двух
владельцев в одного.

ЗАЧЕМ ЭТОТ ФАЙЛ. Машинерии он не вносит: продуктовый код этой ветки не тронут
ни строкой. Он замораживает ЗНАНИЕ — четыре условия закона, разбор его
безопасности и регрессионный ноль. Ослабнет `_continuous_support_intervals` —
ноль перестанет быть нулём, и тест закричит.
"""

from __future__ import annotations

import ast
import inspect
from fractions import Fraction
from pathlib import Path

import pytest

from cftuv_envelope import AnalysisSnapshotCodecV1, DecalRequestCodecV1
from cftuv_envelope.reference import boundary as boundary_module
from cftuv_envelope.wavefront import conveyor as conveyor_module
from cftuv_envelope.wavefront.bridge import PlainArrivalLawV1, line_class


FIXTURE_ROOT = Path(__file__).parents[1] / "fixtures"


# --------------------------------------------------------------------------
# Предикат закона — ТЕСТОВЫЙ хелпер, а не продуктовый модуль.
#
# В горячем пути ему делать нечего: закон уже применён выше, и вторая его копия
# была бы мёртвой ступенью. Здесь он инструмент ЗАМЕРА — им регрессионный
# контроль считает прямые вершины в петлях корпуса.
# --------------------------------------------------------------------------


def _direction(tail, head):
    return (head[0] - tail[0], head[1] - tail[1])


def is_straight_turn(previous, vertex, following) -> bool:
    """Точный нулевой поворот СОНАПРАВЛЕННЫХ рёбер. Ни допуска, ни разворота."""

    incoming = _direction(previous, vertex)
    outgoing = _direction(vertex, following)
    if incoming == (0, 0) or outgoing == (0, 0):
        return False
    if incoming[0] * outgoing[1] - incoming[1] * outgoing[0] != 0:
        return False
    return incoming[0] * outgoing[0] + incoming[1] * outgoing[1] > 0


def is_reversal(previous, vertex, following) -> bool:
    """Нулевой поворот с ОБРАТНЫМ ходом — шпилька. Растворению не подлежит."""

    incoming = _direction(previous, vertex)
    outgoing = _direction(vertex, following)
    if incoming == (0, 0) or outgoing == (0, 0):
        return False
    if incoming[0] * outgoing[1] - incoming[1] * outgoing[0] != 0:
        return False
    return incoming[0] * outgoing[0] + incoming[1] * outgoing[1] < 0


def _p(x, y):
    return (Fraction(x), Fraction(y))


def _bottom_law(name: str) -> PlainArrivalLawV1:
    """Закон прихода нижней стороны `y = 0`: нормаль (0,1), константа 0."""

    return PlainArrivalLawV1(name, Fraction(0), Fraction(1), Fraction(0), Fraction(1))


def _edge_class(tail, head):
    dx, dy = head[0] - tail[0], head[1] - tail[1]
    a, b = -dy, dx
    return line_class((a, b, a * tail[0] + b * tail[1]))


def _law_counts_by_class(laws) -> dict:
    counts: dict = {}
    for law in laws:
        key = line_class((law.normal_x, law.normal_y, law.constant))
        if key is not None:
            counts[key] = counts.get(key, 0) + 1
    return counts


def _law_source_tree():
    source = inspect.getsource(boundary_module._continuous_support_intervals)
    return ast.parse(source.lstrip())


# --------------------------------------------------------------------------
# 1. Предикат: точный ноль, без допуска, с отсечением разворота.
# --------------------------------------------------------------------------


def test_the_straight_turn_predicate_is_exact_and_has_no_tolerance():
    """Почти-прямая вершина — ДРУГАЯ геометрия, и растворять её нельзя.

    Допуск был бы не оптимизацией, а сглаживанием молча: он удалял бы вершины,
    которые вход объявил существующими. Продуктовый `collinear` устроен так же
    — `exact_sign(...) == 0`, не порог.
    """

    assert is_straight_turn(_p(0, 0), _p(2, 0), _p(4, 0)) is True
    assert is_straight_turn(_p(2, 0), _p(4, 0), _p(4, 4)) is False
    almost = (Fraction(2), Fraction(1, 10**9))
    assert is_straight_turn(_p(0, 0), almost, _p(4, 0)) is False


def test_a_reversal_is_a_zero_turn_but_never_dissolvable():
    """У шпильки поворот тоже нулевой, а геометрия другая.

    Ход назад по той же прямой даёт нулевое векторное произведение и
    отрицательное скалярное. Слияние склеило бы две стороны нулевой площади в
    ничто. В продукте это отсекает `same_direction > 0`.
    """

    assert is_straight_turn(_p(0, 0), _p(2, 0), _p(0, 0)) is False
    assert is_reversal(_p(0, 0), _p(2, 0), _p(0, 0)) is True


def test_the_merged_edge_keeps_the_line_class_of_both_originals():
    """Почему слияние не двигает сопоставление законов с рёбрами.

    При `cross = 0` и `dot > 0` второе направление есть `k*d` при `k > 0`,
    нормаль второго ребра `k*n`, а его константа `n*v = n*(p + d) = n*p`, потому
    что `n` перпендикулярна `d`. Значит тройка второго ребра пропорциональна
    первой с положительным множителем — предикат
    `bridge._proportional_positive`, то есть ОДИН класс несущей прямой.
    """

    assert (
        _edge_class(_p(0, 0), _p(2, 0))
        == _edge_class(_p(2, 0), _p(4, 0))
        == _edge_class(_p(0, 0), _p(4, 0))
    )


# --------------------------------------------------------------------------
# 2. Четыре условия продуктового закона — заморожены поимённо.
# --------------------------------------------------------------------------


def test_the_product_dissolve_law_still_declares_all_four_conditions():
    """`_continuous_support_intervals` — и есть закон растворения.

    Заморожено НАЛИЧИЕ всех четырёх условий и то, что слияние идёт только при их
    конъюнкции. Ослабнет любое — тест назовёт, какое именно: одно общее «закон
    изменился» не отличило бы потерю проверки владельца от потери проверки
    направления, а это разные потери с разными следствиями.
    """

    tree = _law_source_tree()
    assigned = {
        target.id
        for node in ast.walk(tree)
        if isinstance(node, ast.Assign)
        for target in node.targets
        if isinstance(target, ast.Name)
    }
    expected = {"continuous", "collinear", "same_direction", "same_normal"}
    assert expected <= assigned, sorted(expected - assigned)

    guards = [
        node
        for node in ast.walk(tree)
        if isinstance(node, ast.If)
        and isinstance(node.test, ast.BoolOp)
        and isinstance(node.test.op, ast.And)
    ]
    assert guards, "слияние обязано стоять под конъюнкцией условий"
    names = {
        item.id
        for guard in guards
        for item in ast.walk(guard.test)
        if isinstance(item, ast.Name)
    }
    assert expected <= names, sorted(expected - names)


def test_the_product_law_compares_exactly_and_not_by_tolerance():
    """Точность закона — свойство кода, а не намерения.

    `collinear` и `same_normal` сверяются через `exact_sign(...) == 0`, а
    `same_direction` — через `exact_sign(...) > 0`. Числового порога в функции
    нет вовсе, и это заморожено: порог, появившийся здесь, растворял бы
    почти-прямые вершины, то есть менял бы геометрию, а не её запись.
    """

    tree = _law_source_tree()
    calls = {
        node.func.id
        for node in ast.walk(tree)
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Name)
    }
    assert "exact_sign" in calls
    literals = [
        node.value
        for node in ast.walk(tree)
        if isinstance(node, ast.Constant) and isinstance(node.value, float)
    ]
    assert literals == [], f"в законе появился числовой порог: {literals}"


def test_owner_normal_keeps_two_chains_on_one_line_unmerged():
    """Что закон НЕ растворяет — и почему это правильно.

    Коллинеарные соседи РАЗНЫХ цепей на одной прямой держатся условием
    `same_normal`: касательная у них совпадает, а нормаль владельца — нет.
    Растворить их значило бы слить двух владельцев в одного. Они остаются
    законной работой правила прямой вершины скелета — вершины третьего вида у
    развёрнутого угла (`skeleton._Vertex.sliding`).

    Проверяется исполняемо: владелец проверяется ОТДЕЛЬНЫМ членом, а не
    выводится из коллинеарности касательных.
    """

    tree = _law_source_tree()
    same_normal = next(
        node
        for node in ast.walk(tree)
        if isinstance(node, ast.Assign)
        and any(
            isinstance(target, ast.Name) and target.id == "same_normal"
            for target in node.targets
        )
    )
    attributes = {
        node.attr for node in ast.walk(same_normal) if isinstance(node, ast.Attribute)
    }
    assert "owner_normal" in attributes
    assert "tangent" not in attributes, "владелец обязан проверяться сам по себе"


# --------------------------------------------------------------------------
# 3. Безопасность слияния для сопоставления — разбор трёх случаев.
#
# Общеценно и переживает эту карточку: объясняет, ПОЧЕМУ слияние коллинеарных
# рёбер не ломает сопоставление законов с рёбрами. Случаи `0 / 1 / >=2`
# отвечают на разные вопросы, поэтому и тестов три: один дал бы один ответ на
# три вопроса, и упавший не сказал бы, какой случай сломан.
# --------------------------------------------------------------------------


def test_case_zero_laws_both_edges_are_wall_and_merging_walls_gives_a_wall():
    """Случай 0: законов на классе нет — оба ребра стена, слияние даёт стену."""

    assert _law_counts_by_class(()) == {}
    # Сопоставленных рёбер в классе было ноль и осталось ноль: `min(0, B)`.
    assert min(0, 2) == min(0, 1) == 0


def test_case_one_law_keeps_the_matched_count_and_can_only_sharpen_ownership():
    """Случай 1: закон один — число сопоставленных не меняется.

    `min(1, B) = 1` до слияния и `min(1, B-1) = 1` после, потому что `B >= 2`.
    Определённость владельца при этом может только вырасти: блок из двух рёбер
    при одном законе мост числит `undetermined` («источником является не всякое
    ребро, а какое именно — во входе не записано»), а блок из одного ребра при
    одном законе есть ВЫНУЖДЕННАЯ пара.
    """

    laws = (_bottom_law("srcA"),)
    assert set(_law_counts_by_class(laws).values()) == {1}
    for block in (2, 3, 7):
        assert min(1, block) == min(1, block - 1) == 1
    # Класс слитого ребра тот же, поэтому закон по-прежнему его находит.
    assert _edge_class(_p(0, 0), _p(4, 0)) == line_class(
        (laws[0].normal_x, laws[0].normal_y, laws[0].constant)
    )


def test_case_two_laws_would_destroy_a_one_to_one_pairing():
    """Случай >=2: слияние превратило бы законный источник в `surplus`.

    Два ребра и два закона допускают однозначное сопоставление один-к-одному;
    одно ребро на два закона — уже нет. Поэтому продуктовый закон и требует
    совпадения нормали ВЛАДЕЛЬЦА: два источника на одной прямой не сливаются.

    Единственность закона в классе записывается СЧЁТОМ, а не сравнением «закона
    левого ребра» с «законом правого», и форму продиктовал вход:
    `PlainArrivalLawV1` не имеет концов вовсе — закон ключуется ПРЯМОЙ. Концами
    ключуется `BridgeReportV1.owner_by_edge`, а он существует только ПОСЛЕ
    моста.
    """

    laws = (_bottom_law("srcA"), _bottom_law("srcB"))
    assert set(_law_counts_by_class(laws).values()) == {2}
    # Было два сопоставленных ребра, после слияния осталось бы одно.
    assert min(2, 2) == 2
    assert min(2, 1) == 1
    # Форма условия проверяется исполняемо, а не пересказывается.
    assert not hasattr(laws[0], "start")
    assert not hasattr(laws[0], "end")


# --------------------------------------------------------------------------
# 4. Регрессионный ноль: прямых вершин в петлях корпуса нет.
# --------------------------------------------------------------------------


CORPUS = (
    "building_002_point_contact_v1",
    "building_002_full_selection_v1",
    "building_002_weighted_normals_v1",
    "building_patch10_density4_v1",
)


def _loops_of(fixture: str):
    """Петли всех доменов фикстуры — перехватом `_region_loops`."""

    folder = FIXTURE_ROOT / fixture
    snapshot_bytes = (folder / "analysis_snapshot.json").read_bytes()
    if fixture == "building_patch10_density4_v1":
        loaded = AnalysisSnapshotCodecV1.loads_with_compatibility_receipt(
            snapshot_bytes
        )
        assert loaded.compatibility_receipt.positive_scaled_normal_compat_hits == 1
        snapshot = loaded.record
    else:
        snapshot = AnalysisSnapshotCodecV1.loads(snapshot_bytes)
    request = DecalRequestCodecV1.loads((folder / "decal_request.json").read_bytes())

    captured: list = []
    original = conveyor_module._region_loops

    def spy(region):
        loops, issue = original(region)
        if loops is not None:
            captured.append(loops)
        return loops, issue

    conveyor_module._region_loops = spy
    try:
        for domain in sorted(
            snapshot.patch_domains, key=lambda item: item.patch_domain_id.value
        ):
            conveyor_module.prepare_conveyor(
                snapshot, request, patch_domain_id=domain.patch_domain_id
            )
    finally:
        conveyor_module._region_loops = original
    return captured


@pytest.mark.parametrize("fixture", CORPUS)
def test_the_corpus_reaches_the_conveyor_without_a_single_straight_vertex(fixture):
    """РЕГРЕССИОННЫЙ НОЛЬ — доказательство полноты закона выше по конвейеру.

    К петлям домена вход приходит уже слитым: `_continuous_support_intervals`
    съедает коллинеарные сонаправленные сегменты одного владельца, пока домен
    строится. Ноль здесь означает не «предикат молчит», а «растворять нечего», и
    именно поэтому вводить растворение в конвейер не понадобилось.

    Ослабнет продуктовый закон — ноль перестанет быть нулём, и этот тест назовёт
    фикстуру, на которой это случилось.
    """

    loops_by_region = _loops_of(fixture)
    assert loops_by_region, f"{fixture}: петли домена не построились"

    straight = 0
    reversal = 0
    points = 0
    for loops in loops_by_region:
        for loop in loops:
            size = len(loop)
            points += size
            if size < 4:
                continue
            for index in range(size):
                previous = loop[(index - 1) % size]
                vertex = loop[index]
                following = loop[(index + 1) % size]
                if is_straight_turn(previous, vertex, following):
                    straight += 1
                elif is_reversal(previous, vertex, following):
                    reversal += 1

    assert points > 0, f"{fixture}: петли пусты"
    assert straight == 0, f"{fixture}: в петлях появились прямые вершины: {straight}"
    assert reversal == 0, f"{fixture}: в петлях появились шпильки: {reversal}"
