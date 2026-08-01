"""Закон растворения прямых вершин: предикат и его четыре красных контроля.

Растворение убирает вершину, которой геометрически нет: у стыка двух
коллинеарных СОНАПРАВЛЕННЫХ рёбер граница есть одна прямая, разрезанная лишним
узлом. Узел этот стоит дважды — в скелете он рождает вершину третьего вида
(развёрнутый угол, `_Vertex.sliding`, своя ветка счёта места и времён), а в
мосте удваивает блок класса несущей прямой, из-за чего один закон на два ребра
даёт `undetermined` и фронт не идёт вовсе.

Тесты здесь проверяют ЗАКОН на чистой геометрии, до всякой плумбинга: предикат,
стоп-лист и два случая, в которых растворять нельзя. Красные колонки названы
поимённо в карточке (R1–R4).
"""

from __future__ import annotations

from fractions import Fraction

import pytest

from cftuv_envelope.wavefront.bridge import PlainArrivalLawV1, line_class
from cftuv_envelope.wavefront.dissolve import (
    DISSOLVE_STRAIGHT_LAW_V3,
    dissolve_straight_vertices,
    is_reversal,
    is_straight_turn,
    law_counts_by_class,
)


def _p(x, y):
    return (Fraction(x), Fraction(y))


#: Квадрат 4x4 с ЛИШНЕЙ прямой вершиной (2,0) на нижней стороне.
SQUARE_WITH_STRAIGHT_VERTEX = (
    _p(0, 0),
    _p(2, 0),
    _p(4, 0),
    _p(4, 4),
    _p(0, 4),
)

#: Закон прихода нижней стороны `y = 0`: нормаль (0,1), константа 0.
def _bottom_law(name: str) -> PlainArrivalLawV1:
    return PlainArrivalLawV1(name, Fraction(0), Fraction(1), Fraction(0), Fraction(1))


def _edge_class(tail, head):
    dx, dy = head[0] - tail[0], head[1] - tail[1]
    a, b = -dy, dx
    return line_class((a, b, a * tail[0] + b * tail[1]))


# --------------------------------------------------------------------------
# Предикат: точный ноль, без допуска, и без разворота.
# --------------------------------------------------------------------------


def test_the_straight_turn_predicate_is_exact_and_has_no_tolerance():
    """Почти-прямая вершина — ДРУГАЯ геометрия, и растворять её нельзя.

    Допуск здесь был бы не оптимизацией, а сглаживанием молча: он удалял бы
    вершины, которые вход объявил существующими. Предикат целочисленно-дробный,
    поэтому «почти» отличимо от «точно» на любом масштабе.
    """

    assert is_straight_turn(_p(0, 0), _p(2, 0), _p(4, 0)) is True
    assert is_straight_turn(_p(2, 0), _p(4, 0), _p(4, 4)) is False
    # Отклонение в одну миллиардную — уже не прямая вершина.
    almost = (Fraction(2), Fraction(1, 10**9))
    assert is_straight_turn(_p(0, 0), almost, _p(4, 0)) is False


def test_a_reversal_is_a_zero_turn_but_never_dissolvable():
    """R4. У шпильки поворот тоже нулевой, а геометрия другая.

    Ход назад по той же прямой даёт нулевое векторное произведение, но
    отрицательное скалярное. Слияние склеило бы две стороны нулевой площади в
    ничто, то есть удалило бы не запись геометрии, а саму геометрию.
    """

    assert is_straight_turn(_p(0, 0), _p(2, 0), _p(0, 0)) is False
    assert is_reversal(_p(0, 0), _p(2, 0), _p(0, 0)) is True


def test_the_merged_edge_keeps_the_line_class_of_both_originals():
    """Доказательство, вынесенное в исполняемую форму.

    При `cross = 0` и `dot > 0` второе направление есть `k·d` при `k > 0`,
    нормаль второго ребра `k·n`, а константа `n·v = n·(p + d) = n·p`, потому что
    `n ⊥ d`. Значит тройка второго ребра пропорциональна первой с положительным
    множителем — тот самый `_proportional_positive`, то есть ОДИН класс. Отсюда
    и следует, что слияние не двигает сопоставление по классам.
    """

    first = _edge_class(_p(0, 0), _p(2, 0))
    second = _edge_class(_p(2, 0), _p(4, 0))
    merged = _edge_class(_p(0, 0), _p(4, 0))
    assert first == second == merged


# --------------------------------------------------------------------------
# R1–R3: растворение и два случая, когда оно запрещено.
# --------------------------------------------------------------------------


def test_a_straight_wall_vertex_dissolves_and_the_loop_loses_exactly_one_point():
    """R1. Прямая вершина стены растворяется, геометрия границы та же.

    Законов на классе нет, значит оба ребра — стена, и слияние двух стен даёт
    стену: сопоставление не двигается вовсе, а узел из скелета уходит.
    """

    loops, report = dissolve_straight_vertices(
        (SQUARE_WITH_STRAIGHT_VERTEX,), laws=(), protected_points=frozenset()
    )
    assert len(loops[0]) == len(SQUARE_WITH_STRAIGHT_VERTEX) - 1
    assert _p(2, 0) not in loops[0]
    assert report.law == DISSOLVE_STRAIGHT_LAW_V3
    assert report.dissolved_count == 1
    assert report.dissolved_points == (_p(2, 0),)
    assert report.changed is True
    # Остальные вершины на месте и в прежнем порядке обхода.
    assert loops[0] == (_p(0, 0), _p(4, 0), _p(4, 4), _p(0, 4))


def test_a_vertex_named_by_an_anchor_is_never_dissolved():
    """R2. Стоп-лист якорей сильнее предиката. Ссылка есть — не растворять.

    Якорь веера, конец chain use, граница владельца и вершина junction суть
    ИМЕНОВАННЫЕ места: растворить названное место значит потерять имя, а не
    узел. Исключений у этого правила нет.
    """

    loops, report = dissolve_straight_vertices(
        (SQUARE_WITH_STRAIGHT_VERTEX,),
        laws=(),
        protected_points=frozenset({_p(2, 0)}),
    )
    assert loops[0] == SQUARE_WITH_STRAIGHT_VERTEX
    assert report.dissolved_count == 0
    assert report.anchored_count == 1
    assert report.changed is False


def test_two_laws_on_one_line_class_hold_the_vertex():
    """R3. Разные законы по сторонам — вершина неприкосновенна.

    Два ребра и два закона в классе допускают однозначное сопоставление
    один-к-одному. Слияние оставило бы одно ребро на два закона, то есть
    превратило бы законный источник в `surplus`, — это потеря, а не упрощение.

    Условие записано СЧЁТОМ законов в классе, а не сравнением «закона левого
    ребра» с «законом правого»: `PlainArrivalLawV1` не имеет концов вовсе и
    ключуется ПРЯМОЙ, а какое ребро класса чей закон — ровно то, что мост
    называет незаписанным во входе.
    """

    laws = (_bottom_law("srcA"), _bottom_law("srcB"))
    assert set(law_counts_by_class(laws).values()) == {2}

    loops, report = dissolve_straight_vertices(
        (SQUARE_WITH_STRAIGHT_VERTEX,), laws=laws, protected_points=frozenset()
    )
    assert loops[0] == SQUARE_WITH_STRAIGHT_VERTEX
    assert report.dissolved_count == 0
    assert report.contested_count == 1


def test_one_law_on_the_class_dissolves_and_can_only_improve_ownership():
    """Один закон на класс — растворение разрешено и полезно.

    До слияния блок класса состоял из двух рёбер при одном законе, то есть был
    `undetermined`: «источником является не всякое ребро, а какое именно — во
    входе не записано», и фронт не шёл. После слияния блок из одного ребра при
    одном законе есть ВЫНУЖДЕННАЯ пара. Число сопоставленных рёбер при этом не
    меняется (`min(1, B)` и `min(1, B−1)` при `B ≥ 2` равны единице), поэтому
    регрессии нет ни при каком размере блока.
    """

    laws = (_bottom_law("srcA"),)
    loops, report = dissolve_straight_vertices(
        (SQUARE_WITH_STRAIGHT_VERTEX,), laws=laws, protected_points=frozenset()
    )
    assert len(loops[0]) == 4
    assert report.dissolved_count == 1
    assert report.contested_count == 0


def test_a_hairpin_inside_a_loop_is_counted_and_kept():
    """R4 в составе петли: шпилька считается отдельно от «занятых» вершин."""

    spike = (_p(0, 0), _p(4, 0), _p(2, 0), _p(4, 4), _p(0, 4))
    loops, report = dissolve_straight_vertices(
        (spike,), laws=(), protected_points=frozenset()
    )
    assert loops[0] == spike
    assert report.reversal_count == 1
    assert report.dissolved_count == 0


@pytest.mark.parametrize(
    "loop",
    (
        (_p(0, 0), _p(2, 0), _p(4, 0)),
        (_p(0, 0), _p(1, 0)),
        (),
    ),
    ids=["triangle", "segment", "empty"],
)
def test_loops_shorter_than_four_points_are_never_touched(loop):
    """Треугольник с прямой вершиной есть вырожденный отрезок.

    Растворить её значило бы поменять не запись геометрии, а саму геометрию:
    от петли осталось бы две вершины. Порог записан явно, а не выведен из
    последствий.
    """

    loops, report = dissolve_straight_vertices(
        (loop,), laws=(), protected_points=frozenset()
    )
    assert loops[0] == loop
    assert report.dissolved_count == 0
