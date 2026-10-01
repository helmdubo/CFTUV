"""Сборка батча: десятичный контекст задан, имя вершины не теряется молча.

Две правки аудита 2026-10-02 (предложения 3 и 5), и обе про то, что раньше
решалось неявно: контекст `Decimal` брался у окружения, а имя `src:` — у первого
региона, назвавшего точку.
"""

from __future__ import annotations

import decimal
from fractions import Fraction
from types import SimpleNamespace

import pytest

from cftuv_envelope.exact_sqrt_sum import SqrtSumV1
from cftuv_envelope.materialize import assemble
from cftuv_envelope.materialize.admit import MaterializationOutcome
from cftuv_envelope.materialize.frames import MaterializationRefusal

import materialize_factories as factories


def _irrational_values():
    """Значения, у которых середина оболочки длиннее 28 цифр: контекст виден."""

    guard = factories.budget()
    return (
        SqrtSumV1.radical(1, 2, guard),
        SqrtSumV1.radical(1, 3, guard),
        SqrtSumV1.radical(7, 5, guard) + SqrtSumV1.rational(Fraction(1, 3)),
    )


def test_the_decimal_context_is_pinned_and_not_inherited():
    context = assemble.DECIMAL_CONTEXT
    assert context.prec == assemble.DECIMAL_DIGITS == 28
    assert context.rounding == decimal.ROUND_HALF_EVEN
    assert (context.Emin, context.Emax, context.capitals, context.clamp) == (
        -999999,
        999999,
        1,
        0,
    )
    trapped = {signal for signal, enabled in context.traps.items() if enabled}
    assert trapped == {
        decimal.InvalidOperation,
        decimal.DivisionByZero,
        decimal.Overflow,
    }


@pytest.mark.parametrize("divisor", (1, 3, 131072, 4096))
def test_decimal_of_ignores_the_ambient_decimal_context(divisor):
    """Чужой `prec`, режим округления и ловушки ответа не меняют ни цифры."""

    hostile = (
        decimal.Context(prec=5, rounding=decimal.ROUND_DOWN, traps=[]),
        decimal.Context(prec=60, rounding=decimal.ROUND_UP, traps=[decimal.Inexact]),
        decimal.Context(prec=3, rounding=decimal.ROUND_CEILING, Emin=-3, Emax=3),
    )
    for value in _irrational_values():
        reference = assemble.decimal_of(value, divisor)
        # 28 значащих цифр: контекст «по умолчанию» читал их так же.
        assert len(reference.as_tuple().digits) <= 28
        for ambient in hostile:
            with decimal.localcontext(ambient) as active:
                assert assemble.decimal_of(value, divisor) == reference
                assert active.prec == ambient.prec  # окружение не тронуто
        # Тот же ответ и под тем же значением, посчитанным без ловушек.
        assert assemble.decimal_of(value, divisor) == reference
    # Закреплённый контекст не накапливает флаги между вызовами: работает копия.
    assert not any(assemble.DECIMAL_CONTEXT.flags.values())


def test_the_old_prec_only_context_really_moved_the_answer():
    """Контроль чувствительности: у прежней записи (одна `prec`) цифра ехала.

    Без него предыдущий тест мог бы проходить на значениях, у которых режим
    округления не виден вовсе.
    """

    from cftuv_envelope.materialize.lift import ENCLOSURE_BITS

    moved = 0
    for value in _irrational_values():
        low, high = value.enclosure(ENCLOSURE_BITS)
        middle = (low + high) / 2
        with decimal.localcontext(decimal.Context(rounding=decimal.ROUND_UP)) as old:
            old.prec = 28  # прежний код: задавалась только точность
            legacy = decimal.Decimal(middle.numerator) / decimal.Decimal(
                middle.denominator
            )
        moved += legacy != assemble.decimal_of(value, 1)
    assert moved >= 1


# --------------------------------------------------------------------------
# Имя вершины при сварке точек
# --------------------------------------------------------------------------


def _point(x, y):
    return (SqrtSumV1.rational(Fraction(x)), SqrtSumV1.rational(Fraction(y)))


def _item(region, *points):
    return (region, SimpleNamespace(face=SimpleNamespace(points=tuple(_point(*p) for p in points))))


def _table(**names):
    """`{(регион, узел): имя вершины}`; ключ задаётся строкой `region|x|y`."""

    mapping = {}
    for spec, vertex_id in names.items():
        region, x, y = spec.split("_")
        mapping[(region, (int(x), int(y)))] = vertex_id
    return SimpleNamespace(node_vertex_ids=mapping)


TRIANGLE = ((0, 0), (4, 0), (0, 4))


def test_a_point_named_once_keeps_its_name_and_reports_nothing():
    table = _table(a_0_0="v0", a_4_0="v1", a_0_4="v2")
    notes: list = []
    cycles, points = assemble.intern_vertices(
        [_item("a", *TRIANGLE), _item("a", *TRIANGLE)], table, notes
    )
    assert [key for key, _ in cycles[0]] == ["src:v0", "src:v1", "src:v2"]
    assert cycles[0] == cycles[1] and set(points) == {"src:v0", "src:v1", "src:v2"}
    assert notes == []


def test_a_second_source_vertex_on_a_welded_point_is_named_not_dropped_silently():
    """Те же точки, а регион `b` зовёт их ДРУГИМИ вершинами исходника."""

    table = _table(
        a_0_0="v0", a_4_0="v1", a_0_4="v2", b_0_0="w0", b_4_0="v1", b_0_4="w2"
    )
    notes: list = []
    cycles, _points = assemble.intern_vertices(
        [_item("a", *TRIANGLE), _item("b", *TRIANGLE), _item("b", *TRIANGLE)],
        table,
        notes,
    )
    # Первый регион выиграл, сварка сохранена: вершина одна.
    assert [key for key, _ in cycles[1]] == ["src:v0", "src:v1", "src:v2"]
    # Две отброшенные пары названы ОДИН раз каждая (а не по числу вхождений).
    assert len(notes) == 2
    assert all(note.startswith(assemble.SOURCE_VERTEX_NAME_DROPPED) for note in notes)
    assert any("w0" in note and "src:v0" in note for note in notes)
    assert any("w2" in note and "src:v2" in note for note in notes)


def test_a_name_that_arrives_after_an_unnamed_first_occurrence_is_also_reported():
    table = _table(b_0_0="v0")
    notes: list = []
    cycles, _points = assemble.intern_vertices(
        [_item("a", *TRIANGLE), _item("b", *TRIANGLE)], table, notes
    )
    assert cycles[0][0][0].startswith("node:")
    assert len(notes) == 1 and "v0" in notes[0] and "node:" in notes[0]


def test_two_different_points_under_one_source_name_are_a_named_refusal():
    """Раньше точка ключа молча перезаписывалась: геометрия слипалась."""

    table = _table(a_0_0="v0", a_4_0="v0")
    with pytest.raises(MaterializationRefusal) as refusal:
        assemble.intern_vertices([_item("a", *TRIANGLE)], table)
    assert refusal.value.outcome is MaterializationOutcome.BATCH_DID_NOT_VALIDATE
    assert "VERTEX_KEY_COLLISION: src:v0" in refusal.value.detail
