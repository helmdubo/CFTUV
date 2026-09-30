"""`overlay_signature` с памятью представлений: тот же кортеж, тот же порядок.

Подпись наложения сортирует вершины, пролёты и листья по `repr`. Память
представлений (`signature_memo`) считает `repr` неизменяемой части раз на
объект, а `repr` кортежа собирает из текстов частей так же, как это делает
`tuple.__repr__`. Это ЦЕНА, а не семантика, поэтому эталон — исходное тело
функции (копия ниже): подпись обязана совпасть и по значению, и по `repr`
(порядок элементов — часть `repr`), внутри памяти и вне её.
"""

from __future__ import annotations

from dataclasses import dataclass
from fractions import Fraction

import pytest

from cftuv_envelope.exact_sqrt_sum import (
    SqrtSumV1,
    reset_factorization_memory,
    unlimited_reference_budget,
)
from cftuv_envelope.wavefront import symbolic_component as component
from cftuv_envelope.wavefront.event_time import EventPointV1, EventTimeV1
from cftuv_envelope.wavefront.superlevel_closure import (
    SegmentRefV1,
    SpanFamilyRefV1,
)
from cftuv_envelope.wavefront.symbolic_overlay import (
    JunctionRefV1,
    SymbolicOverlayV1,
    SymbolicSpanBindingV1,
    SymbolicVertexV1,
)


@pytest.fixture(autouse=True)
def _cold_state():
    reset_factorization_memory()
    yield
    reset_factorization_memory()


def reference_signature(overlay):
    """Исходное тело `overlay_signature` до замены: сортировка `key=repr`."""

    def trace_authority(trace):
        if trace is None:
            return ("UNAVAILABLE",)
        crash_time = trace.crash_time
        if crash_time is None:
            return ("TRACE_WITHOUT_CRASH",)
        return ("BOUNDED", crash_time.canonical())

    vertices = tuple(sorted((
        (
            v.ref, v.prev, v.next, v.prev_leaf, v.next_leaf,
            v.birth.canonical(), v.point, v.sliding, v.provenance,
            trace_authority(v.trace),
        )
        for v in overlay.vertices.values() if v.alive
    ), key=repr))
    spans = tuple(sorted((
        (leaf, item.physical_edge_id, item.start, item.end)
        for leaf, item in overlay.spans.items()
    ), key=repr))
    return vertices, spans, tuple(sorted(overlay.changed, key=repr))


@dataclass(frozen=True, slots=True)
class FakeTrace:
    crash_time: object


BUDGET = unlimited_reference_budget()


def _time(dividend, divisor):
    return EventTimeV1.normalized(Fraction(dividend), divisor, BUDGET)


def _root(coefficient, radicand):
    return SqrtSumV1.radical(Fraction(coefficient), radicand, BUDGET)


def _leaf(index):
    family = SpanFamilyRefV1(
        (index, Fraction(index, 7)), ((index,), (index + 1,))
    )
    return SegmentRefV1(family, None, None, (index, index + 1))


def _ref(index):
    return JunctionRefV1(
        "EXISTING", (index, SqrtSumV1.rational(Fraction(index, 3)).terms)
    )


def build_overlay(count=9):
    vertices = {}
    for index in range(count):
        ref = _ref(index)
        point = EventPointV1(
            _root(index + 1, 2) + SqrtSumV1.rational(Fraction(1, index + 2)),
            _root(Fraction(index + 3, 5), 3),
        )
        birth = _time(index + 1, _root(index + 2, 5) + SqrtSumV1.rational(3))
        trace = (
            None if index % 3 == 0
            else FakeTrace(None) if index % 3 == 1
            else FakeTrace(_time(index + 4, _root(7, 11) + SqrtSumV1.rational(2)))
        )
        vertices[ref] = SymbolicVertexV1(
            ref,
            _ref((index + 1) % count),
            None if index % 4 == 0 else _ref((index + 2) % count),
            _leaf(index),
            _leaf(index + 1),
            birth,
            point,
            None if index % 2 else _root(index + 2, 7),
            frozenset({("A", index), ("B", index + 1)}),
            trace=trace,
            alive=index != 5,
        )
    spans = {
        _leaf(index): SymbolicSpanBindingV1(
            _leaf(index), 100 + index, _ref(index), None
        )
        for index in range(count)
    }
    return SymbolicOverlayV1(
        vertices, spans, {_leaf(2), _leaf(0), _leaf(7)}, _time(1, SqrtSumV1.rational(1))
    )


def test_signature_equals_reference_outside_the_memo():
    overlay = build_overlay()

    signature = component.overlay_signature(overlay)

    assert signature == reference_signature(overlay)
    assert repr(signature) == repr(reference_signature(overlay))


def test_signature_equals_reference_inside_the_memo_and_on_clones():
    overlay = build_overlay()
    clones = [component.clone_overlay(overlay) for _ in range(3)]

    with component.signature_memo():
        results = [component.overlay_signature(item) for item in (overlay, *clones)]

    expected = reference_signature(overlay)
    for signature in results:
        assert signature == expected
        assert repr(signature) == repr(expected)


def test_dead_vertices_and_changed_leaves_are_reflected():
    overlay = build_overlay()
    with component.signature_memo():
        before = component.overlay_signature(overlay)
        next(iter(overlay.vertices.values())).alive = False
        overlay.changed.add(_leaf(3))
        after = component.overlay_signature(overlay)

    assert len(after[0]) == len(before[0]) - 1
    assert len(after[2]) == len(before[2]) + 1
    assert repr(after) == repr(reference_signature(overlay))


def test_memo_computes_each_shared_part_once(monkeypatch):
    calls = []
    original = EventTimeV1.canonical

    def counting(self):
        calls.append(1)
        return original(self)

    monkeypatch.setattr(EventTimeV1, "canonical", counting)
    overlay = build_overlay()
    clone = component.clone_overlay(overlay)

    with component.signature_memo():
        component.overlay_signature(overlay)
        first = len(calls)
        component.overlay_signature(clone)
        assert len(calls) == first
    component.overlay_signature(clone)
    assert len(calls) == first + first


def test_memo_scope_closes_and_nested_scopes_share_one_memo():
    assert component._SIGNATURE_MEMO.get() is None
    with component.signature_memo():
        outer = component._SIGNATURE_MEMO.get()
        assert outer is not None
        with component.signature_memo():
            assert component._SIGNATURE_MEMO.get() is outer
        assert component._SIGNATURE_MEMO.get() is outer
    assert component._SIGNATURE_MEMO.get() is None
    with pytest.raises(RuntimeError):
        with component.signature_memo():
            raise RuntimeError("scope must close on error")
    assert component._SIGNATURE_MEMO.get() is None
