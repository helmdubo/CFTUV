"""Память сертификата вложения привязки источника (`_embedding.EMBEDDING_MEMO_LIMIT`).

Сертификат - O(n^2) точных предикатов по ВСЕМУ патчу (на полусфере в 512 граней - ~35 с, около 90 % метрики домена),
а нажатие кнопки зовёт его шесть раз над ОДНИМИ числами: на ступенях near-planar и развёртки лестницы целого патча,
карты запроса и суженной карты. Память отвечает по ЗНАЧЕНИЯМ всех аргументов; попадание обязано быть побитово равно
промаху и расчёту без памяти. Исполняемое доказательство:

| что утверждается                                                         | тест |
|--------------------------------------------------------------------------|------|
| три лестницы одного нажатия сертифицируют ОДНИ входы - расчёт один        | `test_the_three_ladders_of_one_press_certify_the_same_inputs_and_compute_once` |
| попадание = промах = расчёт без памяти (байты снапшота и текст отказа)    | `test_a_memo_hit_is_byte_identical_to_a_miss_and_to_no_memo` |
| каждый вход сертификата - часть ключа; то же значение - та же запись      | `test_every_input_of_the_certificate_is_part_of_the_key` |
| память ограничена, вытесняется давнее; нулевой предел её выключает        | `test_the_memo_is_bounded_by_recency_and_a_zero_limit_switches_it_off` |
"""

from __future__ import annotations

from dataclasses import dataclass
from fractions import Fraction

import pytest

import cftuv_envelope as kernel
from cftuv_envelope import _embedding
from cftuv_envelope._embedding import (
    build_source_snap_embedding_certificate,
    clear_embedding_memo,
    embedding_memo_limit,
    embedding_memo_stats,
)
from cftuv_envelope.planar_metric import PlanarMetricAdmissionError

import band_factories as factories


@dataclass(frozen=True)
class _Face:
    """Грань в том, что сертификат читает: номер, цикл вершин и цикл рёбер."""

    face_id: object
    vertex_cycle: tuple
    edge_cycle: tuple


def _vertex(index):
    return kernel.SourceVertexId(f"v{index}")


def _square():
    """Единичный квадрат из двух треугольников: `(позиции, грани)`; рёбра `e0..e4`, диагональ `e4`."""

    corners = ((0, 0, 0), (1, 0, 0), (1, 1, 0), (0, 1, 0))
    positions = {_vertex(index): tuple(Fraction(axis) for axis in point) for index, point in enumerate(corners)}
    edge = lambda index: kernel.PhysicalEdgeId(f"e{index}")  # noqa: E731
    faces = (
        _Face(kernel.SourceFaceId("f0"), (_vertex(0), _vertex(1), _vertex(2)), (edge(0), edge(1), edge(4))),
        _Face(kernel.SourceFaceId("f1"), (_vertex(0), _vertex(2), _vertex(3)), (edge(4), edge(2), edge(3))),
    )
    return positions, faces


def _certificate(before, after, faces, intended=(), unclassifiable=(), law=kernel.GridSnappingLawV1.SOURCE_ONLY_GRID_SNAP_V1):
    return build_source_snap_embedding_certificate(
        before=before,
        after=after,
        faces=faces,
        intended_corners=intended,
        unclassifiable_corners=unclassifiable,
        snapping_law=law,
    )


def _refused_or_bytes(parts, cap):
    """Снапшот полосы побайтно либо названный отказ: то, что сравнивают память и расчёт без неё."""

    try:
        snapshot, _request, _band = factories.band_domain(parts, reach_cap=cap)
    except PlanarMetricAdmissionError as error:
        return ("refused", error.outcome.value, str(error))
    return kernel.AnalysisSnapshotCodecV1.dumps(snapshot)


def test_the_three_ladders_of_one_press_certify_the_same_inputs_and_compute_once():
    """Целый патч, карта запроса и суженная карта - три лестницы одного нажатия - над одними позициями и гранями."""

    with embedding_memo_limit(8):
        for cap in (Fraction(1, 2), Fraction(3, 10), None):  # карта запроса, суженная карта, целый патч
            try:
                factories.band_domain(factories.dome_ring(sides=8, rings=4), reach_cap=cap)
            except PlanarMetricAdmissionError:
                pass  # исход лестницы не важен: важен вход сертификата
        found = embedding_memo_stats()
    # Шесть обращений (две ступени на лестницу), один расчёт: входы всех трёх лестниц побитово те же.
    assert found["misses"] == 1 and found["hits"] == 5 and found["entries"] == 1, found


@pytest.mark.parametrize(
    "name, parts, cap",
    (
        ("disk band", lambda: factories.dome_rim(), "1/2"),
        ("ring: column", lambda: factories.column_top(rows=3), "1/2"),
        ("ring: cone", lambda: factories.frustum_ring(segments=12, rows=8), "1/2"),
        ("ring: dome", lambda: factories.dome_ring(sides=16, rings=8), "1/4"),
        ("ring: dome refused by the seam", lambda: factories.dome_ring(sides=16, rings=8), "1/2"),
        ("whole patch refused", lambda: factories.column_top(rows=3), None),
    ),
)
def test_a_memo_hit_is_byte_identical_to_a_miss_and_to_no_memo(name, parts, cap):
    with embedding_memo_limit(0):
        reference = _refused_or_bytes(parts(), cap)
    with embedding_memo_limit(8):
        miss = _refused_or_bytes(parts(), cap)
        before = embedding_memo_stats()["hits"]
        hit = _refused_or_bytes(parts(), cap)
        assert embedding_memo_stats()["hits"] > before, name
    assert reference == miss == hit, name


def test_every_input_of_the_certificate_is_part_of_the_key():
    positions, faces = _square()
    snapped = {**positions, _vertex(2): (Fraction(1), Fraction(1), Fraction(1, 1000))}
    corner = (_vertex(0), _vertex(1), _vertex(2))
    with embedding_memo_limit(8):
        base = _certificate(positions, positions, faces, intended=(corner,))
        assert _certificate(positions, positions, faces, intended=(corner,)) is base  # то же значение - та же запись
        assert embedding_memo_stats() == {"hits": 1, "misses": 1, "entries": 1}
        # Равная КОПИЯ словаря позиций - то же значение, а не другой вход.
        assert _certificate(dict(positions), dict(positions), faces, intended=(corner,)) is base
        variants = (
            ("coordinate before", {**positions, _vertex(1): (Fraction(2), Fraction(0), Fraction(0))}, None, faces, (corner,), (), None),
            ("coordinate after", positions, snapped, faces, (corner,), (), None),
            ("faces", positions, None, faces[:1], (corner,), (), None),
            ("intended corners", positions, None, faces, (), (), None),
            ("unclassifiable corners", positions, None, faces, (corner,), (corner,), None),
            ("snapping law", positions, None, faces, (corner,), (), kernel.GridSnappingLawV1.INTEGER_GRID_SNAP_V1),
        )
        misses = embedding_memo_stats()["misses"]
        for name, before, after, face_set, intended, unclassifiable, law in variants:
            result = _certificate(
                before,
                before if after is None else after,
                face_set,
                intended=intended,
                unclassifiable=unclassifiable,
                **({} if law is None else {"law": law}),
            )
            misses += 1
            assert embedding_memo_stats()["misses"] == misses, name
            assert result is not base, name


def test_the_memo_is_bounded_by_recency_and_a_zero_limit_switches_it_off():
    positions, faces = _square()

    def other(shift):
        return {**positions, _vertex(3): (Fraction(0), Fraction(1), Fraction(shift))}

    with embedding_memo_limit(2):
        first = _certificate(other(1), other(1), faces)
        _certificate(other(2), other(2), faces)
        _certificate(other(1), other(1), faces)  # освежает первую
        _certificate(other(3), other(3), faces)  # вытесняет вторую (давнюю), не первую
        assert embedding_memo_stats()["entries"] == 2
        hits = embedding_memo_stats()["hits"]
        assert _certificate(other(1), other(1), faces) is first and embedding_memo_stats()["hits"] == hits + 1
        misses = embedding_memo_stats()["misses"]
        _certificate(other(2), other(2), faces)
        assert embedding_memo_stats()["misses"] == misses + 1
    with embedding_memo_limit(0):
        one = _certificate(positions, positions, faces)
        two = _certificate(positions, positions, faces)
        assert one == two and one is not two
        assert embedding_memo_stats() == {"hits": 0, "misses": 0, "entries": 0}
    clear_embedding_memo()
    assert _embedding.EMBEDDING_MEMO_LIMIT == 8


@pytest.mark.parametrize("limit, unhashable, expected", [(0, False, 2), (8, False, 1), (8, True, 2)])
def test_dispatch_only_replaces_the_three_leaf_paths_and_preserves_memo_identity(monkeypatch, limit, unhashable, expected):
    from cftuv_envelope import backend
    positions, faces = _square()
    calls, certificate = [], object()
    def dispatch(*args):
        calls.append(args)
        return certificate
    monkeypatch.setattr(backend, "embedding_compute", dispatch)
    intended = ([_vertex(0), _vertex(1), _vertex(2)],) if unhashable else ()
    with embedding_memo_limit(limit):
        with backend.use_backend("PYTHON", "PYTHON", "NATIVE") as ledger:
            first = _certificate(positions, positions, faces, intended=intended)
            second = _certificate(positions, positions, faces, intended=intended)
        assert first is second is certificate and len(calls) == expected
        assert ledger.record().embedding_cache_hits == (1 if expected == 1 else 0)
        assert not ledger.record().embedding_native_calls and not ledger.record().embedding_python_calls


def test_existing_python_memo_hit_is_same_object_under_native_and_never_calls_leaf(monkeypatch):
    from cftuv_envelope import backend
    positions, faces = _square()
    with embedding_memo_limit(8):
        python = _certificate(positions, positions, faces)
        before = embedding_memo_stats()
        monkeypatch.setattr(backend, "embedding_compute", lambda *a: pytest.fail("memo hit must not call leaf"))
        with backend.use_backend("PYTHON", "PYTHON", "NATIVE") as ledger:
            assert _certificate(positions, positions, faces) is python
        assert ledger.record().embedding_cache_hits == 1 and not ledger.record().embedding_ran
        assert ledger.record().embedding_native_calls == ledger.record().embedding_python_calls == 0
        assert embedding_memo_stats() == {**before, "hits": before["hits"] + 1}
    backend.note_embedding_cache_hit()  # вне scope — no-op, не заводит память и журнал
