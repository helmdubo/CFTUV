"""HOLE_CHANNEL_V1: дыра домена с ползущими стенами вливается в контур каналом в своей тени.

Один выделенный шов на патче с окном: грань единственного источника обходит окно кругом и не односвязна
(`FACE_CHAIN_DOES_NOT_CLOSE`). Канал от дальней от источника вершины окна до контура делает её односвязной, а
покрытие ближе начала канала остаётся тем же, что у домена без окна.
"""

from __future__ import annotations

from dataclasses import replace
from fractions import Fraction

import pytest

from cftuv_envelope.robust.grid import GridSpecV1
from cftuv_envelope.wavefront import bridge as bridge_module
from cftuv_envelope.wavefront.bridge import BridgeOutcome, bridge_arrival_laws, unit_speed_laws_of
from cftuv_envelope.wavefront.coverage import coverage_at
from cftuv_envelope.wavefront.faces import FaceOutcome, build_faces
from cftuv_envelope.wavefront.hole_channel import channel_holes
from cftuv_envelope.wavefront.polygon import LoopV1, PolygonV1
from cftuv_envelope.wavefront.skeleton import SkeletonOutcome, build_skeleton
from cftuv_envelope.wavefront.wall_creep import drop_wall_faces

LATTICE = GridSpecV1(scale=64)
OUTER = ((0, 0), (40, 0), (40, 30), (0, 30))
WINDOW = ((15, 10), (15, 20), (25, 20), (25, 10))  # дыра — по часовой
SECOND = ((30, 22), (30, 26), (36, 26), (36, 22))


def _loops(*loops):
    return tuple(tuple((Fraction(x), Fraction(y)) for x, y in loop) for loop in loops)


def _bottom_law():
    """Закон нижнего ребра (выделенный шов) с концами опоры, как его отдаёт конвейер."""

    polygon = PolygonV1(LoopV1(OUTER, (1600, 0, 0, 0)))
    law = unit_speed_laws_of(polygon)[0]
    return replace(law, span=((Fraction(0), Fraction(0)), (Fraction(40), Fraction(0))), motion=(0.0, 1.0))


def _solve(loops):
    report = bridge_arrival_laws(loops, (_bottom_law(),), lattice=LATTICE, weighted_fronts=True, creeping_walls=True, hole_channels=True)
    assert report.outcome is BridgeOutcome.EXACT, report.findings
    skeleton = build_skeleton(report.polygon)
    if skeleton.outcome is not SkeletonOutcome.EXACT:
        return report, skeleton, None
    return report, skeleton, drop_wall_faces(build_faces(report.polygon, skeleton), report)


@pytest.mark.parametrize("holes", [(WINDOW,), (WINDOW, SECOND)], ids=["one window", "two windows"])
def test_a_window_is_channelled_and_the_partition_closes(holes):
    report, skeleton, partition = _solve(_loops(OUTER, *holes))
    assert report.hole_channel_count == len(holes) and report.hole_channel_refused_count == 0
    assert len(report.polygon.loops) == 1
    assert partition is not None and partition.outcome is FaceOutcome.EXACT


def test_without_the_channel_the_source_face_wraps_the_window_and_is_refused_by_name(monkeypatch):
    monkeypatch.setattr(bridge_module, "channel_holes", lambda *args: None)
    _report, skeleton, partition = _solve(_loops(OUTER, WINDOW))
    assert skeleton.outcome is not SkeletonOutcome.EXACT or partition.outcome is not FaceOutcome.EXACT


def test_the_channel_starts_in_the_shadow_of_the_window():
    """Канал уходит от дальней от источника вершины окна и не трогает источник: начало — не ближе дальнего угла окна."""

    report, _skeleton, _partition = _solve(_loops(OUTER, WINDOW))
    # якорь — ближайшая к окну точка источника (15, 0); дальний от него угол окна — (25, 20)
    far_corner = (10**2 + 20**2) ** 0.5
    assert report.hole_channel_min_distance_micro == pytest.approx(far_corner * 1e6, rel=1e-6)
    source = [(start, end) for start, end, q in report.polygon.edges() if q and start[1] == end[1] == 0]
    assert source == [((0, 0), (40 * LATTICE.scale, 0))]


@pytest.mark.parametrize("alpha", [Fraction(1), Fraction(5)])
def test_coverage_closer_than_the_window_is_the_coverage_without_the_window(alpha):
    _r, _s, channelled = _solve(_loops(OUTER, WINDOW))
    _r, _s, plain = _solve(_loops(OUTER))
    scaled = alpha * LATTICE.scale
    assert coverage_at(channelled, scaled).doubled_area == coverage_at(plain, scaled).doubled_area


def test_no_holes_or_no_span_means_no_channel():
    law = _bottom_law()
    assert channel_holes(_loops(OUTER), (law,), frozenset(), LATTICE) is None
    assert channel_holes(_loops(OUTER, WINDOW), (replace(law, span=None),), frozenset(), LATTICE) is None


def test_stationary_walls_never_channel():
    """Без ползущих стен (первый проход, все швы выделены) дыры остаются дырами: ответ прежний."""

    report = bridge_arrival_laws(_loops(OUTER, WINDOW), (_bottom_law(),), lattice=LATTICE, weighted_fronts=True)
    assert report.hole_channel_count == 0 and len(report.polygon.loops) == 2


def test_creeping_walls_alone_never_channel():
    """Каналы — третий проход: второй (ползущие стены) дыры не трогает, и закрывшийся им домен остаётся прежним."""

    report = bridge_arrival_laws(
        _loops(OUTER, WINDOW), (_bottom_law(),), lattice=LATTICE, weighted_fronts=True, creeping_walls=True
    )
    assert report.hole_channel_count == 0 and len(report.polygon.loops) == 2


def test_a_hole_that_carries_a_source_is_not_cut():
    """Выделенный шов на контуре окна: контур окна — фронт, а не стена; канал его не режет и назван отказом."""

    window = _loops(OUTER, WINDOW)
    hole = window[1]
    edge = (hole[0], hole[1])
    law = replace(_bottom_law(), span=edge)
    made = channel_holes(window, (law,), frozenset(), LATTICE)
    assert made.channel_count == 0 and made.refused_count == 1 and made.loops == window
