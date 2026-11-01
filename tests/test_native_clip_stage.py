"""CLIP_STAGE_NATIVE_V1: the clip stage around the clip in Rust (`materialize_w1.rs`): the seam edges the clip may not cut (`clip.seam_edges`) and the triangles of the
emitted pieces (`clip.piece_triangles`), against the oracle.

Every call of the cases and of the rung fixtures is replayed on both: the same frozenset with the same iteration order; the same triangles, articles of the budget and
`SIGN_COUNTS`. A polygon without a triangulation is the oracle's refusal with its text; a missing fact is declined by the port and raised by the oracle.
"""

from __future__ import annotations

import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
for path in (ROOT / "kernel" / "src", ROOT / "kernel" / "tests", ROOT / "tests"):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

try:
    import cftuv_native
except ModuleNotFoundError as error:
    if error.name != "cftuv_native":
        raise
    cftuv_native = None

needs_native = pytest.mark.skipif(
    cftuv_native is None or not hasattr(cftuv_native, "piece_triangles"), reason="расширение cftuv_native не собрано (или собрано без CLIP_STAGE_NATIVE_V1)"
)


@pytest.fixture
def approved():
    from cftuv_native import materialize_hot_op as op
    from cftuv_native import pin

    op.refresh()
    pin.refresh()
    assert op.status() == "available", op.status()
    yield
    op.refresh()


def _captured():
    """`(seam calls, piece calls)` of the cases and the rung fixtures (the oracle's run); a piece call keeps the cap and mode of its budget."""

    from cftuv_envelope import backend
    from test_native_chord_stations import _materialize_rung, _rung_domains
    from test_native_surface_lift import _cases, _step

    seams, pieces = [], []
    saved_seam, saved_piece = backend.seam_edges_compute, backend.piece_triangles_compute

    def seam(oracle, *arguments):
        seams.append(arguments)
        return saved_seam(oracle, *arguments)

    def piece(oracle, polygons, points, budget):
        pieces.append((polygons, dict(points), budget.cap, budget.mode))
        return saved_piece(oracle, polygons, points, budget)

    backend.seam_edges_compute, backend.piece_triangles_compute = seam, piece
    try:
        for prepare, laws, alpha in _cases():
            _step(prepare, laws, alpha, "PYTHON")
        for prepared, coverage, _alpha in _rung_domains():
            _materialize_rung(prepared, coverage, "PYTHON")
    finally:
        backend.seam_edges_compute, backend.piece_triangles_compute = saved_seam, saved_piece
    return seams, pieces


def _piece_run(function, polygons, points, cap, mode):
    from cftuv_envelope import exact_sqrt_sum
    from cftuv_envelope.exact_sqrt_sum import ExactWorkBudgetV1

    budget = ExactWorkBudgetV1(mode=mode, cap=cap, stage="MATERIALIZE", domain_id="test")
    before = dict(exact_sqrt_sum.SIGN_COUNTS)
    try:
        found = function(polygons, points, budget)
    except Exception as raised:  # noqa: BLE001 - the oracle's outcome is part of the answer
        found = (type(raised).__name__, str(raised))
    after = dict(exact_sqrt_sum.SIGN_COUNTS)
    return found, budget.spent_by_article(), {key: after.get(key, 0) - before.get(key, 0) for key in after}


def _same_pieces(polygons, points, cap, mode):
    """The port and the oracle on one call, both after a warm-up run of the oracle (the canonicalization memory only grows: both then see the same one)."""

    from cftuv_envelope.materialize.clip import piece_triangles

    _piece_run(piece_triangles, polygons, points, cap, mode)
    found = _piece_run(cftuv_native.piece_triangles, polygons, points, cap, mode)
    expected = _piece_run(piece_triangles, polygons, points, cap, mode)
    assert found == expected
    return expected


def test_the_clip_stage_routes_through_the_dispatcher():
    clip = (ROOT / "kernel/src/cftuv_envelope/materialize/clip.py").read_text(encoding="utf-8")
    assert "seam = backend.seam_edges_compute(seam_edges, frame_faces, polygons, facts, layout, lattice_alpha)" in clip
    domain = (ROOT / "kernel/src/cftuv_envelope/materialize/domain.py").read_text(encoding="utf-8")
    assert "backend.piece_triangles_compute(piece_triangles, cut.polygons, points, budget)" in domain


@needs_native
def test_every_seam_and_every_piece_of_the_cases_is_the_oracles(approved):
    from cftuv_envelope.materialize.clip import seam_edges

    seams, pieces = _captured()
    assert seams and pieces, "the cases clipped nothing: the test would prove nothing"
    edges = 0
    for arguments in seams:
        expected = seam_edges(*arguments)
        found = cftuv_native.seam_edges(*arguments)
        assert type(found) is frozenset and found == expected
        assert [tuple(pair) for pair in found] == [tuple(pair) for pair in expected]
        edges += len(expected)
    assert edges, "no seam edge: the test would prove nothing"
    ears = 0
    for polygons, points, cap, mode in pieces:
        found = _same_pieces(polygons, points, cap, mode)[0]
        ears += sum(len(keys) > 3 for face in polygons for keys in face)
        assert isinstance(found, list)
    assert ears, "no piece was ear-clipped: the test would prove nothing"


@needs_native
def test_a_flat_piece_is_the_oracles_refusal(approved):
    from cftuv_envelope.exact_sqrt_sum import SqrtSumV1

    _seams, pieces = _captured()
    polygons, points, cap, mode = pieces[0]
    flat = dict(points)
    for index, name in enumerate(("flat:a", "flat:b", "flat:c", "flat:d")):
        flat[name] = (SqrtSumV1.rational(index), SqrtSumV1.rational(2 * index))
    forced = [*polygons, [("flat:a", "flat:b", "flat:c", "flat:d")]]
    found = _same_pieces(forced, flat, cap, mode)[0]
    assert found[0] == "MaterializationRefusal" and "an emitted clip face of 4 vertices has no triangulation" in found[1], found


@needs_native
def test_a_bounded_budget_is_the_oracles(approved):
    from cftuv_envelope.exact_sqrt_sum import ExactWorkBudgetModeV1

    from cftuv_envelope.exact_sqrt_sum import SqrtSumV1

    _seams, pieces = _captured()
    for polygons, points, _cap, _mode in pieces:
        _same_pieces(polygons, points, 0, ExactWorkBudgetModeV1.BOUNDED)
    # three collinear points with irrational coordinates: the binary64 filter cannot decide the turn, the exact layer answers on the bounded budget
    root = SqrtSumV1(((2, 1),))
    points = {"r:a": (SqrtSumV1.zero(), SqrtSumV1.zero()), "r:b": (root, root), "r:c": (root.scaled(2), root.scaled(2)), "r:d": (SqrtSumV1.zero(), SqrtSumV1.rational(1))}
    _same_pieces([[("r:a", "r:b", "r:c", "r:d")]], points, 0, ExactWorkBudgetModeV1.BOUNDED)


@needs_native
def test_a_missing_fact_is_declined_and_the_oracle_raises(approved):
    from cftuv_envelope.materialize.clip import seam_edges

    seams, _pieces = _captured()
    for frame_faces, polygons, facts, layout, lattice_alpha in seams:
        if seam_edges(frame_faces, polygons, facts, layout, lattice_alpha):
            with pytest.raises(cftuv_native.NATIVE_REFUSALS):
                cftuv_native.seam_edges(frame_faces, polygons, {}, layout, lattice_alpha)
            with pytest.raises(KeyError):
                seam_edges(frame_faces, polygons, {}, layout, lattice_alpha)
            return
    pytest.fail("no seam: the test would prove nothing")
