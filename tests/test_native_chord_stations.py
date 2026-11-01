"""CHORD_STATIONS_NATIVE_V1: `mesh.station_values` of the clip stage (`clip.cut_domain`: the new vertices of the clip with `chords` and `anchors`) in Rust
(`cftuv_mesh::stations::chord_station_values`), against the oracle.

Every call the oracle makes in the developable and host-clipped cases is replayed on both: the same dict in the same order, the same additions to `tally`, the same articles of
the budget and the same `SIGN_COUNTS`. A forced second answer that is no rung and no chord is the oracle's `STATION_VALUE_CONFLICT` with its text; an exhausted budget is the
oracle's exhaustion. The whole: the cases materialized and certified under the W1 stage NATIVE and PYTHON give the same answer and the same certificate.
"""

from __future__ import annotations

import sys
from collections import Counter
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
    cftuv_native is None or not hasattr(cftuv_native, "chord_station_values"), reason="расширение cftuv_native не собрано (или собрано без CHORD_STATIONS_NATIVE_V1)"
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


def _cases():
    from test_native_surface_lift import _cases as cases

    return cases()


def _captured():
    """`(arguments, keywords)` of every `station_values` call of the clip stage in the cases (the oracle's run), the mutable ones copied before the call."""

    from cftuv_envelope import backend
    from test_native_surface_lift import _step

    calls = []
    saved = backend.chord_stations_compute

    def capture(oracle, frame_faces, lists, layout, table, lattice_alpha, budget, tally, *, chords, anchors):
        calls.append(((list(frame_faces), [list(item) for item in lists], layout, table, lattice_alpha, budget.cap, budget.mode), {"chords": chords, "anchors": dict(anchors)}))
        return saved(oracle, frame_faces, lists, layout, table, lattice_alpha, budget, tally, chords=chords, anchors=anchors)

    backend.chord_stations_compute = capture
    try:
        for prepare, laws, alpha in _cases():
            _step(prepare, laws, alpha, "PYTHON")
    finally:
        backend.chord_stations_compute = saved
    return calls


def _budget(cap, mode):
    from cftuv_envelope.exact_sqrt_sum import ExactWorkBudgetV1

    return ExactWorkBudgetV1(mode=mode, cap=cap, stage="MATERIALIZE", domain_id="test")


def _same(arguments, keywords):
    """The port and the oracle on one call, both after a warm-up run of the oracle (the canonicalization memory only grows: both then see the same one)."""

    from cftuv_envelope.materialize.mesh import station_values

    _run(station_values, arguments, keywords)
    found = _run(cftuv_native.chord_station_values, arguments, keywords)
    expected = _run(station_values, arguments, keywords)
    assert found == expected
    return expected


def _run(function, arguments, keywords, cap=None):
    """`(answer or exception text, tally, articles, sign counts gained)` of one call on a fresh budget and tally."""

    from cftuv_envelope import exact_sqrt_sum
    from cftuv_envelope.materialize.mesh import station_values

    frame_faces, lists, layout, table, lattice_alpha, budget_cap, mode = arguments
    budget = _budget(budget_cap if cap is None else cap, mode)
    tally = Counter({"PRESENT": 1})
    before = dict(exact_sqrt_sum.SIGN_COUNTS)
    try:
        if function is station_values:
            found = list(station_values(frame_faces, lists, layout, table, lattice_alpha, budget, tally, **keywords).items())
        else:
            found = list(function(frame_faces, lists, layout, table, budget, tally, keywords["chords"], keywords["anchors"]).items())
    except Exception as raised:  # noqa: BLE001 - the oracle's outcome is part of the answer
        found = (type(raised).__name__, str(raised), getattr(raised, "station_conflict", None))
    after = dict(exact_sqrt_sum.SIGN_COUNTS)
    return found, list(tally.items()), budget.spent_by_article(), {key: after.get(key, 0) - before.get(key, 0) for key in after}


@needs_native
def test_every_clip_stage_call_of_the_cases_is_the_oracles(approved):
    from cftuv_envelope.materialize.mesh import station_values

    calls = _captured()
    assert len(calls) >= 10
    for arguments, keywords in calls:
        _same(arguments, keywords)


RUNG_FIXTURES = (("sagging_wall_rung_chord_v1", ("0.4", "0.6")), ("wall_noise_top_rung_clip_v1", None))


def _rung_domains():
    """`(prepared, coverage, alpha)` of the fixtures whose clip vertices lie on a rung (`kernel/tests/test_join_station_conflict.py`): the chord law answers there."""

    import cftuv_envelope as kernel
    from cftuv_envelope.wavefront import conveyor_coverage, prepare_conveyor

    found = []
    for name, alphas in RUNG_FIXTURES:
        folder = ROOT / "kernel" / "fixtures" / name
        snapshot = kernel.AnalysisSnapshotCodecV1.loads((folder / "analysis_snapshot.json").read_bytes())
        requests = sorted(folder.glob("decal_request*.json")) if alphas is None else [folder / f"decal_request_alpha_{alpha}.json" for alpha in alphas]
        for path in requests:
            request = kernel.DecalRequestCodecV1.loads(path.read_bytes())
            prepared = prepare_conveyor(snapshot, request)
            alpha = str(prepared.requested_alpha.value)
            found.append((prepared, conveyor_coverage(prepared, alpha), alpha))
    return found


def _materialize_rung(prepared, coverage, materialize_backend):
    from cftuv_envelope import backend
    from cftuv_envelope.contracts.geometry_batch import DecalTopologyLawV1
    from cftuv_envelope.contracts.metric import NearPlanarLiftLawV1
    from cftuv_envelope.materialize.admit import materialization_request
    from cftuv_envelope.materialize.clip_memo import memo_disabled
    from cftuv_envelope.materialize.domain import materialize_domain

    with backend.use_backend("PYTHON", materialize_backend=materialize_backend), memo_disabled():
        return materialize_domain(
            prepared, coverage, request=materialization_request(prepared, uv_policy_id="UV_DIRECT_STRIP_V1"),
            near_planar_lift_law=NearPlanarLiftLawV1.SOURCE_FACES_CLIPPED_V1, decal_topology_law=DecalTopologyLawV1.PLANAR_POLYGONS_V1,
        )


@needs_native
def test_the_rung_fixtures_interpolate_on_the_shared_edge_like_the_oracle(approved, monkeypatch):
    from cftuv_envelope import backend
    from cftuv_envelope.materialize.mesh import RUNG_CHORD_STATIONS
    from cftuv_envelope.materialize.step import answer_differences

    calls = []
    saved = backend.chord_stations_compute

    def capture(oracle, frame_faces, lists, layout, table, lattice_alpha, budget, tally, *, chords, anchors):
        calls.append(((list(frame_faces), [list(item) for item in lists], layout, table, lattice_alpha, budget.cap, budget.mode), {"chords": chords, "anchors": dict(anchors)}))
        return saved(oracle, frame_faces, lists, layout, table, lattice_alpha, budget, tally, chords=chords, anchors=anchors)

    native = []
    real = cftuv_native.chord_station_values
    monkeypatch.setattr(cftuv_native, "chord_station_values", lambda *arguments: native.append(1) or real(*arguments))
    interpolated = 0
    for prepared, coverage, _alpha in _rung_domains():
        backend.chord_stations_compute = capture
        try:
            expected = _materialize_rung(prepared, coverage, "PYTHON")
        finally:
            backend.chord_stations_compute = saved
        found = _materialize_rung(prepared, coverage, "NATIVE")
        assert answer_differences(expected, found) == ()
        assert expected.counters == found.counters
        interpolated += dict(expected.counters).get(RUNG_CHORD_STATIONS, 0)
    assert interpolated, "the chord law answered nowhere: the test would prove nothing"
    assert native, "the NATIVE run never reached the port"
    replayed = 0
    for arguments, keywords in calls:
        replayed += dict(_same(arguments, keywords)[1]).get(RUNG_CHORD_STATIONS, 0)
    assert replayed, "no replayed call interpolated"

@needs_native
def test_a_conflict_is_the_oracles_refusal(approved):
    from cftuv_envelope.exact_sqrt_sum import SqrtSumV1
    from cftuv_envelope.materialize.mesh import station_values

    for arguments, keywords in _captured():
        frame_faces, lists, layout, table, lattice_alpha, cap, mode = arguments
        plain = [index for index, frame_face in enumerate(frame_faces) if not frame_face.is_fan]
        pairs = [(a, b) for a in plain for b in plain if a < b and layout.region_of(frame_faces[a]) == layout.region_of(frame_faces[b])]
        if not pairs:
            continue
        first, second = pairs[0]
        lists = [list(item) for item in lists]
        lists[first] = [*lists[first], ("forced:conflict", (SqrtSumV1.rational(3), SqrtSumV1.rational(5)))]
        lists[second] = [*lists[second], ("forced:conflict", (SqrtSumV1.rational(7), SqrtSumV1.rational(-11)))]
        forced = (frame_faces, lists, layout, table, lattice_alpha, cap, mode)
        expected = _same(forced, keywords)
        if not isinstance(expected[0], tuple):
            continue
        assert "STATION_VALUE_CONFLICT" in expected[0][1]
        return
    pytest.fail("no case had two plain faces in one region: the test would prove nothing")


@needs_native
def test_an_exhausted_budget_is_the_oracles_exhaustion(approved):
    from cftuv_envelope.exact_sqrt_sum import ExactWorkBudgetModeV1
    from cftuv_envelope.materialize.mesh import station_values

    exhausted = 0
    for arguments, keywords in _captured():
        bounded = (*arguments[:5], 0, ExactWorkBudgetModeV1.BOUNDED)
        expected = _same(bounded, keywords)
        exhausted += isinstance(expected[0], tuple)
    assert exhausted, "no call paid anything: the test would prove nothing"


@needs_native
def test_a_case_answers_and_certifies_the_same_under_both_backends(approved, monkeypatch):
    from test_native_surface_lift import _certificate_fields, _step

    from cftuv_envelope.materialize.step import answer_differences

    calls = []
    real = cftuv_native.chord_station_values
    monkeypatch.setattr(cftuv_native, "chord_station_values", lambda *arguments: calls.append(len(arguments)) or real(*arguments))
    compared = 0
    for prepare, laws, alpha in _cases():
        oracle = _step(prepare, laws, alpha, "PYTHON")
        port = _step(prepare, laws, alpha, "NATIVE")
        for (expected, expected_signs), (found, found_signs) in zip(oracle, port):
            assert answer_differences(expected, found) == ()
            assert expected.counters == found.counters
            assert _certificate_fields(expected.parametric) == _certificate_fields(found.parametric)
            assert expected_signs == found_signs
            compared += 1
    assert compared >= 10
    assert calls, "no clip stage was stationed natively: the test would prove nothing"


def test_the_clip_stage_routes_through_the_dispatcher():
    text = (ROOT / "kernel/src/cftuv_envelope/materialize/clip.py").read_text(encoding="utf-8")
    assert "extra = backend.chord_stations_compute(\n        station_values," in text.replace("\r\n", "\n")
