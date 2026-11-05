"""The native `materialize.uv_unfold.unfold` (UNFOLD_ALONG_U_V1, `native/cftuv-unfold`) against the oracle: the same `UnfoldResultV1` under `UNFOLD_NATIVE_EQUALITY_V1`
(`held` and the numbers exact, `u` and the slopes within `NATIVE_EQUALITY`), and the count of answers that are equal bit for bit.

The inputs: every unfold the kernel's fixtures compute (field domains of `building.002`, `sagging_wall`, cut fans; the straight, skew, L, ring and near-planar domains), captured at the
dispatcher, plus the synthetic strips of `kernel/tests/test_uv_straighten.py` (held and free row, a sliver, a squeezed wedge, a flip).
"""

from __future__ import annotations

import dataclasses
import sys
from decimal import Decimal
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
for path in (ROOT / "kernel" / "src", ROOT / "kernel" / "tests"):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

try:
    import cftuv_native
except ModuleNotFoundError as error:
    if error.name != "cftuv_native":
        raise
    cftuv_native = None

needs_native = pytest.mark.skipif(cftuv_native is None or not hasattr(cftuv_native, "unfold_u"), reason="расширение cftuv_native не собрано (или собрано без unfold_u)")


@pytest.fixture
def approved():
    from cftuv_native import materialize_hot_op as op

    op.refresh()
    assert op.status() == "available", op.status()
    yield
    op.refresh()


def _synthetic():
    from test_uv_straighten import _strip, _with_sliver

    return [
        _strip((0.0, 1.0, 2.5, 4.0)),
        _strip((0.0, 1.0, 2.5, 4.0), hold=False),
        _strip((0.0, 1.0, 2.0), source_s=(0.0, 1.0, 2.0), top_z=1.0),
        _with_sliver(0.0),
        _with_sliver(1e-6),
        _strip((0.0, 3.0, 6.0), source_s=(0.0, 1.0, 2.0)),
        _strip((0.0, 1.0, 2.0), source_s=(0.0, 1.0, 0.2)),
    ]


def _captured():
    """Every `UnfoldInputV1` the kernel's fixtures hand the dispatcher (the Straight strips policy, two alphas each)."""

    import materialize_factories as factories

    from cftuv_envelope import backend
    from cftuv_envelope.contracts.geometry_batch import DecalTopologyLawV1
    from cftuv_envelope.contracts.metric import NearPlanarLiftLawV1
    from cftuv_envelope.materialize.admit import materialization_request
    from cftuv_envelope.materialize.step import step_domain
    from cftuv_envelope.wavefront import prepare_conveyor

    calls = []
    saved = backend.unfold_compute

    def capture(oracle, data):
        calls.append(data)
        return oracle(data)

    backend.unfold_compute = capture
    try:
        for name in ("building_002_full_selection_v1", "building_002_point_contact_v1", "mesh2_patch0_cut_fans_v1", "sagging_wall_convex_partition_v1"):
            snapshot, request = factories.load_fixture(name)
            prepared = prepare_conveyor(snapshot, request)
            alpha = str(prepared.requested_alpha.value)
            for text in (alpha, str(Decimal(alpha) * Decimal("0.5"))):
                step_domain(
                    prepared, text, request=materialization_request(prepared, uv_policy_id="UV_DIRECT_STRIP_V1"), near_planar_lift_law=NearPlanarLiftLawV1.SOURCE_FACES_CLIPPED_V1,
                    decal_topology_law=DecalTopologyLawV1.SILHOUETTE_TOPOLOGY_V1, digests=False,
                )
    finally:
        backend.unfold_compute = saved
    return calls


@needs_native
def test_every_unfold_of_the_kernel_fixtures_and_the_strips_agrees_with_the_oracle(approved):
    from cftuv_envelope.materialize.uv_unfold import results_agree, unfold

    inputs = _captured()
    assert len(inputs) >= 8, len(inputs)
    inputs += _synthetic()
    bitwise = 0
    for data in inputs:
        oracle, native = unfold(data), cftuv_native.unfold_u(data)
        assert results_agree(oracle, native), data
        bitwise += oracle == native
    # The port repeats the order of the binary64 operations: on one machine every answer is equal bit for bit (the tolerance is for another compiler or libm).
    assert bitwise == len(inputs), (bitwise, len(inputs))
    assert sum(len(data.r) for data in inputs) > 500, "the fixtures must carry real islands, not only the strips"


@needs_native
def test_what_the_port_cannot_read_is_declined_and_the_oracle_answers(approved):
    from test_uv_straighten import _strip

    data = _strip((0.0, 1.0, 2.0))
    for broken in (dataclasses.replace(data, corners=data.corners[:-1]), dataclasses.replace(data, r=data.r[:-1]), dataclasses.replace(data, positions=("x",) + data.positions[1:])):
        with pytest.raises(cftuv_native.NATIVE_REFUSALS):
            cftuv_native.unfold_u(broken)
    with pytest.raises(cftuv_native.NATIVE_REFUSALS):
        cftuv_native.unfold_u(object())


@needs_native
def test_the_dispatcher_runs_the_port_under_the_w1_switch(approved, monkeypatch):
    from test_uv_straighten import _strip

    from cftuv_envelope import backend
    from cftuv_envelope.materialize.uv_unfold import unfold

    data = _strip((0.0, 1.0, 2.0))
    calls = []
    real = cftuv_native.unfold_u
    monkeypatch.setattr(cftuv_native, "unfold_u", lambda *args: calls.append(args) or real(*args))
    with backend.use_backend("PYTHON", materialize_backend="NATIVE") as ledger:
        assert backend.unfold_compute(unfold, data) == unfold(data)
    assert len(calls) == 1 and ledger.record().materialize_native_calls >= 1
    with backend.use_backend("PYTHON", materialize_backend="PYTHON"):
        backend.unfold_compute(unfold, data)
    assert len(calls) == 1


def test_the_straight_law_routes_its_unfold_through_the_dispatcher():
    import ast

    text = (ROOT / "kernel/src/cftuv_envelope/materialize/uv_straighten.py").read_text(encoding="utf-8")
    assert "backend.unfold_compute(unfold, data)" in text
    direct = [node for node in ast.walk(ast.parse(text)) if isinstance(node, ast.Call) and isinstance(node.func, ast.Name) and node.func.id == "unfold"]
    assert direct == [], "a direct call of the oracle would skip the native port silently"
