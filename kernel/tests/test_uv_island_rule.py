"""Правило Straight strips: одна pChain - один непрерывный UV-остров. Перепись на числах и на настоящих доменах с красным контролем."""

from __future__ import annotations

from pathlib import Path

import pytest

import cftuv_envelope as kernel
from cftuv_envelope.contracts.geometry_batch import DecalTopologyLawV1
from cftuv_envelope.contracts.metric import NearPlanarLiftLawV1
from cftuv_envelope.ids import PolicyId
from cftuv_envelope.materialize.admit import MaterializationOutcome, materialization_request
from cftuv_envelope.materialize.domain import materialize_domain
import cftuv_envelope.materialize.mesh as mesh
from cftuv_envelope.uv_islands import batch_island_census, pchains_of_uses, uv_island_census
from cftuv_envelope.wavefront import conveyor_coverage, prepare_conveyor

UV = PolicyId("UV_DIRECT_STRIP_V1")
FIXTURES = Path(__file__).resolve().parents[1] / "fixtures"
#: `(папка, имя запроса, alpha)`: настоящие домены поля, в которых есть съеденные куски. Висячих pChain на них быть не должно.
FIELD_DOMAINS = (
    ("sagging_wall_rung_chord_v1", "decal_request_alpha_0.4.json", "0.4"),
    ("sagging_wall_rung_chord_v1", "decal_request_alpha_0.6.json", "0.6"),
    ("wall_noise_top_rung_clip_v1", "decal_request.json", "0.5"),
)
QUAD_A = ("a", "b", "c", "d")
QUAD_B = ("d", "c", "e", "f")


def _uv(**given):
    return given


def test_two_faces_with_one_uv_on_their_edge_are_one_island_of_one_pchain():
    census = uv_island_census((QUAD_A, QUAD_B), (_uv(a=0, b=1, c=2, d=3), _uv(d=3, c=2, e=4, f=5)), (("p",), ("p",)))
    assert len(census.islands) == 1 and census.seam_edges == 0
    assert census.satisfied == ("p",) and census.violations == ()


def test_a_seam_inside_one_pchain_is_the_violation_and_between_two_pchains_it_is_not():
    uvs = (_uv(a=0, b=1, c=2, d=3), _uv(d=9, c=2, e=4, f=5))
    inside = uv_island_census((QUAD_A, QUAD_B), uvs, (("p",), ("p",)))
    assert inside.violations == (("p", 2),) and inside.seam_edges == 1
    between = uv_island_census((QUAD_A, QUAD_B), uvs, (("p",), ("q",)))
    assert between.violations == () and between.satisfied == ("p", "q")


def test_an_island_on_two_pchains_is_a_wedge_and_owns_nothing():
    census = uv_island_census((QUAD_A, QUAD_B), (_uv(a=0, b=1, c=2, d=3), _uv(d=9, c=2, e=4, f=5)), (("p",), ("p", "q")))
    assert [island.pchains for island in census.wedges] == [("p", "q")]
    assert census.violations == (("q", 0),)


def test_a_seam_that_splits_nothing_is_counted_as_internal():
    faces = (("a", "b", "c"), ("c", "b", "d"), ("a", "c", "d"))
    uvs = (_uv(a=0, b=1, c=2), _uv(c=2, b=1, d=3), _uv(a=7, c=2, d=3))
    census = uv_island_census(faces, uvs, (("p",),) * 3)
    assert len(census.islands) == 1 and census.seam_edges == 1 and census.internal_seam_edges == 1


def _field(folder: str, name: str, alpha: str):
    root = FIXTURES / folder
    snapshot = kernel.AnalysisSnapshotCodecV1.loads((root / "analysis_snapshot.json").read_bytes())
    request = kernel.DecalRequestCodecV1.loads((root / name).read_bytes())
    prepared = prepare_conveyor(snapshot, request)
    assert prepared.outcome.value == "EXACT", prepared.detail
    result = materialize_domain(
        prepared,
        conveyor_coverage(prepared, alpha),
        request=materialization_request(prepared, uv_policy_id=UV),
        near_planar_lift_law=NearPlanarLiftLawV1.SOURCE_FACES_CLIPPED_V1,
        decal_topology_law=DecalTopologyLawV1.PLANAR_POLYGONS_V1,
    )
    assert result.outcome is MaterializationOutcome.MATERIALIZED, result.detail
    return prepared, result


@pytest.mark.parametrize("folder,name,alpha", FIELD_DOMAINS)
def test_every_pchain_of_a_field_domain_is_one_island(folder, name, alpha):
    prepared, result = _field(folder, name, alpha)
    census = batch_island_census(result.batch, pchains_of_uses(prepared.context))
    assert census.violations == ()


def test_red_control_without_the_consumed_piece_law_the_withdrawn_junction_splits_a_pchain(monkeypatch):
    folder, name, alpha = FIELD_DOMAINS[0]
    monkeypatch.setattr(mesh, "_consumed_piece_facts", lambda table, pending, answers: ({}, pending[0][1]))
    prepared, result = _field(folder, name, alpha)
    assert dict(result.counters)["STATION_SKIP_JOIN_WITHDRAWN_AT_STATION_CONFLICT"] >= 1
    assert batch_island_census(result.batch, pchains_of_uses(prepared.context)).violations != ()
