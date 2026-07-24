"""Backend-neutral exact planar arrangement protocol."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol

from .contracts import (
    BoundaryVertexOccurrenceV1,
    PointContactRecordV1,
    RawCoverageEdgeV1,
    RawCoverageLoopV1,
    RawCoverageRegionV1,
    RawCoverageVertexV1,
    ReachabilityCertificateV1,
)
from .planar_types import PlanarRegion


@dataclass(frozen=True, slots=True)
class ArrangementUnionV1:
    vertices: frozenset[RawCoverageVertexV1]
    edges: frozenset[RawCoverageEdgeV1]
    loops: frozenset[RawCoverageLoopV1]
    regions: frozenset[RawCoverageRegionV1]
    exact_area_expression: str
    input_segment_count: int
    all_possible_pair_count: int
    broadphase_candidate_pair_count: int
    narrowphase_test_count: int
    intersection_count: int
    atomic_edge_count: int


@dataclass(frozen=True, slots=True)
class ArrangementUnionV2(ArrangementUnionV1):
    boundary_vertex_occurrences: frozenset[BoundaryVertexOccurrenceV1]
    point_contacts: frozenset[PointContactRecordV1]
    branch_point_count: int
    max_incident_degree: int
    rotation_comparison_count: int
    face_walk_count: int


class PlanarArrangementBackend(Protocol):
    backend_identity: str
    backend_version: str

    def build_arrangement(
        self,
        contribution_regions: tuple[PlanarRegion, ...],
        domain_regions: tuple[PlanarRegion, ...],
    ) -> object: ...

    def exact_union(
        self,
        contribution_regions: tuple[PlanarRegion, ...],
        domain_regions: tuple[PlanarRegion, ...],
        reachability_by_instance: dict[str, ReachabilityCertificateV1],
    ) -> ArrangementUnionV2: ...
