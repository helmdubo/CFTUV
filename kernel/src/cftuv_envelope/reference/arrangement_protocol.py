"""Backend-neutral exact planar arrangement protocol."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol

from .contracts import (
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
    intersection_count: int
    atomic_edge_count: int


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
    ) -> ArrangementUnionV1: ...
