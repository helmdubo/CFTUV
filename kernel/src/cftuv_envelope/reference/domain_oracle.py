"""Independent source-face union oracle for sparse-domain tests.

This module is intentionally not imported by the runtime reference package.
It may inspect source-face cycles because its sole purpose is to prove that the
authoritative sparse boundary has the same exact semantic coverage.
"""

from __future__ import annotations

from dataclasses import dataclass

from .arrangement import ExactSegmentArrangementBackend
from .common import (
    GeometryContext,
    make_segment,
    source_vertex_certificate,
    stable_id,
)
from .domain_geometry import SparsePatchDomainGeometryV1
from .planar_types import PlanarLoop, PlanarRegion, exact_sign, polygon_signed_area
from .provenance import make_reference_provenance


_ORACLE_BACKEND = ExactSegmentArrangementBackend()


@dataclass(frozen=True, slots=True)
class FaceRegionUnionOracleV1:
    source_face_regions: tuple[PlanarRegion, ...]
    exact_area_expression: str
    outer_component_count: int
    hole_count: int
    boundary_physical_edge_ids: frozenset[str]
    boundary_geometry_keys: frozenset[tuple[tuple[str, str], tuple[str, str]]]
    face_boundary_segment_count: int


@dataclass(frozen=True, slots=True)
class SparseDomainEquivalenceReportV1:
    patch_domain_id: str
    exact_area_equal: bool
    outer_component_count_equal: bool
    hole_count_equal: bool
    boundary_physical_edge_lineage_equal: bool
    boundary_geometry_equal: bool
    source_face_contributors_equal: bool
    internal_source_edges_absent: bool
    face_boundary_segment_count: int
    sparse_domain_segment_count: int
    source_face_count: int
    internal_source_edge_ids: frozenset[str]

    @property
    def exact_equivalent(self) -> bool:
        return all(
            (
                self.exact_area_equal,
                self.outer_component_count_equal,
                self.hole_count_equal,
                self.boundary_physical_edge_lineage_equal,
                self.boundary_geometry_equal,
                self.source_face_contributors_equal,
                self.internal_source_edges_absent,
            )
        )


def _geometry_keys(arrangement):
    vertices = {item.vertex_id: item.point for item in arrangement.vertices}
    return frozenset(
        tuple(
            sorted(
                (
                    (
                        vertices[edge.start_vertex_id].x.expression,
                        vertices[edge.start_vertex_id].y.expression,
                    ),
                    (
                        vertices[edge.end_vertex_id].x.expression,
                        vertices[edge.end_vertex_id].y.expression,
                    ),
                )
            )
        )
        for edge in arrangement.edges
    )


def build_source_face_union_oracle(
    context: GeometryContext,
) -> FaceRegionUnionOracleV1:
    """Build the former per-face domain only inside the independent oracle."""

    regions = []
    for face in sorted(
        (
            item
            for item in context.snapshot.surface_ir.source_faces
            if item.patch_id == context.compilation.owner_patch_id
        ),
        key=lambda item: item.face_id.value,
    ):
        segments = []
        for ordinal, edge_id in enumerate(face.edge_cycle):
            start_id = face.vertex_cycle[ordinal]
            end_id = face.vertex_cycle[(ordinal + 1) % len(face.vertex_cycle)]
            support_id = stable_id("face-oracle-support", face.face_id, edge_id)
            segments.append(
                make_segment(
                    stable_id("face-oracle-edge", face.face_id, edge_id),
                    context.points_by_id[start_id],
                    context.points_by_id[end_id],
                    support_ids=frozenset({support_id}),
                    provenance=make_reference_provenance(
                        support_ids=frozenset({support_id}),
                        physical_edge_ids=frozenset({edge_id.value}),
                        source_face_ids=frozenset({face.face_id.value}),
                        patch_domain_ids=frozenset(
                            {context.compilation.plan_key.patch_domain_id.value}
                        ),
                    ),
                    start_certificates=frozenset(
                        {source_vertex_certificate(start_id)}
                    ),
                    end_certificates=frozenset(
                        {source_vertex_certificate(end_id)}
                    ),
                )
            )
        area_sign = exact_sign(
            polygon_signed_area(segment.start for segment in segments)
        )
        if area_sign == 0:
            raise ValueError(f"source face {face.face_id} has zero planar area")
        if area_sign < 0:
            segments = [
                segment.reversed() for segment in reversed(segments)
            ]
        regions.append(
            PlanarRegion(
                stable_id("face-oracle-region", face.face_id),
                PlanarLoop(
                    stable_id("face-oracle-loop", face.face_id),
                    tuple(segments),
                ),
            )
        )
    regions_tuple = tuple(regions)
    union = _ORACLE_BACKEND.exact_union(regions_tuple, regions_tuple, {})
    return FaceRegionUnionOracleV1(
        source_face_regions=regions_tuple,
        exact_area_expression=union.exact_area_expression,
        outer_component_count=len(union.regions),
        hole_count=sum(len(item.hole_loop_ids) for item in union.regions),
        boundary_physical_edge_ids=frozenset(
            edge_id
            for edge in union.edges
            for edge_id in edge.provenance.physical_edge_ids
        ),
        boundary_geometry_keys=_geometry_keys(union),
        face_boundary_segment_count=sum(
            len(region.outer.segments) for region in regions_tuple
        ),
    )


def compare_sparse_domain_to_face_union(
    context: GeometryContext,
    sparse: SparsePatchDomainGeometryV1,
) -> SparseDomainEquivalenceReportV1:
    face_oracle = build_source_face_union_oracle(context)
    sparse_union = _ORACLE_BACKEND.exact_union(
        sparse.domain_regions, sparse.domain_regions, {}
    )
    sparse_boundary_ids = frozenset(
        edge_id
        for edge in sparse_union.edges
        for edge_id in edge.provenance.physical_edge_ids
    )
    edge_counts = {}
    for face in context.snapshot.surface_ir.source_faces:
        if face.patch_id != context.compilation.owner_patch_id:
            continue
        for edge_id in face.edge_cycle:
            edge_counts[edge_id.value] = edge_counts.get(edge_id.value, 0) + 1
    internal_ids = frozenset(
        edge_id for edge_id, count in edge_counts.items() if count > 1
    )
    return SparseDomainEquivalenceReportV1(
        patch_domain_id=sparse.patch_domain_id.value,
        exact_area_equal=(
            sparse_union.exact_area_expression
            == face_oracle.exact_area_expression
        ),
        outer_component_count_equal=(
            len(sparse_union.regions) == face_oracle.outer_component_count
        ),
        hole_count_equal=(
            sum(len(item.hole_loop_ids) for item in sparse_union.regions)
            == face_oracle.hole_count
        ),
        boundary_physical_edge_lineage_equal=(
            sparse_boundary_ids == face_oracle.boundary_physical_edge_ids
        ),
        boundary_geometry_equal=(
            _geometry_keys(sparse_union)
            == face_oracle.boundary_geometry_keys
        ),
        source_face_contributors_equal=(
            sparse.source_face_contributor_ids
            == frozenset(
                face.face_id
                for face in context.snapshot.surface_ir.source_faces
                if face.patch_id == context.compilation.owner_patch_id
            )
        ),
        internal_source_edges_absent=internal_ids.isdisjoint(
            frozenset(
                edge_id
                for loop in (*sparse.outer_loops, *sparse.hole_loops)
                for segment in loop.segments
                for edge_id in segment.provenance.physical_edge_ids
            )
        ),
        face_boundary_segment_count=face_oracle.face_boundary_segment_count,
        sparse_domain_segment_count=sparse.sparse_segment_count,
        source_face_count=len(sparse.source_face_contributor_ids),
        internal_source_edge_ids=internal_ids,
    )
