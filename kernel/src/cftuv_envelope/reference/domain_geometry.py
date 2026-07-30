"""Sparse PatchDomain geometry from authoritative boundary facts.

Runtime domain geometry is defined only by BoundaryLoopV1, its ordered
ChainUses, PhysicalChain edge/vertex identity, and explicit constraints.
Source-face cycles remain contributor evidence and never become runtime
arrangement boundaries.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum

from ..contracts.analysis import (
    BoundaryLoopConstraintTargetV1,
    BoundaryLoopKind,
    BoundaryConstraintKind,
    ChainUseConstraintTargetV1,
    PhysicalEdgeSequenceConstraintTargetV1,
)
from ..ids import PatchDomainId, PhysicalEdgeId, SourceFaceId
from .common import (
    GeometryContext,
    ReferenceGeometryError,
    make_segment,
    source_vertex_certificate,
    stable_id,
)
from .contracts import ReferenceOutcome
from .planar_types import (
    BoundedSupportSegment,
    PlanarLoop,
    PlanarRegion,
    exact_sign,
    point_key,
    polygon_signed_area,
)
from .provenance import ReferenceProvenanceV1, make_reference_provenance


class BoundaryRole(str, Enum):
    OUTER = "OUTER"
    HOLE = "HOLE"
    EXPLICIT_BARRIER = "EXPLICIT_BARRIER"


@dataclass(frozen=True, slots=True)
class BlockingBoundarySegment:
    segment: BoundedSupportSegment
    role: BoundaryRole
    concave_vertex_keys: frozenset[tuple[str, str]] = frozenset()


@dataclass(frozen=True, slots=True)
class SparsePatchDomainGeometryV1:
    """Один semantic PatchDomain без внутренних source-face границ."""

    patch_domain_id: PatchDomainId
    outer_loops: tuple[PlanarLoop, ...]
    hole_loops: tuple[PlanarLoop, ...]
    explicit_barriers: tuple[BlockingBoundarySegment, ...]
    source_face_contributor_ids: frozenset[SourceFaceId]

    @property
    def domain_regions(self) -> tuple[PlanarRegion, ...]:
        if len(self.outer_loops) != 1:
            raise ReferenceGeometryError(
                ReferenceOutcome.REFERENCE_INPUT_CONTRACT_INVALID,
                "SparsePatchDomainGeometryV1 requires exactly one connected outer loop",
            )
        return (
            PlanarRegion(
                region_id=stable_id("sparse-patch-domain-region", self.patch_domain_id),
                outer=self.outer_loops[0],
                holes=self.hole_loops,
            ),
        )

    @property
    def blocking_segments(self) -> tuple[BlockingBoundarySegment, ...]:
        result = []
        for loop in self.outer_loops:
            concave = _concave_outer_vertex_keys(loop)
            result.extend(
                BlockingBoundarySegment(segment, BoundaryRole.OUTER, concave)
                for segment in loop.segments
            )
        result.extend(
            BlockingBoundarySegment(segment, BoundaryRole.HOLE)
            for loop in self.hole_loops
            for segment in loop.segments
        )
        result.extend(self.explicit_barriers)
        return tuple(result)

    @property
    def boundary_provenance(self) -> tuple[ReferenceProvenanceV1, ...]:
        return tuple(item.segment.provenance for item in self.blocking_segments)

    @property
    def boundary_segment_count(self) -> int:
        return sum(len(loop.segments) for loop in (*self.outer_loops, *self.hole_loops))

    @property
    def sparse_segment_count(self) -> int:
        return self.boundary_segment_count + len(self.explicit_barriers)


def _fail(message: str) -> ReferenceGeometryError:
    return ReferenceGeometryError(
        ReferenceOutcome.REFERENCE_INPUT_CONTRACT_INVALID,
        message,
    )


def _constraint_edges(
    context: GeometryContext, constraint
) -> frozenset[PhysicalEdgeId]:
    target = constraint.target
    if isinstance(target, PhysicalEdgeSequenceConstraintTargetV1):
        return frozenset(target.ordered_physical_edge_ids)
    if isinstance(target, ChainUseConstraintTargetV1):
        chain_use = context.uses_by_id[target.chain_use_id]
        return frozenset(
            context.chains_by_id[
                chain_use.physical_chain_id
            ].ordered_physical_edge_ids
        )
    if isinstance(target, BoundaryLoopConstraintTargetV1):
        loop = next(
            (
                item
                for item in context.snapshot.boundary_loops
                if item.boundary_loop_id == target.boundary_loop_id
            ),
            None,
        )
        if loop is None:
            raise _fail(
                f"constraint references unknown BoundaryLoop {target.boundary_loop_id}"
            )
        return frozenset(
            edge_id
            for chain_use_id in loop.ordered_chain_use_ids
            for edge_id in context.chains_by_id[
                context.uses_by_id[chain_use_id].physical_chain_id
            ].ordered_physical_edge_ids
        )
    return frozenset()


def _chain_segment_records(context: GeometryContext, chain_use):
    chain = context.chains_by_id[chain_use.physical_chain_id]
    vertices = context.directed_chain_vertices(chain_use)
    pairs = tuple(zip(vertices, vertices[1:]))
    if chain.is_closed:
        pairs += ((vertices[-1], vertices[0]),)
    if len(pairs) != len(chain.ordered_physical_edge_ids):
        raise _fail(
            f"PhysicalChain {chain.physical_chain_id} edge/vertex cardinality is invalid"
        )
    return tuple(
        (start_id, end_id, context._edge_for_pair(chain, start_id, end_id))
        for start_id, end_id in pairs
    )


def _normalize_loop_winding(
    context: GeometryContext,
    loop: PlanarLoop,
    kind: BoundaryLoopKind,
) -> PlanarLoop:
    area_sign = context._sign(context._polygon_signed_area(loop.points))
    if area_sign == 0:
        raise _fail(f"BoundaryLoop {loop.loop_id} has zero planar area")
    expected_positive = kind is BoundaryLoopKind.OUTER
    if (area_sign > 0) == expected_positive:
        return loop
    return PlanarLoop(
        loop.loop_id,
        tuple(segment.reversed() for segment in reversed(loop.segments)),
    )


def _validate_closed_loop(loop: PlanarLoop) -> None:
    if not loop.segments:
        raise _fail(f"BoundaryLoop {loop.loop_id} has no physical segments")
    for current, following in zip(
        loop.segments, loop.segments[1:] + loop.segments[:1]
    ):
        if point_key(current.end) != point_key(following.start):
            raise _fail(
                f"BoundaryLoop {loop.loop_id} is not closed in ordered ChainUse order"
            )


def _concave_outer_vertex_keys(loop: PlanarLoop) -> frozenset[tuple[str, str]]:
    points = loop.points
    concave = set()
    for previous, current, following in zip(
        points[-1:] + points[:-1], points, points[1:] + points[:1]
    ):
        from .planar_types import cross, point_sub

        if exact_sign(
            cross(point_sub(current, previous), point_sub(following, current))
        ) < 0:
            concave.add(point_key(current))
    return frozenset(concave)


def _loop_segment(
    context: GeometryContext,
    *,
    boundary_loop_id,
    chain_use,
    start_id,
    end_id,
    edge_id,
    constraint_ids: frozenset[str],
):
    chain = context.chains_by_id[chain_use.physical_chain_id]
    support_id = stable_id(
        "domain-boundary-support",
        context.compilation.plan_key.patch_domain_id,
        boundary_loop_id,
        chain_use.chain_use_id,
        edge_id,
    )
    provenance = make_reference_provenance(
        support_ids=frozenset({support_id}),
        physical_edge_ids=frozenset({edge_id.value}),
        physical_chain_ids=frozenset({chain.physical_chain_id.value}),
        boundary_loop_ids=frozenset({boundary_loop_id.value}),
        chain_use_ids=frozenset({chain_use.chain_use_id.value}),
        patch_domain_ids=frozenset(
            {context.compilation.plan_key.patch_domain_id.value}
        ),
        boundary_constraint_ids=constraint_ids,
        lineage_ids=frozenset(item.value for item in chain.source_lineage),
    )
    return make_segment(
        stable_id(
            "sparse-domain-edge",
            boundary_loop_id,
            chain_use.chain_use_id,
            edge_id,
        ),
        context.points_by_id[start_id],
        context.points_by_id[end_id],
        support_ids=frozenset({support_id}),
        provenance=provenance,
        start_certificates=frozenset({source_vertex_certificate(start_id)}),
        end_certificates=frozenset({source_vertex_certificate(end_id)}),
        boundary_constraint_ids=constraint_ids,
    )


def _build_boundary_loop(
    context: GeometryContext,
    loop,
    constraints_by_edge: dict[PhysicalEdgeId, set],
) -> PlanarLoop:
    segments = []
    for chain_use_id in loop.ordered_chain_use_ids:
        chain_use = context.uses_by_id.get(chain_use_id)
        if chain_use is None:
            raise _fail(f"BoundaryLoop {loop.boundary_loop_id} references unknown ChainUse")
        if (
            chain_use.patch_domain_id
            != context.compilation.plan_key.patch_domain_id
            or chain_use.boundary_loop_id != loop.boundary_loop_id
        ):
            raise _fail(
                f"BoundaryLoop {loop.boundary_loop_id} and ChainUse "
                f"{chain_use.chain_use_id} are not reciprocal"
            )
        for start_id, end_id, edge_id in _chain_segment_records(context, chain_use):
            constraint_ids = frozenset(
                item.boundary_constraint_id.value
                for item in constraints_by_edge.get(edge_id, ())
            )
            segments.append(
                _loop_segment(
                    context,
                    boundary_loop_id=loop.boundary_loop_id,
                    chain_use=chain_use,
                    start_id=start_id,
                    end_id=end_id,
                    edge_id=edge_id,
                    constraint_ids=constraint_ids,
                )
            )
    planar_loop = PlanarLoop(loop.boundary_loop_id.value, tuple(segments))
    _validate_closed_loop(planar_loop)
    return _normalize_loop_winding(context, planar_loop, loop.kind)


def _explicit_barrier_segments(
    context: GeometryContext,
    constraints_by_edge: dict[PhysicalEdgeId, set],
    boundary_edge_ids: frozenset[PhysicalEdgeId],
) -> tuple[BlockingBoundarySegment, ...]:
    explicit_constraints = tuple(
        constraint
        for constraint in context.snapshot.boundary_constraints
        if constraint.patch_domain_id
        == context.compilation.plan_key.patch_domain_id
        and constraint.constraint_kind
        is BoundaryConstraintKind.PHYSICAL_DOMAIN_BARRIER
    )
    explicit_edge_ids = frozenset(
        edge_id
        for constraint in explicit_constraints
        for edge_id in _constraint_edges(context, constraint)
    )
    result = []
    for edge_id in sorted(explicit_edge_ids - boundary_edge_ids, key=lambda item: item.value):
        edge = context.source_edges_by_id[edge_id]
        constraints = constraints_by_edge.get(edge_id, ())
        constraint_ids = frozenset(
            item.boundary_constraint_id.value for item in constraints
        )
        chain_uses = tuple(
            chain_use
            for chain_use in context.snapshot.chain_uses
            if edge_id
            in context.chains_by_id[
                chain_use.physical_chain_id
            ].ordered_physical_edge_ids
            and chain_use.patch_domain_id
            == context.compilation.plan_key.patch_domain_id
        )
        chain_ids = frozenset(
            item.physical_chain_id.value for item in chain_uses
        )
        loop_ids = frozenset(
            item.boundary_loop_id.value
            for item in chain_uses
            if item.boundary_loop_id is not None
        )
        lineage_ids = frozenset(
            lineage.value
            for chain_use in chain_uses
            for lineage in context.chains_by_id[
                chain_use.physical_chain_id
            ].source_lineage
        )
        support_id = stable_id(
            "explicit-barrier-support",
            context.compilation.plan_key.patch_domain_id,
            edge_id,
        )
        segment = make_segment(
            stable_id("explicit-barrier-edge", edge_id),
            context.points_by_id[edge.vertex_a_id],
            context.points_by_id[edge.vertex_b_id],
            support_ids=frozenset({support_id}),
            provenance=make_reference_provenance(
                support_ids=frozenset({support_id}),
                physical_edge_ids=frozenset({edge_id.value}),
                physical_chain_ids=chain_ids,
                boundary_loop_ids=loop_ids,
                chain_use_ids=frozenset(
                    item.chain_use_id.value for item in chain_uses
                ),
                patch_domain_ids=frozenset(
                    {context.compilation.plan_key.patch_domain_id.value}
                ),
                boundary_constraint_ids=constraint_ids,
                lineage_ids=lineage_ids,
            ),
            start_certificates=frozenset(
                {source_vertex_certificate(edge.vertex_a_id)}
            ),
            end_certificates=frozenset(
                {source_vertex_certificate(edge.vertex_b_id)}
            ),
            boundary_constraint_ids=constraint_ids,
        )
        result.append(
            BlockingBoundarySegment(segment, BoundaryRole.EXPLICIT_BARRIER)
        )
    return tuple(result)


def build_sparse_patch_domain_geometry(
    context: GeometryContext,
) -> SparsePatchDomainGeometryV1:
    """Build the runtime domain without reading source-face edge cycles."""

    domain_id = context.compilation.plan_key.patch_domain_id
    loops = tuple(
        sorted(
            (
                item
                for item in context.snapshot.boundary_loops
                if item.patch_domain_id == domain_id
            ),
            key=lambda item: item.boundary_loop_id.value,
        )
    )
    constraints_by_edge: dict[PhysicalEdgeId, set] = {}
    for constraint in context.snapshot.boundary_constraints:
        if constraint.patch_domain_id != domain_id:
            continue
        for edge_id in _constraint_edges(context, constraint):
            constraints_by_edge.setdefault(edge_id, set()).add(constraint)

    planar_by_id = {
        loop.boundary_loop_id: _build_boundary_loop(
            context, loop, constraints_by_edge
        )
        for loop in loops
    }
    outer_loops = tuple(
        planar_by_id[item.boundary_loop_id]
        for item in loops
        if item.kind is BoundaryLoopKind.OUTER
    )
    hole_loops = tuple(
        planar_by_id[item.boundary_loop_id]
        for item in loops
        if item.kind is BoundaryLoopKind.HOLE
    )
    if len(outer_loops) != 1:
        raise _fail(
            "PatchDomain must provide exactly one connected OUTER BoundaryLoopV1"
        )
    boundary_edge_ids = frozenset(
        PhysicalEdgeId(edge_id)
        for loop in (*outer_loops, *hole_loops)
        for segment in loop.segments
        for edge_id in segment.provenance.physical_edge_ids
    )
    geometry = SparsePatchDomainGeometryV1(
        patch_domain_id=domain_id,
        outer_loops=outer_loops,
        hole_loops=hole_loops,
        explicit_barriers=_explicit_barrier_segments(
            context, constraints_by_edge, boundary_edge_ids
        ),
        source_face_contributor_ids=frozenset(
            face.face_id
            for face in context.snapshot.surface_ir.source_faces
            if face.patch_id == context.compilation.owner_patch_id
        ),
    )
    # Force the connected-domain invariant at construction time.
    geometry.domain_regions
    return geometry
