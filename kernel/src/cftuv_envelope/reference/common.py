"""Shared exact geometry extraction for EC2 Envelope variants."""

from __future__ import annotations

from dataclasses import dataclass
from hashlib import sha256

import sympy as sp

from ..contracts.analysis import (
    AnalysisSnapshotV1,
    CertifiedPlanarSupportDirectionV1,
    ChainUseOrientation,
    ChainUseV1,
    OrientedOwnerSectorV1,
    PlanarPatchFrameV1,
)
from ..contracts.metric import (
    CertifiedAffineSupportDirectionV2,
    RationalAffinePlanarMetricV2,
)
from .._density_policy import is_huber_density_policy
from ..ids import ChainUseId, PhysicalEdgeId, SourceVertexId
from ..robust.grid import reset_snap_counts, set_active_grid
from .contracts import ReferenceEnvelopeCompilationV1, ReferenceOutcome
from .planar_types import (
    BoundedSupportSegment,
    ConstructionCertificate,
    ConstructionKind,
    ExactPlanarPoint,
    ExactPlanarVector,
    PlanarLoop,
    PlanarRegion,
    exact_normalize,
    exact_sign,
    point_add,
    point_sub,
    polygon_signed_area,
    vector_scale,
)
from .metric import ExactPlanarMetric, _DensityExactMemo
from .provenance import (
    ReferenceProvenanceV1,
    make_reference_provenance,
    merge_provenance,
)


class ReferenceGeometryError(ValueError):
    def __init__(self, outcome: ReferenceOutcome, message: str):
        super().__init__(message)
        self.outcome = outcome


def stable_id(kind: str, *parts: object) -> str:
    payload = "\x1f".join((kind, *(str(part) for part in parts))).encode("utf-8")
    return f"{kind}:{sha256(payload).hexdigest()[:24]}"


@dataclass(frozen=True, slots=True)
class SourceSupportSegment:
    chain_use_id: ChainUseId
    physical_edge_id: PhysicalEdgeId
    source_vertex_start_id: SourceVertexId
    source_vertex_end_id: SourceVertexId
    start: ExactPlanarPoint
    end: ExactPlanarPoint
    tangent: ExactPlanarVector
    owner_normal: ExactPlanarVector
    support_id: str
    front_component_id: str
    provenance: ReferenceProvenanceV1


@dataclass(frozen=True, slots=True)
class GeometryContext:
    compilation: ReferenceEnvelopeCompilationV1
    snapshot: AnalysisSnapshotV1
    frame: PlanarPatchFrameV1 | RationalAffinePlanarMetricV2
    metric: ExactPlanarMetric
    points_by_id: dict[SourceVertexId, ExactPlanarPoint]
    uses_by_id: dict
    chains_by_id: dict
    source_edges_by_id: dict
    source_faces_by_id: dict
    provenance_by_spec_id: dict[str, ReferenceProvenanceV1]
    angular_ideal_cache: dict[object, tuple]
    angular_support_cache: dict[object, tuple]
    support_segment_cache: dict[tuple[ChainUseId, str], tuple]
    support_geometry_key: tuple

    @classmethod
    def build(
        cls,
        compilation: ReferenceEnvelopeCompilationV1,
        frame: PlanarPatchFrameV1 | RationalAffinePlanarMetricV2,
        *,
        require_evaluation_binding: bool = True,
        require_certified_bound_supports: bool = True,
        density_exact_memo: _DensityExactMemo | None = None,
    ) -> GeometryContext:
        snapshot = compilation.analysis_snapshot
        from .evaluation_geometry import (
            EvaluationGeometryBindingInvalid,
            verify_evaluation_geometry_binding,
        )

        binding = compilation.evaluation_geometry_binding
        if binding is not None or require_evaluation_binding:
            try:
                verify_evaluation_geometry_binding(
                    binding,
                    compilation,
                    frame,
                    require_certified_bound_supports=(
                        require_certified_bound_supports
                    ),
                )
            except EvaluationGeometryBindingInvalid as exc:
                raise ReferenceGeometryError(
                    ReferenceOutcome.REFERENCE_EVALUATION_GEOMETRY_BINDING_INVALID,
                    str(exc),
                ) from exc
        if binding is not None:
            coordinate_records = binding.source_vertex_coordinates
        elif isinstance(frame, PlanarPatchFrameV1):
            coordinate_records = frame.source_vertex_coordinates
        else:
            coordinate_records = frame.exact_source_vertex_coordinates
        points = {}
        for item in coordinate_records:
            coordinate = item.domain_coordinate
            if isinstance(frame, PlanarPatchFrameV1):
                x, y = coordinate.x, coordinate.y
            else:
                x = sp.Rational(
                    coordinate.x.numerator, coordinate.x.denominator
                )
                y = sp.Rational(
                    coordinate.y.numerator, coordinate.y.denominator
                )
            points[item.source_vertex_id] = ExactPlanarPoint.from_values(x, y)
        metric = ExactPlanarMetric.from_descriptor(
            frame,
            density_exact_memo=density_exact_memo,
        )
        # Граница домена. Слияние осмысленно только внутри одного домена,
        # поэтому наблюдение обнуляется здесь, а решётка объявляется здесь же:
        # метрика — её единственный источник, всё остальное её только читает.
        reset_snap_counts()
        set_active_grid(metric.chart_grid)
        context = cls(
            compilation=compilation,
            snapshot=snapshot,
            frame=frame,
            metric=metric,
            points_by_id=points,
            uses_by_id={item.chain_use_id: item for item in snapshot.chain_uses},
            chains_by_id={item.physical_chain_id: item for item in snapshot.physical_chains},
            source_edges_by_id={item.edge_id: item for item in snapshot.surface_ir.source_edges},
            source_faces_by_id={item.face_id: item for item in snapshot.surface_ir.source_faces},
            provenance_by_spec_id={
                item.envelope_spec_id: item.provenance
                for item in compilation.source_provenance
            },
            angular_ideal_cache={},
            angular_support_cache={},
            support_segment_cache=(
                {}
                if density_exact_memo is None
                else density_exact_memo.support_segments
            ),
            support_geometry_key=tuple(
                sorted(
                    (
                        vertex_id.value,
                        point.x.expression,
                        point.y.expression,
                    )
                    for vertex_id, point in points.items()
                )
            ),
        )
        if binding is not None and require_certified_bound_supports:
            from .angular import verify_evaluation_direction_binding_reasons

            verify_evaluation_direction_binding_reasons(context)
        return context

    def directed_chain_vertices(self, chain_use: ChainUseV1) -> tuple[SourceVertexId, ...]:
        chain = self.chains_by_id[chain_use.physical_chain_id]
        vertices = chain.ordered_source_vertex_ids
        if chain_use.orientation is ChainUseOrientation.B_START_TO_END:
            vertices = tuple(reversed(vertices))
        return vertices

    def _density_bounded(self) -> bool:
        return is_huber_density_policy(
            self.compilation.decal_request.angular_profile_selection_policy_id
        )

    def _sign(self, expression) -> int:
        if not self._density_bounded():
            return exact_sign(expression)
        from .angular import _density_exact_sign

        return _density_exact_sign(expression, self.metric)

    def _dot_g(self, left, right):
        if not self._density_bounded():
            return self.metric.dot_g(left, right)
        from .angular import _density_dot_expression

        return _density_dot_expression(self.metric, left, right)

    def _oriented_cross(self, left, right):
        if not self._density_bounded():
            return self.metric.oriented_cross(left, right)
        lx, ly = self.metric.density_expressions(left)
        rx, ry = self.metric.density_expressions(right)
        return self.metric.owner_orientation_sign * (lx * ry - ly * rx)

    def _unit_g(self, vector):
        if not self._density_bounded():
            return self.metric.unit_g(vector)
        from .angular import (
            _density_dot_expression,
            _density_unit_from_squared,
        )

        return _density_unit_from_squared(
            vector,
            _density_dot_expression(self.metric, vector, vector),
            self.metric,
        )

    def _owner_normal_g(self, tangent, *, owner_left, normalize=True):
        if not self._density_bounded():
            return self.metric.owner_normal_g(
                tangent,
                owner_left=owner_left,
                normalize=normalize,
            )
        from .angular import (
            _density_dot_expression,
            _density_runtime_vector,
            _density_unit_from_squared,
        )

        tx, ty = self.metric.density_expressions(tangent)
        gram = self.metric.gram
        covector_x = gram[0][0] * tx + gram[0][1] * ty
        covector_y = gram[1][0] * tx + gram[1][1] * ty
        side = self.metric.owner_orientation_sign * (1 if owner_left else -1)
        normal = _density_runtime_vector(
            -side * covector_y,
            side * covector_x,
        )
        if not normalize:
            return normal
        return _density_unit_from_squared(
            normal,
            _density_dot_expression(self.metric, normal, normal),
            self.metric,
        )

    def _edge_for_pair(
        self, chain, start: SourceVertexId, end: SourceVertexId
    ) -> PhysicalEdgeId:
        allowed = set(chain.ordered_physical_edge_ids)
        matches = [
            edge.edge_id
            for edge in self.snapshot.surface_ir.source_edges
            if edge.edge_id in allowed
            and {edge.vertex_a_id, edge.vertex_b_id} == {start, end}
        ]
        if len(matches) != 1:
            raise ReferenceGeometryError(
                ReferenceOutcome.PHYSICAL_EDGE_PROVENANCE_INCOMPLETE,
                f"physical chain edge provenance is not total for {start}->{end}",
            )
        return matches[0]

    def _analysis_direction(
        self, chain_use_id: ChainUseId, source_vertex_id: SourceVertexId | None = None
    ) -> ExactPlanarVector | None:
        candidates = []
        for sector in self.snapshot.angular_owner_sectors:
            for reference in (sector.incoming_support_ref, sector.outgoing_support_ref):
                if reference.chain_use_id != chain_use_id:
                    continue
                payload = reference.direction_payload
                if isinstance(payload, CertifiedPlanarSupportDirectionV1):
                    candidates.append(
                        ExactPlanarVector.from_values(
                            payload.direction_in_domain.x,
                            payload.direction_in_domain.y,
                        )
                    )
                elif isinstance(payload, CertifiedAffineSupportDirectionV2):
                    if (
                        not isinstance(
                            self.frame, RationalAffinePlanarMetricV2
                        )
                        or payload.reference_metric_id
                        != self.frame.reference_metric_id
                    ):
                        raise ReferenceGeometryError(
                            ReferenceOutcome.PLANAR_OWNER_INTERIOR_DIRECTION_REQUIRED,
                            "affine support direction references another metric",
                        )
                    candidates.append(
                        ExactPlanarVector.from_values(
                            sp.Rational(
                                payload.direction_in_domain.x.numerator,
                                payload.direction_in_domain.x.denominator,
                            ),
                            sp.Rational(
                                payload.direction_in_domain.y.numerator,
                                payload.direction_in_domain.y.denominator,
                            ),
                        )
                    )
        if not candidates:
            return None
        first = self._unit_g(candidates[0])
        for candidate in candidates[1:]:
            normalized = self._unit_g(candidate)
            if (
                self._sign(self._oriented_cross(first, normalized)) != 0
                or self._sign(self._dot_g(first, normalized)) <= 0
            ):
                raise ReferenceGeometryError(
                    ReferenceOutcome.PLANAR_OWNER_INTERIOR_DIRECTION_REQUIRED,
                    f"conflicting certified support directions for {chain_use_id}",
                )
        return first

    def _face_side_normal(
        self,
        chain_use: ChainUseV1,
        edge_id: PhysicalEdgeId,
        start: ExactPlanarPoint,
        end: ExactPlanarPoint,
        tangent: ExactPlanarVector,
    ) -> ExactPlanarVector | None:
        owner_patch = chain_use.owner_patch_id
        adjacent_faces = [
            face
            for face in self.snapshot.surface_ir.source_faces
            if face.patch_id == owner_patch and edge_id in face.edge_cycle
        ]
        cycle_normals = []
        for face in adjacent_faces:
            edge_ordinal = face.edge_cycle.index(edge_id)
            cycle_start = self.points_by_id[face.vertex_cycle[edge_ordinal]]
            cycle_end = self.points_by_id[
                face.vertex_cycle[(edge_ordinal + 1) % len(face.vertex_cycle)]
            ]
            cycle_points = tuple(
                self.points_by_id[item] for item in face.vertex_cycle
            )
            cycle_area_sign = self._sign(
                self._polygon_signed_area(cycle_points)
            )
            same_direction = (
                self._sign(
                    self._dot_g(
                        tangent, self._point_sub(cycle_end, cycle_start)
                    )
                )
                > 0
            )
            coordinate_interior_is_left = (
                cycle_area_sign > 0
            ) == same_direction
            interior_is_left = coordinate_interior_is_left == (
                self.metric.owner_orientation_sign > 0
            )
            cycle_normals.append(
                self._owner_normal_g(
                    tangent,
                    owner_left=interior_is_left,
                    normalize=False,
                )
            )
        if len(cycle_normals) == 1:
            return cycle_normals[0]
        if cycle_normals:
            first = cycle_normals[0]
            if all(
                self._sign(self._oriented_cross(first, item)) == 0
                and self._sign(self._dot_g(first, item)) > 0
                for item in cycle_normals[1:]
            ):
                return first
        if chain_use.orientation in (
            ChainUseOrientation.A_START_TO_END,
            ChainUseOrientation.B_START_TO_END,
        ):
            # B reverses the directed physical path above; owner interior remains
            # the left side of the semantic direction for both patch-side uses.
            return self._owner_normal_g(
                tangent,
                owner_left=True,
                normalize=False,
            )
        return None

    def _polygon_signed_area(self, points) -> sp.Expr:
        """Площадь Density читает через memo текущей geometry-транзакции."""

        if not self._density_bounded():
            return polygon_signed_area(points)
        vertices = tuple(points)
        if len(vertices) < 3:
            return sp.Integer(0)
        twice_area = sp.Integer(0)
        for start, end in zip(vertices, vertices[1:] + vertices[:1]):
            sx, sy = self.metric.density_expressions(start)
            ex, ey = self.metric.density_expressions(end)
            twice_area += sx * ey - sy * ex
        return exact_normalize(twice_area / 2)

    def _point_sub(
        self,
        left: ExactPlanarPoint,
        right: ExactPlanarPoint,
    ) -> ExactPlanarVector:
        """Разность Density-точек не пересекает process-global parser."""

        if not self._density_bounded():
            return point_sub(left, right)
        lx, ly = self.metric.density_expressions(left)
        rx, ry = self.metric.density_expressions(right)
        return ExactPlanarVector.from_values(lx - rx, ly - ry)

    def _validate_segment_corners(
        self,
        segments: list[SourceSupportSegment],
        *,
        chain_is_closed: bool,
        chain_use_id: ChainUseId,
    ) -> None:
        """Проверить нелинейные стыки сегментов против объявленных углов."""

        adjacent = tuple(zip(segments, segments[1:]))
        if chain_is_closed and len(segments) > 1:
            adjacent += ((segments[-1], segments[0]),)
        sectors_by_id = {
            item.owner_sector_id: item
            for item in self.snapshot.angular_owner_sectors
        }
        declared_corner_vertices = {
            relation.source_vertex_id
            for relation in self.snapshot.corner_relations
            if relation.owner_sector_id in sectors_by_id
            and chain_use_id
            in sectors_by_id[
                relation.owner_sector_id
            ].ordered_incident_chain_use_ids
        }
        for incoming, outgoing in adjacent:
            if self._sign(
                self._oriented_cross(
                    incoming.tangent, outgoing.tangent
                )
            ) == 0:
                continue
            if incoming.source_vertex_end_id not in declared_corner_vertices:
                raise ReferenceGeometryError(
                    ReferenceOutcome.PLANAR_CHAIN_SUPPORT_NOT_LINEAR,
                    "non-linear ChainUse requires an explicit segment-corner "
                    f"support relation at {incoming.source_vertex_end_id}",
                )

    def support_segments_for_use(
        self, chain_use_id: ChainUseId, spec_id: str
    ) -> tuple[SourceSupportSegment, ...]:
        chain_use = self.uses_by_id[chain_use_id]
        chain = self.chains_by_id[chain_use.physical_chain_id]
        vertices = self.directed_chain_vertices(chain_use)
        pairs = tuple(zip(vertices, vertices[1:]))
        if chain.is_closed:
            pairs += ((vertices[-1], vertices[0]),)
        if not pairs:
            raise ReferenceGeometryError(
                ReferenceOutcome.PLANAR_CHAIN_SUPPORT_NOT_LINEAR,
                f"ChainUse {chain_use_id} has no bounded support segments",
            )
        component_ids = sorted(
            (
                item.front_component_id.value
                for item in self.compilation.front_components
                if item.chain_use_id == chain_use_id
            )
        )
        if len(component_ids) != 1:
            raise ReferenceGeometryError(
                ReferenceOutcome.PLANAR_OWNER_INTERIOR_DIRECTION_REQUIRED,
                f"v1 geometry requires one owner-interior component for {chain_use_id}",
            )
        base_provenance = self.provenance_by_spec_id[spec_id]
        cache_key = (
            self.support_geometry_key,
            chain_use_id,
            spec_id,
            base_provenance,
            tuple(component_ids),
        )
        cached = self.support_segment_cache.get(cache_key)
        if cached is not None:
            return cached
        # Chart-lattice binding двигает концы физического ребра и тем самым
        # объявляет НОВУЮ evaluation-прямую. Analysis direction остаётся
        # сертификатом исходной геометрии и после такого сдвига уже не обязана
        # быть точно касательной/нормалью новой прямой. Сторону owner'а берём
        # из того же face-cycle, но нормаль выводим из bound edge один раз для
        # обоих consumers.
        analysis_direction = (
            None
            if self.compilation.evaluation_geometry_binding is not None
            else self._analysis_direction(chain_use_id)
        )
        result = []
        for ordinal, (start_id, end_id) in enumerate(pairs):
            if start_id not in self.points_by_id or end_id not in self.points_by_id:
                raise ReferenceGeometryError(
                    ReferenceOutcome.REFERENCE_PLANAR_FRAME_REQUIRED,
                    f"planar metric lacks chain coordinate {start_id}->{end_id}",
                )
            start = self.points_by_id[start_id]
            end = self.points_by_id[end_id]
            edge_id = self._edge_for_pair(chain, start_id, end_id)
            tangent = self._unit_g(self._point_sub(end, start))
            face_normal = self._face_side_normal(chain_use, edge_id, start, end, tangent)
            normal = None
            if analysis_direction is not None:
                if self._sign(
                    self._dot_g(analysis_direction, tangent)
                ) == 0:
                    normal = analysis_direction
                elif self._sign(
                    self._oriented_cross(
                        analysis_direction, tangent
                    )
                ) == 0:
                    normal = face_normal
                else:
                    raise ReferenceGeometryError(
                        ReferenceOutcome.PLANAR_OWNER_INTERIOR_DIRECTION_REQUIRED,
                        f"certified support direction is neither tangent nor normal for {edge_id}",
                    )
            normal = normal or face_normal
            if normal is None:
                raise ReferenceGeometryError(
                    ReferenceOutcome.PLANAR_OWNER_INTERIOR_DIRECTION_REQUIRED,
                    f"owner-interior normal is ambiguous for {edge_id}",
                )
            normal = self._unit_g(normal)
            support_id = stable_id("source-support", chain_use_id, edge_id)
            source_faces = frozenset(
                face.face_id.value
                for face in self.snapshot.surface_ir.source_faces
                if edge_id in face.edge_cycle
            )
            provenance = merge_provenance(
                base_provenance,
                make_reference_provenance(
                    support_ids=frozenset({support_id}),
                    physical_edge_ids=frozenset({edge_id.value}),
                    source_face_ids=source_faces,
                ),
            )
            result.append(
                SourceSupportSegment(
                    chain_use_id=chain_use_id,
                    physical_edge_id=edge_id,
                    source_vertex_start_id=start_id,
                    source_vertex_end_id=end_id,
                    start=start,
                    end=end,
                    tangent=tangent,
                    owner_normal=normal,
                    support_id=support_id,
                    front_component_id=component_ids[0],
                    provenance=provenance,
                )
            )
        self._validate_segment_corners(
            result,
            chain_is_closed=chain.is_closed,
            chain_use_id=chain_use_id,
        )
        sealed = tuple(result)
        self.support_segment_cache[cache_key] = sealed
        return sealed


def source_vertex_certificate(source_vertex_id: SourceVertexId) -> ConstructionCertificate:
    return ConstructionCertificate(
        kind=ConstructionKind.SOURCE_VERTEX,
        source_vertex_ids=frozenset({source_vertex_id.value}),
    )


def support_vertex_certificate(*support_ids: str) -> ConstructionCertificate:
    return ConstructionCertificate(
        kind=ConstructionKind.SUPPORT_SUPPORT_INTERSECTION,
        support_ids=frozenset(support_ids),
    )


def make_segment(
    segment_id: str,
    start: ExactPlanarPoint,
    end: ExactPlanarPoint,
    *,
    support_ids: frozenset[str],
    provenance: ReferenceProvenanceV1,
    start_certificates: frozenset[ConstructionCertificate],
    end_certificates: frozenset[ConstructionCertificate],
    boundary_constraint_ids: frozenset[str] = frozenset(),
) -> BoundedSupportSegment:
    return BoundedSupportSegment(
        segment_id=segment_id,
        start=start,
        end=end,
        support_ids=support_ids,
        boundary_constraint_ids=boundary_constraint_ids,
        provenance=provenance,
        start_constructions=start_certificates,
        end_constructions=end_certificates,
    )


def make_region(
    region_id: str,
    segments: tuple[BoundedSupportSegment, ...],
    *,
    instance_id: str,
    spec_id: str,
) -> PlanarRegion | None:
    area = polygon_signed_area(segment.start for segment in segments)
    sign = exact_sign(area)
    if sign == 0:
        return None
    if sign < 0:
        segments = tuple(segment.reversed() for segment in reversed(segments))
    return PlanarRegion(
        region_id=region_id,
        outer=PlanarLoop(stable_id("envelope-loop", region_id), segments),
        contributor_instance_ids=frozenset({instance_id}),
        contributor_spec_ids=frozenset({spec_id}),
    )


def offset_point(
    point: ExactPlanarPoint, normal: ExactPlanarVector, alpha: sp.Expr
) -> ExactPlanarPoint:
    return point_add(point, vector_scale(normal, alpha))
