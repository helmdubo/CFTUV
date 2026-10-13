"""Проверка снапшота анализа хоста: структурные ссылки и уникальность, метрики домена, углы, стыки, терминалы (шов EXPORT `snapshot_check`).

ПЕРЕНОС БЕЗ ИЗМЕНЕНИЯ ПОВЕДЕНИЯ из `validation.py` (EXPORT_SLICE_S0): `validate_analysis_snapshot`, её два интервальных предиката и разбор пар маршрутов стыка
(`junction_route_pairs`, его зовёт и `validate_cross_contract_references`). Тело функции то же; отличие одно - вызов `_junction_route_pairs` стал вызовом
`junction_route_pairs`. Три помощника сборки списка (`check_unique`, `require_refs`, `values_of`) переехали в `validation_issues`, потому что их зовёт и `validation.py`.
Этот модуль - ЭТАЛОН, с которым сверяется нативная операция `snapshot_check`: он не диспетчеризуется (диспетчер стоит у вызывающего хоста), а его закрепление
следует за правилом `tests/test_architecture.py` о закреплённых эталонах.
"""

from __future__ import annotations

from decimal import Decimal
from .contracts.analysis import (
    ANALYSIS_SNAPSHOT_SCHEMA_V1,
    AnalysisCapability,
    AnalysisSnapshotV1,
    BoundaryLoopConstraintTargetV1,
    CertifiedPlanarSupportDirectionV1,
    CertifiedReflexAngleMeasureV1,
    ChainUseConstraintTargetV1,
    DeclaredRoutePairsTopologyV1,
    IntrinsicSurfaceMetricDescriptorV1,
    JunctionRelationKind,
    JunctionRoutePairV1,
    PhysicalEdgeSequenceConstraintTargetV1,
    PlanarPatchFrameV1,
    RationalAffinePlanarMetricV2,
    SurfaceRegime,
    TerminalEndpointRole,
    TJunctionRouteTopologyV1,
    UnavailableBoundaryConstraintTargetV1,
    UnavailableJunctionRouteTopologyV1,
    UnavailableReflexAngleMeasureV1,
    UnavailableSourceSupportDirectionV1,
    UnavailableSurfaceMetricDescriptorV1,
    XJunctionRouteTopologyV1,
    YJunctionRouteTopologyV1,
)
from .contracts.surface import SurfacePayloadMode
from .ids import OpaqueId
from .numeric import IntervalEndpointKind, LocalPoint3V1
from .validation_chain_station import seam_neighbour_face_issues
from .validation_issues import ValidationCode, ValidationIssue, add_issue as _issue, check_unique as _check_unique, require_refs as _require_refs
from .validation_metric import chart_source_vertices, metric_covers_patch, validate_metric_against_source, validate_rational_affine_planar_metric
from .validation_source_edges import source_edge_zero_length_issues


def _interval_strictly_above(interval: object, value: Decimal) -> bool:
    return interval.lower > value or (
        interval.lower == value and interval.lower_kind is IntervalEndpointKind.OPEN
    )


def _interval_strictly_below(interval: object, value: Decimal) -> bool:
    return interval.upper < value or (
        interval.upper == value and interval.upper_kind is IntervalEndpointKind.OPEN
    )


def junction_route_pairs(topology: object) -> frozenset[JunctionRoutePairV1]:
    if isinstance(topology, TJunctionRouteTopologyV1):
        return frozenset({topology.through_route_pair})
    if isinstance(
        topology,
        (
            XJunctionRouteTopologyV1,
            YJunctionRouteTopologyV1,
            DeclaredRoutePairsTopologyV1,
        ),
    ):
        return topology.route_pairs
    return frozenset()


def validate_analysis_snapshot(
    snapshot: AnalysisSnapshotV1,
    *,
    developable_stretch_budget=None,
) -> tuple[ValidationIssue, ...]:
    issues: list[ValidationIssue] = []
    if snapshot.schema_version != ANALYSIS_SNAPSHOT_SCHEMA_V1:
        _issue(issues, ValidationCode.SCHEMA_VERSION, ("schema_version",), "unsupported snapshot schema")
    if snapshot.surface_ir.schema_version != "cftuv.envelope.patch_surface_ir.v1":
        _issue(issues, ValidationCode.SCHEMA_VERSION, ("surface_ir", "schema_version"), "unsupported surface schema")
    if snapshot.source_revision != snapshot.surface_ir.source_revision:
        _issue(issues, ValidationCode.CROSS_CONTRACT_MISMATCH, ("surface_ir", "source_revision"), "surface and snapshot revisions differ")

    required = {
        AnalysisCapability.PATCH_DOMAIN_V1,
        AnalysisCapability.PHYSICAL_CHAIN_DIRECTED_USES_V1,
        AnalysisCapability.PATCH_SURFACE_IR_V1,
        AnalysisCapability.ORIENTED_OWNER_SECTOR_V1,
        AnalysisCapability.REFLEX_ANGLE_CERTIFICATE_V1,
        AnalysisCapability.RELATION_LINEAGE_V1,
    }
    missing_capabilities = required - set(snapshot.analysis_capabilities)
    if missing_capabilities:
        _issue(issues, ValidationCode.CAPABILITY, ("analysis_capabilities",), "missing required v1 capabilities")

    patch_ids = _check_unique(issues, snapshot.patches, "patch_id", "patches")
    domain_ids = _check_unique(issues, snapshot.patch_domains, "patch_domain_id", "patch_domains")
    vertex_ids = _check_unique(issues, snapshot.source_vertices, "vertex_id", "source_vertices")
    chain_ids = _check_unique(issues, snapshot.physical_chains, "physical_chain_id", "physical_chains")
    use_ids = _check_unique(issues, snapshot.chain_uses, "chain_use_id", "chain_uses")
    loop_ids = _check_unique(issues, snapshot.boundary_loops, "boundary_loop_id", "boundary_loops")
    boundary_ids = _check_unique(issues, snapshot.boundary_constraints, "boundary_constraint_id", "boundary_constraints")
    angular_sector_ids = _check_unique(issues, snapshot.angular_owner_sectors, "owner_sector_id", "angular_owner_sectors")
    angle_certificate_ids = _check_unique(issues, snapshot.reflex_angle_certificates, "certificate_id", "reflex_angle_certificates")
    _check_unique(issues, snapshot.corner_relations, "corner_relation_id", "corner_relations")
    _check_unique(issues, snapshot.junction_relations, "junction_relation_id", "junction_relations")
    _check_unique(issues, snapshot.terminal_relations, "terminal_relation_id", "terminal_relations")

    metric_domain_ids = [item.patch_domain_id for item in snapshot.surface_metric_descriptors]
    if len(metric_domain_ids) != len(set(metric_domain_ids)):
        _issue(issues, ValidationCode.DUPLICATE_ID, ("surface_metric_descriptors",), "duplicate patch_domain_id")
    _require_refs(issues, set(metric_domain_ids), domain_ids, ("surface_metric_descriptors", "patch_domain_id"))
    if set(metric_domain_ids) != domain_ids:
        _issue(issues, ValidationCode.SURFACE_METRIC, ("surface_metric_descriptors",), "exactly one metric descriptor is required per PatchDomain")

    _require_refs(issues, {domain.owner_patch_id for domain in snapshot.patch_domains}, patch_ids, ("patch_domains", "owner_patch_id"))
    _require_refs(issues, snapshot.surface_ir.source_vertex_ids, vertex_ids, ("surface_ir", "source_vertex_ids"))
    if snapshot.surface_ir.source_vertex_ids != vertex_ids:
        _issue(issues, ValidationCode.CROSS_CONTRACT_MISMATCH, ("surface_ir", "source_vertex_ids"), "surface vertex set must equal snapshot vertex set")

    full_surface = snapshot.surface_ir.payload_mode is SurfacePayloadMode.FULL_HOST_SURFACE
    fixture_payload = AnalysisCapability.EC0_COORDINATE_FREE_FIXTURE_V5 in snapshot.analysis_capabilities
    if full_surface:
        if fixture_payload:
            _issue(issues, ValidationCode.CAPABILITY, ("analysis_capabilities",), "fixture capability cannot label full host surface")
        full_required = {
            AnalysisCapability.AUTHORITATIVE_SURFACE_METRIC_V1,
            AnalysisCapability.PHYSICAL_EDGE_TABLE_V1,
            AnalysisCapability.BOUNDARY_CONSTRAINT_TARGET_V1,
            AnalysisCapability.TYPED_JUNCTION_ROUTE_TOPOLOGY_V1,
            AnalysisCapability.TERMINAL_RELATION_V1,
        }
        if full_required - set(snapshot.analysis_capabilities):
            _issue(issues, ValidationCode.CAPABILITY, ("analysis_capabilities",), "full host surface lacks R1 geometry-boundary capabilities")
        for vertex in snapshot.source_vertices:
            if not isinstance(vertex.position, LocalPoint3V1):
                _issue(issues, ValidationCode.CAPABILITY, ("source_vertices", str(vertex.vertex_id), "position"), "full surface requires local coordinates")
    elif not fixture_payload:
        _issue(issues, ValidationCode.CAPABILITY, ("analysis_capabilities",), "coordinate-free payload requires explicit fixture capability")

    domain_by_id = {item.patch_domain_id: item for item in snapshot.patch_domains}
    use_by_id = {item.chain_use_id: item for item in snapshot.chain_uses}
    chain_by_id = {item.physical_chain_id: item for item in snapshot.physical_chains}
    loop_by_id = {item.boundary_loop_id: item for item in snapshot.boundary_loops}
    boundary_by_id = {
        item.boundary_constraint_id: item for item in snapshot.boundary_constraints
    }
    edge_ids = _check_unique(issues, snapshot.surface_ir.source_edges, "edge_id", "surface_ir.source_edges")
    edge_by_id = {item.edge_id: item for item in snapshot.surface_ir.source_edges}
    face_ids = _check_unique(issues, snapshot.surface_ir.source_faces, "face_id", "surface_ir.source_faces")
    triangle_ids = _check_unique(issues, snapshot.surface_ir.surface_triangles, "triangle_id", "surface_ir.surface_triangles")
    triangle_by_id = {item.triangle_id: item for item in snapshot.surface_ir.surface_triangles}
    for edge in snapshot.surface_ir.source_edges:
        _require_refs(issues, {edge.vertex_a_id, edge.vertex_b_id}, vertex_ids, ("surface_ir", "source_edges", str(edge.edge_id), "endpoints"))
    for face in snapshot.surface_ir.source_faces:
        _require_refs(issues, {face.patch_id}, patch_ids, ("surface_ir", "source_faces", str(face.face_id), "patch_id"))
        _require_refs(issues, set(face.vertex_cycle), vertex_ids, ("surface_ir", "source_faces", str(face.face_id), "vertex_cycle"))
        _require_refs(issues, set(face.edge_cycle), edge_ids, ("surface_ir", "source_faces", str(face.face_id), "edge_cycle"))
        _require_refs(issues, set(face.triangle_ids), triangle_ids, ("surface_ir", "source_faces", str(face.face_id), "triangle_ids"))
        for index, edge_id in enumerate(face.edge_cycle):
            edge = edge_by_id.get(edge_id)
            if edge is None:
                continue
            expected = {
                face.vertex_cycle[index],
                face.vertex_cycle[(index + 1) % len(face.vertex_cycle)],
            }
            if {edge.vertex_a_id, edge.vertex_b_id} != expected:
                _issue(issues, ValidationCode.SURFACE_TOPOLOGY, ("surface_ir", "source_faces", str(face.face_id), "edge_cycle", str(index)), "physical edge endpoints do not match the face vertex cycle")
    for triangle in snapshot.surface_ir.surface_triangles:
        _require_refs(issues, {triangle.source_face_id}, face_ids, ("surface_ir", "surface_triangles", str(triangle.triangle_id), "source_face_id"))
        _require_refs(issues, set(triangle.vertex_ids), vertex_ids, ("surface_ir", "surface_triangles", str(triangle.triangle_id), "vertex_ids"))
        _require_refs(issues, {edge_id for edge_id in triangle.physical_edge_ids if edge_id is not None}, edge_ids, ("surface_ir", "surface_triangles", str(triangle.triangle_id), "physical_edge_ids"))
        triangle_pairs = (
            {triangle.vertex_ids[0], triangle.vertex_ids[1]},
            {triangle.vertex_ids[1], triangle.vertex_ids[2]},
            {triangle.vertex_ids[2], triangle.vertex_ids[0]},
        )
        for index, edge_id in enumerate(triangle.physical_edge_ids):
            if edge_id is None or edge_id not in edge_by_id:
                continue
            edge = edge_by_id[edge_id]
            if {edge.vertex_a_id, edge.vertex_b_id} != triangle_pairs[index]:
                _issue(issues, ValidationCode.SURFACE_TOPOLOGY, ("surface_ir", "surface_triangles", str(triangle.triangle_id), "physical_edge_ids", str(index)), "triangle physical edge does not match its endpoint pair")

    listed_triangle_owners: dict[OpaqueId, list[OpaqueId]] = {}
    for face in snapshot.surface_ir.source_faces:
        expected = {triangle.triangle_id for triangle in snapshot.surface_ir.surface_triangles if triangle.source_face_id == face.face_id}
        if set(face.triangle_ids) != expected:
            _issue(issues, ValidationCode.SURFACE_TOPOLOGY, ("surface_ir", "source_faces", str(face.face_id), "triangle_ids"), "face must list all and only triangles whose source_face_id names it")
        for triangle_id in face.triangle_ids:
            listed_triangle_owners.setdefault(triangle_id, []).append(face.face_id)
            triangle = triangle_by_id.get(triangle_id)
            if triangle is not None and triangle.source_face_id != face.face_id:
                _issue(issues, ValidationCode.SURFACE_TOPOLOGY, ("surface_ir", "source_faces", str(face.face_id), "triangle_ids"), "triangle source_face_id disagrees with face listing")
    for triangle_id in triangle_ids:
        if len(listed_triangle_owners.get(triangle_id, [])) != 1:
            _issue(issues, ValidationCode.SURFACE_TOPOLOGY, ("surface_ir", "surface_triangles", str(triangle_id)), "triangle must be listed by exactly one source face")

    patch_vertices: dict[OpaqueId, set[OpaqueId]] = {patch_id: set() for patch_id in patch_ids}
    for face in snapshot.surface_ir.source_faces:
        patch_vertices.setdefault(face.patch_id, set()).update(face.vertex_cycle)
    planarity_ids: set[OpaqueId] = set()
    for descriptor in snapshot.surface_metric_descriptors:
        path = ("surface_metric_descriptors", str(descriptor.patch_domain_id))
        domain = domain_by_id.get(descriptor.patch_domain_id)
        if isinstance(descriptor, UnavailableSurfaceMetricDescriptorV1):
            if full_surface:
                _issue(issues, ValidationCode.SURFACE_METRIC, path, "FULL_HOST_SURFACE cannot use an unavailable metric descriptor")
            continue
        if domain is None:
            continue
        if isinstance(descriptor, PlanarPatchFrameV1):
            certificate = descriptor.planarity_certificate
            if certificate.certificate_id in planarity_ids:
                _issue(issues, ValidationCode.DUPLICATE_ID, path + ("planarity_certificate",), "duplicate planarity certificate ID")
            planarity_ids.add(certificate.certificate_id)
            if domain.surface_regime is not SurfaceRegime.PLANAR:
                _issue(issues, ValidationCode.SURFACE_METRIC, path, "PlanarPatchFrameV1 requires a PLANAR PatchDomain")
            if certificate.patch_domain_id != descriptor.patch_domain_id or not certificate.exact or certificate.max_absolute_plane_distance.value != 0:
                _issue(issues, ValidationCode.SURFACE_METRIC, path + ("planarity_certificate",), "exact planar frame requires a zero-distance certificate for the same domain")
            for field_name in ("axis_u", "axis_v", "normal"):
                vector = getattr(descriptor, field_name)
                if vector.x == 0 and vector.y == 0 and vector.z == 0:
                    _issue(issues, ValidationCode.SURFACE_METRIC, path + (field_name,), "authoritative frame vector cannot be zero")
            coordinate_ids = [item.source_vertex_id for item in descriptor.source_vertex_coordinates]
            if len(coordinate_ids) != len(set(coordinate_ids)):
                _issue(issues, ValidationCode.DUPLICATE_ID, path + ("source_vertex_coordinates",), "duplicate source vertex coordinate")
            _require_refs(issues, set(coordinate_ids), vertex_ids, path + ("source_vertex_coordinates",))
            required_vertices = patch_vertices.get(domain.owner_patch_id, set())
            if full_surface and set(coordinate_ids) != required_vertices:
                _issue(issues, ValidationCode.SURFACE_METRIC, path + ("source_vertex_coordinates",), "planar coordinates must cover all and only owner-patch surface vertices")
        elif isinstance(descriptor, RationalAffinePlanarMetricV2):
            for metric_issue in validate_rational_affine_planar_metric(
                descriptor, developable_stretch_budget=developable_stretch_budget
            ):
                _issue(
                    issues,
                    metric_issue.code,
                    path + metric_issue.path[1:],
                    metric_issue.message,
                )
            certificate = descriptor.planarity_certificate
            if certificate.certificate_id in planarity_ids:
                _issue(issues, ValidationCode.DUPLICATE_ID, path + ("planarity_certificate",), "duplicate planarity certificate ID")
            planarity_ids.add(certificate.certificate_id)
            if domain.surface_regime is not SurfaceRegime.PLANAR:
                _issue(issues, ValidationCode.SURFACE_METRIC, path, "RationalAffinePlanarMetricV2 requires a PLANAR PatchDomain")
            if descriptor.source_revision != snapshot.source_revision:
                _issue(issues, ValidationCode.CROSS_CONTRACT_MISMATCH, path + ("source_revision",), "metric and snapshot revisions differ")
            coordinate_by_id = {
                item.source_vertex_id: item.domain_coordinate
                for item in descriptor.exact_source_vertex_coordinates
            }
            coordinate_ids = chart_source_vertices(coordinate_by_id)
            _require_refs(issues, coordinate_ids, vertex_ids, path + ("exact_source_vertex_coordinates",))
            required_vertices = patch_vertices.get(domain.owner_patch_id, set())
            if full_surface and not metric_covers_patch(certificate, coordinate_ids, required_vertices):
                _issue(issues, ValidationCode.SURFACE_METRIC, path + ("exact_source_vertex_coordinates",), "exact affine coordinates must cover all and only owner-patch surface vertices")
            for source_issue in validate_metric_against_source(
                descriptor, snapshot, domain.owner_patch_id, developable_stretch_budget
            ):
                _issue(issues, source_issue.code, path + source_issue.path[1:], source_issue.message)
        elif isinstance(descriptor, IntrinsicSurfaceMetricDescriptorV1):
            if descriptor.surface_regime != domain.surface_regime or descriptor.surface_regime is SurfaceRegime.PLANAR:
                _issue(issues, ValidationCode.SURFACE_METRIC, path, "intrinsic metric regime must match a non-planar PatchDomain")

    all_sector_ids: set[OpaqueId] = set(angular_sector_ids)
    for domain in snapshot.patch_domains:
        all_sector_ids.update(sector.owner_sector_id for sector in domain.sectors)
        _require_refs(issues, set(domain.boundary_constraint_ids), boundary_ids, ("patch_domains", str(domain.patch_domain_id), "boundary_constraint_ids"))
        _require_refs(issues, {sector.chain_use_id for sector in domain.sectors}, use_ids, ("patch_domains", str(domain.patch_domain_id), "sectors"))
        for constraint_id in domain.boundary_constraint_ids:
            constraint = boundary_by_id.get(constraint_id)
            if constraint is not None and constraint.patch_domain_id != domain.patch_domain_id:
                _issue(issues, ValidationCode.CROSS_CONTRACT_MISMATCH, ("patch_domains", str(domain.patch_domain_id), "boundary_constraint_ids"), "boundary constraint belongs to another PatchDomain")
        for sector in domain.sectors:
            use = use_by_id.get(sector.chain_use_id)
            if use is not None and (
                use.patch_domain_id != domain.patch_domain_id
                or use.owner_patch_id != domain.owner_patch_id
            ):
                _issue(issues, ValidationCode.CROSS_CONTRACT_MISMATCH, ("patch_domains", str(domain.patch_domain_id), "sectors", str(sector.owner_sector_id)), "domain sector ChainUse has another owner/domain")
    for chain in snapshot.physical_chains:
        _require_refs(issues, set(chain.ordered_source_vertex_ids), vertex_ids, ("physical_chains", str(chain.physical_chain_id), "ordered_source_vertex_ids"))
        _require_refs(issues, set(chain.ordered_physical_edge_ids), edge_ids, ("physical_chains", str(chain.physical_chain_id), "ordered_physical_edge_ids"))
        if full_surface:
            vertex_count = len(chain.ordered_source_vertex_ids)
            edge_count = len(chain.ordered_physical_edge_ids)
            expected_edge_count = vertex_count if chain.is_closed else vertex_count - 1
            if vertex_count < 2 or edge_count != expected_edge_count:
                _issue(issues, ValidationCode.SURFACE_TOPOLOGY, ("physical_chains", str(chain.physical_chain_id)), "open chains require E=V-1 and closed chains require E=V")
            else:
                for index, edge_id in enumerate(chain.ordered_physical_edge_ids):
                    edge = edge_by_id.get(edge_id)
                    if edge is None:
                        continue
                    next_index = (index + 1) % vertex_count
                    expected = {chain.ordered_source_vertex_ids[index], chain.ordered_source_vertex_ids[next_index]}
                    if {edge.vertex_a_id, edge.vertex_b_id} != expected:
                        _issue(issues, ValidationCode.SURFACE_TOPOLOGY, ("physical_chains", str(chain.physical_chain_id), "ordered_physical_edge_ids", str(index)), "chain edge endpoints do not match the ordered vertex path")
    for use in snapshot.chain_uses:
        _require_refs(issues, {use.physical_chain_id}, chain_ids, ("chain_uses", str(use.chain_use_id), "physical_chain_id"))
        _require_refs(issues, {use.owner_patch_id}, patch_ids, ("chain_uses", str(use.chain_use_id), "owner_patch_id"))
        _require_refs(issues, {use.patch_domain_id}, domain_ids, ("chain_uses", str(use.chain_use_id), "patch_domain_id"))
        _require_refs(issues, {use.launch_locus.boundary_constraint_id}, boundary_ids, ("chain_uses", str(use.chain_use_id), "launch_locus"))
        domain = domain_by_id.get(use.patch_domain_id)
        if domain is not None and domain.owner_patch_id != use.owner_patch_id:
            _issue(issues, ValidationCode.CROSS_CONTRACT_MISMATCH, ("chain_uses", str(use.chain_use_id)), "ChainUse owner differs from PatchDomain owner")
        launch_constraint = boundary_by_id.get(use.launch_locus.boundary_constraint_id)
        if launch_constraint is not None and launch_constraint.patch_domain_id != use.patch_domain_id:
            _issue(issues, ValidationCode.CROSS_CONTRACT_MISMATCH, ("chain_uses", str(use.chain_use_id), "launch_locus"), "launch constraint belongs to another PatchDomain")
        if use.boundary_loop_id is not None:
            _require_refs(issues, {use.boundary_loop_id}, loop_ids, ("chain_uses", str(use.chain_use_id), "boundary_loop_id"))
        elif full_surface:
            _issue(issues, ValidationCode.MISSING_REFERENCE, ("chain_uses", str(use.chain_use_id), "boundary_loop_id"), "full host ChainUse requires a boundary loop")
    for loop in snapshot.boundary_loops:
        _require_refs(issues, {loop.patch_domain_id}, domain_ids, ("boundary_loops", str(loop.boundary_loop_id), "patch_domain_id"))
        _require_refs(issues, set(loop.ordered_chain_use_ids), use_ids, ("boundary_loops", str(loop.boundary_loop_id), "ordered_chain_use_ids"))
        for use_id in loop.ordered_chain_use_ids:
            use = use_by_id.get(use_id)
            if use is not None and (
                use.patch_domain_id != loop.patch_domain_id
                or use.boundary_loop_id != loop.boundary_loop_id
            ):
                _issue(issues, ValidationCode.CROSS_CONTRACT_MISMATCH, ("boundary_loops", str(loop.boundary_loop_id), "ordered_chain_use_ids"), "BoundaryLoop and ChainUse references are not reciprocal")
    for constraint in snapshot.boundary_constraints:
        _require_refs(issues, {constraint.patch_domain_id}, domain_ids, ("boundary_constraints", str(constraint.boundary_constraint_id), "patch_domain_id"))
        if constraint.originating_chain_use_id is not None:
            _require_refs(issues, {constraint.originating_chain_use_id}, use_ids, ("boundary_constraints", str(constraint.boundary_constraint_id), "originating_chain_use_id"))
            originating_use = use_by_id.get(constraint.originating_chain_use_id)
            if originating_use is not None and originating_use.patch_domain_id != constraint.patch_domain_id:
                _issue(issues, ValidationCode.CROSS_CONTRACT_MISMATCH, ("boundary_constraints", str(constraint.boundary_constraint_id), "originating_chain_use_id"), "originating ChainUse belongs to another PatchDomain")
        target_path = ("boundary_constraints", str(constraint.boundary_constraint_id), "target")
        if isinstance(constraint.target, BoundaryLoopConstraintTargetV1):
            _require_refs(issues, {constraint.target.boundary_loop_id}, loop_ids, target_path)
            loop = loop_by_id.get(constraint.target.boundary_loop_id)
            if loop is not None and loop.patch_domain_id != constraint.patch_domain_id:
                _issue(issues, ValidationCode.CROSS_CONTRACT_MISMATCH, target_path, "target BoundaryLoop belongs to another PatchDomain")
        elif isinstance(constraint.target, ChainUseConstraintTargetV1):
            _require_refs(issues, {constraint.target.chain_use_id}, use_ids, target_path)
            use = use_by_id.get(constraint.target.chain_use_id)
            if use is not None and use.patch_domain_id != constraint.patch_domain_id:
                _issue(issues, ValidationCode.CROSS_CONTRACT_MISMATCH, target_path, "target ChainUse belongs to another PatchDomain")
        elif isinstance(constraint.target, PhysicalEdgeSequenceConstraintTargetV1):
            _require_refs(issues, set(constraint.target.ordered_physical_edge_ids), edge_ids, target_path)
            if full_surface and not constraint.target.ordered_physical_edge_ids:
                _issue(issues, ValidationCode.SURFACE_TOPOLOGY, target_path, "physical-edge boundary target cannot be empty")
        elif isinstance(constraint.target, UnavailableBoundaryConstraintTargetV1) and full_surface:
            _issue(issues, ValidationCode.SURFACE_TOPOLOGY, target_path, "FULL_HOST_SURFACE requires a concrete boundary target")
    for sector in snapshot.angular_owner_sectors:
        _require_refs(issues, {sector.owner_patch_id}, patch_ids, ("angular_owner_sectors", str(sector.owner_sector_id), "owner_patch_id"))
        _require_refs(issues, {sector.patch_domain_id}, domain_ids, ("angular_owner_sectors", str(sector.owner_sector_id), "patch_domain_id"))
        _require_refs(issues, set(sector.ordered_incident_chain_use_ids), use_ids, ("angular_owner_sectors", str(sector.owner_sector_id), "ordered_incident_chain_use_ids"))
        for use_id in sector.ordered_incident_chain_use_ids:
            use = use_by_id.get(use_id)
            if use is not None and (
                use.owner_patch_id != sector.owner_patch_id
                or use.patch_domain_id != sector.patch_domain_id
            ):
                _issue(issues, ValidationCode.CROSS_CONTRACT_MISMATCH, ("angular_owner_sectors", str(sector.owner_sector_id), "ordered_incident_chain_use_ids"), "oriented sector support has another owner/domain")
        if len(sector.ordered_incident_chain_use_ids) < 2:
            _issue(issues, ValidationCode.ANGULAR_CERTIFICATE, ("angular_owner_sectors", str(sector.owner_sector_id)), "ordered incident supports are required")
        elif (
            sector.incoming_support_ref.chain_use_id != sector.ordered_incident_chain_use_ids[0]
            or sector.outgoing_support_ref.chain_use_id != sector.ordered_incident_chain_use_ids[-1]
        ):
            _issue(issues, ValidationCode.ANGULAR_CERTIFICATE, ("angular_owner_sectors", str(sector.owner_sector_id)), "incoming/outgoing refs must match ordered supports")
        _require_refs(issues, {sector.incoming_support_ref.source_launch_boundary_id, sector.outgoing_support_ref.source_launch_boundary_id}, boundary_ids, ("angular_owner_sectors", str(sector.owner_sector_id), "support_refs"))
        for label, support_ref in (
            ("incoming_support_ref", sector.incoming_support_ref),
            ("outgoing_support_ref", sector.outgoing_support_ref),
        ):
            payload = support_ref.direction_payload
            path = ("angular_owner_sectors", str(sector.owner_sector_id), label, "direction_payload")
            if isinstance(payload, UnavailableSourceSupportDirectionV1):
                if full_surface:
                    _issue(issues, ValidationCode.ANGULAR_CERTIFICATE, path, "FULL_HOST_SURFACE requires an authoritative planar support direction")
            elif isinstance(payload, CertifiedPlanarSupportDirectionV1):
                direction = payload.direction_in_domain
                if direction.x == 0 and direction.y == 0:
                    _issue(issues, ValidationCode.ANGULAR_CERTIFICATE, path, "support direction cannot be zero")
    for certificate in snapshot.reflex_angle_certificates:
        _require_refs(issues, {certificate.owner_sector_id}, angular_sector_ids, ("reflex_angle_certificates", str(certificate.certificate_id), "owner_sector_id"))
        if certificate.exact_two_pi:
            _issue(issues, ValidationCode.ANGULAR_CERTIFICATE, ("reflex_angle_certificates", str(certificate.certificate_id), "exact_two_pi"), "2*pi is terminal/junction, never Angular")
        sector = next((item for item in snapshot.angular_owner_sectors if item.owner_sector_id == certificate.owner_sector_id), None)
        payload = certificate.measure_payload
        if isinstance(payload, UnavailableReflexAngleMeasureV1):
            if full_surface:
                _issue(issues, ValidationCode.ANGULAR_CERTIFICATE, ("reflex_angle_certificates", str(certificate.certificate_id), "measure_payload"), "FULL_HOST_SURFACE requires a certified angle payload")
        elif isinstance(payload, CertifiedReflexAngleMeasureV1):
            phi = payload.phi_over_pi
            delta = payload.reflex_excess_over_pi
            if not _interval_strictly_above(phi, Decimal(1)) or not _interval_strictly_below(phi, Decimal(2)):
                _issue(issues, ValidationCode.ANGULAR_CERTIFICATE, ("reflex_angle_certificates", str(certificate.certificate_id), "phi_over_pi"), "certified phi interval must be strictly inside (1, 2)")
            if (
                delta.lower != phi.lower - Decimal(1)
                or delta.upper != phi.upper - Decimal(1)
                or delta.lower_kind is not phi.lower_kind
                or delta.upper_kind is not phi.upper_kind
            ):
                _issue(issues, ValidationCode.ANGULAR_CERTIFICATE, ("reflex_angle_certificates", str(certificate.certificate_id), "reflex_excess_over_pi"), "delta interval must equal phi_over_pi minus one exactly")
            if sector is not None and payload.orientation is not sector.turn_orientation:
                _issue(issues, ValidationCode.ANGULAR_CERTIFICATE, ("reflex_angle_certificates", str(certificate.certificate_id), "orientation"), "angle orientation differs from owner sector")
    for relation in snapshot.corner_relations:
        _require_refs(issues, {relation.source_vertex_id}, vertex_ids, ("corner_relations", str(relation.corner_relation_id), "source_vertex_id"))
        _require_refs(issues, {relation.owner_sector_id}, angular_sector_ids, ("corner_relations", str(relation.corner_relation_id), "owner_sector_id"))
        _require_refs(issues, {relation.reflex_angle_certificate_id}, angle_certificate_ids, ("corner_relations", str(relation.corner_relation_id), "reflex_angle_certificate_id"))
        if relation.exact_two_pi:
            _issue(issues, ValidationCode.ANGULAR_CERTIFICATE, ("corner_relations", str(relation.corner_relation_id)), "2*pi CornerRelation is forbidden")
    all_route_pairing_ids: set[OpaqueId] = set()
    for relation in snapshot.junction_relations:
        _require_refs(issues, {relation.source_vertex_id}, vertex_ids, ("junction_relations", str(relation.junction_relation_id), "source_vertex_id"))
        _require_refs(issues, set(relation.incident_chain_use_ids), use_ids, ("junction_relations", str(relation.junction_relation_id), "incident_chain_use_ids"))
        topology = relation.route_topology
        path = ("junction_relations", str(relation.junction_relation_id), "route_topology")
        if isinstance(topology, UnavailableJunctionRouteTopologyV1):
            if full_surface:
                _issue(issues, ValidationCode.ROUTE_TOPOLOGY, path, "FULL_HOST_SURFACE requires typed junction route topology")
            continue
        expected_type = {
            JunctionRelationKind.T: TJunctionRouteTopologyV1,
            JunctionRelationKind.X: XJunctionRouteTopologyV1,
            JunctionRelationKind.Y: YJunctionRouteTopologyV1,
            JunctionRelationKind.CROSS_PATCH: DeclaredRoutePairsTopologyV1,
            JunctionRelationKind.MIXED_ALPHA: DeclaredRoutePairsTopologyV1,
        }[relation.relation_kind]
        if not isinstance(topology, expected_type):
            _issue(issues, ValidationCode.ROUTE_TOPOLOGY, path, "route topology variant does not match junction kind")
        route_pairs = junction_route_pairs(topology)
        pair_ids = [pair.route_pairing_id for pair in route_pairs]
        if len(pair_ids) != len(set(pair_ids)):
            _issue(issues, ValidationCode.DUPLICATE_ID, path, "duplicate route pairing ID")
        duplicate_global_ids = set(pair_ids) & all_route_pairing_ids
        if duplicate_global_ids:
            _issue(issues, ValidationCode.DUPLICATE_ID, path, "route pairing ID is reused by another JunctionRelation")
        all_route_pairing_ids.update(pair_ids)
        referenced_uses: set[OpaqueId] = set()
        for pair in route_pairs:
            if pair.first_chain_use_id == pair.second_chain_use_id:
                _issue(issues, ValidationCode.ROUTE_TOPOLOGY, path + (str(pair.route_pairing_id),), "route pair endpoints must differ")
            referenced_uses.update({pair.first_chain_use_id, pair.second_chain_use_id})
        if isinstance(topology, TJunctionRouteTopologyV1):
            referenced_uses.update(topology.branch_chain_use_ids)
        elif isinstance(topology, YJunctionRouteTopologyV1):
            referenced_uses.add(topology.trunk_chain_use_id)
            referenced_uses.update(topology.ordered_branch_chain_use_ids)
        _require_refs(issues, referenced_uses, set(relation.incident_chain_use_ids), path)
        if full_surface and referenced_uses != set(relation.incident_chain_use_ids):
            _issue(issues, ValidationCode.ROUTE_TOPOLOGY, path, "typed route topology must cover every incident ChainUse")

    for relation in snapshot.terminal_relations:
        path = ("terminal_relations", str(relation.terminal_relation_id))
        _require_refs(issues, {relation.source_vertex_id}, vertex_ids, path + ("source_vertex_id",))
        _require_refs(issues, {relation.chain_use_id}, use_ids, path + ("chain_use_id",))
        _require_refs(issues, {relation.owner_patch_id}, patch_ids, path + ("owner_patch_id",))
        _require_refs(issues, {relation.patch_domain_id}, domain_ids, path + ("patch_domain_id",))
        if relation.owner_sector_id is not None:
            _require_refs(issues, {relation.owner_sector_id}, all_sector_ids, path + ("owner_sector_id",))
        use = use_by_id.get(relation.chain_use_id)
        if use is not None:
            if use.owner_patch_id != relation.owner_patch_id or use.patch_domain_id != relation.patch_domain_id:
                _issue(issues, ValidationCode.TERMINAL_RELATION, path, "terminal owner patch/domain differs from ChainUse")
            chain = chain_by_id.get(use.physical_chain_id)
            if chain is not None and chain.ordered_source_vertex_ids:
                if relation.endpoint_role is TerminalEndpointRole.START and relation.source_vertex_id != chain.ordered_source_vertex_ids[0]:
                    _issue(issues, ValidationCode.TERMINAL_RELATION, path, "START terminal is not the first physical-chain vertex")
                if relation.endpoint_role is TerminalEndpointRole.END and relation.source_vertex_id != chain.ordered_source_vertex_ids[-1]:
                    _issue(issues, ValidationCode.TERMINAL_RELATION, path, "END terminal is not the last physical-chain vertex")
    return source_edge_zero_length_issues(snapshot) + seam_neighbour_face_issues(snapshot) + tuple(issues)
