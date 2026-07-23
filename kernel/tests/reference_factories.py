from __future__ import annotations

from dataclasses import replace
from decimal import Decimal

from cftuv_envelope import *


def straight_snapshot(
    *,
    faces: tuple[tuple[tuple[float, float], ...], ...],
    source_routes: tuple[dict, ...],
    alpha: str = "1",
    revision_name: str = "straight-fixture",
) -> tuple[AnalysisSnapshotV1, DecalRequestV1]:
    """Build a complete planar PatchSurfaceIR from polygonal face cells."""

    revision = SourceRevision(revision_name)
    patch_id = PatchId("patch")
    domain_id = PatchDomainId("domain")
    all_coordinates = []
    for face in faces:
        for point in face:
            if point not in all_coordinates:
                all_coordinates.append(point)
    vertex_id_by_point = {
        point: SourceVertexId(f"v{index}")
        for index, point in enumerate(all_coordinates)
    }
    edge_id_by_key = {}

    def edge_id(start, end):
        key = tuple(sorted((start, end)))
        if key not in edge_id_by_key:
            edge_id_by_key[key] = PhysicalEdgeId(f"e{len(edge_id_by_key)}")
        return edge_id_by_key[key]

    face_records = []
    triangle_records = []
    edge_counts = {}
    for face_index, face in enumerate(faces):
        vertex_cycle = tuple(vertex_id_by_point[item] for item in face)
        edge_cycle = tuple(
            edge_id(face[index], face[(index + 1) % len(face)])
            for index in range(len(face))
        )
        for item in edge_cycle:
            edge_counts[item] = edge_counts.get(item, 0) + 1
        face_id = SourceFaceId(f"face-{face_index}")
        triangle_ids = []
        for index in range(1, len(face) - 1):
            triangle_id = SurfaceTriangleId(f"face-{face_index}-tri-{index - 1}")
            triangle_ids.append(triangle_id)
            triangle_records.append(
                SurfaceTriangleV1(
                    triangle_id,
                    face_id,
                    (vertex_cycle[0], vertex_cycle[index], vertex_cycle[index + 1]),
                    (
                        edge_cycle[0] if index == 1 else None,
                        edge_cycle[index],
                        edge_cycle[-1] if index + 1 == len(face) - 1 else None,
                    ),
                    LocalVector3V1(0.0, 0.0, 1.0),
                )
            )
        face_records.append(
            SourceFaceV1(
                face_id,
                patch_id,
                vertex_cycle,
                edge_cycle,
                LocalVector3V1(0.0, 0.0, 1.0),
                tuple(triangle_ids),
            )
        )
    source_edge_records = frozenset(
        SourceEdgeV1(edge_id_value, vertex_id_by_point[key[0]], vertex_id_by_point[key[1]])
        for key, edge_id_value in edge_id_by_key.items()
    )
    boundary_constraints = []
    for edge_id_value, count in edge_counts.items():
        if count != 1:
            continue
        constraint_id = BoundaryConstraintId(f"domain-boundary:{edge_id_value.value}")
        boundary_constraints.append(
            BoundaryConstraintV1(
                constraint_id,
                domain_id,
                BoundaryConstraintKind.PHYSICAL_DOMAIN_BARRIER,
                PhysicalEdgeSequenceConstraintTargetV1((edge_id_value,), False),
                None,
                False,
                True,
            )
        )
    chains = []
    uses = []
    sectors = []
    terminal_relations = []
    selected_use_ids = []
    for route in source_routes:
        route_points = tuple(route["points"])
        route_vertex_ids = tuple(vertex_id_by_point[item] for item in route_points)
        route_edge_ids = tuple(
            edge_id_by_key[tuple(sorted((start, end)))]
            for start, end in zip(route_points, route_points[1:])
        )
        if route.get("is_closed", False):
            route_edge_ids += (
                edge_id_by_key[tuple(sorted((route_points[-1], route_points[0])))],
            )
        chain_id = PhysicalChainId(f"chain:{route['name']}")
        chains.append(
            PhysicalChainV1(
                chain_id,
                PhysicalChainKind(route.get("kind", "PHYSICAL_DECAL_SOURCE")),
                route.get("is_closed", False),
                route_vertex_ids,
                route_edge_ids,
                frozenset({LineageId(f"lineage:{route['name']}")}),
                frozenset({LineageId(f"data-record:{route['name']}")}),
            )
        )
        for use_index, use_declaration in enumerate(route.get("uses", (("use", "A_START_TO_END"),))):
            use_suffix, orientation = use_declaration
            use_id = ChainUseId(f"use:{route['name']}:{use_suffix}")
            selected_use_ids.append(use_id)
            launch_id = BoundaryConstraintId(f"launch:{use_id.value}")
            boundary_constraints.append(
                BoundaryConstraintV1(
                    launch_id,
                    domain_id,
                    BoundaryConstraintKind.SOURCE_LAUNCH_BOUNDARY,
                    ChainUseConstraintTargetV1(use_id),
                    use_id,
                    False,
                    True,
                )
            )
            uses.append(
                ChainUseV1(
                    use_id,
                    chain_id,
                    patch_id,
                    domain_id,
                    BoundaryLoopId("source-loop"),
                    ChainUseOrientation(orientation),
                    frozenset({ChainUseRole.DECAL_SOURCE}),
                    LaunchLocusV1(launch_id, OwnerInteriorDirection.OWNER_INTERIOR, True),
                )
            )
            sector_id = OwnerSectorId(f"sector:{use_id.value}")
            sectors.append(PatchSectorV1(sector_id, use_id, True, LawId("owner-sector")))
            if not route.get("is_closed", False):
                for endpoint_role, source_vertex_id in (
                    (TerminalEndpointRole.START, route_vertex_ids[0]),
                    (TerminalEndpointRole.END, route_vertex_ids[-1]),
                ):
                    terminal_relations.append(
                        TerminalRelationV1(
                            TerminalRelationId(
                                f"terminal:{use_id.value}:{endpoint_role.value}"
                            ),
                            source_vertex_id,
                            use_id,
                            patch_id,
                            domain_id,
                            TerminalRelationKind.PHYSICAL_CHAIN_ENDPOINT,
                            endpoint_role,
                            sector_id,
                            frozenset({LineageId(f"lineage:{route['name']}")}),
                        )
                    )
    source_vertices = frozenset(
        SourceVertexV1(vertex_id_by_point[point], LocalPoint3V1(point[0], point[1], 0.0))
        for point in all_coordinates
    )
    surface_ir = PatchSurfaceIRV1(
        "cftuv.envelope.patch_surface_ir.v1",
        revision,
        SurfacePayloadMode.FULL_HOST_SURFACE,
        frozenset(vertex_id_by_point.values()),
        source_edge_records,
        frozenset(face_records),
        frozenset(triangle_records),
    )
    frame = PlanarPatchFrameV1(
        domain_id,
        LocalPoint3V1(0.0, 0.0, 0.0),
        LocalVector3V1(1.0, 0.0, 0.0),
        LocalVector3V1(0.0, 1.0, 0.0),
        LocalVector3V1(0.0, 0.0, 1.0),
        PatchFrameHandedness.RIGHT_HANDED_U_V_NORMAL,
        PlanarityCertificateV1(
            PlanarityCertificateId("planarity"),
            domain_id,
            PlanarityLaw.ALL_DOMAIN_SOURCE_VERTICES_ON_CERTIFIED_PLANE_V1,
            True,
            LocalLengthV1(Decimal(0)),
            SurfaceMetricAuthority.HOST_ANALYSIS_AUTHORITATIVE_V1,
        ),
        PlanarProjectionLaw.DOT_WITH_AUTHORITATIVE_U_V_BASIS_V1,
        frozenset(
            PlanarSourceVertexCoordinateV1(vertex_id_by_point[point], LocalPoint2V1(*point))
            for point in all_coordinates
        ),
    )
    snapshot = AnalysisSnapshotV1(
        ANALYSIS_SNAPSHOT_SCHEMA_V1,
        revision,
        _full_capabilities(),
        frozenset({PatchDescriptorV1(patch_id, SurfaceRegime.PLANAR, PatchShapeClass.MIX, frozenset())}),
        frozenset(
            {
                PatchDomainV1(
                    domain_id,
                    patch_id,
                    SurfaceRegime.PLANAR,
                    frozenset(item.boundary_constraint_id for item in boundary_constraints),
                    frozenset(sectors),
                )
            }
        ),
        frozenset({frame}),
        surface_ir,
        source_vertices,
        frozenset(chains),
        frozenset(uses),
        frozenset(
            {
                BoundaryLoopV1(
                    BoundaryLoopId("source-loop"),
                    domain_id,
                    BoundaryLoopKind.OUTER,
                    tuple(selected_use_ids),
                )
            }
        ),
        frozenset(boundary_constraints),
        frozenset(),
        frozenset(),
        frozenset(),
        frozenset(),
        frozenset(terminal_relations),
    )
    return snapshot, reference_request(*selected_use_ids, alpha=alpha)


def reference_request(*chain_use_ids: ChainUseId, alpha: str = "1") -> DecalRequestV1:
    return DecalRequestV1(
        schema_version=DECAL_REQUEST_SCHEMA_V1,
        decal_request_id=DecalRequestId("reference-request"),
        selected_chain_use_ids=frozenset(chain_use_ids),
        requested_alpha=LocalLengthV1(Decimal(alpha)),
        metric_space=MetricSpace.SOURCE_LOCAL_INTRINSIC,
        angular_profile_family_id=AngularProfileFamilyId.LINEAR_REFLEX_EQUAL_V1,
        angular_profile_selection_policy_id=AngularProfileSelectionPolicyId.MIN_K_FOR_MAX_SUBTURN_V1,
        max_subturn_parameter_id=MaxSubturnParameterId.LINEAR_REFLEX_MAX_SUBTURN_V1,
        max_subturn_value_id=MaxSubturnValueId.LINEAR_REFLEX_MAX_SUBTURN_60_DEGREES_V1,
        max_subturn_exact_value=ExactAngleV1(ExactAngleSymbol.PI_OVER_3),
        cap_policy_id=CapPolicyId.PHYSICAL_TERMINAL_LINEAR_CLOSURE_V1,
        boundary_policy_id=BoundaryPolicyId.BOUNDARY_LIMITED_PROPAGATION,
        interaction_policy_id=InteractionPolicyId.INTRAPATCH_POLICY_B_V1,
        ownership_policy_id=OwnershipPolicyId.TOTAL_DISJOINT_RESOLVED_COVERAGE_V1,
        material_policy_id=PolicyId("material"),
        uv_policy_id=PolicyId("uv"),
    )


def physical_seam_snapshot(
    *, cross_junction: bool = False
) -> tuple[AnalysisSnapshotV1, DecalRequestV1, tuple[PatchDomainId, PatchDomainId]]:
    faces = (
        ((0.0, 0.0), (5.0, 0.0), (5.0, 10.0), (0.0, 10.0)),
        ((5.0, 0.0), (10.0, 0.0), (10.0, 10.0), (5.0, 10.0)),
    )
    base, base_request = straight_snapshot(
        faces=faces,
        source_routes=(
            {
                "name": "physical-seam",
                "points": ((5.0, 0.0), (5.0, 10.0)),
                "kind": "PHYSICAL_SEAM",
                "uses": (("a", "A_START_TO_END"), ("b", "B_START_TO_END")),
            },
        ),
        alpha="1",
        revision_name="physical-seam-a-b",
    )
    patch_ids = (PatchId("patch-a"), PatchId("patch-b"))
    domain_ids = (PatchDomainId("domain-a"), PatchDomainId("domain-b"))
    faces_by_index = tuple(sorted(base.surface_ir.source_faces, key=lambda item: item.face_id.value))
    changed_faces = tuple(
        replace(face, patch_id=patch_ids[index])
        for index, face in enumerate(faces_by_index)
    )
    chain = replace(
        next(iter(base.physical_chains)), kind=PhysicalChainKind.PHYSICAL_SEAM
    )
    base_uses = tuple(sorted(base.chain_uses, key=lambda item: item.chain_use_id.value))
    uses = tuple(
        replace(
            base_uses[index],
            owner_patch_id=patch_ids[index],
            patch_domain_id=domain_ids[index],
            boundary_loop_id=BoundaryLoopId(f"loop-{index}"),
        )
        for index in range(2)
    )
    constraints = []
    constraint_ids_by_domain = {item: set() for item in domain_ids}
    face_by_patch = {face.patch_id: face for face in changed_faces}
    for index, (patch_id, domain_id, use) in enumerate(zip(patch_ids, domain_ids, uses, strict=True)):
        for edge_id in face_by_patch[patch_id].edge_cycle:
            constraint_id = BoundaryConstraintId(
                f"domain-boundary:{domain_id.value}:{edge_id.value}"
            )
            constraints.append(
                BoundaryConstraintV1(
                    constraint_id,
                    domain_id,
                    BoundaryConstraintKind.PHYSICAL_DOMAIN_BARRIER,
                    PhysicalEdgeSequenceConstraintTargetV1((edge_id,), False),
                    None,
                    False,
                    True,
                )
            )
            constraint_ids_by_domain[domain_id].add(constraint_id)
        launch_id = BoundaryConstraintId(f"launch:{use.chain_use_id.value}")
        constraints.append(
            BoundaryConstraintV1(
                launch_id,
                domain_id,
                BoundaryConstraintKind.SOURCE_LAUNCH_BOUNDARY,
                ChainUseConstraintTargetV1(use.chain_use_id),
                use.chain_use_id,
                False,
                True,
            )
        )
        constraint_ids_by_domain[domain_id].add(launch_id)
        uses = tuple(
            replace(item, launch_locus=replace(item.launch_locus, boundary_constraint_id=launch_id))
            if item.chain_use_id == use.chain_use_id
            else item
            for item in uses
        )
    sectors = tuple(
        PatchSectorV1(
            OwnerSectorId(f"sector:{use.chain_use_id.value}"),
            use.chain_use_id,
            True,
            LawId("owner-sector"),
        )
        for use in uses
    )
    domains = tuple(
        PatchDomainV1(
            domain_id,
            patch_id,
            SurfaceRegime.PLANAR,
            frozenset(constraint_ids_by_domain[domain_id]),
            frozenset({sectors[index]}),
        )
        for index, (patch_id, domain_id) in enumerate(zip(patch_ids, domain_ids, strict=True))
    )
    coordinate_by_id = {
        item.source_vertex_id: item.domain_coordinate
        for frame in base.surface_metric_descriptors
        if isinstance(frame, PlanarPatchFrameV1)
        for item in frame.source_vertex_coordinates
    }
    frames = tuple(
        PlanarPatchFrameV1(
            domain_id,
            LocalPoint3V1(0.0, 0.0, 0.0),
            LocalVector3V1(1.0, 0.0, 0.0),
            LocalVector3V1(0.0, 1.0, 0.0),
            LocalVector3V1(0.0, 0.0, 1.0),
            PatchFrameHandedness.RIGHT_HANDED_U_V_NORMAL,
            PlanarityCertificateV1(
                PlanarityCertificateId(f"planarity:{domain_id.value}"),
                domain_id,
                PlanarityLaw.ALL_DOMAIN_SOURCE_VERTICES_ON_CERTIFIED_PLANE_V1,
                True,
                LocalLengthV1(Decimal(0)),
                SurfaceMetricAuthority.HOST_ANALYSIS_AUTHORITATIVE_V1,
            ),
            PlanarProjectionLaw.DOT_WITH_AUTHORITATIVE_U_V_BASIS_V1,
            frozenset(
                PlanarSourceVertexCoordinateV1(vertex_id, coordinate_by_id[vertex_id])
                for vertex_id in face_by_patch[patch_id].vertex_cycle
            ),
        )
        for patch_id, domain_id in zip(patch_ids, domain_ids, strict=True)
    )
    terminals = set()
    for index, use in enumerate(uses):
        for endpoint_role, source_vertex_id in (
            (TerminalEndpointRole.START, chain.ordered_source_vertex_ids[0]),
            (TerminalEndpointRole.END, chain.ordered_source_vertex_ids[-1]),
        ):
            if cross_junction and endpoint_role is TerminalEndpointRole.END:
                continue
            terminals.add(
                TerminalRelationV1(
                    TerminalRelationId(
                        f"terminal:{use.chain_use_id.value}:{endpoint_role.value}"
                    ),
                    source_vertex_id,
                    use.chain_use_id,
                    patch_ids[index],
                    domain_ids[index],
                    TerminalRelationKind.PHYSICAL_CHAIN_ENDPOINT,
                    endpoint_role,
                    sectors[index].owner_sector_id,
                    chain.source_lineage,
                )
            )
    junctions = frozenset()
    if cross_junction:
        route_pair = JunctionRoutePairV1(
            RoutePairingId("cross-patch-route"),
            uses[0].chain_use_id,
            uses[1].chain_use_id,
            StationContinuityLaw.SEMANTIC_CHAIN_USE_S_CONTINUOUS_V1,
        )
        junctions = frozenset(
            {
                JunctionRelationV1(
                    JunctionRelationId("cross-patch-junction"),
                    chain.ordered_source_vertex_ids[-1],
                    tuple(item.chain_use_id for item in uses),
                    JunctionRelationKind.CROSS_PATCH,
                    DeclaredRoutePairsTopologyV1(frozenset({route_pair})),
                    SharedSemanticAnchorId("cross-patch-anchor"),
                )
            }
        )
    snapshot = replace(
        base,
        patches=frozenset(
            PatchDescriptorV1(item, SurfaceRegime.PLANAR, PatchShapeClass.MIX, frozenset())
            for item in patch_ids
        ),
        patch_domains=frozenset(domains),
        surface_metric_descriptors=frozenset(frames),
        surface_ir=replace(base.surface_ir, source_faces=frozenset(changed_faces)),
        physical_chains=frozenset({chain}),
        chain_uses=frozenset(uses),
        boundary_loops=frozenset(
            BoundaryLoopV1(
                BoundaryLoopId(f"loop-{index}"),
                domain_ids[index],
                BoundaryLoopKind.OUTER,
                (uses[index].chain_use_id,),
            )
            for index in range(2)
        ),
        boundary_constraints=frozenset(constraints),
        terminal_relations=frozenset(terminals),
        junction_relations=junctions,
    )
    request = replace(
        base_request,
        selected_chain_use_ids=frozenset(item.chain_use_id for item in uses),
    )
    return snapshot, request, domain_ids


def _interval(lower: str, upper: str) -> CertifiedDecimalIntervalV1:
    return CertifiedDecimalIntervalV1(
        lower=Decimal(lower),
        upper=Decimal(upper),
        lower_kind=IntervalEndpointKind.CLOSED,
        upper_kind=IntervalEndpointKind.CLOSED,
        absolute_error_bound=Decimal(upper) - Decimal(lower),
    )


def junction_snapshot(
    kind: JunctionRelationKind,
) -> tuple[AnalysisSnapshotV1, DecalRequestV1]:
    center = (0.0, 0.0)
    boundary = (
        (-5.0, -5.0),
        (0.0, -5.0),
        (5.0, -5.0),
        (5.0, 0.0),
        (5.0, 5.0),
        (0.0, 5.0),
        (-5.0, 5.0),
        (-5.0, 0.0),
    )
    faces = tuple(
        (center, boundary[index], boundary[(index + 1) % len(boundary)])
        for index in range(len(boundary))
    )
    arms = ((0.0, -5.0), (5.0, 0.0), (0.0, 5.0), (-5.0, 0.0))
    arm_count = 4 if kind is JunctionRelationKind.X else 3
    routes = tuple(
        {
            "name": f"arm-{index}",
            "points": (center, arms[index]),
        }
        for index in range(arm_count)
    )
    snapshot, request = straight_snapshot(
        faces=faces,
        source_routes=routes,
        alpha="1",
        revision_name=f"junction-{kind.value.lower()}",
    )
    use_ids = tuple(sorted(request.selected_chain_use_ids, key=lambda item: item.value))
    pairing = lambda name, first, second: JunctionRoutePairV1(
        RoutePairingId(name),
        first,
        second,
        StationContinuityLaw.SEMANTIC_CHAIN_USE_S_CONTINUOUS_V1,
    )
    if kind is JunctionRelationKind.T:
        topology = TJunctionRouteTopologyV1(
            pairing("through", use_ids[0], use_ids[2]),
            frozenset({use_ids[1]}),
        )
    elif kind is JunctionRelationKind.X:
        topology = XJunctionRouteTopologyV1(
            frozenset(
                {
                    pairing("route-a", use_ids[0], use_ids[2]),
                    pairing("route-b", use_ids[1], use_ids[3]),
                }
            )
        )
    elif kind is JunctionRelationKind.Y:
        topology = YJunctionRouteTopologyV1(
            use_ids[0],
            (use_ids[1], use_ids[2]),
            frozenset(
                {
                    pairing("branch-a", use_ids[0], use_ids[1]),
                    pairing("branch-b", use_ids[0], use_ids[2]),
                }
            ),
            StationContinuityLaw.SEMANTIC_CHAIN_USE_S_CONTINUOUS_V1,
        )
    else:
        raise ValueError(kind)
    anchor_id = next(
        item.vertex_id
        for item in snapshot.source_vertices
        if isinstance(item.position, LocalPoint3V1)
        and item.position.x == 0
        and item.position.y == 0
    )
    relation = JunctionRelationV1(
        JunctionRelationId("junction"),
        anchor_id,
        use_ids,
        kind,
        topology,
        SharedSemanticAnchorId("shared-anchor"),
    )
    terminals = frozenset(
        item for item in snapshot.terminal_relations if item.source_vertex_id != anchor_id
    )
    return replace(
        snapshot,
        junction_relations=frozenset({relation}),
        terminal_relations=terminals,
    ), request


def _full_capabilities() -> frozenset[AnalysisCapability]:
    return frozenset(
        {
            AnalysisCapability.PATCH_DOMAIN_V1,
            AnalysisCapability.PHYSICAL_CHAIN_DIRECTED_USES_V1,
            AnalysisCapability.PATCH_SURFACE_IR_V1,
            AnalysisCapability.ORIENTED_OWNER_SECTOR_V1,
            AnalysisCapability.REFLEX_ANGLE_CERTIFICATE_V1,
            AnalysisCapability.RELATION_LINEAGE_V1,
            AnalysisCapability.AUTHORITATIVE_SURFACE_METRIC_V1,
            AnalysisCapability.PHYSICAL_EDGE_TABLE_V1,
            AnalysisCapability.BOUNDARY_CONSTRAINT_TARGET_V1,
            AnalysisCapability.TYPED_JUNCTION_ROUTE_TOPOLOGY_V1,
            AnalysisCapability.TERMINAL_RELATION_V1,
        }
    )


def angular_snapshot(hidden_count: int) -> tuple[AnalysisSnapshotV1, DecalRequestV1]:
    if hidden_count not in (0, 1, 2):
        raise ValueError(hidden_count)
    revision = SourceRevision(f"angular-k{hidden_count}")
    patch_id = PatchId("patch")
    domain_id = PatchDomainId("domain")
    anchor_id = SourceVertexId("anchor")
    incoming_far_id = SourceVertexId("incoming-far")
    outgoing_far_id = SourceVertexId("outgoing-far")
    outgoing_normal = {
        0: (3 / 5, -4 / 5),
        1: (-7 / 25, -24 / 25),
        2: (-117 / 125, -44 / 125),
    }[hidden_count]
    outgoing_tangent = (outgoing_normal[1], -outgoing_normal[0])
    outgoing_far = {
        0: (-4.0, -3.0),
        1: (-4.8, 1.4),
        2: (-1.76, 4.68),
    }[hidden_count]
    coordinates = {
        incoming_far_id: (0.0, 5.0),
        anchor_id: (0.0, 0.0),
        outgoing_far_id: outgoing_far,
        SourceVertexId("d0"): (-10.0, -10.0),
        SourceVertexId("d1"): (10.0, -10.0),
        SourceVertexId("d2"): (10.0, 10.0),
        SourceVertexId("d3"): (-10.0, 10.0),
    }
    source_vertices = frozenset(
        SourceVertexV1(vertex_id, LocalPoint3V1(x, y, 0.0))
        for vertex_id, (x, y) in coordinates.items()
    )
    incoming_edge_id = PhysicalEdgeId("incoming-edge")
    outgoing_edge_id = PhysicalEdgeId("outgoing-edge")
    face_vertex_ids = (
        incoming_far_id,
        anchor_id,
        outgoing_far_id,
        *(SourceVertexId(f"d{i}") for i in range(4)),
    )
    face_edge_ids = (
        incoming_edge_id,
        outgoing_edge_id,
        *(PhysicalEdgeId(f"de{i}") for i in range(5)),
    )
    source_edges = {
        SourceEdgeV1(face_edge_ids[index], face_vertex_ids[index], face_vertex_ids[(index + 1) % len(face_vertex_ids)])
        for index in range(len(face_vertex_ids))
    }
    face_id = SourceFaceId("domain-face")
    triangles = tuple(
        SurfaceTriangleV1(
            SurfaceTriangleId(f"tri-{index}"),
            face_id,
            (face_vertex_ids[0], face_vertex_ids[index], face_vertex_ids[index + 1]),
            (None, face_edge_ids[index], None),
            LocalVector3V1(0.0, 0.0, 1.0),
        )
        for index in range(1, len(face_vertex_ids) - 1)
    )
    surface_ir = PatchSurfaceIRV1(
        schema_version="cftuv.envelope.patch_surface_ir.v1",
        source_revision=revision,
        payload_mode=SurfacePayloadMode.FULL_HOST_SURFACE,
        source_vertex_ids=frozenset(coordinates),
        source_edges=frozenset(source_edges),
        source_faces=frozenset(
            {
                SourceFaceV1(
                    face_id,
                    patch_id,
                    face_vertex_ids,
                    face_edge_ids,
                    LocalVector3V1(0.0, 0.0, 1.0),
                    tuple(item.triangle_id for item in triangles),
                )
            }
        ),
        surface_triangles=frozenset(triangles),
    )
    use_ids = (ChainUseId("incoming-use"), ChainUseId("outgoing-use"))
    chain_ids = (PhysicalChainId("incoming-chain"), PhysicalChainId("outgoing-chain"))
    launch_ids = (BoundaryConstraintId("incoming-launch"), BoundaryConstraintId("outgoing-launch"))
    chains = frozenset(
        {
            PhysicalChainV1(
                chain_ids[0],
                PhysicalChainKind.PHYSICAL_DECAL_SOURCE,
                False,
                (incoming_far_id, anchor_id),
                (incoming_edge_id,),
                frozenset({LineageId("incoming-lineage")}),
                frozenset(),
            ),
            PhysicalChainV1(
                chain_ids[1],
                PhysicalChainKind.PHYSICAL_DECAL_SOURCE,
                False,
                (anchor_id, outgoing_far_id),
                (outgoing_edge_id,),
                frozenset({LineageId("outgoing-lineage")}),
                frozenset(),
            ),
        }
    )
    uses = frozenset(
        ChainUseV1(
            use_id,
            chain_id,
            patch_id,
            domain_id,
            BoundaryLoopId("outer-loop"),
            ChainUseOrientation.A_START_TO_END,
            frozenset({ChainUseRole.DECAL_SOURCE}),
            LaunchLocusV1(launch_id, OwnerInteriorDirection.OWNER_INTERIOR, True),
        )
        for use_id, chain_id, launch_id in zip(use_ids, chain_ids, launch_ids, strict=True)
    )
    outer_constraint_id = BoundaryConstraintId("outer-boundary")
    constraints = frozenset(
        {
            BoundaryConstraintV1(
                outer_constraint_id,
                domain_id,
                BoundaryConstraintKind.PHYSICAL_DOMAIN_BARRIER,
                PhysicalEdgeSequenceConstraintTargetV1(face_edge_ids, True),
                None,
                False,
                True,
            ),
            *(
                BoundaryConstraintV1(
                    launch_id,
                    domain_id,
                    BoundaryConstraintKind.SOURCE_LAUNCH_BOUNDARY,
                    ChainUseConstraintTargetV1(use_id),
                    use_id,
                    False,
                    True,
                )
                for use_id, launch_id in zip(use_ids, launch_ids, strict=True)
            ),
        }
    )
    sector_ids = (OwnerSectorId("angular-sector"), OwnerSectorId("outgoing-sector"))
    angular_sector = OrientedOwnerSectorV1(
        owner_sector_id=sector_ids[0],
        owner_patch_id=patch_id,
        patch_domain_id=domain_id,
        analysis_proven=True,
        ordered_incident_chain_use_ids=use_ids,
        incoming_support_ref=SourceSupportRefV1(
            use_ids[0],
            launch_ids[0],
            CertifiedPlanarSupportDirectionV1(
                LocalVector2V1(1.0, 0.0),
                SupportDirectionAuthority.HOST_ANALYSIS_AUTHORITATIVE_V1,
            ),
        ),
        outgoing_support_ref=SourceSupportRefV1(
            use_ids[1],
            launch_ids[1],
            CertifiedPlanarSupportDirectionV1(
                LocalVector2V1(*outgoing_normal),
                SupportDirectionAuthority.HOST_ANALYSIS_AUTHORITATIVE_V1,
            ),
        ),
        turn_orientation=TurnOrientation.CW_IN_OWNER_PATCH_ORIENTATION,
        interior_selection_law=InteriorSelectionLaw.OWNER_PATCH_INTERIOR_BETWEEN_ORDERED_SUPPORTS,
    )
    bounds = {0: ("0.20", "0.30"), 1: ("0.50", "0.60"), 2: ("0.80", "0.90")}[hidden_count]
    angle_id = AngleCertificateId("angle")
    angle_certificate = ReflexAngleCertificateV1(
        angle_id,
        sector_ids[0],
        AngleMeasureLaw.ORIENTED_OWNER_SECTOR_ANGLE,
        AngleMeasureSource.HOST_ANALYSIS_EXACT_OR_CERTIFIED,
        StrictAngleRangeCertificate.STRICT_PI_LT_PHI_LT_2PI,
        ReflexExcessLaw.DELTA_EQUALS_PHI_MINUS_PI,
        CertifiedReflexAngleMeasureV1(
            _interval(str(Decimal(1) + Decimal(bounds[0])), str(Decimal(1) + Decimal(bounds[1]))),
            _interval(*bounds),
            TurnOrientation.CW_IN_OWNER_PATCH_ORIENTATION,
            AngleNormalizationLaw.VALUE_OVER_SYMBOLIC_PI_V1,
        ),
        False,
    )
    frame = PlanarPatchFrameV1(
        domain_id,
        LocalPoint3V1(0.0, 0.0, 0.0),
        LocalVector3V1(1.0, 0.0, 0.0),
        LocalVector3V1(0.0, 1.0, 0.0),
        LocalVector3V1(0.0, 0.0, 1.0),
        PatchFrameHandedness.RIGHT_HANDED_U_V_NORMAL,
        PlanarityCertificateV1(
            PlanarityCertificateId("planarity"),
            domain_id,
            PlanarityLaw.ALL_DOMAIN_SOURCE_VERTICES_ON_CERTIFIED_PLANE_V1,
            True,
            LocalLengthV1(Decimal(0)),
            SurfaceMetricAuthority.HOST_ANALYSIS_AUTHORITATIVE_V1,
        ),
        PlanarProjectionLaw.DOT_WITH_AUTHORITATIVE_U_V_BASIS_V1,
        frozenset(
            PlanarSourceVertexCoordinateV1(vertex_id, LocalPoint2V1(x, y))
            for vertex_id, (x, y) in coordinates.items()
        ),
    )
    snapshot = AnalysisSnapshotV1(
        ANALYSIS_SNAPSHOT_SCHEMA_V1,
        revision,
        _full_capabilities(),
        frozenset(
            {PatchDescriptorV1(patch_id, SurfaceRegime.PLANAR, PatchShapeClass.MIX, frozenset())}
        ),
        frozenset(
            {
                PatchDomainV1(
                    domain_id,
                    patch_id,
                    SurfaceRegime.PLANAR,
                    frozenset({outer_constraint_id, *launch_ids}),
                    frozenset(
                        {
                            PatchSectorV1(sector_ids[0], use_ids[0], True, LawId("owner-sector")),
                            PatchSectorV1(sector_ids[1], use_ids[1], True, LawId("owner-sector")),
                        }
                    ),
                )
            }
        ),
        frozenset({frame}),
        surface_ir,
        source_vertices,
        chains,
        uses,
        frozenset(
            {BoundaryLoopV1(BoundaryLoopId("outer-loop"), domain_id, BoundaryLoopKind.OUTER, use_ids)}
        ),
        constraints,
        frozenset({angular_sector}),
        frozenset({angle_certificate}),
        frozenset({CornerRelationV1(CornerRelationId("corner"), anchor_id, sector_ids[0], angle_id, False)}),
        frozenset(),
        frozenset(),
    )
    return snapshot, reference_request(*use_ids)
