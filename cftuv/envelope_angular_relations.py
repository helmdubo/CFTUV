"""Угловые отношения снапшота домена: отражённые углы, опоры владельца и записи сертификатов (шов EXPORT `angular`).

ПЕРЕНОС БЕЗ ИЗМЕНЕНИЯ ПОВЕДЕНИЯ из `envelope_request_export._build_angular_relations` (EXPORT_SLICE_S0): тело функции, замыкания `owner_support` и
`direction_payload` и порядок их вызовов те же; новы только заголовок модуля и строка отложенного импорта имён, которые остались в `envelope_request_export`
(модуль зовёт это место при сборке снапшота, поэтому прямой импорт был бы циклом).

ШОВ ДИСПЕТЧЕРИЗАЦИИ. `build_angular_relations` - единственная точка, через которую `build_envelope_analysis_snapshot` строит угловые отношения: этим именем
(`envelope_request_export._build_angular_relations` - тот же объект под прежним именем) будущая нативная операция `angular` подменит цикл по углам, оставив в Python
сборку `CertifiedReflexAngleMeasureV1` (`certified_reflex_measure`). Тест `tests/test_envelope_export_seams.py` держит, что шов один.
"""

from __future__ import annotations

import importlib
from typing import TYPE_CHECKING

from .envelope_angle_certificate import certified_reflex_measure
from .envelope_angular_sites import angular_sites
from .envelope_chart_band import BeyondChartReach, chart_edge_ends, chart_face_points, chart_points, cut_path_vertices
from .envelope_debug_profile import ANGULAR_STAGE_COUNTERS, EnvelopeDebugProfileBuilderV1
from .envelope_host_outcomes import EnvelopeDebugHostOutcome

if TYPE_CHECKING:
    from .envelope_request_export import _HostChainRecord


def build_angular_relations(
    kernel,
    sympy,
    *,
    analysis_bundle,
    revision: str,
    patch_ids: dict[int, object],
    patch_domains: dict[int, object],
    frames: dict[int, object],
    use_id_by_ref: dict[tuple[int, int, int], object],
    sector_id_by_ref: dict[tuple[int, int, int], object],
    launch_id_by_ref: dict[tuple[int, int, int], object],
    record_by_ref: dict[tuple[int, int, int], _HostChainRecord],
    vertex_ids: dict[int, object],
    triangle_ids: dict[int, object],
    profile: EnvelopeDebugProfileBuilderV1 | None = None,
):
    from cftuv_envelope import backend

    raw = dict(analysis_bundle=analysis_bundle, revision=revision, patch_ids=patch_ids,
               patch_domains=patch_domains, frames=frames, use_id_by_ref=use_id_by_ref,
               sector_id_by_ref=sector_id_by_ref, launch_id_by_ref=launch_id_by_ref,
               record_by_ref=record_by_ref, vertex_ids=vertex_ids, triangle_ids=triangle_ids,
               profile=profile)
    if backend.active_angular_backend() is backend.KernelBackendV1.PYTHON:
        return _build_angular_relations_reference(kernel, sympy, **raw)

    from .envelope_angular_binding import build_binding_inputs, build_dependencies
    from cftuv_envelope.reference.symbolic_backend import backend_mode

    done, answer = backend.angular_compute(
        binding_inputs=build_binding_inputs(kernel),
        dependencies=build_dependencies(kernel, sympy), raw=raw,
        symbolic_mode=backend_mode().value,
    )
    if done:
        return answer
    return _build_angular_relations_reference(kernel, sympy, **raw)


def _build_angular_relations_reference(
    kernel,
    sympy,
    *,
    analysis_bundle,
    revision: str,
    patch_ids: dict[int, object],
    patch_domains: dict[int, object],
    frames: dict[int, object],
    use_id_by_ref: dict[tuple[int, int, int], object],
    sector_id_by_ref: dict[tuple[int, int, int], object],
    launch_id_by_ref: dict[tuple[int, int, int], object],
    record_by_ref: dict[tuple[int, int, int], _HostChainRecord],
    vertex_ids: dict[int, object],
    triangle_ids: dict[int, object],
    profile: EnvelopeDebugProfileBuilderV1 | None = None,
):
    from .envelope_request_export import EnvelopeHostAdapterError, _typed_value

    scalar = lambda value: sympy.Rational(str(float(value)))
    metric_types = importlib.import_module(
        "cftuv_envelope.reference.planar_types"
    )
    angle_measure = importlib.import_module(
        "cftuv_envelope.reference.angle_measure"
    )

    def point_map(frame):
        if isinstance(frame, kernel.RationalAffinePlanarMetricV2):
            return {
                item.source_vertex_id: (
                    sympy.Rational(
                        item.domain_coordinate.x.numerator,
                        item.domain_coordinate.x.denominator,
                    ),
                    sympy.Rational(
                        item.domain_coordinate.y.numerator,
                        item.domain_coordinate.y.denominator,
                    ),
                )
                for item in frame.exact_source_vertex_coordinates
            }
        return {
            item.source_vertex_id: (
                scalar(item.domain_coordinate.x),
                scalar(item.domain_coordinate.y),
            )
            for item in frame.source_vertex_coordinates
        }

    angular_sectors = []
    angle_certificates = []
    corner_relations = []
    used_sector_ids = set()
    host_face_by_patch = {
        int(patch_id): analysis_bundle.patch_surface.patch_faces(int(patch_id))
        for patch_id in analysis_bundle.patch_graph.nodes
    }
    normalized_refs_by_source: dict[
        tuple[int, int, int],
        tuple[tuple[int, int, int], ...],
    ] = {}
    source_refs = {
        (
            record.patch_id,
            record.loop_index,
            record.source_chain_index,
        )
        for record in record_by_ref.values()
    }
    for source_ref in source_refs:
        normalized_refs_by_source[source_ref] = tuple(
            ref
            for ref, record in sorted(record_by_ref.items())
            if (
                record.patch_id,
                record.loop_index,
                record.source_chain_index,
            )
            == source_ref
        )

    for patch_id, patch in sorted(analysis_bundle.patch_graph.nodes.items()):
        patch_id = int(patch_id)
        domain_id = patch_domains[patch_id]
        frame = frames[patch_id]
        coordinates = chart_points(point_map(frame), frame)
        charted = getattr(frame.planarity_certificate, "support_triangle_ids", None)
        cut_vertices = cut_path_vertices(frame)
        metric = kernel.ExactPlanarMetric.from_descriptor(frame)
        kernel_vertex_for_host = vertex_ids

        def vector(value):
            return metric_types.ExactPlanarVector.from_values(*value)

        def expressions(value):
            return value.expressions()

        def owner_support(record: _HostChainRecord, anchor_vertex_id: int):
            host_vertices = tuple(int(item) for item in record.chain.vert_indices)
            if anchor_vertex_id == host_vertices[0]:
                start_vertex_id, end_vertex_id = host_vertices[0], host_vertices[1]
                physical_edge_id = int(record.chain.edge_indices[0])
            elif anchor_vertex_id == host_vertices[-1]:
                start_vertex_id, end_vertex_id = host_vertices[-2], host_vertices[-1]
                physical_edge_id = int(record.chain.edge_indices[-1])
            else:
                raise EnvelopeHostAdapterError(
                    EnvelopeDebugHostOutcome.ENVELOPE_DEBUG_EXACT_ANGULAR_CERTIFICATE_UNAVAILABLE,
                    "BoundaryCorner anchor is not a physical ChainUse endpoint",
                    patch_domain_id=domain_id.value,
                )
            start_name, end_name = chart_edge_ends(frame, kernel_vertex_for_host[start_vertex_id], kernel_vertex_for_host[end_vertex_id])
            start, end = coordinates[start_name], coordinates[end_name]
            tangent = (end[0] - start[0], end[1] - start[1])
            tangent_vector = vector(tangent)
            if metric.dot_g(tangent_vector, tangent_vector) == 0:
                raise EnvelopeHostAdapterError(
                    EnvelopeDebugHostOutcome.ENVELOPE_DEBUG_EXACT_ANGULAR_CERTIFICATE_UNAVAILABLE,
                    "BoundaryCorner incident support is degenerate",
                    patch_domain_id=domain_id.value,
                )
            candidates = []
            for face in host_face_by_patch[patch_id]:
                if physical_edge_id not in face.edge_cycle:
                    continue
                if charted is not None and charted.isdisjoint(triangle_ids[int(item)] for item in face.triangle_ids):
                    # Вершины грани могут стоять на карте-полосе, а сама грань - вне носителя: её проекция не карта.
                    raise BeyondChartReach(face)
                ordinal = face.edge_cycle.index(physical_edge_id)
                cycle = chart_face_points(coordinates, frame, kernel_vertex_for_host, triangle_ids, face)
                twice_area = sum(
                    cycle[index][0] * cycle[(index + 1) % len(cycle)][1]
                    - cycle[index][1] * cycle[(index + 1) % len(cycle)][0]
                    for index in range(len(cycle))
                )
                if twice_area == 0:
                    continue
                face_start = cycle[ordinal]
                face_end = cycle[(ordinal + 1) % len(cycle)]
                face_direction = (
                    face_end[0] - face_start[0],
                    face_end[1] - face_start[1],
                )
                same_direction = (
                    metric.dot_g(tangent_vector, vector(face_direction)) > 0
                )
                coordinate_interior_is_left = (
                    twice_area > 0
                ) == same_direction
                interior_is_left = coordinate_interior_is_left == (
                    metric.owner_orientation_sign > 0
                )
                candidates.append(
                    metric.owner_normal_g(
                        tangent_vector,
                        owner_left=interior_is_left,
                        normalize=False,
                    )
                )
            if not candidates:
                raise EnvelopeHostAdapterError(
                    EnvelopeDebugHostOutcome.ENVELOPE_DEBUG_EXACT_ANGULAR_CERTIFICATE_UNAVAILABLE,
                    "BoundaryCorner support has no exact owner-face direction",
                    patch_domain_id=domain_id.value,
                )
            first = candidates[0]
            if any(
                metric.oriented_cross(first, item) != 0
                or metric.dot_g(first, item) <= 0
                for item in candidates[1:]
            ):
                raise EnvelopeHostAdapterError(
                    EnvelopeDebugHostOutcome.ENVELOPE_DEBUG_EXACT_ANGULAR_CERTIFICATE_UNAVAILABLE,
                    "BoundaryCorner owner-face directions disagree",
                    patch_domain_id=domain_id.value,
                )
            return expressions(first), tangent

        # Счётчики заводятся ДО обхода и всегда, даже нулями: «углов не
        # нашлось» и «стадия ничего не смотрела» иначе — одна пустота.
        stage_counters = dict.fromkeys(ANGULAR_STAGE_COUNTERS, 0)
        for (
            loop_index,
            corner_index,
            is_cut_vertex,
            anchor_vertex_id,
            prev_ref,
            next_ref,
        ) in angular_sites(
            patch,
            patch_id,
            normalized_refs_by_source,
            record_by_ref,
            domain_id,
        ):
            stage_counters[
                "ANGULAR_CUT_VERTICES_CONSIDERED"
                if is_cut_vertex
                else "ANGULAR_CORNERS_CONSIDERED"
            ] += 1
            if prev_ref is None:
                # Угол, выведенный хостом геометрически на петле из одной цепи: стыка цепей в нём нет (см. `angular_sites`).
                stage_counters["ANGULAR_CORNERS_OFF_JUNCTION"] += 1
                continue
            if anchor_vertex_id not in vertex_ids:
                raise EnvelopeHostAdapterError(
                    EnvelopeDebugHostOutcome.ENVELOPE_DEBUG_EXACT_ANGULAR_CERTIFICATE_UNAVAILABLE,
                    "BoundaryCorner source vertex is absent from PatchSurfaceIR",
                    patch_domain_id=domain_id.value,
                )
            if vertex_ids[anchor_vertex_id] in cut_vertices:
                # Вершина разреза кольца на карте раздвоена: её угол разрезан на два угла со стеной, и отношения нет.
                stage_counters["ANGULAR_CORNERS_AT_RING_CUT"] += 1
                continue
            try:
                incoming_normal, incoming_tangent = owner_support(
                    record_by_ref[prev_ref], anchor_vertex_id
                )
                outgoing_normal, outgoing_tangent = owner_support(
                    record_by_ref[next_ref], anchor_vertex_id
                )
            except BeyondChartReach:
                # Карта-полоса не знает вершин дальше досягаемости: угол, чьи опоры или грани вне носителя, фронт не видит.
                stage_counters["ANGULAR_CORNERS_BEYOND_CHART_REACH"] += 1
                continue
            incoming_vector = vector(incoming_normal)
            outgoing_vector = vector(outgoing_normal)
            incoming_tangent_vector = vector(incoming_tangent)
            outgoing_tangent_vector = vector(outgoing_tangent)
            turn_cross = metric.oriented_cross(
                incoming_tangent_vector,
                outgoing_tangent_vector,
            )
            if turn_cross == 0:
                if metric.dot_g(
                    incoming_tangent_vector,
                    outgoing_tangent_vector,
                ) > 0:
                    stage_counters["ANGULAR_COLLINEAR_SKIPPED"] += 1
                    if is_cut_vertex:
                        # Излом БЫЛ в 3D, но проекция чарта его вырождает:
                        # разрез лишний. Он законен — оба закона линейности
                        # выполняются — но объявлен, а не спрятан.
                        stage_counters["ANGULAR_CUT_CHART_COLLINEAR"] += 1
                    continue
                raise EnvelopeHostAdapterError(
                    EnvelopeDebugHostOutcome.ENVELOPE_DEBUG_EXACT_ANGULAR_CERTIFICATE_UNAVAILABLE,
                    "exact 2*pi corner must be Terminal/Junction, not Angular",
                    patch_domain_id=domain_id.value,
                )
            incoming_interior_side = metric.oriented_cross(
                incoming_tangent_vector,
                incoming_vector,
            )
            outgoing_interior_side = metric.oriented_cross(
                outgoing_tangent_vector,
                outgoing_vector,
            )
            if (
                incoming_interior_side == 0
                or outgoing_interior_side == 0
                or (
                    incoming_interior_side > 0
                ) != (
                    outgoing_interior_side > 0
                )
            ):
                raise EnvelopeHostAdapterError(
                    EnvelopeDebugHostOutcome.ENVELOPE_DEBUG_EXACT_ANGULAR_CERTIFICATE_UNAVAILABLE,
                    "BoundaryCorner exact owner-interior sides disagree",
                    patch_domain_id=domain_id.value,
                )
            is_reflex = (
                turn_cross * incoming_interior_side < 0
            )
            if not is_reflex:
                continue
            stage_counters["ANGULAR_REFLEX_CORNERS"] += 1
            if is_cut_vertex:
                stage_counters["ANGULAR_CUT_REFLEX_RELATIONS"] += 1
            cosine, sine = metric.angle_g(
                incoming_vector, outgoing_vector
            )
            orientation = (
                kernel.TurnOrientation.CCW_IN_OWNER_PATCH_ORIENTATION
                if turn_cross > 0
                else kernel.TurnOrientation.CW_IN_OWNER_PATCH_ORIENTATION
            )
            try:
                # δ выводится из φ уже после канонизации: иначе равенство
                # `δ == φ − 1` не доживает до записи (см. модуль меры).
                measure = certified_reflex_measure(
                    kernel, angle_measure, sine, cosine, orientation
                )
            except angle_measure.CertifiedAngleUnavailable as error:
                raise EnvelopeHostAdapterError(
                    EnvelopeDebugHostOutcome.ENVELOPE_DEBUG_EXACT_ANGULAR_CERTIFICATE_UNAVAILABLE,
                    str(error),
                    patch_domain_id=domain_id.value,
                ) from error
            owner_sector_id = sector_id_by_ref[prev_ref]
            if owner_sector_id in used_sector_ids:
                raise EnvelopeHostAdapterError(
                    EnvelopeDebugHostOutcome.ENVELOPE_DEBUG_MULTIPLE_ANGULAR_RELATIONS_PER_CHAIN_UNSUPPORTED,
                    "V0 cannot encode two Angular relations through one owner-sector record",
                    patch_domain_id=domain_id.value,
                )
            used_sector_ids.add(owner_sector_id)

            def exact_rational(value):
                value = sympy.factor(value)
                if value.is_Rational is not True:
                    raise EnvelopeHostAdapterError(
                        EnvelopeDebugHostOutcome.ENVELOPE_DEBUG_EXACT_ANGULAR_CERTIFICATE_UNAVAILABLE,
                        "affine support direction is not rational",
                        patch_domain_id=domain_id.value,
                    )
                return kernel.ExactRationalV1(
                    int(value.p), int(value.q)
                )

            def direction_payload(value):
                return kernel.CertifiedAffineSupportDirectionV2(
                    kernel.ExactVector2V1(
                        exact_rational(value[0]),
                        exact_rational(value[1]),
                    ),
                    frame.reference_metric_id,
                )

            incoming_use_id = use_id_by_ref[prev_ref]
            outgoing_use_id = use_id_by_ref[next_ref]
            angular_sectors.append(
                kernel.OrientedOwnerSectorV1(
                    owner_sector_id,
                    patch_ids[patch_id],
                    domain_id,
                    True,
                    (incoming_use_id, outgoing_use_id),
                    kernel.SourceSupportRefV1(
                        incoming_use_id,
                        launch_id_by_ref[prev_ref],
                        direction_payload(incoming_normal),
                    ),
                    kernel.SourceSupportRefV1(
                        outgoing_use_id,
                        launch_id_by_ref[next_ref],
                        direction_payload(outgoing_normal),
                    ),
                    orientation,
                    kernel.InteriorSelectionLaw.OWNER_PATCH_INTERIOR_BETWEEN_ORDERED_SUPPORTS,
                )
            )
            angle_id = kernel.AngleCertificateId(
                _typed_value(
                    "angle-certificate",
                    revision,
                    patch_id,
                    loop_index,
                    corner_index,
                )
            )
            angle_certificates.append(
                kernel.ReflexAngleCertificateV1(
                    angle_id,
                    owner_sector_id,
                    kernel.AngleMeasureLaw.ORIENTED_OWNER_SECTOR_ANGLE,
                    kernel.AngleMeasureSource.HOST_ANALYSIS_EXACT_OR_CERTIFIED,
                    kernel.StrictAngleRangeCertificate.STRICT_PI_LT_PHI_LT_2PI,
                    kernel.ReflexExcessLaw.DELTA_EQUALS_PHI_MINUS_PI,
                    measure,
                    False,
                )
            )
            corner_relations.append(
                kernel.CornerRelationV1(
                    kernel.CornerRelationId(
                        _typed_value(
                            "corner-relation",
                            revision,
                            patch_id,
                            loop_index,
                            corner_index,
                        )
                    ),
                    vertex_ids[anchor_vertex_id],
                    owner_sector_id,
                    angle_id,
                    False,
                )
            )
            stage_counters["ANGULAR_RELATIONS_BUILT"] += 1
        if profile is not None:
            for name, value in sorted(stage_counters.items()):
                profile.set_counter(name, value, domain_id.value)
    return (
        frozenset(angular_sectors),
        frozenset(angle_certificates),
        frozenset(corner_relations),
    )
