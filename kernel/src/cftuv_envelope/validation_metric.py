"""Проверки рациональной аффинной планарной метрики: оба закона допуска.

Отдельный модуль, а не ветка в `validation.py`: тот стоит РОВНО на своём
потолке в `tests/test_architecture.py`, и потолок не поднимается — поднятие
числа в таблице бюджетов есть заявление «я осознанно наращиваю долг», а не
способ поместить закон.

Что здесь живёт. Метрика несёт сертификат допуска плоскости, и сертификатов
ДВА, потому что законов допуска два. Прежде проверяющий знал ровно один
(`EXACT_SOURCE_PLANE_V1`) и отвергал `NearPlanarProjectionCertificateV1`,
который пишет ТОТ ЖЕ строитель, — полевой отказ на наклонном плоском домене
владельца был именно этим. Ветка near-planar по проводу здесь и появляется.

Правило разбора: тип сертификата определяется ТОЧНЫМ `type() is` по
вайр-идентичности, а поля проверяются явно. `__post_init__` записи ничего не
доказывает проверяющему: запись могла прийти из кодека, из подделки, из другой
версии контракта — проверяющий обязан спросить провод, а не автора.
"""

from __future__ import annotations

from fractions import Fraction
from hashlib import sha256
import math

from .contracts.metric import (
    AffineChartOrientationV1,
    AffineFrameSelectionLawV1,
    AffineReconstructionLawV1,
    ExactMatrix2V1,
    ExactPoint3V1,
    ExactRationalV1,
    ExactSourcePlaneCertificateV1,
    ExactVector3V1,
    EmbeddingCertifiedRationalAffinePlanarMetricV1,
    PRODUCT_SKIRT_ABSOLUTE_BUDGET,
    NearPlanarProjectionCertificateV1,
    NearPlanarResidualBudgetLawV1,
    PlanarityAdmissionLawV1,
    RationalAffinePlanarMetricV2,
)
from .contracts.analysis import SourceVertexV1
from .contracts.surface import SourceFaceV1
from .ids import LineageId, PatchDomainId, SourceRevision
from ._embedding import (
    build_projection_embedding_certificate,
    patch_plane_normal,
    projection_violations,
    source_snap_violations,
)
from ._metric_wire import (
    classify_metric_normal_wire,
    metric_normal_wire_rejection_message,
)
from .numeric import LocalPoint3V1
from .validation_issues import ValidationCode, ValidationIssue, add_issue


def fraction_of(value: ExactRationalV1) -> Fraction:
    return Fraction(value.numerator, value.denominator)


def fraction_point3(
    value: ExactPoint3V1 | ExactVector3V1,
) -> tuple[Fraction, Fraction, Fraction]:
    return fraction_of(value.x), fraction_of(value.y), fraction_of(value.z)


def fraction_matrix2(
    value: ExactMatrix2V1,
) -> tuple[tuple[Fraction, Fraction], tuple[Fraction, Fraction]]:
    return (
        (fraction_of(value.m00), fraction_of(value.m01)),
        (fraction_of(value.m10), fraction_of(value.m11)),
    )


def fraction_dot3(left, right) -> Fraction:
    return sum(
        (a * b for a, b in zip(left, right, strict=True)),
        Fraction(0),
    )


def fraction_cross3(left, right):
    return (
        left[1] * right[2] - left[2] * right[1],
        left[2] * right[0] - left[0] * right[2],
        left[0] * right[1] - left[1] * right[0],
    )


def _stable_id(kind: str, *parts: object) -> str:
    payload = "\x1f".join((kind, *(str(item) for item in parts)))
    return f"{kind}:{sha256(payload.encode('utf-8')).hexdigest()[:24]}"


def position_under_grid_law(
    position: LocalPoint3V1, certificate
) -> tuple[Fraction, Fraction, Fraction]:
    """Позиция источника такой, какой её делает ОБЪЯВЛЕННЫЙ закон решётки.

    Проверка «карта восстанавливает вершину точно» обязана сверяться с тем
    входом, который метрика реально получила. До решётки это была сама
    координата binary64; у всякого закона, который двигает источник, — её
    узел. Сверяться с непривязанной координатой значило бы требовать от
    привязанной метрики невозможного, а снять проверку — потерять
    единственное место, где карта сверяется с источником.

    Условие — `snaps_source`, а не имя закона: привязку конструкций эта
    проверка не видит вовсе (она про вершины источника), поэтому
    `SOURCE_ONLY_GRID_SNAP_V1` обязан идти здесь тем же путём, что и
    `INTEGER_GRID_SNAP_V1`.

    Узел перевычисляется здесь заново, из объявленного в сертификате масштаба,
    а не берётся у построителя: так проверяется и то, что метрика привязана к
    ТОМУ масштабу, который она объявляет.
    """

    from .robust.grid import GridSpecV1, snap_value

    exact = tuple(
        Fraction(*float(value).as_integer_ratio())
        for value in (position.x, position.y, position.z)
    )
    if (
        not certificate.snapping_law.snaps_source
        or certificate.source_scale is None
    ):
        return exact
    grid = GridSpecV1(scale=certificate.source_scale)
    return tuple(
        Fraction(snap_value(item, grid), certificate.source_scale)
        for item in exact
    )


def expected_source_position(
    position: LocalPoint3V1,
    vertex_id,
    metric: RationalAffinePlanarMetricV2,
) -> tuple[Fraction, Fraction, Fraction]:
    """Куда объявленные законы метрики обязаны были положить эту вершину.

    Законов, двигающих источник, ДВА, и они идут в объявленном порядке:
    сначала решётка (узел), затем — только для вершин, названных
    спроецированными, — проекция этого узла на сертифицированную плоскость.
    Прежде проверялся один первый, и метрика полевого наклонного домена
    расходилась с ожиданием ровно на величину второго: «exact affine
    reconstruction disagrees with the source position the declared grid law
    produces» на двух вершинах из шести.

    Плоскость берётся из ЗАПИСЕЙ, новых полей для этого не нужно. Опора
    проекции — позиция первой вершины кадра: от себя она не отклоняется, в
    перечень отклонившихся не попадает, не проецируется и потому совпадает с
    `exact_origin` метрики. Формула `p − ((p−o)·n/(n·n))·n` инвариантна к
    ненулевому множителю нормали (в том числе к знаку), поэтому записанная
    каноническая примитивная нормаль задаёт ту же плоскость, что и любой другой
    её вывод, и пересчитывать Ньюэлла здесь не нужно и незачем.

    Вершина, которую «забыли» назвать спроецированной, судится узлом — и
    расходится с картой. Это не отдельная проверка, а следствие: перечень
    спроецированных вершин есть заявление о том, что с ними делали, и ложное
    заявление обязано ломать сверку.
    """

    node = position_under_grid_law(position, metric.grid_certificate)
    certificate = metric.planarity_certificate
    if type(certificate) is not NearPlanarProjectionCertificateV1:
        return node
    if vertex_id not in certificate.projected_source_vertex_ids:
        return node
    normal = fraction_point3(certificate.exact_plane_normal)
    normal_squared = fraction_dot3(normal, normal)
    if not normal_squared:
        # Вырожденная нормаль — уже названный отказ метрики; здесь остаётся
        # только не делить на ноль.
        return node
    origin = fraction_point3(metric.exact_origin)
    scale = fraction_dot3(
        tuple(node[axis] - origin[axis] for axis in range(3)), normal
    ) / normal_squared
    return tuple(node[axis] - scale * normal[axis] for axis in range(3))


def _recomputed_budget(certificate, grid_certificate) -> Fraction | None:
    """Бюджет, пересчитанный по ОБЪЯВЛЕННОМУ закону из записанных полей.

    Проверяется не «поле непусто», а то, что записанное число воспроизводится
    законом, который сертификат называет. Пока пересчёта не было, поле
    `residual_budget_law` было украшением: строитель писал один закон, считал
    по другому, и никто не мог заметить.

    `None` — «объявленный закон не даёт числа из того, что записано», то есть
    сам отказ.
    """

    if certificate.residual_budget_law is (
        NearPlanarResidualBudgetLawV1.PRODUCT_SKIRT_ABSOLUTE_V1
    ):
        # Число принадлежит ЗАКОНУ, а не записи: сертификат его повторяет, а
        # владеет им ядро. Сверка с константой — то же самое требование, что и
        # к двум другим законам: объявил закон — воспроизведи его величину.
        return PRODUCT_SKIRT_ABSOLUTE_BUDGET
    if certificate.residual_budget_law is (
        NearPlanarResidualBudgetLawV1.GRID_STEP_CELL_V1
    ):
        step = grid_certificate.window_step
        return None if step is None else fraction_of(step)
    if certificate.residual_budget_law is (
        NearPlanarResidualBudgetLawV1.RELATIVE_EXTENT_OR_ULP_V1
    ):
        return max(
            fraction_of(certificate.relative_extent_factor)
            * max(
                fraction_of(certificate.planar_extent),
                fraction_of(certificate.minimum_extent),
            ),
            certificate.coordinate_ulp_multiplier
            * fraction_of(certificate.max_coordinate_ulp),
        )
    return None


def _check_near_planar_certificate(
    issues: list[ValidationIssue],
    path: tuple[str, ...],
    metric: RationalAffinePlanarMetricV2,
) -> None:
    """Ветка near-planar: всё по проводу, ничего на веру от `__post_init__`."""

    certificate = metric.planarity_certificate
    if (
        certificate.patch_domain_id != metric.patch_domain_id
        or certificate.source_revision != metric.source_revision
        or certificate.admission_law
        is not PlanarityAdmissionLawV1.NEAR_PLANAR_PROJECTION_V1
        or certificate.reconstruction_law
        is not AffineReconstructionLawV1.O_PLUS_U_A_PLUS_V_B_V1
        or certificate.exact
    ):
        add_issue(
            issues,
            ValidationCode.SURFACE_METRIC,
            path,
            "near-planar certificate must be a non-exact same-domain "
            "same-revision projection of the declared reconstruction law",
        )
    projected = certificate.projected_source_vertex_ids
    if not projected or not projected <= certificate.source_vertex_ids:
        add_issue(
            issues,
            ValidationCode.SURFACE_METRIC,
            path + ("projected_source_vertex_ids",),
            "projected vertices must be a non-empty subset of the "
            "certificate source vertices",
        )
    # Власть допуска ПРОДУКТОВАЯ и единая для обоих законов решётки — решение
    # владельца от 2026-08-01. Прежде ожидание выводилось из `snaps_source`
    # (ячейка либо представление); те законы остались членами перечисления и
    # остались пересчитываемыми ниже, но допуском больше не распоряжаются.
    expected_law = NearPlanarResidualBudgetLawV1.PRODUCT_SKIRT_ABSOLUTE_V1
    if certificate.residual_budget_law is not expected_law:
        add_issue(
            issues,
            ValidationCode.SURFACE_METRIC,
            path + ("residual_budget_law",),
            "near-planar admission is governed by the product skirt budget; "
            "the declared residual-budget law is not it",
        )
    budget = fraction_of(certificate.residual_budget)
    recomputed = _recomputed_budget(certificate, metric.grid_certificate)
    if recomputed is None or recomputed != budget:
        add_issue(
            issues,
            ValidationCode.SURFACE_METRIC,
            path + ("residual_budget",),
            "recorded residual budget is not what its declared law produces",
        )
    if fraction_of(certificate.max_residual_squared) > budget * budget:
        add_issue(
            issues,
            ValidationCode.SURFACE_METRIC,
            path + ("max_residual_squared",),
            "recorded residual exceeds the recorded budget",
        )


def validate_rational_affine_planar_metric(
    metric: RationalAffinePlanarMetricV2,
) -> tuple[ValidationIssue, ...]:
    issues: list[ValidationIssue] = []
    path = ("RationalAffinePlanarMetricV2",)
    if (
        metric.frame_selection_law
        is not AffineFrameSelectionLawV1.CANONICAL_SOURCE_VERTEX_BASIS_V1
    ):
        add_issue(
            issues,
            ValidationCode.SURFACE_METRIC,
            path + ("frame_selection_law",),
            "unsupported deterministic affine-frame law",
        )
    certificate = metric.planarity_certificate
    certificate_path = path + ("planarity_certificate",)
    # Разбор по ВАЙР-идентичности. `isinstance` пропустил бы наследника, а
    # доверие `__post_init__` — запись, собранную не конструктором.
    if type(certificate) is ExactSourcePlaneCertificateV1:
        if (
            certificate.patch_domain_id != metric.patch_domain_id
            or certificate.admission_law
            is not PlanarityAdmissionLawV1.EXACT_SOURCE_PLANE_V1
            or certificate.reconstruction_law
            is not AffineReconstructionLawV1.O_PLUS_U_A_PLUS_V_B_V1
            or not certificate.exact
        ):
            add_issue(
                issues,
                ValidationCode.SURFACE_METRIC,
                certificate_path,
                "metric requires an exact same-domain source-plane certificate",
            )
    elif type(certificate) is NearPlanarProjectionCertificateV1:
        _check_near_planar_certificate(issues, certificate_path, metric)
    else:
        # Ниже сертификат читается по полям, которых у неизвестного типа может
        # не быть вовсе. Разбор кончается здесь названным отказом, а не
        # `AttributeError` изнутри проверяющего.
        add_issue(
            issues,
            ValidationCode.SURFACE_METRIC,
            certificate_path,
            "unknown planarity-admission certificate on the wire",
        )
        _check_gram(
            issues,
            path,
            metric,
            fraction_point3(metric.exact_basis_a),
            fraction_point3(metric.exact_basis_b),
        )
        return tuple(issues)
    basis_a = fraction_point3(metric.exact_basis_a)
    basis_b = fraction_point3(metric.exact_basis_b)
    normal = fraction_cross3(basis_a, basis_b)
    if not any(normal):
        add_issue(
            issues,
            ValidationCode.SURFACE_METRIC,
            path + ("exact_basis_a", "exact_basis_b"),
            "affine basis vectors must be linearly independent",
        )
    declared = fraction_point3(certificate.exact_plane_normal)
    normal_wire = classify_metric_normal_wire(declared, normal)
    if not normal_wire.accepted:
        add_issue(
            issues,
            ValidationCode.SURFACE_METRIC,
            certificate_path + ("exact_plane_normal",),
            metric_normal_wire_rejection_message(normal_wire),
        )
    _check_gram(issues, path, metric, basis_a, basis_b)
    coordinate_ids = [
        item.source_vertex_id
        for item in metric.exact_source_vertex_coordinates
    ]
    if len(coordinate_ids) != len(set(coordinate_ids)):
        add_issue(
            issues,
            ValidationCode.DUPLICATE_ID,
            path + ("exact_source_vertex_coordinates",),
            "duplicate source vertex coordinate",
        )
    if set(coordinate_ids) != set(certificate.source_vertex_ids):
        add_issue(
            issues,
            ValidationCode.SURFACE_METRIC,
            path + ("exact_source_vertex_coordinates",),
            "coordinate and planarity-certificate vertex sets differ",
        )
    return tuple(issues)


def _source_embedding_inputs(*, source_vertices, source_faces, owner_patch_id):
    faces = tuple(
        sorted(
            (item for item in source_faces if item.patch_id == owner_patch_id),
            key=lambda item: item.face_id.value,
        )
    )
    required_ids = tuple(
        sorted(
            {vertex_id for face in faces for vertex_id in face.vertex_cycle},
            key=lambda item: item.value,
        )
    )
    vertex_by_id = {item.vertex_id: item for item in source_vertices}
    positions = {
        vertex_id: tuple(
            Fraction(*float(value).as_integer_ratio())
            for value in (
                vertex_by_id[vertex_id].position.x,
                vertex_by_id[vertex_id].position.y,
                vertex_by_id[vertex_id].position.z,
            )
        )
        for vertex_id in required_ids
    }
    return faces, required_ids, positions


def _projection_coordinates(metric):
    coordinates = {
        item.source_vertex_id: (
            fraction_of(item.domain_coordinate.x),
            fraction_of(item.domain_coordinate.y),
        )
        for item in metric.exact_source_vertex_coordinates
    }
    return coordinates


def _sub3(left, right):
    return tuple(a - b for a, b in zip(left, right, strict=True))


def _projected_source_positions(positions, required_ids, normal):
    anchor = positions[required_ids[0]]
    normal_squared = fraction_dot3(normal, normal)
    projected = dict(positions)
    off_plane = []
    for vertex_id in required_ids:
        deviation = fraction_dot3(_sub3(positions[vertex_id], anchor), normal)
        if deviation == 0:
            continue
        off_plane.append(vertex_id)
        scale = deviation / normal_squared
        projected[vertex_id] = tuple(
            positions[vertex_id][axis] - scale * normal[axis]
            for axis in range(3)
        )
    return projected, tuple(off_plane)


def _source_near_planar_facts(positions, required_ids, normal):
    """Recompute every numeric near-planar claim from snapped source facts."""

    anchor = positions[required_ids[0]]
    normal_squared = fraction_dot3(normal, normal)
    residuals = []
    for vertex_id in required_ids:
        deviation = fraction_dot3(_sub3(positions[vertex_id], anchor), normal)
        if deviation:
            residuals.append(deviation * deviation / normal_squared)
    planar_extent = max(
        max(point[axis] for point in positions.values())
        - min(point[axis] for point in positions.values())
        for axis in range(3)
    )
    max_ulp = Fraction(0)
    for point in positions.values():
        for value in point:
            ulp = math.ulp(float(value)) if value else math.ulp(1.0)
            max_ulp = max(max_ulp, Fraction(*ulp.as_integer_ratio()))
    return planar_extent, max_ulp, max(residuals, default=Fraction(0))


def _check_near_planar_source_facts(issues, path, certificate, source_facts):
    """Compare recorded near-planar evidence with an independent recomputation."""

    planar_extent, max_ulp, max_residual_squared = source_facts
    claims = (
        (
            "relative_extent_factor",
            fraction_of(certificate.relative_extent_factor),
            Fraction(1, 10**7),
        ),
        (
            "minimum_extent",
            fraction_of(certificate.minimum_extent),
            Fraction(1, 10**3),
        ),
        (
            "coordinate_ulp_multiplier",
            certificate.coordinate_ulp_multiplier,
            64,
        ),
        (
            "max_coordinate_ulp",
            fraction_of(certificate.max_coordinate_ulp),
            max_ulp,
        ),
        (
            "planar_extent",
            fraction_of(certificate.planar_extent),
            planar_extent,
        ),
        (
            "residual_budget",
            fraction_of(certificate.residual_budget),
            PRODUCT_SKIRT_ABSOLUTE_BUDGET,
        ),
        (
            "max_residual_squared",
            fraction_of(certificate.max_residual_squared),
            max_residual_squared,
        ),
    )
    for field, declared, expected in claims:
        if declared != expected:
            add_issue(
                issues,
                ValidationCode.SURFACE_METRIC,
                path + (field,),
                f"{field} differs from exact source recomputation",
            )
    if certificate.residual_budget_law is not (
        NearPlanarResidualBudgetLawV1.PRODUCT_SKIRT_ABSOLUTE_V1
    ):
        add_issue(
            issues,
            ValidationCode.SURFACE_METRIC,
            path + ("residual_budget_law",),
            "residual budget law differs from the normative product law",
        )


def _canonical_frame(positions, required_ids):
    origin = positions[required_ids[0]]
    candidates = tuple(
        (vertex_id, _sub3(positions[vertex_id], origin))
        for vertex_id in required_ids[1:]
    )
    first = next(vector for _, vector in candidates if any(vector))
    second = next(
        vector for _, vector in candidates if any(fraction_cross3(first, vector))
    )
    return origin, first, second


def _expected_domain_coordinates(metric, projected):
    origin = fraction_point3(metric.exact_origin)
    basis_a = fraction_point3(metric.exact_basis_a)
    basis_b = fraction_point3(metric.exact_basis_b)
    inverse = fraction_matrix2(metric.exact_inverse_gram_matrix)
    result = {}
    for vertex_id, position in projected.items():
        relative = _sub3(position, origin)
        rhs_a = fraction_dot3(basis_a, relative)
        rhs_b = fraction_dot3(basis_b, relative)
        result[vertex_id] = (
            inverse[0][0] * rhs_a + inverse[0][1] * rhs_b,
            inverse[1][0] * rhs_a + inverse[1][1] * rhs_b,
        )
    return result


def _coordinate_orientation(coordinates, faces):
    for face in faces:
        cycle = tuple(coordinates[item] for item in face.vertex_cycle)
        area = sum(
            (
                cycle[index][0] * cycle[(index + 1) % len(cycle)][1]
                - cycle[index][1] * cycle[(index + 1) % len(cycle)][0]
                for index in range(len(cycle))
            ),
            Fraction(0),
        )
        if area:
            return (area > 0) - (area < 0)
    return 0


def _embedding_issue(issues, path, outcome):
    add_issue(
        issues,
        ValidationCode.SURFACE_METRIC,
        path,
        f"embedding certificate failed exact recomputation: {outcome.value}",
    )


def _recompute_embedding_inputs(issues, path, record, faces, required_ids, positions):
    from .source_grid import resolve_source_grid

    metric = record.metric
    grid = resolve_source_grid(
        positions=dict(positions),
        faces=faces,
        snapping_law=metric.grid_certificate.snapping_law,
        enforce_embedding=False,
    )
    if grid.certificate != metric.grid_certificate:
        add_issue(
            issues,
            ValidationCode.SURFACE_METRIC,
            path + ("metric", "grid_certificate"),
            "grid certificate differs from exact source recomputation",
        )
    expected_snap = grid.source_snap_embedding_certificate
    if expected_snap != record.source_snap_embedding_certificate:
        add_issue(
            issues,
            ValidationCode.SURFACE_METRIC,
            path + ("source_snap_embedding_certificate",),
            "source-snap embedding certificate differs from exact recomputation",
        )
    for outcome in source_snap_violations(expected_snap):
        _embedding_issue(
            issues, path + ("source_snap_embedding_certificate",), outcome
        )
    normal = patch_plane_normal(grid.positions, faces)
    declared_normal = fraction_point3(metric.planarity_certificate.exact_plane_normal)
    normal_wire = classify_metric_normal_wire(declared_normal, normal)
    if not normal_wire.accepted:
        add_issue(
            issues,
            ValidationCode.SURFACE_METRIC,
            path + ("metric", "planarity_certificate", "exact_plane_normal"),
            "declared plane normal differs from exact source recomputation: "
            + metric_normal_wire_rejection_message(normal_wire),
        )
    projected, off_plane = _projected_source_positions(
        grid.positions, required_ids, normal
    )
    certificate = metric.planarity_certificate
    if type(certificate) is NearPlanarProjectionCertificateV1:
        _check_near_planar_source_facts(
            issues,
            path + ("metric", "planarity_certificate"),
            certificate,
            _source_near_planar_facts(grid.positions, required_ids, normal),
        )
    declared_off_plane = (
        certificate.projected_source_vertex_ids
        if type(certificate) is NearPlanarProjectionCertificateV1
        else frozenset()
    )
    if set(off_plane) != set(declared_off_plane):
        add_issue(
            issues,
            ValidationCode.SURFACE_METRIC,
            path + ("metric", "planarity_certificate", "projected_source_vertex_ids"),
            "projected source IDs differ from exact source recomputation",
        )
    origin, basis_a, basis_b = _canonical_frame(projected, required_ids)
    declared_frame = (
        fraction_point3(metric.exact_origin),
        fraction_point3(metric.exact_basis_a),
        fraction_point3(metric.exact_basis_b),
    )
    if declared_frame != (origin, basis_a, basis_b):
        add_issue(
            issues,
            ValidationCode.SURFACE_METRIC,
            path + ("metric", "exact_origin", "exact_basis_a", "exact_basis_b"),
            "declared affine frame differs from canonical projected source frame",
        )
    expected_coordinates = _expected_domain_coordinates(metric, projected)
    if _projection_coordinates(metric) != expected_coordinates:
        add_issue(
            issues,
            ValidationCode.SURFACE_METRIC,
            path + ("metric", "exact_source_vertex_coordinates"),
            "declared domain coordinates differ from exact source reconstruction",
        )
    sign = _coordinate_orientation(expected_coordinates, faces)
    declared_sign = (
        1
        if metric.chart_orientation
        is AffineChartOrientationV1.COORDINATE_CCW_MATCHES_OWNER_PATCH
        else -1
    )
    if sign != declared_sign:
        add_issue(
            issues,
            ValidationCode.SURFACE_METRIC,
            path + ("metric", "chart_orientation"),
            "chart orientation differs from exact source reconstruction",
        )
    return grid, normal, off_plane, expected_coordinates, sign


def _check_embedding_metric_identity_authority(
    issues,
    path,
    metric,
    required_ids,
    *,
    expected_source_revision,
    expected_patch_domain_id,
    expected_source_lineage,
):
    """Bind self-consistent IDs to caller-owned source identity."""

    authority_claims = (
        ("source_revision", metric.source_revision, expected_source_revision),
        ("patch_domain_id", metric.patch_domain_id, expected_patch_domain_id),
        ("source_lineage", metric.source_lineage, expected_source_lineage),
    )
    for field, declared, expected in authority_claims:
        if declared != expected:
            add_issue(
                issues,
                ValidationCode.SURFACE_METRIC,
                path + ("metric", field),
                f"{field} differs from caller-owned source identity",
            )
    expected_metric_id = _stable_id(
        "reference-metric",
        metric.source_revision.value,
        metric.patch_domain_id.value,
        required_ids[0].value,
        *(item.value for item in required_ids),
    )
    if metric.reference_metric_id.value != expected_metric_id:
        add_issue(
            issues,
            ValidationCode.SURFACE_METRIC,
            path + ("metric", "reference_metric_id"),
            "reference metric ID differs from deterministic source recomputation",
        )
    certificate = metric.planarity_certificate
    kind = (
        "exact-source-plane"
        if type(certificate) is ExactSourcePlaneCertificateV1
        else "near-planar-projection"
    )
    expected_certificate_id = _stable_id(
        kind,
        metric.source_revision.value,
        metric.patch_domain_id.value,
    )
    if certificate.certificate_id.value != expected_certificate_id:
        add_issue(
            issues,
            ValidationCode.SURFACE_METRIC,
            path + ("metric", "planarity_certificate", "certificate_id"),
            "planarity certificate ID differs from deterministic recomputation",
        )


def validate_embedding_certified_rational_affine_planar_metric(
    record: EmbeddingCertifiedRationalAffinePlanarMetricV1,
    *,
    source_vertices: tuple[SourceVertexV1, ...],
    source_faces: tuple[SourceFaceV1, ...],
    owner_patch_id,
    expected_source_revision: SourceRevision,
    expected_patch_domain_id: PatchDomainId,
    expected_source_lineage: frozenset[LineageId],
) -> tuple[ValidationIssue, ...]:
    """Recompute evidence and bind it to caller-owned source identity.

    The three ``expected_*`` values are authority inputs, not conveniences:
    an integration must obtain them from its trusted source envelope.  Passing
    the claims from ``record.metric`` back as expectations is not validation.
    They intentionally have no defaults so a future caller cannot omit this
    binding while believing it used the complete validator.
    """

    issues = list(validate_rational_affine_planar_metric(record.metric))
    path = ("EmbeddingCertifiedRationalAffinePlanarMetricV1",)
    faces, required_ids, positions = _source_embedding_inputs(
        source_vertices=source_vertices,
        source_faces=source_faces,
        owner_patch_id=owner_patch_id,
    )
    _check_embedding_metric_identity_authority(
        issues,
        path,
        record.metric,
        required_ids,
        expected_source_revision=expected_source_revision,
        expected_patch_domain_id=expected_patch_domain_id,
        expected_source_lineage=expected_source_lineage,
    )
    grid, normal, off_plane, expected_coordinates, sign = (
        _recompute_embedding_inputs(
            issues, path, record, faces, required_ids, positions
        )
    )
    projection = record.near_planar_projection_embedding_certificate
    if bool(off_plane) != (projection is not None):
        add_issue(
            issues,
            ValidationCode.SURFACE_METRIC,
            path + ("near_planar_projection_embedding_certificate",),
            "projection certificate presence differs from exact source planarity",
        )
    if projection is None or not off_plane:
        return tuple(issues)
    expected_projection = build_projection_embedding_certificate(
        before=grid.positions,
        projected=expected_coordinates,
        faces=faces,
        normal=normal,
        expected_orientation_sign=sign,
    )
    if expected_projection != projection:
        add_issue(
            issues,
            ValidationCode.SURFACE_METRIC,
            path + ("near_planar_projection_embedding_certificate",),
            "projection embedding certificate differs from exact recomputation",
        )
    for outcome in projection_violations(expected_projection):
        _embedding_issue(
            issues,
            path + ("near_planar_projection_embedding_certificate",),
            outcome,
        )
    return tuple(issues)


def _check_gram(
    issues: list[ValidationIssue],
    path: tuple[str, ...],
    metric: RationalAffinePlanarMetricV2,
    basis_a,
    basis_b,
) -> None:
    gram = fraction_matrix2(metric.exact_gram_matrix)
    expected_gram = (
        (
            fraction_dot3(basis_a, basis_a),
            fraction_dot3(basis_a, basis_b),
        ),
        (
            fraction_dot3(basis_b, basis_a),
            fraction_dot3(basis_b, basis_b),
        ),
    )
    if gram != expected_gram:
        add_issue(
            issues,
            ValidationCode.SURFACE_METRIC,
            path + ("exact_gram_matrix",),
            "Gram matrix does not equal the exact A/B dot products",
        )
    determinant = gram[0][0] * gram[1][1] - gram[0][1] * gram[1][0]
    if determinant <= 0:
        add_issue(
            issues,
            ValidationCode.SURFACE_METRIC,
            path + ("exact_gram_matrix",),
            "Gram matrix must be positive definite",
        )
        return
    inverse = fraction_matrix2(metric.exact_inverse_gram_matrix)
    expected_inverse = (
        (
            gram[1][1] / determinant,
            -gram[0][1] / determinant,
        ),
        (
            -gram[1][0] / determinant,
            gram[0][0] / determinant,
        ),
    )
    if inverse != expected_inverse:
        add_issue(
            issues,
            ValidationCode.SURFACE_METRIC,
            path + ("exact_inverse_gram_matrix",),
            "inverse Gram matrix is not exact",
        )
