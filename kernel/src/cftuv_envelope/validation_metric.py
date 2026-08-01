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

from .contracts.metric import (
    AffineFrameSelectionLawV1,
    AffineReconstructionLawV1,
    ExactMatrix2V1,
    ExactPoint3V1,
    ExactRationalV1,
    ExactSourcePlaneCertificateV1,
    ExactVector3V1,
    PRODUCT_SKIRT_ABSOLUTE_BUDGET,
    NearPlanarProjectionCertificateV1,
    NearPlanarResidualBudgetLawV1,
    PlanarityAdmissionLawV1,
    RationalAffinePlanarMetricV2,
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
    # Прежде стояло равенство `A × B`: проверялось не свойство плоскости, а
    # конкретный вывод. Каноничность записи не требуется намеренно — это
    # свойство того, что ПИШЕТ построитель. Проверка общая для обоих законов
    # допуска: она о плоскости, а не о том, чем её допустили.
    if any(normal) and any(declared) and any(fraction_cross3(declared, normal)):
        add_issue(
            issues,
            ValidationCode.SURFACE_METRIC,
            certificate_path + ("exact_plane_normal",),
            "plane normal must span the same plane as the A/B basis",
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
