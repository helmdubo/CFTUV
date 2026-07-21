"""Metric contract source transform для decal backend."""

from dataclasses import dataclass, fields
from math import isfinite, sqrt

from .model import LocalDecalSettings, WorldDecalSettings


_TRANSFORM_REL_TOLERANCE = 1e-6


class DecalSourceTransformError(ValueError):
    """Source transform нельзя безопасно свести к local metric."""

    def __init__(self, code, message):
        self.code = str(code)
        super().__init__(f"{self.code}: {message}")


@dataclass(frozen=True)
class DecalSourceTransform:
    """Проверенный positive uniform scale source object."""

    uniform_scale: float
    determinant: float


@dataclass(frozen=True)
class MetricContext:
    """One-time world/local conversion proof for a source matrix."""

    source_matrix_fingerprint: tuple[str, ...]
    uniform_scale: float


@dataclass(frozen=True)
class LocalDecalRequest:
    """Typed local settings paired with the matrix proof that produced them."""

    settings: LocalDecalSettings
    metric_context: MetricContext


def _basis_columns(matrix):
    try:
        basis = matrix.to_3x3()
        columns = tuple(
            tuple(float(basis[row][column]) for row in range(3))
            for column in range(3)
        )
        determinant = float(basis.determinant())
    except (AttributeError, IndexError, TypeError, ValueError) as exc:
        raise DecalSourceTransformError(
            "INVALID_SOURCE_TRANSFORM",
            "matrix_world does not expose a finite 3x3 affine basis",
        ) from exc
    if not isfinite(determinant) or not all(
        isfinite(value) for column in columns for value in column
    ):
        raise DecalSourceTransformError(
            "INVALID_SOURCE_TRANSFORM",
            "matrix_world contains non-finite basis values",
        )
    return columns, determinant


def _dot(a, b):
    return sum(left * right for left, right in zip(a, b))


def validate_decal_source_transform(matrix_world):
    """Принимает rotation/translation + positive uniform scale без shear."""

    if matrix_world is None:
        raise DecalSourceTransformError(
            "MISSING_SOURCE_TRANSFORM",
            "source object has no matrix_world",
        )
    columns, determinant = _basis_columns(matrix_world)
    lengths_squared = tuple(_dot(column, column) for column in columns)
    scale_squared = sum(lengths_squared) / 3.0
    if scale_squared <= 0.0:
        raise DecalSourceTransformError(
            "DEGENERATE_SOURCE_TRANSFORM",
            "source transform has zero metric scale",
        )

    scale = sqrt(scale_squared)
    if determinant < 0.0:
        raise DecalSourceTransformError(
            "MIRRORED_SOURCE_TRANSFORM",
            "negative determinant/reflection is not supported for decals",
        )
    if determinant == 0.0:
        raise DecalSourceTransformError(
            "DEGENERATE_SOURCE_TRANSFORM",
            "source transform determinant is zero",
        )

    metric_tolerance = scale_squared * _TRANSFORM_REL_TOLERANCE
    diagonal_error = max(
        abs(length_squared - scale_squared)
        for length_squared in lengths_squared
    )
    off_diagonal_error = max(
        abs(_dot(columns[left], columns[right]))
        for left, right in ((0, 1), (0, 2), (1, 2))
    )
    if max(diagonal_error, off_diagonal_error) > metric_tolerance:
        raise DecalSourceTransformError(
            "NON_UNIFORM_OR_SHEARED_SOURCE_TRANSFORM",
            "decals require isotropic positive scale without shear",
        )
    return DecalSourceTransform(
        uniform_scale=scale,
        determinant=determinant,
    )


def _matrix_fingerprint(matrix_world):
    """Exact affine representation fingerprint; never used as geometry policy."""

    try:
        return ("4x4",) + tuple(
            float(matrix_world[row][column]).hex()
            for row in range(4)
            for column in range(4)
        )
    except (IndexError, TypeError, ValueError):
        # Headless/tests may expose only the metric 3x3 basis. Production
        # Blender matrices take the 4x4 branch and include translation.
        try:
            basis = matrix_world.to_3x3()
            return ("3x3",) + tuple(
                float(basis[row][column]).hex()
                for row in range(3)
                for column in range(3)
            )
        except (AttributeError, IndexError, TypeError, ValueError) as exc:
            raise DecalSourceTransformError(
                "INVALID_SOURCE_TRANSFORM",
                "matrix_world does not expose an affine matrix",
            ) from exc


def metric_context_for_source(source_obj):
    matrix_world = getattr(source_obj, "matrix_world", None)
    transform = validate_decal_source_transform(matrix_world)
    return MetricContext(
        source_matrix_fingerprint=_matrix_fingerprint(matrix_world),
        uniform_scale=transform.uniform_scale,
    )


def decal_settings_to_local(settings, uniform_scale):
    """Переводит world dimensions/texel density в source local metric."""

    scale = float(uniform_scale)
    if scale <= 0.0:
        raise ValueError("uniform_scale must be positive")
    if not isinstance(settings, WorldDecalSettings):
        raise TypeError("WorldDecalSettings required before metric conversion")
    values = {
        item.name: getattr(settings, item.name)
        for item in fields(WorldDecalSettings)
    }
    values.update(
        width_corner=settings.width_corner / scale,
        width_seam=settings.width_seam / scale,
        height_trim=settings.height_trim / scale,
        offset=settings.offset / scale,
        uv_length_scale=settings.uv_length_scale * scale,
    )
    return LocalDecalSettings(**values)


def local_decal_request_for_source(settings, source_obj):
    """Perform the only world->local conversion and retain its proof."""

    context = metric_context_for_source(source_obj)
    return local_decal_request_for_metric_context(settings, context)


def local_decal_request_for_metric_context(settings, context):
    """Конвертирует новый world request через session-owned metric proof."""

    if not isinstance(context, MetricContext):
        raise TypeError("MetricContext required for decal request conversion")
    return LocalDecalRequest(
        settings=decal_settings_to_local(settings, context.uniform_scale),
        metric_context=context,
    )


def local_decal_settings_for_source(settings, source_obj):
    """Проверяет source transform и возвращает backend-local settings."""

    return local_decal_request_for_source(settings, source_obj).settings
