"""Metric contract source transform для decal backend."""

from dataclasses import dataclass, replace
from math import isfinite, sqrt


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


def decal_settings_to_local(settings, uniform_scale):
    """Переводит world dimensions/texel density в source local metric."""

    scale = float(uniform_scale)
    if scale <= 0.0:
        raise ValueError("uniform_scale must be positive")
    return replace(
        settings,
        width_corner=settings.width_corner / scale,
        width_seam=settings.width_seam / scale,
        height_trim=settings.height_trim / scale,
        offset=settings.offset / scale,
        uv_length_scale=settings.uv_length_scale * scale,
    )


def local_decal_settings_for_source(settings, source_obj):
    """Проверяет source transform и возвращает backend-local settings."""

    transform = validate_decal_source_transform(
        getattr(source_obj, "matrix_world", None)
    )
    return decal_settings_to_local(settings, transform.uniform_scale)
