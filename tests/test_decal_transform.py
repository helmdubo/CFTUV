from __future__ import annotations

from math import pi
from types import SimpleNamespace

import pytest

from cftuv import decals as decals_module
from cftuv.decal_transform import (
    DecalSourceTransformError,
    decal_settings_to_local,
    local_decal_settings_for_source,
    validate_decal_source_transform,
)
from cftuv.model import (
    DecalSettings,
    LocalDecalSettings,
    PatchGraph,
    WorldDecalSettings,
)


class _Matrix3:
    def __init__(self, rows):
        self.rows = tuple(tuple(float(value) for value in row) for row in rows)

    def __getitem__(self, index):
        return self.rows[index]

    def to_3x3(self):
        return self

    def determinant(self):
        a, b, c = self.rows
        return (
            a[0] * (b[1] * c[2] - b[2] * c[1])
            - a[1] * (b[0] * c[2] - b[2] * c[0])
            + a[2] * (b[0] * c[1] - b[1] * c[0])
        )


def _diagonal(x, y, z):
    return _Matrix3(((x, 0.0, 0.0), (0.0, y, 0.0), (0.0, 0.0, z)))


def test_rotation_and_positive_uniform_scale_are_metric_preserving():
    # 90° Z rotation + scale 2; translation не входит в 3x3 basis.
    matrix = _Matrix3(((0.0, -2.0, 0.0), (2.0, 0.0, 0.0), (0.0, 0.0, 2.0)))

    transform = validate_decal_source_transform(matrix)

    assert transform.uniform_scale == pytest.approx(2.0)
    assert transform.determinant == pytest.approx(8.0)

    tiny = validate_decal_source_transform(_diagonal(1e-5, 1e-5, 1e-5))
    assert tiny.uniform_scale == pytest.approx(1e-5)


def test_world_decal_settings_convert_to_local_metric_at_scale_two():
    settings = DecalSettings(
        width_corner=0.20,
        width_seam=0.15,
        height_trim=0.25,
        offset=0.02,
        uv_length_scale=0.25,
        corner_acute_split_angle=pi / 3.0,
        corner_apex_limit=8.0,
    )

    local = decal_settings_to_local(settings, 2.0)

    assert local.width_corner == pytest.approx(0.10)
    assert local.width_seam == pytest.approx(0.075)
    assert local.height_trim == pytest.approx(0.125)
    assert local.offset == pytest.approx(0.01)
    assert local.uv_length_scale == pytest.approx(0.50)
    assert local.corner_acute_split_angle == settings.corner_acute_split_angle
    assert local.corner_apex_limit == settings.corner_apex_limit
    assert isinstance(settings, WorldDecalSettings)
    assert isinstance(local, LocalDecalSettings)
    assert not isinstance(local, WorldDecalSettings)
    with pytest.raises(TypeError, match="WorldDecalSettings required"):
        decal_settings_to_local(local, 2.0)


@pytest.mark.parametrize(
    ("matrix", "code"),
    (
        (_diagonal(2.0, 1.0, 1.0), "NON_UNIFORM_OR_SHEARED_SOURCE_TRANSFORM"),
        (
            _Matrix3(((1.0, 0.25, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))),
            "NON_UNIFORM_OR_SHEARED_SOURCE_TRANSFORM",
        ),
        (_diagonal(-1.0, 1.0, 1.0), "MIRRORED_SOURCE_TRANSFORM"),
        (_diagonal(0.0, 0.0, 0.0), "DEGENERATE_SOURCE_TRANSFORM"),
    ),
)
def test_unsupported_source_transforms_are_explicit(matrix, code):
    with pytest.raises(DecalSourceTransformError) as exc_info:
        validate_decal_source_transform(matrix)

    assert exc_info.value.code == code
    assert str(exc_info.value).startswith(code + ":")


def test_structured_generation_receives_backend_local_settings(monkeypatch):
    captured = []
    monkeypatch.setattr(
        decals_module,
        "_generate_decal_transaction",
        lambda graph, source, settings, *_args, **_kwargs: (
            captured.append(settings)
            or decals_module._DecalTransactionResult(
                obj=SimpleNamespace(name="Decal_Seams_Source"),
                topology_changed=True,
                policy_counts=(),
            )
        ),
    )
    source = SimpleNamespace(name="Source", matrix_world=_diagonal(2.0, 2.0, 2.0))

    result = decals_module.generate_decal_result(
        PatchGraph(),
        source,
        DecalSettings(width_seam=0.15, offset=0.02, uv_length_scale=0.25),
        "SEAMS",
        scene=object(),
    )

    assert result.status == decals_module.PreviewStatus.UPDATED
    assert captured[0].width_seam == pytest.approx(0.075)
    assert captured[0].offset == pytest.approx(0.01)
    assert captured[0].uv_length_scale == pytest.approx(0.50)


def test_structured_generation_rejects_transform_before_transaction(monkeypatch):
    published = []
    monkeypatch.setattr(
        decals_module,
        "_generate_decal_transaction",
        lambda *_args, **_kwargs: published.append(True),
    )
    source = SimpleNamespace(name="Source", matrix_world=_diagonal(2.0, 1.0, 1.0))

    result = decals_module.generate_decal_result(
        PatchGraph(), source, DecalSettings(), "SEAMS", scene=object()
    )

    assert result.status == decals_module.PreviewStatus.ERROR
    assert result.reason.startswith("NON_UNIFORM_OR_SHEARED_SOURCE_TRANSFORM:")
    assert published == []


def test_source_adapter_reads_matrix_world():
    source = SimpleNamespace(matrix_world=_diagonal(4.0, 4.0, 4.0))

    local = local_decal_settings_for_source(
        DecalSettings(width_corner=0.2), source
    )

    assert local.width_corner == pytest.approx(0.05)
