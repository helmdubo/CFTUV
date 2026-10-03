"""Ключ исполнения батча и закон UV как ЯВНЫЙ параметр материализации.

Аудит материализатора нашёл дыру: `materialize_domain` брал `decal_request_id`
батча из ПЕРЕДАННОГО запроса и не сверял его с запросом, по которому
скомпилирована подготовка, а свип подставлял закон UV через
`dataclasses.replace` и держал старый id. Здесь исполняемая форма починки:

* `materialization_request(prepared, uv_policy_id=...)` — скомпилированный
  запрос с заменой ТОЛЬКО закона UV; ключ исполнения батча — ровно ключ плана;
* `admit` отказывает именованным `REQUEST_DOES_NOT_MATCH_PREPARATION`, если
  переданный запрос отличается от скомпилированного чем-то, кроме законов
  выхода и alpha, либо покрытие посчитано на подготовке с другим ключом плана;
* публичная нормаль плоскости домена (`plane_normal_binary64`) согласована с
  обходом треугольников батча — на прямой и на зеркальной карте.
"""

from __future__ import annotations

import dataclasses
import math
from functools import lru_cache

import pytest

from cftuv_envelope.contracts.metric import ExactRationalV1
from cftuv_envelope.ids import DecalRequestId, PolicyId
from cftuv_envelope.materialize.admit import (
    OUTPUT_POLICY_FIELDS,
    MaterializationOutcome,
    materialization_request,
    request_mismatch,
)
from cftuv_envelope.materialize.audit import audit_batch
from cftuv_envelope.materialize.domain import materialize_domain
from cftuv_envelope.materialize.lift import plane_normal_binary64
from cftuv_envelope.materialize.uv_law import UV_DIRECT_STRIP_V1
from cftuv_envelope.wavefront.conveyor import ConveyorOutcome

import materialize_factories as factories


@lru_cache(maxsize=None)
def _case(name):
    if name == "weighted":
        return factories.field_domain("building_002_weighted_normals_v1")
    return factories.two_edge_chain_domain()


def _materialize(name, request):
    prepared, coverage, _request = _case(name)
    return materialize_domain(prepared, coverage, request=request)


# --------------------------------------------------------------------------
# Закон UV как параметр, личность запроса — скомпилированная
# --------------------------------------------------------------------------


@pytest.mark.parametrize("name", ("weighted", "two_edge"))
def test_the_materialization_request_changes_only_the_uv_law(name):
    prepared, _coverage, _request = _case(name)
    compiled = prepared.compilation.decal_request
    chosen = materialization_request(prepared, uv_policy_id=UV_DIRECT_STRIP_V1)
    assert chosen.uv_policy_id == UV_DIRECT_STRIP_V1
    assert dataclasses.replace(chosen, uv_policy_id=compiled.uv_policy_id) == compiled
    # Строка принимается так же, как идентификатор (хост хранит имена строками).
    assert materialization_request(prepared, uv_policy_id="UV_DIRECT_STRIP_V1") == chosen


@pytest.mark.parametrize("name", ("weighted", "two_edge"))
def test_the_batch_carries_exactly_the_compiled_execution_key_and_records_the_law(name):
    prepared, _coverage, _request = _case(name)
    result = _materialize(
        name, materialization_request(prepared, uv_policy_id=UV_DIRECT_STRIP_V1)
    )
    assert result.outcome is MaterializationOutcome.MATERIALIZED, result.detail
    key = prepared.compilation.plan_key
    assert result.batch.decal_request_id == key.decal_request_id
    assert result.batch.patch_domain_id == key.patch_domain_id
    assert "cftuv.envelope.uv_policy.UV_DIRECT_STRIP_V1" in {
        item.value for item in result.batch.contract_versions
    }


def test_a_materialization_request_needs_a_compiled_one():
    prepared, _coverage, _request = _case("two_edge")
    bare = dataclasses.replace(prepared, compilation=None)
    with pytest.raises(ValueError, match="no compiled DecalRequestV1"):
        materialization_request(bare, uv_policy_id=UV_DIRECT_STRIP_V1)


# --------------------------------------------------------------------------
# Ключ исполнения: именованный отказ вместо чужого id в батче
# --------------------------------------------------------------------------


def test_a_request_with_another_id_is_refused_by_name_and_the_detail_names_the_field():
    prepared, _coverage, _request = _case("weighted")
    good = materialization_request(prepared, uv_policy_id=UV_DIRECT_STRIP_V1)
    foreign = dataclasses.replace(good, decal_request_id=DecalRequestId("another"))
    result = _materialize("weighted", foreign)
    assert result.outcome is MaterializationOutcome.REQUEST_DOES_NOT_MATCH_PREPARATION
    assert result.detail.startswith("decal_request_id:")
    assert "another" in result.detail
    assert result.batch is None and result.content_digest == ""


@pytest.mark.parametrize(
    "field,value",
    (
        ("selected_chain_use_ids", frozenset()),
        ("max_subturn_value_id", None),
        ("angular_profile_selection_policy_id", None),
        ("developable_stretch_budget", ExactRationalV1(7, 20)),
    ),
)
def test_every_plan_affecting_field_is_guarded(field, value):
    prepared, _coverage, _request = _case("weighted")
    good = materialization_request(prepared, uv_policy_id=UV_DIRECT_STRIP_V1)
    if value is None:
        # Любое иное значение того же поля: берём другого члена перечисления.
        members = [
            item
            for item in type(getattr(good, field))
            if item != getattr(good, field)
        ]
        assert members, f"{field} must have a second member for this test"
        value = members[0]
    changed = dataclasses.replace(good, **{field: value})
    result = _materialize("weighted", changed)
    assert result.outcome is MaterializationOutcome.REQUEST_DOES_NOT_MATCH_PREPARATION
    assert result.detail.startswith(f"{field}:")


def test_the_output_laws_and_alpha_may_differ_and_nothing_else_may():
    prepared, coverage, _request = _case("weighted")
    good = materialization_request(prepared, uv_policy_id=UV_DIRECT_STRIP_V1)
    assert OUTPUT_POLICY_FIELDS == {
        "requested_alpha",
        "uv_policy_id",
        "material_policy_id",
    }
    other_material = dataclasses.replace(
        good, material_policy_id=PolicyId("SOME_MATERIAL_V1")
    )
    assert request_mismatch(prepared, coverage, other_material) == ""
    result = _materialize("weighted", other_material)
    assert result.outcome is MaterializationOutcome.MATERIALIZED
    # Закон материала виден в батче: его берёт запрос выхода, не подготовка.
    assert {face.material_id.value for face in result.batch.faces} == {
        "SOME_MATERIAL_V1"
    }
    # Тот же запрос на другой alpha: покрытие несёт свою, запрос не оспаривается.
    other_alpha = dataclasses.replace(
        good,
        requested_alpha=dataclasses.replace(
            good.requested_alpha, value=good.requested_alpha.value + 1
        ),
    )
    assert request_mismatch(prepared, coverage, other_alpha) == ""


def test_a_coverage_of_another_plan_is_refused_by_name():
    prepared, _coverage, _request = _case("weighted")
    other_prepared, other_coverage, _ = _case("two_edge")
    good = materialization_request(prepared, uv_policy_id=UV_DIRECT_STRIP_V1)
    result = materialize_domain(prepared, other_coverage, request=good)
    assert result.outcome is MaterializationOutcome.REQUEST_DOES_NOT_MATCH_PREPARATION
    assert "coverage plan_key" in result.detail
    assert other_prepared is not prepared


def test_exactness_is_reported_before_the_key_and_the_key_before_the_law():
    prepared, coverage, _request = _case("weighted")
    good = materialization_request(prepared, uv_policy_id=UV_DIRECT_STRIP_V1)
    foreign = dataclasses.replace(good, decal_request_id=DecalRequestId("another"))
    broken = dataclasses.replace(
        coverage, outcome=ConveyorOutcome.COVERAGE_DID_NOT_CLOSE
    )
    assert (
        materialize_domain(prepared, broken, request=foreign).outcome
        is MaterializationOutcome.COVERAGE_IS_NOT_EXACT
    )
    # Запрос подготовки без подмены закона — это отладочный закон: отказ по закону,
    # а не по ключу (ключ у него тот).
    debug = prepared.compilation.decal_request
    assert (
        materialize_domain(prepared, coverage, request=debug).outcome
        is MaterializationOutcome.UV_POLICY_UNSUPPORTED
    )
    assert (
        materialize_domain(prepared, coverage, request=foreign).outcome
        is MaterializationOutcome.REQUEST_DOES_NOT_MATCH_PREPARATION
    )


# --------------------------------------------------------------------------
# Публичная нормаль плоскости
# --------------------------------------------------------------------------


def _triangle_normals(batch):
    position = {item.vert_key: item.position for item in batch.vertices}
    for face in batch.faces:
        a, b, c = (position[key] for key in face.ordered_vert_keys)
        ab = (b.x - a.x, b.y - a.y, b.z - a.z)
        ac = (c.x - a.x, c.y - a.y, c.z - a.z)
        yield (
            ab[1] * ac[2] - ab[2] * ac[1],
            ab[2] * ac[0] - ab[0] * ac[2],
            ab[0] * ac[1] - ab[1] * ac[0],
        )


def _faces_the_normal(batch, normal) -> tuple[int, int]:
    """`(треугольников по нормали, против неё)` у батча; нулевые не считаются."""

    front = back = 0
    for tri in _triangle_normals(batch):
        dot = tri[0] * normal.x + tri[1] * normal.y + tri[2] * normal.z
        if abs(dot) < 1e-12:
            continue
        front += dot > 0
        back += dot < 0
    return front, back


@pytest.mark.parametrize("name", ("weighted", "two_edge"))
def test_the_plane_normal_is_unit_and_agrees_with_the_triangle_winding(name):
    prepared, _coverage, _request = _case(name)
    result = _materialize(
        name, materialization_request(prepared, uv_policy_id=UV_DIRECT_STRIP_V1)
    )
    normal = plane_normal_binary64(prepared.context.frame)
    assert math.isclose(
        math.sqrt(normal.x**2 + normal.y**2 + normal.z**2), 1.0, rel_tol=1e-15
    )
    front, back = _faces_the_normal(result.batch, normal)
    assert front > 0 and back == 0
    # И с нормалью исходной грани владельца (аудит сетки): ни одного вывернутого.
    source = sorted(
        prepared.context.snapshot.surface_ir.source_faces,
        key=lambda item: item.face_id.value,
    )[0].polygon_normal
    assert audit_batch(result.batch, (source.x, source.y, source.z)).flipped_vs_source == 0
    assert normal.x * source.x + normal.y * source.y + normal.z * source.z > 0.99


def test_a_mirrored_chart_flips_both_the_winding_and_the_normal_together():
    prepared, coverage, _request = _case("two_edge")
    straight_normal = plane_normal_binary64(prepared.context.frame)
    mirrored_frame = dataclasses.replace(
        prepared.context.frame,
        chart_orientation=type(prepared.context.frame.chart_orientation)(
            "COORDINATE_CW_MATCHES_OWNER_PATCH"
        ),
    )
    swapped = dataclasses.replace(
        prepared, context=dataclasses.replace(prepared.context, frame=mirrored_frame)
    )
    result = materialize_domain(
        swapped,
        coverage,
        request=materialization_request(swapped, uv_policy_id=UV_DIRECT_STRIP_V1),
    )
    assert result.outcome is MaterializationOutcome.MATERIALIZED
    mirrored_normal = plane_normal_binary64(mirrored_frame)
    assert (mirrored_normal.x, mirrored_normal.y, mirrored_normal.z) == (
        -straight_normal.x,
        -straight_normal.y,
        -straight_normal.z,
    )
    front, back = _faces_the_normal(result.batch, mirrored_normal)
    assert front > 0 and back == 0
