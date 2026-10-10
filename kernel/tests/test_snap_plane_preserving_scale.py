"""Закон масштаба решётки `PLANE_PRESERVING_V1`: первый масштаб, на котором углы восстановлены И патч лежит точно в своей плоскости.

Закон заказывает только повтор хоста после названного отказа лотереи привязки (`SOURCE_SNAP_PLANE_PRESERVED_RETRY_V1`); умолчание ядра -
прежний закон «первый масштаб, восстановивший углы», и его ответ этим параметром не меняется ни в одном байте.

Данные - два настоящих патча `cover.008` (позиции binary64 такими, какими их несёт хост): 118 (плоскость x = const с шумом в 1 ulp float32: на
масштабе 16384 вершины садятся в два узла, на 8192 - в один) и 1002 (плоскость y = const, пять мелких масштабов рвут её). Оба на умолчании
идут по near-planar ветке с приведённым базисом решётки плоскости и ниже по конвейеру отказывают.
"""

from __future__ import annotations

from dataclasses import replace
from fractions import Fraction

import pytest

from cftuv_envelope.contracts.analysis import SourceVertexV1
from cftuv_envelope.contracts.metric import (
    ExactSourcePlaneCertificateV1,
    GridScaleLawV1,
    GridScaleTrialOutcomeV1,
    GridSnappingLawV1,
    IntegerGridCertificateV1,
    NearPlanarFramePolicyV1,
    NearPlanarProjectionCertificateV1,
    PlanarityAdmissionLawV1,
)
from cftuv_envelope.contracts.surface import SourceFaceV1
from cftuv_envelope.ids import PatchDomainId, PatchId, PhysicalEdgeId, SourceFaceId, SourceRevision, SourceVertexId
from cftuv_envelope.numeric import LocalPoint3V1, LocalVector3V1
from cftuv_envelope.planar_metric import build_embedding_certified_rational_affine_planar_metric
from cftuv_envelope.source_grid import resolve_source_grid, select_grid_scale, snapped_plane_is_exact
from cftuv_envelope.validation_metric import validate_embedding_certified_rational_affine_planar_metric

PATCH_118 = {
    "vertices": {
        "v0": (10.127838134765625, 14.90847110748291, 0.6442580223083496),
        "v1": (10.127837181091309, 10.938209533691406, 0.6442580223083496),
        "v2": (10.127838134765625, 14.908470153808594, 4.026504039764404),
        "v3": (10.127837181091309, 10.938211441040039, 4.026504039764404),
        "v4": (10.127838134765625, 13.623340606689453, 4.026504039764404),
        "v5": (10.127838134765625, 12.22334098815918, 4.026504039764404),
        "v6": (10.127838134765625, 12.223339080810547, 0.6442580223083496),
        "v7": (10.127838134765625, 13.62334156036377, 0.6442580223083496),
        "v8": (10.127838134765625, 12.223339080810547, 3.144237518310547),
        "v9": (10.127838134765625, 13.62334156036377, 3.144237518310547),
    },
    "faces": (
        ("f0", ("v8", "v9", "v4", "v5")),
        ("f1", ("v8", "v5", "v3", "v1", "v6")),
        ("f2", ("v9", "v7", "v0", "v2", "v4")),
    ),
}

PATCH_1002 = {
    "vertices": {
        "v0": (10.211957931518555, 30.033676147460938, 8.730934143066406),
        "v1": (10.027795791625977, 30.033676147460938, 8.730934143066406),
        "v2": (10.027793884277344, 30.033676147460938, 4.5201005935668945),
        "v3": (10.211957931518555, 30.033676147460938, 3.166018009185791),
        "v4": (10.027803421020508, 30.034900665283203, 3.164886236190796),
    },
    "faces": (("f0", ("v2", "v1", "v0", "v3", "v4")),),
}

#: Плоский патч на узлах решётки и скат с изломом: у первого плоскость точна на любом масштабе, у второго - ни на каком.
FLAT = {
    "vertices": {"v0": (0.0, 0.0, 0.0), "v1": (1.0, 0.0, 0.0), "v2": (1.0, 1.0, 0.0), "v3": (0.0, 1.0, 0.0)},
    "faces": (("f0", ("v0", "v1", "v2", "v3")),),
}
ROOF = {
    "vertices": {
        "v0": (0.0, 0.0, 0.0),
        "v1": (1.0, 0.0, 0.0),
        "v2": (1.0, 1.0, 0.0),
        "v3": (0.0, 1.0, 0.0),
        "v4": (2.0, 0.0, 1.0),
        "v5": (2.0, 1.0, 1.0),
    },
    "faces": (("f0", ("v0", "v1", "v2", "v3")), ("f1", ("v1", "v4", "v5", "v2"))),
}


def _vertices(patch):
    return {name: SourceVertexId(name) for name in patch["vertices"]}


def _faces(patch):
    ids = _vertices(patch)
    faces = []
    for name, cycle in patch["faces"]:
        pairs = [tuple(sorted((cycle[i], cycle[(i + 1) % len(cycle)]))) for i in range(len(cycle))]
        faces.append(
            SourceFaceV1(
                face_id=SourceFaceId(name),
                patch_id=PatchId("patch"),
                vertex_cycle=tuple(ids[item] for item in cycle),
                edge_cycle=tuple(PhysicalEdgeId("-".join(pair)) for pair in pairs),
                polygon_normal=LocalVector3V1(0.0, 0.0, 1.0),
                triangle_ids=(),
            )
        )
    return tuple(faces)


def _positions(patch):
    ids = _vertices(patch)
    return {ids[name]: tuple(Fraction(*float(value).as_integer_ratio()) for value in point) for name, point in patch["vertices"].items()}


def _resolve(patch, law):
    return resolve_source_grid(
        positions=_positions(patch),
        faces=_faces(patch),
        snapping_law=GridSnappingLawV1.SOURCE_ONLY_GRID_SNAP_V1,
        scale_law=law,
    )


FIRST = GridScaleLawV1.FIRST_ANGLE_RESTORING_V1
PLANE = GridScaleLawV1.PLANE_PRESERVING_V1
TORN = GridScaleTrialOutcomeV1.RELATIONS_RESTORED_PLANE_TORN
RESTORED = GridScaleTrialOutcomeV1.RELATIONS_RESTORED
NOT_RESTORED = GridScaleTrialOutcomeV1.RELATIONS_NOT_RESTORED


def test_the_default_law_takes_the_first_angle_restoring_scale_and_tears_the_plane():
    for patch, scale in ((PATCH_118, 16384), (PATCH_1002, 8192)):
        facts = _resolve(patch, FIRST)
        certificate = facts.certificate

        assert certificate.source_scale == scale
        assert certificate.scales_skipped_for_plane == 0 and certificate.scale_law is FIRST
        assert [trial.outcome for trial in certificate.scale_trials] == [RESTORED]
        assert not snapped_plane_is_exact(facts.positions, _faces(patch))


def test_the_omitted_law_is_the_default_law_bit_for_bit():
    assert resolve_source_grid(
        positions=_positions(PATCH_118), faces=_faces(PATCH_118), snapping_law=GridSnappingLawV1.SOURCE_ONLY_GRID_SNAP_V1
    ) == _resolve(PATCH_118, FIRST)


def test_the_plane_preserving_law_takes_the_first_scale_that_restores_the_angles_and_keeps_the_plane():
    facts = _resolve(PATCH_118, PLANE)
    certificate = facts.certificate

    assert certificate.source_scale == 8192
    assert [(trial.scale, trial.outcome) for trial in certificate.scale_trials] == [(16384, TORN), (8192, RESTORED)]
    assert certificate.scales_skipped_for_plane == 1 and certificate.scale_law is PLANE
    assert certificate.restored_right_corners == certificate.intended_right_corners == 10
    assert snapped_plane_is_exact(facts.positions, _faces(PATCH_118))
    # скипнутая проба - ТОТ масштаб, который выбрал бы первый закон, с тем же числом восстановленных углов
    assert certificate.scale_trials[0].scale == _resolve(PATCH_118, FIRST).certificate.source_scale
    assert certificate.scale_trials[0].restored_right_corners == certificate.intended_right_corners


def test_a_deeper_tear_skips_every_scale_that_keeps_the_angles_and_breaks_the_plane():
    certificate = _resolve(PATCH_1002, PLANE).certificate

    assert certificate.source_scale == 256
    assert [trial.outcome for trial in certificate.scale_trials] == [TORN] * 5 + [RESTORED]
    assert [trial.scale for trial in certificate.scale_trials] == [8192, 4096, 2048, 1024, 512, 256]
    assert certificate.scales_skipped_for_plane == 5


def test_a_patch_on_an_exact_plane_is_answered_identically_by_both_laws():
    first, plane = _resolve(FLAT, FIRST), _resolve(FLAT, PLANE)

    assert plane == first and plane.certificate.scales_skipped_for_plane == 0


def test_a_patch_with_no_plane_in_the_window_keeps_the_answer_of_the_first_law():
    """Скату плоскость не обещана ни на одном масштабе: отказ стал бы новой причиной, которой у прежнего закона не было."""

    first, plane = _resolve(ROOF, FIRST), _resolve(ROOF, PLANE)

    assert plane == first and plane.certificate.scale_law is FIRST


def test_the_plane_law_without_the_faces_is_refused_not_guessed():
    facts = _resolve(FLAT, FIRST)
    with pytest.raises(ValueError, match="needs the patch faces"):
        select_grid_scale(
            positions=_positions(FLAT),
            intended=(),
            window=None,
            search_order=facts.certificate.search_order,
            scale_law=PLANE,
        )


def test_the_certificate_refuses_a_record_that_contradicts_its_own_trials():
    certificate = _resolve(PATCH_118, PLANE).certificate
    first, winner = certificate.scale_trials

    with pytest.raises(ValueError, match="последняя проба"):
        replace(certificate, scale_trials=(first, replace(winner, outcome=TORN)))
    with pytest.raises(ValueError, match="прошедшая проба до победителя"):
        replace(certificate, scale_trials=(replace(first, outcome=RESTORED), winner))
    with pytest.raises(ValueError, match="исход пробы расходится"):
        replace(certificate, scale_trials=(replace(first, restored_right_corners=0), winner))
    with pytest.raises(ValueError, match="исход пробы расходится"):
        replace(certificate, scale_trials=(replace(first, outcome=NOT_RESTORED), winner))


def _metric(patch, law):
    ids = _vertices(patch)
    return build_embedding_certified_rational_affine_planar_metric(
        source_revision=SourceRevision("revision"),
        patch_domain_id=PatchDomainId("domain"),
        owner_patch_id=PatchId("patch"),
        source_vertices=tuple(
            SourceVertexV1(vertex_id=ids[name], position=LocalPoint3V1(*point)) for name, point in patch["vertices"].items()
        ),
        source_faces=_faces(patch),
        planarity_policy=PlanarityAdmissionLawV1.NEAR_PLANAR_PROJECTION_V1,
        grid_policy=GridSnappingLawV1.SOURCE_ONLY_GRID_SNAP_V1,
        near_planar_frame_policy=NearPlanarFramePolicyV1.REDUCED_INTEGER_PLANE_LATTICE_BASIS_V1,
        grid_scale_law=law,
    )


def _issues(patch, record):
    ids = _vertices(patch)
    return validate_embedding_certified_rational_affine_planar_metric(
        record,
        source_vertices=tuple(SourceVertexV1(vertex_id=ids[name], position=LocalPoint3V1(*point)) for name, point in patch["vertices"].items()),
        source_faces=_faces(patch),
        owner_patch_id=PatchId("patch"),
        expected_source_revision=SourceRevision("revision"),
        expected_patch_domain_id=PatchDomainId("domain"),
        expected_source_lineage=frozenset(),
    )


@pytest.mark.parametrize("patch", (PATCH_118, PATCH_1002), ids=("118", "1002"))
def test_the_builder_under_each_law_gives_a_record_the_validator_reproduces(patch):
    """Метрика на умолчании - near-planar с приведённым базисом; под законом плоскости - точная плоскость; оба пересчитываются валидатором."""

    default, retried = _metric(patch, FIRST), _metric(patch, PLANE)

    assert isinstance(default.metric.planarity_certificate, NearPlanarProjectionCertificateV1)
    assert isinstance(retried.metric.planarity_certificate, ExactSourcePlaneCertificateV1)
    assert default.metric.grid_certificate.scale_law is FIRST and retried.metric.grid_certificate.scale_law is PLANE
    assert _issues(patch, default) == ()
    assert _issues(patch, retried) == ()


def test_a_certificate_that_claims_the_other_scale_is_not_reproduced():
    """Отрицательный контроль: метрика законом плоскости с сертификатом умолчания (и наоборот) не проходит пересчёт."""

    default, retried = _metric(PATCH_118, FIRST), _metric(PATCH_118, PLANE)
    forged_retried = replace(retried, metric=replace(retried.metric, grid_certificate=default.metric.grid_certificate))
    forged_default = replace(default, metric=replace(default.metric, grid_certificate=retried.metric.grid_certificate))

    assert _issues(PATCH_118, forged_retried) != ()
    assert _issues(PATCH_118, forged_default) != ()


def test_the_default_builder_call_does_not_know_the_law():
    """Умолчание ядра - прежний путь: вызов без закона равен вызову с первым законом."""

    ids = _vertices(PATCH_118)
    plain = build_embedding_certified_rational_affine_planar_metric(
        source_revision=SourceRevision("revision"),
        patch_domain_id=PatchDomainId("domain"),
        owner_patch_id=PatchId("patch"),
        source_vertices=tuple(SourceVertexV1(vertex_id=ids[name], position=LocalPoint3V1(*point)) for name, point in PATCH_118["vertices"].items()),
        source_faces=_faces(PATCH_118),
        planarity_policy=PlanarityAdmissionLawV1.NEAR_PLANAR_PROJECTION_V1,
        grid_policy=GridSnappingLawV1.SOURCE_ONLY_GRID_SNAP_V1,
        near_planar_frame_policy=NearPlanarFramePolicyV1.REDUCED_INTEGER_PLANE_LATTICE_BASIS_V1,
    )

    assert plain == _metric(PATCH_118, FIRST)
    assert isinstance(plain.metric.grid_certificate, IntegerGridCertificateV1)
