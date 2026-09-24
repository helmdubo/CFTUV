from __future__ import annotations

import dataclasses
import hashlib
import inspect
import types
import typing
from dataclasses import FrozenInstanceError

import pytest

import cftuv_envelope
from cftuv_envelope import (
    ArrivalModelId,
    ChainUseId,
    EnvelopeInstanceId,
    EnvelopeSpecId,
    EqualityLocusId,
    FrontComponentId,
    FrontReadingId,
    InteractionApplicationId,
    InteractionCandidateId,
    InteractionComponentId,
    MutualArrivalCertificateId,
    PatchId,
    ResolvedContributionId,
    SameAlphaInteractionBatchId,
)


# Имя среза в константе меняется вместе с самим API: под старым именем
# новое число — враньё в заголовке. Прежнее закрепление среза C-R2C:
# 31cc1ec8fcd2768c5c69bb3ae15c0e754f58668f9eb775267e174241fb2480cb.
# Затем R1B: 9748dfddf08b289d4786f918b40940669ba5f1d5bf1fd1333dd5864df12ed566
# — три имени закона решётки, без которых хост не может назвать политику.
# Разница R1C — ещё три: `GridScaleSearchOrderV1`, `GridScaleTrialOutcomeV1`
# и `GridScaleTrialV1`. Масштаб теперь выбирается перебором, и читатель
# сертификата обязан уметь назвать типы, которыми этот перебор записан.
# FAN-DIR-BIND добавляет два обязательных tagged public record, а corrective
# seal — отдельный закон bound-tag, чтобы tag/law не могли расходиться.
# CHART-LATTICE-BOUND добавляет отдельный binding record, его одноэлементный
# закон, вершину и codec/validator: существующий CompiledPlanV1 не расширен,
# поэтому EC0 plan bytes при этом остаются прежними.
# Финальная recertification того же среза добавляет typed reason и отдельный
# evaluation-geometry certificate; legacy certificate остаётся отдельным типом,
# чтобы synthetic bytes не двигались.
# DENS-K добавляет именованные density IDs и отдельный tagged interval
# certificate; прежний SelectionIntervalCertificateV1 не расширен.
# CORR-DENS-02 добавляет V2 authority/window/spec/support одного полного
# рационального веера и отдельную evaluation-only H-lift власть; прежние V1
# records и selection certificate остаются отдельными типами.
# DENS-PROJECTIVE-CHART добавляет tagged atlas/piece/pole-ownership только
# для окон, пересекающих coordinate poles; single-chart V2 остаётся отдельным.
# P0-4 добавляет два embedding-сертификата, отдельный wrapper/codec/schema и
# два entrypoint-а; прежний RationalAffinePlanarMetricV2 остаётся неизменным.
# Прежнее закрепление среза P0-4:
# 36a104dce066d8a7893a1406f6d53f7bb06696cba38f8cd03fe102380ec48ea2.
# DENSITY-CANONICAL-RESTORATION добавляет ЧЕТЫРЕ имени: символ канонического
# отношения, закон восстановления, имя допуска намерения вместе с его
# категорией и сам сертификат. Сертификат селекции при этом НЕ расширен — иначе
# сдвинулись бы байты каждого угла корпуса, включая точные, где ничего не
# менялось; восстановления живут отдельной коллекцией плана.
# Прежнее закрепление того же среза (только восстановление счёта):
# 3d0371468591724de4b09caeebfece7e209359c562e8aff6257ca66271d58217.
# Продолжение той же карточки переносит гарантию подшага на восстановленный
# угол и добавляет ДВА имени: закон гарантии (оба его значения — старое и
# новое — названы в одном перечислении, чтобы разница была читаемой) и запись
# власти канонического веера. Прежние records не расширены ни одним полем.
PUBLIC_API_DENSITY_CANONICAL_RESTORATION_V1_SHA256 = (
    "ce36b3edfb2a59512cfce691b3a63208ac9d5d4e73142bf4e51fb646cc463cf7"
)

P0_4_ADDITIVE_PUBLIC_NAMES = (
    "ProjectionAnchorSelectionLawV1",
    "SourceSnapEmbeddingCertificateV1",
    "NearPlanarProjectionEmbeddingCertificateV1",
    "EmbeddingCertifiedRationalAffinePlanarMetricV1",
    "EMBEDDING_CERTIFIED_RATIONAL_AFFINE_PLANAR_METRIC_SCHEMA_V1",
    "EmbeddingCertifiedRationalAffinePlanarMetricCodecV1",
    "build_embedding_certified_rational_affine_planar_metric",
    "validate_embedding_certified_rational_affine_planar_metric",
)


def _public_record_types():
    seen = set()
    result = []
    for name in dir(cftuv_envelope):
        value = getattr(cftuv_envelope, name)
        if (
            inspect.isclass(value)
            and dataclasses.is_dataclass(value)
            and value.__module__.startswith("cftuv_envelope")
            and value not in seen
        ):
            seen.add(value)
            result.append(value)
    return tuple(result)


def _contains_forbidden_public_container(annotation) -> bool:
    if annotation in (typing.Any, object, list, dict):
        return True
    origin = typing.get_origin(annotation)
    if origin in (list, dict):
        return True
    if origin in (typing.Union, types.UnionType, tuple, frozenset):
        return any(
            _contains_forbidden_public_container(item)
            for item in typing.get_args(annotation)
            if item is not Ellipsis
        )
    return False


def test_all_public_records_are_frozen_and_slotted():
    records = _public_record_types()
    assert records
    for record in records:
        assert record.__dataclass_params__.frozen, record
        assert hasattr(record, "__slots__"), record


def test_top_level_public_api_is_explicit_and_snapshotted():
    public_names = {
        name
        for name in dir(cftuv_envelope)
        if not name.startswith("_") or name == "__version__"
    }
    assert public_names == set(cftuv_envelope.__all__)
    assert len(cftuv_envelope.__all__) == len(set(cftuv_envelope.__all__))
    digest = hashlib.sha256("\n".join(cftuv_envelope.__all__).encode("utf-8")).hexdigest()
    assert digest == PUBLIC_API_DENSITY_CANONICAL_RESTORATION_V1_SHA256


DENSITY_CANONICAL_RESTORATION_ADDITIVE_PUBLIC_NAMES = (
    "CanonicalReflexAngleRelationV1",
    "CanonicalAngleRestorationLawV1",
    "AngleTolerancePolicyIdV1",
    "CanonicalAngleRestorationCertificateV1",
    "SubturnGuaranteeLawV1",
    "CanonicalSubturnFanAuthorityV1",
)


def test_public_growth_is_additive_and_names_every_new_surface():
    """Рост публичного API идёт хвостом и каждый срез назван поимённо.

    Проверяются ДВА последних среза, а не только новый: если новый блок
    вытеснил бы предыдущий, «аддитивность» осталась бы верной побуквенно и
    ложной по смыслу.
    """

    latest = DENSITY_CANONICAL_RESTORATION_ADDITIVE_PUBLIC_NAMES
    assert len(cftuv_envelope.__all__) == 521
    assert cftuv_envelope.__all__[-len(latest) :] == latest
    previous = cftuv_envelope.__all__[
        -len(latest) - len(P0_4_ADDITIVE_PUBLIC_NAMES) : -len(latest)
    ]
    assert previous == P0_4_ADDITIVE_PUBLIC_NAMES


def test_public_dto_annotations_have_no_mutable_or_untyped_containers():
    for record in _public_record_types():
        hints = typing.get_type_hints(record)
        for field in dataclasses.fields(record):
            assert not _contains_forbidden_public_container(hints[field.name]), (
                record.__name__,
                field.name,
                hints[field.name],
            )


def test_typed_ids_reject_cross_identity_equality_and_mutation():
    patch_id = PatchId("shared-text")
    chain_use_id = ChainUseId("shared-text")
    assert patch_id != chain_use_id
    with pytest.raises(FrozenInstanceError):
        patch_id.value = "changed"


def test_session_d_public_ids_are_nominally_distinct():
    identities = (
        InteractionComponentId("shared-text"),
        ArrivalModelId("shared-text"),
        FrontReadingId("shared-text"),
        InteractionCandidateId("shared-text"),
        MutualArrivalCertificateId("shared-text"),
        EqualityLocusId("shared-text"),
        InteractionApplicationId("shared-text"),
        ResolvedContributionId("shared-text"),
        SameAlphaInteractionBatchId("shared-text"),
        EnvelopeSpecId("shared-text"),
        EnvelopeInstanceId("shared-text"),
        FrontComponentId("shared-text"),
    )
    assert len(set(identities)) == len(identities)


def test_nested_collections_are_immutable(projections):
    projection = projections[0]
    with pytest.raises(AttributeError):
        projection.snapshot.patches.add(next(iter(projection.snapshot.patches)))
    with pytest.raises(FrozenInstanceError):
        projection.request.decal_request_id = projection.request.decal_request_id
