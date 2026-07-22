from __future__ import annotations

import dataclasses
import inspect
import types
import typing
from dataclasses import FrozenInstanceError

import pytest

import cftuv_envelope
from cftuv_envelope import ChainUseId, PatchId


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


def test_nested_collections_are_immutable(projections):
    projection = projections[0]
    with pytest.raises(AttributeError):
        projection.snapshot.patches.add(next(iter(projection.snapshot.patches)))
    with pytest.raises(FrozenInstanceError):
        projection.request.decal_request_id = projection.request.decal_request_id

