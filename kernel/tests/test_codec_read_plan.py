"""План чтения записи кодека (`codec._ReadPlan`) не меняет ни байты, ни объекты, ни отказы.

Чтение записи брало из определения её класса `dataclasses.fields` и `typing.get_type_hints` на КАЖДУЮ запись; разбор
подсказок (строковые аннотации через `eval`) был около 60 % чтения снапшота `building` и 78-91 % времени стадии
`SNAPSHOT_VALIDATION` выгрузки хоста. Теперь план считается один раз на класс. Эти проверки доказывают три вещи, а не
обещают их: (1) план равен определению класса, включая то, как разрешаются подсказки (и неразрешимые отказывают тем же
образом КАЖДЫЙ раз: отказ не запоминается); (2) прежняя ветка чтения записи из родителя e85b063 (без `_ReadPlan`)
и чтение с планом дают одинаковые объекты и побайтно те же канонические байты на каждом
сохранённом снапшоте и запросе; (3) названные отказы чтения (лишнее поле, недостающее, тег не того типа, умолчание,
названное явно) совпадают текстом в обоих режимах; а подсказки класса разрешаются ровно один раз.
"""

from __future__ import annotations

import dataclasses
import json
import typing
from dataclasses import dataclass
from enum import Enum
from pathlib import Path

import pytest

import cftuv_envelope as kernel
from cftuv_envelope import codec
from cftuv_envelope.ids import OpaqueId
from cftuv_envelope.schema import is_wire_default_field


FIXTURES = Path(__file__).resolve().parents[1] / "fixtures"
SNAPSHOTS = sorted(FIXTURES.rglob("analysis_snapshot.json"))
REQUESTS = sorted(FIXTURES.rglob("decal_request.json"))
#: Снапшот, который принимает кодек (в наборе есть и тот, что отказывает названным отказом метрики).
SMALL_SNAPSHOT = FIXTURES / "building_002_weighted_normals_v1" / "analysis_snapshot.json"
LARGE_SNAPSHOT = (
    FIXTURES
    / "sem_clb_02_lost_domains_v1"
    / "cases"
    / "building_all_seams_patch_006_lost_resolved_v1"
    / "analysis_snapshot.json"
)


def _name(path: Path) -> str:
    return path.relative_to(FIXTURES).parent.as_posix()


def _fresh_caches(monkeypatch) -> None:
    """Все кэши кодека пусты: следующий вызов идёт путём ПЕРВОЙ встречи класса."""

    monkeypatch.setattr(codec, "_READ_PLANS", {})
    monkeypatch.setattr(codec, "_RECORD_FIELDS", {})
    monkeypatch.setattr(codec, "_OPAQUE_KINDS", set())


def _without_plans(monkeypatch) -> None:
    """Независимая прежняя ветка записи (e85b063^): не использует ни план, ни его построитель."""

    decode_other = codec._decode_as

    def legacy_decode(data, annotation):
        if (
            not isinstance(annotation, type)
            or issubclass(annotation, (OpaqueId, Enum))
            or not dataclasses.is_dataclass(annotation)
        ):
            return decode_other(data, annotation)
        if not isinstance(data, dict):
            raise codec.ContractCodecError(f"{annotation.__name__} must be a JSON object")
        if data.get("$type") != annotation.__name__:
            raise codec.ContractCodecError(
                f"expected record {annotation.__name__}, got {data.get('$type')}"
            )
        expected = {field.name for field in dataclasses.fields(annotation)} | {"$type"}
        required = {
            field.name for field in dataclasses.fields(annotation) if not is_wire_default_field(field)
        } | {"$type"}
        extra = set(data) - expected
        missing = required - set(data)
        if extra or missing:
            raise codec.ContractCodecError(
                f"{annotation.__name__} field mismatch; extra={sorted(extra)}, missing={sorted(missing)}"
            )
        hints = typing.get_type_hints(annotation)
        kwargs = {
            field.name: legacy_decode(data[field.name], hints[field.name])
            for field in dataclasses.fields(annotation)
            if field.name in data
        }
        for field in dataclasses.fields(annotation):
            if is_wire_default_field(field) and field.name in kwargs and kwargs[field.name] == field.default:
                raise codec.ContractCodecError(
                    f"{annotation.__name__}.{field.name} equals its default and must be omitted on the wire"
                )
        return annotation(**kwargs)

    monkeypatch.setattr(codec, "_decode_as", legacy_decode)

    def forbidden_plan(annotation):
        pytest.fail(f"legacy oracle used a read plan for {annotation.__name__}")

    monkeypatch.setattr(codec, "_read_plan", forbidden_plan)


def _outcome(call):
    """`('ok', результат)` либо `('refused', тип, текст)`: отказ сравнивается словами, а не фактом."""

    try:
        return ("ok", call())
    except Exception as error:  # noqa: BLE001 - сравнивается именно класс и текст отказа
        return ("refused", type(error).__name__, str(error))


def _record_classes() -> list[type]:
    return sorted(codec._TYPE_REGISTRY.values(), key=lambda cls: cls.__qualname__)


# --------------------------------------------------------------------------
# 1. План равен определению класса
# --------------------------------------------------------------------------


@pytest.mark.parametrize("record_type", _record_classes(), ids=lambda cls: cls.__qualname__)
def test_the_plan_of_every_public_record_equals_its_definition(record_type):
    plan = codec._ReadPlan(record_type)
    record_fields = dataclasses.fields(record_type)
    wire_default = [item for item in record_fields if is_wire_default_field(item)]

    assert plan.names == tuple(item.name for item in record_fields)
    assert plan.expected == {item.name for item in record_fields} | {"$type"}
    assert plan.required == {
        item.name for item in record_fields if item not in wire_default
    } | {"$type"}
    assert plan.wire_defaults == tuple((item.name, item.default) for item in wire_default)

    expected = _outcome(lambda: typing.get_type_hints(record_type))
    first = _outcome(lambda: plan.hints(record_type))
    second = _outcome(lambda: plan.hints(record_type))
    # Подсказки те же, что отдал бы `get_type_hints` сейчас, и повтор не отличается от первого вызова:
    # разрешимые запоминаются, неразрешимые отказывают тем же образом каждый раз.
    assert first == expected
    assert second == expected


def test_every_wire_default_field_of_the_contracts_is_in_a_plan():
    named = {
        (cls.__name__, name)
        for cls in _record_classes()
        for name, _ in codec._ReadPlan(cls).wire_defaults
    }
    assert ("AnalysisSnapshotV1", "seam_neighbour_faces") in named
    assert ("DecalRequestV1", "chart_reach_cap") in named
    assert ("DecalRequestV1", "developable_stretch_budget") in named


class _LaterDefined:
    pass


@dataclass(frozen=True)
class _ResolvesLater:
    value: _LaterDefined | None


@dataclass(frozen=True)
class _NeverResolves:
    value: _DefinedNowhere  # noqa: F821 - неразрешимая ссылка нарочно


def test_a_forward_reference_resolves_to_the_same_hints_as_without_the_plan(monkeypatch):
    monkeypatch.setattr(codec, "_READ_PLANS", {})
    plan = codec._read_plan(_ResolvesLater)
    assert plan.hints(_ResolvesLater) == typing.get_type_hints(_ResolvesLater)
    assert plan.hints(_ResolvesLater) is plan.hints(_ResolvesLater)


def test_an_unresolvable_forward_reference_refuses_identically_every_time(monkeypatch):
    monkeypatch.setattr(codec, "_READ_PLANS", {})
    data = {"$type": "_NeverResolves", "value": 1}
    first = _outcome(lambda: codec._decode_as(data, _NeverResolves))
    second = _outcome(lambda: codec._decode_as(data, _NeverResolves))
    _without_plans(monkeypatch)
    legacy = _outcome(lambda: codec._decode_as(data, _NeverResolves))
    assert first[:2] == ("refused", "NameError")
    assert first == second == legacy


def test_field_errors_precede_forward_reference_errors_even_after_a_failed_resolution(monkeypatch):
    complete = {"$type": "_NeverResolves", "value": 1}
    incomplete = {"$type": "_NeverResolves", "bogus": 1}
    with monkeypatch.context() as scope:
        _without_plans(scope)
        legacy = _outcome(lambda: codec._decode_as(incomplete, _NeverResolves))
    _fresh_caches(monkeypatch)
    assert _outcome(lambda: codec._decode_as(incomplete, _NeverResolves)) == legacy
    assert _outcome(lambda: codec._decode_as(complete, _NeverResolves))[:2] == ("refused", "NameError")
    assert _outcome(lambda: codec._decode_as(incomplete, _NeverResolves)) == legacy
    assert legacy == (
        "refused",
        "ContractCodecError",
        "_NeverResolves field mismatch; extra=['bogus'], missing=['value']",
    )


def test_a_failed_forward_reference_can_resolve_on_a_later_read(monkeypatch):
    _fresh_caches(monkeypatch)
    data = {"$type": "_NeverResolves", "value": 1}
    assert _outcome(lambda: codec._decode_as(data, _NeverResolves))[:2] == ("refused", "NameError")
    monkeypatch.setitem(globals(), "_DefinedNowhere", int)
    decoded = codec._decode_as(data, _NeverResolves)
    with monkeypatch.context() as scope:
        _without_plans(scope)
        assert codec._decode_as(data, _NeverResolves) == decoded == _NeverResolves(1)


# --------------------------------------------------------------------------
# 2. Чтение с планом и без него: те же объекты и те же байты
# --------------------------------------------------------------------------


@pytest.mark.parametrize("path", SNAPSHOTS, ids=_name)
def test_a_stored_snapshot_decodes_to_the_same_record_with_and_without_the_plan(path, monkeypatch):
    raw = path.read_bytes()
    reader = kernel.AnalysisSnapshotCodecV1
    legacy_outcome = None
    with monkeypatch.context() as scope:
        _fresh_caches(scope)
        _without_plans(scope)
        legacy_outcome = _outcome(lambda: reader.loads(raw))
    _fresh_caches(monkeypatch)
    cold = _outcome(lambda: reader.loads(raw))
    warm = _outcome(lambda: reader.loads(raw))

    # Отказ чтения (набор хранит и снапшот с названным отказом метрики) тоже ответ: он обязан совпасть словами.
    if legacy_outcome[0] == "refused":
        assert cold == legacy_outcome
        assert warm == legacy_outcome
        return
    legacy, decoded = legacy_outcome[1], cold[1]
    assert decoded == legacy
    assert warm[1] == legacy
    # Хранимые байты фикстуры — каноническая запись прежнего писателя: оба режима обязаны вернуть именно их.
    assert reader.dumps(decoded) == raw
    assert reader.dumps(legacy) == raw
    again = reader.loads(reader.dumps(decoded))
    assert reader.dumps(again) == raw


@pytest.mark.parametrize("path", REQUESTS, ids=_name)
def test_a_stored_request_decodes_to_the_same_record_with_and_without_the_plan(path, monkeypatch):
    raw = path.read_bytes()
    reader = kernel.DecalRequestCodecV1
    with monkeypatch.context() as scope:
        _fresh_caches(scope)
        _without_plans(scope)
        legacy = reader.loads(raw)
    _fresh_caches(monkeypatch)
    decoded = reader.loads(raw)
    assert decoded == legacy
    assert reader.dumps(decoded) == reader.dumps(legacy) == raw


def test_the_snapshot_codec_round_trip_the_host_validates_is_a_fixed_point(monkeypatch):
    """Ровно то, что выпускает выгрузка хоста: `dumps -> loads -> dumps` на сохранённом снапшоте."""

    reader = kernel.AnalysisSnapshotCodecV1
    snapshot = reader.loads(LARGE_SNAPSHOT.read_bytes())
    _fresh_caches(monkeypatch)
    payload = reader.dumps(snapshot)
    decoded = reader.loads(payload)
    assert reader.dumps(decoded) == payload == LARGE_SNAPSHOT.read_bytes()


# --------------------------------------------------------------------------
# 3. Названные отказы чтения совпадают текстом; подсказки класса разрешаются один раз
# --------------------------------------------------------------------------


def _wire(path: Path) -> dict:
    return json.loads(path.read_bytes())


REFUSAL_CASES = (
    "extra field",
    "missing field",
    "wrong id tag",
    "wrong record tag",
    "default named explicitly",
    "not an object",
)


def _reading_refusals() -> dict[str, tuple[type, bytes, str]]:
    snapshot = _wire(SMALL_SNAPSHOT)
    extra = dict(snapshot, bogus=1)
    missing = {key: value for key, value in snapshot.items() if key != "source_revision"}
    wrong_tag = json.loads(json.dumps(snapshot))
    wrong_tag["source_revision"]["$id_type"] = "PatchId"
    wrong_nested = json.loads(json.dumps(snapshot))
    nested = next(
        key for key, value in wrong_nested.items() if isinstance(value, dict) and "$type" in value
    )
    wrong_nested[nested]["$type"] = "NotThatRecord"

    request_path = next(path for path in REQUESTS if _name(path) == "building_002_weighted_normals_v1")
    request = _wire(request_path)
    default = next(
        item.default
        for item in dataclasses.fields(kernel.DecalRequestV1)
        if item.name == "chart_reach_cap"
    )
    explicit_default = dict(request, chart_reach_cap=codec.to_canonical_data(default))

    def encode(value) -> bytes:
        return json.dumps(value).encode("utf-8")

    return {
        "extra field": (kernel.AnalysisSnapshotCodecV1, encode(extra), "extra=['bogus']"),
        "missing field": (kernel.AnalysisSnapshotCodecV1, encode(missing), "missing=['source_revision']"),
        "wrong id tag": (kernel.AnalysisSnapshotCodecV1, encode(wrong_tag), "expected SourceRevision"),
        "wrong record tag": (kernel.AnalysisSnapshotCodecV1, encode(wrong_nested), "NotThatRecord"),
        "default named explicitly": (
            kernel.DecalRequestCodecV1,
            encode(explicit_default),
            "DecalRequestV1.chart_reach_cap equals its default and must be omitted on the wire",
        ),
        "not an object": (kernel.AnalysisSnapshotCodecV1, b"[1, 2]", "AnalysisSnapshotV1 must be a JSON object"),
    }


@pytest.mark.parametrize("case", REFUSAL_CASES)
def test_a_named_reading_refusal_has_the_same_words_with_and_without_the_plan(case, monkeypatch):
    assert set(REFUSAL_CASES) == set(_reading_refusals())
    reader, payload, fragment = _reading_refusals()[case]
    with monkeypatch.context() as scope:
        _fresh_caches(scope)
        _without_plans(scope)
        legacy = _outcome(lambda: reader.loads(payload))
    _fresh_caches(monkeypatch)
    cold = _outcome(lambda: reader.loads(payload))
    warm = _outcome(lambda: reader.loads(payload))

    assert legacy[:2] == ("refused", "ContractCodecError")
    assert fragment in legacy[2]
    assert cold == legacy
    assert warm == legacy


def test_the_type_hints_of_a_record_class_are_resolved_once_per_process(monkeypatch):
    _fresh_caches(monkeypatch)
    resolved: list[type] = []
    real = codec.get_type_hints

    def counting(annotation):
        resolved.append(annotation)
        return real(annotation)

    monkeypatch.setattr(codec, "get_type_hints", counting)
    raw = LARGE_SNAPSHOT.read_bytes()
    reader = kernel.AnalysisSnapshotCodecV1
    reader.loads(raw)
    classes = len(resolved)
    assert classes > 10
    assert len(set(resolved)) == classes, "a class was resolved twice within one read"
    for _ in range(3):
        reader.loads(raw)
        reader.loads(reader.dumps(reader.loads(raw)))
    assert len(resolved) == classes, "a later read resolved hints again"


def test_plans_are_taken_only_by_classes_that_decode_as_records(monkeypatch):
    """Идентификатор и перечисление идут своими ветками и в плане записей не появляются."""

    _fresh_caches(monkeypatch)
    kernel.AnalysisSnapshotCodecV1.loads(SMALL_SNAPSHOT.read_bytes())
    planned = set(codec._READ_PLANS)
    assert planned, "reading a snapshot must have taken plans"
    assert not any(issubclass(cls, OpaqueId) for cls in planned)
    assert not any(issubclass(cls, Enum) for cls in planned)
    assert kernel.AnalysisSnapshotV1 in planned
