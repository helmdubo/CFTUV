"""Память скорости кнопки при сдвиге alpha: ответ побитово тот же, меняется только цена.

Срез PERF MATERIALIZE-SPEED нашёл в тёплом нажатии работу, которая от alpha не зависит либо считается дважды:

1. дайджест батча считали при сборке и ещё раз при проверке; теперь повтор с ТЕМИ ЖЕ частями возвращает прежнее значение,
   а любая подменённая часть (равная, но другой объект) даёт честный пересчёт;
2. покрытие региона считают ОДИН раз (площади и контуры одним вызовом в `conveyor_coverage`): контуры едут в записи региона
   (`ConveyorRegionCoverageV1.contours`) к материализатору, а процессной памяти покрытий нет (цена не зависит от истории);
3. оболочки `alpha` контактов и вселенная простых `q` считаются на первом покрытии и едут с подготовкой: сравнение
   `alpha` с запрошенной идёт по оболочке, разложения `q` возвращаются в память канонизации без единой факторизации;
4. каноническая кодировка (ключи сортировки, имена полей) быстрее, а байты те же, что у прежней реализации;
5. замечания к снапшоту, посчитанные вызывающим, дают тот же итог проверки.

Тест не принимает на веру «похоже на правду»: каждая память сравнивается с вычислением БЕЗ неё.
"""

from __future__ import annotations

import json
import pickle
from dataclasses import fields, replace
from decimal import Decimal
from enum import Enum
from fractions import Fraction

import pytest
import sympy as sp

import cftuv_envelope as kernel
import cftuv_envelope.canonical as canonical
import cftuv_envelope.codec as codec
import cftuv_envelope.exact_sqrt_sum as exact
import cftuv_envelope.wavefront.coverage as coverage_module
from cftuv_envelope.contracts.geometry_batch import GeometryBatchV1
from cftuv_envelope.ids import OpaqueId, SemanticDigestValue
from cftuv_envelope.reference.alpha_bounds import alpha_bounds, sign_against
from cftuv_envelope.reference.planar_types import exact_sign
from cftuv_envelope.wavefront import conveyor_coverage, prepare_conveyor
from cftuv_envelope.wavefront.coverage import CoverageOutcome, coverage_at

from factories import geometry_batch
from wavefront_cases import FIELD_FIXTURE


def _field_inputs():
    root = FIELD_FIXTURE.parent
    snapshot = kernel.AnalysisSnapshotCodecV1.loads((root / "analysis_snapshot.json").read_bytes())
    request = kernel.DecalRequestCodecV1.loads((root / "decal_request.json").read_bytes())
    return snapshot, request


def _field_preparation():
    return prepare_conveyor(*_field_inputs())


def _answer(result):
    return (
        result.outcome,
        result.alpha,
        result.lattice_alpha,
        result.doubled_area,
        result.counters,
        tuple(
            (
                region.region_id,
                region.outcome,
                region.doubled_area,
                tuple(
                    (face.owner, face.envelope_spec_id, face.envelope_instance_id, face.doubled_area)
                    for face in region.faces
                ),
            )
            for region in result.regions
        ),
    )


# ---------------------------------------------------------------- 1. дайджест батча


def test_the_semantic_digest_is_computed_once_for_the_same_parts_and_again_for_any_replaced_part(monkeypatch):
    batch = geometry_batch()
    calls = []
    real = canonical._compute_semantic_digest
    monkeypatch.setattr(canonical, "_compute_semantic_digest", lambda item: calls.append(1) or real(item))
    canonical._LAST_DIGEST = None

    first = canonical.geometry_batch_semantic_digest(batch)
    # `replace` меняет только поле дайджеста: части те же объекты — тот же ответ без пересчёта.
    stamped = replace(batch, semantic_digest=SemanticDigestValue(first.sha256_hex))
    assert canonical.geometry_batch_semantic_digest(stamped) == first
    assert len(calls) == 1
    # Равный, но другой кортеж/множество — другой объект: честный пересчёт, тот же ответ.
    rebuilt = replace(stamped, vertices=frozenset(set(stamped.vertices)), faces=tuple(list(stamped.faces)))
    assert canonical.geometry_batch_semantic_digest(rebuilt) == first
    assert len(calls) == 2
    # Другое содержимое — другой дайджест, память его не прячет.
    changed = replace(stamped, faces=stamped.faces[:-1])
    assert canonical.geometry_batch_semantic_digest(changed) != first
    assert len(calls) == 3


def test_the_digest_memo_key_covers_every_batch_field_except_the_digest_itself():
    names = {item.name for item in fields(GeometryBatchV1)}
    assert set(canonical._KEY_FIELDS) == names - {"semantic_digest"}


def test_the_validator_still_sees_a_wrong_declared_digest_after_the_digest_was_remembered():
    batch = geometry_batch()
    good = replace(batch, semantic_digest=SemanticDigestValue(canonical.geometry_batch_semantic_digest(batch).sha256_hex))
    assert kernel.validate_geometry_batch(good) == ()
    wrong = replace(good, semantic_digest=SemanticDigestValue("0" * 64))
    assert [item.path for item in kernel.validate_geometry_batch(wrong)] == [("semantic_digest",)]


# ---------------------------------------------------------------- 2. покрытие региона


def test_coverage_at_keeps_no_memory_between_calls_and_every_call_carries_its_own_budget(monkeypatch):
    prepared = _field_preparation()
    partition = prepared.regions[0].partition
    calls = []
    real = coverage_module._coverage_at
    monkeypatch.setattr(coverage_module, "_coverage_at", lambda *args: calls.append(1) or real(*args))

    first_budget, second_budget = exact.exact_work_budget(stage="A"), exact.exact_work_budget(stage="B")
    first = coverage_at(partition, Fraction(1, 4), first_budget)
    second = coverage_at(partition, Fraction(1, 4), second_budget)
    assert len(calls) == 2, "the second coverage of the same partition and alpha is computed, not remembered"
    assert second.faces == first.faces and second.doubled_area == first.doubled_area
    assert first.work_budget is first_budget and second.work_budget is second_budget
    assert not hasattr(coverage_module, "_RECENT") and not hasattr(coverage_module, "clear_recent_coverage")
    held = [name for name, value in vars(coverage_module).items() if not name.startswith("__") and isinstance(value, (dict, list, set))]
    assert held == [], f"the coverage module keeps no process memory between evaluations: {held}"


def test_a_refused_coverage_is_an_outcome_not_a_value():
    prepared = _field_preparation()
    partition = prepared.regions[0].partition
    refused = coverage_at(partition, Fraction(-1))
    assert refused.outcome is CoverageOutcome.ALPHA_IS_NEGATIVE


# ---------------------------------------------------------------- 3. память подготовки


ALPHAS = ("0.25", "0.5", "2", "7")


def test_alpha_bounds_decide_exactly_what_the_exact_sign_decides():
    candidates = [
        sp.Rational(3, 7),
        sp.Rational(5, 2),
        sp.sqrt(2),
        sp.sqrt(sp.Rational(5, 3)) * sp.Rational(2, 5) + sp.Rational(3, 7),
        sp.sqrt(3) - sp.sqrt(2),
        sp.Integer(0),
        sp.sqrt(5) + sp.sqrt(7) * sp.Rational(1, 3),
    ]
    requested_values = [sp.Rational(text) for text in ("0", "0.25", "1", "1.5", "2", "2.5", "4.5")]
    requested_values += [sp.Rational(3, 7), sp.Rational(5, 2)]  # точные совпадения: оболочка молчит, решает точный путь
    for alpha in candidates:
        bounds = alpha_bounds(alpha)
        assert bounds is not None and bounds[0] <= bounds[1]
        for requested in requested_values:
            assert sign_against(alpha, bounds, requested) == exact_sign(alpha - requested), (alpha, requested)
            assert sign_against(alpha, None, requested) == exact_sign(alpha - requested)


def test_the_bounds_and_the_universe_travel_with_the_preparation_and_the_copy_factors_nothing(monkeypatch):
    prepared = _field_preparation()
    bare = replace(prepared, contact_memo=None)
    cold = {alpha: _answer(conveyor_coverage(prepared, alpha)) for alpha in ALPHAS}
    memo = prepared.contact_memo
    kinds = {key[0] for key in memo.entries}
    assert {"contacts", "bounds", "blocking", "prime-universe"} <= kinds

    clone = pickle.loads(pickle.dumps(prepared, protocol=pickle.HIGHEST_PROTOCOL))
    # Новый процесс воркера: память канонизации пуста, и разложений в ней нет.
    exact.reset_factorization_memory()
    factored = []
    real = exact._rho_factors
    monkeypatch.setattr(exact, "_rho_factors", lambda number, budget: factored.append(number) or real(number, budget))
    for alpha in ALPHAS:
        assert _answer(conveyor_coverage(clone, alpha)) == cold[alpha], alpha
    assert factored == []
    # Без памяти (подготовка без `contact_memo`) ответ тот же — это не память решает дело.
    exact.reset_factorization_memory()
    for alpha in ALPHAS:
        assert _answer(conveyor_coverage(bare, alpha)) == cold[alpha], alpha
    assert clone.contact_memo.computed == 0


def test_the_replay_respects_the_factorization_memory_limit(monkeypatch):
    q_values = (Fraction(3), Fraction(5, 3), Fraction(12), Fraction(7, 5))
    store = {}
    exact.prime_universe_remembered(q_values, None, store)
    delta = store[next(iter(store))][1]
    assert len(delta) >= 3
    exact.reset_factorization_memory()
    monkeypatch.setattr(exact, "_FACTORIZATION_MEMO_ENTRIES", 2)
    exact.prime_universe_remembered(q_values, None, store)
    assert len(exact._FACTORIZATION_MEMO) <= 2
    # Вытеснение — самое давнее, как в `_factorization_pairs`: последняя запись дельты на месте.
    assert delta[-1][0] in exact._FACTORIZATION_MEMO


def test_the_remembered_universe_equals_the_computed_one_and_a_failure_is_not_stored():
    q_values = (Fraction(3), Fraction(5, 3), Fraction(12), Fraction(7, 5))
    plain = exact._prime_universe_from_q_values(q_values)
    store = {}
    assert exact.prime_universe_remembered(q_values, None, store) == plain
    assert len(store) == 1
    exact.reset_factorization_memory()
    assert exact.prime_universe_remembered(q_values, None, store) == plain
    # Простые вселенной, раскладывающиеся сами в себя, лежат в памяти канонизации.
    for prime in plain:
        assert exact._FACTORIZATION_MEMO[prime] == ((prime, 1),)
    with pytest.raises(exact.NegativeRadicandError):
        exact.prime_universe_remembered((Fraction(-1),), None, store)
    assert len(store) == 1


# ---------------------------------------------------------------- 4. кодировка


def _reference_data(value):
    """Прежняя реализация `to_canonical_data` (до ускорения): ответ быстрой обязан совпасть с ней побитово."""

    from dataclasses import fields as dataclass_fields, is_dataclass

    from cftuv_envelope.schema import is_wire_default_field

    if isinstance(value, OpaqueId):
        return {"$id_type": type(value).__name__, "value": value.value}
    if isinstance(value, Enum):
        return value.value
    if isinstance(value, Decimal):
        return codec._canonical_decimal(value)
    if is_dataclass(value) and not isinstance(value, type):
        result = {"$type": type(value).__name__}
        for item in dataclass_fields(value):
            held = getattr(value, item.name)
            if is_wire_default_field(item) and held == item.default:
                continue
            result[item.name] = _reference_data(held)
        return result
    if isinstance(value, tuple):
        return [_reference_data(item) for item in value]
    if isinstance(value, frozenset):
        encoded = [_reference_data(item) for item in value]
        return sorted(
            encoded,
            key=lambda item: json.dumps(
                item, ensure_ascii=False, allow_nan=False, sort_keys=True, separators=(",", ":")
            ).encode("utf-8"),
        )
    if isinstance(value, dict):
        return {key: _reference_data(item) for key, item in value.items()}
    return value


def _reference_bytes(value) -> bytes:
    return json.dumps(
        _reference_data(value), ensure_ascii=False, allow_nan=False, sort_keys=True, separators=(",", ":")
    ).encode("utf-8")


def test_the_canonical_bytes_equal_the_reference_implementation_on_real_records():
    snapshot, request = _field_inputs()
    # Поле, опускаемое на проводе, пока равно умолчанию (`wire_default_field`): запрос с умолчанием и с названным допуском.
    wide = replace(request, developable_stretch_budget=kernel.ExactRationalV1(7, 20))
    objects = [geometry_batch(), geometry_batch(alternate_diagonal=True), snapshot, request, wide]
    for item in objects:
        assert codec.canonical_json_bytes(item) == _reference_bytes(item), type(item).__name__
    assert b"developable_stretch_budget" not in codec.canonical_json_bytes(request)
    assert b"developable_stretch_budget" in codec.canonical_json_bytes(wide)


def test_the_canonical_bytes_equal_the_reference_on_nested_sets_unicode_floats_and_decimals():
    value = {
        "b": frozenset({(1, "ä"), (2, "ж"), (10, "z")}),
        "a": (Decimal("1.50"), Decimal("0"), 3.25, None, True, "tab\tq\"\\"),
        "c": frozenset({frozenset({1, 2}), frozenset({3})}),
        "d": {"y": (), "x": frozenset()},
    }
    assert codec.canonical_json_bytes(value) == _reference_bytes(value)


def test_the_text_encoder_falls_back_to_the_standard_one_with_the_same_bytes(monkeypatch):
    batch = geometry_batch()
    expected = codec.canonical_json_bytes(batch)
    monkeypatch.setattr(codec, "_encode_text", codec._ENCODER.encode)
    assert codec.canonical_json_bytes(batch) == expected


# ---------------------------------------------------------------- 5. замечания к снапшоту


def test_precomputed_snapshot_issues_give_the_same_verdict_as_the_default_path():
    snapshot, request = _field_inputs()
    issues = kernel.validate_analysis_snapshot(snapshot)
    default = kernel.validate_snapshot_request_references(snapshot, request)
    assert kernel.validate_snapshot_request_references(snapshot, request, issues) == default
    # Параметр действительно читается: переданное замечание попадает в итог, а снапшот заново не проверяется.
    sentinel = kernel.ValidationIssue(kernel.ValidationCode.CAPABILITY, ("sentinel",), "precomputed")
    verdict = kernel.validate_snapshot_request_references(snapshot, request, (sentinel,))
    assert verdict[0] is sentinel and len(verdict) == len(default) + 1
