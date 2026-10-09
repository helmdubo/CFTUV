"""Рациональность закона прихода: сохранённые исходы и измеримое исключение общей алгебры.

Оракул ниже — независимая копия прежних двух функций из afc989c, без вызова
нового предиката или переключателя. Сверяются дроби, отказы и исключения,
включая формы, на которых родная арифметика сильнее прежнего доказательства.
"""

from __future__ import annotations

import itertools
import json
import re
from fractions import Fraction
from pathlib import Path

import pytest
import sympy as sp

import cftuv_envelope as kernel
from cftuv_envelope.reference import symbolic_backend
from cftuv_envelope.reference.common import GeometryContext
from cftuv_envelope.reference.native_exact import (
    EXACT_NATIVE_OUTSIDE_FIELD,
    OutsideNativeField,
    RadicalSumV1,
    rational_ratio,
)
from cftuv_envelope.reference.symbolic_backend import (
    DisagreementPolicyV1,
    ExactSymbolicBackendDisagreement,
    SymbolicBackendV1,
)
from cftuv_envelope.reference.validation import (
    validate_reference_geometry_payload,
)
from cftuv_envelope.wavefront import conveyor as conveyor_module

FIXTURES = Path(__file__).resolve().parents[1] / "fixtures"
FIELD_SCENE_CALLS = FIXTURES / "arrival_law_rescale" / "field_scene_calls_v1.json"
#: Фикстуры, чьи законы прихода читает `_arrival_laws`: полевой домен, веса нормалей (иррациональная нормаль), полный
#: выбор (четыре веера), стена с перекладинами и разрезом, выпуклое разбиение, меш с вырезанными веерами.
ARRIVAL_FIXTURES = (
    "building_002_point_contact_v1",
    "building_002_weighted_normals_v1",
    "building_002_full_selection_v1",
    "mesh2_patch0_cut_fans_v1",
    "sagging_wall_convex_partition_v1",
    "wall_noise_top_rung_clip_v1",
)


def oracle(value, scale):
    """afc989c: exact_rational + _rational_after_scaling, без нового кода."""

    candidate_value = sp.sympify(sp.radsimp(value / scale))
    if not candidate_value.is_Rational:
        return None
    candidate = Fraction(int(candidate_value.p), int(candidate_value.q))
    residue = sp.simplify(
        value - sp.Rational(candidate.numerator, candidate.denominator) * scale
    )
    return candidate if residue.is_zero is True else None


def _capture(function, value, scale):
    try:
        return ("answer", function(value, scale))
    except Exception as error:
        return ("exception", type(error), str(error))


S = sp.sqrt
R = sp.Rational


def _fraction(text: str | None) -> Fraction | None:
    if text is None:
        return None
    numerator, denominator = text.split("/")
    return Fraction(int(numerator), int(denominator))


@pytest.fixture(autouse=True)
def _clean_backend_state():
    """Режим, политика и счётчики символьного бэкенда — состояние процесса; тест не оставляет следа."""

    symbolic_backend.reset_backend_counts()
    with symbolic_backend.symbolic_backend(
        symbolic_backend.DEFAULT_BACKEND, DisagreementPolicyV1.RAISE
    ):
        yield
    symbolic_backend.reset_backend_counts()


# --------------------------------------------------------------------------
# 1. Записанные вызовы
# --------------------------------------------------------------------------


def test_the_native_predicate_equals_the_sympy_proof_on_the_calls_recorded_on_the_field_scene():
    """Записанные на сцене `building` аргументы: ответ родного предиката равен записанному и живому оракулу."""

    payload = json.loads(FIELD_SCENE_CALLS.read_text(encoding="utf-8"))
    calls = payload["calls"]
    mismatches = []
    answers = set()
    for value_text, scale_text, recorded_text in calls:
        value, scale = sp.sympify(value_text), sp.sympify(scale_text)
        native = rational_ratio(value, scale)
        recorded = _fraction(recorded_text)
        if not (native == recorded == oracle(value, scale) == conveyor_module._rational_after_scaling(value, scale)):
            mismatches.append((value_text, scale_text, recorded, native))
        assert native is not None
        answers.add((native > 0) - (native < 0))
    assert mismatches == []
    assert symbolic_backend.BACKEND_COUNTS == {"arrival_law_rescale.native": len(calls)}
    # Положительные контроли: выборка не выродилась в один знак ответа и не пуста по рациональным и одночленным.
    assert len(calls) >= 90
    assert answers == {-1, 0, 1}
    assert any("Pow(" not in value and "Pow(" not in scale for value, scale, _ in calls)
    assert any("Pow(" in value and "Pow(" in scale for value, scale, _ in calls)
    # Большие радиканды поля (нормы метрики карты) в выборке есть: именно на них факторизация стоила бы дорого.
    assert any(
        int(number) >= 10**9
        for value, scale, _ in calls
        for number in re.findall(r"Integer\((\d+)\)", value + scale)
    )


@pytest.fixture(scope="module")
def fixture_calls():
    """Все вызовы `_rational_after_scaling`, которые делает `_arrival_laws` на фикстурах ядра."""

    calls = []
    original = conveyor_module._rational_after_scaling

    def recorder(value, scale):
        calls.append((value, scale))
        return original(value, scale)

    conveyor_module._rational_after_scaling = recorder
    try:
        for name in ARRIVAL_FIXTURES:
            root = FIXTURES / name
            snapshot = kernel.AnalysisSnapshotCodecV1.loads(
                (root / "analysis_snapshot.json").read_bytes()
            )
            request = kernel.DecalRequestCodecV1.loads(
                (root / "decal_request.json").read_bytes()
            )
            compiled = kernel.compile_reference_envelopes(snapshot, request)
            frame, _ = validate_reference_geometry_payload(
                compiled.compilation.analysis_snapshot,
                compiled.compilation.plan_key.patch_domain_id,
            )
            context = GeometryContext.build(compiled.compilation, frame)
            reading = conveyor_module._arrival_laws(context)
            assert reading.detail is None, (name, reading.detail)
    finally:
        conveyor_module._rational_after_scaling = original
    return calls


def test_the_native_predicate_equals_the_sympy_proof_on_every_call_of_the_kernel_fixtures(
    fixture_calls,
):
    # Положительный контроль: харвест не пуст, иначе «ни одного расхождения» ничего бы не значило.
    assert len(fixture_calls) >= 300
    mismatches = [
        (sp.srepr(value), sp.srepr(scale))
        for value, scale in fixture_calls
        if rational_ratio(value, scale) != oracle(value, scale)
    ]
    assert mismatches == []


def test_every_fixture_call_is_decided_natively_and_the_sympy_proof_is_not_consulted(
    fixture_calls, monkeypatch
):
    """Решает родной предикат, и уступок sympy на вызовах фикстур нет: `outside_field` не бывает."""

    def forbidden(*args):
        raise AssertionError("the sympy proof was consulted on an in-field call")

    monkeypatch.setattr(conveyor_module, "_rational_after_scaling_sympy", forbidden)
    for value, scale in fixture_calls:
        conveyor_module._rational_after_scaling(value, scale)
    counts = symbolic_backend.BACKEND_COUNTS
    assert counts == {"arrival_law_rescale.native": len(fixture_calls)}


# --------------------------------------------------------------------------
# 2. Сетка выражений: рациональные, нуль, отрицательные, «вложенно выглядящие»
# --------------------------------------------------------------------------

IN_FIELD = (
    sp.Integer(0),
    sp.Integer(1),
    sp.Integer(-1),
    sp.Integer(2),
    R(1, 2),
    R(-3, 7),
    R(9, 4),
    S(2),
    -S(2),
    S(3),
    S(6),
    S(8),
    S(12),
    S(18),
    S(75),
    S(R(1, 2)),
    S(R(2, 3)),
    3 * S(2),
    R(1, 4) * S(3),
    -R(5, 7) * S(6),
    S(426753013) / 8192,
    -S(426753013) / 8192,
    S(1420008490) / 16384,
    1 + S(2),
    1 - S(2),
    S(2) + S(3),
    S(2) - S(3),
    S(2) + S(3) + S(5),
    S(2) * S(3),
    2 * S(2) + 3 * S(8),
    S(12) + S(27),
    S(3) + S(12),
    # Вложенно выглядящие, но в поле: произведения и степени сумм, которые sympy не раскрывает сам.
    (1 + S(2)) * (1 - S(2)),
    (S(2) + S(3)) * (S(2) - S(3)),
    (S(2) + S(3)) ** 2,
    (1 + S(2)) ** 2,
    (1 + S(2)) ** -2,
    1 / (1 + S(2)),
    1 / (S(2) + S(3)),
    sp.Pow(1 + S(2), 3),
    sp.Mul(1 + S(2), 1 - S(2), evaluate=False),
)

#: Делитель из четырёх квадратных классов: сумма читается в поле, но обратить её сопряжением родная арифметика не
#: берётся (`_INVERSE_CLASS_LIMIT`), и это названный отказ, а не `None`.
FOUR_CLASSES = 1 + S(2) + S(3) + S(5)

#: Вне поля `RadicalSumV1`: корень из суммы, четвёртый корень, трансцендентные.
OUTSIDE_FIELD = (
    S(3 + 2 * S(2)),
    S(5 + 2 * S(6)),
    S(2 + S(3)),
    S(10 - 2 * S(5)),
    sp.Pow(2, R(1, 4)),
    sp.pi,
    sp.sin(1),
)


def test_the_conveyor_matches_parent_including_refusals_on_the_expression_grid():
    scales = tuple(item for item in IN_FIELD if item != 0)
    fractions = 0
    refusals = 0
    for value, scale in itertools.product(IN_FIELD, scales):
        expected = _capture(oracle, value, scale)
        actual = _capture(conveyor_module._rational_after_scaling, value, scale)
        assert actual == expected, (sp.srepr(value), sp.srepr(scale), actual, expected)
        fractions += expected[0] == "answer" and expected[1] is not None
        refusals += expected == ("answer", None)
    assert fractions >= 200
    assert refusals >= 100
    assert symbolic_backend.BACKEND_COUNTS["arrival_law_rescale.native"] > 0
    assert symbolic_backend.BACKEND_COUNTS["arrival_law_rescale.legacy_shape"] > 0


def test_a_stronger_native_proof_must_not_expand_the_conveyor_admitted_set():
    """Положительный контроль независимого знания: `(1-sqrt2)(1+sqrt2) == -1`, а не «похоже на».

    Это не зависит от версии sympy: ответ проверен вычислением в поле.
    """

    product = sp.Mul(1 - S(2), 1 + S(2), evaluate=False)
    assert oracle(product, sp.Integer(1)) is None
    assert oracle(product, sp.Integer(-2)) is None
    assert rational_ratio(product, sp.Integer(1)) == Fraction(-1)
    assert rational_ratio(product, sp.Integer(-2)) == Fraction(1, 2)
    assert (RadicalSumV1.rational(1) - RadicalSumV1.sqrt_of_rational(2)) * (
        RadicalSumV1.rational(1) + RadicalSumV1.sqrt_of_rational(2)
    ) == RadicalSumV1.rational(-1)
    for mode in SymbolicBackendV1:
        with symbolic_backend.symbolic_backend(mode, DisagreementPolicyV1.RAISE):
            assert conveyor_module._rational_after_scaling(product, sp.Integer(1)) is None
            assert conveyor_module._rational_after_scaling(product, sp.Integer(-2)) is None
    assert symbolic_backend.DISAGREEMENTS == []


@pytest.mark.parametrize(
    "value,scale,expected",
    [
        (sp.Integer(0), S(426753013) / 8192, Fraction(0)),
        (-S(426753013) / 8192, S(426753013) / 8192, Fraction(-1)),
        (S(426753013) / 8192, -S(426753013) / 8192, Fraction(-1)),
        (sp.Integer(1), (S(426753013) / 8192) ** 2, Fraction(67108864, 426753013)),
        (3 * S(2), S(8), Fraction(3, 2)),
        (S(2) * S(3), S(6), Fraction(1)),
        (S(12), S(3), Fraction(2)),
        (S(2), S(426753013) / 8192, None),
        (S(426753013) / 8192 + 1, S(426753013) / 8192, None),
        (S(2) + S(3), S(2), None),
        (sp.Integer(1), S(2), None),
        # Знаменатель нуль: частного нет, и оба пути отвечают `None`, а не исключением.
        (sp.Integer(1), sp.Integer(0), None),
        (sp.Integer(0), sp.Integer(0), None),
    ],
)
@pytest.mark.parametrize("mode", list(SymbolicBackendV1))
def test_the_named_cases_have_one_answer_in_every_backend_mode(value, scale, expected, mode):
    with symbolic_backend.symbolic_backend(mode, DisagreementPolicyV1.RAISE):
        assert conveyor_module._rational_after_scaling(value, scale) == expected


def test_a_divisor_of_four_square_classes_keeps_the_legacy_answer_without_native_inversion():
    """Предел обращения назван: частное `x / x` на таком делителе sympy сокращает само, родной предикат уступает."""

    with pytest.raises(OutsideNativeField):
        rational_ratio(sp.Integer(1), FOUR_CLASSES)
    assert rational_ratio(FOUR_CLASSES, sp.Integer(2)) == oracle(FOUR_CLASSES, sp.Integer(2)) is None
    assert conveyor_module._rational_after_scaling(FOUR_CLASSES, FOUR_CLASSES) == Fraction(1)
    assert symbolic_backend.BACKEND_COUNTS == {"arrival_law_rescale.legacy_shape": 1}


def test_an_expression_outside_the_field_is_a_named_refusal_not_a_none():
    for value in OUTSIDE_FIELD:
        with pytest.raises(OutsideNativeField) as caught:
            rational_ratio(value, sp.Integer(1))
        assert caught.value.code == EXACT_NATIVE_OUTSIDE_FIELD
        with pytest.raises(OutsideNativeField):
            rational_ratio(sp.Integer(1), value)


def test_outside_the_fast_shape_the_oracle_answers_and_the_yield_is_counted():
    """Уступка названа счётом и возвращает ТОТ ЖЕ ответ, что дал бы режим `SYMPY`."""

    scale = S(2) + S(3)
    value = S(5 + 2 * S(6))
    expected = oracle(value, scale)
    assert conveyor_module._rational_after_scaling(value, scale) == expected
    assert symbolic_backend.BACKEND_COUNTS == {"arrival_law_rescale.legacy_shape": 1}
    # Тот же вопрос в режиме SYMPY не считает ничего: счёт описывает именно уступку родного пути.
    symbolic_backend.reset_backend_counts()
    with symbolic_backend.symbolic_backend(SymbolicBackendV1.SYMPY):
        assert conveyor_module._rational_after_scaling(value, scale) == expected
    assert symbolic_backend.BACKEND_COUNTS == {}


# --------------------------------------------------------------------------
# 3. Переключатель: что считает какой режим
# --------------------------------------------------------------------------


def test_each_backend_mode_computes_exactly_the_paths_it_names(monkeypatch):
    value, scale = -S(426753013) / 8192, S(426753013) / 8192
    native_calls, oracle_calls = [], []
    native_original = conveyor_module.rational_ratio
    oracle_original = conveyor_module._rational_after_scaling_sympy

    def counting_native(*args):
        native_calls.append(args)
        return native_original(*args)

    def counting_oracle(*args):
        oracle_calls.append(args)
        return oracle_original(*args)

    monkeypatch.setattr(conveyor_module, "rational_ratio", counting_native)
    monkeypatch.setattr(conveyor_module, "_rational_after_scaling_sympy", counting_oracle)
    expectations = {
        SymbolicBackendV1.SYMPY: (0, 1),
        SymbolicBackendV1.NATIVE_EXACT: (1, 0),
        SymbolicBackendV1.SHADOW: (1, 1),
    }
    for mode, (native_expected, oracle_expected) in expectations.items():
        native_calls.clear()
        oracle_calls.clear()
        with symbolic_backend.symbolic_backend(mode):
            assert conveyor_module._rational_after_scaling(value, scale) == Fraction(-1)
        assert (len(native_calls), len(oracle_calls)) == (native_expected, oracle_expected), mode
    assert symbolic_backend.BACKEND_COUNTS == {
        "arrival_law_rescale.native": 1,
        "arrival_law_rescale.shadow_checked": 1,
    }


def test_the_shadow_mode_names_a_disagreement_and_returns_the_sympy_answer(monkeypatch):
    value, scale = -S(426753013) / 8192, S(426753013) / 8192
    monkeypatch.setattr(conveyor_module, "rational_ratio", lambda *_: Fraction(7))
    with symbolic_backend.symbolic_backend(SymbolicBackendV1.SHADOW, DisagreementPolicyV1.RAISE):
        with pytest.raises(ExactSymbolicBackendDisagreement) as caught:
            conveyor_module._rational_after_scaling(value, scale)
    assert caught.value.site == "arrival_law_rescale"
    with symbolic_backend.symbolic_backend(SymbolicBackendV1.SHADOW, DisagreementPolicyV1.RECORD):
        assert conveyor_module._rational_after_scaling(value, scale) == Fraction(-1)
    assert [site for site, _ in symbolic_backend.DISAGREEMENTS] == [
        "arrival_law_rescale",
        "arrival_law_rescale",
    ]


# --------------------------------------------------------------------------
# 4. sympy-доказательство ушло с пути законов прихода
# --------------------------------------------------------------------------


@pytest.mark.parametrize(
    "name,rescaled",
    [
        ("building_002_point_contact_v1", 5),
        ("building_002_weighted_normals_v1", 2),
        ("building_002_full_selection_v1", 16),
    ],
)
def test_the_arrival_laws_are_read_without_radsimp_and_simplify(name, rescaled, monkeypatch):
    """Законы прихода и веера читаются, а `radsimp` и `simplify` не вызываются ни разу.

    Отрицательный контроль встроен: те же фикстуры под режимом `SYMPY` их вызывают (счёт растёт), иначе «ни разу» не
    отличить от «подмена не работает».
    """

    calls = {"radsimp": 0, "simplify": 0}
    for attribute in calls:
        original = getattr(sp, attribute)

        def counting(*args, __original=original, __name=attribute, **kwargs):
            calls[__name] += 1
            return __original(*args, **kwargs)

        monkeypatch.setattr(sp, attribute, counting)

    root = FIXTURES / name
    snapshot = kernel.AnalysisSnapshotCodecV1.loads((root / "analysis_snapshot.json").read_bytes())
    request = kernel.DecalRequestCodecV1.loads((root / "decal_request.json").read_bytes())
    compiled = kernel.compile_reference_envelopes(snapshot, request)
    frame, _ = validate_reference_geometry_payload(
        compiled.compilation.analysis_snapshot, compiled.compilation.plan_key.patch_domain_id
    )

    context = GeometryContext.build(compiled.compilation, frame)

    def read(mode):
        with symbolic_backend.symbolic_backend(mode):
            return conveyor_module._arrival_laws(context)

    native_reading = read(SymbolicBackendV1.NATIVE_EXACT)
    assert calls == {"radsimp": 0, "simplify": 0}
    assert native_reading.detail is None
    assert native_reading.rescaled_count == rescaled
    sympy_reading = read(SymbolicBackendV1.SYMPY)
    assert calls["radsimp"] > 0 and calls["simplify"] > 0
    # Тот же ответ: законы, веера и деградировавшие углы равны побитово.
    assert native_reading == sympy_reading


@pytest.mark.parametrize("mode", list(SymbolicBackendV1))
@pytest.mark.parametrize(
    "value,scale",
    [
        (1, 0), (0, 0), (1, 2), (1.0, 2.0), (Fraction(1), Fraction(2)),
        (None, sp.Integer(1)), (sp.Integer(1), None),
        (sp.Float(1), sp.Integer(2)), (sp.oo, sp.Integer(1)),
        (sp.I, sp.Integer(1)), (sp.nan, sp.Integer(1)),
        (S(2) + sp.Rational(1, 2**200), S(2)),
        (S(2) - sp.Rational(1, 2**200), S(2)),
        (sp.Add(S(2), -S(2), evaluate=False), sp.Integer(1)),
        (sp.Pow(4, R(1, 2), evaluate=False), sp.Integer(2)),
        (sp.Mul(R(1, 2), S(8), evaluate=False), S(2)),
        (sp.Integer(1), sp.Add(S(2), -S(2), evaluate=False)),
        (sp.Symbol("x"), sp.Symbol("x")),
        (S(3 + 2 * S(2)), 1 + S(2)),
    ],
)
def test_error_types_text_and_partial_canonicalization_equal_the_parent(value, scale, mode):
    expected = _capture(oracle, value, scale)
    with symbolic_backend.symbolic_backend(mode, DisagreementPolicyV1.RAISE):
        assert _capture(conveyor_module._rational_after_scaling, value, scale) == expected
    assert symbolic_backend.DISAGREEMENTS == []


def test_complex_shapes_do_not_expand_or_invert_for_the_native_attempt(monkeypatch):
    def forbidden(*args):
        raise AssertionError("native arithmetic visited a legacy-only shape")

    monkeypatch.setattr(conveyor_module, "rational_ratio", forbidden)
    cases = (
        (FOUR_CLASSES, FOUR_CLASSES),
        ((1 + S(2)) ** 10000, (1 + S(2)) ** 10000),
        (sp.Mul(1 - S(2), 1 + S(2), evaluate=False), sp.Integer(1)),
    )
    for value, scale in cases:
        assert conveyor_module._rational_after_scaling(value, scale) == oracle(value, scale)
    assert symbolic_backend.BACKEND_COUNTS == {"arrival_law_rescale.legacy_shape": len(cases)}


def test_a_native_field_refusal_keeps_the_named_fallback(monkeypatch):
    def refuse(*args):
        raise OutsideNativeField("injected field refusal")

    monkeypatch.setattr(conveyor_module, "rational_ratio", refuse)
    for mode in (SymbolicBackendV1.NATIVE_EXACT, SymbolicBackendV1.SHADOW):
        with symbolic_backend.symbolic_backend(mode):
            assert conveyor_module._rational_after_scaling(-S(2), S(2)) == oracle(-S(2), S(2))
    assert symbolic_backend.BACKEND_COUNTS == {"arrival_law_rescale.outside_field": 2}


def test_an_unexpanded_constant_preserves_the_arrival_law_refusal(monkeypatch):
    from types import SimpleNamespace

    product = sp.Mul(1 - S(2), 1 + S(2), evaluate=False)
    normal = SimpleNamespace(expressions=lambda: (S(2), sp.Integer(0)))
    constant = SimpleNamespace(as_expr=lambda: product * S(2))
    actual = conveyor_module._read_arrival_law("unexpanded", normal, constant, sp.Integer(2))
    with monkeypatch.context() as old:
        old.setattr(conveyor_module, "_rational_after_scaling", oracle)
        expected = conveyor_module._read_arrival_law("unexpanded", normal, constant, sp.Integer(2))
    assert actual == expected == (None, False, "запись закона не рациональна и после масштаба")


@pytest.mark.parametrize(
    "value,scale,expected",
    [
        (sp.Integer(0), S(2), Fraction(0)),
        (sp.Integer(0), sp.Integer(0), None),
        (sp.Integer(1), sp.Integer(0), None),
        (sp.Integer(-2), sp.Integer(-3), Fraction(2, 3)),
        (3 * S(2), S(8), Fraction(3, 2)),
        (S(12), -2 * S(3), Fraction(-1)),
        (S(2), S(3), None),
        (R(-3, 7) * S(2), R(9, 5) * S(18), Fraction(-5, 63)),
        (S(10**40 + 1), S(4 * (10**40 + 1)), Fraction(1, 2)),
        (S(10**40 + 1), S(10**40 + 2), None),
        (S(2) + R(1, 2**200), S(2), None),
        (sp.Mul(1 - S(2), 1 + S(2), evaluate=False), sp.Integer(1), Fraction(-1)),
    ],
)
def test_native_ratio_exact_classes_without_numerical_tolerance(value, scale, expected):
    assert rational_ratio(value, scale) == expected


def test_zero_does_not_bypass_the_native_denominator_class_limit():
    with pytest.raises(OutsideNativeField, match="too many square classes to invert"):
        rational_ratio(sp.Integer(0), FOUR_CLASSES)
