"""V2: история факторизации не меняет имена; геометрические координаты сохраняют свой канон."""

from decimal import Decimal
from fractions import Fraction
import json
import os
from pathlib import Path
import subprocess
import sys

import pytest
import sympy as sp

from cftuv_envelope.numeric import LocalLengthV1
from cftuv_envelope.reference import native_exact as nx, symbolic_backend as sb
from cftuv_envelope.reference.boundary import _contact_candidates, resolve_component_alphas
from cftuv_envelope.reference.planar_types import ExactScalar, point_key
from cftuv_envelope.reference.strip import strip_envelope_instance_id

from reference_factories import straight_snapshot
from test_contact_candidates_memo import CONCAVE, _geometry, _sources


def _run_kernel_child(code):
    # Источник уже загруженного ядра, а не соседний с тестами checkout/src.
    package_root = Path(nx.__file__).resolve().parents[1]
    env = dict(os.environ, PYTHONPATH=os.pathsep.join((str(package_root.parent), *sys.path)))
    flags = ["-I"] if sys.flags.isolated or os.environ.get("CFTUV_TEST_REQUIRE_WHEEL") == "1" else []
    guard = """
import sys
from pathlib import Path
import cftuv_envelope

def check_parent_kernel_origin():
    expected = Path(sys.argv[1]).resolve()
    for name, module in tuple(sys.modules.items()):
        if name != "cftuv_envelope" and not name.startswith("cftuv_envelope."):
            continue
        paths = (getattr(module, "__file__", None),
                 getattr(getattr(module, "__spec__", None), "origin", None))
        assert all(paths), f"KERNEL_CHILD_ORIGIN_INVALID: {name}: missing file/spec origin"
        for raw in paths:
            path = Path(raw).resolve()
            assert path.is_file() and path.is_relative_to(expected), (
                f"KERNEL_CHILD_ORIGIN_INVALID: {name}: {raw}; expected {expected}"
            )

check_parent_kernel_origin()
"""
    return subprocess.run(
        [sys.executable, *flags, "-c", guard + code + "\ncheck_parent_kernel_origin()", str(package_root)],
        env=env, check=True, capture_output=True, text=True,
    )


def test_child_keeps_the_parent_kernel_origin():
    run = _run_kernel_child("print(Path(cftuv_envelope.__file__).resolve().parent)")
    assert Path(run.stdout.strip()) == Path(nx.__file__).resolve().parents[1]


@pytest.mark.parametrize("attribute", ("__file__", "__spec__.origin"))
def test_child_rejects_a_foreign_kernel_origin(tmp_path, attribute):
    foreign = tmp_path / "checkout" / "src" / "cftuv_envelope" / "__init__.py"
    foreign.parent.mkdir(parents=True)
    foreign.write_text("# Другой источник ядра.\n", encoding="utf-8")
    # Проверяется настоящий child; parent и второй канал происхождения остаются прежними.
    code = f"cftuv_envelope.{attribute} = {str(foreign)!r}"
    with pytest.raises(subprocess.CalledProcessError) as caught:
        _run_kernel_child(code)
    assert "KERNEL_CHILD_ORIGIN_INVALID" in caught.value.stderr
    assert str(foreign) in caught.value.stderr


def test_factor_cache_history_changes_the_old_control_but_never_v2():
    # Отдельный процесс: отрицательный контроль не оставляет прогретый кэш другим тестам.
    code = """
import json
import sympy as sp
from sympy.core.cache import clear_cache
from sympy.ntheory.factor_ import factor_cache
from cftuv_envelope.reference.native_exact import RadicalSumV1, canonical_text
n = 65537**2 * 100003
factor_cache.clear(); clear_cache()
cold = sp.sqrt(n)
a = sp.srepr(cold)
first = canonical_text(RadicalSumV1.sqrt_of_rational(n))
sp.factorint(n); clear_cache()
b = sp.srepr(sp.sqrt(n))
second = canonical_text(RadicalSumV1.sqrt_of_rational(100003).scaled(65537))
print(json.dumps([a, b, first, second]))
"""
    run = _run_kernel_child(code)
    cold, warm, first, second = json.loads(run.stdout)
    assert cold != warm, "контроль обязан воспроизвести старую зависимость от истории"
    assert first == second == "Sqrt(Rational(429522722195107, 1))"


def test_native_canon_and_reader_do_not_call_sympy(monkeypatch):
    def forbidden(*args, **kwargs):
        raise AssertionError("канон V2 не должен вызывать SymPy")

    for name in ("sqrt", "factor", "factorint", "cancel", "srepr", "sympify"):
        monkeypatch.setattr(sp, name, forbidden)
    for sign in (-1, 1):
        value = nx.RadicalSumV1.sqrt_of_rational(Fraction(14, 9)).scaled(sign)
        scalar = ExactScalar.canonical(value)
        assert scalar.expression == ("-" if sign < 0 else "") + "Sqrt(Rational(14, 9))"
        assert scalar.native() == value
        assert ExactScalar.canonical(scalar.expression) == scalar


@pytest.mark.parametrize("square", (Fraction(2), Fraction(14, 9), Fraction(65537**2 * 100003, 49)))
@pytest.mark.parametrize("sign", (-1, 1))
def test_every_public_input_route_roundtrips_the_same_value(square, sign):
    native = nx.RadicalSumV1.sqrt_of_rational(square).scaled(sign)
    text = nx.canonical_text(native)
    legacy = ExactScalar.from_value(nx.to_sympy(native))
    for value in (native, legacy, ExactScalar(text), text, nx.to_sympy(native)):
        got = ExactScalar.canonical(value)
        assert got.expression == text
        assert got.native() == native
        assert sp.simplify(got.as_expr() - nx.to_sympy(native)) == 0
    assert ExactScalar.from_value(text).native() == native


@pytest.mark.parametrize("value", (0, -7, Fraction(2, 3), Fraction(-5, 9), Decimal("0.45"), 0.5))
def test_rational_text_is_unchanged(value):
    assert ExactScalar.canonical(value) == ExactScalar.from_value(value)


@pytest.mark.parametrize("value", (sp.sqrt(2) + sp.sqrt(3), sp.pi, sp.sqrt(1 + sp.sqrt(2))))
def test_unsupported_identity_is_named_and_counted_without_legacy_fallback(value):
    before = sb.BACKEND_COUNTS.get("exact_scalar_text.canon_unsupported", 0)
    with pytest.raises(nx.ExactScalarTextCanonUnsupported) as caught:
        ExactScalar.canonical(value)
    assert caught.value.code == "EXACT_SCALAR_TEXT_CANON_UNSUPPORTED"
    assert sb.BACKEND_COUNTS["exact_scalar_text.canon_unsupported"] == before + 1


@pytest.mark.parametrize("matrix", ((1, 0, 0, 1), (1, 0, 1, 1), (2, 1, 1, 2)))
def test_contact_points_and_concave_keys_share_the_rational_coordinate_canon(matrix):
    # Рациональный сдвиг/скос меняет нормали и alpha на иррациональные, но контакты
    # в вершинах остаются рациональными координатами исходного контура.
    a, b, c, d = matrix
    def transform(point):
        x, y = point
        return a*x + b*y + 0.125, c*x + d*y - 0.25

    snapshot, request = straight_snapshot(
        faces=tuple(tuple(map(transform, face)) for face in CONCAVE),
        source_routes=({"name": "source", "points": tuple(map(transform, ((0, 0), (10, 0))))},),
        alpha="20",
    )
    context, domain = _geometry(snapshot, request)
    answers, concave_hits, irrational = [], 0, 0
    for mode in sb.SymbolicBackendV1:
        with sb.symbolic_backend(mode):
            for source in _sources(context):
                for boundary in domain.blocking_segments:
                    for alpha, station, point in _contact_candidates(context, source, boundary):
                        values = point.natives()
                        by_value = any(
                            values == tuple(ExactScalar(text).native() for text in key)
                            for key in boundary.concave_vertex_keys
                        )
                        assert (point_key(point) in boundary.concave_vertex_keys) == by_value
                        if by_value:
                            concave_hits += 1
                            assert all(value.as_rational() is not None for value in values)
                            irrational += ExactScalar.canonical(alpha).native().as_rational() is None
            answers.append(resolve_component_alphas(context, LocalLengthV1(Decimal("20")), domain))
    assert concave_hits > 0
    if matrix != (1, 0, 0, 1):
        assert irrational > 0, "скошенный контур обязан проверить иррациональную alpha"
    assert answers[0] == answers[1] == answers[2]
    assert any(resolution.event_keys for resolution in answers[0][0].values())


def test_strip_identity_is_value_based_and_rejects_multiple_terms():
    snapshot, request = straight_snapshot(
        faces=CONCAVE,
        source_routes=({"name": "source", "points": ((0, 0), (10, 0))},),
        alpha="20",
    )
    context, _ = _geometry(snapshot, request)
    from cftuv_envelope.contracts.envelopes import StripEnvelopeSpec
    spec = next(item for item in context.compilation.envelope_specs if isinstance(item, StripEnvelopeSpec))
    first = nx.RadicalSumV1.sqrt_of_rational(8)
    second = nx.RadicalSumV1.sqrt_of_rational(2).scaled(2)
    assert strip_envelope_instance_id(spec, first) == strip_envelope_instance_id(spec, second)
    with pytest.raises(nx.ExactScalarTextCanonUnsupported):
        strip_envelope_instance_id(spec, first + nx.RadicalSumV1.rational(1))


def test_event_key_dedup_is_unchanged_for_other_representatives_of_contact_values(monkeypatch):
    from cftuv_envelope.reference import boundary
    snapshot, request = straight_snapshot(
        faces=tuple(tuple((x, x + y) for x, y in face) for face in CONCAVE),
        source_routes=({"name": "source", "points": ((0, 0), (10, 10))},), alpha="20",
    )
    context, domain = _geometry(snapshot, request)
    alpha = LocalLengthV1(Decimal("20"))
    changed = []
    original = boundary._contacts_bounded

    def other_representative(*args, **kwargs):
        for (value, station, point), bounds in original(*args, **kwargs):
            if isinstance(value, nx.RadicalSumV1) and value.as_rational() is None:
                alternate = nx.RadicalSumV1(tuple((r * 49, c / 7) for r, c in value.terms))
                assert alternate == value
                changed.append(value)
                value = alternate
            yield (value, station, point), bounds

    with sb.symbolic_backend(sb.SymbolicBackendV1.NATIVE_EXACT):
        expected = resolve_component_alphas(context, alpha, domain)
        monkeypatch.setattr(boundary, "_contacts_bounded", other_representative)
        actual = resolve_component_alphas(context, alpha, domain)
    assert changed
    assert actual == expected
    assert any(item.event_keys for item in actual[0].values())


# --------------------------------------------------------------------------
# Все экземпляры огибающих называют эффективную alpha каноном V2, а не только полоса
# --------------------------------------------------------------------------
# Срез V2-COMPLETION: `cap.py`, `angular.py` и `raw_coverage.py` писали строку alpha прежним `ExactScalar.from_value` (`srepr(factor(cancel(...)))`):
# имя экземпляра Cap/Angular и `effective_alpha` в записи RAW зависели от вида выражения `sympy` и от истории разложений процесса, а полоса того же
# домена с той же alpha называлась каноном V2.

def _cap_fixture():
    return straight_snapshot(
        faces=CONCAVE,
        source_routes=({"name": "source", "points": ((0, 0), (10, 0))},),
        alpha="4",
    )


def _evaluators():
    from cftuv_envelope.contracts.envelopes import AngularEnvelopeSpec, CapEnvelopeSpec
    from cftuv_envelope.reference.angular import evaluate_angular_envelope
    from cftuv_envelope.reference.cap import evaluate_cap_envelope
    from reference_factories import angular_snapshot

    return {
        "cap": (_cap_fixture, CapEnvelopeSpec, evaluate_cap_envelope),
        "angular": (lambda: angular_snapshot(1), AngularEnvelopeSpec, evaluate_angular_envelope),
    }


def _instance_of(kind, effective_alpha):
    from cftuv_envelope.reference.common import stable_id

    build, spec_type, evaluate = _evaluators()[kind]
    context, _ = _geometry(*build())
    spec = next(item for item in context.compilation.envelope_specs if isinstance(item, spec_type))
    instance = evaluate(context, spec, LocalLengthV1(Decimal("1")), effective_alpha)
    return instance, stable_id("envelope-instance", spec.envelope_spec_id, instance.effective_alpha.expression)


@pytest.mark.parametrize("kind", ("cap", "angular"))
@pytest.mark.parametrize("square", (8, 65537**2 * 100003))
def test_cap_and_angular_name_an_irrational_effective_alpha_by_the_v2_canon(kind, square):
    expected = nx.canonical_text(nx.RadicalSumV1.sqrt_of_rational(square))
    forms = (
        sp.sqrt(sp.Integer(square)),
        sp.Pow(sp.Integer(square), sp.Rational(1, 2), evaluate=False),
        nx.to_sympy(nx.RadicalSumV1.sqrt_of_rational(square)),
    )
    ids = set()
    for form in forms:
        instance, id_by_text = _instance_of(kind, form)
        assert instance.effective_alpha.expression == expected
        assert instance.envelope_instance_id == id_by_text
        ids.add(instance.envelope_instance_id)
    assert len(ids) == 1


@pytest.mark.parametrize("kind", ("cap", "angular"))
def test_cap_and_angular_names_do_not_depend_on_the_factorization_history_of_the_process(kind):
    from sympy.core.cache import clear_cache
    from sympy.ntheory.factor_ import factor_cache

    n = 65537**2 * 100003
    try:
        factor_cache.clear()
        clear_cache()
        cold = sp.sqrt(n)
        sp.factorint(n)
        clear_cache()
        warm = sp.sqrt(n)
    finally:
        factor_cache.clear()
        clear_cache()
    # Красный контроль: прежний путь действительно зависел от истории, и тест это видит; V2 - нет.
    assert ExactScalar.from_value(cold) != ExactScalar.from_value(warm), "контроль обязан воспроизвести старую зависимость от истории"
    cold_instance, _ = _instance_of(kind, cold)
    warm_instance, _ = _instance_of(kind, warm)
    assert cold_instance.effective_alpha == warm_instance.effective_alpha
    assert cold_instance.envelope_instance_id == warm_instance.envelope_instance_id
    assert cold_instance.effective_alpha.expression == "Sqrt(Rational(429522722195107, 1))"


def test_a_raw_evaluation_with_an_irrational_alpha_names_every_instance_like_its_component():
    from cftuv_envelope.reference import compile_reference_envelopes, evaluate_reference_raw_coverage

    def transform(point):
        x, y = point
        return 2 * x + y + 0.125, x + 2 * y - 0.25

    snapshot, request = straight_snapshot(
        faces=tuple(tuple(map(transform, face)) for face in CONCAVE),
        source_routes=({"name": "source", "points": tuple(map(transform, ((0, 0), (10, 0))))},),
        alpha="20",
    )
    raw = evaluate_reference_raw_coverage(compile_reference_envelopes(snapshot, request).compilation, Decimal("20")).raw_coverage
    assert raw is not None
    components = {item.effective_alpha.expression for item in raw.component_effective_alphas}
    irrational = [item for item in raw.envelope_instances if item.effective_alpha.native().as_rational() is None]
    assert {item.envelope_variant for item in irrational} == {"StripEnvelope", "CapEnvelope"}, "случай обязан нести иррациональную alpha у полосы и торцов"
    for item in irrational:
        assert item.effective_alpha.expression == nx.canonical_text(item.effective_alpha.native())
        assert item.effective_alpha.expression in components
    for item in raw.boundary_resolved_envelopes:
        assert item.effective_alpha.expression == nx.canonical_text(item.effective_alpha.native())


def _from_value_of_an_alpha(source):
    """Места, где имя alpha (`alpha`, `effective`, `effective_alpha`) уходит в прежний `ExactScalar.from_value`: строка alpha обязана быть каноном V2."""

    import ast

    found = []
    for node in ast.walk(ast.parse(source)):
        if (
            isinstance(node, ast.Call)
            and isinstance(node.func, ast.Attribute)
            and node.func.attr == "from_value"
            and isinstance(node.func.value, ast.Name)
            and node.func.value.id == "ExactScalar"
            and node.args
        ):
            argument = node.args[0]
            name = argument.id if isinstance(argument, ast.Name) else argument.attr if isinstance(argument, ast.Attribute) else ""
            if name in {"alpha", "effective", "effective_alpha"}:
                found.append(node.lineno)
    return found


def test_the_alpha_text_rule_catches_the_old_call_and_passes_the_canon():
    assert _from_value_of_an_alpha("x = ExactScalar.from_value(effective_alpha)\n") == [1]
    assert _from_value_of_an_alpha("y = ExactScalar.from_value(self.alpha)\n") == [1]
    assert _from_value_of_an_alpha("x = ExactScalar.canonical(effective_alpha)\n") == []
    assert _from_value_of_an_alpha("x = ExactScalar.from_value(requested)\n") == []


def test_no_reference_evaluator_writes_an_alpha_text_by_the_old_from_value():
    root = Path(nx.__file__).resolve().parent
    offenders = {
        path.name: lines
        for path in sorted(root.glob("*.py"))
        if (lines := _from_value_of_an_alpha(path.read_text(encoding="utf-8")))
    }
    assert not offenders, f"строка alpha пишется прежним ExactScalar.from_value (нужен ExactScalar.canonical, канон V2): {offenders}"


@pytest.mark.parametrize("mode", list(sb.SymbolicBackendV1), ids=lambda mode: mode.value)
@pytest.mark.parametrize("kind", ("cap", "angular"))
def test_the_sympy_mode_names_alphas_by_the_v2_canon_and_has_no_sympy_fallback_for_text(mode, kind):
    # `SYMPY` - откат по ЗНАЧЕНИЯМ: прежних имён alpha (`srepr(factor(cancel(...)))`) он не воспроизводит, и уступки sympy для текста нет.
    with sb.symbolic_backend(mode):
        assert ExactScalar.canonical(sp.sqrt(8)).expression == "Sqrt(Rational(8, 1))"
        assert ExactScalar.canonical(-sp.sqrt(sp.Rational(14, 9))).expression == "-Sqrt(Rational(14, 9))"
        for outside in (sp.sqrt(2) + sp.sqrt(3), sp.pi):
            with pytest.raises(nx.ExactScalarTextCanonUnsupported):
                ExactScalar.canonical(outside)
        instance, id_by_text = _instance_of(kind, sp.sqrt(8))
    assert instance.effective_alpha.expression == "Sqrt(Rational(8, 1))"
    assert instance.envelope_instance_id == id_by_text
