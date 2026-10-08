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
    env = dict(os.environ, PYTHONPATH=str(Path(__file__).resolve().parents[1] / "src"))
    run = subprocess.run([sys.executable, "-c", code], env=env, check=True,
                         capture_output=True, text=True)
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
