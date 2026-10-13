"""Выделение Density не меняет тела эталона 57e3b346 и не возвращается в фасады.

Отпечатки только перенесённых определений: прежние angular/direction/adaptive
фасады могут развиваться независимо. Разрешённая разница — локальный импорт
из прежнего angular заменён прямым импортом нового владельца.
"""

from __future__ import annotations

import ast
from copy import deepcopy
from hashlib import sha256
from importlib import import_module
import json
from pathlib import Path

import pytest
import sympy as sp
from mpmath import iv

from cftuv_envelope.reference import density_sign
from cftuv_envelope.reference.metric import ExactPlanarMetric


from kernel_test_paths import PACKAGE_ROOT

SOURCE = PACKAGE_ROOT / "reference"
# DENSITY_EXTRACTED_AST_V1: тела baseline 57e3b346; только три разрешённых локальных импорта направлены новым владельцам.
EXTRACTED = (
    ("angular", "density_sign", "c02237b89dfa9acfbdcfe7625a736ec68b9095091cc14bdb3cd4690c588c96ac"),
    ("angular", "density_vectors", "8b8be59cbed47664408fe9a2f18c5b0b2bf5c34a8de389d0f867483736f89767"),
    ("angular", "density_turn", "86d3bf9a5b1fde0728f3ba1f935520200169f1f5639e6cd1d1718deb8f24fc6b"),
    ("direction_binding", "density_binding_types", "a5537135598e3df6c6ef3e55720332ed7bde96dc23fe6da18a8977ce9ee7fbd3"),
    ("direction_binding", "density_binding_values", "c4db87cdde6cc09dda2ca812c6f4c3354fd09f645acd981cdc0bd876e06bded4"),
    ("adaptive_density_fan", "density_fan_types", "cb038b68c238648d4b0884e8806836b0e91b540508bce1d486684eac3bf5e1a9"),
    ("adaptive_density_fan", "density_projective", "c52df743fbd7535a05b65b7e35eac8b8d19bd928bc348761b4fd6e905bf2a12f"),
)
FACADE_NAMES = {"angular", "direction_binding", "adaptive_density_fan"}


def _tree(module):
    return ast.parse((SOURCE / f"{module}.py").read_text(encoding="utf-8"))


def _definitions(tree):
    return [node for node in tree.body if isinstance(node, (ast.FunctionDef, ast.ClassDef))]


def _structural_ast(value):
    """AST-поля явно включают пустые значения; пустой type_params не меняет тело."""
    if isinstance(value, ast.AST):
        fields = {name: _structural_ast(child) for name, child in ast.iter_fields(value)
                  if name != "type_params" or child != []}
        parameters = getattr(value, "type_params", ())
        if parameters and "type_params" not in fields:
            fields["type_params"] = _structural_ast(parameters)
        return [type(value).__name__, fields]
    if isinstance(value, list):
        return [_structural_ast(child) for child in value]
    if type(value) is float:
        return ["float", value.hex()]
    if type(value) is complex:
        return ["complex", value.real.hex(), value.imag.hex()]
    if type(value) is bytes:
        return ["bytes", value.hex()]
    if value is Ellipsis:
        return ["ellipsis"]
    if value is None or type(value) in (str, int, bool):
        return value
    raise TypeError(f"unsupported extracted AST value: {type(value).__name__}")


def _definition_digest(module):
    wire = ["DENSITY_EXTRACTED_AST_V1", _structural_ast(module)]
    return sha256(json.dumps(wire, sort_keys=True, separators=(",", ":"), ensure_ascii=True).encode("ascii")).hexdigest()


@pytest.mark.parametrize("facade,leaf,digest", EXTRACTED)
def test_extracted_definition_bodies_are_the_unchanged_reference(facade, leaf, digest):
    del facade
    module = ast.Module(body=_definitions(_tree(leaf)), type_ignores=[])
    assert _definition_digest(module) == digest


@pytest.mark.parametrize("facade,leaf,digest", EXTRACTED)
def test_reference_digest_rejects_a_real_definition_body_edit(facade, leaf, digest):
    del facade
    module = ast.Module(body=deepcopy(_definitions(_tree(leaf))), type_ignores=[])
    assert _definition_digest(module) == digest
    body = module.body[0].body
    offset = int(isinstance(body[0], ast.Expr) and isinstance(body[0].value, ast.Constant)
                 and isinstance(body[0].value.value, str))
    body.insert(offset, ast.Raise(exc=ast.Call(func=ast.Name(id="RuntimeError", ctx=ast.Load()),
                                             args=[ast.Constant(value="changed reference body")], keywords=[]), cause=None))
    assert _definition_digest(module) != digest


def test_empty_type_parameter_metadata_is_neutral_but_actual_parameters_are_retained():
    module = ast.parse("def f(x): return x")
    expected = _definition_digest(module)
    module.body[0].type_params = []
    assert _definition_digest(module) == expected
    module.body[0].type_params = [ast.Name(id="T", ctx=ast.Load())]
    assert _definition_digest(module) != expected


def test_reference_digest_ignores_layout_and_preserves_literal_types():
    assert _definition_digest(ast.parse("def f(): return 1")) == _definition_digest(ast.parse("# layout\ndef f():\n    return 1\n"))
    assert len({_definition_digest(ast.parse(f"def f(): return {literal}"))
                for literal in ("1", "1.0", "True", "b'x'", "'x'", "...", "1j")}) == 7


@pytest.mark.parametrize("facade,leaf,digest", EXTRACTED)
def test_legacy_names_are_identical_aliases_of_their_new_owner(facade, leaf, digest):
    del digest
    facade_module = import_module(f"cftuv_envelope.reference.{facade}")
    leaf_module = import_module(f"cftuv_envelope.reference.{leaf}")
    for node in _definitions(_tree(leaf)):
        assert getattr(facade_module, node.name) is getattr(leaf_module, node.name)


@pytest.mark.parametrize("facade,leaf,digest", EXTRACTED)
def test_density_leaves_have_no_import_back_to_a_legacy_facade(facade, leaf, digest):
    del facade, digest
    for node in ast.walk(_tree(leaf)):
        if isinstance(node, ast.ImportFrom):
            assert not (node.level == 1 and node.module in FACADE_NAMES)
        elif isinstance(node, ast.Import):
            assert all(alias.name.split(".")[-1] not in FACADE_NAMES for alias in node.names)


def test_residual_exact_fallback_has_no_facade_bounce():
    tree = _tree("density_residual")
    assert all(
        node.module not in FACADE_NAMES
        for node in ast.walk(tree)
        if isinstance(node, ast.ImportFrom) and node.level == 1
    )


def test_density_sign_still_uses_interval_first_and_restores_precision(monkeypatch):
    identity = ((sp.Integer(1), sp.Integer(0)), (sp.Integer(0), sp.Integer(1)))
    metric = ExactPlanarMetric(identity, identity, 1)
    expression = sp.sqrt(2) - 1

    def forbidden(*args, **kwargs):
        raise AssertionError("interval proof must precede the quadratic fallback")

    monkeypatch.setattr(density_sign, "exact_quadratic_value", forbidden)
    saved = iv.prec
    try:
        iv.prec = 53
        assert density_sign._density_exact_sign(expression, metric) == 1
        assert metric._density_exact_memo.signs[expression] == 1
        assert iv.prec == 53
        assert density_sign._density_exact_sign(expression, metric) == 1
    finally:
        iv.prec = saved
