"""Выделение Density не меняет тела эталона 57e3b346 и не возвращается в фасады.

Отпечатки только перенесённых определений: прежние angular/direction/adaptive
фасады могут развиваться независимо. Разрешённая разница — локальный импорт
из прежнего angular заменён прямым импортом нового владельца.
"""

from __future__ import annotations

import ast
from hashlib import sha256
from importlib import import_module
from pathlib import Path

import pytest
import sympy as sp
from mpmath import iv

from cftuv_envelope.reference import density_sign
from cftuv_envelope.reference.metric import ExactPlanarMetric


from kernel_test_paths import PACKAGE_ROOT

SOURCE = PACKAGE_ROOT / "reference"
EXTRACTED = (
    ("angular", "density_sign", "bf95e9ed15f7c6b2b82f4d10eb89d2487a18e9fc0d7f65431a1707d27f1393ed"),
    ("angular", "density_vectors", "1086442cfec68def808c266efcc1033532da017ba7ef074cb2fd0d84c4ee137f"),
    ("angular", "density_turn", "3ef7921e09fbe0221a73beb8bc8996949034842f2e8e7ff2ea5568777f003d02"),
    ("direction_binding", "density_binding_types", "5d1e45c62d7bc5d69b2ee46234ae5854937481ad5c4d4c0e2600fe2a0e56b876"),
    ("direction_binding", "density_binding_values", "185f422aff645072d380156bfaa98bf02b752236cc180efb806482826807ee2d"),
    ("adaptive_density_fan", "density_fan_types", "e1207a6b50caaecbb2f81d997d4b46dee93d801d6fd0362a4d7bcffe7f950415"),
    ("adaptive_density_fan", "density_projective", "a5b2abc2fcb5ad03fd843470329ff98e969b997d082e089e35c9e850035ec9d5"),
)
FACADE_NAMES = {"angular", "direction_binding", "adaptive_density_fan"}


def _tree(module):
    return ast.parse((SOURCE / f"{module}.py").read_text(encoding="utf-8"))


def _definitions(tree):
    return [node for node in tree.body if isinstance(node, (ast.FunctionDef, ast.ClassDef))]


@pytest.mark.parametrize("facade,leaf,digest", EXTRACTED)
def test_extracted_definition_bodies_are_the_unchanged_reference(facade, leaf, digest):
    del facade
    module = ast.Module(body=_definitions(_tree(leaf)), type_ignores=[])
    assert sha256(ast.dump(module).encode("utf-8")).hexdigest() == digest


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
