from __future__ import annotations

import importlib
import subprocess
import sys

import pytest

import cftuv_envelope


# Полный публичный список уже отдельно заморожен дайджестом в
# test_public_contracts.py. Здесь замораживается оставшаяся часть dir(), чтобы
# ленивый фасад не менял даже исторически видимые служебные имена.
_TOP_LEVEL_DIR_EXTRAS = (
    "_PUBLIC_VALUE_MODULES",
    "__all__",
    "__builtins__",
    "__cached__",
    "__doc__",
    "__file__",
    "__loader__",
    "__name__",
    "__package__",
    "__path__",
    "__spec__",
    "_analysis",
    "_coverage",
    "_debug",
    "_density_policy",
    "_envelopes",
    "_events",
    "_geometry_batch",
    "_ids",
    "_metric",
    "_numeric",
    "_outcomes",
    "_ownership",
    "_plan",
    "_request",
    "_seeds",
    "_surface",
    "_tessellation",
)

_WAVEFRONT_DIR_EXTRAS = (
    "__all__",
    "__builtins__",
    "__cached__",
    "__doc__",
    "__file__",
    "__loader__",
    "__name__",
    "__package__",
    "__path__",
    "__spec__",
    "bridge",
    "cell_grid",
    "conveyor",
    "coverage",
    "event_time",
    "events",
    "faces",
    "motorcycle",
    "polygon",
    "skeleton",
    "sqrt_sum",
)


def test_top_level_dir_matches_eager_facade_snapshot():
    expected = sorted((*cftuv_envelope.__all__, *_TOP_LEVEL_DIR_EXTRAS))
    assert sorted(dir(cftuv_envelope)) == expected

    cftuv_envelope.PatchId
    assert sorted(dir(cftuv_envelope)) == expected


def test_top_level_lazy_exports_preserve_module_identity_and_aliases():
    for name, (module_name, symbol_name) in cftuv_envelope._EXPORTS.items():
        expected = getattr(importlib.import_module(module_name), symbol_name)
        assert getattr(cftuv_envelope, name) is expected, name

    assert (
        cftuv_envelope.CompiledPatchEvaluationPlanV1
        is cftuv_envelope.CompiledPatchEvaluationPlan
    )
    assert cftuv_envelope.TessellationPlanV1 is cftuv_envelope.TessellationPlan


def test_import_star_preserves_both_facade_surfaces():
    root_namespace: dict[str, object] = {}
    exec("from cftuv_envelope import *", root_namespace)
    assert all(
        root_namespace[name] is getattr(cftuv_envelope, name)
        for name in cftuv_envelope.__all__
    )

    wavefront = importlib.import_module("cftuv_envelope.wavefront")
    wavefront_namespace: dict[str, object] = {}
    exec("from cftuv_envelope.wavefront import *", wavefront_namespace)
    assert all(
        wavefront_namespace[name] is getattr(wavefront, name)
        for name in wavefront.__all__
    )


def test_wavefront_dir_and_lazy_exports_match_eager_facade_snapshot():
    wavefront = importlib.import_module("cftuv_envelope.wavefront")
    expected_dir = sorted((*wavefront.__all__, *_WAVEFRONT_DIR_EXTRAS))
    assert sorted(dir(wavefront)) == expected_dir

    for name, (module_name, symbol_name) in wavefront._EXPORTS.items():
        expected = getattr(importlib.import_module(module_name), symbol_name)
        assert getattr(wavefront, name) is expected, name
    assert sorted(dir(wavefront)) == expected_dir


@pytest.mark.parametrize(
    "module_name",
    ("cftuv_envelope", "cftuv_envelope.wavefront"),
)
def test_missing_lazy_name_keeps_exact_attribute_error(module_name: str):
    module = importlib.import_module(module_name)
    with pytest.raises(AttributeError) as caught:
        getattr(module, "__hyg1_missing__")

    assert str(caught.value) == (
        f"module {module_name!r} has no attribute '__hyg1_missing__'"
    )
    assert caught.value.__cause__ is None
    assert caught.value.__context__ is None


def test_lazy_root_resolution_does_not_leak_child_modules():
    source_root = str(__file__.rsplit("tests", 1)[0] + "src")
    script = f"""
import sys
sys.path.insert(0, {source_root!r})
import cftuv_envelope as package
before = sorted(dir(package))
for name in package.__all__:
    getattr(package, name)
assert sorted(dir(package)) == before
leaks = set(package.__dict__) & set(package._CHILD_MODULE_NAMES)
assert not leaks, sorted(leaks)
"""
    result = subprocess.run(
        [sys.executable, "-c", script],
        check=False,
        capture_output=True,
        text=True,
    )
    assert result.returncode == 0, result.stderr


def test_wavefront_leaf_imports_do_not_require_sympy_or_mpmath():
    source_root = str(__file__.rsplit("tests", 1)[0] + "src")
    script = f"""
import importlib.abc
import sys
sys.path.insert(0, {source_root!r})

class BlockSymbolicPackages(importlib.abc.MetaPathFinder):
    def find_spec(self, fullname, path=None, target=None):
        if fullname == "sympy" or fullname.startswith("sympy."):
            raise ImportError("blocked " + fullname)
        if fullname == "mpmath" or fullname.startswith("mpmath."):
            raise ImportError("blocked " + fullname)
        return None

sys.meta_path.insert(0, BlockSymbolicPackages())
import cftuv_envelope as package
import cftuv_envelope.wavefront.event_time
import cftuv_envelope.wavefront.events
import cftuv_envelope.wavefront.skeleton
for child_name in package._CHILD_MODULE_NAMES:
    assert child_name not in package.__dict__, child_name
    try:
        getattr(package, child_name)
    except AttributeError as exc:
        assert str(exc) == (
            f"module 'cftuv_envelope' has no attribute {{child_name!r}}"
        )
    else:
        raise AssertionError(child_name + " leaked onto the root facade")
assert not any(
    name == "sympy" or name.startswith("sympy.")
    or name == "mpmath" or name.startswith("mpmath.")
    for name in sys.modules
)
"""
    result = subprocess.run(
        [sys.executable, "-c", script],
        check=False,
        capture_output=True,
        text=True,
    )
    assert result.returncode == 0, result.stderr


def test_direct_child_imports_work_without_root_attribute_leaks():
    source_root = str(__file__.rsplit("tests", 1)[0] + "src")
    script = f"""
import sys
sys.path.insert(0, {source_root!r})
import cftuv_envelope as package
import cftuv_envelope.codec as codec_alias
from cftuv_envelope import codec as codec_from
assert codec_alias is sys.modules["cftuv_envelope.codec"]
assert codec_from is codec_alias
assert codec_alias.canonical_json_bytes({{"value": 1}}) == b'{{"value":1}}'
for child_name in package._CHILD_MODULE_NAMES:
    assert child_name not in package.__dict__, child_name
    try:
        getattr(package, child_name)
    except AttributeError as exc:
        assert str(exc) == (
            f"module 'cftuv_envelope' has no attribute {{child_name!r}}"
        )
    else:
        raise AssertionError(child_name + " leaked onto the root facade")
"""
    result = subprocess.run(
        [sys.executable, "-c", script],
        check=False,
        capture_output=True,
        text=True,
    )
    assert result.returncode == 0, result.stderr
