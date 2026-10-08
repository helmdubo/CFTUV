from __future__ import annotations

import ast
import importlib.util
from pathlib import Path
from types import SimpleNamespace

import cftuv_envelope
import pytest

from kernel_test_paths import InstalledKernelGuard, KERNEL_ROOT, PACKAGE_ROOT, kernel_reference


FORBIDDEN = {"bpy", "mathutils", "cftuv"}
FORBIDDEN_IMPLEMENTATION_MODULES = {
    "blender_adapter.py",
    "boolean.py",
    "evaluator.py",
    "materializer.py",
    "wavefront_runtime.py",
}


def test_blender_and_host_packages_are_physically_absent():
    for name in sorted(FORBIDDEN):
        assert importlib.util.find_spec(name) is None, name


def test_runtime_import_graph_has_no_host_or_blender_edges():
    package_root = Path(cftuv_envelope.__file__).resolve().parent
    assert not (
        FORBIDDEN_IMPLEMENTATION_MODULES
        & {path.name for path in package_root.rglob("*.py")}
    )
    for path in package_root.rglob("*.py"):
        tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                roots = {alias.name.split(".", 1)[0] for alias in node.names}
            elif isinstance(node, ast.ImportFrom) and node.level == 0 and node.module:
                roots = {node.module.split(".", 1)[0]}
            else:
                roots = set()
            assert roots.isdisjoint(FORBIDDEN), (path, node.lineno, roots)


def test_no_geometry_implementation_modules_exist():
    package_root = Path(cftuv_envelope.__file__).resolve().parent
    assert FORBIDDEN_IMPLEMENTATION_MODULES.isdisjoint(
        {path.name for path in package_root.iterdir() if path.is_file()}
    )


def test_extracted_support_and_runtime_source_inventory_is_not_empty():
    assert (PACKAGE_ROOT / "reference" / "arrangement.py").is_file()
    for relative in (
        "tools/generate_contract_schemas.py",
        "tools/generate_surface_contract_schemas.py",
        "tools/fan_congruence_check.py",
        "artifacts/envelope_c_r2c_fixture/historical_df587ed_result.json",
        "artifacts/envelope_c_r2c_fixture/selected_c262_result.json",
        "artifacts/kernel_audit_exact_proof/p0_3_post_p0_2b_absolute_digests.json",
    ):
        assert (KERNEL_ROOT / relative).is_file(), relative
    assert kernel_reference("kernel/tests/test_isolation.py") == Path(__file__).resolve()


def test_wheel_guard_rejects_checkout_origin_even_after_a_cached_valid_module(tmp_path):
    wheel = tmp_path / "site-packages" / "cftuv_envelope"
    checkout = tmp_path / "checkout" / "src" / "cftuv_envelope"
    for root in (wheel, checkout):
        root.mkdir(parents=True)
        (root / "__init__.py").write_text("", encoding="utf-8")
    module = SimpleNamespace(__file__=str(wheel / "__init__.py"), __spec__=SimpleNamespace(origin=str(wheel / "__init__.py")))
    guard = InstalledKernelGuard(wheel)
    modules = {"cftuv_envelope": module}
    guard.check(modules)
    guard.check(modules)
    assert guard.path_resolutions == 1
    module.__file__ = str(checkout / "__init__.py")
    with pytest.raises(AssertionError, match="KERNEL_WHEEL_ORIGIN_INVALID"):
        guard.check(modules)
    module.__file__ = str(wheel / "__init__.py")
    module.__spec__.origin = str(checkout / "__init__.py")
    with pytest.raises(AssertionError, match="KERNEL_WHEEL_ORIGIN_INVALID"):
        guard.check(modules)

