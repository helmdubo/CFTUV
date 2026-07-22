from __future__ import annotations

import ast
import importlib.util
from pathlib import Path

import cftuv_envelope


FORBIDDEN = {"bpy", "mathutils", "cftuv"}
FORBIDDEN_IMPLEMENTATION_MODULES = {
    "arrangement.py",
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
    assert not (FORBIDDEN_IMPLEMENTATION_MODULES & {path.name for path in package_root.rglob("*.py")})
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

