"""Static import wall for the standalone runtime package."""

from __future__ import annotations

import argparse
import ast
from pathlib import Path


FORBIDDEN_ROOTS = {"bpy", "mathutils", "cftuv"}
FORBIDDEN_MODULE_FILES = {
    "blender_adapter.py",
    "boolean.py",
    "evaluator.py",
    "materializer.py",
    "wavefront_runtime.py",
}


def scan(source_root: Path) -> tuple[str, ...]:
    errors: list[str] = []
    for path in sorted(source_root.rglob("*.py")):
        if path.name in FORBIDDEN_MODULE_FILES:
            errors.append(f"forbidden future implementation module: {path}")
        tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
        for node in ast.walk(tree):
            roots: tuple[str, ...] = ()
            if isinstance(node, ast.Import):
                roots = tuple(alias.name.split(".", 1)[0] for alias in node.names)
            elif isinstance(node, ast.ImportFrom) and node.level == 0 and node.module:
                roots = (node.module.split(".", 1)[0],)
            for root in roots:
                if root in FORBIDDEN_ROOTS:
                    errors.append(f"{path}:{node.lineno}: forbidden import {root}")
    return tuple(errors)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("source_root", type=Path)
    args = parser.parse_args()
    errors = scan(args.source_root)
    if errors:
        for error in errors:
            print(error)
        return 1
    print("forbidden-import-scan: OK")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

