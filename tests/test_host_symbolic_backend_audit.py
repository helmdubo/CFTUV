"""Аудит общих алгебраических зависимостей хоста, без требования хоста в extracted kernel."""
from __future__ import annotations

import ast
from pathlib import Path

HOST = Path(__file__).resolve().parents[1] / "cftuv"

HOST_SYMPY_AUDIT = {
    "envelope_debug_renderer.py": (
        "A_LATER",
        "float(sympify(srepr)) при рисовании отладочных точек: холодный путь рендера, читается родным мостом",
    ),
    "envelope_export_input.py": (
        "HOST_WARMUP",
        "прогрев ленивой подгрузки sympy в родителе (sympy.Symbol('warm') + 1): ~0.35 с на процесс, "
        "пока в ядре остаётся класс (б)",
    ),
    "envelope_request_export.py": (
        "A_LATER",
        "Rational(str(float)) при выгрузке координат и factor в exact_rational: холодный экспорт хоста",
    ),
}


def _host_sympy_importers() -> set[str]:
    users = set()
    for path in sorted(HOST.rglob("*.py")):
        for node in ast.walk(ast.parse(path.read_text(encoding="utf-8-sig"))):
            if isinstance(node, ast.Import) and any(
                item.name.split(".")[0] in {"sympy", "mpmath"} for item in node.names
            ):
                users.add(path.name)
            elif isinstance(node, ast.ImportFrom) and (node.module or "").split(".")[0] in {"sympy", "mpmath"}:
                users.add(path.name)
            elif (
                isinstance(node, ast.Call)
                and getattr(node.func, "attr", getattr(node.func, "id", "")) == "import_module"
                and node.args
                and isinstance(node.args[0], ast.Constant)
                and node.args[0].value in {"sympy", "mpmath"}
            ):
                users.add(path.name)
    return users


def test_host_sympy_users_are_the_audited_files():
    assert _host_sympy_importers() == set(HOST_SYMPY_AUDIT), sorted(
        _host_sympy_importers() ^ set(HOST_SYMPY_AUDIT)
    )
