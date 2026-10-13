"""Три шва диспетчеризации стадии EXPORT воркера (`task_inputs`): `frame`, `angular`, `snapshot_check`.

EXPORT воркера - это кадр метрики домена (`FRAME_ADMISSION`), угловые отношения (`ANGULAR_RELATIONS`) и проверка снапшота (`SNAPSHOT_VALIDATION` и проверка
запроса); каждый из трёх кусков может стать нативной операцией, и каждый стоит за ОДНИМ именованным местом вызова. Нативная операция подменит содержимое шва, а не
разбросанные вызовы, поэтому единственность шва держит этот тест, а не абзац документа:

* `frame`: `envelope_request_export._rational_affine_metric`, одна точка вызова в `build_envelope_analysis_snapshot` (под таймером `FRAME_ADMISSION`);
* `angular`: `envelope_angular_relations.build_angular_relations` (под прежним именем `_build_angular_relations` в `envelope_request_export`), одна точка вызова;
* `snapshot_check`: `envelope_angle_certificate.reject_exported_snapshot` (проверка объекта на выгрузке) и `envelope_snapshot_check.request_snapshot_issues`
  (замечания под допуском запроса). Круг кодека (`check_snapshot_bytes`) этими швами НЕ ходит: его проверка ДЕКОДИРОВАННОГО объекта - независимый эталон
  (SNAPSHOT_BYTE_CHECK_FIRST_PRESS_IN_BACKGROUND_V1), и нативный вердикт его не подменяет.

Перенос кода в `envelope_angular_relations.py` и `validation_snapshot.py` был БЕЗ ИЗМЕНЕНИЯ ПОВЕДЕНИЯ: равенство снапшотов и запросов держат существующие тесты выгрузки.
"""

from __future__ import annotations

import ast
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
HOST = ROOT / "cftuv"
KERNEL = ROOT / "kernel" / "src" / "cftuv_envelope"


def _functions(path: Path) -> dict:
    tree = ast.parse(path.read_text(encoding="utf-8"))
    found: dict = {}
    for node in ast.walk(tree):
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
            found.setdefault(node.name, node)
    return found


def _called_names(node) -> list:
    names = []
    for item in ast.walk(node):
        if isinstance(item, ast.Call):
            func = item.func
            if isinstance(func, ast.Name):
                names.append(func.id)
            elif isinstance(func, ast.Attribute):
                names.append(func.attr)
    return names


def test_the_snapshot_assembly_reaches_frame_angular_and_validation_only_through_their_seams():
    assembly = _functions(HOST / "envelope_request_export.py")["build_envelope_analysis_snapshot"]
    calls = _called_names(assembly)
    assert calls.count("_rational_affine_metric") == 1
    assert calls.count("_build_angular_relations") == 1
    assert calls.count("validate_under_policy") == 1
    # the validators are not called from the assembly itself: the object check is the seam of `validate_exported_snapshot`
    assert "validate_analysis_snapshot" not in calls
    assert "validate_reference_geometry_payload" not in calls


def test_the_angular_seam_is_the_moved_function_under_its_old_name():
    from cftuv import envelope_angular_relations, envelope_request_export

    assert envelope_request_export._build_angular_relations is envelope_angular_relations.build_angular_relations
    functions = _functions(HOST / "envelope_request_export.py")
    assert "_build_angular_relations" not in functions, "the angular loop lives in envelope_angular_relations.py, not in the big file"
    assert "owner_support" not in functions and "direction_payload" not in functions
    moved = _functions(HOST / "envelope_angular_relations.py")
    assert {"build_angular_relations", "owner_support", "direction_payload"} <= set(moved)


def test_the_object_check_of_the_export_goes_through_the_seam_and_the_byte_check_stays_the_independent_oracle():
    functions = _functions(HOST / "envelope_angle_certificate.py")
    assert "reject_exported_snapshot" in functions and "_reject_snapshot" in functions
    export_calls = _called_names(functions["validate_exported_snapshot"])
    assert export_calls.count("reject_exported_snapshot") == 1
    assert "_reject_snapshot" not in export_calls
    byte_calls = _called_names(functions["check_snapshot_bytes"])
    assert byte_calls.count("_reject_snapshot") == 1, "the decoded object is checked by the pure Python validators"
    assert "reject_exported_snapshot" not in byte_calls
    # the seam is the pure oracle today: one call of `_reject_snapshot` over the object itself
    assert _called_names(functions["reject_exported_snapshot"]) == ["_reject_snapshot"]


def test_the_worker_snapshot_check_computes_its_issues_through_the_seam_only():
    tree = ast.parse((HOST / "envelope_snapshot_check.py").read_text(encoding="utf-8"))
    functions = {node.name: node for node in ast.walk(tree) if isinstance(node, ast.FunctionDef)}
    assert "request_snapshot_issues" in functions
    assert _called_names(functions["request_snapshot_issues"]).count("validate_analysis_snapshot") == 1
    check_class = next(node for node in tree.body if isinstance(node, ast.ClassDef) and node.name == "WorkerSnapshotCheck")
    call = next(node for node in check_class.body if isinstance(node, ast.FunctionDef) and node.name == "__call__")
    calls = _called_names(call)
    assert calls.count("request_snapshot_issues") == 1
    assert "validate_analysis_snapshot" not in calls


def test_the_snapshot_validator_lives_in_its_own_module_and_the_facade_keeps_the_name():
    import cftuv_envelope.validation as facade
    import cftuv_envelope.validation_snapshot as snapshot_module

    assert facade.validate_analysis_snapshot is snapshot_module.validate_analysis_snapshot
    names = {node.name for node in ast.parse((KERNEL / "validation.py").read_text(encoding="utf-8")).body if isinstance(node, ast.FunctionDef)}
    assert "validate_analysis_snapshot" not in names, "the snapshot validator is defined once, in validation_snapshot.py"
    assert "_junction_route_pairs" not in names and "junction_route_pairs" not in names
