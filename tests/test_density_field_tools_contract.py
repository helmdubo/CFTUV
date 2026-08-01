from __future__ import annotations

import ast
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace

import pytest


ROOT = Path(__file__).resolve().parents[1]
FIELD_GATE = ROOT / "tools" / "run_envelope_mr1_building_gate.py"
FIELD_CYCLE = ROOT / "tools" / "field_cycle.ps1"
FIELD_BAT = ROOT / "tools" / "field_cycle.bat"
BUTTON_SWEEP = ROOT / "tools" / "blender_field_sweep.py"
NAMED_OMISSION = "UNMEASURED_REQUEST_POLICY_KNOB"


def _run(*arguments: str) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        arguments,
        cwd=ROOT,
        text=True,
        capture_output=True,
        check=False,
    )


def _json_output(result: subprocess.CompletedProcess[str]) -> dict:
    assert result.returncode == 0, result.stderr or result.stdout
    lines = [line for line in result.stdout.splitlines() if line.strip()]
    return json.loads(lines[-1])


@pytest.mark.parametrize(
    ("density", "effective", "eligible"),
    (("0", 0, True), ("1", 1, True), ("4", 4, True), ("legacy", None, False)),
)
def test_direct_gate_contract_requires_explicit_density(
    density,
    effective,
    eligible,
):
    result = _run(
        sys.executable,
        str(FIELD_GATE),
        "--contract-only",
        "--",
        "out.json",
        "all",
        "building.002",
        "queue",
        density,
    )
    payload = _json_output(result)
    assert payload["schema"].endswith(".v2")
    assert payload["effective_density"] == effective
    assert payload["acceptance_eligible"] is eligible


@pytest.mark.parametrize("density", ("", "2", "3", "5", "-1", "none", "1 extra"))
def test_direct_gate_rejects_unmeasured_density(density):
    arguments = [
        sys.executable,
        str(FIELD_GATE),
        "--contract-only",
        "--",
        "out.json",
        "all",
        "building.002",
        "queue",
    ]
    if density:
        arguments.extend(density.split())
    result = _run(*arguments)
    assert result.returncode != 0
    assert NAMED_OMISSION in result.stderr


def test_field_cycle_dry_run_forwards_density_as_fifth_gate_argument():
    result = _run(
        "powershell",
        "-NoProfile",
        "-ExecutionPolicy",
        "Bypass",
        "-File",
        str(FIELD_CYCLE),
        "-DryRun",
        "-Density",
        "1",
        "-ObjectName",
        "building",
        "-Engines",
        "queue",
    )
    payload = _json_output(result)
    assert payload["density"] == "1"
    assert payload["acceptance_eligible"] is True
    assert payload["gate_arguments"][-3:] == ["building", "queue", "1"]


def test_field_cycle_missing_density_is_named_failure():
    result = _run(
        "powershell",
        "-NoProfile",
        "-ExecutionPolicy",
        "Bypass",
        "-File",
        str(FIELD_CYCLE),
        "-DryRun",
    )
    assert result.returncode != 0
    assert NAMED_OMISSION in result.stdout


@pytest.mark.parametrize(
    "matrix",
    (
        "0,1",
        "0,1,4,4",
        "4,1,0",
        "legacy,0,1,4",
        "0,1,2,3,4",
        "0, 1,4",
    ),
)
def test_button_sweep_rejects_any_non_acceptance_matrix(matrix):
    result = _run(
        sys.executable,
        str(BUTTON_SWEEP),
        "--contract-only",
        "--",
        "out.json",
        matrix,
        "receipts",
    )
    assert result.returncode != 0
    assert NAMED_OMISSION in result.stderr


def test_button_sweep_contract_counts_exactly_9_true_button_calls():
    """Решением владельца охват свипа — три меша × один сценарий × три плотности.

    Прежний контракт (13 мешей × 2 сценария = 78) выводил свою норму из ЖИВОЙ
    сцены и о неё же ломался: владелец правит сцену непрерывно, мешей со швами
    стало не 13, и свип упал `DENSITY_SWEEP_CALL_COUNT_MISMATCH` до первой
    кнопки. Охват стал свойством инструмента и назван поимённо.
    """

    result = _run(
        sys.executable,
        str(BUTTON_SWEEP),
        "--contract-only",
        "--",
        "out.json",
        "0,1,4",
        "receipts",
    )
    payload = _json_output(result)
    assert payload["schema"].endswith(".v3")
    assert payload["density_matrix"] == [0, 1, 4]
    assert payload["swept_meshes"] == [
        "building",
        "building.002",
        "building.004",
    ]
    assert payload["scenario"] == "all_seams"
    assert payload["expected_calls_per_density"] == 3
    assert payload["expected_total_calls"] == 9


def _module(path: Path) -> ast.Module:
    return ast.parse(path.read_text(encoding="utf-8"))


def _assignment(module: ast.Module, name: str):
    for node in module.body:
        if isinstance(node, ast.Assign):
            if any(isinstance(item, ast.Name) and item.id == name for item in node.targets):
                return ast.literal_eval(node.value)
    raise AssertionError(f"assignment {name} not found")


def _function(module: ast.Module, name: str) -> ast.FunctionDef:
    return next(
        node
        for node in module.body
        if isinstance(node, ast.FunctionDef) and node.name == name
    )


def test_direct_gate_cannot_omit_density_at_request_callsite():
    module = _module(FIELD_GATE)
    calls = [
        node
        for node in ast.walk(module)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Name)
        and node.func.id == "build_envelope_decal_request"
    ]
    assert len(calls) == 1
    assert {item.arg for item in calls[0].keywords} >= {
        "decal_request_id_value",
        "density",
    }


def test_acceptance_tools_cannot_shadow_installed_product_with_repo_path():
    for path in (FIELD_GATE, BUTTON_SWEEP):
        source = path.read_text(encoding="utf-8")
        assert "sys.path.insert" not in source
        assert "repository path shadows installed product" in source
        assert "loaded_addon_root != addon_root" in source
        assert "loaded_kernel_root != kernel_root" in source
        assert "installed fingerprints differ" in source


@pytest.mark.parametrize(
    ("path", "error_name"),
    (
        (FIELD_GATE, "DensityGateContractError"),
        (BUTTON_SWEEP, "DensitySweepContractError"),
    ),
)
def test_repo_path_shadowing_is_an_executed_adversarial_negative(
    path,
    error_name,
    tmp_path,
):
    function = _function(_module(path), "_installer_identity")
    namespace = {
        "Path": Path,
        "bpy": SimpleNamespace(
            utils=SimpleNamespace(
                user_resource=lambda _name: str(tmp_path / "scripts")
            )
        ),
        "cftuv": SimpleNamespace(__file__=str(ROOT / "cftuv" / "__init__.py")),
        "kernel": SimpleNamespace(
            __file__=str(
                ROOT
                / "kernel"
                / "src"
                / "cftuv_envelope"
                / "__init__.py"
            )
        ),
        error_name: RuntimeError,
        "INSTALLER_STAMP_MISMATCH": "INSTALLER_STAMP_MISMATCH",
    }
    ast.fix_missing_locations(function)
    exec(compile(ast.Module(body=[function], type_ignores=[]), str(path), "exec"), namespace)
    with pytest.raises(RuntimeError, match="repository path shadows installed product"):
        namespace["_installer_identity"]({"head": "0" * 40})


def test_direct_and_button_use_the_same_canonical_queue_projection():
    direct = _function(_module(FIELD_GATE), "_queue_semantic_digest")
    button = _function(_module(BUTTON_SWEEP), "_queue_semantic_digest")
    assert ast.dump(direct, include_attributes=False) == ast.dump(
        button,
        include_attributes=False,
    )


def test_every_perf_cap_carries_a_named_price():
    """Каждый кап назван ценой, и таблицы двух инструментов больше не одна.

    У свипа охват сужен решением владельца: `"2"` вышел вместе со сценарием,
    `building.004` вошёл вместе с мешом. Его 5.0 — НАЗВАННЫЙ выбор, а не
    замер: 1.19 с измерены на d1, а бюджет обязан покрывать d4, который у
    этого меша ещё не мерен.

    Два капа подняты ценой новых EXACT-доменов, а не потерей скорости:
    `building` 120 -> 180 (d4 стал 111/122 против 105; семь бывших
    bridge-отказников стоят 67.6 с, старые домены не подорожали — 82.1 с;
    прогон 149.71) и `"2"` 17.3 -> 30.0 в гейте (d4 стал 20/32 против 19,
    +16.4 с, прогон 28.30 при полевой SLA-рамке 30, которая ДЕРЖИТСЯ).
    Курс владельца: ширина продукта важнее полировки скорости.
    """

    assert _assignment(_module(FIELD_GATE), "_FIELD_BUDGET_SECONDS") == {
        "building.002": 5.0,
        "2": 30.0,
        "building": 180.0,
    }
    assert _assignment(_module(BUTTON_SWEEP), "_FIELD_BUDGET_SECONDS") == {
        "building.002": 5.0,
        "building.004": 5.0,
        "building": 180.0,
    }


def test_sweep_sets_and_reads_back_ui_density_before_true_operator():
    source = BUTTON_SWEEP.read_text(encoding="utf-8")
    assignment = 'settings.envelope_debug_fan_density = str(density)'
    readback = 'settings.envelope_debug_fan_density != str(density)'
    operator = "bpy.ops.hotspotuv.build_exact_reference_envelope_debug()"
    assert source.index(assignment) < source.index(readback) < source.index(operator)
    assert source.count(operator) == 1


@pytest.mark.parametrize(
    "outcome",
    (
        {"CANCELLED"},
        {"RUNNING_MODAL"},
        {"FINISHED", "CANCELLED"},
        frozenset({"FINISHED"}),
    ),
)
def test_button_refuses_every_non_finished_operator_result(outcome):
    function = _function(_module(BUTTON_SWEEP), "_require_finished")
    namespace = {
        "DensitySweepContractError": RuntimeError,
        "BUTTON_OPERATOR_DID_NOT_FINISH": "BUTTON_OPERATOR_DID_NOT_FINISH",
    }
    exec(
        compile(
            ast.Module(body=[function], type_ignores=[]),
            str(BUTTON_SWEEP),
            "exec",
        ),
        namespace,
    )
    with pytest.raises(RuntimeError, match="BUTTON_OPERATOR_DID_NOT_FINISH"):
        namespace["_require_finished"](outcome)


def test_button_invalidates_stale_sidecar_before_operator_and_requires_identity():
    source = BUTTON_SWEEP.read_text(encoding="utf-8")
    invalidate = "text_name = _invalidate_sidecar(obj)"
    operator = "bpy.ops.hotspotuv.build_exact_reference_envelope_debug()"
    fresh = "_fresh_sidecar(obj, text_name, outcome)"
    assert source.index(invalidate) < source.index(operator) < source.index(fresh)

    module = _module(BUTTON_SWEEP)
    invalidate_function = _function(module, "_invalidate_sidecar")
    identity_function = _function(module, "_sidecar_identity")

    class Texts:
        def __init__(self, remove_works=True):
            self.text = object()
            self.remove_works = remove_works
            self.removed = 0

        def get(self, _name):
            return self.text

        def remove(self, _text):
            self.removed += 1
            if self.remove_works:
                self.text = None

    texts = Texts()
    namespace = {
        "re": __import__("re"),
        "bpy": SimpleNamespace(data=SimpleNamespace(texts=texts)),
        "envelope_debug_text_name": lambda obj: f"sidecar:{obj.name}",
        "DensitySweepContractError": RuntimeError,
        "BUTTON_SIDECAR_NOT_FRESH": "BUTTON_SIDECAR_NOT_FRESH",
    }
    exec(
        compile(
            ast.Module(
                body=[invalidate_function, identity_function],
                type_ignores=[],
            ),
            str(BUTTON_SWEEP),
            "exec",
        ),
        namespace,
    )
    assert (
        namespace["_invalidate_sidecar"](SimpleNamespace(name="building.002"))
        == "sidecar:building.002"
    )
    assert texts.removed == 1
    assert texts.text is None
    stubborn = Texts(remove_works=False)
    namespace["bpy"].data.texts = stubborn
    with pytest.raises(RuntimeError, match="BUTTON_SIDECAR_NOT_FRESH"):
        namespace["_invalidate_sidecar"](
            SimpleNamespace(name="building.002")
        )

    valid = {
        "object_name": "building.002",
        "decal_request_ids": [
            "host-v0:decal-request-density:" + "a" * 24
        ],
    }
    assert namespace["_sidecar_identity"](valid, "building.002") == (
        valid["decal_request_ids"][0],
    )
    for spoof in (
        [],
        {**valid, "object_name": "stale-object"},
        {**valid, "decal_request_ids": []},
        {**valid, "decal_request_ids": ["wrong-id"]},
        {**valid, "decal_request_ids": valid["decal_request_ids"] * 2},
    ):
        with pytest.raises(RuntimeError, match="BUTTON_SIDECAR_NOT_FRESH"):
            namespace["_sidecar_identity"](spoof, "building.002")


def test_button_does_not_read_sidecar_after_cancelled_or_running_modal():
    source = BUTTON_SWEEP.read_text(encoding="utf-8")
    function = _function(_module(BUTTON_SWEEP), "_fresh_sidecar")

    class Text:
        reads = 0

        def as_string(self):
            self.reads += 1
            return "{}"

    text = Text()
    namespace = {
        "bpy": SimpleNamespace(
            data=SimpleNamespace(texts=SimpleNamespace(get=lambda _name: text))
        ),
        "_require_finished": lambda outcome: (
            None
            if outcome == {"FINISHED"}
            else (_ for _ in ()).throw(RuntimeError("not finished"))
        ),
        "DensitySweepContractError": RuntimeError,
        "BUTTON_SIDECAR_NOT_FRESH": "BUTTON_SIDECAR_NOT_FRESH",
        "json": json,
        "_sidecar_identity": lambda payload, name: (),
    }
    exec(
        compile(
            ast.Module(body=[function], type_ignores=[]),
            str(BUTTON_SWEEP),
            "exec",
        ),
        namespace,
    )
    obj = SimpleNamespace(name="building.002")
    for outcome in ({"CANCELLED"}, {"RUNNING_MODAL"}):
        with pytest.raises(RuntimeError, match="not finished"):
            namespace["_fresh_sidecar"](obj, "sidecar", outcome)
    assert text.reads == 0
    namespace["bpy"].data.texts = SimpleNamespace(get=lambda _name: None)
    with pytest.raises(RuntimeError, match="BUTTON_SIDECAR_NOT_FRESH"):
        namespace["_fresh_sidecar"](obj, "sidecar", {"FINISHED"})


@pytest.mark.parametrize(
    ("patch", "expected"),
    (
        ({"result": ["CANCELLED"]}, "BUTTON_OPERATOR_DID_NOT_FINISH"),
        ({"result": ["RUNNING_MODAL"]}, "BUTTON_OPERATOR_DID_NOT_FINISH"),
        ({"result": {"FINISHED"}}, "BUTTON_OPERATOR_DID_NOT_FINISH"),
        (
            {"sidecar_absent_before_operator": False},
            "BUTTON_SIDECAR_NOT_FRESH",
        ),
        (
            {"sidecar_fresh_after_operator": False},
            "BUTTON_SIDECAR_NOT_FRESH",
        ),
        (
            {"sidecar_fresh_after_operator": None},
            "BUTTON_SIDECAR_NOT_FRESH",
        ),
    ),
)
def test_final_sweep_consumer_rejects_forged_report(patch, expected):
    function = _function(_module(BUTTON_SWEEP), "_receipt_failure")
    namespace = {
        "_EXPECTED_CALLS_PER_DENSITY": 1,
        "_EXPECTED_TOTAL_CALLS": 1,
        "_FIELD_BUDGET_SECONDS": {},
        "_SWEEP_SCENARIO": "all_seams",
        "DENSITY_SWEEP_CALL_COUNT_MISMATCH": (
            "DENSITY_SWEEP_CALL_COUNT_MISMATCH"
        ),
        "BUTTON_OPERATOR_DID_NOT_FINISH": "BUTTON_OPERATOR_DID_NOT_FINISH",
        "BUTTON_SIDECAR_NOT_FRESH": "BUTTON_SIDECAR_NOT_FRESH",
        "SOURCE_MESH_FINGERPRINT_CHANGED": (
            "SOURCE_MESH_FINGERPRINT_CHANGED"
        ),
        "FIELD_PERFORMANCE_BUDGET_EXCEEDED": (
            "FIELD_PERFORMANCE_BUDGET_EXCEEDED"
        ),
    }
    exec(
        compile(
            ast.Module(body=[function], type_ignores=[]),
            str(BUTTON_SWEEP),
            "exec",
        ),
        namespace,
    )
    report = {
        "result": ["FINISHED"],
        "sidecar_absent_before_operator": True,
        "sidecar_fresh_after_operator": True,
        "mesh": "mesh",
        "scenario": "all_seams",
        **patch,
    }
    densities = [
        {
            "operator_invocations": 1,
            "source_fingerprints": [{"unchanged": True}],
            "reports": [report],
        }
    ]
    scene_identity = {"sha256": "a" * 64}
    assert (
        namespace["_receipt_failure"](
            densities,
            scene_identity,
            dict(scene_identity),
        )
        == expected
    )


def test_final_sweep_consumer_accepts_only_complete_fresh_finished_report():
    function = _function(_module(BUTTON_SWEEP), "_receipt_failure")
    namespace = {
        "_EXPECTED_CALLS_PER_DENSITY": 1,
        "_EXPECTED_TOTAL_CALLS": 1,
        "_FIELD_BUDGET_SECONDS": {},
        "_SWEEP_SCENARIO": "all_seams",
        "DENSITY_SWEEP_CALL_COUNT_MISMATCH": (
            "DENSITY_SWEEP_CALL_COUNT_MISMATCH"
        ),
        "BUTTON_OPERATOR_DID_NOT_FINISH": "BUTTON_OPERATOR_DID_NOT_FINISH",
        "BUTTON_SIDECAR_NOT_FRESH": "BUTTON_SIDECAR_NOT_FRESH",
        "SOURCE_MESH_FINGERPRINT_CHANGED": (
            "SOURCE_MESH_FINGERPRINT_CHANGED"
        ),
        "FIELD_PERFORMANCE_BUDGET_EXCEEDED": (
            "FIELD_PERFORMANCE_BUDGET_EXCEEDED"
        ),
    }
    exec(
        compile(
            ast.Module(body=[function], type_ignores=[]),
            str(BUTTON_SWEEP),
            "exec",
        ),
        namespace,
    )
    report = {
        "result": ["FINISHED"],
        "sidecar_absent_before_operator": True,
        "sidecar_fresh_after_operator": True,
        "mesh": "mesh",
        "scenario": "all_seams",
    }
    densities = [
        {
            "operator_invocations": 1,
            "source_fingerprints": [{"unchanged": True}],
            "reports": [report],
        }
    ]
    identity = {"sha256": "a" * 64}
    assert namespace["_receipt_failure"](
        densities,
        identity,
        dict(identity),
    ) is None


def test_direct_parity_rejects_every_spoofed_authority_field():
    function = _function(_module(BUTTON_SWEEP), "_verify_direct_identity")
    namespace = {
        "json": json,
        "MEASUREMENT_RULE": "REQUEST_POLICY_KNOBS_MEASURED_V1",
        "DensitySweepContractError": RuntimeError,
        "BUTTON_PARITY_MISMATCH": "BUTTON_PARITY_MISMATCH",
    }
    exec(
        compile(
            ast.Module(body=[function], type_ignores=[]),
            str(BUTTON_SWEEP),
            "exec",
        ),
        namespace,
    )
    policy = {
        "selection_policy_id": "SELECTION",
        "max_subturn_parameter_id": "PARAMETER",
        "max_subturn_value_id": "VALUE",
    }
    request_ids = ["request"]
    head = "1" * 40
    receipt = {
        "schema": "cftuv.envelope.runtime_metric_building_gate.v2",
        "measurement_rule": "REQUEST_POLICY_KNOBS_MEASURED_V1",
        "repository_head": head,
        "requested_density": 1,
        "effective_density": 1,
        **policy,
        "request_ids": request_ids,
        "acceptance_eligible": True,
        "python_safe_path": True,
        "status": "COMPLETE",
    }
    namespace["_verify_direct_identity"](
        receipt,
        policy,
        1,
        request_ids,
        head,
    )
    spoofs = {
        "measurement_rule": "OTHER",
        "requested_density": 0,
        "effective_density": 4,
        "selection_policy_id": "OTHER",
        "max_subturn_parameter_id": "OTHER",
        "max_subturn_value_id": "OTHER",
        "request_ids": ["stale"],
        "acceptance_eligible": False,
        "python_safe_path": False,
    }
    for key, value in spoofs.items():
        forged = {**receipt, key: value}
        with pytest.raises(RuntimeError, match="BUTTON_PARITY_MISMATCH"):
            namespace["_verify_direct_identity"](
                forged,
                policy,
                1,
                request_ids,
                head,
            )


def test_scene_identity_hash_catches_same_size_restored_mtime_spoof(tmp_path):
    module = _module(BUTTON_SWEEP)
    functions = [
        _function(module, "_stream_sha256"),
        _function(module, "_source_file_identity"),
    ]
    namespace = {"hashlib": __import__("hashlib"), "Path": Path}
    exec(
        compile(
            ast.Module(body=functions, type_ignores=[]),
            str(BUTTON_SWEEP),
            "exec",
        ),
        namespace,
    )
    scene = tmp_path / "scene.blend"
    scene.write_bytes(b"AAAA")
    before_stat = scene.stat()
    before = namespace["_source_file_identity"](scene)
    scene.write_bytes(b"BBBB")
    os.utime(
        scene,
        ns=(before_stat.st_atime_ns, before_stat.st_mtime_ns),
    )
    after = namespace["_source_file_identity"](scene)
    assert before["size"] == after["size"]
    assert before["mtime_ns"] == after["mtime_ns"]
    assert before["sha256"] != after["sha256"]
    assert before != after


def test_receipts_name_policy_knobs_fingerprints_parity_and_no_scene_save():
    combined = FIELD_GATE.read_text(encoding="utf-8") + BUTTON_SWEEP.read_text(
        encoding="utf-8"
    )
    for required in (
        "requested_density",
        "effective_density",
        "selection_policy_id",
        "max_subturn_value_id",
        "request_ids",
        "request_policy_knobs",
        "repository_head",
        "installer_stamp",
        "source_mesh_fingerprint_before",
        "source_mesh_fingerprint_after",
        "parity_projection",
        "semantic_digest",
        "python_safe_path",
        "sha256",
    ):
        assert required in combined
    assert "save_as_mainfile" not in combined
    assert "bpy.ops.wm.save" not in combined


def test_real_field_runs_fail_closed_without_python_safe_path():
    for path in (FIELD_GATE, BUTTON_SWEEP):
        source = path.read_text(encoding="utf-8")
        assert 'os.environ.get("PYTHONSAFEPATH") != "1"' in source
        assert "PYTHONSAFEPATH_REQUIRED" in source
    cycle = FIELD_CYCLE.read_text(encoding="utf-8")
    assert '$env:PYTHONSAFEPATH = "1"' in cycle


def test_batch_wrapper_keeps_density_argument_visible():
    source = FIELD_BAT.read_text(encoding="utf-8")
    assert "-Density 1" in source
    assert "%*" in source
    cycle = FIELD_CYCLE.read_text(encoding="utf-8")
    assert "INSTALL_SOURCE_NOT_CANONICAL" in cycle
    assert 'canonicalInstallRoot = "E:\\GITHUB\\CFTUV"' in cycle


# --------------------------------------------------------------------------
# Классификация ступени домена: власть одна, и она хостовая.
# --------------------------------------------------------------------------


HOST_EXPORT = ROOT / "cftuv" / "envelope_request_export.py"


def _frozenset_members(module: ast.Module, name: str) -> set[str]:
    """Имена членов множества `name = frozenset({A.x, A.y, ...})` из ИСХОДНИКА.

    Читается текстом, а не импортом: `cftuv.envelope_request_export` тянет
    `mathutils` через `cftuv.model` и вне Blender не импортируется вовсе.
    """

    for node in module.body:
        if not isinstance(node, ast.Assign):
            continue
        if not any(
            isinstance(item, ast.Name) and item.id == name for item in node.targets
        ):
            continue
        return {
            item.attr for item in ast.walk(node.value) if isinstance(item, ast.Attribute)
        }
    raise AssertionError(f"assignment {name} not found")


def test_field_gate_consumes_the_host_metric_stage_authority():
    """Ступень домена классифицирует ХОСТ, и только он.

    Инструмент держал собственное двухэлементное множество имён против шести у
    хоста, поэтому четыре отказа метрики — оба закрытия окна решётки,
    отсутствие степени двойки в нём и превышение бюджета невязки — расписка
    молча переносила на ступень COMPILE. Запись `DECISIONS.md` за 2026-07-31
    прямо это запрещает: множество отказов ступени METRIC названо ОДИН раз,
    чтобы разведение имени не переносило домен на другую ступень. Кнопка
    владельца всё это время классифицировала верно; врал только инструмент.

    Способ проверки — чтение исходника, как в `test_architecture.py`. Причина:
    охраняемое свойство — «у инструмента нет СВОЕГО множества имён», и это
    свойство исходника, а не поведения. Поведенческий тест прошёл бы и на
    инструменте, отрастившем второе множество в другом месте; и импортировать
    ни инструмент (bpy), ни хост (mathutils) вне Blender нельзя.
    """

    source = FIELD_GATE.read_text(encoding="utf-8")
    assert "METRIC_STAGE_OUTCOMES," in source, "инструмент не импортирует власть"
    assert "exc.outcome in METRIC_STAGE_OUTCOMES" in source

    # Собственного множества имён исходов в инструменте не остаётся.
    gate = _module(FIELD_GATE)
    literals = [
        node
        for node in ast.walk(gate)
        if isinstance(node, (ast.Set, ast.Tuple, ast.List))
        and any(
            isinstance(item, ast.Attribute)
            and isinstance(item.value, ast.Name)
            and item.value.id == "EnvelopeDebugHostOutcome"
            for item in node.elts
        )
    ]
    assert literals == [], ast.dump(literals[0]) if literals else ""

    # И потребляемая власть — та самая, у которой все шесть имён.
    members = _frozenset_members(_module(HOST_EXPORT), "METRIC_STAGE_OUTCOMES")
    assert members == {
        "ENVELOPE_DEBUG_EXACT_PLANAR_FRAME_UNAVAILABLE",
        "RUNTIME_NEAR_PLANAR_PROJECTION_POLICY_REQUIRED",
        "NEAR_PLANAR_RESIDUAL_BUDGET_EXCEEDED",
        "GRID_WINDOW_CLOSED",
        "NO_POWER_OF_TWO_STEP_IN_WINDOW",
        "NO_GRID_SCALE_RESTORES_RELATIONS",
    }


def test_field_gate_echoes_a_compact_receipt_line_not_the_whole_json(tmp_path):
    """Эхо расписки — строка, а не побайтовый дубликат только что записанного.

    Замер принципала: полный JSON в stdout — 51 тыс. символов на
    `building.002`, ~170 тыс. на `building` за прогон. Конвейер его не
    потребляет: `field_cycle.ps1` проверяет наличие ФАЙЛА и отдаёт его путь
    `field_cycle_summary.py`, который читает файл. Единственный потребитель
    stdout ворот — путь `--contract-only`, и он выходит ранним `SystemExit` до
    импорта bpy, то есть до этой строки вовсе.
    """

    source = FIELD_GATE.read_text(encoding="utf-8")
    assert "print(json.dumps(payload" not in source
    assert "print(_receipt_echo(output, payload))" in source

    function = _function(_module(FIELD_GATE), "_receipt_echo")
    namespace = {"hashlib": hashlib}
    exec(
        compile(
            ast.Module(body=[function], type_ignores=[]),
            str(FIELD_GATE),
            "exec",
        ),
        namespace,
    )
    # Дайджест берётся с ЗАПИСАННОГО файла, а не со строки перед записью:
    # `write_text` переводит `\n` в `os.linesep`, и на Windows дайджест строки
    # разошёлся бы с `sha256sum` файла.
    receipt = tmp_path / "run.json"
    receipt.write_text('{"status": "COMPLETE"}\n', encoding="utf-8")
    line = namespace["_receipt_echo"](
        receipt,
        {
            "status": "COMPLETE",
            "gate_failure": None,
            "runs": [
                {
                    "domains": [
                        {"stage": "RAW_READY"},
                        {"stage": "RAW_READY"},
                        {"stage": "METRIC_REJECTED"},
                    ]
                },
                {"domains": [{"stage": "COMPILE_REJECTED"}]},
                {"domains": []},
            ],
        },
    )
    assert "run.json" in line
    assert "status=COMPLETE" in line
    assert "gate_failure=None" in line
    assert "domains[COMPILE_REJECTED=1 METRIC_REJECTED=1 RAW_READY=2]" in line
    assert f"sha256={hashlib.sha256(receipt.read_bytes()).hexdigest()}" in line
    # Компактность — предмет требования, а не побочный эффект. Полная расписка
    # в поле — 51 тыс. символов на `building.002`, ~170 тыс. на `building`.
    assert len(line) < 300


# --------------------------------------------------------------------------
# Регистрация аддона в свипе: идемпотентна на уровне СВИПА, и кем — в расписке.
# --------------------------------------------------------------------------


def _sweep_classes(*flags):
    """Классы аддона в полевой семантике: истину знает `cls.is_registered`.

    Имя класса намеренно НЕ попадает никуда, кроме самого класса: `bpy.types`
    его не публикует, и мок обязан это воспроизводить, а не опровергать.
    """

    return tuple(
        type(f"HOTSPOTUV_Fake{index}", (), {"is_registered": flag})
        for index, flag in enumerate(flags)
    )


def _sweep_registration(classes, recorder):
    """Выполнить обе функции регистрации свипа в подставном окружении.

    Функции берутся ИЗ ИСХОДНИКА (`ast`) и исполняются с подставными `bpy` и
    `cftuv`: настоящие требуют Blender, а проверяемое свойство — чистая логика
    трёх состояний, и она обязана быть проверяема без него.

    `bpy.types` здесь ПУСТ, и это не упрощение, а полевой факт: зонд в
    настоящем Blender при зарегистрированном аддоне вернул `None` для каждого
    `getattr(bpy.types, cls.__name__)`. Пустой `bpy.types` рядом с
    `is_registered == True` — ровно то состояние, в котором прежний предикат
    отвечал «ничего не зарегистрировано», и мок обязан его удерживать.
    """

    module = _module(BUTTON_SWEEP)
    namespace = {
        "bpy": SimpleNamespace(types=SimpleNamespace()),
        "cftuv": SimpleNamespace(register=recorder),
        "DensitySweepContractError": RuntimeError,
        "ADDON_REGISTRATION_PARTIAL": "ADDON_REGISTRATION_PARTIAL",
        "ADDON_REGISTERED_BY_STARTUP_PREFERENCES": "startup_preferences",
        "ADDON_REGISTERED_BY_SWEEP": "sweep",
    }
    for name in ("_registered_cftuv_classes", "_ensure_addon_registered"):
        function = _function(module, name)
        exec(
            compile(
                ast.Module(body=[function], type_ignores=[]),
                str(BUTTON_SWEEP),
                "exec",
            ),
            namespace,
        )
    # `from cftuv.operators import classes` внутри функций — подставляем модуль.
    # Прежнее значение ВОЗВРАЩАЕТСЯ, а не удаляется: `cftuv.operators` мог быть
    # уже импортирован другим тестом, и `del` вынес бы настоящий модуль из
    # `sys.modules` — соседний файл падал бы из-за подстановки в этом.
    previous = sys.modules.get("cftuv.operators")
    sys.modules["cftuv.operators"] = SimpleNamespace(classes=classes)
    try:
        return namespace["_ensure_addon_registered"]()
    finally:
        if previous is None:
            sys.modules.pop("cftuv.operators", None)
        else:
            sys.modules["cftuv.operators"] = previous


def test_sweep_registers_the_addon_only_when_it_is_not_registered_yet():
    """Свип обязан работать И с автовключением из настроек, И без него.

    Полевой факт: владелец включил CFTUV в настройках Blender, настройки
    автосохраняются, и теперь любой фоновый Blender регистрирует аддон на
    старте. Безусловный `cftuv.register()` после этого падает
    `already registered as a subclass 'HOTSPOTUV_AddonPreferences'` — до первой
    кнопки, то есть до всякого измерения.
    """

    calls = []
    # Ничего не зарегистрировано — регистрирует свип.
    assert _sweep_registration(
        _sweep_classes(False, False), lambda: calls.append(1)
    ) == "sweep"
    assert calls == [1]

    # Всё уже зарегистрировано настройками — свип не трогает ничего.
    calls.clear()
    assert _sweep_registration(
        _sweep_classes(True, True), lambda: calls.append(1)
    ) == "startup_preferences"
    assert calls == []


def test_sweep_refuses_a_half_registered_addon_by_name():
    """Частичная регистрация — названный отказ, а не выбор наугад.

    Зарегистрирована часть классов: звать `register()` нельзя (упадёт на уже
    зарегистрированных), пропустить нельзя (недостающие операторы не
    появятся). Предикат обязан различать это состояние, иначе полусломанная
    регистрация читалась бы как «уже есть».
    """

    calls = []
    with pytest.raises(RuntimeError, match="ADDON_REGISTRATION_PARTIAL"):
        _sweep_registration(
            _sweep_classes(True, False), lambda: calls.append(1)
        )
    assert calls == []


def test_sweep_registration_asks_the_class_not_bpy_types_by_name():
    """Предикат — `cls.is_registered`, и `bpy.types` он не спрашивает вовсе.

    ЗДЕСЬ БЫЛ ДЕФЕКТ, и дефект был в МОКЕ. Прежний тест строил `bpy.types`,
    публикующий классы по их именам (`setattr(fake, cls.__name__, cls)`), и на
    такой подставке предикат `getattr(bpy.types, cls.__name__, None) is cls`
    проходил все три состояния зелёным. Мок утверждал семантику, которой в
    Blender НЕТ: полевой зонд принципала в настоящем Blender — аддон
    зарегистрирован стартом, prefs-автовключение активно — вернул `None` для
    КАЖДОГО класса `cftuv.operators.classes`: ни операторов, ни
    `PropertyGroup`, ни `AddonPreferences`. `bpy.types` не публикует классы по
    именам Python-классов. Тождество было тождественно ЛОЖНЫМ, свип читал
    «ничего не зарегистрировано», звал `register()` и падал тем же
    `ValueError: already registered as a subclass 'HOTSPOTUV_AddonPreferences'`.
    Тем же зондом `cls.is_registered` вернул `True` у всех.

    Урок предметный: мок, придуманный из головы, проверяет придуманную
    семантику и красит её зелёным. Здесь `bpy.types` ПУСТ — как в поле, — и
    зелёный цвет обязан держаться на `is_registered`, а не на устройстве
    подставки.
    """

    predicate = _function(_module(BUTTON_SWEEP), "_registered_cftuv_classes")
    # Смотрим на КОД, а не на текст: докстрока называет прежний предикат как
    # запись о дефекте, и это не его возвращение.
    attributes = {
        node.attr for node in ast.walk(predicate) if isinstance(node, ast.Attribute)
    }
    names = {node.id for node in ast.walk(predicate) if isinstance(node, ast.Name)}
    assert "is_registered" in attributes
    assert "bpy" not in names, "предикат не должен спрашивать bpy.types"
    assert "getattr" not in names, "предикат спрашивает класс напрямую"

    # Полевое состояние: зарегистрированы все, а `bpy.types` пуст.
    calls = []
    assert _sweep_registration(
        _sweep_classes(True, True, True), lambda: calls.append(1)
    ) == "startup_preferences"
    assert calls == []


def test_sweep_owns_registration_idempotence_and_records_who_registered():
    """Идемпотентность живёт в СВИПЕ, а `cftuv.register()` не тронут.

    Прятать двойную регистрацию внутри продукта запрещено: там она честный
    дефект, и Blender сообщает о ней по имени. А перечень классов свип обязан
    брать у продукта — собственная копия имён разошлась бы с ним ровно так же,
    как разошлось множество имён ступени METRIC.
    """

    source = BUTTON_SWEEP.read_text(encoding="utf-8")
    assert "addon_registration = _ensure_addon_registered()" in source
    assert '"addon_registration": addon_registration,' in source
    # Перечень классов берётся у продукта, а не переписан в инструменте.
    assert "from cftuv.operators import classes" in source

    # `cftuv.register()` зовётся РОВНО ОДИН раз и только из-под предиката.
    # Строковая проверка здесь не годится: законный вызов внутри
    # `_ensure_addon_registered` выглядит так же, как прежний безусловный.
    module = _module(BUTTON_SWEEP)
    guarded = _function(module, "_ensure_addon_registered")
    callers = [
        node.name
        for node in module.body
        if isinstance(node, ast.FunctionDef)
        and any(
            isinstance(call.func, ast.Attribute)
            and call.func.attr == "register"
            and isinstance(call.func.value, ast.Name)
            and call.func.value.id == "cftuv"
            for call in ast.walk(node)
            if isinstance(call, ast.Call)
        )
    ]
    assert callers == [guarded.name], callers

    # Сам продукт не получил обхода двойной регистрации: там она честный
    # дефект, о котором Blender сообщает по имени. Два очевидных способа её
    # спрятать — предикат наличия и проглоченное исключение — оба закрыты.
    operators = (ROOT / "cftuv" / "operators.py").read_text(encoding="utf-8")
    registration = _function(ast.parse(operators), "register")
    hidden = [
        node
        for node in ast.walk(registration)
        if (
            isinstance(node, ast.Call)
            and isinstance(node.func, ast.Name)
            and node.func.id in {"getattr", "hasattr"}
        )
        or isinstance(node, ast.ExceptHandler)
    ]
    assert hidden == [], "продукт не должен прятать двойную регистрацию"


# --------------------------------------------------------------------------
# Охват свипа: три ИМЕНОВАННЫХ меша, а не то, что нашлось в живой сцене.
# --------------------------------------------------------------------------


def _sweep_meshes(objects_by_name):
    """`_swept_meshes` в подставном `bpy.data.objects`."""

    function = _function(_module(BUTTON_SWEEP), "_swept_meshes")
    namespace = {
        "bpy": SimpleNamespace(
            data=SimpleNamespace(objects=SimpleNamespace(get=objects_by_name.get))
        ),
        "DensitySweepContractError": RuntimeError,
        "SWEPT_MESH_ABSENT": "SWEPT_MESH_ABSENT",
        "_SWEPT_MESH_NAMES": ("building", "building.002", "building.004"),
    }
    exec(
        compile(
            ast.Module(body=[function], type_ignores=[]),
            str(BUTTON_SWEEP),
            "exec",
        ),
        namespace,
    )
    return namespace["_swept_meshes"]()


def _mesh(name):
    return SimpleNamespace(name=name, type="MESH")


def test_sweep_takes_the_three_named_meshes_in_declared_order():
    """Охват — свойство ИНСТРУМЕНТА, а не содержимого сцены.

    Посторонние меши со швами в сцене больше не расширяют охват, и их
    появление не двигает контракт 3 × 1 × 3.
    """

    scene = {name: _mesh(name) for name in
             ("building", "building.002", "building.004", "building.007", "Cube")}
    assert [obj.name for obj in _sweep_meshes(scene)] == [
        "building",
        "building.002",
        "building.004",
    ]


@pytest.mark.parametrize(
    "absent",
    ("building", "building.002", "building.004"),
)
def test_sweep_refuses_a_missing_named_mesh_by_name(absent):
    """Пропавший меш — событие, а не молча суженный охват.

    Прежний свип выводил норму из живой сцены и о неё же ломался. Новый
    норму объявляет, поэтому расхождение с ней обязано быть названо: расписка
    на два меша, выданная за расписку контракта, солгала бы о том, что мерили.
    """

    scene = {
        name: _mesh(name)
        for name in ("building", "building.002", "building.004")
        if name != absent
    }
    with pytest.raises(RuntimeError, match="SWEPT_MESH_ABSENT"):
        _sweep_meshes(scene)
    # И не-меш под тем же именем — тоже отсутствие, а не подмена.
    scene[absent] = SimpleNamespace(name=absent, type="EMPTY")
    with pytest.raises(RuntimeError, match="SWEPT_MESH_ABSENT"):
        _sweep_meshes(scene)


def test_sweep_runs_one_scenario_and_verifies_parity_for_every_swept_mesh():
    """`single_edge` снят, а сверка с прямой распиской осталась у всех трёх.

    Прежде условием сверки было попадание меша в таблицу бюджетов: охват
    сверки и потолок времени держались одним ключом, и меш без бюджета молча
    оставался несверенным.
    """

    source = BUTTON_SWEEP.read_text(encoding="utf-8")
    assert "if obj.name in _FIELD_BUDGET_SECONDS:" not in source

    module = _module(BUTTON_SWEEP)
    # Проверяется КОД, а не текст: `single_edge` остаётся в комментарии как
    # запись о снятом сценарии, и это не его возвращение.
    scenarios = {
        node.value
        for node in ast.walk(module)
        if isinstance(node, ast.Constant) and node.value == "single_edge"
    }
    assert scenarios == set(), "сценарий single_edge снят решением владельца"

    density_run = _function(module, "_density_run")
    operator_calls = [
        node
        for node in ast.walk(density_run)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Name)
        and node.func.id == "_run_operator"
    ]
    parity_calls = [
        node
        for node in ast.walk(density_run)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Name)
        and node.func.id == "_verify_direct_parity"
    ]
    # По одной кнопке и по одной сверке на меш — обе безусловны в теле цикла.
    assert len(operator_calls) == 1
    assert len(parity_calls) == 1
