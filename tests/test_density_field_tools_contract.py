from __future__ import annotations

import ast
import json
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


def test_button_sweep_contract_counts_exactly_78_true_button_calls():
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
    assert payload["density_matrix"] == [0, 1, 4]
    assert payload["expected_calls_per_density"] == 26
    assert payload["expected_total_calls"] == 78


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


def test_budgets_are_unchanged_in_both_acceptance_tools():
    expected = {"building.002": 5.0, "2": 17.3, "building": 120.0}
    assert _assignment(_module(FIELD_GATE), "_FIELD_BUDGET_SECONDS") == expected
    assert _assignment(_module(BUTTON_SWEEP), "_FIELD_BUDGET_SECONDS") == expected


def test_sweep_sets_and_reads_back_ui_density_before_true_operator():
    source = BUTTON_SWEEP.read_text(encoding="utf-8")
    assignment = 'settings.envelope_debug_fan_density = str(density)'
    readback = 'settings.envelope_debug_fan_density != str(density)'
    operator = "bpy.ops.hotspotuv.build_exact_reference_envelope_debug()"
    assert source.index(assignment) < source.index(readback) < source.index(operator)
    assert source.count(operator) == 1


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
