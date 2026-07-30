"""Density acceptance: настоящая кнопка на каждом меше сцены, d=0/1/4."""

from __future__ import annotations

import hashlib
import json
import os
from pathlib import Path
import re
import subprocess
import sys
import traceback


ROOT = Path(__file__).resolve().parents[1]
RECEIPT_SCHEMA_V2 = "cftuv.envelope.density_button_sweep.v2"
MEASUREMENT_RULE = "REQUEST_POLICY_KNOBS_MEASURED_V1"
UNMEASURED_REQUEST_POLICY_KNOB = "UNMEASURED_REQUEST_POLICY_KNOB"
INSTALLER_STAMP_MISSING = "INSTALLER_STAMP_MISSING"
INSTALLER_STAMP_MISMATCH = "INSTALLER_STAMP_MISMATCH"
BUTTON_PARITY_MISMATCH = "BUTTON_PARITY_MISMATCH"
BUTTON_OPERATOR_DID_NOT_FINISH = "BUTTON_OPERATOR_DID_NOT_FINISH"
BUTTON_SIDECAR_NOT_FRESH = "BUTTON_SIDECAR_NOT_FRESH"
SOURCE_MESH_FINGERPRINT_CHANGED = "SOURCE_MESH_FINGERPRINT_CHANGED"
DENSITY_SWEEP_CALL_COUNT_MISMATCH = "DENSITY_SWEEP_CALL_COUNT_MISMATCH"
FIELD_PERFORMANCE_BUDGET_EXCEEDED = "FIELD_PERFORMANCE_BUDGET_EXCEEDED"
PYTHONSAFEPATH_REQUIRED = "PYTHONSAFEPATH_REQUIRED"
_DENSITY_MATRIX = (0, 1, 4)
_EXPECTED_CALLS_PER_DENSITY = 26
_EXPECTED_TOTAL_CALLS = 78
_FIELD_BUDGET_SECONDS = {
    "building.002": 5.0,
    "2": 17.3,
    "building": 120.0,
}
_STAMP_PATTERN = re.compile(
    r"commit\s+([0-9a-f]{7,40})\s+\(([^)]+)\)",
)


class DensitySweepContractError(RuntimeError):
    """Fail-closed нарушение приёмки настоящей кнопкой."""


def _arguments_after_separator(argv=None) -> tuple[str, ...]:
    values = tuple(sys.argv if argv is None else argv)
    return values[values.index("--") + 1 :] if "--" in values else ()


def _parse_density_matrix(value: str | None) -> tuple[int, ...]:
    if value != "0,1,4":
        raise DensitySweepContractError(
            f"{UNMEASURED_REQUEST_POLICY_KNOB}: matrix must be exactly 0,1,4"
        )
    try:
        matrix = tuple(int(item) for item in value.split(","))
    except (AttributeError, ValueError) as exc:
        raise DensitySweepContractError(
            f"{UNMEASURED_REQUEST_POLICY_KNOB}: density matrix is required"
        ) from exc
    if matrix != _DENSITY_MATRIX:
        raise DensitySweepContractError(
            f"{UNMEASURED_REQUEST_POLICY_KNOB}: matrix must be exactly 0,1,4"
        )
    return matrix


def _parsed_sweep_arguments(arguments) -> dict:
    if len(arguments) != 3:
        raise DensitySweepContractError(
            f"{UNMEASURED_REQUEST_POLICY_KNOB}: "
            "output, matrix and direct receipt directory are required"
        )
    matrix = _parse_density_matrix(arguments[1])
    return {
        "output": arguments[0],
        "density_matrix": list(matrix),
        "direct_receipt_directory": arguments[2],
        "expected_calls_per_density": _EXPECTED_CALLS_PER_DENSITY,
        "expected_total_calls": _EXPECTED_TOTAL_CALLS,
    }


def _contract_only() -> None:
    print(
        json.dumps(
            {
                "schema": RECEIPT_SCHEMA_V2,
                "measurement_rule": MEASUREMENT_RULE,
                **_parsed_sweep_arguments(_arguments_after_separator()),
            },
            sort_keys=True,
        )
    )


if __name__ == "__main__" and "--contract-only" in sys.argv:
    try:
        _contract_only()
    except DensitySweepContractError as exc:
        raise SystemExit(str(exc)) from exc
    raise SystemExit(0)

if __name__ == "__main__" and os.environ.get("PYTHONSAFEPATH") != "1":
    raise SystemExit(
        f"{PYTHONSAFEPATH_REQUIRED}: kernel field runs require PYTHONSAFEPATH=1"
    )


import bmesh
import bpy

import cftuv
import cftuv_envelope as kernel
from cftuv.envelope_debug_renderer import envelope_debug_text_name
from cftuv.envelope_request_policy import envelope_angular_policy


def _git_text(*arguments: str) -> str:
    try:
        return subprocess.check_output(
            ("git", "-C", str(ROOT), *arguments),
            text=True,
            encoding="utf-8",
        ).strip()
    except (OSError, subprocess.CalledProcessError) as exc:
        raise DensitySweepContractError(
            f"{UNMEASURED_REQUEST_POLICY_KNOB}: repository identity unavailable"
        ) from exc


def _repository_identity() -> dict:
    head = _git_text("rev-parse", "HEAD")
    branch = _git_text("rev-parse", "--abbrev-ref", "HEAD")
    if not re.fullmatch(r"[0-9a-f]{40}", head):
        raise DensitySweepContractError(
            f"{UNMEASURED_REQUEST_POLICY_KNOB}: invalid repository HEAD"
        )
    return {"head": head, "branch": branch}


def _package_fingerprint(root: Path) -> str:
    digest = hashlib.sha256()
    for path in sorted(root.rglob("*.py")):
        if "__pycache__" in path.parts:
            continue
        digest.update(path.relative_to(root).as_posix().encode("utf-8"))
        digest.update(path.read_bytes().replace(b"\r\n", b"\n"))
    return digest.hexdigest()[:16]


def _installer_identity(repository: dict) -> dict:
    scripts = Path(bpy.utils.user_resource("SCRIPTS"))
    addon_root = (scripts / "addons" / "cftuv").resolve()
    kernel_root = (scripts / "modules" / "cftuv_envelope").resolve()
    loaded_addon_root = Path(cftuv.__file__).resolve().parent
    loaded_kernel_root = Path(kernel.__file__).resolve().parent
    if loaded_addon_root != addon_root or loaded_kernel_root != kernel_root:
        raise DensitySweepContractError(
            f"{INSTALLER_STAMP_MISMATCH}: repository path shadows installed product"
        )
    addon_path = addon_root / "_install_stamp.txt"
    kernel_path = kernel_root / "_install_stamp.txt"
    try:
        addon_stamp = addon_path.read_text(encoding="utf-8-sig").strip()
        kernel_stamp = kernel_path.read_text(encoding="utf-8-sig").strip()
    except (OSError, UnicodeError) as exc:
        raise DensitySweepContractError(
            f"{INSTALLER_STAMP_MISSING}: addon/kernel stamp is required"
        ) from exc
    found = _STAMP_PATTERN.search(addon_stamp)
    if (
        not addon_stamp
        or addon_stamp != kernel_stamp
        or found is None
        or not repository["head"].startswith(found.group(1))
    ):
        raise DensitySweepContractError(
            f"{INSTALLER_STAMP_MISMATCH}: stamp does not match full HEAD"
        )
    fingerprints = {
        "cftuv_repository": _package_fingerprint(ROOT / "cftuv"),
        "cftuv_installed": _package_fingerprint(addon_root),
        "kernel_repository": _package_fingerprint(
            ROOT / "kernel" / "src" / "cftuv_envelope"
        ),
        "kernel_installed": _package_fingerprint(kernel_root),
    }
    if (
        fingerprints["cftuv_repository"] != fingerprints["cftuv_installed"]
        or fingerprints["kernel_repository"] != fingerprints["kernel_installed"]
    ):
        raise DensitySweepContractError(
            f"{INSTALLER_STAMP_MISMATCH}: installed fingerprints differ"
        )
    return {
        "raw": addon_stamp,
        "commit": found.group(1),
        "branch": found.group(2),
        "matched_repository_head": True,
        "loaded_cftuv_path": str(Path(cftuv.__file__).resolve()),
        "loaded_kernel_path": str(Path(kernel.__file__).resolve()),
        "fingerprints": fingerprints,
    }


def _identity_value(value) -> str:
    return str(value.value) if hasattr(value, "value") else str(value)


def _policy_receipt(density: int) -> dict:
    policy = envelope_angular_policy(kernel, density)
    return {
        "requested_density": density,
        "effective_density": policy.density,
        "selection_policy_id": _identity_value(policy.selection_policy_id),
        "max_subturn_parameter_id": _identity_value(policy.parameter_id),
        "max_subturn_value_id": _identity_value(policy.value_id),
    }


def _mesh_fingerprint(obj) -> str:
    mesh = obj.data
    payload = {
        "vertices": [
            (vertex.index, tuple(float(value) for value in vertex.co))
            for vertex in mesh.vertices
        ],
        "edges": [
            (
                edge.index,
                tuple(int(value) for value in edge.vertices),
                bool(edge.use_seam),
            )
            for edge in mesh.edges
        ],
        "faces": [
            (polygon.index, tuple(int(value) for value in polygon.vertices))
            for polygon in mesh.polygons
        ],
    }
    encoded = json.dumps(
        payload,
        sort_keys=True,
        separators=(",", ":"),
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def _settings():
    return bpy.context.scene.hotspotuv_settings


def _select_seams(obj, edge_indices=None):
    bpy.context.view_layer.objects.active = obj
    if bpy.context.mode != "OBJECT":
        bpy.ops.object.mode_set(mode="OBJECT")
    for other in bpy.context.selected_objects:
        other.select_set(False)
    obj.select_set(True)
    bpy.ops.object.mode_set(mode="EDIT")
    bpy.context.tool_settings.mesh_select_mode = (False, True, False)
    bm = bmesh.from_edit_mesh(obj.data)
    bm.edges.ensure_lookup_table()
    for edge in bm.edges:
        edge.select_set(False)
    wanted = 0
    for edge in bm.edges:
        seam = edge.seam if edge_indices is None else edge.index in edge_indices
        if seam:
            edge.select_set(True)
            wanted += 1
    bmesh.update_edit_mesh(obj.data)
    return wanted


def _invalidate_sidecar(obj) -> str:
    text_name = envelope_debug_text_name(obj)
    previous = bpy.data.texts.get(text_name)
    if previous is not None:
        bpy.data.texts.remove(previous)
    if bpy.data.texts.get(text_name) is not None:
        raise DensitySweepContractError(BUTTON_SIDECAR_NOT_FRESH)
    return text_name


def _require_finished(outcome) -> None:
    if type(outcome) is not set or outcome != {"FINISHED"}:
        raise DensitySweepContractError(
            f"{BUTTON_OPERATOR_DID_NOT_FINISH}: {outcome!r}"
        )


def _sidecar_identity(payload: dict, object_name: str) -> tuple[str, ...]:
    if type(payload) is not dict:
        raise DensitySweepContractError(
            f"{BUTTON_SIDECAR_NOT_FRESH}: payload is not an exact object"
        )
    request_ids = payload.get("decal_request_ids")
    if (
        payload.get("object_name") != object_name
        or type(request_ids) is not list
        or len(request_ids) != 1
        or type(request_ids[0]) is not str
        or re.fullmatch(
            r"host-v0:decal-request-density:[0-9a-f]{24}",
            request_ids[0],
        )
        is None
    ):
        raise DensitySweepContractError(
            f"{BUTTON_SIDECAR_NOT_FRESH}: object/request identity differs"
        )
    return tuple(request_ids)


def _fresh_sidecar(obj, text_name: str, outcome) -> dict:
    _require_finished(outcome)
    text = bpy.data.texts.get(text_name)
    if text is None:
        raise DensitySweepContractError(
            f"{BUTTON_SIDECAR_NOT_FRESH}: sidecar absent after FINISHED"
        )
    try:
        payload = json.loads(text.as_string())
    except (ValueError, TypeError) as exc:
        raise DensitySweepContractError(BUTTON_SIDECAR_NOT_FRESH) from exc
    _sidecar_identity(payload, obj.name)
    return payload


def _queue_semantic_digest(payload: dict) -> str:
    semantic = {
        key: value
        for key, value in payload.items()
        if key
        not in {
            "prepare_seconds",
            "coverage_seconds",
            "contour_seconds",
            "timings",
        }
    }
    encoded = json.dumps(
        semantic,
        ensure_ascii=False,
        sort_keys=True,
        separators=(",", ":"),
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def _button_parity_projection(sidecar: dict) -> list[dict]:
    projection = []
    queue = sidecar.get("queue") or {}
    for domain in queue.get("domains", ()):
        preparation = domain.get("preparation_outcome")
        coverage = domain.get("coverage_outcome")
        if preparation != "EXACT":
            stage = "QUEUE_PREPARE_REJECTED"
        elif coverage != "EXACT":
            stage = "QUEUE_COVERAGE_REJECTED"
        else:
            stage = "QUEUE_RESOLVED"
        projection.append(
            {
                "patch_domain_id": domain["patch_domain_id"],
                "stage": stage,
                "preparation_outcome": preparation,
                "coverage_outcome": coverage,
                "resolved": stage == "QUEUE_RESOLVED",
                "semantic_digest": _queue_semantic_digest(domain),
            }
        )
    return sorted(projection, key=lambda item: item["patch_domain_id"])


def _queue_seconds(sidecar: dict) -> float:
    return sum(
        (item.get("prepare_seconds") or 0.0)
        + (item.get("coverage_seconds") or 0.0)
        + (item.get("contour_seconds") or 0.0)
        for item in (sidecar.get("queue") or {}).get("domains", ())
    )


def _run_operator(obj, label, density: int, edge_indices=None) -> dict:
    report = {
        "mesh": obj.name,
        "scenario": label,
        **_policy_receipt(density),
        "operator_invoked": False,
    }
    try:
        selected = _select_seams(obj, edge_indices)
        report["selected_edges"] = selected
        if not selected:
            report["result"] = "SKIP_NO_SELECTION"
            return report
        settings = _settings()
        settings.envelope_debug_engine = "QUEUE"
        settings.envelope_debug_fan_density = str(density)
        if settings.envelope_debug_fan_density != str(density):
            raise DensitySweepContractError(
                f"{UNMEASURED_REQUEST_POLICY_KNOB}: UI density readback mismatch"
            )
        text_name = _invalidate_sidecar(obj)
        report["sidecar_absent_before_operator"] = True
        report["operator_invoked"] = True
        outcome = bpy.ops.hotspotuv.build_exact_reference_envelope_debug()
        report["result"] = sorted(outcome)
        sidecar = _fresh_sidecar(obj, text_name, outcome)
        report["sidecar_fresh_after_operator"] = True
        report["request_ids"] = list(_sidecar_identity(sidecar, obj.name))
        report["requested_alpha"] = sidecar.get("requested_alpha")
        report["request_policy_knobs"] = {
            "density": {
                "requested": density,
                "effective": int(settings.envelope_debug_fan_density),
                "id": report["selection_policy_id"],
            },
            "alpha": {
                "requested": sidecar.get("requested_alpha"),
                "effective": sidecar.get("requested_alpha"),
                "id": "requested_alpha",
            },
        }
        report["parity_projection"] = _button_parity_projection(sidecar)
        report["queue_seconds"] = _queue_seconds(sidecar)
        for key in (
            "envelope_debug_status",
            "envelope_debug_outcome",
            "envelope_debug_domain_status",
            "envelope_debug_stage_summary",
            "envelope_debug_queue_timing",
        ):
            report[key] = getattr(settings, key, "")
    except Exception:
        report["result"] = "EXCEPTION"
        report["traceback"] = traceback.format_exc(limit=8)
    finally:
        try:
            bpy.ops.object.mode_set(mode="OBJECT")
        except Exception:
            pass
    return report


def _slug(value: str) -> str:
    return re.sub(r"[^0-9A-Za-z]+", "_", value).lower()


def _direct_receipt_path(directory: Path, object_name: str, density: int) -> Path:
    return directory / f"{_slug(object_name)}_density_{density}_field_run.json"


def _verify_direct_parity(
    directory: Path,
    object_name: str,
    density: int,
    button_projection,
    button_request_ids,
    repository_head: str,
) -> dict:
    path = _direct_receipt_path(directory, object_name, density)
    try:
        receipt = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, ValueError) as exc:
        raise DensitySweepContractError(
            f"{BUTTON_PARITY_MISMATCH}: direct receipt unavailable {path}"
        ) from exc
    expected_policy = _policy_receipt(density)
    _verify_direct_identity(
        receipt,
        expected_policy,
        density,
        button_request_ids,
        repository_head,
    )
    run = next(
        (
            item
            for item in receipt.get("runs", ())
            if item.get("scope") == "all_seam_chains_l0"
        ),
        None,
    )
    direct_projection = None if run is None else run.get("parity_projection")
    if direct_projection != button_projection:
        raise DensitySweepContractError(
            f"{BUTTON_PARITY_MISMATCH}: domain projection differs"
        )
    return {
        "direct_receipt": str(path),
        "request_ids": button_request_ids,
        "domain_count": len(button_projection),
        "resolved_count": sum(item["resolved"] for item in button_projection),
        "semantic_digests": [
            item["semantic_digest"] for item in button_projection
        ],
        "matched": True,
    }


def _verify_direct_identity(
    receipt: dict,
    expected_policy: dict,
    density: int,
    button_request_ids,
    repository_head: str,
) -> None:
    expected = {
        "schema": "cftuv.envelope.runtime_metric_building_gate.v2",
        "measurement_rule": MEASUREMENT_RULE,
        "repository_head": repository_head,
        "requested_density": density,
        "effective_density": density,
        "selection_policy_id": expected_policy["selection_policy_id"],
        "max_subturn_parameter_id": expected_policy[
            "max_subturn_parameter_id"
        ],
        "max_subturn_value_id": expected_policy["max_subturn_value_id"],
        "request_ids": button_request_ids,
        "acceptance_eligible": True,
        "python_safe_path": True,
        "status": "COMPLETE",
    }
    actual = {key: receipt.get(key) for key in expected}
    actual_encoded = json.dumps(
        actual,
        ensure_ascii=False,
        sort_keys=True,
        separators=(",", ":"),
    )
    expected_encoded = json.dumps(
        expected,
        ensure_ascii=False,
        sort_keys=True,
        separators=(",", ":"),
    )
    if actual_encoded != expected_encoded:
        raise DensitySweepContractError(
            f"{BUTTON_PARITY_MISMATCH}: direct receipt identity differs"
        )


def _stream_sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        while True:
            block = stream.read(1024 * 1024)
            if not block:
                break
            digest.update(block)
    return digest.hexdigest()


def _source_file_identity(path: Path) -> dict:
    stat = path.stat()
    return {
        "path": str(path),
        "size": stat.st_size,
        "mtime_ns": stat.st_mtime_ns,
        "sha256": _stream_sha256(path),
    }


def _density_run(
    density: int,
    meshes,
    direct_directory: Path,
    repository_head: str,
) -> dict:
    reports = []
    fingerprints = []
    parity = []
    for obj in meshes:
        before = _mesh_fingerprint(obj)
        all_seams = _run_operator(obj, "all_seams", density)
        first_seam = next(
            (edge.index for edge in obj.data.edges if edge.use_seam),
            None,
        )
        single_edge = _run_operator(
            obj,
            "single_edge",
            density,
            edge_indices=set() if first_seam is None else {first_seam},
        )
        after = _mesh_fingerprint(obj)
        for report in (all_seams, single_edge):
            report["source_mesh_fingerprint_before"] = before
            report["source_mesh_fingerprint_after"] = after
            report["source_mesh_unchanged"] = before == after
        reports.extend((all_seams, single_edge))
        fingerprints.append(
            {
                "mesh": obj.name,
                "before": before,
                "after": after,
                "unchanged": before == after,
            }
        )
        if obj.name in _FIELD_BUDGET_SECONDS:
            parity.append(
                _verify_direct_parity(
                    direct_directory,
                    obj.name,
                    density,
                    all_seams.get("parity_projection"),
                    all_seams.get("request_ids"),
                    repository_head,
                )
            )
    invocations = sum(bool(item["operator_invoked"]) for item in reports)
    return {
        **_policy_receipt(density),
        "operator_invocations": invocations,
        "expected_operator_invocations": _EXPECTED_CALLS_PER_DENSITY,
        "source_fingerprints": fingerprints,
        "direct_button_parity": parity,
        "reports": reports,
    }


def _receipt_failure(densities, scene_before: dict, scene_after: dict) -> str | None:
    if any(
        item["operator_invocations"] != _EXPECTED_CALLS_PER_DENSITY
        for item in densities
    ):
        return DENSITY_SWEEP_CALL_COUNT_MISMATCH
    reports = [report for item in densities for report in item["reports"]]
    if len(reports) != _EXPECTED_TOTAL_CALLS:
        return DENSITY_SWEEP_CALL_COUNT_MISMATCH
    for report in reports:
        if (
            type(report.get("result")) is not list
            or report["result"] != ["FINISHED"]
        ):
            return BUTTON_OPERATOR_DID_NOT_FINISH
        if (
            report.get("sidecar_absent_before_operator") is not True
            or report.get("sidecar_fresh_after_operator") is not True
        ):
            return BUTTON_SIDECAR_NOT_FRESH
    if any(not item["unchanged"] for density in densities for item in density["source_fingerprints"]):
        return SOURCE_MESH_FINGERPRINT_CHANGED
    if scene_before != scene_after:
        return "SOURCE_SCENE_WAS_SAVED"
    for report in reports:
        budget = _FIELD_BUDGET_SECONDS.get(report["mesh"])
        if (
            report["scenario"] == "all_seams"
            and budget is not None
            and (report.get("queue_seconds") is None or report["queue_seconds"] > budget)
        ):
            return FIELD_PERFORMANCE_BUDGET_EXCEEDED
    return None


def main() -> None:
    parsed = _parsed_sweep_arguments(_arguments_after_separator())
    output = Path(parsed["output"])
    direct_directory = Path(parsed["direct_receipt_directory"])
    if not direct_directory.is_dir():
        raise DensitySweepContractError(
            f"{BUTTON_PARITY_MISMATCH}: direct receipt directory is absent"
        )
    repository = _repository_identity()
    installer = _installer_identity(repository)
    cftuv.register()
    scene_path = Path(bpy.data.filepath)
    scene_before = _source_file_identity(scene_path)
    meshes = [
        obj
        for obj in bpy.data.objects
        if obj.type == "MESH" and any(edge.use_seam for edge in obj.data.edges)
    ]
    if len(meshes) * 2 != _EXPECTED_CALLS_PER_DENSITY:
        raise DensitySweepContractError(DENSITY_SWEEP_CALL_COUNT_MISMATCH)
    densities = [
        _density_run(
            density,
            meshes,
            direct_directory,
            repository["head"],
        )
        for density in parsed["density_matrix"]
    ]
    scene_after = _source_file_identity(scene_path)
    failure = _receipt_failure(densities, scene_before, scene_after)
    payload = {
        "schema": RECEIPT_SCHEMA_V2,
        "measurement_rule": MEASUREMENT_RULE,
        "repository_head": repository["head"],
        "repository_branch": repository["branch"],
        "installer_stamp": installer,
        "python_safe_path": True,
        "requested_density_matrix": parsed["density_matrix"],
        "expected_calls_per_density": _EXPECTED_CALLS_PER_DENSITY,
        "expected_total_calls": _EXPECTED_TOTAL_CALLS,
        "actual_total_calls": sum(
            item["operator_invocations"] for item in densities
        ),
        "source_scene_before": scene_before,
        "source_scene_after": scene_after,
        "scene_saved": scene_before != scene_after,
        "status": "COMPLETE" if failure is None else "REJECTED",
        "gate_failure": failure,
        "densities": densities,
    }
    text = json.dumps(payload, ensure_ascii=False, indent=1, sort_keys=True)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(text + "\n", encoding="utf-8")
    for density in densities:
        for report in density["reports"]:
            print(
                f"SWEEP d={density['effective_density']} "
                f"{report['mesh']:<28} {report['scenario']:<12} "
                f"{report.get('result')}"
            )
    print("SWEEP_DONE", payload["actual_total_calls"])
    if failure is not None:
        raise DensitySweepContractError(failure)


if __name__ == "__main__":
    main()
