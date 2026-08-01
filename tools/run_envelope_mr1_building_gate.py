"""Read-only Blender field gate for M-R1 on building.002."""

from __future__ import annotations

import hashlib
import json
import os
from pathlib import Path
import re
import subprocess
import sys
import time


RECEIPT_SCHEMA_V2 = "cftuv.envelope.runtime_metric_building_gate.v2"
MEASUREMENT_RULE = "REQUEST_POLICY_KNOBS_MEASURED_V1"
UNMEASURED_REQUEST_POLICY_KNOB = "UNMEASURED_REQUEST_POLICY_KNOB"
INSTALLER_STAMP_MISSING = "INSTALLER_STAMP_MISSING"
INSTALLER_STAMP_MISMATCH = "INSTALLER_STAMP_MISMATCH"
REQUEST_POLICY_EFFECTIVE_MISMATCH = "REQUEST_POLICY_EFFECTIVE_MISMATCH"
FIELD_PERFORMANCE_BUDGET_EXCEEDED = "FIELD_PERFORMANCE_BUDGET_EXCEEDED"
PYTHONSAFEPATH_REQUIRED = "PYTHONSAFEPATH_REQUIRED"
_DENSITY_ARGUMENTS = frozenset({"0", "1", "4", "legacy"})
_FIELD_BUDGET_SECONDS = {
    "building.002": 5.0,
    # 17.3 -> 30.0. Цена нового EXACT-домена: `2` d4 даёт 20/32 (было 19) —
    # зеркальный домен решился, и его глубокий веер стоит +16.4 с (прогон
    # 28.30 с). Кап поставлен РАВНЫМ полевой SLA-рамке `2` (total ≤ 30,
    # канонический кап из `DECISIONS.md` за 2026-07-30), а не выведен из
    # замера с запасом: рамка уже названа владельцем, и заводить рядом второе
    # число значило бы держать одну величину дважды. Рамка ДЕРЖИТСЯ — 28.30 из
    # 30, и это факт замера, а не округление в свою пользу.
    "2": 30.0,
    # 120 -> 180. Цена шести новых EXACT-доменов: после зеркальных вееров
    # `building` d4 даёт 111/122 EXACT (было 105) — шесть бывших
    # bridge-отказников теперь СЧИТАЮТСЯ, седьмой отказывает честным
    # `FACES_DID_NOT_ASSEMBLE`. По-доменный разрез принципала: семь бывших
    # отказников стоят 67.6 с, старые домены НЕ подорожали (82.1 с ≈ прежний
    # уровень), полный прогон 149.71 с. Кап = 82.1 + 67.6 + ~20% запаса.
    # Поднят по курсу владельца: ширина продукта важнее полировки скорости.
    # Это плата за СЧЁТ, а не за замедление — старая работа осталась прежней.
    #
    # 180 -> 240. Цена продуктового допуска кривизны 1/80 (T15): кривые домены,
    # которые прежде отвергались на METRIC, теперь СЧИТАЮТСЯ и стоят ~+49 с на
    # d4. Замер принципала: `building` d4 = 198.6 с при капе 180 — REJECTED;
    # d0/d1 влезали и в 180 (131.7 / 125.4). Кап = 198.6 + ~20% запаса.
    # Владелец ранее принял «~5 минут на `building` допустимо», и 240 внутри
    # этой рамки. Опять плата за СЧЁТ: дороже стало не прежнее, а то, что
    # раньше не считалось вовсе.
    "building": 240.0,
}
_STAMP_PATTERN = re.compile(
    r"commit\s+([0-9a-f]{7,40})\s+\(([^)]+)\)",
)


class DensityGateContractError(RuntimeError):
    """Fail-closed нарушение измерительного контракта Density."""


def _arguments_after_separator(argv=None) -> tuple[str, ...]:
    values = tuple(sys.argv if argv is None else argv)
    return values[values.index("--") + 1 :] if "--" in values else ()


def _parse_density_argument(value: str | None) -> tuple[int | str, int | None]:
    if value not in _DENSITY_ARGUMENTS:
        raise DensityGateContractError(
            f"{UNMEASURED_REQUEST_POLICY_KNOB}: "
            "explicit density must be one of 0,1,4,legacy"
        )
    if value == "legacy":
        return "legacy", None
    density = int(value)
    return density, density


def _parsed_gate_arguments(arguments) -> dict:
    if len(arguments) != 5:
        raise DensityGateContractError(
            f"{UNMEASURED_REQUEST_POLICY_KNOB}: exactly five gate arguments "
            "including density are required"
        )
    requested_density, effective_density = _parse_density_argument(arguments[4])
    return {
        "output": arguments[0],
        "scopes": arguments[1],
        "object_name": arguments[2],
        "engines": arguments[3],
        "requested_density": requested_density,
        "effective_density": effective_density,
        "acceptance_eligible": effective_density is not None,
    }


def _contract_only() -> None:
    print(
        json.dumps(
            {
                "schema": RECEIPT_SCHEMA_V2,
                "measurement_rule": MEASUREMENT_RULE,
                **_parsed_gate_arguments(_arguments_after_separator()),
            },
            sort_keys=True,
        )
    )


if __name__ == "__main__" and "--contract-only" in sys.argv:
    try:
        _contract_only()
    except DensityGateContractError as exc:
        raise SystemExit(str(exc)) from exc
    raise SystemExit(0)

if __name__ == "__main__" and os.environ.get("PYTHONSAFEPATH") != "1":
    raise SystemExit(
        f"{PYTHONSAFEPATH_REQUIRED}: kernel field runs require PYTHONSAFEPATH=1"
    )


import bmesh
import bpy


ROOT = Path(__file__).resolve().parents[1]

import cftuv  # noqa: E402
from cftuv.analysis import build_analysis_bundle  # noqa: E402
from cftuv.envelope_debug_profile import (  # noqa: E402
    EnvelopeDebugProfileBuilderV1,
)
from cftuv.envelope_request_export import (  # noqa: E402
    METRIC_STAGE_OUTCOMES,
    EnvelopeHostAdapterError,
    _typed_value,
    build_envelope_analysis_snapshot,
    build_envelope_decal_request,
    build_envelope_topology_debug_scene,
)
from cftuv.envelope_request_policy import (  # noqa: E402
    MEASURED_REQUEST_ALPHA,
)
from cftuv.envelope_topology_debug import (  # noqa: E402
    EnvelopeTopologyPathKind,
)
import cftuv_envelope as kernel  # noqa: E402


def _git_text(*arguments: str) -> str:
    try:
        return subprocess.check_output(
            ("git", "-C", str(ROOT), *arguments),
            text=True,
            encoding="utf-8",
        ).strip()
    except (OSError, subprocess.CalledProcessError) as exc:
        raise DensityGateContractError(
            f"{UNMEASURED_REQUEST_POLICY_KNOB}: repository identity unavailable"
        ) from exc


def _repository_identity() -> dict:
    head = _git_text("rev-parse", "HEAD")
    branch = _git_text("rev-parse", "--abbrev-ref", "HEAD")
    if not re.fullmatch(r"[0-9a-f]{40}", head):
        raise DensityGateContractError(
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
        raise DensityGateContractError(
            f"{INSTALLER_STAMP_MISMATCH}: repository path shadows installed product"
        )
    addon_path = addon_root / "_install_stamp.txt"
    kernel_path = kernel_root / "_install_stamp.txt"
    try:
        addon_stamp = addon_path.read_text(encoding="utf-8-sig").strip()
        kernel_stamp = kernel_path.read_text(encoding="utf-8-sig").strip()
    except (OSError, UnicodeError) as exc:
        raise DensityGateContractError(
            f"{INSTALLER_STAMP_MISSING}: addon/kernel stamp is required"
        ) from exc
    if not addon_stamp or addon_stamp != kernel_stamp:
        raise DensityGateContractError(
            f"{INSTALLER_STAMP_MISMATCH}: addon and kernel stamps differ"
        )
    found = _STAMP_PATTERN.search(addon_stamp)
    if found is None or not repository["head"].startswith(found.group(1)):
        raise DensityGateContractError(
            f"{INSTALLER_STAMP_MISMATCH}: stamp does not name repository HEAD"
        )
    addon_source = ROOT / "cftuv"
    kernel_source = ROOT / "kernel" / "src" / "cftuv_envelope"
    fingerprints = {
        "cftuv_repository": _package_fingerprint(addon_source),
        "cftuv_installed": _package_fingerprint(addon_root),
        "kernel_repository": _package_fingerprint(kernel_source),
        "kernel_installed": _package_fingerprint(kernel_root),
    }
    if (
        fingerprints["cftuv_repository"] != fingerprints["cftuv_installed"]
        or fingerprints["kernel_repository"] != fingerprints["kernel_installed"]
    ):
        raise DensityGateContractError(
            f"{INSTALLER_STAMP_MISMATCH}: installed fingerprints differ"
        )
    return {
        "raw": addon_stamp,
        "addon_path": str(addon_path),
        "kernel_path": str(kernel_path),
        "commit": found.group(1),
        "branch": found.group(2),
        "matched_repository_head": True,
        "loaded_cftuv_path": str(Path(cftuv.__file__).resolve()),
        "loaded_kernel_path": str(Path(kernel.__file__).resolve()),
        "fingerprints": fingerprints,
    }


def _identity_value(value) -> str:
    return str(value.value) if hasattr(value, "value") else str(value)


def _request_policy_receipt(
    request,
    requested_density: int | str,
    effective_density: int | None,
) -> dict:
    selection_policy_id = _identity_value(
        request.angular_profile_selection_policy_id
    )
    value_id = _identity_value(request.max_subturn_value_id)
    parameter_id = _identity_value(request.max_subturn_parameter_id)
    expected_selection = (
        "MIN_K_FOR_MAX_SUBTURN_V1"
        if effective_density is None
        else "HUBER_EMANATED_COUNT_DENSITY_A_V1"
    )
    expected_value = (
        "LINEAR_REFLEX_MAX_SUBTURN_60_DEGREES_V1"
        if effective_density is None
        else f"LINEAR_REFLEX_DENSITY_{effective_density}_V1"
    )
    expected_parameter = (
        "LINEAR_REFLEX_MAX_SUBTURN_V1"
        if effective_density is None
        else "LINEAR_REFLEX_DENSITY_A_V1"
    )
    if (
        selection_policy_id != expected_selection
        or value_id != expected_value
        or parameter_id != expected_parameter
    ):
        raise DensityGateContractError(
            f"{REQUEST_POLICY_EFFECTIVE_MISMATCH}: request policy differs "
            "from requested density"
        )
    policy_fields = {}
    for name in request.__dataclass_fields__:
        if name == "requested_alpha" or name.endswith(
            ("_policy_id", "_parameter_id", "_value_id", "_exact_value")
        ):
            value = getattr(request, name)
            if name.endswith("_exact_value"):
                value = value.symbol
            if name == "requested_alpha":
                value = value.value
            policy_fields[name] = _identity_value(value)
    alpha = policy_fields.get("requested_alpha")
    if alpha is None:
        raise DensityGateContractError(
            f"{UNMEASURED_REQUEST_POLICY_KNOB}: requested_alpha is absent"
        )
    return {
        "requested_density": requested_density,
        "effective_density": effective_density,
        "selection_policy_id": selection_policy_id,
        "max_subturn_parameter_id": parameter_id,
        "max_subturn_value_id": value_id,
        "request_id": _identity_value(request.decal_request_id),
        "request_policy_knobs": {
            "density": {
                "requested": requested_density,
                "effective": effective_density,
                "id": selection_policy_id,
            },
            "alpha": {
                "requested": str(MEASURED_REQUEST_ALPHA),
                "effective": alpha,
                "id": "requested_alpha",
            },
        },
        "request_policy_fields": policy_fields,
    }


def _merged_request_policy_receipt(receipts) -> dict:
    if not receipts:
        raise DensityGateContractError(
            f"{UNMEASURED_REQUEST_POLICY_KNOB}: no request policy was measured"
        )
    first = dict(receipts[0])
    request_ids = sorted({item["request_id"] for item in receipts})
    first.pop("request_id")
    for item in receipts[1:]:
        comparable = dict(item)
        comparable.pop("request_id")
        if comparable != first:
            raise DensityGateContractError(
                f"{REQUEST_POLICY_EFFECTIVE_MISMATCH}: scope requests disagree"
            )
    first["request_ids"] = request_ids
    return first


def _merged_scope_policy_receipt(runs) -> dict:
    receipts = [dict(run["request_policy"]) for run in runs]
    if not receipts:
        raise DensityGateContractError(
            f"{UNMEASURED_REQUEST_POLICY_KNOB}: no scope receipt"
        )
    first = receipts[0]
    request_ids = set(first.pop("request_ids"))
    for item in receipts[1:]:
        request_ids.update(item.pop("request_ids"))
        if item != first:
            raise DensityGateContractError(
                f"{REQUEST_POLICY_EFFECTIVE_MISMATCH}: run policies disagree"
            )
    first["request_ids"] = sorted(request_ids)
    return first


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
        payload, sort_keys=True, separators=(",", ":")
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def _runtime_payload(
    compilation, request, reference_metric_build_seconds: float
) -> tuple[dict, object | None]:
    result = kernel.evaluate_filtered_runtime_raw_coverage(
        compilation,
        request.requested_alpha,
        reference_metric_build_seconds=reference_metric_build_seconds,
    )
    raw = result.raw_result.raw_coverage
    return {
        "outcome": result.raw_result.outcome.value,
        "diagnostics": [
            {
                "outcome": item.outcome.value,
                "message": item.message,
            }
            for item in result.raw_result.diagnostics
        ],
        "semantic_digest": raw.semantic_digest if raw is not None else None,
        "construction_authority": "RationalAffinePlanarMetricV2",
        "performance": {
            "filtered_predicate_count": (
                result.performance.filtered_predicate_count
            ),
            "fast_path_certified_count": (
                result.performance.fast_path_certified_count
            ),
            "exact_fallback_count": (
                result.performance.exact_fallback_count
            ),
            "fallback_fraction": result.performance.fallback_fraction,
            "reference_metric_build_seconds": (
                result.performance.reference_metric_build_seconds
            ),
            "runtime_view_build_seconds": (
                result.performance.runtime_view_build_seconds
            ),
            "raw_coverage_seconds": (
                result.performance.raw_coverage_seconds
            ),
            "expression_size": result.performance.expression_size,
            "peak_memory_bytes": result.performance.peak_memory_bytes,
        },
    }, raw


def _queue_payload(patch_id: int, domain_id: str, snapshot, request) -> dict:
    """Стадия QUEUE рядом с RAW: подготовка, покрытие, их цена и исход.

    Столбцы RAW не трогаются: очередь считается ДОПОЛНИТЕЛЬНО, на тех же
    снапшоте и запросе, и её числа лежат отдельным разделом. Иначе сравнивать
    было бы нечего — осталась бы одна колонка, и та новая.
    """

    from cftuv.envelope_queue_export import queue_domain_payload, run_queue_domain

    alpha_text = str(request.requested_alpha.value)
    prepared, domain = run_queue_domain(
        patch_id,
        domain_id,
        snapshot,
        request,
        alpha_text,
    )
    if domain.preparation_outcome != "EXACT":
        stage = "QUEUE_PREPARE_REJECTED"
    elif domain.coverage_outcome != "EXACT":
        stage = "QUEUE_COVERAGE_REJECTED"
    else:
        stage = "QUEUE_RESOLVED"
    semantic_digest = _queue_semantic_digest(queue_domain_payload(domain))
    return {
        "stage": stage,
        "alpha": alpha_text,
        "preparation_outcome": domain.preparation_outcome,
        "coverage_outcome": domain.coverage_outcome,
        "detail": domain.detail,
        "prepare_seconds": domain.prepare_seconds,
        "coverage_seconds": domain.coverage_seconds,
        "contour_seconds": domain.contour_seconds,
        "faces": len(domain.faces),
        "wall_edges": sum(item.wall_edge_count for item in domain.regions),
        "owners": sorted({face.envelope_spec_id for face in domain.faces}),
        "lattice_scale": domain.lattice_scale,
        "semantic_digest": semantic_digest,
        "region_outcomes": [
            {
                "region_id": item.region_id,
                "bridge": item.bridge_outcome,
                "skeleton": item.skeleton_outcome,
                "faces": item.face_outcome,
                "coverage": item.coverage_outcome,
                "findings": list(item.findings),
            }
            for item in domain.regions
        ],
        "preparation": prepared,
    }


def _queue_alpha_change(entries, alpha_text: str) -> dict:
    """Цена смены alpha на ТЁПЛЫХ подготовках, по всем доменам сразу.

    Это и есть число приёмки: подготовка alpha-независима, поэтому ползунок
    обязан платить только за покрытие. Меряется здесь, а не в Blender, потому
    что фоновый прогон воспроизводим.
    """

    from cftuv.envelope_queue_export import recompute_queue_coverage

    if not entries:
        return {"domains": 0, "elapsed_seconds": 0.0, "outcomes": []}
    started = time.perf_counter()
    scene = recompute_queue_coverage(entries, alpha_text)
    elapsed = time.perf_counter() - started
    return {
        "domains": len(scene.domains),
        "alpha": alpha_text,
        "elapsed_seconds": elapsed,
        "faces": sum(len(item.faces) for item in scene.domains),
        "owner_palette_slots": [
            {"envelope_spec_id": spec_id, "palette_slot": slot}
            for spec_id, slot in scene.palette
        ],
        "palette_wrapped": scene.palette_wrapped,
        "outcomes": [item.coverage_outcome for item in scene.domains],
    }


def _scope_inputs(bundle, edge_ids: tuple[int, ...], profile):
    """Топология и адресация доменов одного scope. Вынесено из `_run_scope`."""

    topology = build_envelope_topology_debug_scene(
        bundle,
        frozenset(edge_ids),
        profile=profile,
    )
    revision = f"host-source:{bundle.source_revision.digest}:{bundle.source_revision.source_name}"
    patch_ids = tuple(
        sorted(
            int(patch_id)
            for patch_id in bundle.patch_graph.nodes
            if _typed_value("patch-domain", revision, int(patch_id))
            in topology.patch_domain_ids
        )
    )
    selected_edges_by_domain = {
        domain_id: set() for domain_id in topology.patch_domain_ids
    }
    for path in topology.paths:
        if (
            path.kind is EnvelopeTopologyPathKind.DIRECTED_CHAIN_USE
            and path.selected
            and path.patch_domain_id is not None
        ):
            selected_edges_by_domain[path.patch_domain_id].update(
                path.host_edge_ids
            )
    request_id = _typed_value(
        "decal-request",
        revision,
        tuple(
            sorted(
                path.physical_chain_id
                for path in topology.paths
                if path.kind is EnvelopeTopologyPathKind.SELECTED_SOURCE
                and path.physical_chain_id is not None
            )
        ),
    )
    return revision, patch_ids, selected_edges_by_domain, request_id


def _domain_row(
    patch_id: int,
    domain_id: str,
    stage: str,
    outcome: str,
    message: str,
    queue: dict | None,
    raw=None,
    runtime: dict | None = None,
) -> dict:
    """Строка домена в отчёте — одна форма на все четыре исхода scope."""

    return {
        "patch_id": patch_id,
        "patch_domain_id": domain_id,
        "stage": stage,
        "outcome": outcome,
        "message": message,
        "raw_semantic_digest": (
            raw.semantic_digest if raw is not None else None
        ),
        "runtime": runtime,
        "queue": queue,
    }


def _parity_projection(domains) -> list[dict]:
    projection = []
    for domain in domains:
        queue = domain.get("queue")
        if queue is None:
            continue
        projection.append(
            {
                "patch_domain_id": domain["patch_domain_id"],
                "stage": queue["stage"],
                "preparation_outcome": queue["preparation_outcome"],
                "coverage_outcome": queue["coverage_outcome"],
                "resolved": queue["stage"] == "QUEUE_RESOLVED",
                "semantic_digest": queue["semantic_digest"],
            }
        )
    return sorted(projection, key=lambda item: item["patch_domain_id"])


def _performance_receipt(runs, object_name: str) -> dict:
    budget = _FIELD_BUDGET_SECONDS.get(object_name)
    run = next(
        (item for item in runs if item["scope"] == "all_seam_chains_l0"),
        None,
    )
    if run is None or budget is None:
        return {
            "scope": "all_seam_chains_l0",
            "budget_seconds": budget,
            "measured_queue_seconds": None,
            "passed": budget is None,
        }
    measured = sum(
        (domain.get("queue") or {}).get("prepare_seconds") or 0.0
        for domain in run["domains"]
    ) + sum(
        (
            ((domain.get("queue") or {}).get("coverage_seconds") or 0.0)
            + ((domain.get("queue") or {}).get("contour_seconds") or 0.0)
        )
        for domain in run["domains"]
    )
    return {
        "scope": "all_seam_chains_l0",
        "budget_seconds": budget,
        "measured_queue_seconds": measured,
        "passed": measured <= budget,
    }


def _metric_seconds(profile, domain_id) -> float:
    """Сумма FRAME_ADMISSION домена — цена метрики, уже уплаченная профилем."""

    return sum(
        item.elapsed_seconds
        for item in profile.snapshot().timings
        if item.stage == "FRAME_ADMISSION"
        and item.patch_domain_id == domain_id
    )


def _snapshot_request(
    bundle,
    patch_id: int,
    selected_edges,
    profile,
    domain_id: str,
    request_id: str,
    density_contract: tuple[int | str, int | None],
):
    with profile.measure("SNAPSHOT_EXPORT", domain_id):
        snapshot = build_envelope_analysis_snapshot(
            bundle,
            included_patch_ids=frozenset({patch_id}),
            profile=profile,
        )
        request = build_envelope_decal_request(
            snapshot,
            frozenset(selected_edges),
            MEASURED_REQUEST_ALPHA,
            decal_request_id_value=request_id,
            density=density_contract[1],
        )
    return snapshot, request, _request_policy_receipt(
        request,
        *density_contract,
    )


def _run_scope(
    bundle,
    name: str,
    edge_ids: tuple[int, ...],
    density_contract: tuple[int | str, int | None],
    object_name: str = "building.002",
    engines: frozenset[str] = frozenset({"raw", "queue"}),
) -> dict:
    profile = EnvelopeDebugProfileBuilderV1(object_name, "M_R1_FIELD")
    started = time.perf_counter()
    (
        revision,
        patch_ids,
        selected_edges_by_domain,
        request_id,
    ) = _scope_inputs(bundle, edge_ids, profile)
    elapsed = time.perf_counter() - started
    domains = []
    queue_entries = []
    policy_receipts = []
    for patch_id in patch_ids:
        domain_id = _typed_value("patch-domain", revision, patch_id)
        try:
            snapshot, request, policy_receipt = _snapshot_request(
                bundle,
                patch_id,
                selected_edges_by_domain[domain_id],
                profile,
                domain_id,
                request_id,
                density_contract,
            )
            policy_receipts.append(policy_receipt)
        except EnvelopeHostAdapterError as exc:
            # Власть классификации ступени одна и хостовая. Своё множество
            # имён здесь было двухэлементным против шести, и четыре отказа
            # метрики расписка молча переносила на ступень COMPILE.
            stage = (
                "METRIC_REJECTED"
                if exc.outcome in METRIC_STAGE_OUTCOMES
                else "COMPILE_REJECTED"
            )
            domains.append(
                _domain_row(
                    patch_id, domain_id, stage, exc.outcome.value, str(exc), None
                )
            )
            continue
        with profile.measure("QUEUE_PREPARE", domain_id):
            queue = _queue_payload(patch_id, domain_id, snapshot, request)
        preparation = queue.pop("preparation")
        if queue["stage"] == "QUEUE_RESOLVED":
            queue_entries.append((patch_id, domain_id, preparation))
        if "raw" not in engines:
            # QUEUE-only: на большом выделении RAW стоит часы, и его отсутствие
            # объявлено выбором движков, а не потерей колонки.
            domains.append(
                _domain_row(
                    patch_id,
                    domain_id,
                    queue["stage"],
                    queue["preparation_outcome"],
                    "raw engine skipped by request",
                    queue,
                )
            )
            continue
        with profile.measure("COMPILE", domain_id):
            compiled = kernel.compile_reference_envelopes(snapshot, request)
        if compiled.compilation is None:
            domains.append(
                _domain_row(
                    patch_id,
                    domain_id,
                    "COMPILE_REJECTED",
                    compiled.outcome.value,
                    "; ".join(item.message for item in compiled.diagnostics),
                    queue,
                )
            )
            continue
        with profile.measure("RAW_UNION", domain_id):
            runtime_payload, raw = _runtime_payload(
                compiled.compilation,
                request,
                _metric_seconds(profile, domain_id),
            )
        raw_result = runtime_payload["outcome"]
        stage = "RAW_READY" if raw is not None else "RAW_REJECTED"
        domains.append(
            _domain_row(
                patch_id,
                domain_id,
                stage,
                raw_result,
                (
                    "authoritative exact RawCoverage available"
                    if raw is not None
                    else "authoritative exact RawCoverage unavailable"
                ),
                queue,
                raw=raw,
                runtime=runtime_payload,
            )
        )
    result = {
        "scope": name,
        "selected_physical_edge_ids": list(edge_ids),
        "topology_elapsed_seconds": elapsed,
        "profile": profile.snapshot().to_payload(),
        "domains": domains,
        "queue_alpha_change": _queue_alpha_change(queue_entries, "0.5"),
        "request_policy": _merged_request_policy_receipt(policy_receipts),
    }
    result["parity_projection"] = _parity_projection(domains)
    return result


def _analysis_bundle(obj):
    """Анализу нужна та же EDIT_MODE-среда, что настоящему оператору."""

    bpy.context.view_layer.objects.active = obj
    obj.select_set(True)
    bpy.ops.object.mode_set(mode="EDIT")
    try:
        bm = bmesh.from_edit_mesh(obj.data)
        bm.faces.ensure_lookup_table()
        return build_analysis_bundle(
            bm,
            tuple(face.index for face in bm.faces),
            obj,
        )
    finally:
        bpy.ops.object.mode_set(mode="OBJECT")


def _receipt_echo(output, payload: dict) -> str:
    """Компактное эхо расписки: путь, исход, счёт доменов по ступеням, дайджест.

    Прежде в stdout уходил ПОЛНЫЙ JSON — побайтовый дубликат только что
    записанного файла. Замер принципала: 51 тыс. символов на `building.002`,
    ~170 тыс. на `building` за прогон. Конвейер это эхо не потребляет:
    `field_cycle.ps1` проверяет наличие ФАЙЛА и передаёт его путь
    `field_cycle_summary.py`, который читает файл; единственный потребитель
    stdout — путь `--contract-only`, и он до сюда не доходит вовсе (ранний
    `SystemExit` до импорта bpy).

    Дайджест считается ЧТЕНИЕМ ЗАПИСАННОГО ФАЙЛА, а не строки перед записью:
    `Path.write_text` переводит `\\n` в `os.linesep`, и на Windows дайджест
    строки не совпал бы с `sha256sum` файла — то есть врал бы ровно тому, кто
    решит его проверить. Байты файла не двигаются: перевод строк остался
    прежним, изменился только источник дайджеста.
    """

    stages: dict[str, int] = {}
    for run in payload["runs"]:
        for domain in run.get("domains", ()):
            stage = domain.get("stage") or "UNKNOWN"
            stages[stage] = stages.get(stage, 0) + 1
    digest = hashlib.sha256(output.read_bytes()).hexdigest()
    counts = " ".join(f"{name}={stages[name]}" for name in sorted(stages))
    return (
        f"M-R1 receipt {output} status={payload['status']} "
        f"gate_failure={payload['gate_failure']} "
        f"domains[{counts}] sha256={digest}"
    )


def _selected_scopes(obj, requested: str):
    seam_edges = tuple(edge.index for edge in obj.data.edges if edge.use_seam)
    scopes = (
        ("gate_edge_12", (12,)),
        ("one_chain_l0", (2,)),
        ("three_chains_l0", (2, 3, 7)),
        ("ten_chains_l0", tuple(range(1, 11))),
        ("all_seam_chains_l0", seam_edges),
    )
    if requested == "all":
        return scopes
    requested_names = frozenset(requested.split(","))
    selected = tuple(item for item in scopes if item[0] in requested_names)
    if not selected:
        raise ValueError("requested M-R1 field scopes are unknown")
    return selected


def main() -> None:
    # После `--`: output, scopes, object, engines, density (0/1/4/legacy).
    # Движки: "raw,queue" (умолчание) либо "queue" — RAW на большом выделении
    # стоит часы, и его пропуск обязан быть явным выбором, а не тихой потерей.
    arguments = _arguments_after_separator()
    parsed = _parsed_gate_arguments(arguments)
    output = Path(parsed["output"])
    object_name = parsed["object_name"]
    engines = frozenset(parsed["engines"].split(","))
    if not engines <= {"raw", "queue"}:
        raise ValueError(f"неизвестные движки: {sorted(engines)}")
    repository = _repository_identity()
    installer = _installer_identity(repository)
    density_contract = (
        parsed["requested_density"],
        parsed["effective_density"],
    )
    obj = bpy.data.objects.get(object_name)
    if obj is None or obj.type != "MESH":
        raise RuntimeError(f"{object_name} mesh object is unavailable")
    before = _mesh_fingerprint(obj)
    bundle = _analysis_bundle(obj)
    scopes = _selected_scopes(obj, parsed["scopes"])
    runs = []
    output.parent.mkdir(parents=True, exist_ok=True)
    for name, edges in scopes:
        print(f"M-R1 field scope start: {name}", flush=True)
        runs.append(
            _run_scope(
                bundle,
                name,
                edges,
                density_contract,
                obj.name,
                engines,
            )
        )
        checkpoint = {
            "schema": RECEIPT_SCHEMA_V2,
            "measurement_rule": MEASUREMENT_RULE,
            "repository_head": repository["head"],
            "repository_branch": repository["branch"],
            "installer_stamp": installer,
            "python_safe_path": True,
            "requested_density": density_contract[0],
            "effective_density": density_contract[1],
            "source_file": bpy.data.filepath,
            "source_object": obj.name,
            "status": "IN_PROGRESS",
            "runs": runs,
        }
        output.write_text(
            json.dumps(
                checkpoint,
                ensure_ascii=False,
                indent=2,
                sort_keys=True,
            )
            + "\n",
            encoding="utf-8",
        )
        print(f"M-R1 field scope complete: {name}", flush=True)
    after = _mesh_fingerprint(obj)
    policy = _merged_scope_policy_receipt(runs)
    performance = _performance_receipt(runs, obj.name)
    unchanged = before == after
    gate_failure = None
    if not unchanged:
        gate_failure = "SOURCE_MESH_FINGERPRINT_CHANGED"
    elif not performance["passed"]:
        gate_failure = FIELD_PERFORMANCE_BUDGET_EXCEEDED
    payload = {
        "schema": RECEIPT_SCHEMA_V2,
        "measurement_rule": MEASUREMENT_RULE,
        "repository_head": repository["head"],
        "repository_branch": repository["branch"],
        "installer_stamp": installer,
        "python_safe_path": True,
        **policy,
        "source_file": bpy.data.filepath,
        "source_object": obj.name,
        "status": "COMPLETE" if gate_failure is None else "REJECTED",
        "acceptance_eligible": parsed["acceptance_eligible"],
        "gate_failure": gate_failure,
        "mesh_counts": {
            "vertices": len(obj.data.vertices),
            "edges": len(obj.data.edges),
            "faces": len(obj.data.polygons),
        },
        "mesh_fingerprint_before": before,
        "mesh_fingerprint_after": after,
        "source_mesh_unchanged": unchanged,
        "performance": performance,
        "runs": runs,
    }
    output.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2, sort_keys=True)
        + "\n",
        encoding="utf-8",
    )
    print(_receipt_echo(output, payload))
    if gate_failure is not None:
        raise DensityGateContractError(gate_failure)


if __name__ == "__main__":
    main()
