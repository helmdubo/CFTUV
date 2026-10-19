"""Lap instrumentation for the cold-press profile (diagnostic only; nothing in the repo is touched).

Used in TWO places with the same code:
  * the Blender parent (sequential composition: an in-process fake pool runs `serve_task` for every task, so the parent process IS the worker),
  * every pool worker (patched into the worker bootstrap string, in the profiling process only).

`install(host)` wraps functions (outermost call only per lap name), keyed by the patch id of the domain that is being worked on (`CURRENT`).
Context tags `@EXPORT / @PREPARE / @PRODUCE / @REPLY` say which worker stage a leaf call belongs to.
"""

from __future__ import annotations

import importlib
import inspect
import json
import os
import sys
import time

LAP: dict = {}  # name -> [calls, seconds]
CALLS: dict = {}  # name -> [(enter, exit), ...] for the parent "P." laps
SPAN: dict = {}  # name -> [first_enter_perf, last_exit_perf]  (absolute perf_counter, comparable across processes on Windows)
DOMAIN: dict = {}  # patch -> {name: seconds}
CURRENT = [None]
CTX: list = []
STAGES: list = []
MISSING: list = []
TASKS: list = []  # per-task records of this process (worker probe)
ENABLED = [True]
_WRAPPED: set = set()


def reset() -> None:
    CALLS.clear()
    SPAN.clear()
    LAP.clear()
    DOMAIN.clear()
    CTX.clear()
    STAGES.clear()
    TASKS.clear()
    CURRENT[0] = None


def add(name: str, seconds: float) -> None:
    entry = LAP.get(name)
    if entry is None:
        LAP[name] = [1, seconds]
    else:
        entry[0] += 1
        entry[1] += seconds
    patch = CURRENT[0]
    if patch is not None:
        row = DOMAIN.setdefault(patch, {})
        row[name] = row.get(name, 0.0) + seconds


def _ctx_label(name: str) -> str:
    where = CTX[-1] if CTX else "-"
    return f"{name}@{where}"


def _module(host: str, spec: str):
    root, _, rest = spec.partition(":")
    prefix = host if root == "host" else "cftuv_native" if root == "native" else "cftuv_envelope"
    return importlib.import_module(f"{prefix}.{rest}" if rest else prefix)


def _owner_and_attr(module, dotted: str):
    parts = dotted.split(".")
    owner = module
    for part in parts[:-1]:
        owner = getattr(owner, part)
    return owner, parts[-1]


def _patch_everywhere(original, replacement, prefixes) -> int:
    count = 0
    for name, module in list(sys.modules.items()):
        if module is None or not name.startswith(prefixes):
            continue
        namespace = getattr(module, "__dict__", None)
        if not namespace:
            continue
        for key, value in list(namespace.items()):
            if value is original:
                namespace[key] = replacement
                count += 1
    return count


def wrap(host: str, mod: str, dotted: str, lap: str, *, ctx: str | None = None, tag: bool = False, prefixes=None) -> bool:
    """Wrap `mod:dotted` (function, method, classmethod, staticmethod) so that its OUTERMOST call adds `lap` (per lap name, so recursion is not double counted)."""

    if (mod, dotted, lap) in _WRAPPED:
        return True
    try:
        module = _module(host, mod)
        owner, attr = _owner_and_attr(module, dotted)
        raw = inspect.getattr_static(owner, attr)
    except Exception as exc:  # noqa: BLE001
        MISSING.append(f"{mod}:{dotted} ({type(exc).__name__}: {exc})")
        return False
    _WRAPPED.add((mod, dotted, lap))
    kind = "function"
    original = raw
    if isinstance(raw, classmethod):
        kind, original = "classmethod", raw.__func__
    elif isinstance(raw, staticmethod):
        kind, original = "staticmethod", raw.__func__
    depth = [0]

    def timed(*a, **k):
        if depth[0] or not ENABLED[0]:
            return original(*a, **k)
        depth[0] = 1
        if ctx:
            CTX.append(ctx)
        started = time.perf_counter()
        try:
            return original(*a, **k)
        finally:
            ended = time.perf_counter()
            seconds = ended - started
            depth[0] = 0
            if ctx:
                CTX.pop()
            add(_ctx_label(lap) if tag else lap, seconds)
            if lap.startswith("P."):
                CALLS.setdefault(lap, []).append((started, ended))
            span = SPAN.get(lap)
            if span is None:
                SPAN[lap] = [started, ended]
            else:
                span[1] = ended

    timed.__wrapped__ = original
    timed.__name__ = getattr(original, "__name__", attr)
    timed.__qualname__ = getattr(original, "__qualname__", attr)
    replacement = {"function": timed, "classmethod": classmethod(timed), "staticmethod": staticmethod(timed)}[kind]
    setattr(owner, attr, replacement)
    if kind == "function" and owner is module:
        _patch_everywhere(original, timed, prefixes or (host, "cftuv_envelope", "cftuv_native"))
    return True


def wrap_result(host: str, mod: str, dotted: str, lap: str, prefix: str, *, ctx: str | None = None, pick=None) -> bool:
    """Like `wrap`, and the `.timings` of the result (pairs `(name, seconds)`) are added as `prefix.NAME`."""

    try:
        module = _module(host, mod)
        owner, attr = _owner_and_attr(module, dotted)
        original = inspect.getattr_static(owner, attr)
    except Exception as exc:  # noqa: BLE001
        MISSING.append(f"{mod}:{dotted} ({type(exc).__name__}: {exc})")
        return False
    depth = [0]

    def timed(*a, **k):
        if depth[0] or not ENABLED[0]:
            return original(*a, **k)
        depth[0] = 1
        if ctx:
            CTX.append(ctx)
        started = time.perf_counter()
        try:
            result = original(*a, **k)
        finally:
            seconds = time.perf_counter() - started
            depth[0] = 0
            if ctx:
                CTX.pop()
            add(lap, seconds)
        target = result if pick is None else pick(result)
        try:
            for name, value in getattr(target, "timings", ()) or ():
                add(f"{prefix}.{name}", float(value))
        except Exception:  # noqa: BLE001
            pass
        return result

    timed.__wrapped__ = original
    setattr(owner, attr, timed)
    if owner is module:
        _patch_everywhere(original, timed, (host, "cftuv_envelope", "cftuv_native"))
    return True


def install_profile_stages(host: str) -> None:
    """Stage seconds of the debug profile (`measure` / `add_timing`): inclusive, per patch, `stage.NAME`."""

    try:
        profile_module = importlib.import_module(f"{host}.envelope_debug_profile")
        builder = profile_module.EnvelopeDebugProfileBuilderV1
    except Exception as exc:  # noqa: BLE001
        MISSING.append(f"profile builder ({exc})")
        return
    original_measure = builder.measure
    original_add = builder.add_timing
    from contextlib import contextmanager

    @contextmanager
    def measure(self, stage, patch_domain_id=None):
        STAGES.append(stage)
        started = time.perf_counter()
        try:
            with original_measure(self, stage, patch_domain_id):
                yield
        finally:
            STAGES.pop()
            add("stage." + stage, time.perf_counter() - started)

    builder.measure = measure


def install(host: str = "cftuv") -> list:
    """The standard lap set. Returns the list of names that could not be resolved (printed into the report)."""

    install_profile_stages(host)
    H, K = "host", "kernel"
    # ---- worker task skeleton ----
    wrap(host, f"{H}:envelope_production_task", "_solve_cold_production_task", "W.cold_task")
    wrap(host, f"{H}:envelope_production_task", "solve_cold_production_task", "W.cold_task_outer")
    wrap(host, f"{H}:envelope_domain_pool", "_packed_cold_reply", "W.pack_prepared_reply", ctx="REPLY")
    wrap(host, f"{H}:envelope_domain_pool", "_reply_frame", "W.reply_frame", ctx="REPLY")
    # ---- EXPORT ----
    wrap(host, f"{H}:envelope_export_input", "task_inputs", "EXPORT.task_inputs", ctx="EXPORT")
    wrap(host, f"{H}:envelope_metric_export", "build_envelope_patch_metric_export", "EXPORT.metric_export")
    wrap(host, f"{H}:envelope_request_export", "build_envelope_decal_request", "EXPORT.decal_request")
    wrap(host, f"{H}:envelope_angle_certificate", "validate_exported_snapshot", "EXPORT.validate_exported_snapshot")
    wrap(host, f"{K}:validation", "validate_analysis_snapshot", "leaf.validate_analysis_snapshot", tag=True)
    wrap(host, f"{K}:reference.validation", "validate_reference_geometry_payload", "leaf.validate_reference_geometry_payload", tag=True)
    wrap(host, f"{K}:reference.validation", "validate_reference_geometry_certificates", "leaf.validate_reference_geometry_certificates", tag=True)
    wrap(host, f"{K}:validation", "validate_snapshot_request_references", "leaf.validate_snapshot_request_references", tag=True)
    wrap(host, f"{K}:codec", "AnalysisSnapshotCodecV1.dumps", "leaf.codec_dumps", tag=True)
    wrap(host, f"{K}:codec", "AnalysisSnapshotCodecV1.loads", "leaf.codec_loads", tag=True)
    wrap(host, f"{K}:_developable", "build_developable_chart", "leaf.build_developable_chart", tag=True)
    wrap(host, f"{K}:_embedding", "build_source_snap_embedding_certificate", "leaf.embedding_certificate", tag=True)
    # ---- PREPARE ----
    wrap_result(host, f"{H}:envelope_production_task", "prepare_for_production_recorded", "PREPARE.total", "prep.lap", ctx="PREPARE", pick=lambda r: r[0])
    wrap_result(host, f"{K}:wavefront.conveyor", "prepare_conveyor", "PREPARE.prepare_conveyor", "prepx.lap")
    wrap(host, f"{K}:backend", "skeleton_compute", "native.skeleton_compute")
    wrap(host, f"{K}:backend", "embedding_compute", "native.embedding_compute")
    wrap(host, f"{K}:reference.compile", "compile_reference_envelopes", "PC.compile_reference_envelopes", ctx="PLAN_COMPILE")
    wrap(host, f"{K}:reference.compile", "_finalize_compilation", "PC.finalize")
    wrap(host, f"{K}:reference.compile", "_attach_evaluation_geometry", "PC.attach_evaluation_geometry")
    wrap(host, f"{K}:reference.compile", "_attach_direction_bindings", "PC.attach_direction_bindings")
    wrap(host, f"{K}:reference.compile", "_evaluation_density_spec", "PC.evaluation_density_spec")
    wrap(host, f"{K}:reference.compile", "_seal_evaluation_geometry_authority", "PC.seal_evaluation_geometry_authority")
    wrap(host, f"{K}:reference.compile", "_attach_front_reading_declarations", "PC.attach_front_reading_declarations")
    wrap(host, f"{K}:reference.compile", "declare_reference_self_contacts", "PC.declare_reference_self_contacts")
    wrap(host, f"{K}:reference.compile", "_resolve_angular_profile_selection", "PC.resolve_angular_profile_selection")
    wrap(host, f"{K}:reference.compile", "_synchronize_effective_hidden_support_records", "PC.sync_effective_hidden_support")
    wrap(host, f"{K}:reference.density_fan_binding", "bind_density_fan", "PC.bind_density_fan", tag=True)
    wrap(host, f"{K}:reference.direction_binding", "certify_adaptive_huber_density_direction_fan", "PC.certify_adaptive_huber_fan", tag=True)
    wrap(host, f"{K}:_chain_station", "chain_station_plans", "PC.chain_station_plans", tag=True)
    wrap(host, f"{K}:reference.angular", "_ideal_angular_support_data", "PC.ideal_angular_support_data", tag=True)
    wrap(host, f"{K}:reference.angular", "verify_evaluation_direction_binding_reasons", "PC.verify_evaluation_direction_binding_reasons", tag=True)
    wrap(host, f"{K}:reference.evaluation_binding_noise", "evaluation_count_is_feasible", "PC.evaluation_count_is_feasible", tag=True)
    wrap(host, f"{K}:reference.common", "GeometryContext.support_segments_for_use", "PC.support_segments_for_use", tag=True)
    # DOMAIN_BUILD parts
    wrap(host, f"{K}:reference.common", "GeometryContext.build", "DB.geometry_context_build", tag=True)
    wrap(host, f"{K}:reference.angular", "seal_angular_support_cache", "DB.seal_angular_support_cache", tag=True)
    wrap(host, f"{K}:reference.boundary", "build_domain_geometry", "DB.build_domain_geometry", tag=True)
    # ARRIVAL_LAWS / BRIDGE / lattice
    wrap(host, f"{K}:wavefront.conveyor", "_arrival_laws", "AL.arrival_laws", ctx="ARRIVAL_LAWS")
    wrap(host, f"{K}:wavefront.bridge", "bridge_arrival_laws", "BR.bridge_arrival_laws", ctx="BRIDGE")
    # ---- PRODUCE ----
    wrap(host, f"{H}:envelope_production_task", "produce_domain", "PRODUCE.total", ctx="PRODUCE")
    wrap(host, f"{K}:materialize.step", "step_domain", "PRODUCE.step_domain")
    wrap_result(host, f"{K}:wavefront.conveyor", "conveyor_coverage", "PRODUCE.coverage", "cov.lap")
    wrap_result(host, f"{K}:materialize.domain", "materialize_domain", "PRODUCE.materialize", "mat.lap")
    wrap(host, f"{K}:wavefront.conveyor", "_instance_ids_by_spec", "EA.instance_ids_by_spec")
    wrap(host, f"{K}:reference.boundary", "resolve_component_alphas", "EA.resolve_component_alphas", tag=True)
    wrap(host, f"{K}:reference.boundary", "_native_contacts", "EA.native_contacts")
    wrap(host, f"{K}:backend", "effective_alpha_compute", "EA.native_effective_alpha_compute")
    wrap(host, f"{K}:backend", "coverage_compute", "native.coverage_compute")
    wrap(host, f"{K}:backend", "covered_at", "native.covered_at")
    wrap(host, f"{K}:backend", "clip_compute", "native.clip_compute")
    wrap(host, f"{K}:backend", "exact_mesh_compute", "native.w1_exact_mesh")
    wrap(host, f"{K}:backend", "canonical_triangles_compute", "native.w1_canonical_triangles")
    wrap(host, f"{K}:materialize.parametric", "certificate_of", "S6A.certificate_of")
    wrap(host, f"{K}:materialize.parametric", "capture_inputs", "S6A.capture_inputs")
    wrap(host, f"{H}:envelope_production_result", "deferred_result", "PACK.deferred_result")
    wrap(host, f"{H}:envelope_production_view", "build_domain_view", "PACK.build_domain_view")
    # ---- release-mode table: digests, checks, bookkeeping ----
    wrap(host, f"{K}:materialize.structure", "batch_structure", "REL.batch_structure")
    wrap(host, f"{K}:materialize.offset_normal", "offset_normals_digest", "REL.offset_normals_digest")
    wrap(host, f"{K}:codec", "canonical_json_bytes", "REL.canonical_json_bytes", tag=True)
    wrap(host, f"{K}:materialize.domain", "finalize_digests", "REL.finalize_digests")
    wrap(host, f"{H}:envelope_kernel_backend", "entered_backend", "REL.ledger.entered_backend")
    wrap(host, f"{H}:envelope_kernel_backend", "ledger_record", "REL.ledger.ledger_record")
    wrap(host, f"{H}:envelope_kernel_backend", "with_preparation_record", "REL.ledger.with_preparation_record")
    wrap(host, f"{H}:envelope_host_labels", "record_host_tokens", "REL.host_tokens.record_host_tokens")
    wrap(host, f"{H}:envelope_snapshot_check", "WorkerSnapshotCheck.clean_issues_for", "REL.snapshot_check.clean_issues_for")
    wrap(host, f"{H}:envelope_export_input", "replay_export_records", "REL.replay_export_records")
    wrap_reject_snapshot(host)
    return list(MISSING)


def wrap_reject_snapshot(host: str) -> None:
    """`_reject_snapshot(kernel, snapshot, *, stage)`: the object check (`stage='snapshot'`, always) and the decoded-bytes check (`stage='canonical bytes'`, only on the byte check)."""

    try:
        module = importlib.import_module(f"{host}.envelope_angle_certificate")
        original = module._reject_snapshot
    except Exception as exc:  # noqa: BLE001
        MISSING.append(f"_reject_snapshot ({exc})")
        return

    def timed(kernel, snapshot, *, stage):
        started = time.perf_counter()
        try:
            return original(kernel, snapshot, stage=stage)
        finally:
            add("CHECK.reject_snapshot[" + stage + "]", time.perf_counter() - started)
            add("CHECK.reject_snapshot[" + stage + "]@" + (CTX[-1] if CTX else "-"), time.perf_counter() - started)

    module._reject_snapshot = timed


# --------------------------------------------------------------------------
# worker role
# --------------------------------------------------------------------------


def install_worker(package: str, dump_dir: str, laps: bool = True) -> None:
    """Inside a pool worker (called from the patched bootstrap): per-task records plus the lap set; dumped to <dump_dir>/<pid>.json at exit and every 25 tasks."""

    import atexit

    pool_module = importlib.import_module(package + ".envelope_domain_pool")
    if laps:
        try:
            install(package)
        except Exception as exc:  # noqa: BLE001
            MISSING.append(f"install failed: {exc!r}")
    original = pool_module.serve_task
    pid = os.getpid()

    def dump() -> None:
        try:
            data = {"pid": pid, "tasks_count": len(TASKS), "laps": {k: v for k, v in LAP.items()}, "domain": {str(k): v for k, v in DOMAIN.items()}, "missing": MISSING,
                    "t_first_import": T_START, "t_now": time.perf_counter(), "wall_now": time.time()}
            tmp = os.path.join(dump_dir, f"{pid}.json.tmp")
            with open(tmp, "w", encoding="utf-8") as handle:
                json.dump(data, handle)
            os.replace(tmp, os.path.join(dump_dir, f"{pid}.json"))
        except Exception:  # noqa: BLE001
            pass

    first_modules = {}

    def serve(task):
        patch = int(getattr(task, "patch_id", -1))
        CURRENT[0] = patch
        if not first_modules:
            first_modules["before"] = set(sys.modules)
        before = {k: v[1] for k, v in LAP.items()}
        cpu0, t0, wall0 = time.process_time(), time.perf_counter(), time.time()
        try:
            return original(task)
        finally:
            if "after" not in first_modules:
                first_modules["after"] = sorted(set(sys.modules) - first_modules["before"])
                MISSING.append("first_task_new_modules:" + ",".join(first_modules["after"][:400]))
            cpu1, t1 = time.process_time(), time.perf_counter()
            CURRENT[0] = None
            delta = {k: round(v[1] - before.get(k, 0.0), 7) for k, v in LAP.items() if v[1] - before.get(k, 0.0) > 0.0}
            TASKS.append({"patch": patch, "t0": t0, "t1": t1, "wall0": wall0, "cpu": cpu1 - cpu0, "pid": pid, "laps": delta, "kind": type(task).__name__})
            add("W.serve_task", t1 - t0)

    import threading

    written = [0]

    def flush_tasks() -> None:
        """Appends the task records not yet written to <pid>.jsonl (small writes, so the dump never stalls a press)."""
        count = len(TASKS)
        if count == written[0]:
            return
        with open(os.path.join(dump_dir, f"{pid}.jsonl"), "a", encoding="utf-8") as handle:
            for record in TASKS[written[0]:count]:
                handle.write(json.dumps(record) + "\n")
        written[0] = count

    def periodic() -> None:
        while True:
            time.sleep(0.5)
            try:
                flush_tasks()
            except Exception:  # noqa: BLE001
                pass

    atexit.register(flush_tasks)

    threading.Thread(target=periodic, name="draglaps-dump", daemon=True).start()

    pool_module.serve_task = serve
    original_sized = pool_module.read_frame_sized

    def sized(stream):
        value = original_sized(stream)
        if value is not None:
            add("W.task_unpickle", float(value[3]))
        return value

    pool_module.read_frame_sized = sized
    atexit.register(dump)


T_START = time.perf_counter()
