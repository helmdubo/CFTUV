"""Ворота SNAPSHOT_BYTE_CHECK_FIRST_PRESS_IN_BACKGROUND_V1 на полевых случаях: итоговый вердикт фона РАВЕН вердикту полной проверки первого нажатия.

    blender -b <сцена.blend> --python-exit-code 1 --python tools/blender_byte_check_background_gate.py -- --case building --workers 12 --out r.json \
        [--root ДЕРЕВО] [--native-path КАТАЛОГ] [--seed-violation 50]

Случай - встроенный случай `tools/blender_tree_ab.py` (`building`, `rounded`, `two`, `cover008`, ...; сцена - загруженный .blend). Скрипт ничего не сохраняет и UI не нужен.

ЧТО ДЕЛАЕТ (одна сессия Blender, один пул):
1. ЭТАЛОН - переключатель `all`: первое нажатие на свежей сессии проверяет круг кодека на КАЖДОМ домене, как проверяло до фона. Записано: исход, деталь, дайджест содержимого каждого домена.
2. НАЖАТИЕ - переключатель снят (умолчание кнопки `sample`), память «код проверен» забыта: первое нажатие сеанса. Воркеры откладывают круг кодека (`PRODUCTION_SNAPSHOT_BYTE_CHECK_DEFERRED`), нажатие их
   не гонит; сразу после нажатия замерена задержка следующего прогона пула (`pool.run([])`) ПОКА фон идёт - она не длиннее шага.
3. ФОН - ждём конец задачи (`job.wait`): `CLEAN` без находок (или `VIOLATION` ровно по тем доменам, которые эталон отказал, если задан `--seed-violation`), проверено не меньше, чем отложено.
4. СЛЕДУЮЩЕЕ ДЕЙСТВИЕ - тот же прогон на той же сессии: исходы, детали и дайджесты содержимого ВСЕХ доменов равны эталону (отказанные фоном - тем же исходом и текстом, что дала полная проверка).
5. Идентичность кода после фона помечена проверенной (следующее холодное нажатие - выборка).

`--seed-violation K`: воркеры (патчем их загрузчика в этом процессе) отдают кодеку неподвижную точку, сломанную у доменов, чей `sha256(идентичность) % K == 0`: настоящий вердикт настоящего отказа на настоящем пуле, а не имитация родителя.
Выход 0 - все проверки выполнены; 1 - что-то не равно (в отчёте `failures`). Запуск - с `PYTHONSAFEPATH=1`.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import sys
import time
import traceback
from pathlib import Path

TOOLS = Path(__file__).resolve().parent
sys.path.insert(0, str(TOOLS))

import blender_tree_ab as ab  # noqa: E402

JOB_TIMEOUT_SECONDS = 300.0
#: Загрузчик воркера: неподвижная точка кодека сломана у части доменов (второй `dumps` проверки даёт другие байты).
SEED_PATCH = r'''
import hashlib as _hashlib
import cftuv_envelope as _k
_codec = _k.AnalysisSnapshotCodecV1
_real_dumps = _codec.dumps
_parity = {"n": 0}
def _seeded_dumps(snapshot):
    payload = _real_dumps(snapshot)
    ids = [item.patch_domain_id.value for item in snapshot.patch_domains]
    if len(ids) == 1 and int.from_bytes(_hashlib.sha256(ids[0].encode("utf-8")).digest()[:8], "big") % SEED_MODULUS == 0:
        _parity["n"] += 1
        return payload + b" " if _parity["n"] % 2 == 0 else payload
    return payload
_codec.dumps = staticmethod(_seeded_dumps)
'''


def _arguments(argv):
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", default=str(Path(__file__).resolve().parents[1]))
    parser.add_argument("--case", required=True)
    parser.add_argument("--workers", type=int, default=12)
    parser.add_argument("--native-path", default="")
    parser.add_argument("--seed-violation", type=int, default=0, help="K: ломает круг кодека у доменов, чей sha256 по модулю K равен нулю (воркеры)")
    parser.add_argument("--out", required=True)
    return parser.parse_args(argv)


def _seed_workers(modulus: int) -> None:
    import cftuv.envelope_domain_pool as pool_module

    last = ab.WORKER_MAIN_CALL
    assert last in pool_module._BOOTSTRAP, "the worker bootstrap changed: update SEED_PATCH"
    pool_module._BOOTSTRAP = pool_module._BOOTSTRAP.replace(last, f"SEED_MODULUS = {int(modulus)}\n" + SEED_PATCH + "\n" + last)


def _rows(run) -> dict:
    return {
        int(item.patch_id): {
            "domain_id": str(item.domain_id),
            "outcome": str(item.outcome),
            "detail": str(item.detail),
            "content_digest": "" if not item.is_materialized else str(getattr(item, "content_digest")),  # по имени поля: ленивое свойство отложенного результата
        }
        for item in run.results
    }


def _counts(run) -> dict:
    return {item.name: item.value for item in run.profile.counters if item.patch_domain_id is None and item.name.startswith("PRODUCTION_SNAPSHOT_BYTE_CHECK")}


def _press(controller, bundle, selected, case, settings, mesh_settings, key, data_key, workers):
    import cftuv.envelope_production_export as export_module
    from cftuv.envelope_kernel_backend import kernel_backend_of
    from cftuv.envelope_request_policy import envelope_dissolve_uv_slide, envelope_stretch_budget

    started = time.perf_counter()
    run = export_module.run_production(
        controller, bundle, frozenset(selected), case.alpha, source_object_key=key, source_data_key=data_key, density=case.density,
        developable_stretch_budget=envelope_stretch_budget(case.stretch), silhouette_uv_slide=envelope_dissolve_uv_slide(settings.envelope_debug_dissolve_uv_tolerance),
        workers=workers, kernel_backend=kernel_backend_of(mesh_settings),
    )
    return run, time.perf_counter() - started


def _compare(reference: dict, other: dict, label: str, failures: list) -> int:
    if set(reference) != set(other):
        failures.append(f"{label}: domain set differs: only reference {sorted(set(reference) - set(other))[:10]}, only other {sorted(set(other) - set(reference))[:10]}")
    different = 0
    for patch in sorted(set(reference) & set(other)):
        if reference[patch] != other[patch]:
            different += 1
            if different <= 5:
                failures.append(f"{label}: patch {patch} differs: reference {reference[patch]['outcome']} {reference[patch]['detail'][:120]!r} != {other[patch]['outcome']} {other[patch]['detail'][:120]!r}")
    if different:
        failures.append(f"{label}: {different} domains differ from the reference")
    return different


def gate(args, report: dict, failures: list) -> None:
    import bmesh
    import bpy

    ab.load_tree(Path(args.root).resolve(), args.native_path)
    import mpmath  # noqa: F401
    import sympy  # noqa: F401
    import cftuv_envelope.materialize.step  # noqa: F401
    import cftuv_envelope.wavefront  # noqa: F401
    import cftuv.envelope_snapshot_byte_check as byte_check
    from cftuv.analysis import build_analysis_bundle
    from cftuv.analysis_surface import source_revision_from_bmesh
    from cftuv.envelope_debug_session import WINDOW_MANAGER_SESSION_ATTRIBUTE, EnvelopeDebugSessionController
    from cftuv.envelope_domain_pool import get_domain_pool
    from cftuv.envelope_kernel_backend import kernel_backend_of
    from cftuv.envelope_pool_prewarm import prewarm_pool
    from cftuv.envelope_worker_python import read_worker_python

    case = ab.parse_case(args.case)
    report["native"] = ab.verify_native(args.native_path, "")
    ns = argparse.Namespace(scene_kind=case.scene, mesh=case.mesh)
    scene, obj, settings, mesh_settings = ab._open_case(ns)
    if case.offset >= 0:
        mesh_settings.offset = case.offset
    selected = ab.select_edges(obj, "stored" if case.scene == "cover008" else "seam")
    report["selected_edges"] = len(selected)
    report["selection_sha256_16"] = hashlib.sha256(json.dumps(sorted(selected)).encode()).hexdigest()[:16]
    source_bm = bmesh.from_edit_mesh(obj.data)
    source_bm.faces.ensure_lookup_table()
    face_indices = tuple(face.index for face in source_bm.faces)
    key, data_key = int(obj.as_pointer()), int(obj.data.as_pointer())

    def new_session():
        controller = EnvelopeDebugSessionController()
        setattr(bpy.context.window_manager, WINDOW_MANAGER_SESSION_ATTRIBUTE, controller)
        revision = source_revision_from_bmesh(source_bm, obj, face_indices)
        bundle = controller.get_analysis_bundle(key, data_key, revision, lambda: build_analysis_bundle(source_bm, face_indices, obj))
        return controller, bundle

    interpreter = read_worker_python()
    if args.seed_violation:
        _seed_workers(args.seed_violation)
        report["seed_violation_modulus"] = args.seed_violation
    pool = get_domain_pool(args.workers, interpreter)
    report["pool_prewarm_seconds"] = prewarm_pool(args.workers, interpreter)
    report["pool_interpreter"] = str(pool.interpreter)
    backend = kernel_backend_of(mesh_settings)

    # 1. ЭТАЛОН: полная проверка на нажатии, как было до фона.
    os.environ["CFTUV_SNAPSHOT_BYTE_CHECK"] = "all"  # ворота выбирают проверку сами: эталон - полная
    byte_check.forget_checked_identities()
    controller, bundle = new_session()
    reference_run, reference_seconds = _press(controller, bundle, selected, case, settings, mesh_settings, key, data_key, args.workers)
    reference = _rows(reference_run)
    report["reference"] = {"press_seconds": reference_seconds, "counts": _counts(reference_run), "domains": len(reference), "outcomes": _outcomes(reference)}
    refused_reference = {patch for patch, row in reference.items() if row["outcome"] != "MATERIALIZED"}

    # 2. НАЖАТИЕ: умолчание кнопки на свежей сессии, код ещё не проверен.
    os.environ.pop("CFTUV_SNAPSHOT_BYTE_CHECK", None)  # умолчание кнопки продукта
    byte_check.forget_checked_identities()
    controller, bundle = new_session()
    press_run, press_seconds = _press(controller, bundle, selected, case, settings, mesh_settings, key, data_key, args.workers)
    job = controller.byte_check_job
    counts = _counts(press_run)
    deferred = counts.get(byte_check.BYTE_CHECK_DEFERRED, 0)
    latencies = []
    for _ in range(5):
        started = time.perf_counter()
        pool.run([])
        latencies.append(time.perf_counter() - started)
    report["press"] = {"press_seconds": press_seconds, "counts": counts, "deferred": deferred, "job_state_right_after_press": None if job is None else job.state, "pool_run_latency_while_idle_work": latencies}
    # Домены, дошедшие до приёмки снапшота (отказ предполёта источника идёт раньше), одни и те же: эталон проверил каждый, нажатие отложило (или проверило само) каждый.
    reached = deferred + counts.get(byte_check.BYTE_CHECK_FIRST_PRESS, 0)
    if report["reference"]["counts"].get(byte_check.BYTE_CHECK_ALL) != reached:
        failures.append(f"the full check saw {report['reference']['counts']}, the first press decided {counts}")
    if job is None or deferred <= 0:
        failures.append(f"the first press deferred nothing: counts {counts}, job {job}")
        return
    if counts.get(byte_check.BYTE_CHECK_FIRST_PRESS, 0):
        report["press"]["note"] = f"{counts[byte_check.BYTE_CHECK_FIRST_PRESS]} domains were exported by the parent and checked inline"

    # 3. ФОН: конец задачи и вердикт.
    finished = job.wait(JOB_TIMEOUT_SECONDS)
    report["background"] = {
        "finished": finished, "state": job.state, "expected": job.expected, "checked": job.checked, "seconds_from_launch": job.seconds,
        "violations": [{"domain_id": item.domain_id, "outcome": item.outcome_name, "message": item.message[:200]} for item in job.violations], "text": job.text(),
    }
    if not finished:
        failures.append("the background check did not finish in time")
        return
    if job.checked < job.expected:
        failures.append(f"background verified {job.checked} of {job.expected}")
    # Домены эталона, отказанные ПОЛНОЙ проверкой именем круга кодека: фон обязан найти ровно их.
    byte_refused = {reference[patch]["domain_id"] for patch in refused_reference if "fixed point of the codec" in reference[patch]["detail"] or "canonical bytes" in reference[patch]["detail"]}
    found = {item.domain_id for item in job.violations}
    report["background"]["reference_byte_refused"] = len(byte_refused)
    if found != byte_refused:
        failures.append(f"background found {len(found)} domains, the full check refused {len(byte_refused)}: only background {sorted(found - byte_refused)[:5]}, only reference {sorted(byte_refused - found)[:5]}")
    wanted = "VIOLATION" if byte_refused else "CLEAN"
    if job.state != wanted:
        failures.append(f"background state {job.state}, expected {wanted}")

    # 4. СЛЕДУЮЩЕЕ ДЕЙСТВИЕ: исходы и дайджесты всех доменов равны эталону.
    action_run, action_seconds = _press(controller, bundle, selected, case, settings, mesh_settings, key, data_key, args.workers)
    action = _rows(action_run)
    report["next_action"] = {"press_seconds": action_seconds, "counts": _counts(action_run), "outcomes": _outcomes(action)}
    first_press_rows = _rows(press_run)
    report["next_action"]["differs_from_reference"] = _compare(reference, action, "next action", failures)
    report["press"]["differs_from_reference_before_background"] = sum(1 for patch in reference if first_press_rows.get(patch) != reference[patch])
    # Нажатие, кроме отказанных фоном доменов, равно эталону (объявленное изменение поведения - только эти домены).
    unexpected = [patch for patch in reference if first_press_rows.get(patch) != reference[patch] and reference[patch]["domain_id"] not in byte_refused]
    if unexpected:
        failures.append(f"the first press differs from the reference on domains the background did not flag: {unexpected[:10]}")

    # 5. Идентичность кода.
    again = byte_check.resolve_plan(backend, environ={}, defer=True)
    report["identity_checked_after_background"] = not again.first_press
    if again.first_press:
        failures.append("the code identity is not marked checked after a finished background check")


def _outcomes(rows: dict) -> dict:
    found: dict = {}
    for row in rows.values():
        found[row["outcome"]] = found.get(row["outcome"], 0) + 1
    return dict(sorted(found.items()))


def main(argv) -> int:
    args = _arguments(argv)
    report: dict = {"case": args.case, "workers": args.workers, "root": args.root, "seed_violation": args.seed_violation, "started_at": time.time()}
    failures: list = []
    try:
        gate(args, report, failures)
    except Exception:  # noqa: BLE001 - трасса и есть отчёт
        failures.append(traceback.format_exc())
        print(failures[-1])
    try:
        from cftuv.envelope_domain_pool import shutdown_domain_pool

        shutdown_domain_pool()
    except Exception as exc:  # noqa: BLE001
        print("pool shutdown:", type(exc).__name__, exc)
    report["failures"] = failures
    Path(args.out).write_text(json.dumps(report, ensure_ascii=False, indent=1, sort_keys=True, default=str) + "\n", encoding="utf-8")
    print(f"BYTE_CHECK_BACKGROUND_GATE_{'OK' if not failures else 'FAILED'} case={args.case} workers={args.workers} failures={len(failures)}", flush=True)
    return 0 if not failures else 1


if __name__ == "__main__":
    tail = sys.argv[sys.argv.index("--") + 1 :] if "--" in sys.argv else sys.argv[1:]
    code = main(tail)
    if code:
        raise SystemExit(code)
