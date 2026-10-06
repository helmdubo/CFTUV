"""A/B нативного ядра против Python-эталона на полевых случаях: исход, дайджест батча, цена, секунды — по каждому домену.

    blender -b E:\\testscene.blend --python-exit-code 1 --python tools/blender_native_ab.py -- \\
        [--cases mesh:alpha:density:stretch,...] [--steps 3] [--width-step 0.01] [--workers 0] \\
        [--native-path <каталог с cftuv_native>] [--install-dispatch] [--root <дерево>] [--out <json>]

Каждый случай (по умолчанию 22 полевых — рецепт `field_rel.py`: меш, alpha, плотность веера, допуск растяжения) считается кнопкой
(`run_production`, настройка `Kernel backend` читается из свойства сцены, как у оператора) на ряде ширин `alpha + width_step * n`,
`n = 0..steps`, ДВАЖДЫ: бэкендом `PYTHON` и бэкендом `NATIVE`. Кэши сессии, память стадии резки и (при `--workers`) пул воркеров между
двумя прогонами сбрасываются, порядок чередуется от случая к случаю (процессные `lru_cache` ядра иначе грели бы второго). Домены
сравниваются попарно по номеру патча: исход, дайджест содержимого батча и семантический дайджест — это ОТВЕТ (расхождение — код
возврата 1); шесть статей `EXACT_WORK_*` — ЦЕНА (расхождение тоже код 1: нативное ядро обязано стоить столько же, сколько эталон);
секунды и кто на самом деле посчитал (запись бэкенда домена: `native` / `python` / `mixed`, названный откат) — только в таблице.

`--install-dispatch` ставит диспетчеры бэкенда на место вызова эталона подменой имён в модулях ядра (`backend.install_dispatch`), не правя файлов
ядра: без неё (и пока точки не подключены в самом ядре) заказ `NATIVE` не зовёт ни одной нативной операции, и каждый домен назван
`NATIVE_NOT_REACHED`; сравнивать тогда нечего, и отчёт говорит об этом строкой `NOTE`.

Без нативного ядра (`cftuv_native` не импортируется) отчёт называет статус `UNAVAILABLE`, пишет JSON и завершается нулём: сравнивать
нечего, и это не ошибка. Нативное ядро, которое импортируется, но стоит не на той версии эталона (`stale(...)`), сравнивается: откаты на
Python названы в таблице, ответ при этом тот же. Последняя строка: `NATIVE_AB_OK|NATIVE_AB_UNAVAILABLE|NATIVE_AB_FAILED ...`.

Blender только headless, без `--factory-startup`, без сохранения .blend. Аддон берётся из дерева репозитория (установленный снимается).
"""

from __future__ import annotations

import argparse
import json
import sys
import time
import traceback
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
#: 22 полевых случая `mesh:alpha:density:stretch` (рецепт `field_rel.py`).
FIELD_CASES = (
    "2:0.25:2:20,2.001:0.25:2:20,building:0.25:2:20,building.001:0.25:2:20,building.002:0.25:2:20,building.003:0.25:2:20,"
    "building.004:0.25:2:20,CFTUV_D_PERIODIC_CYLINDER:0.25:2:20,half_sphere:0.25:2:20,rounded_wall.001:0.25:2:20,"
    "rounded_wall_noise_top:0.25:2:20,sagging_wall:0.25:2:20,wall_noise_top:0.25:2:20,walls.001:0.25:2:20,walls.002:0.25:2:20,"
    "walls.003:0.25:2:20,walls.006:0.25:2:20,sagging_wall:0.987:2:42,rounded_wall_noise_top:0.5:2:42,"
    "rounded_wall_noise_top:0.2239:2:42,sagging_wall:0.2239:2:42,building:0.2239:2:42"
)
PRICE_PREFIX = "EXACT_WORK_"
STATUS_OK, STATUS_UNAVAILABLE, STATUS_FAILED = "OK", "UNAVAILABLE", "FAILED"


# --------------------------------------------------------------------------
# Чистая часть: строка домена, сравнение, сводка, таблица (без Blender, под тестом)
# --------------------------------------------------------------------------


def domain_row(result) -> dict:
    """Строка домена: ответ (исход, дайджесты), цена (`EXACT_WORK_*`), секунды и запись бэкенда."""

    counters = dict(result.counters)
    record = getattr(result, "backend_record", None)
    batch = result.batch
    return {
        "patch_id": int(result.patch_id),
        "outcome": str(result.outcome),
        "content_digest": result.content_digest,
        "semantic_digest": "" if batch is None else batch.semantic_digest.value,
        "prices": {name: value for name, value in sorted(counters.items()) if name.startswith(PRICE_PREFIX)},
        "seconds": float(result.seconds),
        "placement": str(result.placement),
        "step_path": str(result.step_path),
        "ran": "python" if record is None else record.ran,
        "native_calls": 0 if record is None else record.native_calls,
        "python_calls": 0 if record is None else record.python_calls,
        "fallbacks": [] if record is None else list(record.outcomes),
    }


def compare_domains(python_rows, native_rows) -> dict:
    """`{answer: [...], price: [...], missing: [...]}`: различия двух прогонов по номеру патча (пусто — побитово то же)."""

    first = {item["patch_id"]: item for item in python_rows}
    second = {item["patch_id"]: item for item in native_rows}
    found: dict = {"answer": [], "price": [], "missing": []}
    for patch in sorted(first.keys() | second.keys()):
        if patch not in first or patch not in second:
            found["missing"].append(f"patch {patch}: only in {'python' if patch in first else 'native'}")
            continue
        one, other = first[patch], second[patch]
        for name in ("outcome", "content_digest", "semantic_digest"):
            if one[name] != other[name]:
                found["answer"].append(f"patch {patch}: {name} {one[name]!r} != {other[name]!r}")
        if one["prices"] != other["prices"]:
            names = sorted(name for name in one["prices"].keys() | other["prices"].keys() if one["prices"].get(name) != other["prices"].get(name))
            found["price"].append(f"patch {patch}: {', '.join(names)}")
    return found


def summarize_width(width, python_rows, native_rows) -> dict:
    """Одна ширина случая: различия, секунды двух бэкендов, кто посчитал в нативном прогоне и названные откаты."""

    computed = [item for item in native_rows if item["placement"] != "cache"]
    python_seconds = sum(item["seconds"] for item in python_rows if item["placement"] != "cache")
    native_seconds = sum(item["seconds"] for item in computed)
    ran = {"native": 0, "python": 0, "mixed": 0}
    fallbacks: dict = {}
    for item in computed:
        ran[item["ran"]] += 1
        for name in item["fallbacks"]:
            fallbacks.setdefault(name, []).append(item["patch_id"])
    differences = compare_domains(python_rows, native_rows)
    return {
        "width": width,
        "domains": len(native_rows),
        "differences": differences,
        "python_seconds": round(python_seconds, 3),
        "native_seconds": round(native_seconds, 3),
        "speedup": round(python_seconds / native_seconds, 2) if native_seconds else None,
        "ran": ran,
        "fallbacks": {name: sorted(set(found)) for name, found in sorted(fallbacks.items())},
    }


def has_differences(summary: dict) -> bool:
    return any(summary["differences"][kind] for kind in ("answer", "price", "missing"))


def _fallback_text(fallbacks: dict) -> str:
    return "; ".join(f"{name}: {len(patches)}" for name, patches in fallbacks.items())


def format_table(report: dict) -> str:
    """Таблица случаев и ширин: различия ответа и цены, секунды, исполнитель и откаты."""

    header = ("case", "width", "doms", "ans", "price", "py s", "nat s", "x", "native/python/mixed", "fallbacks")
    rows = [header]
    for case in report.get("cases", ()):
        if "failure" in case:
            rows.append((case["case"], "-", "-", "-", "-", "-", "-", "-", "-", "FAILED: " + case["failure"].strip().splitlines()[-1][:60]))
            continue
        for item in case["widths"]:
            differences = item["differences"]
            ran = item["ran"]
            rows.append(
                (
                    case["case"],
                    f"{item['width']:g}",
                    str(item["domains"]),
                    str(len(differences["answer"]) + len(differences["missing"])),
                    str(len(differences["price"])),
                    f"{item['python_seconds']:.2f}",
                    f"{item['native_seconds']:.2f}",
                    "-" if item["speedup"] is None else f"{item['speedup']:.2f}",
                    f"{ran['native']}/{ran['python']}/{ran['mixed']}",
                    _fallback_text(item["fallbacks"]),
                )
            )
    widths = [max(len(row[column]) for row in rows) for column in range(len(header))]
    lines = ["  ".join(cell.ljust(widths[column]) for column, cell in enumerate(row)).rstrip() for row in rows]
    lines.insert(1, "  ".join("-" * width for width in widths))
    return "\n".join(lines)


def unavailable_report(status: dict, root: str, cases) -> dict:
    """Отчёт без сравнения: нативного ядра нет (названный статус), прогонять нечего."""

    return {"status": STATUS_UNAVAILABLE, "native_status": status, "root": root, "cases": [], "planned": list(cases)}


def final_line(report: dict) -> str:
    status = report["status"]
    if status == STATUS_UNAVAILABLE:
        native = report["native_status"]
        return f"NATIVE_AB_UNAVAILABLE coverage={native['coverage']} clip={native['clip']} detail={native['detail']!r}"
    totals = report["totals"]
    return (
        f"NATIVE_AB_{status} cases={totals['cases']} domains={totals['domains']} answer_differences={totals['answer']} "
        f"price_differences={totals['price']} python_seconds={totals['python_seconds']:.1f} native_seconds={totals['native_seconds']:.1f} "
        f"native_domains={totals['native']} python_domains={totals['python']} mixed_domains={totals['mixed']}"
    )


def totals_of(cases) -> dict:
    totals = {"cases": 0, "domains": 0, "answer": 0, "price": 0, "python_seconds": 0.0, "native_seconds": 0.0, "native": 0, "python": 0, "mixed": 0}
    for case in cases:
        totals["cases"] += 1
        for item in case.get("widths", ()):
            differences = item["differences"]
            totals["domains"] += item["domains"]
            totals["answer"] += len(differences["answer"]) + len(differences["missing"])
            totals["price"] += len(differences["price"])
            totals["python_seconds"] += item["python_seconds"]
            totals["native_seconds"] += item["native_seconds"]
            for name in ("native", "python", "mixed"):
                totals[name] += item["ran"][name]
    return totals


# --------------------------------------------------------------------------
# Blender: дерево, меш, прогон кнопки
# --------------------------------------------------------------------------


def _arguments():
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", default=str(ROOT))
    parser.add_argument("--cases", default=FIELD_CASES)
    parser.add_argument("--steps", type=int, default=3)
    parser.add_argument("--width-step", type=float, default=0.01)
    parser.add_argument("--workers", type=int, default=0)
    parser.add_argument("--native-path", default="")
    parser.add_argument("--install-dispatch", action="store_true")
    parser.add_argument("--out", default="")
    return parser.parse_args(sys.argv[sys.argv.index("--") + 1 :] if "--" in sys.argv else [])


def _load_tree(root: Path, native_path: str) -> None:
    installed = sys.modules.get("cftuv")
    if installed is not None:
        try:
            installed.unregister()
        except Exception as exc:  # noqa: BLE001 - диагностика окружения, не причина отказа
            print("installed unregister failed:", type(exc).__name__, exc)
    for name in tuple(sys.modules):
        if name in {"cftuv", "cftuv_envelope"} or name.startswith(("cftuv.", "cftuv_envelope.")):
            del sys.modules[name]
    for path in (root / "kernel" / "src", root):
        text = str(path)
        if text in sys.path:
            sys.path.remove(text)
        sys.path.insert(0, text)
    if native_path:
        sys.path.append(native_path)  # после дерева: колесо не затеняет ядро, а ядро — колесо
    import cftuv
    import cftuv_envelope

    assert Path(cftuv.__file__).resolve().parent == (root / "cftuv").resolve()
    assert Path(cftuv_envelope.__file__).resolve().parent == (root / "kernel" / "src" / "cftuv_envelope").resolve()
    cftuv.register()


def _select_seams(obj) -> list:
    import bmesh
    import bpy

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
    chosen = []
    for edge in bm.edges:
        edge.select_set(bool(edge.seam))
        if edge.seam:
            chosen.append(edge.index)
    bmesh.update_edit_mesh(obj.data)
    return chosen


def _fresh_state(controller, workers: int) -> None:
    """Кэши сессии, память стадии резки и пул воркеров сброшены: прогон одного бэкенда не читает память другого."""

    from cftuv.envelope_domain_pool import shutdown_domain_pool
    from cftuv_envelope.materialize.clip_memo import MEMO

    controller.clear()
    MEMO.clear()
    MEMO.reset_stats()
    if workers:
        shutdown_domain_pool()


def _run_backend(controller, spec: str, backend: str, args) -> list:
    """Строки доменов по ширинам случая под заказанным бэкендом: `[(ширина, [строка домена, ...]), ...]`."""

    import bmesh
    import bpy

    from cftuv.analysis import build_analysis_bundle
    from cftuv.analysis_surface import source_revision_from_bmesh
    from cftuv.envelope_kernel_backend import kernel_backend_of
    from cftuv.envelope_production_export import run_production
    from cftuv.envelope_request_policy import envelope_dissolve_uv_slide, envelope_stretch_budget

    mesh_name, alpha_text, density, stretch = spec.split(":")
    settings = bpy.context.scene.hotspotuv_settings
    mesh_settings = bpy.context.scene.hotspotuv_decal_mesh
    mesh_settings.kernel_backend = backend  # тем же путём, каким его выставит панель
    assert kernel_backend_of(mesh_settings) == backend
    _fresh_state(controller, args.workers)
    obj = bpy.data.objects[mesh_name]
    selected = _select_seams(obj)
    source_bm = bmesh.from_edit_mesh(obj.data)
    source_bm.faces.ensure_lookup_table()
    face_indices = tuple(face.index for face in source_bm.faces)
    revision = source_revision_from_bmesh(source_bm, obj, face_indices)
    object_key, data_key = int(obj.as_pointer()), int(obj.data.as_pointer())
    bundle = controller.get_analysis_bundle(
        object_key, data_key, revision, lambda: build_analysis_bundle(source_bm, face_indices, obj)
    )
    slide = envelope_dissolve_uv_slide(settings.envelope_debug_dissolve_uv_tolerance)
    widths = []
    for number in range(args.steps + 1):
        width = round(float(alpha_text) + args.width_step * number, 6)
        run = run_production(
            controller,
            bundle,
            frozenset(selected),
            width,
            source_object_key=object_key,
            source_data_key=data_key,
            density=density,
            developable_stretch_budget=envelope_stretch_budget(int(stretch)),
            silhouette_uv_slide=slide,
            workers=args.workers,
            kernel_backend=mesh_settings.kernel_backend,
        )
        assert run.kernel_backend == backend
        widths.append((width, [domain_row(item) for item in run.results]))
    if bpy.context.mode != "OBJECT":
        bpy.ops.object.mode_set(mode="OBJECT")
    return widths


def _case(controller, spec: str, order: int, args) -> dict:
    """Один случай: оба бэкенда (порядок чередуется), сравнение по ширинам."""

    sequence = ("PYTHON", "NATIVE") if order % 2 == 0 else ("NATIVE", "PYTHON")
    runs: dict = {}
    seconds: dict = {}
    for backend in sequence:
        started = time.perf_counter()
        runs[backend] = _run_backend(controller, spec, backend, args)
        seconds[backend] = round(time.perf_counter() - started, 2)
    python, native = runs["PYTHON"], runs["NATIVE"]
    assert [item[0] for item in python] == [item[0] for item in native]
    widths = [summarize_width(width, rows, native[index][1]) for index, (width, rows) in enumerate(python)]
    return {"case": spec, "order": list(sequence), "wall_seconds": seconds, "widths": widths}


def main() -> int:
    args = _arguments()
    root = Path(args.root).resolve()
    _load_tree(root, args.native_path)
    import bpy

    from cftuv_envelope.backend import UNAVAILABLE, install_dispatch, native_status

    status = native_status()
    print("native status:", json.dumps(status.as_record()), flush=True)
    installed = list(install_dispatch()) if args.install_dispatch else []
    cases = [item for item in args.cases.split(",") if item]
    if status.coverage == UNAVAILABLE and status.clip == UNAVAILABLE:
        report = unavailable_report(status.as_record(), str(root), cases)
    else:
        from cftuv.envelope_debug_session import WINDOW_MANAGER_SESSION_ATTRIBUTE, EnvelopeDebugSessionController

        controller = getattr(bpy.context.window_manager, WINDOW_MANAGER_SESSION_ATTRIBUTE, None)
        if not isinstance(controller, EnvelopeDebugSessionController):
            controller = EnvelopeDebugSessionController()
            setattr(bpy.context.window_manager, WINDOW_MANAGER_SESSION_ATTRIBUTE, controller)
        results = []
        for order, spec in enumerate(cases):
            try:
                case = _case(controller, spec, order, args)
            except Exception:  # noqa: BLE001 - причина идёт в отчёт и в код возврата
                case = {"case": spec, "failure": traceback.format_exc()}
                print(case["failure"], flush=True)
            results.append(case)
            print("AB_CASE", json.dumps({key: value for key, value in case.items() if key != "widths"}), flush=True)
        totals = totals_of(results)
        failed = any("failure" in case for case in results) or totals["answer"] or totals["price"]
        report = {
            "status": STATUS_FAILED if failed else STATUS_OK,
            "native_status": status.as_record(),
            "dispatch_installed": installed,
            "root": str(root),
            "workers": args.workers,
            "steps": args.steps,
            "width_step": args.width_step,
            "cases": results,
            "totals": totals,
        }
        print(format_table(report), flush=True)
        if totals["native"] + totals["mixed"] == 0:
            print("NOTE: no native operation was called; dispatch installed: " + (", ".join(installed) or "no (pass --install-dispatch)"), flush=True)
        for case in results:
            for item in case.get("widths", ()):
                for kind in ("answer", "price", "missing"):
                    for line in item["differences"][kind][:5]:
                        print(f"DIFFERENT[{kind}] {case['case']} width {item['width']:g}: {line}", flush=True)
    if args.out:
        Path(args.out).write_text(json.dumps(report, ensure_ascii=False, indent=1, sort_keys=True) + "\n", encoding="utf-8")
    try:
        from cftuv.envelope_domain_pool import shutdown_domain_pool

        shutdown_domain_pool()
    except Exception as exc:  # noqa: BLE001 - уборка не меняет итог
        print("pool shutdown:", type(exc).__name__, exc)
    print(final_line(report), flush=True)
    return 1 if report["status"] == STATUS_FAILED else 0


if __name__ == "__main__":
    code = main()
    if code:
        raise SystemExit(code)
