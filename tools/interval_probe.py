"""interval_probe: заверенный интервал ширины (`cftuv_envelope.materialize.interval`) против настоящей устойчивости структуры.

    blender -b E:/testscene.blend --python-exit-code 1 --python tools/interval_probe.py -- --mesh building --base 0.2239 \\
        [--density 2] [--stretch 42] [--dissolve 0.390625] [--samples 4] [--out <json>]

Меш считается кнопкой («Build Decal Mesh», `run_production`) один раз при `--base`; у каждого домена берётся записанный интервал
(`ProductionDomainResultV1.alpha_interval`) и подпись структуры. Дальше каждый домен считается ТЕМ ЖЕ `produce_domain` на подготовке
сессии при ширинах `base * (1 +- 0.001)` и `base * (1 +- 0.01)` и при ширинах внутри и чуть за границами его интервала. Печатается:

* ширины интервалов (медиана, в процентах alpha, по обе стороны) и доли шагов 0.1 % и 1 %, которые остаются внутри записи;
* таблица «внутри интервала / структура та же» по шагам: `inside_same` (честный попадание), `inside_changed` (структуру изменил класс,
  которого интервал не заверяет: диагонали ячейки, допуск силуэта и др. - перечень в записи), `outside_same` (консервативный запас),
  `outside_changed` (граница поймала изменение);
* то же для ширин, выбранных ВНУТРИ интервала (`--samples` на домен) и чуть за его границами;
* доля компонент положения вершин, аффинных по ширине, у доменов с той же структурой (основа будущего быстрого пути);
* цену записи: секунды таблицы событий (первый вызов домена) и подписи структуры.

Ничего не пишется в сцену и не сохраняется. Серийно (`workers=0`, пула нет): домены считает этот процесс. Последняя строка при успехе:
`INTERVAL_PROBE_OK`.
"""

from __future__ import annotations

import argparse
import json
import statistics
import sys
import time
import traceback
from pathlib import Path

import bmesh
import bpy

ROOT = Path(__file__).resolve().parents[1]
STEPS = (0.001, 0.01)


def _arguments():
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", default=str(ROOT))
    parser.add_argument("--mesh", required=True)
    parser.add_argument("--base", type=float, required=True)
    parser.add_argument("--density", type=int, default=2)
    parser.add_argument("--stretch", type=int, default=42)
    parser.add_argument("--dissolve", type=float, default=0.390625)
    parser.add_argument("--samples", type=int, default=4)
    parser.add_argument("--out", default="")
    return parser.parse_args(sys.argv[sys.argv.index("--") + 1 :] if "--" in sys.argv else [])


def _load_tree(root: Path) -> None:
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
    import cftuv
    import cftuv_envelope

    assert Path(cftuv.__file__).resolve().parent == (root / "cftuv").resolve()
    assert Path(cftuv_envelope.__file__).resolve().parent == (root / "kernel" / "src" / "cftuv_envelope").resolve()
    cftuv.register()


def _select_seams(obj) -> list:
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


def _positions(result) -> dict:
    return {item.vert_key.value: (item.position.x, item.position.y, item.position.z) for item in result.batch.vertices}


def _affine_share(low, middle, high, tolerance=1e-10) -> tuple[int, int]:
    """`(аффинных, всех)` компонент положения: середина равна середине соседей (шаги по ширине равноудалены)."""

    total = affine = 0
    for key, value in middle.items():
        if key not in low or key not in high:
            continue
        for index, component in enumerate(value):
            total += 1
            midpoint = (low[key][index] + high[key][index]) / 2
            if abs(component - midpoint) / max(abs(component), 1e-3) <= tolerance:
                affine += 1
    return affine, total


def _median(values):
    return statistics.median(values) if values else None


def _install_timers() -> dict:
    """Секундомеры цены записи: таблица событий (один раз на подготовку) и подпись структуры (на каждый батч)."""

    import cftuv_envelope.materialize.domain as domain_module
    import cftuv_envelope.materialize.interval as interval_module

    spent = {"event_table_seconds": 0.0, "event_tables": 0, "structure_seconds": 0.0, "structures": 0}
    build_table, batch_structure = interval_module._build_table, domain_module.batch_structure

    def timed(function, seconds, count):
        def wrapper(*arguments):
            started = time.perf_counter()
            try:
                return function(*arguments)
            finally:
                spent[seconds] += time.perf_counter() - started
                spent[count] += 1

        return wrapper

    interval_module._build_table = timed(build_table, "event_table_seconds", "event_tables")
    domain_module.batch_structure = timed(batch_structure, "structure_seconds", "structures")
    return spent


class _Session:
    """Сцена, кнопка и подготовки домена: `evaluate(патч, ширина)` считает домен тем же `produce_domain`, что воркер."""

    def __init__(self, args) -> None:
        import cftuv.envelope_production_export as export_module
        from cftuv.analysis import build_analysis_bundle
        from cftuv.analysis_surface import source_revision_from_bmesh
        from cftuv.envelope_debug_session import WINDOW_MANAGER_SESSION_ATTRIBUTE, EnvelopeDebugSessionController

        self.export_module = export_module
        self.captured: dict = {}
        self.original = export_module.produce_domain

        def produce(patch_id, domain_id, prepared, alpha_text, **kwargs):
            self.captured[int(patch_id)] = (domain_id, prepared, kwargs)
            return self.original(patch_id, domain_id, prepared, alpha_text, **kwargs)

        export_module.produce_domain = produce
        controller = getattr(bpy.context.window_manager, WINDOW_MANAGER_SESSION_ATTRIBUTE, None)
        if not isinstance(controller, EnvelopeDebugSessionController):
            controller = EnvelopeDebugSessionController()
            setattr(bpy.context.window_manager, WINDOW_MANAGER_SESSION_ATTRIBUTE, controller)
        controller.clear()
        obj = bpy.data.objects[args.mesh]
        self.selected = _select_seams(obj)
        source_bm = bmesh.from_edit_mesh(obj.data)
        source_bm.faces.ensure_lookup_table()
        face_indices = tuple(face.index for face in source_bm.faces)
        revision = source_revision_from_bmesh(source_bm, obj, face_indices)
        self.keys = (int(obj.as_pointer()), int(obj.data.as_pointer()))
        self.controller = controller
        self.bundle = controller.get_analysis_bundle(
            *self.keys, revision, lambda: build_analysis_bundle(source_bm, face_indices, obj)
        )

    def press(self, args):
        from cftuv.envelope_request_policy import envelope_dissolve_uv_slide, envelope_stretch_budget

        return self.export_module.run_production(
            self.controller, self.bundle, frozenset(self.selected), args.base,
            source_object_key=self.keys[0], source_data_key=self.keys[1], density=args.density,
            developable_stretch_budget=envelope_stretch_budget(args.stretch),
            silhouette_uv_slide=envelope_dissolve_uv_slide(args.dissolve),
            workers=0, domain_pool=None,
        )

    def evaluate(self, patch_id, alpha):
        domain_id, prepared, kwargs = self.captured[patch_id]
        return self.original(patch_id, domain_id, prepared, str(float(alpha)), **kwargs)


class _Tally:
    """Счёт зонда: шаги ширины против записи, ширины внутри и за границами, аффинность положения."""

    def __init__(self) -> None:
        self.steps = {step: {"inside_same": 0, "inside_changed": 0, "outside_same": 0, "outside_changed": 0} for step in STEPS}
        self.sampled = {"inside_same": 0, "inside_changed": 0, "beyond_same": 0, "beyond_changed": 0}
        self.statuses: dict = {}
        self.low_percent: list = []
        self.high_percent: list = []
        self.affine = [0, 0]
        self.rows: list = []

    def step_share(self, step) -> dict:
        counts = self.steps[step]
        total = sum(counts.values())
        inside = counts["inside_same"] + counts["inside_changed"]
        return {"steps": total, "inside_record": inside, "inside_fraction": round(inside / total, 4) if total else None, **counts}


def _probe_steps(session, args, patch_id, result, tally) -> None:
    """Шаги `base * (1 +- step)`: внутри ли записи и осталась ли структура; положение трёх соседних ширин на аффинность."""

    record = result.alpha_interval
    neighbours = {}
    for step in STEPS:
        for sign in (-1, 1):
            alpha = args.base * (1 + sign * step)
            other = session.evaluate(patch_id, alpha)
            if not other.is_materialized:
                continue
            same = other.structure_digest == result.structure_digest
            tally.steps[step][("inside" if record.contains(alpha) else "outside") + ("_same" if same else "_changed")] += 1
            if same and step == STEPS[0]:
                neighbours[sign] = _positions(other)
    if len(neighbours) == 2:
        found = _affine_share(neighbours[-1], _positions(result), neighbours[1])
        tally.affine[0] += found[0]
        tally.affine[1] += found[1]


def _probe_bounds(session, args, patch_id, result, tally) -> None:
    """Ширины ВНУТРИ интервала (подпись обязана совпасть, кроме классов без заверения) и чуть за его границами (подпись должна уйти)."""

    record = result.alpha_interval
    high = record.high if record.high is not None else record.alpha * 3
    inside = [
        record.alpha + (edge - record.alpha) * fraction
        for edge in (record.low, high)
        for fraction in (0.03, 0.5, 0.97)[: max(1, args.samples // 2)]
    ]
    beyond = [record.low - (record.low * 0.002 + 1e-7)] if record.low > 0 else []
    if record.high is not None:
        beyond.append(record.high * 1.002 + 1e-7)
    for label, widths in (("inside", [value for value in inside if record.low < value < high and value > 0]), ("beyond", beyond)):
        for width in widths:
            other = session.evaluate(patch_id, width)
            if other.is_materialized:
                tally.sampled[label + ("_same" if other.structure_digest == result.structure_digest else "_changed")] += 1


def _probe_domain(session, args, patch_id, result, tally) -> None:
    record = result.alpha_interval
    status = "MISSING" if record is None else record.status
    tally.statuses[status] = tally.statuses.get(status, 0) + 1
    row = {"patch": patch_id, "status": status, "structure": result.structure_digest}
    tally.rows.append(row)
    if record is None:
        return
    row.update(low=record.low, high=record.high, events=record.events, clip_events=record.clip_events, reason=record.reason)
    if record.status == "CERTIFIED":
        tally.low_percent.append(100.0 * (record.alpha - record.low) / record.alpha)
        if record.high is not None:
            tally.high_percent.append(100.0 * (record.high - record.alpha) / record.alpha)
    _probe_steps(session, args, patch_id, result, tally)
    if record.status == "CERTIFIED":
        _probe_bounds(session, args, patch_id, result, tally)


def _summary(args, run, base, tally, spent) -> dict:
    return {
        "mesh": args.mesh,
        "base": args.base,
        "domains": len(run.results),
        "materialized": len(base),
        "status": tally.statuses,
        "median_low_percent_of_alpha": _median(tally.low_percent),
        "median_high_percent_of_alpha": _median(tally.high_percent),
        "unbounded_above": sum(1 for row in tally.rows if row.get("status") == "CERTIFIED" and row.get("high") is None),
        "steps": {f"{step * 100:g}%": tally.step_share(step) for step in STEPS},
        "sampled": tally.sampled,
        "affine_position_components_at_0.1%": {"affine": tally.affine[0], "total": tally.affine[1]},
        "price_of_the_record": {key: round(value, 4) if isinstance(value, float) else value for key, value in spent.items()},
    }


def main() -> int:
    args = _arguments()
    _load_tree(Path(args.root).resolve())
    spent = _install_timers()
    session = _Session(args)
    failure, summary, tally = None, {}, _Tally()
    try:
        run = session.press(args)
        base = {item.patch_id: item for item in run.results if item.is_materialized}
        print(f"[probe] {args.mesh} alpha={args.base} domains={len(run.results)} materialized={len(base)}", flush=True)
        for patch_id, result in sorted(base.items()):
            _probe_domain(session, args, patch_id, result, tally)
        summary = _summary(args, run, base, tally, spent)
    except Exception:  # noqa: BLE001 - зонд печатает причину и падает кодом возврата
        failure = traceback.format_exc()
        print(failure)
    print("[probe] SUMMARY " + json.dumps(summary, ensure_ascii=False, default=str), flush=True)
    if args.out:
        Path(args.out).write_text(
            json.dumps({"summary": summary, "rows": tally.rows, "failure": failure}, ensure_ascii=False, indent=1, sort_keys=True, default=str) + "\n",
            encoding="utf-8",
        )
    from cftuv.envelope_domain_pool import shutdown_domain_pool

    shutdown_domain_pool()
    print("INTERVAL_PROBE_" + ("FAILED" if failure else "OK"), flush=True)
    return 1 if failure else 0


if __name__ == "__main__":
    code = main()
    if code:
        raise SystemExit(code)
