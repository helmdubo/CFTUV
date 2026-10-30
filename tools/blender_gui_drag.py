"""blender_gui_drag: the REAL width drag in a WINDOWED Blender started by this tool (its own process), measured per redraw and per mouse event.

    python tools/blender_gui_drag.py launch --root <tree> --native-path <dir> --scratch <dir> --tag <name> [--live on] [--settle on] [--gpu off] [--coherent off] [--timeout 900]

THE LAUNCHER (system Python). Waits while other heavy processes run (a headless `blender -b`, `blender_tree_ab`, `pytest`, `cargo`: polled every 60 s, up to 20 min; the wait is reported),
copies the frozen cover scene (hash checked) into `<scratch>/<tag>/`, points every `BLENDER_USER_*` directory and TEMP at `<scratch>` (the owner's preferences, add-ons, recent files and
autosave are never read for writing nor written; sympy and mpmath are copied once into the scratch scripts/modules, the bundled Python has neither), starts
`blender.exe --enable-event-simulate --no-window-focus --window-geometry ... <copy> --python <this file> -- drive ...`, records the PID, waits, and on timeout kills ONLY that PID and its
children (`taskkill /PID <pid> /T /F`). It never touches another Blender.

THE DRIVER (inside that Blender). Loads the tree under test from `--root` with the native directory first (`blender_tree_ab.load_tree`) and prints the stamp (tree, HEAD, dirty files,
native build); switches autosave and preference saving off; presses Build Decal Mesh (the cold build); frames the decal, maximizes the 3D view; then drives the modal operator
`Adjust Decal Width` with SIMULATED events (`Window.event_simulate`, real OS input is ignored by Blender in this mode): the key chord Ctrl+Shift+Alt+W, MOUSEMOVE at 60 Hz along a path of
fast and slow segments with two pauses of `--pause` seconds, then LMB. The switches (`--live`, `--settle`, `--gpu`, `--coherent`) are set on the panel settings before the drag.

WHAT IT RECORDS (one JSON, `<scratch>/<tag>/gui_drag.json`; screenshots `<scratch>/<tag>/shot_*.png`): every viewport redraw (SpaceView3D POST_PIXEL), every mouse event sent, every model
frame (`preview_mesh_now`: its width, how many domains it held, what the regular windows would hold and why, how many distinct exact widths the shown structure comes from), every applied
exact result (tick or COMMIT), every Python mesh write and node input set. Per run: redraw gaps and event-to-redraw latency while the hand moves, held domains and structure generations per
frame, swaps (applied exact results) while moving, per pause and after the release, pause -> first and last exact swap, the committed mesh digest. Then the driver writes the JSON and leaves
with `os._exit` (no quit.blend, no preferences written); nothing is ever saved.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import shutil
import subprocess
import sys
import time
import traceback
from collections import Counter
from pathlib import Path

HERE = Path(__file__).resolve().parent
BLENDER = Path("C:/Program Files/Blender Foundation/Blender 4.5/blender.exe")
FROZEN_COVER = Path("C:/Users/helmd/AppData/Local/Temp/cftuv_cover008_frozen_v2/buildings2_2.blend")
FROZEN_COVER_SHA = "a044b5aa"
#: The hand "moves" while the previous event is younger than this (the settle on pause uses the same 0.12 s); a gap between events longer than PAUSE_GAP is a pause.
MOVING_SECONDS = 0.12
PAUSE_GAP = 0.5
#: Heavy work of other sessions that would disturb the timing: a substring of the command line (lower case) and the process name.
HEAVY = (("blender.exe", " -b "), ("blender.exe", "--background"), ("python.exe", "blender_tree_ab"), ("python.exe", "pytest"), ("cargo.exe", ""), ("rustc.exe", ""))


# --------------------------------------------------------------------------
# Launcher (system Python)
# --------------------------------------------------------------------------


def _processes() -> list:
    script = "Get-CimInstance Win32_Process | ForEach-Object { '{0}|{1}|{2}' -f $_.ProcessId, $_.Name, $_.CommandLine }"
    found = subprocess.run(["powershell", "-NoProfile", "-Command", script], capture_output=True, text=True, timeout=120).stdout
    rows = []
    for line in found.splitlines():
        pid, _sep, rest = line.partition("|")
        name, _sep, command = rest.partition("|")
        if pid.strip().isdigit():
            rows.append((int(pid), name.strip().lower(), command.strip()))
    return rows


def heavy_processes(own: int) -> list:
    """Other sessions' heavy work (a headless Blender, tree_ab, pytest, cargo): `[(pid, name, command), ...]`; this process is not counted."""

    return [(pid, name, command[:160]) for pid, name, command in _processes() if pid != own and any(name == wanted and needle in f" {command.lower()} " for wanted, needle in HEAVY)]


def wait_quiet(limit: float = 1200.0, poll: float = 60.0) -> dict:
    """Waits while other heavy processes run (poll every `poll` s, at most `limit` s): what was seen and how long it waited."""

    started, seen = time.monotonic(), []
    while True:
        busy = heavy_processes(os.getpid())
        if not busy:
            return {"waited_seconds": round(time.monotonic() - started, 1), "seen": seen, "quiet": True}
        seen.append({"at": round(time.monotonic() - started, 1), "busy": busy[:6]})
        if time.monotonic() - started >= limit:
            return {"waited_seconds": round(time.monotonic() - started, 1), "seen": seen, "quiet": False}
        time.sleep(poll)


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1 << 20), b""):
            digest.update(block)
    return digest.hexdigest()


def user_environment(scratch: Path, byte_check: str = "all") -> dict:
    """Environment of the started Blender: every user directory and TEMP in `scratch` (sympy and mpmath copied once into its scripts/modules), CFTUV_* switches cleared.

    The snapshot byte check is pinned to the full check (`CFTUV_SNAPSHOT_BYTE_CHECK=all`, the field tools' reference); `byte_check="product"` leaves the button's own default, for timing
    the drag the way the owner's button runs it (the run JSON records which).
    """

    user = scratch / "bl_user"
    modules = user / "scripts" / "modules"
    owner_modules = Path(os.environ.get("APPDATA", "")) / "Blender Foundation" / "Blender" / "4.5" / "scripts" / "modules"
    for name in ("sympy", "mpmath"):
        if not (modules / name).exists():
            shutil.copytree(owner_modules / name, modules / name)  # read once from the owner's modules, written only into the scratch
    environment = {key: value for key, value in os.environ.items() if not key.startswith("CFTUV_")}
    for key, folder in (("BLENDER_USER_RESOURCES", user), ("BLENDER_USER_CONFIG", user / "config"), ("BLENDER_USER_SCRIPTS", user / "scripts"),
                        ("BLENDER_USER_DATAFILES", user / "datafiles"), ("BLENDER_USER_EXTENSIONS", user / "extensions"), ("TEMP", scratch / "tmp"), ("TMP", scratch / "tmp")):
        folder.mkdir(parents=True, exist_ok=True)
        environment[key] = str(folder)
    environment["PYTHONDONTWRITEBYTECODE"] = "1"
    if byte_check == "all":
        environment["CFTUV_SNAPSHOT_BYTE_CHECK"] = "all"
    return environment


def _png(path: Path, rgba) -> None:
    """An RGBA image (numpy uint8, rows top-down) as a PNG: zlib only."""

    import struct
    import zlib

    import numpy as np

    height, width = int(rgba.shape[0]), int(rgba.shape[1])
    rows = np.concatenate([np.zeros((height, 1), dtype=np.uint8), np.ascontiguousarray(rgba).reshape(height, width * 4)], axis=1).tobytes()

    def chunk(tag: bytes, data: bytes) -> bytes:
        return struct.pack(">I", len(data)) + tag + data + struct.pack(">I", zlib.crc32(tag + data) & 0xFFFFFFFF)

    path.write_bytes(b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 8, 6, 0, 0, 0)) + chunk(b"IDAT", zlib.compress(rows, 6)) + chunk(b"IEND", b""))


def capture_window(pid: int, path: Path) -> None:
    """The client area of the biggest visible window of process `pid`, as DWM composed it (`PrintWindow`, PW_CLIENTONLY | PW_RENDERFULLCONTENT: also when the window is behind others)."""

    import ctypes
    from ctypes import wintypes

    import numpy as np

    user32, gdi32 = ctypes.windll.user32, ctypes.windll.gdi32
    handle = ctypes.c_void_p
    for function, result, arguments in ((user32.GetDC, handle, [handle]), (gdi32.CreateCompatibleDC, handle, [handle]), (gdi32.CreateCompatibleBitmap, handle, [handle, ctypes.c_int, ctypes.c_int]),
                                        (gdi32.SelectObject, handle, [handle, handle]), (user32.PrintWindow, wintypes.BOOL, [handle, handle, wintypes.UINT]),
                                        (gdi32.GetDIBits, ctypes.c_int, [handle, handle, wintypes.UINT, wintypes.UINT, ctypes.c_void_p, ctypes.c_void_p, wintypes.UINT]),
                                        (gdi32.DeleteObject, wintypes.BOOL, [handle]), (gdi32.DeleteDC, wintypes.BOOL, [handle]), (user32.ReleaseDC, ctypes.c_int, [handle, handle]),
                                        (user32.GetClientRect, wintypes.BOOL, [handle, ctypes.POINTER(wintypes.RECT)])):
        function.restype, function.argtypes = result, arguments
    found = []

    @ctypes.WINFUNCTYPE(wintypes.BOOL, handle, wintypes.LPARAM)
    def each(window, _parameter):
        owner = wintypes.DWORD()
        user32.GetWindowThreadProcessId(window, ctypes.byref(owner))
        if owner.value == pid and user32.IsWindowVisible(window):
            rect = wintypes.RECT()
            user32.GetClientRect(window, ctypes.byref(rect))
            found.append((rect.right * rect.bottom, window, rect.right, rect.bottom))
        return True

    user32.EnumWindows(each, 0)
    if not found:
        raise RuntimeError(f"no visible window of pid {pid}")
    _area, window, width, height = max(found, key=lambda item: item[0])
    screen = user32.GetDC(window)
    memory = gdi32.CreateCompatibleDC(screen)
    bitmap = gdi32.CreateCompatibleBitmap(screen, width, height)
    previous = gdi32.SelectObject(memory, bitmap)
    try:
        user32.PrintWindow(window, memory, 3)
        header = (ctypes.c_uint32 * 10)(40, width, (-height) & 0xFFFFFFFF, 1 | (32 << 16), 0, 0, 0, 0, 0, 0)  # BITMAPINFOHEADER: top-down, 32 bit, BI_RGB
        data = ctypes.create_string_buffer(width * height * 4)
        gdi32.GetDIBits(memory, bitmap, 0, height, data, header, 0)
    finally:
        gdi32.SelectObject(memory, previous)
        gdi32.DeleteObject(bitmap)
        gdi32.DeleteDC(memory)
        user32.ReleaseDC(window, screen)
    bgra = np.frombuffer(data.raw, dtype=np.uint8).reshape(height, width, 4)
    rgba = bgra[:, :, [2, 1, 0, 3]].copy()
    rgba[:, :, 3] = 255
    _png(path, rgba)


def capture_requests(pid: int, folder: Path, stop) -> None:
    """Launcher thread: every screenshot request the driver writes (`<folder>/requests/<nn>_<label>`) is captured within ~10 ms as `<folder>/shot_<nn>_<label>.png`."""

    import ctypes

    ctypes.windll.user32.SetProcessDPIAware()  # the window's real pixels, not a scaled copy
    seen = set()
    while not stop.is_set():
        requests = folder / "requests"
        for item in sorted(requests.iterdir()) if requests.exists() else ():
            if item.name in seen:
                continue
            seen.add(item.name)
            try:
                capture_window(pid, folder / f"shot_{item.name}.png")
            except Exception as exc:  # noqa: BLE001 - a missing screenshot is named
                (folder / f"shot_{item.name}.error.txt").write_text(f"{type(exc).__name__}: {exc}", encoding="utf-8")
        stop.wait(0.01)


def _wait(process, timeout: float, temp: Path, since: float) -> tuple:
    """`(exit code or None, killed, crash report path)`: waits for the Blender started here; a crash report written in its TEMP after `since` (Blender then waits on a dialog) or the timeout kills
    ONLY that PID and its children (`taskkill /PID <pid> /T /F`)."""

    end = time.monotonic() + timeout
    while time.monotonic() < end:
        try:
            return process.wait(timeout=2.0), False, ""
        except subprocess.TimeoutExpired:
            crashes = [path for path in temp.glob("*.crash.txt") if path.stat().st_mtime >= since]
            if crashes:
                time.sleep(5.0)
                if process.poll() is None:
                    subprocess.run(["taskkill", "/PID", str(process.pid), "/T", "/F"], capture_output=True)
                return process.poll(), True, str(crashes[0])
    subprocess.run(["taskkill", "/PID", str(process.pid), "/T", "/F"], capture_output=True)  # only the Blender started here and its pool
    return None, True, ""


def launch(args) -> int:
    scratch = Path(args.scratch).resolve()
    run_dir = scratch / args.tag
    run_dir.mkdir(parents=True, exist_ok=True)
    quiet = wait_quiet(args.wait_limit) if args.wait_limit > 0 else {"waited_seconds": 0.0, "seen": [], "quiet": None}
    if not _sha256(FROZEN_COVER).startswith(FROZEN_COVER_SHA):
        raise SystemExit("the frozen cover scene changed: refusing to run")
    scene = run_dir / "scene" / FROZEN_COVER.name
    scene.parent.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(FROZEN_COVER, scene)
    if not _sha256(scene).startswith(FROZEN_COVER_SHA):
        raise SystemExit("the scene copy differs from the frozen scene")
    out = run_dir / "gui_drag.json"
    for stale in [out, *run_dir.glob("shot_*"), *(run_dir / "requests").glob("*")]:
        stale.unlink(missing_ok=True)
    drive = ["--root", args.root, "--native-path", args.native_path, "--out", str(out), "--shots", str(run_dir), "--live", args.live, "--settle", args.settle,
             "--gpu", args.gpu, "--coherent", args.coherent, "--workers", str(args.workers), "--pause", str(args.pause), "--view-mpp", str(args.view_mpp), "--limit", str(args.timeout - 30)]
    command = [str(BLENDER), "--enable-event-simulate", "--no-window-focus", "--window-geometry", "40", "40", "1600", "1000", str(scene), "--python", str(Path(__file__).resolve()), "--", "drive", *drive]
    started = time.monotonic()
    process = subprocess.Popen(command, env=user_environment(scratch, args.byte_check), stdout=(run_dir / "blender.log").open("w", encoding="utf-8"), stderr=subprocess.STDOUT)
    with (scratch / "pids.log").open("a", encoding="utf-8") as stream:
        stream.write(f"{time.strftime('%Y-%m-%dT%H:%M:%S')} {args.tag} pid={process.pid}\n")
    import threading

    stop = threading.Event()
    shooter = threading.Thread(target=capture_requests, args=(process.pid, run_dir, stop), daemon=True)
    shooter.start()
    code, killed, crash = _wait(process, args.timeout, scratch / "tmp", time.time())
    stop.set()
    shooter.join(5.0)
    record = json.loads(out.read_text(encoding="utf-8")) if out.exists() else {}
    if record.get("summary"):
        record["summary"]["shots"] = sorted(str(path) for path in run_dir.glob("shot_*.png"))
        record["summary"]["shot_problems"] = sorted(path.read_text(encoding="utf-8") for path in run_dir.glob("shot_*.error.txt"))
    if crash:
        record["failure"] = (record.get("failure") or "") + f"\nBLENDER_CRASHED: {crash}"
    record["launcher"] = {"pid": process.pid, "exit": code, "killed": killed, "crash_report": crash, "wall": round(time.monotonic() - started, 1), "quiet": quiet,
                          "frozen_scene_unchanged": _sha256(FROZEN_COVER).startswith(FROZEN_COVER_SHA), "scene_copy": str(scene)}
    out.write_text(json.dumps(record, indent=1, default=str) + "\n", encoding="utf-8")
    print(json.dumps({"tag": args.tag, "exit": code, "killed": killed, "summary": record.get("summary"), "failure": (record.get("failure") or "")[-600:]}, indent=1, default=str))
    return 0 if code == 0 and not record.get("failure") else 1


# --------------------------------------------------------------------------
# Driver (inside the started Blender)
# --------------------------------------------------------------------------


def drag_plan(pause: float) -> list:
    """`(kind, seconds from the first move, value)`: the width path as shares of the start width (fast and slow segments, two pauses), the screenshots and the release."""

    plan, clock = [], 0.0
    for start, end, seconds, label in ((0.0, 0.30, 0.5, "fast_up"), (0.30, 0.40, 1.5, "slow_up"), (None, None, pause, "pause1"), (0.40, -0.20, 0.6, "fast_down"),
                                      (-0.20, -0.25, 1.0, "slow_down"), (None, None, pause, "pause2"), (-0.25, 0.10, 0.4, "fast_back"), (None, None, 0.3, "dwell")):
        plan.append(("phase", clock, label))
        if start is None:
            plan.append(("shot", clock + 0.30, f"{label}_early"))
            if label != "dwell":
                plan.append(("shot", clock + seconds - 0.15, f"{label}_late"))
        else:
            count = max(1, round(seconds * 60))
            plan.extend(("move", clock + seconds * (index + 1) / count, start + (end - start) * (index + 1) / count) for index in range(count))
            if label in ("fast_up", "slow_up", "fast_down"):
                plan.append(("shot", clock + seconds * 0.55, f"{label}_moving"))
        clock += seconds
    plan.append(("release", clock, None))
    return sorted(plan, key=lambda item: item[1])


def percentiles(values) -> dict:
    values = sorted(float(item) for item in values)
    if not values:
        return {"n": 0}
    pick = lambda share: round(values[min(len(values) - 1, int(share * len(values)))], 5)  # noqa: E731
    return {"n": len(values), "p50": pick(0.5), "p90": pick(0.9), "p99": pick(0.99), "max": round(values[-1], 5)}


class Recorder:
    """Wrappers around module attributes of the tree under test and the two Blender handlers; everything is a `time.perf_counter` moment."""

    def __init__(self) -> None:
        self.draws, self.deps, self.frames, self.applies, self.writes, self.nodes, self.events, self.shots = [], [], [], [], [], [], [], []
        self.originals = []
        self.handle = None

    def wrap(self, module, name: str, around) -> None:
        original = getattr(module, name)
        setattr(module, name, around(original))
        self.originals.append((module, name, original))

    def install(self, controller) -> None:
        import bpy

        import cftuv.envelope_production_mesh as production_mesh
        import cftuv.envelope_width_live as live
        import cftuv.envelope_width_preview_model as preview_model

        try:
            import cftuv.envelope_width_gpu_drag as gpu
        except ImportError:
            gpu = None
        self.wrap(live, "preview_mesh_now", lambda original: self._frame(original, controller, preview_model))
        self.wrap(live, "_apply", lambda original: self._apply(original, controller))
        for name in ("write_preview_geometry", "rebuild_mesh_from_image"):
            self.wrap(production_mesh, name, lambda original, name=name: self._timed(original, self.writes, name))
        if gpu is not None:
            for name in ("set_width", "install"):
                self.wrap(gpu, name, lambda original, name=name: self._timed(original, self.nodes, name))
        bpy.app.handlers.depsgraph_update_post.append(self._deps)
        self.handle = bpy.types.SpaceView3D.draw_handler_add(self._draw, (), "WINDOW", "POST_PIXEL")

    def _deps(self, *_args) -> None:
        self.deps.append(time.perf_counter())

    def _draw(self) -> None:
        last = self.frames[-1] if self.frames else None
        exact = self.applies[-1] if self.applies else None
        self.draws.append((time.perf_counter(), None if last is None else last["width"], None if exact is None else exact["alpha"]))

    def _timed(self, original, table: list, name: str):
        def wrapper(*args, **kwargs):
            started = time.perf_counter()
            try:
                return original(*args, **kwargs)
            finally:
                table.append((started, time.perf_counter() - started, name))

        return wrapper

    def _frame(self, original, controller, preview_model):
        def wrapper(session_controller, width, owners=None):
            started = time.perf_counter()
            state = original(session_controller, width, owners)
            spent = time.perf_counter() - started
            model = getattr(controller, "width_model", None)
            regular = None if model is None else preview_model.frame_summary(model, float(width))
            log = getattr(controller, "width_live_drag", None)
            self.frames.append({
                "at": started, "seconds": spent, "width": float(width), "held": None if state is None else state.held, "live": None if state is None else state.live,
                "outcome": "NONE" if state is None else state.outcome, "regular_held": None if regular is None else regular.held_domains,
                "reasons": {} if regular is None else dict(Counter(reason for _patch, reason in regular.held)),
                "generations": None if log is None else len(set(log.verified.values())), "gpu": bool(getattr(controller, "width_gpu_rig", None)),
                "extrapolated": getattr(state, "extrapolated", None), "estimated": getattr(state, "estimated", None),
            })
            return state

        return wrapper

    def _apply(self, original, controller):
        def wrapper(session_controller, request, job, value):
            started = time.perf_counter()
            before = getattr(controller, "width_displayed", None)
            try:
                return original(session_controller, request, job, value)
            finally:
                plan = getattr(value, "plan", None)
                after = getattr(controller, "width_displayed", None)
                self.applies.append({
                    "at": started, "seconds": time.perf_counter() - started, "alpha": float(request.alpha), "sequence": int(request.sequence),
                    "grade": str(getattr(request.payload, "grade", "COMMIT")), "requested_at": float(request.requested_at),
                    "layout_changed": before is not None and after is not None and before.layout_digest != after.layout_digest,
                    "plan": None if plan is None else {"incremental": bool(plan.incremental), "reason": plan.reason, "served": len(plan.served or ()), "held": dict(Counter((plan.held or {}).values()))},
                })

        return wrapper

    def remove(self) -> None:
        import bpy

        for module, name, original in reversed(self.originals):
            setattr(module, name, original)
        while self._deps in bpy.app.handlers.depsgraph_update_post:
            bpy.app.handlers.depsgraph_update_post.remove(self._deps)
        if self.handle is not None:
            bpy.types.SpaceView3D.draw_handler_remove(self.handle, "WINDOW")


def tree_stamp(root: Path) -> dict:
    """Which tree runs: its HEAD, how many files differ from it, where `cftuv` was imported from."""

    import cftuv

    def git(*parts):
        try:
            return subprocess.run(["git", "-C", str(root), *parts], capture_output=True, text=True, timeout=60).stdout.strip()
        except (OSError, subprocess.SubprocessError) as exc:
            return f"{type(exc).__name__}: {exc}"

    return {"root": str(root), "head": git("rev-parse", "HEAD"), "dirty": len([line for line in git("status", "--porcelain").splitlines() if line.strip()]), "cftuv": str(Path(cftuv.__file__).resolve())}


def _windows():
    import bpy

    window = bpy.context.window_manager.windows[0]
    areas = [area for area in window.screen.areas if area.type == "VIEW_3D"]
    area = max(areas, key=lambda item: item.width * item.height)
    region = next(item for item in area.regions if item.type == "WINDOW")
    return window, area, region


def _override(window, area, region):
    import bpy

    return bpy.context.temp_override(window=window, screen=window.screen, area=area, region=region)


def aim_view(window, area, source, decal, controller, metres_per_pixel: float) -> None:
    """The view looks at the decal vertex nearest to the centroid of the selected chains (the pivot of the width tool) from the framed direction, at `metres_per_pixel` in the view centre."""

    import numpy as np
    from bpy_extras import view3d_utils
    from mathutils import Vector

    from cftuv.envelope_width_live import resolved_preview_inputs
    from cftuv.envelope_width_preview import chain_centroid

    centre = source.matrix_world @ Vector(chain_centroid(resolved_preview_inputs(controller, controller.width_build)))
    mesh = decal.data
    points = np.empty(len(mesh.vertices) * 3, dtype=np.float64)
    mesh.vertices.foreach_get("co", points)
    points = points.reshape(-1, 3)
    matrix = np.array(decal.matrix_world, dtype=np.float64)
    world = points @ matrix[:3, :3].T + matrix[:3, 3]
    nearest = world[int(np.argmin(((world - np.array(centre)) ** 2).sum(axis=1)))]
    rv3d = area.spaces.active.region_3d
    rv3d.view_location = Vector(nearest)
    area.tag_redraw()


def zoom_view(area, metres_per_pixel: float) -> float:
    """One correction of the view distance towards `metres_per_pixel` in the view centre, measured with the projection of the LAST redraw (call again after a redraw); the measured value."""

    from bpy_extras import view3d_utils

    region = next(item for item in area.regions if item.type == "WINDOW")
    rv3d = area.spaces.active.region_3d
    middle = (region.width / 2.0, region.height / 2.0)
    here = view3d_utils.region_2d_to_location_3d(region, rv3d, middle, rv3d.view_location)
    there = view3d_utils.region_2d_to_location_3d(region, rv3d, (middle[0] + 1.0, middle[1]), rv3d.view_location)
    measured = max((there - here).length, 1e-12)
    rv3d.view_distance *= metres_per_pixel / measured
    area.tag_redraw()
    return measured


def mouse_path(pivot, metres_per_pixel: float, width: float, region_size) -> tuple:
    """`(points of a share, shift)`: the window-region position for a width share `f` lies on a ray from the pivot at radius `r0 + width * f / (metres_per_pixel * gain)`. Shift (fine, x0.1) is held
    when the plain gain would put the whole path within 120 pixels; the ray is the first of eight directions on which the whole path stays inside the region with a 20 px margin."""

    import math

    low, high, middle = -0.25, 0.40, 0.075
    centre = (region_size[0] / 2.0, region_size[1] / 2.0)
    distance = math.hypot(centre[0] - pivot[0], centre[1] - pivot[1])
    rays = [((centre[0] - pivot[0]) / distance, (centre[1] - pivot[1]) / distance, distance)] if distance > 1.0 else []
    rays += [(math.cos(math.pi * step / 4.0), math.sin(math.pi * step / 4.0), 0.0) for step in range(8)]
    for shift in (False, True):
        gain = 0.1 if shift else 1.0
        per_share = width / (metres_per_pixel * gain)
        if not shift and per_share * (high - low) < 120.0:
            continue
        for ux, uy, through in rays:
            # the middle of the path at the region centre when the ray from the pivot passes it, the near end at least 60 px from the pivot
            start = max(through - per_share * middle, 60.0 - per_share * low)
            ends = [(pivot[0] + ux * (start + per_share * share), pivot[1] + uy * (start + per_share * share)) for share in (low, high)]
            if all(20.0 <= x <= region_size[0] - 20.0 and 20.0 <= y <= region_size[1] - 20.0 for x, y in ends):
                return (lambda share, ux=ux, uy=uy, start=start, per=per_share: (pivot[0] + ux * (start + per * share), pivot[1] + uy * (start + per * share))), shift
    raise RuntimeError(f"GUI_DRAG_PATH_DOES_NOT_FIT: pivot {pivot}, {metres_per_pixel:.3g} m/px, width {width:.4g}, region {region_size}")


class Driver:
    """The stages of one GUI run, each a `bpy.app.timers` callback (the event loop runs between them)."""

    def __init__(self, args) -> None:
        self.args = args
        self.report: dict = {"schema": "cftuv.gui-drag.v1", "args": vars(args), "failure": None}
        self.recorder = Recorder()
        self.started = time.perf_counter()
        self.actions, self.index, self.t0, self.release_at, self.last_xy = [], 0, None, None, None
        self.session = {}

    def stage(self, function, delay: float = 0.0) -> None:
        import bpy

        def run():
            try:
                if time.perf_counter() - self.started > self.args.limit:
                    raise TimeoutError(f"GUI_DRAG_TIMEOUT after {self.args.limit} s in {function.__name__}")
                return function()
            except Exception:  # noqa: BLE001 - the traceback is the report
                self.finish(traceback.format_exc())
                return None

        bpy.app.timers.register(run, first_interval=delay)

    def boot(self):
        import bpy

        sys.path.insert(0, str(HERE))
        import blender_tree_ab as tree_ab

        root = Path(self.args.root).resolve()
        tree_ab.load_tree(root, self.args.native_path)
        self.report["native"] = tree_ab.verify_native(self.args.native_path, "")
        self.report["stamp"] = tree_stamp(root)
        print("[CFTUV][gui-drag] STAMP", json.dumps(self.report["stamp"]), "native", self.report["native"]["build_id"], flush=True)
        from cftuv.envelope_debug_session import WINDOW_MANAGER_SESSION_ATTRIBUTE, EnvelopeDebugSessionController
        from cftuv.envelope_domain_pool import get_domain_pool
        from cftuv.envelope_worker_python import read_worker_python

        case = tree_ab.parse_case("cover008")
        window, area, region = _windows()
        with _override(window, area, region):
            scene, obj, settings, mesh_settings = tree_ab._open_case(argparse.Namespace(scene_kind=case.scene, mesh=case.mesh))
        settings.envelope_debug_engine, settings.envelope_debug_workers = "QUEUE", self.args.workers
        settings.envelope_debug_fan_density, settings.envelope_debug_max_stretch, settings.envelope_debug_alpha = case.density, case.stretch, case.alpha
        mesh_settings.kernel_backend, mesh_settings.offset = "NATIVE", case.offset
        controller = getattr(bpy.context.window_manager, WINDOW_MANAGER_SESSION_ATTRIBUTE, None)
        if not isinstance(controller, EnvelopeDebugSessionController):
            controller = EnvelopeDebugSessionController()
            setattr(bpy.context.window_manager, WINDOW_MANAGER_SESSION_ATTRIBUTE, controller)
        get_domain_pool(self.args.workers, read_worker_python())
        try:
            from cftuv.envelope_pool_prewarm import prewarm_pool

            prewarm_pool(self.args.workers, read_worker_python())
        except ImportError:
            pass
        with _override(window, area, region):
            tree_ab.select_edges(obj, "stored")
            started = time.perf_counter()
            assert bpy.ops.hotspotuv.build_envelope_decal_mesh() == {"FINISHED"}
            self.report["cold_press_s"] = round(time.perf_counter() - started, 3)
            if bpy.context.mode != "OBJECT":
                bpy.ops.object.mode_set(mode="OBJECT")
            bpy.ops.screen.screen_full_area()
        self.session = {"scene": scene, "obj": obj, "settings": settings, "mesh_settings": mesh_settings, "controller": controller}
        self.stage(self.frame_view, 0.5)

    def frame_view(self):
        import bpy

        import cftuv.envelope_production_mesh as mesh_module

        window, area, region = _windows()
        obj = self.session["obj"]
        decal = bpy.data.objects[mesh_module.decal_object_name(obj.name)]
        space = area.spaces.active
        space.show_region_ui = False
        with _override(window, area, region):
            for item in window.view_layer.objects:
                item.select_set(item in (obj, decal))
            window.view_layer.objects.active = obj
            bpy.ops.view3d.localview(frame_selected=True)  # the source and its decal alone, framed
            decal.select_set(False)
        aim_view(window, area, obj, decal, self.session["controller"], self.args.view_mpp)
        space.shading.type = "SOLID"
        space.shading.color_type = "OBJECT"
        obj.color = (0.55, 0.55, 0.58, 1.0)
        decal.color, decal.show_wire, decal.show_in_front = (1.0, 0.45, 0.08, 1.0), True, True
        self.session["decal"] = decal
        self.session["zooms"] = []
        self.stage(self.zoom, 0.4)

    def zoom(self):
        _window, area, _region = _windows()
        self.session["zooms"].append(zoom_view(area, self.args.view_mpp))
        if len(self.session["zooms"]) < 3:
            return 0.4
        self.report["view_zoom_measured"] = self.session["zooms"]
        self.stage(self.ready, 1.0)
        return None

    def ready(self):
        import bpy

        from cftuv.envelope_width_session import view_scale_of

        window, area, region = _windows()
        settings, mesh_settings, controller = self.session["settings"], self.session["mesh_settings"], self.session["controller"]
        for name, value in (("drag_live_mesh", self.args.live == "on"), ("drag_settle_on_pause", self.args.settle == "on"), ("drag_gpu_affine", self.args.gpu == "on"), ("drag_coherent", self.args.coherent == "on")):
            if hasattr(mesh_settings, name):
                setattr(mesh_settings, name, value)
            elif value:
                raise RuntimeError(f"GUI_DRAG_SWITCH_MISSING: this tree has no switch {name}")
        with _override(window, area, region):
            view = view_scale_of(bpy.context)
        if view is None:
            raise RuntimeError("GUI_DRAG_NO_VIEW_SCALE")
        width = float(settings.envelope_debug_alpha)
        point, shift = mouse_path(view.pivot, view.metres_per_pixel, width, (region.width, region.height))
        self.report["view"] = {"pivot": view.pivot, "metres_per_pixel": view.metres_per_pixel, "shift": shift, "width": width, "region": [region.x, region.y, region.width, region.height]}
        to_window = lambda share: (int(round(region.x + point(share)[0])), int(round(region.y + point(share)[1])))  # noqa: E731
        self.actions = [("move", -0.30, 0.0), ("chord", -0.20, None)]
        self.actions += [(kind, at, value) for kind, at, value in drag_plan(self.args.pause)]
        self.actions = [(kind, at, value, to_window(value) if kind == "move" else None) for kind, at, value in self.actions]
        self.shift = shift
        self.recorder.install(controller)
        self.shot("start")
        self.t0 = time.perf_counter() + 0.4
        self.stage(self.drive, 0.0)

    def shot(self, label: str) -> None:
        """A screenshot request for the launcher (it captures the window from outside: `capture_window`). Inside Blender, the screenshot operator under a context override crashed it twice
        (`screenshot_data_create` redraws the windows while the override is live: `BPY_context_member_get`, and the allocator of the compute thread), and the framebuffer of a draw callback
        is not the composed view in 4.5."""

        number = len(self.recorder.shots)
        name = f"{number:02d}_{label}"
        (Path(self.args.shots) / "requests").mkdir(exist_ok=True)
        (Path(self.args.shots) / "requests" / name).write_text(str(time.perf_counter()), encoding="utf-8")
        self.recorder.shots.append({"label": label, "at": time.perf_counter(), "seconds": 0.0, "path": str(Path(self.args.shots) / f"shot_{name}.png"), "problem": ""})

    def send(self, kind: str, xy, value: str, **modifiers) -> None:
        window, _area, _region = _windows()
        window.event_simulate(type=kind, value=value, x=xy[0], y=xy[1], **modifiers)
        self.recorder.events.append((time.perf_counter(), kind, value, xy))

    def drive(self):
        now = time.perf_counter() - self.t0
        due = []
        while self.index < len(self.actions) and self.actions[self.index][1] <= now:
            due.append(self.actions[self.index])
            self.index += 1
        moves = [item for item in due if item[0] == "move"]
        for item in due:
            kind, at, value, xy = item
            if kind == "move" and item is moves[-1]:
                self.last_xy = xy
                self.send("MOUSEMOVE", xy, "NOTHING", shift=self.shift)
                self.recorder.events[-1] += (at, value, len(moves) - 1)
            elif kind == "chord":
                self.send("W", self.last_xy, "PRESS", ctrl=True, shift=True, alt=True)
                self.send("W", self.last_xy, "RELEASE", ctrl=True, shift=True, alt=True)
            elif kind == "shot":
                self.shot(value)
            elif kind == "phase":
                self.recorder.events.append((time.perf_counter(), "PHASE", value, None))
            elif kind == "release":
                self.send("LEFTMOUSE", self.last_xy, "PRESS", shift=self.shift)
                self.send("LEFTMOUSE", self.last_xy, "RELEASE", shift=self.shift)
                self.release_at = time.perf_counter()
                self.stage(self.wait_commit, 0.12)
                return None
        if self.index >= len(self.actions):
            return None
        return max(0.001, self.actions[self.index][1] - (time.perf_counter() - self.t0))

    def wait_commit(self):
        commits = [item for item in self.recorder.applies if item["grade"] == "COMMIT" and item["at"] >= self.release_at]
        if not self.session.get("released_shot"):
            self.session["released_shot"] = True
            self.shot("released")
        if commits:
            self.report["commit_landed_after"] = round(commits[0]["at"] + commits[0]["seconds"] - self.release_at, 4)
            self.stage(self.after_commit, 0.4)
            return None
        if time.perf_counter() - self.release_at > 90.0:
            self.report["commit_landed_after"] = None
            self.stage(self.after_commit, 0.0)
            return None
        return 0.05

    def after_commit(self):
        self.shot("after_commit")
        self.stage(lambda: self.finish(None), 0.4)
        return None

    def finish(self, failure) -> None:
        import bpy

        self.report["failure"] = failure
        try:
            self.recorder.remove()
            if failure is None:
                self.report.update(analyse(self.recorder, self.release_at, float(self.report["view"]["width"])))
                import cftuv.envelope_production_mesh as mesh_module

                self.report["final_mesh_digest"] = mesh_module.mesh_content_digest(self.session["decal"].data)
                self.report["final_width"] = float(self.session["settings"].envelope_debug_alpha)
                log = getattr(self.session["controller"], "width_live_drag", None)
                if log is not None:
                    from cftuv.envelope_width_stream import report as stream_report

                    self.report["drag_log"] = stream_report(log)
                self.report["preview_log"] = {key: value for key, value in vars_of(getattr(self.session["controller"], "width_preview_log", None)).items() if not isinstance(value, float) or value == value}
        except Exception:  # noqa: BLE001
            self.report["failure"] = (failure or "") + "\nanalysis: " + traceback.format_exc()
        self.report["raw"] = {"draws": self.recorder.draws, "frames": self.recorder.frames, "applies": self.recorder.applies, "events": self.recorder.events,
                              "writes": self.recorder.writes, "nodes": self.recorder.nodes, "shots": self.recorder.shots, "deps": len(self.recorder.deps), "release_at": self.release_at}
        Path(self.args.out).write_text(json.dumps(self.report, indent=1, default=str) + "\n", encoding="utf-8")
        print("[CFTUV][gui-drag]", json.dumps(self.report.get("summary"), default=str), flush=True)
        print("GUI_DRAG_OK" if self.report["failure"] is None else "GUI_DRAG_FAILED", flush=True)
        try:
            from cftuv.envelope_domain_pool import shutdown_domain_pool

            shutdown_domain_pool()
        except Exception as exc:  # noqa: BLE001 - leaving anyway
            print("pool shutdown:", type(exc).__name__, exc, flush=True)
        bpy.context.preferences.use_preferences_save = False
        os._exit(0 if self.report["failure"] is None else 1)  # no quit.blend, no preferences, nothing saved


def vars_of(item) -> dict:
    if item is None:
        return {}
    names = getattr(type(item), "__slots__", ()) or tuple(getattr(item, "__dict__", {}))
    return {name: getattr(item, name) for name in names if not name.startswith("_")}


# --------------------------------------------------------------------------
# Analysis (plain lists of the recorder)
# --------------------------------------------------------------------------


def motion_windows(events: list, release_at) -> tuple:
    """`(moving spans, pauses)` from the MOUSEMOVE moments: a span is a run of events with gaps under PAUSE_GAP; a pause is the gap between two spans (its last event, its next event)."""

    moves = [item[0] for item in events if item[1] == "MOUSEMOVE"]
    spans, pauses, start = [], [], None
    for index, moment in enumerate(moves):
        start = moment if start is None else start
        last = index + 1 == len(moves) or moves[index + 1] - moment > PAUSE_GAP
        if last:
            spans.append((start, moment))
            if index + 1 < len(moves):
                pauses.append((moment, moves[index + 1]))
            start = None
    if moves and release_at is not None:
        pauses.append((moves[-1], release_at))  # the dwell before the release: the last rest
    return spans, pauses, moves


def _moving(moment: float, moves: list) -> bool:
    before = [item for item in moves if item <= moment]
    after = [item for item in moves if item > moment]
    return bool(before) and moment - before[-1] < MOVING_SECONDS and bool(after) and after[0] - before[-1] < PAUSE_GAP


def analyse(recorder: Recorder, release_at, width: float) -> dict:
    spans, pauses, moves = motion_windows(recorder.events, release_at)
    shots = [(item["at"], item["at"] + item["seconds"]) for item in recorder.shots]
    draws = [item[0] for item in recorder.draws]
    gaps, jumps = [], []
    for (at, shown, _exact), (later, shown_later, _exact_later) in zip(recorder.draws, recorder.draws[1:]):
        if any(start <= at <= end for start, end in spans) and not any(at - 0.01 <= start <= later for start, _end in shots):
            gaps.append(later - at)
            if shown is not None and shown_later is not None:
                jumps.append(100.0 * abs(shown_later - shown) / width)
    latency = []
    for moment, kind, *_rest in recorder.events:
        if kind != "MOUSEMOVE" or not any(start <= moment <= end for start, end in spans):
            continue
        frame = next((item for item in recorder.frames if item["at"] >= moment), None)
        draw = None if frame is None else next((item for item in draws if item >= frame["at"] + frame["seconds"]), None)
        if draw is not None and not any(moment - 0.01 <= start <= draw for start, _end in shots):
            latency.append(draw - moment)
    moving_frames = [item for item in recorder.frames if any(start <= item["at"] <= end + MOVING_SECONDS for start, end in spans)]
    reasons: Counter = Counter()
    for item in moving_frames:
        reasons.update(item["reasons"])
    ticks = [item for item in recorder.applies if item["grade"] != "COMMIT"]
    per_pause = []
    for last, resumed in pauses:
        inside = [item for item in ticks if last < item["at"] <= resumed]
        per_pause.append({"rest_seconds": round(resumed - last, 3), "swaps": len(inside), "layout_changes": sum(1 for item in inside if item["layout_changed"]),
                          "first_exact_after": None if not inside else round(inside[0]["at"] + inside[0]["seconds"] - last, 4),
                          "last_exact_after": None if not inside else round(inside[-1]["at"] + inside[-1]["seconds"] - last, 4), "widths": [round(item["alpha"], 6) for item in inside]})
    moving_swaps = [item for item in ticks if _moving(item["at"], moves)]
    held = [item["held"] for item in moving_frames if item["held"] is not None]
    summary = {
        "draws_moving": len(gaps) + 1 if gaps else 0, "redraw_gap_moving": percentiles(gaps), "gaps_over_33ms": sum(1 for gap in gaps if gap > 0.033), "gaps_over_50ms": sum(1 for gap in gaps if gap > 0.05),
        "event_to_redraw": percentiles(latency), "shown_width_jump_percent": percentiles(jumps),
        "frames_moving": len(moving_frames), "held_moving": percentiles(held), "frames_with_held_moving": sum(1 for value in held if value),
        "regular_held_moving": percentiles([item["regular_held"] for item in moving_frames if item["regular_held"] is not None]),
        "held_reasons_moving_frame_count": dict(reasons), "generations_moving": percentiles([item["generations"] for item in moving_frames if item["generations"] is not None]),
        "extrapolated_moving": percentiles([item["extrapolated"] for item in moving_frames if item["extrapolated"] is not None]),
        "estimated_moving": percentiles([item["estimated"] for item in moving_frames if item["estimated"] is not None]),
        "frame_cost": percentiles([item["seconds"] for item in moving_frames]), "swaps_moving": len(moving_swaps), "swaps_total_before_release": len([item for item in ticks if release_at is None or item["at"] < release_at]),
        "pauses": per_pause, "python_writes": dict(Counter(item[2] for item in recorder.writes)), "node_calls": dict(Counter(item[2] for item in recorder.nodes)),
        "apply_cost": percentiles([item["seconds"] for item in ticks]), "shots": [item["path"] for item in recorder.shots if not item["problem"]],
        "shot_problems": [item["problem"] for item in recorder.shots if item["problem"]], "events_sent": sum(1 for item in recorder.events if item[1] == "MOUSEMOVE"),
    }
    return {"summary": summary}


def _arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("mode", choices=("launch", "drive"))
    parser.add_argument("--root", required=True)
    parser.add_argument("--native-path", default="")
    parser.add_argument("--scratch", default="")
    parser.add_argument("--tag", default="run")
    parser.add_argument("--out", default="")
    parser.add_argument("--shots", default="")
    for name in ("live", "settle"):
        parser.add_argument(f"--{name}", default="on", choices=("on", "off"))
    for name in ("gpu", "coherent"):
        parser.add_argument(f"--{name}", default="off", choices=("on", "off"))
    parser.add_argument("--byte-check", default="all", choices=("all", "product"), help="all: the full snapshot byte check (reference); product: the button's default (timing)")
    parser.add_argument("--workers", type=int, default=12)
    parser.add_argument("--pause", type=float, default=1.5)
    parser.add_argument("--view-mpp", type=float, default=1.5e-3, help="metres per pixel in the view centre (the view looks at the decal next to the pivot)")
    parser.add_argument("--timeout", type=float, default=900.0)
    parser.add_argument("--wait-limit", type=float, default=1200.0, help="seconds to wait while other heavy processes run (0: do not wait)")
    parser.add_argument("--limit", type=float, default=870.0)
    argv = sys.argv[sys.argv.index("--") + 1 :] if "--" in sys.argv else sys.argv[1:]
    return parser.parse_args(argv)


def main() -> int:
    args = _arguments()
    if args.mode == "launch":
        return launch(args)
    import faulthandler

    import bpy

    sys.dont_write_bytecode = True
    faulthandler.enable(file=(Path(args.shots) / "faulthandler.log").open("w", encoding="utf-8"), all_threads=True)  # a crash names the Python stack of every thread
    preferences = bpy.context.preferences
    preferences.use_preferences_save = False
    preferences.filepaths.use_auto_save_temporary_files = False
    preferences.view.smooth_view = 0
    preferences.view.show_splash = False
    driver = Driver(args)
    driver.stage(driver.boot, 1.0)
    return 0


if __name__ == "__main__":
    code = main()
    if code:
        raise SystemExit(code)
