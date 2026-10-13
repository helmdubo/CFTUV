"""A/B of two source TREES (a base and a branch) on the same scene: is the product's answer the same, and how long does it take?

    python tools/blender_tree_ab.py --base origin/main --branch my-branch --scene path/to/scene.blend --case building --case rounded \
        [--workers 0,8] [--presses cold,step,step] [--timing --reps 3] [--native-path DIR] [--blender EXE] [--work-dir DIR] [--out report.json]

Typical use: a performance change that must not move the answer. The scene is opened by a fresh HEADLESS Blender (`blender -b`, never a window: `require_headless` refuses anything else) for every (tree, case, worker count, repetition); the
button is replayed call for call (`run_production` + `build_mesh_arrays`, the cold press under `gc_paused()` when the tree has it, as the operator does; the steps, ticks and confirms too when the tree has GC_PAUSED_DURING_WIDTH_TICK_V1, as the live path does) and then the width
steps the live width tool takes, on ONE controller, the way a session does. The scene is never saved.

INPUTS
  --base, --branch   a directory (used as it is) or a git ref (frozen with `git archive` into <work-dir>/trees/<sha>, reused when it is there already).
  --scene            the .blend (`PATH` is the `testscene`; `KEY=PATH` names another scene a case needs, e.g. `cover008=...`). The SHA-256 of every scene is taken before the first
                     run and after the last: a change is a failure (`--expect-scene-sha KEY=HEX` also pins the content).
  --case             `building`, `rounded`, `two`, `sagging0987`, `sagging02239`, `half_sphere`, `cover008` (the built-in recipes), or `NAME=mesh:alpha:density:stretch[:offset[:scene]]`.
  --workers          comma list, `0` = every domain in this process, `8` = the pool. Each count is compared with the SAME count of the other tree.
  --presses          `cold,step,step`: `cold` is the first press (alpha), each `step` widens by `--step-width` (0.0022), as the width tool does.
                     A drag is `cold,tick,tick,...,confirm`: a `tick` is an exact refresh DURING the drag (a tree that has `run_production(lean=...)` runs it as WIDTH_REFRESH_LEAN_V1,
                     a tree that has not runs the full path), `confirm` is the commit at the next width (always the full path). Ticks are compared on geometry, outcomes and counters
                     (digests of the batch are not computed by a lean tick); the confirm is compared like any press, so a lean tick must leave no trace in the committed answer.
                     `incr` (WIDTH_REFRESH_INCREMENTAL_V1) is the tick the tool orders while dragging when the tree has `envelope_width_mesh_preview.plan_tick`: the tree's policy decides whether it is
                     incremental (the domains the certificate serves are moved by it, the rest computed) or a full lean tick (a verification); a tree without it runs a lean tick. An incremental press is
                     compared with the other tree's lean tick of the same width as: every domain the tick COMPUTED bit-identical (answer fields, counters, the hash of its view), every domain the certificate
                     SERVED within `SERVED_TOLERANCE` of the exact mesh coordinates (`DISPLAY_TOLERANCE` for the display-approximate developable domains and their wall partners: they move by FROZEN offset
                     normals; a served domain whose structure digest differs from the exact one is a FLIP: listed in the notes and in the press record, not a failure, it is what the verification policy exists for),
                     the mesh layout (faces, seams, domain tokens) the same unless a flip, the mesh positions within the same tolerances, the owner ordinals of the served domains label-only (the partition of the faces
                     into owner groups is compared: it must be the same).
  --native-path      a directory that holds the `cftuv_native` package, put ahead of everything in BOTH trees and in the pool workers; without it the extension is the one Blender can import.
  --env KEY=VALUE    environment for every Blender (e.g. `CFTUV_SNAPSHOT_BYTE_CHECK=all`), the same for both trees.
  --keys-may-differ  the content keys of the two sides are not compared (listed in the notes): the identity of the executor of a stage is part of a key, so two settings of one tree differ in every key by construction.
  --base-env, --branch-env KEY=VALUE   environment for the Blenders of ONE tree (after --env). With the same directory on both sides this is an A/B of two settings of one tree, e.g.
                     `--base X --branch X --base-env CFTUV_EFFECTIVE_ALPHA_BACKEND=PYTHON --branch-env CFTUV_EFFECTIVE_ALPHA_BACKEND=NATIVE`: the answers are compared and, with --timing, the cold press.

WHAT IS COMPARED, per domain and press (any difference is a failure; the exit code is 1)
  outcome, detail, content and semantic digests, EVERY counter (the six `EXACT_WORK_*` included), diagnostics, normals (and their digests), topology law, orientation, structure digest;
  the set of domains; the hash of the whole mesh (positions, faces, UVs, owners, seams: floats as hex); the content key of every domain the parent computed (the code identity is pinned
  to the same constants in both trees, otherwise two different sources would differ in every key by construction); the counters of the run and of its domains, profile receipts and stages.
  Counters that depend on WHICH pool worker took a domain (the worker-local interval memo, the blob store, the clip memo) are compared by the SUM of their group; timing counters (`*_US`, `*_SECONDS`,
  `*_MS`) and byte counters of the run are not answers. `--allow-counter PREFIX` names counters a change adds on purpose: they are listed, never silently ignored.
  The process counters of the kernel's symbolic backend (`symbolic_backend.BACKEND_COUNTS`, `planar_types.SYMBOLIC_FALLBACK_COUNTS`) are compared too, per press, when the press was computed in this
  process (`--workers 0`; a pool worker keeps counters of its own, which the parent does not see). Label-only fields (step path, placement, clip memo, who ran, who ran the effective-alpha stage) are counted
  and listed, not failures; the stage follows the master switch in a tree that has it (NATIVE by default; EFFECTIVE_ALPHA_NATIVE_DEFAULT_V1) and `--env CFTUV_EFFECTIVE_ALPHA_BACKEND=PYTHON` isolates it to the reference (the other tree ignores the variable), and the notes name how many of its pairs each tree computed natively. A run that failed, or a pool worker that imported another tree, another native build
  or another sympy than the parent (the origin check), is a failure of the run.

TIMING (`--timing --reps N`)  alternating base/branch, order flipped every repetition; before each run the tool waits while a FOREIGN blender.exe is busy (it never stops one) and re-runs a
  repetition during which one was; every Blender it starts is recorded in <work-dir>/pids.log and nothing else is ever stopped. Medians, minima and the change are in the table.

OUTPUT  a JSON report (`--out`, default <work-dir>/tree_ab_<time>.json) and a short table. Exit 0: identical (and the scene unchanged); 1: a difference or a failed run; 2: a request
that cannot be run (no Blender, a tree without the add-on, a missing scene, a scene that changed).
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import re
import shutil
import statistics
import subprocess
import sys
import tarfile
import tempfile
import time
import traceback
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path

SCHEMA = "cftuv.tree-ab.v1"
STEP_WIDTH = 0.0022
EXIT_IDENTICAL, EXIT_DIFFERENT, EXIT_UNRUNNABLE = 0, 1, 2
ABSENT = "<absent>"
#: Pinned in BOTH trees while the keys are recorded: the identity hashes the sources, so two different trees would differ in every key by construction.
PINNED_IDENTITY = ("0123456789abcdef", "fedcba9876543210")
#: Fields of a domain row that are the PRODUCT'S ANSWER.
ANSWER_FIELDS = (
    "outcome", "detail", "content_digest", "semantic_digest", "diagnostics", "normal", "offset_normals_digest", "vertex_normals_hash",
    "decal_topology_law", "chart_orientation", "structure_digest", "view_hash",
)
#: Fields a WIDTH_REFRESH_LEAN_V1 domain does not carry (no batch to digest): skipped when either side's row is lean, and counted in the notes.
LEAN_UNCARRIED_FIELDS = ("content_digest", "semantic_digest", "vertex_normals_hash")
#: Counters a lean run adds or moves on purpose: its own class counters, and the registration in the content store (a lean result is never stored).
LEAN_COUNTER_PREFIXES = ("PRODUCTION_REFRESH_LEAN", "PRODUCTION_CONTENT_REGISTERED")
#: Fields that say HOW the answer was reached (counted and listed, never a failure).
LABEL_FIELDS = ("step_path", "placement", "clip_memo", "ran", "effective_alpha_ran", "materialize_ran", "snapshot_check_ran")
#: Counters whose value depends on which worker a domain landed on; each group is compared by its sum.
SCHEDULING_GROUPS = {
    "interval path": lambda name: name == "PRODUCTION_INTERVAL_FAST_HITS" or name.startswith("PRODUCTION_INTERVAL_FALLBACK_"),
    "pool blobs": lambda name: name in ("PRODUCTION_POOL_BLOB_HITS", "PRODUCTION_POOL_BLOBS_SHIPPED"),
    # the memo of the clip stage lives in each worker: a domain that lands on a worker that has seen it is a hit, on another one a miss (a base-vs-base run of the pool shows 2/3 and 1/4)
    "clip memo": lambda name: name.startswith("PRODUCTION_CLIP_MEMO_"),
}
TIMING_SUFFIXES = ("_US", "_SECONDS", "_MS")
#: WIDTH_REFRESH_INCREMENTAL_V1. The label of a domain served by its certificate, the refresh class that names an incremental press, the tolerance of a served domain against the exact one (metres and UV
#: units: rounding of binary64 after a shift of up to tens of percent of the width) and of a mesh vertex that a developable domain moves by FROZEN offset normals (`DISPLAY_APPROXIMATE_ERROR_LIMIT`).
PLACEMENT_REBASED = "certificate"
REFRESH_INCREMENTAL = "WIDTH_REFRESH_INCREMENTAL_V1"
SERVED_TOLERANCE = 1e-9
DISPLAY_TOLERANCE = 1e-4
#: Counters that an incremental press moves on purpose: its own class counters, the cache hits (a served domain is not a cache hit), the interval path counters (a served domain is not stepped) and the
#: registration in the content store (a lean result is never stored).
INCREMENTAL_COUNTER_PREFIXES = ("PRODUCTION_REFRESH_", "PRODUCTION_RESULT_CACHE_", "PRODUCTION_INTERVAL_", "PRODUCTION_CONTENT_REGISTERED", "PRODUCTION_CLIP_MEMO_", "PRODUCTION_ALPHA_INTERVALS_")


def is_timing_counter(name: str) -> bool:
    return name.endswith(TIMING_SUFFIXES)


def is_volatile_run_counter(name: str) -> bool:
    return is_timing_counter(name) or "BYTES" in name


def scheduling_group(name: str) -> str | None:
    return next((title for title, member in SCHEDULING_GROUPS.items() if member(name)), None)


# --------------------------------------------------------------------------
# Cases, presses, trees (pure)
# --------------------------------------------------------------------------


@dataclass(frozen=True)
class Case:
    name: str
    mesh: str
    alpha: float
    density: str = "2"
    stretch: int = 42
    offset: float = -1.0
    scene: str = "testscene"


BUILT_IN_CASES = {
    case.name: case
    for case in (
        Case("building", "building", 0.2239),
        Case("rounded", "rounded_wall_noise_top", 0.2239),
        Case("two", "2", 0.2239),
        Case("sagging0987", "sagging_wall", 0.987),
        Case("sagging02239", "sagging_wall", 0.2239),
        Case("half_sphere", "half_sphere", 0.2239),
        Case("cover008", "cover.008", 0.04214220866560936, "2", 20, 0.004999999888241291, "cover008"),
    )
}


def parse_case(text: str) -> Case:
    """A built-in name, or `NAME=mesh:alpha:density:stretch[:offset[:scene]]`."""

    if text in BUILT_IN_CASES:
        return BUILT_IN_CASES[text]
    name, separator, spec = text.partition("=")
    parts = spec.split(":")
    if not separator or not name or not 4 <= len(parts) <= 6:
        raise ValueError(f"case {text!r}: a built-in name ({', '.join(BUILT_IN_CASES)}) or NAME=mesh:alpha:density:stretch[:offset[:scene]]")
    return Case(name, parts[0], float(parts[1]), parts[2], int(parts[3]), float(parts[4]) if len(parts) > 4 else -1.0, parts[5] if len(parts) > 5 else "testscene")


#: Kinds of the presses after the cold one (`warm` is the old name of `step`); each widens by the step width over the previous press.
LATER_PRESSES = {"step": "step", "warm": "step", "tick": "tick", "incr": "incr", "confirm": "confirm"}


def parse_presses(text: str, alpha: float, step_width: float = STEP_WIDTH) -> list[tuple[str, str, float]]:
    """`cold,step,tick,confirm` -> `[(label, kind, alpha)]`: `cold`, `step1`, `tick1`, `confirm1`; every press after the cold one widens by `step_width` over the previous one."""

    presses, steps, numbers = [], 0, {}
    for kind in (item.strip() for item in text.split(",") if item.strip()):
        if kind == "cold":
            if presses:
                raise ValueError("`cold` is the first press only: later presses of one session are steps")
            presses.append(("cold", "cold", alpha))
        elif kind in LATER_PRESSES:
            if not presses:
                raise ValueError("the first press is `cold`")
            name = LATER_PRESSES[kind]
            steps += 1
            numbers[name] = numbers.get(name, 0) + 1
            presses.append((f"{name}{numbers[name]}", name, round(alpha + step_width * steps, 6)))
        else:
            raise ValueError(f"press {kind!r}: `cold`, `step`, `tick` or `confirm`")
    if not presses:
        raise ValueError("no presses")
    return presses


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with open(path, "rb") as handle:
        for block in iter(lambda: handle.read(1 << 20), b""):
            digest.update(block)
    return digest.hexdigest()


def _git(arguments: list[str], cwd: Path, check: bool = True) -> subprocess.CompletedProcess:
    return subprocess.run(["git", *arguments], cwd=str(cwd), capture_output=True, check=check)


@dataclass
class Tree:
    label: str
    spec: str
    root: Path
    commit: str = ""
    dirty: bool | None = None

    def as_record(self) -> dict:
        return {"label": self.label, "spec": self.spec, "root": str(self.root), "commit": self.commit, "dirty": self.dirty}


def freeze_ref(ref: str, work_dir: Path, repository: Path) -> tuple[Path, str]:
    """`git archive <ref>` into <work-dir>/trees/<commit>: the tree of a ref frozen, not a checkout that moves under a run."""

    commit = _git(["rev-parse", "--verify", f"{ref}^{{commit}}"], repository).stdout.decode().strip()
    target = Path(work_dir) / "trees" / commit
    if (target / "cftuv").is_dir() and (target / ".frozen").is_file():
        return target, commit
    shutil.rmtree(target, ignore_errors=True)
    target.mkdir(parents=True)
    handle, name = tempfile.mkstemp(suffix=".tar", dir=str(work_dir))
    os.close(handle)
    archive = Path(name)
    try:
        _git(["archive", "--format=tar", "-o", str(archive), commit], repository)
        with tarfile.open(archive) as tar:
            tar.extractall(target, filter="data") if sys.version_info >= (3, 12) else tar.extractall(target)
    finally:
        archive.unlink(missing_ok=True)
    (target / ".frozen").write_text(commit + "\n", encoding="utf-8")
    return target, commit


def resolve_tree(label: str, spec: str, work_dir: Path, repository: Path) -> Tree:
    """A directory is used as it is (with the commit and the dirtiness recorded when it is a work tree); anything else must be a git ref."""

    path = Path(spec)
    if path.is_dir():
        root = path.resolve()
        tree = Tree(label, spec, root)
        probe = _git(["rev-parse", "HEAD"], root, check=False)
        if probe.returncode == 0:
            tree.commit = probe.stdout.decode().strip()
            tree.dirty = bool(_git(["status", "--porcelain", "--", "cftuv", "kernel/src"], root, check=False).stdout.strip())
    else:
        try:
            root, commit = freeze_ref(spec, work_dir, repository)
        except subprocess.CalledProcessError as error:
            raise ValueError(f"{label} {spec!r} is neither a directory nor a git ref of {repository}: {error.stderr.decode(errors='replace').strip()}") from error
        tree = Tree(label, spec, root, commit, False)
    if not (tree.root / "cftuv" / "__init__.py").is_file() or not (tree.root / "kernel" / "src" / "cftuv_envelope").is_dir():
        raise ValueError(f"{label} {spec!r}: {tree.root} is not a CFTUV tree (cftuv/ and kernel/src/cftuv_envelope/ expected)")
    return tree


# --------------------------------------------------------------------------
# Comparison (pure: this is what the unit test exercises)
# --------------------------------------------------------------------------


@dataclass
class Comparison:
    differences: list = field(default_factory=list)
    notes: list = field(default_factory=list)
    domains_compared: int = 0
    presses: dict = field(default_factory=dict)

    @property
    def identical(self) -> bool:
        return not self.differences


def _short(value, limit: int = 220) -> str:
    return str(value)[:limit]


def _counter_difference(first: dict, second: dict, allowed: tuple, volatile, where: str, out: Comparison, groups: bool = True) -> None:
    """Exact per counter, except: volatile ones, the scheduling groups (by sum, here or - `groups=False` - over the whole press by the caller) and the counters a change names on purpose (listed)."""

    def listed(name):
        return any(name.startswith(prefix) for prefix in allowed)

    exact_a = {name: value for name, value in first.items() if not volatile(name) and scheduling_group(name) is None and not listed(name)}
    exact_b = {name: value for name, value in second.items() if not volatile(name) and scheduling_group(name) is None and not listed(name)}
    for name in sorted(set(exact_a) | set(exact_b)):
        if exact_a.get(name, ABSENT) != exact_b.get(name, ABSENT):
            out.differences.append(f"{where}: counter {name}: base {exact_a.get(name, ABSENT)} != branch {exact_b.get(name, ABSENT)}")
    for title in SCHEDULING_GROUPS if groups else ():
        sum_a = sum(value for name, value in first.items() if scheduling_group(name) == title)
        sum_b = sum(value for name, value in second.items() if scheduling_group(name) == title)
        if sum_a != sum_b:
            out.differences.append(f"{where}: scheduling group '{title}' by sum: base {sum_a} != branch {sum_b}")
    named = {name: (first.get(name, ABSENT), second.get(name, ABSENT)) for name in sorted(set(first) | set(second)) if listed(name) and first.get(name) != second.get(name)}
    if named:
        out.notes.append(f"{where}: counters named on purpose (--allow-counter) differ: {named}")


def _compare_rows(label: str, first: dict, second: dict, allowed: tuple, out: Comparison) -> None:
    rows_a = {row["patch"]: row for row in first["rows"]}
    rows_b = {row["patch"]: row for row in second["rows"]}
    if set(rows_a) != set(rows_b):
        out.differences.append(f"[{label}] DOMAIN SET: only base {sorted(set(rows_a) - set(rows_b))[:20]}, only branch {sorted(set(rows_b) - set(rows_a))[:20]}")
    common = sorted(set(rows_a) & set(rows_b))
    out.domains_compared += len(common)
    for patch in common:
        a, b = rows_a[patch], rows_b[patch]
        lean = bool(a.get("lean") or b.get("lean"))
        for name in ANSWER_FIELDS:
            if lean and name in LEAN_UNCARRIED_FIELDS:
                continue
            if a.get(name, ABSENT) != b.get(name, ABSENT):
                out.differences.append(f"[{label}] patch {patch} {name}: base {_short(a.get(name, ABSENT))} != branch {_short(b.get(name, ABSENT))}")
        _counter_difference_domain(label, patch, a, b, allowed, out)
    leaned = sum(1 for patch in common if rows_a[patch].get("lean") or rows_b[patch].get("lean"))
    if leaned:
        out.notes.append(f"[{label}] {leaned} lean (WIDTH_REFRESH_LEAN_V1) domains: batch digests {', '.join(LEAN_UNCARRIED_FIELDS)} not carried, compared on everything else")
    for name in LABEL_FIELDS:
        changed = sum(1 for patch in common if rows_a[patch].get(name) != rows_b[patch].get(name))
        if changed:
            out.notes.append(f"[{label}] label-only field {name}: {changed} domains differ")
    group_totals = {}
    for title in SCHEDULING_GROUPS:
        group_totals[title] = [sum(value for row in rows.values() for name, value in row.get("counters_all", {}).items() if scheduling_group(name) == title) for rows in (rows_a, rows_b)]
        if group_totals[title][0] != group_totals[title][1]:
            out.differences.append(f"[{label}] domain counters, scheduling group '{title}' by sum over the domains: base {group_totals[title][0]} != branch {group_totals[title][1]}")


def _counter_difference_domain(label: str, patch, a: dict, b: dict, allowed: tuple, out: Comparison) -> None:
    first, second = a.get("counters_all", {}), b.get("counters_all", {})
    if a.get("counters_all") is None or b.get("counters_all") is None:
        out.differences.append(f"[{label}] patch {patch}: the counters of the domain are missing in one of the runs")
        return
    _counter_difference(first, second, allowed, is_timing_counter, f"[{label}] patch {patch}", out, groups=False)


def _compare_press(label: str, a: dict, b: dict, allowed: tuple, out: Comparison, compare_keys: bool = True) -> None:
    if a["mesh_hash"] != b["mesh_hash"]:
        out.differences.append(f"[{label}] MESH HASH differs: base {a['mesh_hash'][:16]} {a['mesh_counts']} vs branch {b['mesh_hash'][:16]} {b['mesh_counts']}")
    keys_a, keys_b = [tuple(item) for item in a["content_keys"]], [tuple(item) for item in b["content_keys"]]
    if sorted(keys_a) != sorted(keys_b):
        found = f"{len(set(keys_a) - set(keys_b))} only in base {sorted(set(keys_a) - set(keys_b))[:3]}, {len(set(keys_b) - set(keys_a))} only in branch {sorted(set(keys_b) - set(keys_a))[:3]}"
        if compare_keys:
            out.differences.append(f"[{label}] CONTENT KEYS differ: {found}")
        else:
            out.notes.append(f"[{label}] content keys differ and are NOT compared (--keys-may-differ: the two sides order different executors, whose identity is part of a key): {found}")
    _compare_rows(label, a, b, allowed, out)
    if a.get("lean") or b.get("lean"):
        allowed = (*allowed, *LEAN_COUNTER_PREFIXES)
    if a.get("process_counters") is not None and b.get("process_counters") is not None:
        for name in sorted(set(a["process_counters"]) | set(b["process_counters"])):
            first, second = a["process_counters"].get(name, {}), b["process_counters"].get(name, {})
            for key in sorted(set(first) | set(second)):
                if first.get(key, 0) != second.get(key, 0) and not any(key.startswith(prefix) for prefix in allowed):
                    out.differences.append(f"[{label}] process counter {name}.{key}: base {first.get(key, 0)} != branch {second.get(key, 0)}")
    for side, press in (("base", a), ("branch", b)):
        if press.get("effective_alpha_calls") and any(press["effective_alpha_calls"]):
            out.notes.append(f"[{label}] {side}: effective-alpha pairs computed natively {press['effective_alpha_calls'][0]}, in Python after a named refusal {press['effective_alpha_calls'][1]}")
    for side, press in (("base", a), ("branch", b)):
        if press.get("materialize_calls") and any(press["materialize_calls"]):
            out.notes.append(f"[{label}] {side}: W1 (exact mesh) calls computed natively {press['materialize_calls'][0]}, in Python after a named refusal {press['materialize_calls'][1]}")
    for side, press in (("base", a), ("branch", b)):
        if press.get("snapshot_check_calls") and any(press["snapshot_check_calls"]):
            out.notes.append(f"[{label}] {side}: snapshot checks (EXPORT validators, two per domain) proved natively {press['snapshot_check_calls'][0]}, run by the Python validators after a named decline {press['snapshot_check_calls'][1]}")
    for side, press in (("base", a), ("branch", b)):
        if press.get("density_calls") and any(press["density_calls"]):
            out.notes.append(f"[{label}] {side}: Density calls native {press['density_calls'][0]}, Python {press['density_calls'][1]}, named fallbacks {press['density_calls'][2]}")
    _counter_difference(a["profile_counters"], b["profile_counters"], allowed, is_volatile_run_counter, f"[{label}] run profile", out)
    by_name_a, by_name_b = a.get("profile_domain_counters_by_name"), b.get("profile_domain_counters_by_name")
    if by_name_a is not None and by_name_b is not None:
        # Per counter name, so that `--allow-counter` covers the per-domain decisions a change adds on purpose (e.g. the byte-check decision of every domain), listed and never silently ignored.
        for counter in sorted(set(by_name_a) | set(by_name_b)):
            if by_name_a.get(counter) != by_name_b.get(counter):
                if any(counter.startswith(prefix) for prefix in allowed):
                    out.notes.append(f"[{label}] domain counters named on purpose (--allow-counter) differ: {counter}")
                else:
                    out.differences.append(f"[{label}] domain counter {counter} differs")
        checked = ("profile_receipts", "profile_timing_stages")
    else:
        checked = ("profile_domain_counters", "profile_receipts", "profile_timing_stages")
    for name in checked:
        if a.get(name) != b.get(name):
            out.differences.append(f"[{label}] {name} differ")
    out.presses[label] = {"domains": a["domains"], "materialized": a["materialized"], "outcomes": a["outcomes"], "mesh": a["mesh_hash"][:12], "mesh_counts": a["mesh_counts"], "keys": len(a["content_keys"]),
                          "kind": a.get("kind", ""), "lean": bool(b.get("lean"))}


def _load_dump(path: str) -> dict:
    """The arrays a press dumped (`dump_press`): `{name: ndarray}`."""

    import numpy as np

    with np.load(path) as archive:
        return {name: archive[name] for name in archive.files}


def compare_incremental_press(label: str, a: dict, b: dict, allowed: tuple, out: Comparison, load=_load_dump) -> None:
    """One INCREMENTAL press of the branch (`b`) against the base tree's lean tick of the same width (`a`): see the module docstring."""

    rows_a = {row["patch"]: row for row in a["rows"]}
    rows_b = {row["patch"]: row for row in b["rows"]}
    if set(rows_a) != set(rows_b):
        out.differences.append(f"[{label}] DOMAIN SET: only base {sorted(set(rows_a) - set(rows_b))[:20]}, only branch {sorted(set(rows_b) - set(rows_a))[:20]}")
    common = sorted(set(rows_a) & set(rows_b))
    served = {patch for patch in common if rows_b[patch].get("served")}
    computed = [patch for patch in common if patch not in served]
    approximate = set((b.get("plan") or {}).get("approximate", ()))
    out.domains_compared += len(common)
    for name in ("domains", "materialized", "outcomes"):
        if a[name] != b[name]:
            out.differences.append(f"[{label}] run {name}: base {a[name]} != branch {b[name]}")
    for patch in sorted(served):
        if rows_a[patch]["outcome"] != rows_b[patch]["outcome"]:
            out.differences.append(f"[{label}] patch {patch} (served) outcome: base {rows_a[patch]['outcome']} != branch {rows_b[patch]['outcome']}")
    # the computed domains: the answer of a full lean tick, field by field (the hash of the view makes it bit for bit)
    _compare_rows(label, {"rows": [rows_a[p] for p in computed]}, {"rows": [rows_b[p] for p in computed]}, tuple(allowed), out)
    flips = [patch for patch in sorted(served) if rows_a[patch]["structure_digest"] != rows_b[patch]["structure_digest"]]
    if flips:
        out.notes.append(f"[{label}] FLIP: {len(flips)} served domains whose structure on this width differs from the one the tick carried (the verification finds them): {flips[:20]}")
    served_max = {"position": 0.0, "uv": 0.0}
    computed_max = {"position": 0.0, "uv": 0.0}
    mesh = {"same_layout": None, "position_max": None, "uv_max": None, "owners_equal": None, "owner_partition_equal": None}
    reordered: list = []
    if a.get("dump") and b.get("dump"):
        first, second = load(a["dump"]["path"]), load(b["dump"]["path"])
        # The order of the faces of a batch can change from width to width under the SAME structure digest (the digest sorts the rings), and the token (faces in order) then differs: the served domain keeps the order
        # of its verification base. Such a domain is the same set of faces in another order: its loops are not comparable one by one, the layout hashes differ without any geometric difference.
        tokens_a, tokens_b = a["dump"].get("domain_tokens", {}), b["dump"].get("domain_tokens", {})
        reordered = [patch for patch in sorted(served) if patch not in flips and tokens_a.get(str(patch)) != tokens_b.get(str(patch))]
        if reordered:
            out.notes.append(f"[{label}] label-only: {len(reordered)} served domains keep the face order of their verification base (the structure digest is the same): {reordered[:20]}")
        for patch in common:
            for kind, key in (("position", f"pos_{patch}"), ("uv", f"uv_{patch}")):
                if key not in first or key not in second or (kind == "uv" and patch in reordered):
                    continue
                x, y = first[key], second[key]
                if x.shape != y.shape:
                    if patch not in flips:
                        out.differences.append(f"[{label}] patch {patch} {kind}: shape {x.shape} != {y.shape} without a flip")
                    continue
                gap = float(abs(x - y).max()) if x.size else 0.0
                limit = DISPLAY_TOLERANCE if (patch in approximate and kind == "position") else SERVED_TOLERANCE
                if patch in served:
                    if patch in flips:
                        continue
                    served_max[kind] = max(served_max[kind], gap)
                    if gap > limit:
                        out.differences.append(f"[{label}] patch {patch} (served) {kind} deviates from the exact mesh by {gap:.3e} > {limit:g}")
                else:
                    computed_max[kind] = max(computed_max[kind], gap)
                    # a computed domain next to a display-approximate partner is welded against the partner's FROZEN normals: its mesh coordinates may differ from the exact weld by the display limit
                    allowed_gap = DISPLAY_TOLERANCE if (approximate and kind == "position") else 0.0
                    if gap > allowed_gap:
                        out.differences.append(f"[{label}] patch {patch} (computed) {kind} is not the exact mesh: {gap:.3e} > {allowed_gap:g}")
        for name in ("faces_hash", "seams_hash", "tokens_hash", "face_domain_hash"):
            if a["dump"].get(name) != b["dump"].get(name) and not flips and not reordered:
                out.differences.append(f"[{label}] mesh {name}: base {a['dump'].get(name)} != branch {b['dump'].get(name)} without a flip")
        mesh["same_layout"] = all(a["dump"].get(name) == b["dump"].get(name) for name in ("faces_hash", "seams_hash", "tokens_hash"))
        if mesh["same_layout"] and "mesh_positions" in first and "mesh_positions" in second and first["mesh_positions"].shape == second["mesh_positions"].shape:
            mesh["position_max"] = float(abs(first["mesh_positions"] - second["mesh_positions"]).max()) if first["mesh_positions"].size else 0.0
            mesh["uv_max"] = float(abs(first["mesh_uvs"] - second["mesh_uvs"]).max()) if first["mesh_uvs"].size else 0.0
            limit = DISPLAY_TOLERANCE if approximate else SERVED_TOLERANCE
            if mesh["position_max"] > limit:
                out.differences.append(f"[{label}] mesh positions deviate by {mesh['position_max']:.3e} (limit {limit:g})")
            if mesh["uv_max"] > SERVED_TOLERANCE:
                out.differences.append(f"[{label}] mesh UV deviate by {mesh['uv_max']:.3e} > {SERVED_TOLERANCE:g}")
            mesh["owners_equal"] = a["dump"].get("owners_hash") == b["dump"].get("owners_hash")
            mesh["owner_partition_equal"] = a["dump"].get("owner_partition_hash") == b["dump"].get("owner_partition_hash")
            if mesh["owner_partition_equal"] is False and not flips:
                out.differences.append(f"[{label}] the partition of the faces into owner groups differs without a flip")
    else:
        out.notes.append(f"[{label}] no arrays dump on one side: the positions of the served and computed domains were not compared")
    if mesh["owners_equal"] is False:
        out.notes.append(f"[{label}] label-only: the owner ordinals (cftuv_owner) of the served domains are those of the verification base (the partition into owner groups is the same)")
    out.presses[label] = {
        "domains": a["domains"], "materialized": a["materialized"], "outcomes": a["outcomes"], "mesh": "incremental", "mesh_counts": b["mesh_counts"], "keys": 0, "kind": "incr", "lean": True,
        "served": len(served), "computed": len(computed), "flips": flips, "reordered": reordered, "served_max": served_max, "computed_max": computed_max, "mesh_compare": mesh,
        "composite": (b.get("plan") or {}).get("composite"), "fallback": (b.get("plan") or {}).get("fallback", ""),
    }


def compare_runs(base: dict, branch: dict, allowed: tuple = (), compare_keys: bool = True, load=_load_dump) -> Comparison:
    """Two reports of the SAME case and worker count (`base`, `branch`): every difference of the product's answer, named."""

    out = Comparison()
    for side, report in (("base", base), ("branch", branch)):
        if report.get("failure"):
            out.differences.append(f"{side}: the run FAILED: {_short(report['failure'], 400)}")
        if report.get("worker_origins_ok") is False:
            out.differences.append(f"{side}: WORKER ORIGIN: {'; '.join(report.get('worker_origin_problems', []))[:400]}")
    if out.differences:
        return out
    if base["workers"] != branch["workers"] or base["case"] != branch["case"]:
        out.differences.append(f"not comparable: base {base['case']}/{base['workers']} vs branch {branch['case']}/{branch['workers']}")
        return out
    if list(base["presses"]) != list(branch["presses"]):
        out.differences.append(f"different presses: base {list(base['presses'])} vs branch {list(branch['presses'])}")
        return out
    for label in base["presses"]:
        press = branch["presses"][label]
        if (press.get("plan") or {}).get("mode") == REFRESH_INCREMENTAL:
            compare_incremental_press(label, base["presses"][label], press, tuple(allowed), out, load)
            continue
        _compare_press(label, base["presses"][label], branch["presses"][label], tuple(allowed), out, compare_keys)
    for key in ("selection_sha256_16", "native_build_id"):
        if base.get(key) != branch.get(key):
            out.differences.append(f"{key}: base {base.get(key)} != branch {branch.get(key)} (the runs did not start from the same inputs)")
    return out


# --------------------------------------------------------------------------
# Timing and tables (pure)
# --------------------------------------------------------------------------


def alternating_order(repetition: int) -> tuple[str, str]:
    """Repetition 1: base then branch; 2: branch then base; ... (a drift of the machine does not favour one tree)."""

    return ("base", "branch") if repetition % 2 else ("branch", "base")


def timing_summary(samples: list[float]) -> dict:
    if not samples:
        return {"n": 0}
    return {"n": len(samples), "median": statistics.median(samples), "min": min(samples), "max": max(samples), "samples": [round(value, 4) for value in samples]}


def change_percent(base: float, branch: float) -> float | None:
    return None if not base else (branch - base) / base * 100.0


def foreign_activity(before: dict, after: dict, ours: set, threshold: float = 1.0) -> tuple[float, int]:
    """`(CPU seconds a foreign blender used between two samples, how many foreign ones appeared)`; `ours` are the PIDs this tool started."""

    cpu = sum(max(0.0, after[pid] - before[pid]) for pid in after if pid in before and pid not in ours)
    appeared = sum(1 for pid in after if pid not in before and pid not in ours)
    return (cpu if cpu > threshold else 0.0), appeared


def format_table(rows: list[dict]) -> str:
    """One line per (case, workers): verdict, domains, outcomes, mesh hash, and the timing when it was taken."""

    header = f"{'case':16s} {'workers':>7s} {'verdict':>14s} {'domains':>7s} {'materialized':>12s} {'mesh hash':>12s}  timing (median s, base -> branch)"
    lines = [header, "-" * len(header)]
    for row in rows:
        timing = "; ".join(f"{label} {item['base']:.2f} -> {item['branch']:.2f} ({item['change']:+.1f}%)" for label, item in row.get("timing", {}).items()) or "-"
        lines.append(f"{row['case']:16s} {row['workers']:>7d} {row['verdict']:>14s} {row['domains']:>7d} {row['materialized']:>12d} {row['mesh']:>12s}  {timing}")
    return "\n".join(lines)


def verdict_of(comparison: Comparison) -> str:
    return "IDENTICAL" if comparison.identical else f"DIFFERENT({len(comparison.differences)})"


def summary_row(case: str, workers: int, comparison: Comparison, timing: dict | None = None) -> dict:
    first = next(iter(comparison.presses.values()), {"domains": 0, "materialized": 0, "mesh": "-"})
    return {
        "case": case, "workers": workers, "verdict": verdict_of(comparison), "domains": first["domains"], "materialized": first["materialized"], "mesh": first["mesh"],
        "timing": timing or {},
    }


# --------------------------------------------------------------------------
# Driver: preflight, one Blender per run, the report
# --------------------------------------------------------------------------


def find_blender(explicit: str = "") -> str | None:
    """`--blender`, `$BLENDER_EXE`, `blender` on PATH, then the newest `Blender Foundation\\Blender *\\blender.exe` under Program Files."""

    for candidate in (explicit, os.environ.get("BLENDER_EXE", "")):
        if candidate:
            return candidate if Path(candidate).is_file() else None
    found = shutil.which("blender")
    if found:
        return found
    roots = [os.environ.get(name) for name in ("ProgramFiles", "ProgramW6432")]
    installs = [path for root in roots if root for path in (Path(root) / "Blender Foundation").glob("Blender *\\blender.exe")]
    installs.sort(key=lambda path: tuple(int(part) for part in re.findall(r"\d+", path.parent.name)), reverse=True)  # Blender 4.10 is newer than Blender 4.5
    return str(installs[0]) if installs else None


def blender_cpu() -> dict:
    """`{pid: CPU seconds}` of every running Blender, from PowerShell (Windows) or `ps`; empty when neither answers (the guard then cannot see a foreign Blender, and the report says nothing was waited for)."""

    try:
        if os.name == "nt":
            text = subprocess.run(
                ["powershell", "-NoProfile", "-Command", "Get-Process blender -ErrorAction SilentlyContinue | ForEach-Object { \"$($_.Id) $($_.CPU)\" }"], capture_output=True, text=True, timeout=30,
            ).stdout
        else:
            text = subprocess.run(["ps", "-C", "blender", "-o", "pid=,cputimes="], capture_output=True, text=True, timeout=30).stdout
        return {int(line.split()[0]): float(line.split()[1]) for line in text.splitlines() if len(line.split()) >= 2}
    except (OSError, ValueError, subprocess.SubprocessError):
        return {}


def wait_for_idle(ours: set, seconds_limit: float = 1800.0, window: float = 5.0, log=print) -> str:
    """Waits while a FOREIGN Blender is busy (CPU used in a `window`); never stops anything. Returns a note for the report."""

    deadline = time.time() + seconds_limit
    while time.time() < deadline:
        before = blender_cpu()
        time.sleep(window)
        cpu, appeared = foreign_activity(before, blender_cpu(), ours, threshold=0.75)
        if not cpu and not appeared:
            return "idle"
        log(f"  waiting: a foreign blender is busy ({cpu:.1f} CPU s in {window:.0f} s)")
    return "TIMEOUT waiting for an idle machine"


@dataclass
class RunResult:
    tree: str
    exit_code: int | None
    seconds: float
    pid: int
    report: dict | None
    log: str
    overlapped: bool = False
    note: str = ""


def worker_command(blender: str, scene: Path, tree: Tree, case: Case, workers: int, presses: str, out: Path, options: argparse.Namespace) -> list[str]:
    command = [blender, "-b", str(scene), "--python-exit-code", "1", "--python", str(Path(__file__).resolve()), "--", "--worker", "--root", str(tree.root), "--label", tree.label,
               "--case", case.name, "--scene-kind", case.scene, "--mesh", case.mesh, "--alpha", repr(case.alpha), "--density", case.density, "--stretch", str(case.stretch),
               "--offset", repr(case.offset), "--workers", str(workers), "--presses", presses, "--step-width", repr(options.step_width), "--kernel-backend", options.kernel_backend,
               "--gc-pause", options.gc_pause, "--origin-check", options.origin_check, "--out", str(out)]
    if options.native_path:
        command += ["--native-path", options.native_path]
    if options.expect_native_build_id:
        command += ["--expect-native-build-id", options.expect_native_build_id]
    if options.prime_byte_check:
        command.append("--prime-byte-check")
    return command


def require_headless(command: list[str]) -> None:
    """Only a background Blender (`blender -b ...`) is ever started: a Blender with a window overwrites the owner's last-session recovery file when it exits normally."""

    if len(command) < 3 or command[1] not in ("-b", "--background"):
        raise ValueError(f"refusing to start a Blender that is not headless (-b): {command[:3]}")


def run_one(command: list[str], out: Path, tree_label: str, work_dir: Path, environment: dict, timeout: float, ours: set, guard: bool, log=print) -> RunResult:
    """One fresh Blender. Its PID goes to pids.log before anything else; on a timeout only THAT PID (and its own children) is stopped."""

    require_headless(command)
    log_path = out.with_suffix(".log")
    before = blender_cpu() if guard else {}
    started = time.perf_counter()
    with open(log_path, "wb") as handle:
        process = subprocess.Popen(command, stdout=handle, stderr=subprocess.STDOUT, env=environment)
        ours.add(process.pid)
        with open(work_dir / "pids.log", "a", encoding="utf-8") as pids:
            pids.write(f"{datetime.now().isoformat(timespec='seconds')} {tree_label} {out.stem} pid={process.pid}\n")
        note = ""
        try:
            process.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            note = f"TIMEOUT after {timeout:.0f} s: stopped pid {process.pid} (the one this tool started) and its own children"
            subprocess.run(["taskkill", "/PID", str(process.pid), "/T", "/F"], capture_output=True) if os.name == "nt" else process.kill()
            process.wait()
    seconds = time.perf_counter() - started
    overlapped = False
    if guard:
        cpu, appeared = foreign_activity(before, blender_cpu(), ours)
        overlapped = bool(cpu or appeared)
    report = json.loads(out.read_text(encoding="utf-8")) if out.is_file() else None
    return RunResult(tree_label, process.returncode, seconds, process.pid, report, str(log_path), overlapped, note)


def _arguments(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0], formatter_class=argparse.RawDescriptionHelpFormatter, epilog="see the module docstring for the full contract")
    parser.add_argument("--base", required=True, help="directory or git ref")
    parser.add_argument("--branch", required=True, help="directory or git ref")
    parser.add_argument("--scene", action="append", required=True, help="PATH (the testscene) or KEY=PATH")
    parser.add_argument("--expect-scene-sha", action="append", default=[], help="KEY=HEX, or HEX for the testscene")
    parser.add_argument("--case", action="append", required=True)
    parser.add_argument("--workers", default="0", help="comma list, 0 = in process, N>=2 = the pool")
    parser.add_argument("--presses", default="cold,step,step")
    parser.add_argument("--step-width", type=float, default=STEP_WIDTH)
    parser.add_argument("--native-path", default="")
    parser.add_argument("--expect-native-build-id", default="")
    parser.add_argument("--kernel-backend", default="NATIVE", choices=("NATIVE", "PYTHON"))
    parser.add_argument("--gc-pause", default="auto", choices=("auto", "on", "off"), help="the cold press (and, in a tree with GC_PAUSED_DURING_WIDTH_TICK_V1, every step, tick and confirm) under gc_paused() as the product does (auto: when the tree has it)")
    parser.add_argument("--prime-byte-check", action="store_true", help="the cold press counts as a LATER press of the session (the sampled snapshot byte check)")
    parser.add_argument("--origin-check", default="on", choices=("on", "off"))
    parser.add_argument("--env", action="append", default=[], help="KEY=VALUE for every Blender")
    parser.add_argument("--base-env", action="append", default=[], help="KEY=VALUE for the Blenders of the BASE tree only (after --env): with the same tree on both sides, an A/B of two settings of one tree")
    parser.add_argument("--branch-env", action="append", default=[], help="KEY=VALUE for the Blenders of the BRANCH tree only (after --env)")
    parser.add_argument("--allow-counter", action="append", default=[], help="a counter name prefix a change adds on purpose")
    parser.add_argument("--keys-may-differ", action="store_true", help="the content keys are listed in the notes, not compared: for sides that order different executors (the identity of the executor is part of a key)")
    parser.add_argument("--timing", action="store_true")
    parser.add_argument("--reps", type=int, default=3)
    parser.add_argument("--max-attempts", type=int, default=3)
    parser.add_argument("--timeout", type=float, default=3600.0)
    parser.add_argument("--blender", default="")
    parser.add_argument("--repository", default=".", help="the git repository the refs belong to (default: the current directory)")
    parser.add_argument("--work-dir", default="")
    parser.add_argument("--out", default="")
    return parser.parse_args(argv)


def parse_scenes(values: list[str]) -> dict[str, Path]:
    scenes = {}
    for value in values:
        key, separator, path = value.partition("=")
        if separator and re.fullmatch(r"[A-Za-z0-9_]+", key) and not Path(value).exists():
            scenes[key] = Path(path)
        else:
            scenes["testscene"] = Path(value)
    return scenes


def parse_expected_shas(values: list[str]) -> dict[str, str]:
    found = {}
    for value in values:
        key, separator, digest = value.partition("=")
        found[key if separator else "testscene"] = (digest if separator else value).lower()
    return found


def scene_digests(scenes: dict[str, Path]) -> dict[str, str]:
    return {key: sha256_file(path) for key, path in scenes.items()}


def _environment(values: list[str]) -> dict:
    environment = dict(os.environ, PYTHONDONTWRITEBYTECODE="1", PYTHONSAFEPATH="1")
    for item in values:
        key, separator, value = item.partition("=")
        if not separator:
            raise ValueError(f"--env {item!r}: KEY=VALUE")
        environment[key] = value
    for name in ("P2B_WORKER_GC",):
        environment.pop(name, None)
    return environment


def tree_environment(environment: dict, options: argparse.Namespace, label: str) -> dict:
    """The environment of the Blenders of one tree: the common one, then the `--base-env` / `--branch-env` of that tree."""

    values = getattr(options, f"{label}_env", None) or []
    if not values:
        return environment
    return _environment_over(environment, values)


def _environment_over(environment: dict, values: list[str]) -> dict:
    out = dict(environment)
    for item in values:
        key, separator, value = item.partition("=")
        if not separator:
            raise ValueError(f"--base-env/--branch-env {item!r}: KEY=VALUE")
        out[key] = value
    return out


def run_pair(case: Case, workers: int, trees: dict, scenes: dict, blender: str, options: argparse.Namespace, work_dir: Path, environment: dict, ours: set, log=print) -> tuple[dict, dict]:
    """Equality runs plus, with `--timing`, the repetitions: `({tree label: first report}, {tree label: {press: [seconds]}})`."""

    first: dict = {}
    seconds: dict = {label: {} for label in trees}
    repetitions = options.reps if options.timing else 1
    for repetition in range(1, repetitions + 1):
        for label in alternating_order(repetition):
            for attempt in range(1, options.max_attempts + 1):
                out = work_dir / "runs" / f"{case.name}_w{workers}_{label}_r{repetition}_a{attempt}.json"
                out.parent.mkdir(parents=True, exist_ok=True)
                out.unlink(missing_ok=True)
                idle = wait_for_idle(ours, log=log) if options.timing else "not waited"
                presses = parse_presses(options.presses, case.alpha, options.step_width)
                command = worker_command(blender, scenes[case.scene], trees[label], case, workers, ",".join(kind for _, kind, _ in presses), out, options)
                result = run_one(command, out, label, work_dir, tree_environment(environment, options, label), options.timeout, ours, options.timing, log)
                log(f"{datetime.now():%H:%M:%S} {case.name} w{workers} {label} rep{repetition}.{attempt} pid={result.pid} exit={result.exit_code} {result.seconds:.1f}s idle={idle} overlap={result.overlapped} {result.note}")
                if result.report is None or result.exit_code != 0 or not result.overlapped:
                    break
            if result.report is None:
                result.report = {"failure": f"no report (exit {result.exit_code}); see {result.log}", "workers": workers, "case": case.name, "presses": {}}
            elif result.exit_code != 0 and not result.report.get("failure"):
                result.report["failure"] = f"exit code {result.exit_code}; see {result.log}"
            first.setdefault(label, result.report)
            for press, data in result.report.get("presses", {}).items():
                seconds[label].setdefault(press, []).append(data["run_seconds"])
                seconds[label].setdefault("exact:" + press, []).append(data.get("exact_seconds", data["run_seconds"]))
    return first, seconds


def percentile(samples: list[float], fraction: float) -> float:
    """Linear-interpolated percentile (`fraction` 0.5 = median, 0.9 = p90) of a non-empty list."""

    ordered = sorted(samples)
    position = (len(ordered) - 1) * fraction
    low = int(position)
    high = min(low + 1, len(ordered) - 1)
    return ordered[low] + (ordered[high] - ordered[low]) * (position - low)


def _case_timing(seconds: dict) -> dict:
    out = {}
    for press in seconds.get("base", {}):
        base, branch = timing_summary(seconds["base"][press]), timing_summary(seconds.get("branch", {}).get(press, []))
        if base["n"] and branch["n"]:
            out[press] = {"base": base["median"], "branch": branch["median"], "change": change_percent(base["median"], branch["median"]), "base_all": base, "branch_all": branch}
    # Every tick of every repetition pooled: p50 / p90 of an exact refresh DURING a drag (`exact:` = the run plus the arrays of the mesh), base against branch.
    for kind in ("tick", "step"):
        names = [press for press in seconds.get("base", {}) if press.startswith("exact:" + kind)]
        pooled = {side: [value for press in names for value in seconds.get(side, {}).get(press, [])] for side in ("base", "branch")}
        if pooled["base"] and pooled["branch"]:
            row = {side: {"n": len(values), "p50": percentile(values, 0.5), "p90": percentile(values, 0.9)} for side, values in pooled.items()}
            out[f"{kind}s p50"] = {"base": row["base"]["p50"], "branch": row["branch"]["p50"], "change": change_percent(row["base"]["p50"], row["branch"]["p50"]), "n": row["base"]["n"]}
            out[f"{kind}s p90"] = {"base": row["base"]["p90"], "branch": row["branch"]["p90"], "change": change_percent(row["base"]["p90"], row["branch"]["p90"]), "n": row["base"]["n"]}
    return out


def driver_main(argv: list[str]) -> int:
    options = _arguments(argv)
    log = lambda text="": print(text, flush=True)  # noqa: E731
    try:
        cases = [parse_case(item) for item in options.case]
        workers_list = [int(item) for item in options.workers.split(",") if item.strip()]
        if any(count == 1 or count < 0 for count in workers_list):
            raise ValueError("--workers: 0 (in process) or a pool of at least 2")
        scenes = parse_scenes(options.scene)
        for case in cases:
            if case.scene not in scenes:
                raise ValueError(f"case {case.name} needs the scene {case.scene!r}: pass --scene {case.scene}=PATH")
        missing = [str(path) for path in scenes.values() if not path.is_file()]
        if missing:
            raise ValueError(f"no such scene: {missing}")
        blender = find_blender(options.blender)
        if blender is None:
            raise ValueError("no blender.exe: pass --blender, set BLENDER_EXE or put it on PATH")
        work_dir = Path(options.work_dir or Path(tempfile.gettempdir()) / "cftuv_tree_ab").resolve()
        work_dir.mkdir(parents=True, exist_ok=True)
        repository = Path(options.repository).resolve()
        trees = {"base": resolve_tree("base", options.base, work_dir, repository), "branch": resolve_tree("branch", options.branch, work_dir, repository)}
        environment = _environment(options.env)
        for case in cases:
            parse_presses(options.presses, case.alpha, options.step_width)
    except (ValueError, OSError) as error:
        print(f"TREE_AB_UNRUNNABLE {error}")
        return EXIT_UNRUNNABLE
    return _drive(options, cases, workers_list, scenes, blender, work_dir, trees, environment, log)


def _drive(options, cases, workers_list, scenes, blender, work_dir, trees, environment, log) -> int:
    before = scene_digests(scenes)
    expected = parse_expected_shas(options.expect_scene_sha)
    wrong = {key: (digest, before[key]) for key, digest in expected.items() if key in before and before[key] != digest}
    if wrong:
        print(f"TREE_AB_UNRUNNABLE the scene is not the pinned content: {wrong}")
        return EXIT_UNRUNNABLE
    log(f"blender: {blender}\nbase   {trees['base'].spec} -> {trees['base'].root} {trees['base'].commit[:10]} dirty={trees['base'].dirty}\nbranch {trees['branch'].spec} -> {trees['branch'].root} {trees['branch'].commit[:10]} dirty={trees['branch'].dirty}")
    if trees["base"].root == trees["branch"].root:
        log("NOTE: the same tree on both sides: a self-comparison (what it can show is noise of the harness)")
    ours: set = set()
    started = time.time()
    results, rows = [], []
    for case in cases:
        for workers in workers_list:
            try:
                reports, seconds = run_pair(case, workers, trees, scenes, blender, options, work_dir, environment, ours, log)
                comparison = compare_runs(reports["base"], reports["branch"], tuple(options.allow_counter), not options.keys_may_differ)
            except Exception:  # noqa: BLE001 - a harness error is a named failure of the case, not a crash of the matrix
                comparison = Comparison(differences=["HARNESS ERROR " + traceback.format_exc()[-600:]])
                reports, seconds = {}, {}
            timing = _case_timing(seconds) if options.timing else {}
            rows.append(summary_row(case.name, workers, comparison, timing))
            results.append({"case": case.name, "workers": workers, "verdict": verdict_of(comparison), "differences": comparison.differences, "notes": comparison.notes,
                            "domains_compared": comparison.domains_compared, "presses": comparison.presses, "timing": timing, "seconds": seconds,
                            "reports": {label: report for label, report in reports.items()}})
            for line in comparison.differences[:12]:
                log("  DIFFERENT " + line)
            for line in comparison.notes[:6]:
                log("  note " + line)
    after = scene_digests(scenes)
    scene_report = {key: {"before": before[key], "after": after[key], "unchanged": before[key] == after[key]} for key in before}
    document = {
        "schema": SCHEMA, "started": started, "seconds": time.time() - started, "blender": blender, "trees": {k: v.as_record() for k, v in trees.items()},
        "scenes": {key: str(path) for key, path in scenes.items()}, "scene_sha256": scene_report, "presses": options.presses, "timing": bool(options.timing),
        "environment_overrides": options.env, "tree_environment": {"base": options.base_env, "branch": options.branch_env}, "allow_counter": options.allow_counter, "results": results,
    }
    out = Path(options.out or work_dir / f"tree_ab_{datetime.now():%Y%m%d_%H%M%S}.json")
    out.write_text(json.dumps(document, ensure_ascii=False, indent=1, sort_keys=True, default=str) + "\n", encoding="utf-8")
    log("\n" + format_table(rows))
    differing = sum(1 for item in results if item["verdict"] != "IDENTICAL")
    for key, item in scene_report.items():
        log(f"scene {key}: sha256 before {item['before'][:16]} after {item['after'][:16]} -> {'unchanged' if item['unchanged'] else 'CHANGED'}")
    log(f"report: {out}\npids: {work_dir / 'pids.log'}")
    if not all(item["unchanged"] for item in scene_report.values()):
        log("TREE_AB_UNRUNNABLE a scene changed during the runs")
        return EXIT_UNRUNNABLE
    log(f"TREE_AB_{'IDENTICAL' if not differing else 'DIFFERENT'} cases={len(results)} differing={differing}")
    return EXIT_IDENTICAL if not differing else EXIT_DIFFERENT


# --------------------------------------------------------------------------
# Worker: runs INSIDE Blender (bpy is imported only here)
# --------------------------------------------------------------------------

#: Appended to the pool's bootstrap in the parent's memory: every worker writes where its code came from (modules, code identity, native build and status).
WORKER_PROBE = r'''
import atexit, threading

def _probe_dump(tag):
    try:
        info = {"pid": os.getpid(), "tag": tag, "executable": sys.executable, "python": sys.version.split()[0]}
        info["host_file"] = getattr(sys.modules.get(spec["package"]), "__file__", None)
        for name in ("cftuv_envelope", "cftuv_native", "sympy", "mpmath"):
            module = sys.modules.get(name)
            if module is None:
                try:
                    module = importlib.import_module(name)
                except Exception as exc:
                    info[name + "_error"] = repr(exc)
            info[name + "_file"] = None if module is None else getattr(module, "__file__", None)
        info["sympy_version"] = getattr(sys.modules.get("sympy"), "__version__", None)
        info["mpmath_version"] = getattr(sys.modules.get("mpmath"), "__version__", None)
        try:
            info["code_identity"] = list(importlib.import_module(spec["package"] + ".envelope_content_key").code_identity())
        except Exception as exc:
            info["code_identity_error"] = repr(exc)
        try:
            native = sys.modules.get("cftuv_native") or importlib.import_module("cftuv_native")
            info["native_build_id"] = native.native_build_id()
        except Exception as exc:
            info["native_error"] = repr(exc)
        try:
            from cftuv_envelope.backend import native_status as _status
            info["host_native_status"] = {k: str(v) for k, v in _status().as_record().items()}
        except Exception as exc:
            info["host_native_status_error"] = repr(exc)
        with open(os.path.join(DUMP_DIR, "%d_%s.json" % (os.getpid(), tag)), "w", encoding="utf-8") as handle:
            json.dump(info, handle, indent=1, default=str)
    except Exception as exc:
        try:
            with open(os.path.join(DUMP_DIR, "%d_%s_error.txt" % (os.getpid(), tag)), "w") as handle:
                handle.write(repr(exc))
        except Exception:
            pass

atexit.register(lambda: _probe_dump("exit"))
_timer = threading.Timer(5.0, lambda: _probe_dump("t5"))
_timer.daemon = True
_timer.start()
'''
WORKER_MAIN_CALL = 'importlib.import_module(spec["package"] + ".envelope_domain_pool").worker_main()'


def install_worker_probe(dump_dir: str) -> None:
    import cftuv.envelope_domain_pool as pool_module

    Path(dump_dir).mkdir(parents=True, exist_ok=True)
    for old in Path(dump_dir).glob("*"):
        old.unlink()
    bootstrap = pool_module._BOOTSTRAP
    assert WORKER_MAIN_CALL in bootstrap, "the worker bootstrap changed: update WORKER_PROBE (or run with --origin-check off)"
    pool_module._BOOTSTRAP = bootstrap.replace(WORKER_MAIN_CALL, f"DUMP_DIR = {dump_dir!r}\n" + WORKER_PROBE + "\n" + WORKER_MAIN_CALL)


def parent_identity() -> dict:
    """Where the parent's own code came from. The extension is looked up in `sys.modules`: only the kernel's dispatcher imports it (tests/test_architecture.py), `native_status` has already done so."""

    import cftuv
    import cftuv_envelope
    import mpmath
    import sympy
    from cftuv.envelope_content_key import code_identity
    from cftuv_envelope.backend import native_status

    extension = sys.modules["cftuv_native"]
    return {
        "cftuv_file": cftuv.__file__, "cftuv_envelope_file": cftuv_envelope.__file__, "cftuv_native_file": extension.__file__, "sympy_file": sympy.__file__, "mpmath_file": mpmath.__file__,
        "sympy_version": sympy.__version__, "mpmath_version": mpmath.__version__, "code_identity": list(code_identity()), "native_build_id": native_status().build_id, "python": sys.version.split()[0],
    }


def collect_worker_origins(dump_dir: str, parent: dict, expected_workers: int, wait: float = 10.0) -> tuple[bool, list, list]:
    """The workers' dumps against the parent's identity: the same files, the same code identity, the same native build, the same sympy; every native stage `available`."""

    deadline = time.time() + wait
    while time.time() < deadline and len(list(Path(dump_dir).glob("*_exit.json"))) < expected_workers:
        time.sleep(0.25)
    records = [json.loads(path.read_text(encoding="utf-8")) for path in sorted(Path(dump_dir).glob("*.json"))]
    problems = []
    if len({record["pid"] for record in records}) < expected_workers:
        problems.append(f"only {len({record['pid'] for record in records})} worker pids reported, expected {expected_workers}")
    norm = lambda path: None if path is None else os.path.normcase(os.path.abspath(path))  # noqa: E731
    for record in records:
        tag = f"pid {record['pid']} {record['tag']}"
        for worker_key, parent_key in (("host_file", "cftuv_file"), ("cftuv_envelope_file", "cftuv_envelope_file"), ("cftuv_native_file", "cftuv_native_file"), ("sympy_file", "sympy_file"), ("mpmath_file", "mpmath_file")):
            if norm(record.get(worker_key)) != norm(parent.get(parent_key)):
                problems.append(f"{tag}: {worker_key} {record.get(worker_key)!r} != parent {parent.get(parent_key)!r}")
        for key in ("code_identity", "native_build_id", "sympy_version", "mpmath_version"):
            if record.get(key) != parent.get(key):
                problems.append(f"{tag}: {key} {record.get(key)} != parent {parent.get(key)}")
        for stage in ("coverage", "clip", "skeleton", "snap_embedding"):
            if (record.get("host_native_status") or {}).get(stage) != "available":
                problems.append(f"{tag}: native stage {stage} is {(record.get('host_native_status') or {}).get(stage)!r}")
    return (not problems), problems, records


def _worker_arguments(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--worker", action="store_true")
    parser.add_argument("--root", required=True)
    parser.add_argument("--label", default="")
    parser.add_argument("--case", default="")
    parser.add_argument("--scene-kind", default="testscene")
    parser.add_argument("--mesh", required=True)
    parser.add_argument("--alpha", type=float, required=True)
    parser.add_argument("--density", default="2")
    parser.add_argument("--stretch", type=int, default=42)
    parser.add_argument("--offset", type=float, default=-1.0)
    parser.add_argument("--workers", type=int, default=0)
    parser.add_argument("--presses", default="cold,step,step")
    parser.add_argument("--step-width", type=float, default=STEP_WIDTH)
    parser.add_argument("--native-path", default="")
    parser.add_argument("--expect-native-build-id", default="")
    parser.add_argument("--kernel-backend", default="NATIVE")
    parser.add_argument("--gc-pause", default="auto")
    parser.add_argument("--prime-byte-check", action="store_true")
    parser.add_argument("--origin-check", default="on")
    parser.add_argument("--out", required=True)
    return parser.parse_args(argv)


def load_tree(root: Path, native_path: str) -> None:
    """Replaces the add-on the session may have registered by the tree's own, with the native directory (when named) AHEAD of everything and the origin of each package asserted."""

    installed = sys.modules.get("cftuv")
    if installed is not None:
        try:
            installed.unregister()
        except Exception as exc:  # noqa: BLE001 - a note about the environment, not a reason to stop
            print("installed add-on unregister failed:", type(exc).__name__, exc)
    for name in tuple(sys.modules):
        if name in {"cftuv", "cftuv_envelope", "cftuv_native"} or name.startswith(("cftuv.", "cftuv_envelope.", "cftuv_native.")):
            del sys.modules[name]
    named = [native_path] if native_path else []
    tree = [str(root), str(root / "kernel" / "src")]
    sys.path[:] = [*named, *tree, *(item for item in sys.path if item not in (*named, *tree))]
    import cftuv
    import cftuv_envelope

    assert Path(cftuv.__file__).resolve().parent == (root / "cftuv").resolve(), f"cftuv imported from {cftuv.__file__}, not from {root}"
    assert Path(cftuv_envelope.__file__).resolve().parent == (root / "kernel" / "src" / "cftuv_envelope").resolve(), f"cftuv_envelope imported from {cftuv_envelope.__file__}"
    cftuv.register()


def verify_native(native_path: str, expect_prefix: str) -> dict:
    """The extension the tree's dispatcher imported: from the named directory, the expected build, every stage `available` (a silent fallback to Python would make the A/B compare Python with Python)."""

    from cftuv_envelope.backend import native_status

    status = native_status()  # the dispatcher imports the extension (the only importer); the module is then in sys.modules
    host = status.as_record()
    module = sys.modules.get("cftuv_native")
    assert module is not None, f"the extension does not import: {host}"
    here = Path(module.__file__).resolve().parent
    if native_path:
        assert here == (Path(native_path) / "cftuv_native").resolve(), f"cftuv_native imported from {here}, not from {native_path}"
    build = str(status.build_id)
    if expect_prefix:
        assert build.startswith(expect_prefix), f"unexpected native build id {build}"
    for stage in ("coverage", "clip", "skeleton", "snap_embedding"):
        assert host[stage] == "available", f"native stage {stage} is {host[stage]}"
    if "materialize_w1" in host and os.environ.get("CFTUV_MATERIALIZE_W1_BACKEND", "").strip().upper() != "PYTHON" and "CFTUV_W1_RECORD_DIR" not in os.environ:
        # a tree that has the W1 stage orders it natively by default (MATERIALIZE_W1_NATIVE_DEFAULT_V1) unless the environment isolates it or the corpus recorder runs: the extension must have the operation,
        # or the A/B compares Python with Python (the tree that has no such stage is the base side of an A/B and is not asked)
        assert host.get("materialize_w1") == "available", f"native stage materialize_w1 is {host.get('materialize_w1')}"
    if "snapshot_check" in host and os.environ.get("CFTUV_SNAPSHOT_CHECK_BACKEND", "").strip().upper() != "PYTHON" and SNAPSHOT_RECORD_ENVIRONMENT not in os.environ:
        # a tree that has the EXPORT stage orders it natively by default (SNAPSHOT_CHECK_NATIVE_DEFAULT_V1) unless the environment isolates it or the corpus recorder runs: the extension must have the operation
        assert host.get("snapshot_check") == "available", f"native stage snapshot_check is {host.get('snapshot_check')}"
    return {"module": str(module.__file__), "build_id": build, "host_native_status": {key: str(value) for key, value in host.items()}}


def select_edges(obj, selection: str) -> list:
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
    if selection == "seam":
        chosen = []
        for edge in bm.edges:
            edge.select_set(bool(edge.seam))
            if edge.seam:
                chosen.append(edge.index)
        bmesh.update_edit_mesh(obj.data)
        return chosen
    return [edge.index for edge in bm.edges if edge.select]


def _hash(value) -> str:
    return hashlib.sha256(repr(value).encode("utf-8")).hexdigest()[:16]


def view_hash(item) -> str:
    """The hash of the view of a materialized domain (positions, normals, refs, faces, UV, owners, seams): two results with the same hash lay down the same mesh. Empty for a refusal. A tree without a view hashes nothing."""

    if not item.is_materialized:
        return ""
    try:
        from cftuv.envelope_production_view import build_domain_view
    except ImportError:
        return ""
    view = getattr(item, "view", None) or build_domain_view(item)
    if view.vertices is None:
        return ""
    return _hash((view.vertices.positions, view.vertices.normals, view.vertices.refs, view.faces, view.uvs, view.owners, view.seams))


def domain_rows(run) -> list:
    rows = []
    for item in run.results:
        lean = bool(getattr(item, "lean", False))  # a WIDTH_REFRESH_LEAN_V1 result carries the view, not the batch: its digests are not computed and reading them is an error
        batch = None if lean else item.batch
        counters = [(str(name), value) for name, value in item.counters]
        record = getattr(item, "backend_record", None)
        rows.append({
            "patch": int(item.patch_id), "domain_id": str(item.domain_id), "outcome": str(item.outcome), "detail": str(item.detail), "content_digest": "" if lean else item.content_digest,
            "semantic_digest": "" if batch is None else batch.semantic_digest.value, "counters_all": dict(counters), "diagnostics": list(item.diagnostics), "normal": item.normal,
            "offset_normals_digest": item.offset_normals_digest, "vertex_normals_hash": "" if lean else _hash(item.vertex_normals), "decal_topology_law": item.decal_topology_law,
            "chart_orientation": item.chart_orientation, "step_path": getattr(item, "step_path", ABSENT), "placement": str(item.placement),
            "structure_digest": item.structure_digest, "clip_memo": getattr(item, "clip_memo", ABSENT), "ran": None if record is None else record.ran, "lean": lean,
            "served": getattr(item, "placement", "") == PLACEMENT_REBASED, "view_hash": view_hash(item),
            "effective_alpha_ran": None if record is None else (getattr(record, "effective_alpha_ran", None) or None),
            "effective_alpha_calls": None if record is None else [getattr(record, "effective_alpha_native_calls", 0), getattr(record, "effective_alpha_python_calls", 0)],
            "materialize_ran": None if record is None else (getattr(record, "materialize_ran", None) or None),
            "materialize_calls": None if record is None else [getattr(record, "materialize_native_calls", 0), getattr(record, "materialize_python_calls", 0)],
            "snapshot_check_ran": None if record is None else (getattr(record, "snapshot_check_ran", None) or None),
            "snapshot_check_calls": None if record is None else [getattr(record, "snapshot_check_native_calls", 0), getattr(record, "snapshot_check_python_calls", 0)],
            "density_requested": None if record is None else getattr(record, "density_requested", None),
            "density_calls": None if record is None else [getattr(record, "density_native_calls", 0), getattr(record, "density_python_calls", 0), len(getattr(record, "density_fallbacks", ()))],
        })
    return sorted(rows, key=lambda row: row["patch"])


def mesh_digest(run, mesh_module, offset: float) -> tuple[str, dict]:
    arrays = mesh_module.build_mesh_arrays(run.results, float(offset))
    record = {
        "positions": [[float(v).hex() for v in p] for p in arrays.positions], "faces": [list(f) for f in arrays.faces], "uvs": [[float(v).hex() for v in uv] for uv in arrays.uvs],
        "face_domain": list(arrays.face_domain), "face_owner": list(arrays.face_owner), "seams": [list(e) for e in arrays.seam_edges],
    }
    return hashlib.sha256(json.dumps(record, sort_keys=True).encode()).hexdigest(), {"vertices": len(arrays.positions), "faces": len(arrays.faces), "seams": len(arrays.seam_edges)}


def process_counters() -> dict:
    """The counters of the kernel's symbolic backend in this process (a tree that lacks them has none)."""

    try:
        from cftuv_envelope.reference import planar_types, symbolic_backend
    except ImportError:
        return {}
    return {"backend_counts": dict(symbolic_backend.BACKEND_COUNTS), "symbolic_fallback": dict(planar_types.SYMBOLIC_FALLBACK_COUNTS)}


def counters_delta(before: dict, after: dict) -> dict:
    """What a press added to the process counters, per dict, in key order of first appearance sorted (a dict is compared whole)."""

    return {
        name: {key: after[name].get(key, 0) - before.get(name, {}).get(key, 0) for key in sorted(after[name]) if after[name].get(key, 0) != before.get(name, {}).get(key, 0)}
        for name in after
    }


def press_report(run, mesh_module, offset: float, seconds: float, keys: list, counters: dict | None = None, *, kind: str = "", lean: bool = False, arrays_seconds: float = 0.0) -> dict:
    rows = domain_rows(run)
    mesh_hash, mesh_counts = mesh_digest(run, mesh_module, offset)
    outcomes: dict = {}
    for row in rows:
        outcomes[row["outcome"]] = outcomes.get(row["outcome"], 0) + 1
    profile = run.profile
    alpha_calls = [sum(row["effective_alpha_calls"][index] for row in rows if row.get("effective_alpha_calls")) for index in (0, 1)]
    materialize_calls = [sum(row["materialize_calls"][index] for row in rows if row.get("materialize_calls")) for index in (0, 1)]
    snapshot_check_calls = [sum(row["snapshot_check_calls"][index] for row in rows if row.get("snapshot_check_calls")) for index in (0, 1)]
    density_calls = [sum(row["density_calls"][index] for row in rows if row.get("density_calls")) for index in (0, 1, 2)]
    return {
        "run_seconds": seconds, "arrays_seconds": arrays_seconds, "exact_seconds": seconds + arrays_seconds, "kind": kind, "lean": lean, "rows": rows, "mesh_hash": mesh_hash, "mesh_counts": mesh_counts, "outcomes": outcomes, "materialized": outcomes.get("MATERIALIZED", 0), "domains": len(rows),
        "process_counters": counters, "effective_alpha_calls": alpha_calls, "materialize_calls": materialize_calls, "snapshot_check_calls": snapshot_check_calls, "density_calls": density_calls,
        "cold": bool(run.cold), "content_keys": sorted(keys), "profile_counters": {item.name: item.value for item in profile.counters if item.patch_domain_id is None},
        "profile_domain_counters": _hash(sorted((item.name, item.patch_domain_id, item.value) for item in profile.counters if item.patch_domain_id is not None)),
        "profile_domain_counters_by_name": {
            name: _hash(sorted((item.patch_domain_id, item.value) for item in profile.counters if item.patch_domain_id is not None and item.name == name))
            for name in sorted({item.name for item in profile.counters if item.patch_domain_id is not None})
        },
        "profile_receipts": _hash(sorted((str(item.patch_domain_id), str(item.stage)) for item in profile.receipts)),
        "profile_timing_stages": sorted({item.stage for item in profile.timings}),
        "stage_seconds": {name: round(value, 6) for name, value in sorted(profile.stage_totals.items())},  # timing, not an answer: the seconds of every stage the profile timed (EFFECTIVE_ALPHA among them)
    }


def dump_press(sample, path: Path) -> dict:
    """The arrays of a press for `compare_incremental_press`, from the mesh SAMPLE (the same object in both trees): the mesh positions and UV of every domain at its own vertices and loops, the whole mesh arrays,
    and the hashes of the layout (faces, seams, domain tokens) and of the owner ordinals and their partition."""

    import numpy as np

    arrays_out = {}
    for domain in sample.domains:
        arrays_out[f"pos_{domain.patch_id}"] = np.ascontiguousarray(sample.positions[domain.vertices])
        arrays_out[f"uv_{domain.patch_id}"] = np.ascontiguousarray(sample.uvs[domain.loop_start : domain.loop_start + domain.loop_count])
    arrays_out["mesh_positions"], arrays_out["mesh_uvs"] = np.asarray(sample.positions), np.asarray(sample.uvs)
    np.savez(path, **arrays_out)
    image = sample.image
    partition = {}
    canonical = []
    for patch, owner in zip(image.face_domain.tolist(), image.face_owner.tolist()):
        canonical.append(partition.setdefault((patch, owner), len(partition)))
    return {
        "path": str(path), "faces_hash": _hash((image.face_starts.tolist(), image.face_vertices.tolist())), "seams_hash": _hash(image.seams.tolist()), "tokens_hash": _hash([item.token for item in sample.domains]),
        "owners_hash": _hash(image.face_owner.tolist()), "owner_partition_hash": _hash(canonical), "face_domain_hash": _hash(image.face_domain.tolist()),
        "domain_tokens": {str(item.patch_id): item.token for item in sample.domains},
    }


def complete_rows(item: dict, sample, plan) -> None:
    """The rows of an incremental press name the domains the tick COMPUTED; the served ones are in the sample. Rows for them (flagged `served`, the structure digest of the verification base) make the press a whole."""

    have = {row["patch"] for row in item["rows"]}
    for domain in sample.domains:
        if domain.patch_id in plan.served and domain.patch_id not in have:
            item["rows"].append({"patch": domain.patch_id, "domain_id": domain.domain_id, "outcome": "MATERIALIZED", "detail": "", "structure_digest": domain.structure, "served": True, "counters_all": {}, "lean": True})
    item["rows"].sort(key=lambda row: row["patch"])
    outcomes: dict = {}
    for row in item["rows"]:
        outcomes[row["outcome"]] = outcomes.get(row["outcome"], 0) + 1
    item["domains"], item["outcomes"], item["materialized"] = len(item["rows"]), outcomes, outcomes.get("MATERIALIZED", 0)


def track_live_state(controller, preview, run, arrays, kind: str, alpha: float, plan, lean: bool, offset: float):
    """The sample, the model and the verification record the product keeps after each refresh, so that an `incr` press has what it needs (a tree with `plan_tick` only). The live run (`None` after the cold press)."""

    key = ("tree-ab",)
    if kind == "cold":
        preview.note_button_display(controller, preview.build_sample(run.results, arrays, key, alpha))
        return None
    preview.ensure_model(controller)
    live = preview.finish_live_run(
        run, alpha=alpha, offset=offset, key=key, displayed=controller.width_displayed, previous=controller.width_model, trust=controller.width_trust, lean=lean, plan=plan,
    )
    preview.note_exact_display(controller, live)
    return live


def record_content_keys(key_log: list):
    """Wraps `domain_content_key` (patch -> key, every call) and pins the code identity; returns the real `code_identity` for the origin check."""

    import cftuv.envelope_content_key as content_key_module

    original = content_key_module.domain_content_key

    def recording(export, *args, **kwargs):
        key = original(export, *args, **kwargs)
        (patch,) = tuple(export.bundle.patch_graph.nodes)
        key_log.append((int(patch), key))
        return key

    real = content_key_module.code_identity
    content_key_module.domain_content_key = recording
    content_key_module.code_identity = lambda: PINNED_IDENTITY
    return content_key_module, real


def _open_case(args):
    import bpy

    if args.scene_kind == "cover008":
        try:
            bpy.context.window.scene = bpy.data.scenes["GEOMETRY"]
        except Exception:  # noqa: BLE001
            bpy.context.window_manager.windows[0].scene = bpy.data.scenes["GEOMETRY"]
    scene = bpy.context.scene
    return scene, bpy.data.objects[args.mesh], scene.hotspotuv_settings, scene.hotspotuv_decal_mesh


def run_presses(args, report: dict, state: dict) -> None:
    """The presses of the case on ONE controller, as a session takes them."""

    import bmesh

    import cftuv.envelope_production_export as export_module
    import cftuv.envelope_production_mesh as mesh_module
    from cftuv.analysis import build_analysis_bundle
    from cftuv.analysis_surface import source_revision_from_bmesh
    from cftuv.envelope_kernel_backend import kernel_backend_of
    from cftuv.envelope_request_policy import envelope_dissolve_uv_slide, envelope_stretch_budget

    obj, settings, mesh_settings, controller = state["obj"], state["settings"], state["mesh_settings"], state["controller"]
    selected = select_edges(obj, "stored" if args.scene_kind == "cover008" else "seam")
    report["selected_edges"] = len(selected)
    report["selection_sha256_16"] = hashlib.sha256(json.dumps(sorted(selected)).encode()).hexdigest()[:16]
    source_bm = bmesh.from_edit_mesh(obj.data)
    source_bm.faces.ensure_lookup_table()
    face_indices = tuple(face.index for face in source_bm.faces)
    revision = source_revision_from_bmesh(source_bm, obj, face_indices)
    key, data_key = int(obj.as_pointer()), int(obj.data.as_pointer())
    bundle = controller.get_analysis_bundle(key, data_key, revision, lambda: build_analysis_bundle(source_bm, face_indices, obj))
    try:
        from cftuv.envelope_gc_pause import gc_paused
    except ImportError:
        gc_paused = None
    report["gc_pause_available"] = gc_paused is not None
    # GC_PAUSED_DURING_WIDTH_TICK_V1: the live path computes (the run AND the arrays of the thread's tail) under the pause; a tree without the constant takes only the cold press under it.
    width_pause = gc_paused is not None and hasattr(sys.modules["cftuv.envelope_gc_pause"], "GC_PAUSE_WIDTH_OUTCOME")
    report["gc_pause_width_tick"] = width_pause
    from contextlib import nullcontext

    import inspect

    lean_supported = "lean" in inspect.signature(export_module.run_production).parameters  # WIDTH_REFRESH_LEAN_V1; an older tree runs a tick the full way
    report["lean_supported"] = lean_supported
    try:
        import cftuv.envelope_width_mesh_preview as live_preview
    except ImportError:
        live_preview = None
    incremental_supported = lean_supported and live_preview is not None and hasattr(live_preview, "plan_tick")  # WIDTH_REFRESH_INCREMENTAL_V1; an older tree runs `incr` as a lean tick
    report["incremental_supported"] = incremental_supported
    if incremental_supported:
        # The tool's own override of the small-selection rule (`--env CFTUV_AB_MIN_DOMAINS=0`): a five-domain scene takes the incremental path only when it is asked to.
        import cftuv.envelope_width_incremental as policy_module

        if os.environ.get("CFTUV_AB_MIN_DOMAINS") is not None and hasattr(policy_module, "INCREMENTAL_MIN_DOMAINS"):
            policy_module.INCREMENTAL_MIN_DOMAINS = int(os.environ["CFTUV_AB_MIN_DOMAINS"])
        report["incremental_policy"] = {name: getattr(policy_module, name, None) for name in ("INCREMENTAL_VERIFY_AFTER_TICKS", "INCREMENTAL_VERIFY_AFTER_DISTANCE", "INCREMENTAL_MIN_DOMAINS")}
    report["presses"] = {}
    for label, kind in zip(state["labels"], state["kinds"]):
        alpha = state["alphas"][label]
        before = len(state["keys"])
        paused = gc_paused is not None and args.gc_pause != "off" and (kind == "cold" or (width_pause and kind in ("step", "tick", "incr", "confirm")))
        # GC_FROZEN_AFTER_BUTTON_V1: the button's pause freezes what the press accumulated (a tree that has the parameter); ticks and steps do not.
        pause = gc_paused(**({"freeze": True} if kind == "cold" and "freeze" in inspect.signature(gc_paused).parameters else {})) if paused else nullcontext()
        lean = kind in ("tick", "incr") and lean_supported
        counters_before = process_counters()
        plan = None
        approximate = ()
        started = time.perf_counter()
        with pause:
            if kind == "incr" and incremental_supported:
                live_preview.ensure_model(controller)
                approximate = tuple(row[0] for row in getattr(controller.width_model, "rows", ()) if row[2] == "PREVIEW_DEVELOPABLE_FROZEN_NORMAL_V1")
                plan = live_preview.plan_tick(
                    export_module.REFRESH_INCREMENTAL, controller.width_displayed, controller.width_model, controller.width_incremental, alpha,
                )
            run = export_module.run_production(
                controller, bundle, frozenset(selected), alpha, source_object_key=key, source_data_key=data_key, density=args.density,
                developable_stretch_budget=envelope_stretch_budget(args.stretch), silhouette_uv_slide=envelope_dissolve_uv_slide(settings.envelope_debug_dissolve_uv_tolerance),
                workers=args.workers if args.workers >= 2 else 0, kernel_backend=kernel_backend_of(mesh_settings), **({"lean": True} if lean else {}),
                **({"skipped": plan.served} if plan is not None and plan.incremental else {}),
            )
        seconds = time.perf_counter() - started
        counters = counters_delta(counters_before, process_counters()) if args.workers < 2 else None  # a pool worker keeps its own counters
        # The arrays the live thread builds right after the run (lean arrays for a lean tick), so `exact_seconds` is the time to the arrays of the exact mesh.
        arrays_started = time.perf_counter()
        with (gc_paused() if (paused and kind != "cold") else nullcontext()):
            arrays = mesh_module.build_mesh_arrays(run.results, float(mesh_settings.offset), **({"reports": False} if lean else {}))
        arrays_seconds = time.perf_counter() - arrays_started
        report["presses"][label] = press_report(run, mesh_module, mesh_settings.offset, seconds, state["keys"][before:], counters, kind=kind, lean=lean, arrays_seconds=arrays_seconds)
        item = report["presses"][label]
        live_run = track_live_state(controller, live_preview, run, arrays, kind, alpha, plan, lean, float(mesh_settings.offset)) if incremental_supported else None
        sample = live_run.sample if live_run is not None else (live_preview.build_sample(run.results, arrays, ("tree-ab",), alpha) if live_preview is not None and kind in ("tick", "incr") else None)
        if kind in ("tick", "incr") and sample is not None:
            item["dump"] = dump_press(sample, Path(args.out).with_suffix("").parent / f"{Path(args.out).stem}_{label}.npz")
        if plan is not None:
            item["plan"] = {
                "mode": plan.mode, "reason": plan.reason, "served": len(plan.served), "held": {str(patch): name for patch, name in plan.held.items()}, "domains": plan.domains, "seconds": plan.seconds,
                "composite": plan.composite, "fallback": plan.fallback, "approximate": list(approximate),
            }
            if plan.incremental and live_run is not None:
                complete_rows(item, live_run.sample, plan)
                item["plan"].update(composite=live_run.plan.composite, fallback=live_run.plan.fallback)
        print(f"TREEAB press={label} kind={kind} lean={lean} alpha={alpha} domains={item['domains']} materialized={item['materialized']} keys={len(item['content_keys'])} run={seconds:.2f}s arrays={arrays_seconds:.3f}s", flush=True)


W1_RECORD_ENVIRONMENT = "CFTUV_W1_RECORD_DIR"


def install_w1_recorder():
    """The corpus of W1 of the MATERIALIZE chain (`mesh.exact_mesh`, `mesh.canonical_triangles`) recorded from THIS run: set `CFTUV_W1_RECORD_DIR` to a directory and the two entries are wrapped by the
    corpus recorder (`tools/native_corpus.py`). The stage follows the master switch (NATIVE by default; MATERIALIZE_W1_NATIVE_DEFAULT_V1), and a native stage never calls the oracle the recorder wraps, so the recorder
    orders the stage to the reference (`CFTUV_MATERIALIZE_W1_BACKEND=PYTHON`, read at every derivation of the orders). In-process runs only (`--workers 0`): a pool worker is another process."""

    directory = os.environ.get(W1_RECORD_ENVIRONMENT, "").strip()
    if not directory:
        return None
    os.environ["CFTUV_MATERIALIZE_W1_BACKEND"] = "PYTHON"
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    import cftuv_envelope.materialize.domain as domain_module
    import native_corpus as nc

    recorder = nc.Recorder(Path(directory), nc.run_description({"recorded_by": "blender_tree_ab"}), operations=nc.W1_OPERATIONS)
    domain_module.exact_mesh = recorder.wrap(nc.OP_MESH, nc.ORACLE[nc.OP_MESH])
    domain_module.canonical_triangles = recorder.wrap(nc.OP_CANONICAL, nc.ORACLE[nc.OP_CANONICAL])
    return recorder


SNAPSHOT_RECORD_ENVIRONMENT = "CFTUV_SNAPSHOT_RECORD_DIR"


def install_snapshot_recorder(args):
    """The corpus of the EXPORT stage of the pool worker (`task_inputs`) recorded from THIS run: set `CFTUV_SNAPSHOT_RECORD_DIR` to a directory and every task with an export input that the parent sends
    to the pool is written (`tools/native_snapshot_check_corpus.py`, one file per case). The recorder runs with the stage `snapshot_check` ordered to the reference for the whole process tree (the
    environment is inherited by the pool workers): the corpus is what the PYTHON validators saw. Pool runs only (`--workers >= 2`): the task with an export input exists only for a worker."""

    directory = os.environ.get(SNAPSHOT_RECORD_ENVIRONMENT, "").strip()
    if not directory or int(args.workers) < 2:
        return None
    os.environ["CFTUV_SNAPSHOT_CHECK_BACKEND"] = "PYTHON"
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    import native_snapshot_check_corpus as corpus

    recorder = corpus.Recorder(Path(directory), args.case or args.mesh)
    recorder.install()
    return recorder


def prepare_pool(args, report: dict, state: dict) -> None:
    from cftuv.envelope_domain_pool import get_domain_pool
    from cftuv.envelope_worker_python import read_worker_python

    interpreter = read_worker_python()
    report["worker_python_pref"] = interpreter
    if args.origin_check == "on":
        state["dump_dir"] = str(Path(args.out).with_suffix("")) + "_workers"
        install_worker_probe(state["dump_dir"])
    pool = get_domain_pool(args.workers, interpreter)
    try:
        from cftuv.envelope_pool_prewarm import prewarm_pool

        report["pool_prewarm_seconds"] = prewarm_pool(args.workers, interpreter)
    except ImportError:
        report["pool_prewarm_seconds"] = pool.warm()
    report["pool_interpreter"] = str(pool.interpreter)


def worker_main(argv: list[str]) -> int:
    import bpy

    sys.dont_write_bytecode = True
    args = _worker_arguments(argv)
    root = Path(args.root).resolve()
    load_tree(root, args.native_path)
    import mpmath  # noqa: F401
    import sympy  # noqa: F401
    import cftuv_envelope.materialize.step  # noqa: F401
    import cftuv_envelope.wavefront  # noqa: F401
    from cftuv.envelope_debug_session import WINDOW_MANAGER_SESSION_ATTRIBUTE, EnvelopeDebugSessionController
    from cftuv.envelope_kernel_backend import kernel_backend_of

    presses = parse_presses(args.presses, args.alpha, args.step_width)
    report: dict = {"schema": SCHEMA, "label": args.label, "case": args.case, "root": str(root), "mesh": args.mesh, "scene_kind": args.scene_kind, "workers": args.workers, "alpha": args.alpha,
                    "blender": bpy.app.version_string, "blend": bpy.data.filepath, "started_at": time.time(), "pinned_identity": list(PINNED_IDENTITY), "failure": None}
    failure = None
    state: dict = {"keys": [], "labels": [p[0] for p in presses], "kinds": [p[1] for p in presses], "alphas": {p[0]: p[2] for p in presses}}
    try:
        report["native"] = verify_native(args.native_path, args.expect_native_build_id)
        report["native_build_id"] = report["native"]["build_id"]
        scene, obj, settings, mesh_settings = _open_case(args)
        mesh_settings.kernel_backend = args.kernel_backend
        assert kernel_backend_of(mesh_settings) == args.kernel_backend
        if args.offset >= 0:
            mesh_settings.offset = args.offset
        controller = getattr(bpy.context.window_manager, WINDOW_MANAGER_SESSION_ATTRIBUTE, None)
        if not isinstance(controller, EnvelopeDebugSessionController):
            controller = EnvelopeDebugSessionController()
            setattr(bpy.context.window_manager, WINDOW_MANAGER_SESSION_ATTRIBUTE, controller)
        state.update(obj=obj, settings=settings, mesh_settings=mesh_settings, controller=controller)
        if args.workers >= 2:
            prepare_pool(args, report, state)
        if args.prime_byte_check:
            report["primed_identity"] = _prime_byte_check(kernel_backend_of(mesh_settings))
        state["content_key_module"], state["real_identity"] = record_content_keys(state["keys"])
        state["w1_recorder"] = install_w1_recorder()
        state["snapshot_recorder"] = install_snapshot_recorder(args)
        run_presses(args, report, state)
    except Exception:  # noqa: BLE001 - the traceback is the report
        failure = traceback.format_exc()
        print(failure)
    return _finish(args, report, state, failure)


def _prime_byte_check(backend) -> list | None:
    try:
        import cftuv.envelope_snapshot_byte_check as byte_check

        identity = byte_check.execution_identity_of(backend)
        byte_check.mark_identity_checked(identity)
        return list(identity)
    except ImportError:
        return None


def _finish(args, report: dict, state: dict, failure) -> int:
    if "real_identity" in state:  # the origin check compares the REAL identity of the parent with the workers'
        state["content_key_module"].code_identity = state["real_identity"]
    try:
        from cftuv.envelope_domain_pool import shutdown_domain_pool

        shutdown_domain_pool()
    except Exception as exc:  # noqa: BLE001 - the cleanup does not change the verdict
        print("pool shutdown:", type(exc).__name__, exc)
    if "dump_dir" in state and failure is None:
        parent = parent_identity()
        ok, problems, records = collect_worker_origins(state["dump_dir"], parent, int(args.workers))
        report.update(worker_origins_ok=ok, worker_origin_problems=problems, worker_origin_records=records[: 2 * int(args.workers)], parent_identity=parent)
        if not ok:
            failure = "WORKER_ORIGIN_MISMATCH: " + "; ".join(problems[:5])
            print(failure)
    if state.get("w1_recorder") is not None:
        state["w1_recorder"].write_index()
        print(f"W1_RECORDED {len(state['w1_recorder'].rows)} records {state['w1_recorder'].bytes} bytes", flush=True)
    if state.get("snapshot_recorder") is not None:
        recorded = state["snapshot_recorder"].close()
        report["snapshot_recorded"] = recorded
        print(f"SNAPSHOT_RECORDED {recorded['tasks']} tasks {recorded['file_bytes']} bytes ({recorded['raw_bytes']} raw) case={recorded['case']}", flush=True)
    report["failure"] = failure
    Path(args.out).write_text(json.dumps(report, ensure_ascii=False, indent=1, sort_keys=True, default=str) + "\n", encoding="utf-8")
    print(f"TREEAB_{'OK' if failure is None else 'FAILED'} case={args.case} label={args.label} workers={args.workers} origins={report.get('worker_origins_ok')}", flush=True)
    return 0 if failure is None else 1


def _inside_blender() -> bool:
    try:
        import bpy  # noqa: F401
    except ImportError:
        return False
    return True


if __name__ == "__main__":
    arguments = sys.argv[sys.argv.index("--") + 1 :] if "--" in sys.argv else sys.argv[1:]
    if "--worker" in arguments:
        if not _inside_blender():
            sys.exit("--worker runs inside Blender (the driver starts it); run this file without --worker")
        code = worker_main(arguments)
        if code:
            raise SystemExit(code)
    else:
        sys.exit(driver_main(sys.argv[1:]))
