"""Headless corpus scan of the SILHOUETTE tolerance decisions (no press): margins at alpha0 and alpha0*(1+delta), linear crossing estimates.

    blender -b <scene> --python-exit-code 1 --python silhouette_scan.py -- --root .. --native-path .. --dir <pickles> --delta 0.01 --out out.json
Per domain and per decision (edge merge: chord depth vs CHORD_BUDGET, UV fit residual vs uv_slide; vertex dissolve: chord vs CHORD_BUDGET,
UV slide vs uv_slide; plus the float sort keys of the two greedy orders) the quantity at both widths; a decision whose margin changes sign
between the widths is a direct flip; otherwise the crossing of the linear extrapolation estimates where the event would lie (statistics only).
"""
from __future__ import annotations

import argparse
import json
import math
import pickle
import sys
import time
import traceback
from fractions import Fraction
from pathlib import Path

sys.dont_write_bytecode = True


def main() -> int:
    argv = sys.argv[sys.argv.index("--") + 1:]
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", required=True)
    parser.add_argument("--native-path", required=True)
    parser.add_argument("--dir", required=True)
    parser.add_argument("--delta", type=float, default=0.01)
    parser.add_argument("--limit", type=int, default=0)
    parser.add_argument("--out", required=True)
    args = parser.parse_args(argv)
    root = Path(args.root).resolve()
    sys.path.insert(0, str(root / "tools"))
    import blender_tree_ab as T

    T.load_tree(root, args.native_path)
    report = {"failure": None, "domains": []}
    try:
        import cftuv_envelope.materialize.domain as D
        import cftuv_envelope.materialize.silhouette as S
        from cftuv_envelope import wavefront
        from cftuv_envelope.float_filter import centre_and_bound

        log: dict = {}
        real_merge, real_dissolve, real_edges, real_vertices = S._Mesh._try_merge, S._Mesh._try_dissolve, S._Mesh.dissolve_edges, S._Mesh.dissolve_vertices
        tau_box = {}

        def dissolve_edges(self):
            tau_box["tau"] = float(self.source.uv_slide)
            keys = []
            for (first, second), number in self.half.items():
                other = self.half.get((second, first))
                if first >= second or other is None or other == number:
                    continue
                one, two = self.faces[number], self.faces[other]
                if one.region == two.region:
                    keys.append((self.dihedral(one, two), first, second))
            log["edge_order"] = sorted(keys)
            return real_edges(self)

        def try_merge(self, first, second):
            entry = {}
            one_id, two_id = self.half.get((first, second)), self.half.get((second, first))
            if one_id is not None and two_id is not None and one_id != two_id:
                one, two = self.faces[one_id], self.faces[two_id]
                if not (one.repeats or two.repeats or set(one.ring) & set(two.ring) != {first, second}):
                    union = self._union(one, two, first, second)
                    depth = self._merge_depth(one, two, union)
                    entry["depth"] = depth
                    if depth is not None and S._within(depth, S.CHORD_BUDGET) and not self._affine_union(one, two) and self.source.uv_slide:
                        chart = []
                        for key in union:
                            found = [centre_and_bound(c) for c in self.source.points[key]]
                            chart.append(None if None in found else (found[0][0], found[1][0]))
                        if None not in chart:
                            entry["residual"] = S.uv_fit_residual(chart, [self.uv(one.region, key) for key in union])
            before = self.tally[S.EDGES_DISSOLVED]
            real_merge(self, first, second)
            entry["merged"] = self.tally[S.EDGES_DISSOLVED] > before
            log.setdefault("merges", {})[f"{first}|{second}"] = entry

        def dissolve_vertices(self):
            found = real_vertices(self)
            return found

        def try_dissolve(self, key, incident, covered, gone):
            entry = {}
            line = self._line(key, incident)
            if line is not None:
                b, a = line
                run = (*covered.get(frozenset((b, key)), ()), *covered.get(frozenset((key, a)), ()), key)
                low, high = sorted((b, a))
                start, end = self.xyz(low), self.xyz(high)
                entry["chord"] = max(S._along(start, end, self.xyz(item))[1] for item in run)
                entry["sag"] = S._along(start, end, self.xyz(key))[1]
                entry["slide"] = self._slide(incident[key], low, high, run)
                entry["planned"] = key in self.planned
            ok = real_dissolve(self, key, incident, covered, gone)
            entry["dissolved"] = ok
            log.setdefault("vertices", {})[key] = entry
            return ok

        S._Mesh._try_merge = try_merge
        S._Mesh._try_dissolve = try_dissolve
        S._Mesh.dissolve_edges = dissolve_edges
        files = sorted(Path(args.dir).glob("*.pkl"))
        if args.limit:
            files = files[: args.limit]
        started = time.perf_counter()
        for number, path in enumerate(files):
            with open(path, "rb") as handle:
                blob = pickle.load(handle)
            prepared, kw = blob["prepared"], dict(blob["kw"])
            interval = blob["interval"] or {}
            w0 = Fraction(blob["alpha_text"])
            row = {"patch": blob["patch"], "status": interval.get("status"), "low": interval.get("low"), "high": interval.get("high")}
            w1 = w0 * (1 + Fraction(args.delta))
            if interval.get("status") != "CERTIFIED" or (interval.get("high") is not None and w1 >= Fraction(interval["high"])):
                w1 = w0 * (1 - Fraction(args.delta))
                if interval.get("status") != "CERTIFIED" or w1 <= Fraction(interval["low"]):
                    row["skip"] = "NO_ROOM"
                    report["domains"].append(row)
                    continue
            runs = []
            for w in (w0, w1):
                log.clear()
                tau_box.clear()
                text = blob["alpha_text"] if w == w0 else f"{float(w):.15g}"
                coverage = wavefront.conveyor_coverage(prepared, text)
                result = D.materialize_domain(prepared, coverage, request=kw["request"], near_planar_lift_law=kw["near_planar_lift_law"],
                                              decal_topology_law=kw["decal_topology_law"], digests=False, certify=True)
                runs.append((float(text), result.is_materialized, None if not result.is_materialized else result.structure.digest, dict(log), tau_box.get("tau")))
            row["w"] = [runs[0][0], runs[1][0]]
            row["structure_same"] = runs[0][2] == runs[1][2]
            row["tau"] = runs[0][4]
            row["log0"], row["log1"] = runs[0][3], runs[1][3]
            report["domains"].append(row)
            if number % 100 == 0:
                print(f"SCAN {number}/{len(files)} {time.perf_counter() - started:.0f}s", flush=True)
    except Exception:  # noqa: BLE001
        report["failure"] = traceback.format_exc()
        print(report["failure"], flush=True)
    Path(args.out).write_text(json.dumps(report, default=str), encoding="utf-8")
    print("PROBE_OK" if report["failure"] is None else "PROBE_FAILED", flush=True)
    return 0 if report["failure"] is None else 1


if __name__ == "__main__":
    code = main()
    if code:
        raise SystemExit(code)
