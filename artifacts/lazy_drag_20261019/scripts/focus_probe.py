"""Headless probe on a pickled preparation (no press): which materialize decision flips between two widths of one domain.

    blender -b <scene> --python-exit-code 1 --python focus_probe.py -- --root D:/CFTUV-lazydrag --native-path D:/lazydrag-scratch/site --pickle <file> --widths w1,w2,... --out out.json
Records, per width: the silhouette input (pre-dissolve polygons), every edge-merge and vertex-dissolve decision with its float quantities
(dihedral key, chord depth, UV residual, sag key, chord, slide) and tallies, the structure signature parts, faces/vertices counts.
"""
from __future__ import annotations

import argparse
import json
import pickle
import sys
import traceback
from fractions import Fraction
from pathlib import Path

sys.dont_write_bytecode = True


def main() -> int:
    argv = sys.argv[sys.argv.index("--") + 1:]
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", required=True)
    parser.add_argument("--native-path", required=True)
    parser.add_argument("--pickle", required=True)
    parser.add_argument("--widths", required=True)
    parser.add_argument("--out", required=True)
    args = parser.parse_args(argv)
    root = Path(args.root).resolve()
    sys.path.insert(0, str(root / "tools"))
    import blender_tree_ab as T

    T.load_tree(root, args.native_path)
    report = {"failure": None, "widths": {}}
    try:
        report["native"] = T.verify_native(args.native_path, "")["build_id"]
        import cftuv_envelope.materialize.domain as D
        import cftuv_envelope.materialize.silhouette as S
        from cftuv_envelope import wavefront

        with open(args.pickle, "rb") as handle:
            blob = pickle.load(handle)
        prepared, kw = blob["prepared"], dict(blob["kw"])
        report["patch"] = blob["patch"]
        report["alpha_text"] = blob["alpha_text"]
        report["interval"] = blob["interval"]
        log: list = []
        state = {}
        real_apply = D.apply_silhouette
        real_merge, real_dissolve, real_edges = S._Mesh._try_merge, S._Mesh._try_dissolve, S._Mesh.dissolve_edges
        real_vertices = S._Mesh.dissolve_vertices

        def apply(source, budget):
            state["input_rings"] = sorted(tuple(S._ring_key(r)) if hasattr(S, "_ring_key") else tuple(r) for polys in source.polygons for r in polys)
            state["input_faces"] = sum(len(p) for p in source.polygons)
            state["input_vertices"] = len({k for polys in source.polygons for r in polys for k in r})
            result = real_apply(source, budget)
            state["output_faces"] = sum(len(p) for p in result.polygons)
            state["counters"] = [list(x) for x in result.counters]
            return result

        def dissolve_edges(self):
            cands = []
            for (first, second), number in self.half.items():
                other = self.half.get((second, first))
                if first >= second or other is None or other == number:
                    continue
                one, two = self.faces[number], self.faces[other]
                if one.region == two.region:
                    cands.append((self.dihedral(one, two), first, second))
            log.append({"kind": "EDGE_ORDER", "order": [[repr(a), f, s] for a, f, s in sorted(cands)]})
            return real_edges(self)

        def try_merge(self, first, second):
            before = dict(self.tally)
            one_id, two_id = self.half.get((first, second)), self.half.get((second, first))
            entry = {"kind": "MERGE", "edge": [first, second]}
            if one_id is not None and two_id is not None and one_id != two_id:
                one, two = self.faces[one_id], self.faces[two_id]
                entry["rings"] = [list(one.ring), list(two.ring)]
                if not (one.repeats or two.repeats or set(one.ring) & set(two.ring) != {first, second}):
                    union = self._union(one, two, first, second)
                    depth = self._merge_depth(one, two, union)
                    entry["depth"] = None if depth is None else repr(depth)
                    entry["tolerated"] = None if not self.source.uv_slide else repr(self._tolerated(one.region, union)) if not self._affine_union(one, two) else "AFFINE"
            real_merge(self, first, second)
            entry["tally_delta"] = {k: v - before.get(k, 0) for k, v in self.tally.items() if v != before.get(k, 0)}
            log.append(entry)

        def dissolve_vertices(self):
            found = real_vertices(self)
            return found

        def try_dissolve(self, key, incident, covered, gone):
            before = dict(self.tally)
            entry = {"kind": "VERTEX", "key": key}
            line = self._line(key, incident)
            if line is not None:
                b, a = line
                left, right = frozenset((b, key)), frozenset((key, a))
                run = (*covered.get(left, ()), *covered.get(right, ()), key)
                low, high = sorted((b, a))
                start, end = self.xyz(low), self.xyz(high)
                entry["line"] = [b, a]
                entry["run"] = list(run)
                entry["worst_chord"] = repr(max(S._along(start, end, self.xyz(item))[1] for item in run))
                slide = self._slide(incident[key], low, high, run)
                entry["slide"] = None if slide is None else repr(slide)
                entry["planned"] = key in self.planned
            ok = real_dissolve(self, key, incident, covered, gone)
            entry["dissolved"] = ok
            entry["tally_delta"] = {k: v - before.get(k, 0) for k, v in self.tally.items() if v != before.get(k, 0)}
            log.append(entry)
            return ok

        D.apply_silhouette = apply
        S._Mesh._try_merge = try_merge
        S._Mesh._try_dissolve = try_dissolve
        S._Mesh.dissolve_edges = dissolve_edges
        report["uv_slide"] = None
        for text in args.widths.split(","):
            log.clear()
            state.clear()
            coverage = wavefront.conveyor_coverage(prepared, text)
            result = D.materialize_domain(prepared, coverage, request=kw["request"], near_planar_lift_law=kw["near_planar_lift_law"],
                                          decal_topology_law=kw["decal_topology_law"], digests=False, certify=True)
            row = {"materialized": result.is_materialized, "outcome": str(result.outcome)}
            if result.is_materialized:
                batch = result.batch
                row["faces"] = len(batch.faces)
                row["vertices"] = len(batch.vertices)
                row["rings"] = sorted(sorted(k.value for k in f.ordered_vert_keys) for f in batch.faces)
                row["vertex_keys"] = sorted(v.vert_key.value for v in batch.vertices)
                row["structure"] = dict(result.structure.parts)
                row["interval"] = None if result.interval is None else result.interval.as_record()
                row["counters"] = {str(n): v for n, v in result.counters if "SILHOUETTE" in str(n) or "STATION_PLAN" in str(n) or "CLIP" in str(n) or "POLYGON" in str(n) or "FAN" in str(n) or "QUAD" in str(n) or "DIAGONAL" in str(n) or "SNAP" in str(n)}
                row["diagnostics"] = list(result.diagnostics)
            row.update({k: v for k, v in state.items()})
            row["log"] = list(log)
            report["widths"][text] = row
            print("PROBE", text, row.get("faces"), row.get("vertices"), flush=True)
    except Exception:  # noqa: BLE001
        report["failure"] = traceback.format_exc()
        print(report["failure"], flush=True)
    Path(args.out).write_text(json.dumps(report, indent=1, default=str), encoding="utf-8")
    print("PROBE_OK" if report["failure"] is None else "PROBE_FAILED", flush=True)
    return 0 if report["failure"] is None else 1


if __name__ == "__main__":
    code = main()
    if code:
        raise SystemExit(code)
