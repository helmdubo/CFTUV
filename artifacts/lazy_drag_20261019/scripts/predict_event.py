"""Headless: predict the SILHOUETTE_UV_FIT crossing width of one edge merge of one domain from exact affine data, and compare with the flip.

    blender -b <scene> --python-exit-code 1 --python predict_event.py -- --root .. --native-path .. --pickle <file> --edge-first node:4
       --wa <alpha0> --wb <second width inside the interval> --hi <interval high> --out out.json
1. Two exact materializations (wa, wb) inside the certified interval capture the silhouette input of the union ring of the named merge:
   exact chart points (SqrtSumV1), exact facts (s, r) of its region and lattice alpha. Inside the interval they are exactly affine (AFFINE_DRAG_CERTIFICATE_V1),
   so p(w) = p(wa) + (p(wb) - p(wa)) * (w - wa) / (wb - wa) exactly (the certificate slope; two widths only stand in for it here).
2. m(w) = residual_real(w)^2 - tau^2 with residual_real = the least-squares UV fit residual of `uv_fit_residual` in REAL arithmetic (Fractions of
   300-bit enclosures), tau = silhouette_uv_slide. Sign scan + bisection on [wa, hi] -> predicted switching width w*.
3. The same predicate as the kernel computes it (binary64 `uv_fit_residual` on the binary64 centres of the predicted affine values) around w*: the float band.
4. Ground truth: the kernel's own decision at widths on both sides of w* (full materialize, decimal text) -> the observed flip bracket.
"""
from __future__ import annotations

import argparse
import json
import math
import pickle
import sys
import traceback
from fractions import Fraction
from pathlib import Path

sys.dont_write_bytecode = True
BITS = 300


def main() -> int:
    argv = sys.argv[sys.argv.index("--") + 1:]
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", required=True)
    parser.add_argument("--native-path", required=True)
    parser.add_argument("--pickle", required=True)
    parser.add_argument("--edge-first", required=True)
    parser.add_argument("--wa", required=True)
    parser.add_argument("--wb", required=True)
    parser.add_argument("--hi", required=True)
    parser.add_argument("--out", required=True)
    args = parser.parse_args(argv)
    root = Path(args.root).resolve()
    sys.path.insert(0, str(root / "tools"))
    import blender_tree_ab as T

    T.load_tree(root, args.native_path)
    report = {"failure": None}
    try:
        import cftuv_envelope.materialize.domain as D
        import cftuv_envelope.materialize.silhouette as S
        from cftuv_envelope import wavefront
        from cftuv_envelope.float_filter import centre_and_bound

        with open(args.pickle, "rb") as handle:
            blob = pickle.load(handle)
        prepared, kw = blob["prepared"], dict(blob["kw"])
        grab = {}
        real_merge = S._Mesh._try_merge

        def try_merge(self, first, second):
            if first == args.edge_first and "union" not in grab:
                one_id, two_id = self.half.get((first, second)), self.half.get((second, first))
                one, two = self.faces[one_id], self.faces[two_id]
                union = self._union(one, two, first, second)
                grab["union"] = list(union)
                grab["points"] = [self.source.points[k] for k in union]
                grab["facts"] = [self.source.facts[(one.region, k)] for k in union]
                grab["lattice_alpha"] = Fraction(self.source.lattice_alpha)
                grab["tau"] = Fraction(self.source.uv_slide)
                grab["kernel_residual"] = self._tolerated(one.region, union) if not self._affine_union(one, two) else "AFFINE"
                grab["chart_float"] = [tuple(centre_and_bound(c)[0] for c in p) for p in grab["points"]]
                grab["uv_float"] = [self.uv(one.region, k) for k in union]
            return real_merge(self, first, second)

        S._Mesh._try_merge = try_merge

        def run(text):
            grab.clear()
            coverage = wavefront.conveyor_coverage(prepared, text)
            result = D.materialize_domain(prepared, coverage, request=kw["request"], near_planar_lift_law=kw["near_planar_lift_law"],
                                          decal_topology_law=kw["decal_topology_law"], digests=False, certify=True)
            return dict(grab), result

        def mid(value):
            low, high = value.enclosure(BITS)
            return (low + high) / 2

        wa, wb = Fraction(args.wa), Fraction(args.wb)
        ga, ra = run(args.wa)
        gb, rb = run(args.wb)
        assert ga["union"] == gb["union"], "union ring differs between the two anchor widths"
        n = len(ga["union"])
        report["union"] = ga["union"]
        report["tau"] = str(ga["tau"])
        report["tau_float"] = float(ga["tau"])
        report["lattice_alpha_per_metre"] = [str(ga["lattice_alpha"] / wa), str(gb["lattice_alpha"] / wb)]
        scale = ga["lattice_alpha"] / wa
        # exact affine data: value at wa and slope per metre of width
        P0 = [(mid(p[0]), mid(p[1])) for p in ga["points"]]
        P1 = [(mid(p[0]), mid(p[1])) for p in gb["points"]]
        F0 = [(mid(f[0]), mid(f[1])) for f in ga["facts"]]
        F1 = [(mid(f[0]), mid(f[1])) for f in gb["facts"]]
        d = wb - wa
        PS = [((b[0] - a[0]) / d, (b[1] - a[1]) / d) for a, b in zip(P0, P1)]
        FS = [((b[0] - a[0]) / d, (b[1] - a[1]) / d) for a, b in zip(F0, F1)]
        tau = ga["tau"]

        def inputs(w):
            dw = w - wa
            lat = scale * w
            chart = [(p[0] + s[0] * dw, p[1] + s[1] * dw) for p, s in zip(P0, PS)]
            uvs = [((f[0] + s[0] * dw) / lat, (f[1] + s[1] * dw) / lat) for f, s in zip(F0, FS)]
            return chart, uvs

        def residual_sq_real(w):
            """`uv_fit_residual` in real arithmetic, squared (centring and scaling do not change the LSQ residual)."""
            chart, uvs = inputs(w)
            mx = sum(p[0] for p in chart) / n
            my = sum(p[1] for p in chart) / n
            xs = [p[0] - mx for p in chart]
            ys = [p[1] - my for p in chart]
            sxx = sum(x * x for x in xs); sxy = sum(x * y for x, y in zip(xs, ys)); syy = sum(y * y for y in ys)
            det = sxx * syy - sxy * sxy
            res = []
            for c in (0, 1):
                vals = [uv[c] for uv in uvs]
                cen = sum(vals) / n
                sh = [v - cen for v in vals]
                rx = sum(x * v for x, v in zip(xs, sh)); ry = sum(y * v for y, v in zip(ys, sh))
                bx = (rx * syy - ry * sxy) / det; by = (sxx * ry - sxy * rx) / det
                res.append([v - (bx * x + by * y) for v, x, y in zip(sh, xs, ys)])
            return max(a * a + b * b for a, b in zip(*res))

        def m(w):
            return residual_sq_real(w) - tau * tau

        # check: the affine model reproduces the kernel's float inputs at wa and wb
        report["model_check_wb_chart_max_abs"] = max(abs(float(c) - f) for pc, pf in zip(inputs(wb)[0], gb["chart_float"]) for c, f in zip(pc, pf))
        report["model_check_wb_uv_max_abs"] = max(abs(float(c) - f) for pc, pf in zip(inputs(wb)[1], gb["uv_float"]) for c, f in zip(pc, pf))
        hi = Fraction(args.hi)
        grid = [wa + (hi - wa) * Fraction(k, 400) for k in range(401)]
        signs = [(w, m(w)) for w in grid]
        report["m_at_wa"] = float(signs[0][1])
        crossings = []
        for (w0, m0), (w1, m1) in zip(signs, signs[1:]):
            if (m0 > 0) != (m1 > 0):
                lo, up = w0, w1
                for _ in range(80):
                    md = (lo + up) / 2
                    if (m(md) > 0) == (m0 > 0):
                        lo = md
                    else:
                        up = md
                crossings.append({"w_star": float((lo + up) / 2), "w_star_text": f"{float(lo):.17g}..{float(up):.17g}", "direction": "ACCEPT_ABOVE" if m0 > 0 else "REJECT_ABOVE"})
        report["predicted_crossings"] = crossings
        # the kernel's float predicate on the predicted inputs around w*
        if crossings:
            ws = Fraction(crossings[0]["w_star"])
            band = []
            for k in range(-12, 13):
                w = ws + Fraction(k, 10**14)
                chart, uvs = inputs(w)
                r = S.uv_fit_residual([(float(a), float(b)) for a, b in chart], [(float(a), float(b)) for a, b in uvs])
                band.append([float(w), r, None if r is None else (Fraction(r) <= tau)])
            report["float_predicate_near_w_star"] = band
            # ground truth from the kernel at decimal widths around w*
            truth = []
            for k in (-1000, -100, -10, -2, -1, 0, 1, 2, 10, 100, 1000):
                w = ws + Fraction(k, 10**13)
                text = f"{float(w):.15g}"
                g, _r = run(text)
                truth.append([text, None if not g else repr(g.get("kernel_residual"))])
            report["kernel_truth"] = truth
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
