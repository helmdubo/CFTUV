"""Frozen bind replay of explicit laws seam; candidate tooling only."""

import argparse
import hashlib
import json
import os
from fractions import Fraction
from pathlib import Path
import sys
import time

sys.set_int_max_str_digits(0)

from density_tool_paths import KERNEL_ROOT, PACKAGE_ROOT, TOOLS_ROOT

ROOT = KERNEL_ROOT.parent

from native_density_corpus import read_document, replay, run_record


class _PreparedImporter:
    """Offline input AST import only; never a rawfacts producer acceptance."""

    def __init__(self, laws):
        self.laws, self.memo = laws, {}

    def expression(self, expression):
        import sympy as sp
        from cftuv_envelope.reference.density_scalar_v2 import ScalarLiteralAstV1, DENSITY_SOURCE_CONSTRUCTION_V1

        if expression in self.memo:
            return self.memo[expression]
        rows, visited = [], {}
        def emit(node):
            if node in visited:
                return visited[node]
            if isinstance(node, sp.Rational):
                row = ("Q", int(node.p), int(node.q))
            elif node is sp.pi:
                row = ("pi",)
            elif node.func is sp.atan:
                arg = emit(node.args[0])
                rows.append(("Q", 1, 1))
                row = ("atan2", arg, len(rows)-1)
            else:
                tag = {sp.Add: "Add", sp.Mul: "Mul", sp.Pow: "Pow", sp.sin: "sin",
                       sp.cos: "cos", sp.atan2: "atan2"}.get(node.func)
                if tag is None:
                    raise ValueError(f"DENSITY_PREPARED_IMPORT_PROFILE:{node.func}")
                row = (tag, *(emit(child) for child in node.args))
            rows.append(row)
            visited[node] = len(rows)-1
            return len(rows)-1
        root = emit(expression)
        value = self.laws.wrap(self.laws.arena.import_raw(ScalarLiteralAstV1(DENSITY_SOURCE_CONSTRUCTION_V1, tuple(rows), root)))
        self.memo[expression] = value
        return value

    def register_angles(self, expressions, n):
        import sympy as sp
        from cftuv_envelope.reference.density_angle_v1 import AngleAtomV1

        arguments = set()
        for expression in expressions:
            arguments.update(node.args[0] for node in sp.preorder_traversal(expression) if node.func in (sp.cos, sp.sin))
        registered, gaps = 0, []
        for argument in sorted(arguments, key=sp.default_sort_key):
            cosine = self.expression(sp.cos(argument, evaluate=False)).value
            sine = self.expression(sp.sin(argument, evaluate=False)).value
            coefficient, base = argument.as_coeff_Mul()
            coefficient = Fraction(int(coefficient.p), int(coefficient.q))
            quarter_turns = 0
            if base is sp.pi:
                self.laws.values.register_canonical_ray(cosine, sine, coefficient)
            elif base.func in (sp.atan, sp.atan2):
                source_y = self.expression(base.args[0])
                source_x = self.expression(base.args[1]) if base.func is sp.atan2 else self.laws.scalar(1)
                ray_factor = coefficient
            elif base.func is sp.Add and len(base.args) == 2:
                # Два наблюдённых source atan2 forms: phi и phi-pi/2.
                parts = tuple(term.as_coeff_Mul() for term in base.args)
                pi_parts = tuple((factor, child) for factor, child in parts if child is sp.pi)
                atan_parts = tuple((factor, child) for factor, child in parts if child.func is sp.atan)
                if len(pi_parts) != 1 or len(atan_parts) != 1:
                    gaps.append("UNRECOGNIZED_SOURCE_ANGLE")
                    continue
                pi_factor, _ = pi_parts[0]
                atan_factor, source_atan = atan_parts[0]
                pi_factor = coefficient * Fraction(int(pi_factor.p), int(pi_factor.q))
                atan_factor = coefficient * Fraction(int(atan_factor.p), int(atan_factor.q))
                ray_factor = -atan_factor
                shift = 2 * (pi_factor - ray_factor)
                if ray_factor <= 0 or shift not in (0, -1):
                    gaps.append("OUTSIDE_PRINCIPAL_RECIPE")
                    continue
                quarter_turns = int(shift)
                source_x, source_y = self.laws.scalar(-1), self.expression(source_atan.args[0])
            else:
                gaps.append("UNRECOGNIZED_SOURCE_ANGLE")
                continue
            if base is not sp.pi:
                ordinal = ray_factor*n
                if (type(n) is not int or not 2 <= n <= 6 or ordinal.denominator != 1
                        or not 0 < ordinal < n or self.laws.sign(source_y) <= 0):
                    gaps.append("OUTSIDE_PRINCIPAL_RECIPE")
                    continue
                ratio = self.laws.fraction((source_x*source_x)/(source_x*source_x+source_y*source_y))
                if not 0 <= ratio < 1:
                    gaps.append("OUTSIDE_PRINCIPAL_RECIPE")
                    continue
                self.laws.values.register_ray_with_quarter_turns(
                    cosine, sine, AngleAtomV1(n, ratio, self.laws.sign(source_x)), int(ordinal), quarter_turns)
            registered += 1
        return registered, gaps


def _owned_replay(document):
    from cftuv_envelope.reference.density_construct_v1 import DensityConstructArenaV1, ExpandLimitsV1
    from cftuv_envelope.reference.density_oracle_v1 import DensityOracleLimitsV1, DensityOracleCanonV1, DENSITY_PRINCIPAL_ANGLE_A2_FIXED160_V1
    from cftuv_envelope.reference.density_scalar_v2 import DENSITY_SOURCE_CONSTRUCTION_V1
    from cftuv_envelope.reference.density_shell_v2 import DENSITY_SHELL_CANON_V2_CANDIDATE
    from cftuv_envelope.reference.density_owned_laws_v1 import OwnedDensityLawsV1
    from cftuv_envelope.reference.density_controller_v1 import bind_owned_density_fan

    observations = []
    for row in document["records"]:
        details = {"sequence": row["sequence"], "input_literal_import": True}
        settings = row["input"]["settings"]
        def compute(metric, ideal, orientation, q, reasons, **kwargs):
            limits = DensityOracleLimitsV1(settings["preparation_cap"], settings["search_cap"], settings["box_cap"], settings["height_cap"], ExpandLimitsV1(4096, 16, 65536))
            laws = OwnedDensityLawsV1(DensityConstructArenaV1(), limits)
            importer = _PreparedImporter(laws)
            def qvalue(value):
                return Fraction(int(value.p), int(value.q))
            owned_metric = laws.metric(tuple(qvalue(metric.gram[i][j]) for i, j in ((0, 0), (0, 1), (1, 1))),
                                      tuple(qvalue(metric.inverse_gram[i][j]) for i, j in ((0, 0), (0, 1), (1, 1))), metric.owner_orientation_sign)
            details["legacy_warm_fan_entries_not_reused"] = len(metric._density_exact_memo.fan_bindings)
            details["candidate_initial_memo"] = "COLD; legacy canon does not authorize candidate cached results"
            source = [metric.density_expressions(vector) for vector in ideal]
            owned_ideal = tuple(laws.vector(importer.expression(x), importer.expression(y)) for x, y in source)
            count, gaps = importer.register_angles([value for pair in source for value in pair], len(ideal)-1)
            details["A2_registered_pairs"], details["A2_metadata_gaps"] = count, gaps
            ids = DensityOracleCanonV1(DENSITY_SOURCE_CONSTRUCTION_V1, DENSITY_SHELL_CANON_V2_CANDIDATE, DENSITY_PRINCIPAL_ANGLE_A2_FIXED160_V1)
            return bind_owned_density_fan(laws, owned_metric, owned_ideal, orientation, q, reasons, canon_ids=ids, **kwargs)
        actual = run_record(document, row, compute)
        differences = [key for key in set(row["outcome"]) | set(actual) if row["outcome"].get(key) != actual.get(key)]
        observations.append({**details, "different_fields": sorted(differences), "expected": row["outcome"], "actual": actual})
    return {"records": len(observations), "equal": sum(not row["different_fields"] for row in observations),
            "mismatch": sum(bool(row["different_fields"]) for row in observations), "observations": observations}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--receipt", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--owned", action="store_true")
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    started = time.time()
    source = json.loads(args.receipt.read_text(encoding="utf-8"))
    paths = [Path(row["file"]) for row in (*source["kernel"], *source["field"])]
    tracked = [PACKAGE_ROOT/"reference"/name for name in
               ("adaptive_density_fan.py", "density_legacy_laws_v1.py", "density_controller_v1.py")]
    hashes = {str(path): hashlib.sha256(path.read_bytes()).hexdigest() for path in tracked}
    rows = []
    for path in paths:
        result = _owned_replay(read_document(path)) if args.owned else replay(read_document(path))
        row = {"file": str(path), "sha256": hashlib.sha256(path.read_bytes()).hexdigest(), **result}
        rows.append(row)
        print(json.dumps({key: row[key] for key in ("file", "records", "equal", "mismatch")}), flush=True)
        (args.out/"progress.json").write_text(json.dumps(rows, indent=2)+"\n", encoding="utf-8")
    changed = [str(path) for path in tracked if hashlib.sha256(path.read_bytes()).hexdigest() != hashes[str(path)]]
    totals = {key: sum(row[key] for row in rows) for key in ("records", "equal", "mismatch")}
    receipt = {"status": "COMPLETED_DIAGNOSTIC" if args.owned and not changed else "COMPLETED" if not totals["mismatch"] and not changed else "FAILED",
               "scope": "Default legacy explicit leaf seam against frozen prepared bind corpus; outcomes/refusals/ordered budgets and diagnostics included. No owned acceptance claim.",
               "pid": os.getpid(), "elapsed_seconds": time.time()-started, "totals": totals,
               "sources": rows, "source_hashes": hashes, "source_changed": changed}
    if args.owned:
        receipt["scope"] = "Prepared literal import controller diagnostic. Legacy warm memo is not candidate cache. Recorded named deltas are not accepted; no fresh rawfacts whole-operation parity claim or legacy shell fallback."
    (args.out/"receipt.json").write_text(json.dumps(receipt, indent=2)+"\n", encoding="utf-8")
    print(json.dumps({"status": receipt["status"], "totals": totals}), flush=True)
    return 0 if receipt["status"].startswith("COMPLETED") else 1


if __name__ == "__main__":
    raise SystemExit(main())
