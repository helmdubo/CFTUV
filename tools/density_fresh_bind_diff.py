"""Fresh rational principal producer -> outer bind -> typed facts differential.

No prepared ideal import, Python callback in Rust, product switch, or legacy memo.
"""
import argparse
from fractions import Fraction as F
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time
if hasattr(sys, "set_int_max_str_digits"):
    sys.set_int_max_str_digits(0)

ROOT = Path(os.environ.get("CFTUV_FRESH_WORKTREE_ROOT", Path(__file__).resolve().parents[1]))
ORACLE_ROOT = Path(os.environ.get("CFTUV_FRESH_ORACLE_ROOT", ROOT / "kernel/src"))
NATIVE_ROOT = Path(os.environ.get("CFTUV_FRESH_NATIVE_SOURCE_ROOT", ROOT / "native/cftuv-density/src"))
sys.path.insert(0, str(ORACLE_ROOT))
from cftuv_envelope.reference.density_construct_v1 import DensityConstructArenaV1, ExpandLimitsV1
from cftuv_envelope.reference.density_oracle_v1 import DensityOracleLimitsV1, DensityOracleCanonV1, DENSITY_PRINCIPAL_ANGLE_A2_FIXED160_V1
from cftuv_envelope.reference.density_scalar_v2 import DENSITY_SOURCE_CONSTRUCTION_V1
from cftuv_envelope.reference.density_shell_v2 import DENSITY_SHELL_CANON_V2_CANDIDATE
from cftuv_envelope.reference.density_owned_laws_v1 import OwnedDensityLawsV1
from cftuv_envelope.reference.density_controller_v1 import bind_owned_density_fan
from cftuv_envelope.contracts.analysis import TurnOrientation

CANON = DensityOracleCanonV1(DENSITY_SOURCE_CONSTRUCTION_V1, DENSITY_SHELL_CANON_V2_CANDIDATE, DENSITY_PRINCIPAL_ANGLE_A2_FIXED160_V1)
LAWS = {"V": "VORONOI_OF_NEIGHBOUR_MIDPOINTS", "N": "NARROW_ROTATION_BAND"}

def ratio(value):
    value = F(value)
    return [value.numerator, value.denominator]

def interval(value):
    def coefficient(value):
        row = value.as_tuple()
        return int("".join(map(str, row.digits))) * (-1 if row.sign else 1)
    return {"lower": ratio(value.lower), "upper": ratio(value.upper),
            "absolute_error_bound": ratio(value.absolute_error_bound),
            "quantum": [value.lower.as_tuple().exponent, value.upper.as_tuple().exponent,
                        value.absolute_error_bound.as_tuple().exponent],
            "lower_kind": value.lower_kind.value, "upper_kind": value.upper_kind.value,
            "decimal_coefficients": [coefficient(v) for v in (value.lower, value.upper, value.absolute_error_bound)]}

def certificate(value):
    if value is None:
        return None
    width = value.certified_window_width_lower_bound.as_tuple()
    coefficient = int("".join(map(str, width.digits))) * (-1 if width.sign else 1)
    return {"vector": list(value.bound_primitive_integer_vector),
            "lower": interval(value.ideal_window_lower_slope_envelope),
            "upper": interval(value.ideal_window_upper_slope_envelope),
            "width": [coefficient, width.exponent], "predicates": sorted(value.proven_predicates), "binding_reason": None}

def record(value):
    return {"use_x": value.use_x_denominator, "sign": value.denominator_sign,
            "lower": interval(value.full_lower_slope_envelope), "upper": interval(value.full_upper_slope_envelope)}

def piece(value):
    return {"index": value.piece_index, "use_x": value.use_x_denominator, "sign": value.denominator_sign,
            "lower": interval(value.lower_slope_envelope), "upper": interval(value.upper_slope_envelope),
            "lower_included": value.lower_endpoint_included, "upper_included": value.upper_endpoint_included,
            "increases": value.slope_increases_in_ordinal_order, "pole": value.pole_ownership.value}

def window(value):
    bounds = {"ordinal": value.ordinal, "termination_lower": list(value.termination_lower_slope),
              "termination_upper": list(value.termination_upper_slope), "width": list(value.certified_termination_width),
              "admissible": [list(getattr(value, name)) for name in ("admissible_lower_outward", "admissible_lower_inward", "admissible_upper_inward", "admissible_upper_outward")]}
    if hasattr(value, "pieces"):
        return {"kind": "ATLAS", "bounds": bounds, "pieces": [piece(p) for p in value.pieces], "termination_piece": value.termination_piece_index}
    return {"kind": "SINGLE", "bounds": bounds, "record": record(value)}

def authority(value):
    if value is None:
        return None
    return {"q": value.max_subturn_q, "height": value.minimal_common_height,
            "previous_height": value.exhaustive_previous_height, "termination_height": value.termination_height_upper_bound,
            "vectors": [list(v) for v in value.bound_primitive_integer_vectors],
            "windows": [window(w) for w in value.ordinal_windows],
            "previous_counts": list(value.previous_height_witness.primitive_candidate_counts)}

def cases():
    for inverse in [(F(1), F(0), F(1)), (F(2), F(1, 2), F(1))]:
        for q in range(2, 7):
            for hidden in range(q):
                for owner in (-1, 1):
                    for turn in (-1, 1):
                        for law in LAWS:
                            for lifted in (False, True):
                                for caps in ((65536, 131072, 96, 1000000), (0, 0, 2, 10)):
                                    yield dict(inverse=inverse, q=q, hidden=hidden, owner=owner, turn=turn, law=law, lifted=lifted, caps=caps)

def wire(case):
    inv = case["inverse"]
    incoming = (F(1), F(0))
    outgoing = (F(0), F(case["owner"] * case["turn"]))
    preparation, search, boxes, height = case["caps"]
    return "\t".join(map(str, ("fan", *inv, case["owner"], case["turn"], case["q"], case["hidden"],
                               *incoming, *outgoing, case["law"], "adaptive" if case["lifted"] else "auto",
                               ",".join("0" for _ in range(case["hidden"])) or "_", preparation, search, height, boxes)))

def expected(case):
    preparation, search, boxes, height = case["caps"]
    laws = OwnedDensityLawsV1(DensityConstructArenaV1(), DensityOracleLimitsV1(preparation, search, boxes, height, ExpandLimitsV1(4096, 16, 65536)))
    a, b, c = case["inverse"]
    det = a*c-b*b
    metric = laws.metric((c/det, -b/det, a/det), (a, b, c), case["owner"])
    refusals = []
    try:
        ideal = laws.principal_fan(metric, laws.vector(laws.scalar(1), laws.scalar(0)),
                                   laws.vector(laws.scalar(0), laws.scalar(case["owner"]*case["turn"])), case["turn"], case["hidden"])
        # Producer возвращает ковекторы; публичный Python controller принимает primal normals.
        ideal = tuple(laws.vector(a*vector.x+b*vector.y, b*vector.x+c*vector.y) for vector in ideal)
        orientation = TurnOrientation.CCW_IN_OWNER_PATCH_ORIENTATION if case["turn"] == 1 else TurnOrientation.CW_IN_OWNER_PATCH_ORIENTATION
        certs, fan = bind_owned_density_fan(laws, metric, ideal, orientation, case["q"], (None,)*case["hidden"],
                                           lifted=case["lifted"], law=LAWS[case["law"]], spec_id="synthetic", refusals=refusals, canon_ids=CANON)
        answer = {"certificates": [certificate(v) for v in certs], "authority": authority(fan),
                  "authority_predicates": sorted(fan.proven_predicates) if fan is not None else [], "binding_reasons": [None]*case["hidden"]}
    except Exception as error:
        answer = {"error": f"{type(error).__name__}: {error}"}
    answer["budgets"] = [[kind, [budget.cap, budget.shell_probes, budget.order_steps]] for kind, budget in laws.budgets]
    answer["refusals"] = [text for _, text in refusals]
    return answer

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--replay", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--limit", type=int)
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    started = time.time()
    source_paths = [NATIVE_ROOT/name for name in ("density_bind_v1.rs", "density_laws_v1.rs", "density_result_v1.rs")]
    source_paths += [ORACLE_ROOT/"cftuv_envelope/reference"/name for name in
                     ("density_owned_laws_v1.py", "density_controller_v1.py", "adaptive_density_fan.py", "density_construct_v1.py", "density_angle_value_v1.py", "density_result_v1.py")]
    source_paths += [Path(__file__), args.replay]
    manifest_path = os.environ.get("CFTUV_FRESH_SNAPSHOT_MANIFEST")
    manifest = json.loads(Path(manifest_path).read_text(encoding="utf-8-sig")) if manifest_path else None
    if manifest:
        source_paths += [Path(row["snapshot"]) for row in manifest["files"]]
    source_paths = list(dict.fromkeys(source_paths))
    hashes = {str(path): hashlib.sha256(path.read_bytes()).hexdigest() for path in source_paths}
    process = subprocess.Popen([str(args.replay)], stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True, encoding="utf-8")
    (args.out/"running.json").write_text(json.dumps({"python_pid": os.getpid(), "replay_pid": process.pid, "scope": "fresh rational principal producer, not raw incident frontend"}), encoding="utf-8")
    counts = {"records": 0, "equal": 0, "mismatch": 0}
    try:
        with (args.out/"records.jsonl").open("w", encoding="utf-8") as output:
            for index, case in enumerate(cases()):
                if args.limit is not None and index >= args.limit:
                    break
                if any(__import__("shutil").disk_usage(drive).free < 1500000000 for drive in ("C:/", "E:/")):
                    raise RuntimeError("DISK_BELOW_1_5GB")
                line = wire(case)
                process.stdin.write(line+"\n")
                process.stdin.flush()
                observed = json.loads(process.stdout.readline())
                oracle = expected(case)
                different = sorted(key for key in oracle.keys() | observed.keys() if oracle.get(key) != observed.get(key))
                counts["records"] += 1
                counts["mismatch" if different else "equal"] += 1
                output.write(json.dumps({"index": index, "wire": line, "different": different, "python": oracle, "rust": observed})+"\n")
                output.flush()
                if index % 10 == 0:
                    (args.out/"progress.json").write_text(json.dumps(counts), encoding="utf-8")
                    print(json.dumps(counts), flush=True)
    finally:
        process.stdin.close()
        process.wait(timeout=10)
    changed = [str(path) for path in source_paths if hashlib.sha256(path.read_bytes()).hexdigest() != hashes[str(path)]]
    receipt = {"status": "ACCEPTED_BOUNDED" if not counts["mismatch"] and not changed else "FAILED_DIFFERENTIAL", "scope": "fresh rational principal producer -> outer bind -> typed output; not whole R3 raw incident acceptance or release gate", "counts": counts, "elapsed_seconds": time.time()-started, "source_hashes": hashes, "source_changed": changed, "python_pid": os.getpid(), "replay_pid": process.pid, "snapshot_manifest": manifest_path, "replay_executable_sha256": hashes[str(args.replay)]}
    (args.out/"receipt.json").write_text(json.dumps(receipt, indent=2)+"\n", encoding="utf-8")
    print(json.dumps(receipt), flush=True)
    return bool(counts["mismatch"] or changed)

if __name__ == "__main__":
    raise SystemExit(main())
