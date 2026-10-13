"""Actual Python controller differential, с подготовленными value/callback fixtures.

Authority ID и Decimal repr/quantum вне этого gate; primitive construction в shell
не является geometry callback. Алгоритмы вызываются из reference, не копируются.
"""
from __future__ import annotations

import argparse
from collections import Counter
from fractions import Fraction as F
import hashlib
import json
import lzma
import os
from pathlib import Path
import random
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "kernel/src"))
from cftuv_envelope.reference import adaptive_density_fan as fan, adaptive_density_atlas as atlas
from cftuv_envelope.reference import adaptive_density_band as band, density_binding_values as values
from cftuv_envelope.reference.common import ReferenceGeometryError
from cftuv_envelope.reference.contracts import ReferenceOutcome
from cftuv_envelope.contracts.analysis import TurnOrientation
from cftuv_envelope.contracts.envelopes import AdaptiveRationalFanProjectiveChartPieceV1 as Piece
from cftuv_envelope.contracts.envelopes import AdaptiveProjectivePoleOwnershipV1 as Pole
from cftuv_envelope.contracts.envelopes import AdaptiveRationalFanOrdinalWindowAtlasV1 as AtlasWire
from cftuv_envelope.numeric import CertifiedDecimalIntervalV1 as Env, IntervalEndpointKind as Kind

OLD_PREP = fan._preparation_budget
OLD_SEARCH = fan._work_budget
OLD_VECTOR = fan._vector_from_slope


class CallbackError(ValueError):
    pass


def fraction(value):
    value = F(value)
    return f"{value.numerator}/{value.denominator}"


def vector_text(value):
    return ",".join(map(fraction, value))


def vectors_text(values):
    return ";".join(map(vector_text, values)) or "_"


def envelope(lower, upper=None, width=F(0)):
    result = object.__new__(Env)
    for name, value in (("lower", F(lower)), ("upper", F(lower if upper is None else upper)),
                        ("lower_kind", Kind.CLOSED), ("upper_kind", Kind.CLOSED),
                        ("absolute_error_bound", F(width))):
        object.__setattr__(result, name, value)
    return result


def envelope_text(value):
    kind = lambda k: "C" if k is Kind.CLOSED else "O" if k is Kind.OPEN else "X"
    return "~".join((fraction(value.lower), fraction(value.upper), kind(value.lower_kind),
                     kind(value.upper_kind), fraction(value.absolute_error_bound)))


def local_text(record):
    return ",".join((str(int(record[0])), str(record[1]), envelope_text(record[2]), envelope_text(record[3])))


def piece_text(piece):
    return ",".join((str(piece.piece_index), str(int(piece.use_x_denominator)), str(piece.denominator_sign),
                     envelope_text(piece.lower_slope_envelope), envelope_text(piece.upper_slope_envelope),
                     str(int(piece.lower_endpoint_included)), str(int(piece.upper_endpoint_included)),
                     str(int(piece.slope_increases_in_ordinal_order)),
                     {Pole.NONE: "N", Pole.X_ZERO: "X", Pole.Y_ZERO: "Y"}[piece.pole_ownership]))


def record_text(record):
    if type(record) is atlas.AtlasWindowPrepared:
        return f"A!{record.termination_piece_index}!" + "@".join(map(piece_text, record.pieces)) + "!" + local_text(record.local_record)
    return "S!" + local_text(record)


def window_text(item):
    common = ",".join((str(item.ordinal), fraction(F(*item.termination_lower_slope)),
                        fraction(F(*item.termination_upper_slope)), fraction(F(*item.certified_termination_width)),
                        *[fraction(F(*getattr(item, name))) for name in ("admissible_lower_outward", "admissible_lower_inward", "admissible_upper_inward", "admissible_upper_outward")]))
    if type(item) is AtlasWire:
        return common + f"!A!{item.termination_piece_index}!" + "@".join(map(piece_text, item.pieces))
    return common + "!S!" + local_text((item.use_x_denominator, item.denominator_sign,
                                        item.full_lower_slope_envelope, item.full_upper_slope_envelope))


def authority_text(result, current_counts):
    count = lambda values: ",".join(map(str, values)) or "_"
    return "\t".join(("OK", "Q:" + str(result.max_subturn_q),
                     "HEIGHT:" + str(result.minimal_common_height),
                     "PREVIOUS_HEIGHT:" + str(result.exhaustive_previous_height),
                     "TERMINATION:" + str(result.termination_height_upper_bound),
                     "FAN:" + ";".join(",".join(map(str, v)) for v in result.bound_primitive_integer_vectors),
                     "PREVIOUS_COUNTS:" + count(result.previous_height_witness.primitive_candidate_counts),
                     "COUNTS:" + count(current_counts),
                     "WINDOWS:" + ("#".join(map(window_text, result.ordinal_windows)) or "_")))


def oracle_metric(ideal, orientation, call):
    class Metric:
        def density_expressions(self, value):
            use_x = sys._getframe(1).f_locals["use_x"]
            call(f"D:{vector_text(value)}:{int(use_x)}", lambda: value[0 if use_x else 1], fraction)
            return value
    metric = Metric()
    ideals = fan._IdealTuple(ideal)
    ideals.metric, ideals.window_law = metric, band.WINDOW_LAW_VORONOI
    ideals.band_orientation, ideals.band_cache = orientation, {}
    return metric, ideals


def oracle_case(name, records, *, ideal=None, prep=65536, search=131072, box=96, height=10,
                q=5, orientation=1, throw=0, L="1", I="1", X=None, U="1", limit="false",
                boundaries=((F(1), -F(3, 4)), (F(1), F(3, 4))), width=F(0)):
    if X is None:
        X = [orientation]
    if ideal is None:
        ideal = [(F(1), F(-2)), *[(F(1), F(0)) for _ in records], (F(1), F(2))]
    events, budgets, counters = [], [], [0, 0, 0, 0]
    patterns = (L, I, X, U)

    def next_value(kind):
        value = patterns[kind][counters[kind] % len(patterns[kind])]
        counters[kind] += 1
        return int(value)

    def call(prefix, compute, formatter=str):
        if throw and len(events) + 1 == throw:
            events.append(prefix + ":!CallbackError:refused")
            raise CallbackError("refused")
        try:
            result = compute()
        except (ValueError, ArithmeticError) as error:
            events.append(prefix + ":!" + type(error).__name__ + ":" + str(error))
            raise
        events.append(prefix + ":" + formatter(result))
        return result

    metric, ideals = oracle_metric(ideal, orientation, call)
    oriented = TurnOrientation.CCW_IN_OWNER_PATCH_ORIENTATION if orientation == 1 else TurnOrientation.CW_IN_OWNER_PATCH_ORIENTATION

    def bound(m, i, o, s, q):
        return call(f"B:{o}:{fan._expected_orientation(s)}:{q}", lambda: boundaries, vectors_text)

    def sign(value, metric):
        return call("S:" + fraction(value), lambda: int(value > 0) - int(value < 0))

    def slope(value, use_x, metric):
        return call(f"T:{vector_text(value)}:{int(use_x)}", lambda: F(value[1 if use_x else 0], value[0 if use_x else 1]), fraction)

    def enclosure(value, metric):
        return call("E:" + fraction(value), lambda: envelope(value - width, value + width, width), envelope_text)

    def primitive(value, use_x, sign):
        # Целочисленная генерация shell принадлежит существующему Farey/AtlasSearch.
        if sys._getframe(1).f_code.co_name in ("_farey_shell_candidates", "segment_candidates"):
            return OLD_VECTOR(value, use_x, sign)
        return call(f"V:{fraction(value)}:{int(use_x)}:{sign}", lambda: OLD_VECTOR(value, use_x, sign), vector_text)

    def candidate(value, use_x, sign, metric):
        return call(f"Q:{fraction(value)}:{int(use_x)}:{sign}", lambda: OLD_VECTOR(value, use_x, sign), vector_text)

    def local(metric, ideal, ordinal, vector, q):
        return call(f"L:{ordinal}:{q}:{vector_text(vector)}", lambda: bool(next_value(0)), lambda v: str(int(v)))

    def neighbours(ideal, ordinal):
        return call(f"N:{ordinal}", lambda: (ideal[ordinal - 1], ideal[ordinal + 1]), lambda v: vector_text(v[0]) + "|" + vector_text(v[1]))

    def inside(metric, vector, previous, center, following):
        return call(f"I:{vector_text(vector)}|{vector_text(previous)}|{vector_text(center)}|{vector_text(following)}", lambda: bool(next_value(1)), lambda v: str(int(v)))

    def cross(metric, left, right):
        return call(f"X:{vector_text(left)}|{vector_text(right)}", lambda: F(next_value(2)), fraction)

    def subturn(metric, left, right, q):
        return call(f"U:{q}:{vector_text(left)}|{vector_text(right)}", lambda: bool(next_value(3)), lambda v: str(int(v)))

    def diagnostic(metric, left, right, q):
        def value():
            if limit == "ref":
                raise ReferenceGeometryError(ReferenceOutcome.REFERENCE_CERTIFIED_PREDICATE_UNDECIDABLE, "diagnostic_unproven")
            if limit == "other":
                raise CallbackError("diagnostic_failed")
            return limit == "true"
        return call(f"F:{q}:{vector_text(left)}|{vector_text(right)}", value, lambda v: str(int(v)))

    def comparator(metric, ideal, ordinal):
        def compare(left, right):
            a, b = sum(left) % 7, sum(right) % 7
            return call(f"K:{ordinal}:{vector_text(left)}|{vector_text(right)}", lambda: int(a > b) - int(a < b))
        return compare

    def prep_budget(cap=None):
        budget = OLD_PREP(prep)
        budgets.append(budget)
        return budget

    def search_budget(cap=None):
        budget = OLD_SEARCH(search)
        budgets.append(budget)
        return budget

    fan._subturn_boundary_vectors, fan._sign, fan._slope = bound, sign, slope
    values._density_decimal_envelope = enclosure
    fan._vector_from_slope, fan._candidate_vector, fan._local_candidate = primitive, candidate, local
    fan._vector = lambda x, y, metric=None: (F(x), F(y))
    band.window_neighbours, fan._inside_ordinal = neighbours, inside
    fan._oriented_cross, fan._subturn, fan._subturn_boundary = cross, subturn, diagnostic
    fan._candidate_compare = comparator
    fan._preparation_budget, fan._work_budget, fan._BOX_REFINEMENT_CAP = prep_budget, search_budget, box
    fan._authority_id = lambda *args: "out_of_scope_prepared_value_gate"
    try:
        result = fan._certify_adaptive_density_fan_prepared(metric, ideals, oriented, q, records,
                   binding_reasons=(None,) * len(records), resource_height_cap=height)
        # Current counts наблюдаем из discarded tuple исходного search.
        text = authority_text(result, search_observation[0][3])
    except Exception as error:
        text = "ERROR:" + type(error).__name__ + ":" + str(error)
    budget_text = [f"{b.cap},{b.shell_probes},{b.order_steps}" for b in budgets]
    budget_text += ["_"] * (2 - len(budget_text))
    text += "\tBUDGETS:" + "#".join(budget_text) + "\tTRACE:" + "@".join(events)
    fields = (name, str(prep), str(search), str(box), str(height), str(q), str(orientation), str(throw), L, I,
              ",".join(map(str, X)), U, limit, vectors_text(boundaries), fraction(width), vectors_text(ideal),
              "#".join(map(record_text, records)) or "_")
    return "\t".join(fields), text, len(events)


# Наблюдаем discarded current-count tuple; search algorithms остаются исходными.
search_observation = []
OLD_SINGLE_SEARCH, OLD_ATLAS_SEARCH = fan._search, fan.search_global_height


def single_search(*args, **kwargs):
    result = OLD_SINGLE_SEARCH(*args, **kwargs)
    search_observation[:] = [result]
    return result


def atlas_search(*args, **kwargs):
    result = OLD_ATLAS_SEARCH(*args, **kwargs)
    search_observation[:] = [result]
    return result


fan._search, fan.search_global_height = single_search, atlas_search


def fixtures():
    rows = []
    def add(name, records, **kwargs):
        search_observation.clear()
        command, result, count = oracle_case(name, records, **kwargs)
        rows.append((command, result))
        return count
    local = (True, 1, envelope(-1), envelope(1))
    one = Piece(0, True, 1, envelope(-1), envelope(1), False, False, True, Pole.Y_ZERO)
    atlas_one = atlas.AtlasWindowPrepared((one,), 0, local)
    two_pieces = (Piece(0, True, 1, envelope(-1), envelope(1), False, True, True, Pole.Y_ZERO),
                  Piece(1, False, 1, envelope(-1), envelope(1), False, False, False, Pole.X_ZERO))
    atlas_two = atlas.AtlasWindowPrepared(two_pieces, 0, local)
    plans = [[], [local], [atlas_one], [atlas_two], [local, local], [local, atlas_one]]
    for n, plan in enumerate(plans):
        for box in (0, 1, 2, 96):
            for prep in (0, 1, 2, 65536):
                for search in (0, 1, 2, 131072):
                    add(f"caps-{n}-{box}-{prep}-{search}", plan, box=box, prep=prep, search=search)
    for n, plan in enumerate(plans[1:]):
        for height in (0, 1, 2, 7):
            for L in ("1", "0", "10111"):
                add(f"height-{n}-{height}-{L}", plan, height=height, L=L)
        count = add(f"complete-{n}", plan)
        for at in range(1, count + 1):
            add(f"throw-{n}-{at}", plan, throw=at)
    for box in (0, 1, 2, 96):
        for mode in ("true", "false", "ref", "other"):
            add(f"exhaust-{box}-{mode}", [local], box=box, I="0", limit=mode)
    for n, plan in enumerate(([local], [atlas_one], [atlas_two])):
        for width in (F(0), F(1, 100), F(1, 2)):
            for orientation in (-1, 1):
                for q in range(2, 7):
                    add(f"variants-{n}-{width}-{orientation}-{q}", plan, width=width, orientation=orientation, q=q)
    add("center-invalid", [(True, 1, envelope(0), envelope(1))])
    add("center-error-before-budget", [local], throw=1)
    add("center-envelope-error-before-budget", [local], throw=2)
    for box in (1, 2, 96):
        add(f"refine-success-{box}", [local], box=box, I="0111111111111")
    rng = random.Random(904353)
    for n in range(80):
        plan = rng.choice(plans[1:])
        add(f"random-{n}", plan, prep=rng.choice((0, 2, 65536)), search=rng.choice((0, 5, 131072)),
            box=rng.choice((0, 1, 2, 96)), height=rng.choice((0, 1, 5)), L=rng.choice(("1", "0", "101")),
            I=rng.choice(("1", "0", "01")), X=[rng.choice((0, 1))], U=rng.choice(("1", "0")),
            limit=rng.choice(("true", "false", "ref", "other")), throw=rng.choice((0, 1, 5, 20)))
    return rows


def recorded_caps():
    receipt = Path("D:/CFTUV-r3-density-corpus/bind_boundary_v2_receipt.json")
    paths = set()
    def visit(value):
        if isinstance(value, dict):
            for key, item in value.items():
                if key == "file" and isinstance(item, str) and item.endswith(".recs.xz"):
                    paths.add(item)
                else:
                    visit(item)
        elif isinstance(value, list):
            for item in value:
                visit(item)
    visit(json.loads(receipt.read_text(encoding="utf-8")))
    result = []
    for file in sorted(paths):
        with lzma.open(file, "rt", encoding="utf-8") as stream:
            data = json.load(stream)
        result.append({"file": file, "records": len(data["records"]), "box_caps": dict(Counter(
            row["input"]["settings"]["box_cap"] for row in data["records"]))})
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--emit-fixtures", action="store_true")
    parser.add_argument("--replay", type=Path)
    parser.add_argument("--out", type=Path, default=Path("D:/CFTUV-r3-density-corpus/fan-compile"))
    args = parser.parse_args()
    rows = fixtures()
    if args.emit_fixtures:
        for command, result in rows:
            print("IN\t" + command)
            print("OUT\t" + result)
        return
    if not args.replay:
        parser.error("--replay is required")
    args.out.mkdir(parents=True, exist_ok=True)
    payload = "\n".join(command for command, _ in rows) + "\n"
    (args.out / "replay.input.txt").write_text(payload, encoding="utf-8")
    (args.out / "oracle.stdout.txt").write_text("\n".join(result for _, result in rows) + "\n", encoding="utf-8")
    process = subprocess.Popen([str(args.replay)], stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    (args.out / "pids.json").write_text(json.dumps({"python": os.getpid(), "replay": process.pid}), encoding="utf-8")
    stdout, stderr = process.communicate(payload)
    (args.out / "replay.stdout.txt").write_text(stdout, encoding="utf-8")
    (args.out / "replay.stderr.txt").write_text(stderr, encoding="utf-8")
    actual = stdout.splitlines()
    mismatches = [{"row": n, "case": command.split("\t")[0], "expected": expected, "actual": result}
                  for n, ((command, expected), result) in enumerate(zip(rows, actual)) if expected != result]
    report = {"cases": len(rows), "actual_rows": len(actual), "exit": process.returncode,
              "mismatches": mismatches, "recorded_box_caps": recorded_caps(), "python": sys.version,
              "prepared_value_scope": True, "authority_id_and_decimal_repr_in_scope": False}
    (args.out / "differential.result.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps({"cases": len(rows), "actual_rows": len(actual), "exit": process.returncode,
                      "mismatches": len(mismatches), "out": str(args.out)}, indent=2))
    if mismatches:
        print(json.dumps(mismatches[0], indent=2))
    raise SystemExit(0 if not mismatches and len(rows) == len(actual) and process.returncode == 0 else 1)


if __name__ == "__main__":
    main()
