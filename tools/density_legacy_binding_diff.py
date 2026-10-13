"""Actual Python B(w)/legacy full-fan differential с typed callback fixtures.

Prepared значения сравниваются точно. Expr admission, Decimal repr/quantum,
authority ID и direction_binding wrapper остаются вне этого gate.
"""
from __future__ import annotations

import argparse
from collections import Counter
from fractions import Fraction as F
from decimal import Decimal
import json
import os
from pathlib import Path
import random
import subprocess
import sys
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "kernel/src"))
from cftuv_envelope.reference import adaptive_density_fan as fan, adaptive_density_atlas as atlas
from cftuv_envelope.reference import adaptive_density_band as band
from cftuv_envelope.reference.density_binding_types import _PROVEN_PREDICATES
from cftuv_envelope.contracts.analysis import TurnOrientation
from cftuv_envelope.numeric import CertifiedDecimalIntervalV1 as Env, IntervalEndpointKind as Kind

OLD_PREP = fan._preparation_budget


class CallbackError(ValueError):
    pass


def fraction(value):
    value = F(value)
    return f"{value.numerator}/{value.denominator}"


def vector_text(value):
    return ",".join(map(fraction, value))


def vectors_text(values):
    return ";".join(map(vector_text, values)) or "_"


def envelope(lower, upper=None, error=F(0), lower_kind=Kind.CLOSED, upper_kind=Kind.CLOSED):
    value = object.__new__(Env)
    numeric = lambda value: value if isinstance(value, Decimal) else F(value)
    for name, item in (("lower", numeric(lower)), ("upper", numeric(lower if upper is None else upper)),
                       ("lower_kind", lower_kind), ("upper_kind", upper_kind),
                       ("absolute_error_bound", F(error))):
        object.__setattr__(value, name, item)
    return value


def envelope_text(value):
    kind = lambda k: "C" if k is Kind.CLOSED else "O" if k is Kind.OPEN else "X"
    return "~".join((fraction(value.lower), fraction(value.upper), kind(value.lower_kind),
                     kind(value.upper_kind), fraction(value.absolute_error_bound)))


def single(lower=-2, upper=2, use_x=True, sign=1):
    return use_x, sign, envelope(lower), envelope(upper)


def record_text(record):
    if type(record) is atlas.AtlasWindowPrepared:
        return "A"
    return "S!" + ",".join((str(int(record[0])), str(record[1]),
                           envelope_text(record[2]), envelope_text(record[3])))


def certificate(vector=(1, 0), proven=True):
    return SimpleNamespace(bound_primitive_integer_vector=vector,
                           proven_predicates=_PROVEN_PREDICATES if proven else frozenset())


def input_certificate_text(value):
    if value is None:
        return "N"
    return ",".join(map(str, (*value.bound_primitive_integer_vector,
                             int(value.proven_predicates == _PROVEN_PREDICATES))))


def output_certificate_text(value):
    if value is None:
        return "CERT:N"
    return "CERT:" + ";".join((",".join(map(str, value.bound_primitive_integer_vector)),
        envelope_text(value.ideal_window_lower_slope_envelope), envelope_text(value.ideal_window_upper_slope_envelope),
        fraction(value.certified_window_width_lower_bound), "P=" + str(int(value.proven_predicates == _PROVEN_PREDICATES))))


class Harness:
    def __init__(self, ideal, throw, U, X, I, score, signs):
        self.ideal, self.throw, self.score = ideal, throw, score
        self.events, self.counters = [], [0] * 4
        self.patterns = (list(map(int, U)), X, list(map(int, I)), signs)

    def next(self, slot):
        pattern = self.patterns[slot]
        value = pattern[self.counters[slot] % len(pattern)]
        self.counters[slot] += 1
        return value

    def call(self, prefix, compute, show=str):
        if self.throw and len(self.events) + 1 == self.throw:
            self.events.append(prefix + ":!CallbackError:refused")
            raise CallbackError("refused")
        try:
            result = compute()
        except Exception as error:
            self.events.append(prefix + ":!" + type(error).__name__ + ":" + str(error))
            raise
        self.events.append(prefix + ":" + show(result))
        return result

    def vector(self, x, y, metric=None):
        return self.call(f"V:{x},{y}", lambda: (F(x), F(y)), vector_text)

    def subturn(self, metric, left, right, q):
        return self.call(f"U:{q}:{vector_text(left)}|{vector_text(right)}", lambda: bool(self.next(0)), lambda v: str(int(v)))

    def score_value(self, ordinal, vector):
        if self.score == "asc":
            return F(vector[1])
        if self.score == "desc":
            return F(-vector[1])
        if self.score == "mod":
            return F((abs(vector[1]) + 2 * abs(vector[0])) % 3)
        if self.score == "gram":
            dot = sum(a * b for a, b in zip(vector, self.ideal[ordinal]))
            return dot * dot / sum(a * a for a in vector)
        return F(0)

    def objective(self, metric, ideal, ordinal, vector):
        return self.call(f"O:{ordinal}:" + ",".join(map(str, vector)), lambda: self.score_value(ordinal, vector), fraction)

    def sign(self, value, metric):
        return self.call("S:" + fraction(value), lambda: self.next(3) if self.patterns[3] is not None else int(value > 0) - int(value < 0))

    def cross(self, metric, left, right):
        return self.call(f"X:{vector_text(left)}|{vector_text(right)}", lambda: F(self.next(1)), fraction)

    def triple(self, ideal, ordinal):
        return self.call(f"N:{ordinal}", lambda: (ideal[ordinal - 1], ideal[ordinal], ideal[ordinal + 1]),
                         lambda v: "|".join(map(vector_text, v)))

    def inside(self, metric, candidate, previous, center, following):
        return self.call("I:" + "|".join(map(vector_text, (candidate, previous, center, following))),
                         lambda: bool(self.next(2)), lambda v: str(int(v)))

    def slope(self, candidate, use_x, metric):
        return self.call(f"T:{vector_text(candidate)}:{int(use_x)}",
                         lambda: F(candidate[1 if use_x else 0], candidate[0 if use_x else 1]), fraction)

    def install(self):
        fan._vector, fan._subturn, fan._objective = self.vector, self.subturn, self.objective
        fan._sign, fan._oriented_cross, fan._inside_ordinal = self.sign, self.cross, self.inside
        fan._slope, band.window_triple = self.slope, self.triple


def traced_width_record(record, harness):
    if type(record) is atlas.AtlasWindowPrepared:
        return record
    # Fraction(...) для B(w) снимает subclass; W возникает только при финальном
    # upper.lower-lower.upper. Decimal.__sub__ выполняется в actual host context.
    original = record[3].lower
    base = Decimal if isinstance(original, Decimal) else F
    class TracedEndpoint(base):
        def __sub__(self, other):
            return harness.call(f"W:{fraction(self)}|{fraction(other)}",
                                lambda: base.__sub__(self, other), fraction)
    upper = record[3]
    traced = envelope(upper.lower, upper.upper, upper.absolute_error_bound, upper.lower_kind, upper.upper_kind)
    object.__setattr__(traced, "lower", TracedEndpoint(original))
    return (*record[:3], traced)


def oracle_case(op, name, records, *, certs=(), ideal=None, cap=65536, ordinal=1, orientation=1,
                q=5, throw=0, U="1", X=None, I="1", score="tie", signs=None, width=F(1)):
    if X is None:
        X = [orientation]
    if ideal is None:
        size = max(len(certs), ordinal if op == "CERT" else 0, 1)
        ideal = [(F(1), F(-2)), *[(F(1), F(0)) for _ in range(size)], (F(1), F(2))]
    ideal = tuple(ideal)
    harness = Harness(ideal, throw, U, X, I, score, signs)
    harness.install()
    budgets = []
    def budget_factory(cap_arg=None):
        budget = OLD_PREP(cap)
        budgets.append(budget)
        return budget
    fan._preparation_budget = budget_factory
    oriented = TurnOrientation.CCW_IN_OWNER_PATCH_ORIENTATION if orientation == 1 else TurnOrientation.CW_IN_OWNER_PATCH_ORIENTATION
    width_reply = (records[0][3].lower - records[0][2].upper
                   if op == "CERT" and type(records[0]) is not atlas.AtlasWindowPrepared else F(0))
    try:
        if op == "B":
            text = "BOUND:" + str(fan._legacy_authority_budget(width))
        elif op == "CERT":
            text = output_certificate_text(fan._legacy_density_certificate(None, ideal, ordinal, q, traced_width_record(records[0], harness)))
        else:
            text = "VALID:" + str(int(fan._legacy_full_fan_valid(None, ideal, oriented, q, records, certs)))
    except Exception as error:
        text = "ERROR:" + type(error).__name__ + ":" + str(error)
    budget = f"{budgets[0].cap},{budgets[0].shell_probes},{budgets[0].order_steps}" if budgets else "_"
    text += "\tBUDGET:" + budget + "\tTRACE:" + "@".join(harness.events)
    fields = (op, name, str(cap), str(ordinal), str(orientation), str(q), str(throw), U,
              ",".join(map(str, X)), I, score, "actual" if signs is None else ",".join(map(str, signs)),
              vectors_text(ideal), "#".join(map(record_text, records)) or "_",
              "#".join(map(input_certificate_text, certs)) or "_", fraction(width), fraction(width_reply))
    return "\t".join(fields), text, len(harness.events)


def certificate_fixtures(add):
    for b in (1, 2, 3, 7, 65536, 65537, 2**80 + 1):
        for offset in (-1, 0, 1):
            add("B", f"square-{b}-{offset}", [], width=F(1, b*b + offset or 1))
    for chart in (False, True):
        for sign in (-1, 1):
            for lower, upper in ((-5, 5), (F(-1, 8), F(1, 8)), (F(1, 4), F(1, 2)), (2, 2), (3, 2)):
                record = single(lower, upper, chart, sign)
                for cap in (0, 1, 2, 65536):
                    for score in ("tie", "asc", "desc", "mod", "gram"):
                        add("CERT", f"cert-{chart}-{sign}-{lower}-{upper}-{cap}-{score}", [record], cap=cap, score=score)
    for score in ("tie", "asc", "desc", "mod"):
        record = single(-5, 5)
        count = add("CERT", f"trace-{score}", [record], score=score)
        for at in range(1, count + 1):
            add("CERT", f"throw-{score}-{at}", [record], score=score, throw=at)
    for U in ("0", "01", "10", "10111"):
        add("CERT", f"subturn-{U}", [single(-5, 5)], U=U)
    for signs in ([0], [-1], [1], [-1, 0], [0, 1, 0, -1]):
        add("CERT", f"counterfactual-sign-{signs}", [single(-5, 5)], signs=signs)
    add("CERT", "upfront-large-refusal", [single(0, F(1, 65537**2))])
    add("CERT", "atlas-none-no-ledger", [atlas.AtlasWindowPrepared((), 0, single())], cap=0, throw=1)
    record = (True, -1, envelope(-2, -1, F(1, 4), Kind.OPEN), envelope(1, 2, F(1, 3), upper_kind=Kind.OPEN))
    add("CERT", "envelope-fields-cloned", [record], score="gram")
    rounded_record = single(Decimal('-0.5'), Decimal('0.62345678901234567890123456789'))
    assert F(rounded_record[3].lower - rounded_record[2].upper) != F(rounded_record[3].lower) - F(rounded_record[2].upper)
    add("CERT", "actual-decimal-rounded-certificate-width", [rounded_record])
    decimal_record = single(Decimal('-0.125'), Decimal('0.12499999999999999999999999999'))
    exact_width = F(decimal_record[3].lower) - F(decimal_record[2].upper)
    rounded_width = F(decimal_record[3].lower - decimal_record[2].upper)
    assert fan._legacy_authority_budget(exact_width) == 3
    assert fan._legacy_authority_budget(rounded_width) == 2
    add("CERT", "exact-B-before-rounded-width-refusal", [decimal_record], cap=2)
    add("CERT", "exact-B-before-rounded-width-success", [decimal_record], cap=3)


def full_fixtures(add):
    atlas_record = atlas.AtlasWindowPrepared((), 0, single())
    for certs in ((), (None,), (certificate(),), (certificate(proven=False), certificate()), (None, None),
                  (certificate(), None), (None, certificate()), (certificate(), certificate())):
        for size in range(4):
            for I in ("1", "0", "01"):
                for U in ("1", "0", "10"):
                    for X in ([1], [0], [-1], [1, 0]):
                        add("FULL", f"full-{len(certs)}-{input_certificate_text(certs[0]) if certs else 'empty'}-{size}-{I}-{U}-{X}",
                            [single()] * size, certs=certs, I=I, U=U, X=X)
    for n, records in enumerate(([single()], [atlas_record], [single(0, 2)], [single(-2, 0)],
                                  [single(), single()], [single()], [single(), single(), single()])):
        certs = (certificate(),) if n < 4 else (certificate(), certificate())
        count = add("FULL", f"full-trace-{n}", records, certs=certs)
        for at in range(1, count + 1):
            add("FULL", f"full-throw-{n}-{at}", records, certs=certs, throw=at)
    for orientation in (-1, 1):
        for q in range(2, 7):
            add("FULL", f"full-orientation-q-{orientation}-{q}", [single()], certs=(certificate(),), orientation=orientation, q=q)
    add("FULL", "slope-zero-denominator", [single()], certs=(certificate((0, 1)),))
    add("FULL", "noncanonical-second-vector-refuses-first", [single(), single()],
        certs=(certificate(proven=False), certificate()), throw=2)


def fixtures():
    rows = []
    def add(op, name, records, **kwargs):
        command, result, count = oracle_case(op, name, records, **kwargs)
        rows.append((command, result))
        return count
    certificate_fixtures(add)
    full_fixtures(add)
    rng = random.Random(739270)
    for n in range(180):
        lower = F(rng.randrange(-20, 10), rng.randrange(1, 10))
        upper = lower + F(rng.randrange(1, 20), rng.randrange(1, 10))
        add("CERT", f"random-{n}", [single(lower, upper, rng.choice((False, True)), rng.choice((-1, 1)))],
            cap=rng.choice((0, 1, 2, 65536)), U=rng.choice(("1", "01", "1011")),
            score=rng.choice(("tie", "asc", "desc", "mod", "gram")), throw=rng.choice((0, 0, 1, 4, 10)))
    return rows


def replay_rows(args, rows):
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
    mismatches = [{"row": n, "case": command.split("\t")[1], "expected": expected, "actual": result}
                  for n, ((command, expected), result) in enumerate(zip(rows, actual)) if expected != result]
    report = {"cases": len(rows), "actual_rows": len(actual), "exit": process.returncode,
              "mismatches": mismatches, "python": sys.version,
              "operations": dict(Counter(command.split("\t")[0] for command, _ in rows)),
              "prepared_callback_scope": True, "authority_id_and_decimal_repr_in_scope": False}
    (args.out / "differential.result.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps({**report, "mismatches": len(mismatches), "out": str(args.out)}, indent=2))
    if mismatches:
        print(json.dumps(mismatches[0], indent=2))
    return not mismatches and len(rows) == len(actual) and process.returncode == 0


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--emit-fixtures", action="store_true")
    parser.add_argument("--replay", type=Path)
    parser.add_argument("--out", type=Path, default=Path("D:/CFTUV-r3-density-corpus/legacy-binding"))
    args = parser.parse_args()
    rows = fixtures()
    if args.emit_fixtures:
        for command, result in rows:
            print("IN\t" + command)
            print("OUT\t" + result)
        return
    if not args.replay:
        parser.error("--replay is required")
    raise SystemExit(0 if replay_rows(args, rows) else 1)


if __name__ == "__main__":
    main()
