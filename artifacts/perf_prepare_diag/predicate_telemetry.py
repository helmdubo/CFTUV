"""Телеметрия КЛАССОВ ПРЕДИКАТОВ — раскладка walls.012 по будущим уровням.

Уровни целевой архитектуры (внешний аудит):
  localize -> canonicalize -> float64-filtered predicates
           -> checked int64/int128 fixed-point -> exact algebraic fallback

Каждый предикат ядра относится ровно к одному классу, и класс здесь ИЗМЕРЕН,
а не назначен:

  LATTICE_INT      `robust/predicates.py` — целочисленные определители на
                   решётке. Ширина операндов меряется, чтобы было видно,
                   поместится ли класс в int64/int128 fixed-point.
  SQRTSUM_SIGN     `SqrtSumV1.sign()` — знак суммы корней. Внутри уже есть
                   два уровня: интервальная оболочка (фильтр) и точное
                   сопряжение (дожим). Считаются оба раздельно.
  SQRTSUM_CANON    канонизация: `radical` / `squarefree_split` /
                   `prime_support` / `_prime_universe_from_q_values`. Это НЕ
                   предикат, а приведение к канонической форме, и именно оно
                   тянет целочисленную факторизацию.
  EXACT_ALGEBRA    sympy на пути `reference/` (компиляция оболочек, веер
                   плотности).

Ширины: INT64 (<2^63), INT128 (<2^127), BIGINT — по НАИБОЛЬШЕМУ целому,
которое предикат обязан удержать (числители и знаменатели, радиканды).
"""

from __future__ import annotations

import json
import sys
import time
from collections import defaultdict
from fractions import Fraction

import env  # noqa: F401

from cftuv_envelope import exact_sqrt_sum as ess
from cftuv_envelope.robust import predicates as robust_predicates

INT64 = 1 << 63
INT128 = 1 << 127

STATS = defaultdict(
    lambda: {
        "calls": 0,
        "seconds": 0.0,
        "int64": 0,
        "int128": 0,
        "bigint": 0,
        "max_bits": 0,
        "by_caller": defaultdict(int),
    }
)


def _width_bucket(magnitude: int) -> str:
    if magnitude < INT64:
        return "int64"
    if magnitude < INT128:
        return "int128"
    return "bigint"


def _record(name, magnitude, seconds, caller=""):
    row = STATS[name]
    row["calls"] += 1
    row["seconds"] += seconds
    row[_width_bucket(magnitude)] += 1
    bits = int(magnitude).bit_length()
    if bits > row["max_bits"]:
        row["max_bits"] = bits
    if caller:
        row["by_caller"][caller] += 1


def _caller(depth=2):
    frame = sys._getframe(depth)
    return f"{frame.f_code.co_filename.rsplit('/', 1)[-1]}:{frame.f_code.co_name}"


def _sqrtsum_magnitude(terms) -> int:
    magnitude = 1
    for radicand, coefficient in terms:
        magnitude = max(
            magnitude,
            radicand,
            abs(coefficient.numerator),
            coefficient.denominator,
        )
    return magnitude


# --------------------------------------------------------------------------
# LATTICE_INT
# --------------------------------------------------------------------------


def _wrap_lattice(name):
    original = getattr(robust_predicates, name)

    def wrapper(*args):
        started = time.perf_counter()
        result = original(*args)
        magnitude = 1
        for item in args:
            if isinstance(item, tuple):
                for value in item:
                    if isinstance(value, int):
                        magnitude = max(magnitude, abs(value))
                    elif isinstance(value, tuple):
                        for inner in value:
                            if isinstance(inner, int):
                                magnitude = max(magnitude, abs(inner))
            elif isinstance(item, int):
                magnitude = max(magnitude, abs(item))
        # Определитель удваивает ширину — храним квадрат величины.
        _record(f"LATTICE_INT:{name}", magnitude * magnitude,
                time.perf_counter() - started)
        return result

    setattr(robust_predicates, name, wrapper)


# --------------------------------------------------------------------------
# SQRTSUM_SIGN / SQRTSUM_CANON
# --------------------------------------------------------------------------


def _install():
    for name in ("orient2d", "cross_sign", "dot_sign", "on_segment",
                 "point_in_loop", "compare"):
        if hasattr(robust_predicates, name):
            _wrap_lattice(name)

    original_sign = ess.SqrtSumV1.sign

    def sign(self, *, filter_bits: int = 64):
        started = time.perf_counter()
        before = ess.SIGN_COUNTS["closed_by_conjugation"]
        rational_before = (
            ess.SIGN_COUNTS["closed_rational_zero"]
            + ess.SIGN_COUNTS["closed_rational_nonzero"]
        )
        result = original_sign(self, filter_bits=filter_bits)
        seconds = time.perf_counter() - started
        magnitude = _sqrtsum_magnitude(self.terms)
        if ess.SIGN_COUNTS["closed_by_conjugation"] > before:
            tier = "SQRTSUM_SIGN:exact_conjugation"
        elif (
            ess.SIGN_COUNTS["closed_rational_zero"]
            + ess.SIGN_COUNTS["closed_rational_nonzero"]
        ) > rational_before:
            tier = "SQRTSUM_SIGN:rational_only"
        else:
            tier = "SQRTSUM_SIGN:interval_filter"
        _record(tier, magnitude, seconds, _caller(2))
        return result

    ess.SqrtSumV1.sign = sign

    original_factorize = ess._factorize

    def factorize(n):
        started = time.perf_counter()
        result = original_factorize(n)
        _record("SQRTSUM_CANON:factorize", int(n),
                time.perf_counter() - started, _caller(2))
        return result

    ess._factorize = factorize

    original_radical = ess.SqrtSumV1.radical

    def radical(coefficient, radicand):
        started = time.perf_counter()
        result = original_radical(coefficient, radicand)
        frac = Fraction(radicand)
        _record(
            "SQRTSUM_CANON:radical",
            max(abs(frac.numerator), frac.denominator, 1),
            time.perf_counter() - started,
        )
        return result

    ess.SqrtSumV1.radical = staticmethod(radical)

    original_universe = ess._prime_universe_from_q_values

    def universe(q_values):
        started = time.perf_counter()
        result = original_universe(q_values)
        magnitude = 1
        for raw in q_values:
            q = Fraction(raw)
            magnitude = max(magnitude, abs(q.numerator) * q.denominator)
        _record("SQRTSUM_CANON:prime_universe", magnitude,
                time.perf_counter() - started, _caller(2))
        return result

    ess._prime_universe_from_q_values = universe
    from cftuv_envelope.wavefront import coverage as coverage_module
    from cftuv_envelope.wavefront import motorcycle as motorcycle_module
    from cftuv_envelope.wavefront import skeleton as skeleton_module

    for module in (coverage_module, motorcycle_module, skeleton_module):
        module._prime_universe_from_q_values = universe


_install()


def dump(tag):
    rows = []
    for name, row in sorted(
        STATS.items(), key=lambda item: -item[1]["seconds"]
    ):
        rows.append(
            {
                "class": name,
                "calls": row["calls"],
                "seconds": round(row["seconds"], 4),
                "int64": row["int64"],
                "int128": row["int128"],
                "bigint": row["bigint"],
                "max_operand_bits": row["max_bits"],
                "top_callers": sorted(
                    row["by_caller"].items(), key=lambda item: -item[1]
                )[:4],
            }
        )
    return {"tag": tag, "classes": rows}


def reset():
    STATS.clear()


def main():
    from counterfactual import PATH, variants
    from run_domain import bundle_from_field_snapshot, snapshot_and_request

    density = int(sys.argv[1]) if len(sys.argv) > 1 else 0
    payload, cases = variants()
    selected = frozenset(payload["raw"]["selected_edges"])
    output = {}
    for name in ("A_field", "B_v0_flat"):
        _, _, bundle = bundle_from_field_snapshot(
            PATH, vertex_override=cases[name]
        )
        rows = snapshot_and_request(bundle, selected, density=density)
        patch_id, domain_id, snapshot, request = rows[0]

        from cftuv_envelope.wavefront import conveyor_coverage, prepare_conveyor
        from cftuv.envelope_queue_export import build_queue_domain

        stages = {}
        reset()
        started = time.perf_counter()
        prepared = prepare_conveyor(snapshot, request)
        stages["QUEUE_PREPARE"] = {
            "wall_seconds": round(time.perf_counter() - started, 4),
            **dump("QUEUE_PREPARE"),
        }
        reset()
        started = time.perf_counter()
        coverage = conveyor_coverage(prepared, "0.45")
        stages["QUEUE_COVERAGE"] = {
            "wall_seconds": round(time.perf_counter() - started, 4),
            **dump("QUEUE_COVERAGE"),
        }
        reset()
        started = time.perf_counter()
        build_queue_domain(
            patch_id,
            domain_id,
            prepared,
            coverage,
            prepare_seconds=0.0,
            coverage_seconds=0.0,
        )
        stages["QUEUE_CONTOUR"] = {
            "wall_seconds": round(time.perf_counter() - started, 4),
            **dump("QUEUE_CONTOUR"),
        }
        output[name] = stages
        print(f"##### {name}")
        for stage, data in stages.items():
            print(f"  --- {stage}: wall {data['wall_seconds']}s")
            for item in data["classes"]:
                print(
                    f"      {item['class']:<36} calls={item['calls']:>6} "
                    f"{item['seconds']:>8.3f}s  int64={item['int64']:>6} "
                    f"int128={item['int128']:>5} bigint={item['bigint']:>5} "
                    f"maxbits={item['max_operand_bits']}"
                )
                for caller, count in item["top_callers"]:
                    print(f"          from {caller}: {count}")
    (env.OUT / f"predicate_telemetry_d{density}.json").write_text(
        json.dumps(output, ensure_ascii=False, indent=2), encoding="utf-8"
    )


if __name__ == "__main__":
    main()
