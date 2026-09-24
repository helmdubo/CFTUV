"""САМИ ЧИСЛА: битовые длины радикандов SqrtSum, чарта и законов прихода.

Пишет `coefficient_sizes.json`: по варианту меша и стадии — распределение
битовых длин целых, которые точная арифметика обязана ФАКТОРИЗОВАТЬ, плюс
размеры чарт-базиса/матрицы Грама и коэффициентов законов прихода.
"""

from __future__ import annotations

import json
import statistics
import sys
import time
from fractions import Fraction

import env  # noqa: F401
from counterfactual import PATH, variants
from run_domain import bundle_from_field_snapshot, snapshot_and_request
from measure_numbers import metric_facts

import cftuv_envelope as kernel
from cftuv_envelope import exact_sqrt_sum as ess

FACTORIZATIONS = []


def _instrument():
    original = ess._factorize

    def spy(n):
        started = time.perf_counter()
        result = original(n)
        FACTORIZATIONS.append(
            {
                "n": n,
                "bits": int(n).bit_length(),
                "seconds": time.perf_counter() - started,
                "largest_prime_bits": (
                    max(int(p).bit_length() for p in result) if result else 0
                ),
                "factor_count": len(result),
            }
        )
        return result

    ess._factorize = spy
    # Модуль вызывает `_factorize` по имени модуля — подмены достаточно.


_instrument()


def digest(rows):
    if not rows:
        return {}
    bits = [item["bits"] for item in rows]
    seconds = [item["seconds"] for item in rows]
    heavy = sorted(rows, key=lambda item: -item["seconds"])[:5]
    return {
        "calls": len(rows),
        "distinct_radicands": len({item["n"] for item in rows}),
        "radicand_bits_max": max(bits),
        "radicand_bits_median": int(statistics.median(bits)),
        "radicand_bits_min": min(bits),
        "largest_prime_factor_bits_max": max(
            item["largest_prime_bits"] for item in rows
        ),
        "total_seconds": round(sum(seconds), 4),
        "worst": [
            {
                "bits": item["bits"],
                "seconds": round(item["seconds"], 4),
                "largest_prime_bits": item["largest_prime_bits"],
                "n": str(item["n"]),
            }
            for item in heavy
        ],
    }


def arrival_law_sizes(prepared):
    """Коэффициенты законов прихода: скорость, нормаль, опора — в дробях."""

    numerator_bits = []
    denominator_bits = []
    q_bits = []
    for region in prepared.regions:
        for law in getattr(region, "laws", ()) or ():
            for name in ("speed_squared", "normal_x", "normal_y", "offset"):
                value = getattr(law, name, None)
                if value is None:
                    continue
                frac = Fraction(value)
                numerator_bits.append(abs(frac.numerator).bit_length())
                denominator_bits.append(frac.denominator.bit_length())
        polygon = getattr(region, "polygon", None)
        for loop in (getattr(polygon, "loops", ()) or ()) if polygon else ():
            for value in getattr(loop, "edge_speed_squared", ()) or ():
                frac = Fraction(value)
                q_bits.append(
                    (frac.numerator * frac.denominator).bit_length()
                )
    out = {}
    if numerator_bits:
        out["law_coefficients"] = {
            "count": len(numerator_bits),
            "numerator_bits_max": max(numerator_bits),
            "numerator_bits_median": int(statistics.median(numerator_bits)),
            "denominator_bits_max": max(denominator_bits),
            "denominator_bits_median": int(statistics.median(denominator_bits)),
        }
    if q_bits:
        out["edge_q_radicands"] = {
            "count": len(q_bits),
            "bits_max": max(q_bits),
            "bits_median": int(statistics.median(q_bits)),
        }
    return out


def run(name, override, selected, density):
    _, _, bundle = bundle_from_field_snapshot(PATH, vertex_override=override)
    rows = snapshot_and_request(bundle, selected, density=density)
    patch_id, domain_id, snapshot, request = rows[0]
    _, facts = metric_facts(snapshot, patch_id, domain_id)

    from cftuv_envelope.wavefront import conveyor_coverage, prepare_conveyor
    from cftuv.envelope_queue_export import build_queue_domain

    ess.reset_factorization_memory()
    stages = {}

    FACTORIZATIONS.clear()
    started = time.perf_counter()
    prepared = prepare_conveyor(snapshot, request)
    stages["QUEUE_PREPARE"] = {
        "seconds": round(time.perf_counter() - started, 4),
        "outcome": prepared.outcome.value,
        "factorization": digest(list(FACTORIZATIONS)),
    }

    FACTORIZATIONS.clear()
    started = time.perf_counter()
    coverage = conveyor_coverage(prepared, "0.45")
    stages["QUEUE_COVERAGE"] = {
        "seconds": round(time.perf_counter() - started, 4),
        "outcome": coverage.outcome.value,
        "factorization": digest(list(FACTORIZATIONS)),
    }

    FACTORIZATIONS.clear()
    started = time.perf_counter()
    domain = build_queue_domain(
        patch_id,
        domain_id,
        prepared,
        coverage,
        prepare_seconds=0.0,
        coverage_seconds=0.0,
    )
    stages["QUEUE_CONTOUR"] = {
        "seconds": round(time.perf_counter() - started, 4),
        "faces": len(domain.faces),
        "factorization": digest(list(FACTORIZATIONS)),
    }

    return {
        "variant": name,
        "density": density,
        "metric": facts,
        "arrival_laws": arrival_law_sizes(prepared),
        "counters": dict(prepared.counters),
        "stages": stages,
    }


def main():
    density = int(sys.argv[1]) if len(sys.argv) > 1 else 0
    wanted = sys.argv[2:] or None
    payload, cases = variants()
    selected = frozenset(payload["raw"]["selected_edges"])
    out_path = env.OUT / "coefficient_sizes.json"
    results = {}
    if out_path.exists():
        results = json.loads(out_path.read_text(encoding="utf-8"))
    for name, override in cases.items():
        if wanted and name not in wanted:
            continue
        record = run(name, override, selected, density)
        results[f"walls_012:{name}:d{density}"] = record
        print(json.dumps(record, ensure_ascii=False, default=str))
    out_path.write_text(
        json.dumps(results, ensure_ascii=False, indent=2, default=str),
        encoding="utf-8",
    )


if __name__ == "__main__":
    main()
