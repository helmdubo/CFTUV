"""Коэффициенты законов прихода ДО и ПОСЛЕ рескейла + радиканды q + alpha."""

from __future__ import annotations

import json
import statistics
import sys
from fractions import Fraction

import env  # noqa: F401
from counterfactual import PATH, variants
from run_domain import bundle_from_field_snapshot, snapshot_and_request

from cftuv_envelope.wavefront import conveyor as conveyor_module
from cftuv_envelope import exact_sqrt_sum as ess

LAWS = []
QSETS = []


def _bits(value):
    frac = Fraction(value)
    return abs(frac.numerator).bit_length(), frac.denominator.bit_length()


def _install():
    original_law = conveyor_module._read_arrival_law

    def law_spy(name, normal, constant, speed_squared, density_metric=None):
        law, rescaled, issue = original_law(
            name, normal, constant, speed_squared, density_metric
        )
        if law is not None:
            LAWS.append(
                {
                    "name": name,
                    "rescaled": rescaled,
                    "normal_x": _bits(law.normal_x),
                    "normal_y": _bits(law.normal_y),
                    "constant": _bits(law.constant),
                    "speed_squared": _bits(law.speed_squared),
                }
            )
        return law, rescaled, issue

    conveyor_module._read_arrival_law = law_spy

    original_q = ess._prime_universe_from_q_values

    def q_spy(q_values):
        rows = []
        for raw in q_values:
            q = Fraction(raw)
            if q == 0:
                continue
            rows.append((q.numerator * q.denominator).bit_length())
        QSETS.append({"count": len(rows), "bits": rows})
        return original_q(q_values)

    ess._prime_universe_from_q_values = q_spy
    # Потребители связывают имя на импорте (`from ..exact_sqrt_sum import ...`),
    # поэтому подмены модуля-владельца мало: правим и три места связывания.
    from cftuv_envelope.wavefront import coverage as coverage_module
    from cftuv_envelope.wavefront import motorcycle as motorcycle_module
    from cftuv_envelope.wavefront import skeleton as skeleton_module

    for module in (coverage_module, motorcycle_module, skeleton_module):
        module._prime_universe_from_q_values = q_spy


_install()


def summarise_bits(rows):
    if not rows:
        return {}
    return {
        "count": len(rows),
        "max": max(rows),
        "median": int(statistics.median(rows)),
        "min": min(rows),
    }


def main():
    density = int(sys.argv[1]) if len(sys.argv) > 1 else 0
    payload, cases = variants()
    selected = frozenset(payload["raw"]["selected_edges"])
    out = {}
    for name, override in cases.items():
        LAWS.clear()
        QSETS.clear()
        _, _, bundle = bundle_from_field_snapshot(PATH, vertex_override=override)
        rows = snapshot_and_request(bundle, selected, density=density)
        patch_id, domain_id, snapshot, request = rows[0]
        from cftuv_envelope.wavefront import conveyor_coverage, prepare_conveyor

        ess.reset_sign_counts()
        prepared = prepare_conveyor(snapshot, request)
        prepare_signs = dict(ess.SIGN_COUNTS)
        ess.reset_sign_counts()
        coverage = conveyor_coverage(prepared, "0.45")
        coverage_signs = dict(ess.SIGN_COUNTS)
        all_q = [value for item in QSETS for value in item["bits"]]
        record = {
            "variant": name,
            "outcome": prepared.outcome.value,
            "lattice_scale": prepared.counter("CONVEYOR_LATTICE_SCALE"),
            "lattice_alpha": str(coverage.lattice_alpha),
            "lattice_alpha_bits": _bits(Fraction(str(coverage.lattice_alpha))),
            "alpha": str(coverage.alpha),
            "laws_total": len(LAWS),
            "laws_rescaled": sum(1 for item in LAWS if item["rescaled"]),
            "law_coefficient_bits": {
                "as_written": summarise_bits(
                    [
                        value
                        for item in LAWS
                        if not item["rescaled"]
                        for field in ("normal_x", "normal_y", "constant", "speed_squared")
                        for value in item[field]
                    ]
                ),
                "after_rescale": summarise_bits(
                    [
                        value
                        for item in LAWS
                        if item["rescaled"]
                        for field in ("normal_x", "normal_y", "constant", "speed_squared")
                        for value in item[field]
                    ]
                ),
            },
            "sqrt_sum_q_radicand_bits": summarise_bits(all_q),
            "sqrt_sum_operations": len(QSETS),
            "sign_counts_prepare": prepare_signs,
            "sign_counts_coverage": coverage_signs,
            "laws": LAWS[:12],
        }
        out[name] = record
        print(json.dumps(record, ensure_ascii=False))
    (env.OUT / f"law_sizes_d{density}.json").write_text(
        json.dumps(out, ensure_ascii=False, indent=2), encoding="utf-8"
    )


if __name__ == "__main__":
    main()
