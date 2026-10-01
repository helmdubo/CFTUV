"""ПРОТОТИП (только рантайм-подмена, исходники продукта не тронуты): считать
веер «ровно на пределе Delta_max с иррациональными скрытыми направлениями»
НЕОСУЩЕСТВИМЫМ, чтобы существующий лифт счёта (`_evaluation_density_spec`,
`EVALUATION_GEOMETRY_SUBTURN_COUNT_LIFTED_V1`) поднял H на единицу.

  PYTHONSAFEPATH=1 PYTHONPATH=kernel/src python artifacts/plan_not_compiled/proto_tight_lift.py <dir> <density> [--only p1,p2] [--prepare]

Печатает исход компиляции (и, с --prepare, исход `prepare_conveyor`) и ПЕРВЫЙ
стек исключения проверяющей стороны, если лифт отвергнут независимым
верификатором. Это не предложение патча, а ответ на вопрос «дойдёт ли расчёт до
конца, если назвать этот случай лифтом».
"""
from __future__ import annotations

import json
import sys
import time
import traceback
from fractions import Fraction
from pathlib import Path

from cftuv_envelope import AnalysisSnapshotCodecV1, DecalRequestCodecV1
from cftuv_envelope.reference import adaptive_density_fan as fan
from cftuv_envelope.reference import angular, compile as compile_module
from cftuv_envelope.reference.direction_binding import has_rational_density_support_direction

original_feasible = compile_module._density_ideal_is_subturn_feasible
TRACE = {"tight_lifts": 0}


def tight_aware(metric, ideal, q):
    if not original_feasible(metric, ideal, q):
        return False
    covectors = fan._covectors(metric, ideal)
    if not fan._subturn_boundary(metric, covectors[0], covectors[1], q):
        return True
    # ровно на пределе: осуществимо только если ВСЕ скрытые направления рациональны
    if all(has_rational_density_support_direction(metric, ideal[i]) for i in range(1, len(ideal) - 1)):
        return True
    TRACE["tight_lifts"] += 1
    return False


compile_module._density_ideal_is_subturn_feasible = tight_aware

original_lift_ok = angular._lift_count_is_feasible


def lift_ok(lift, hidden_count):
    # независимый верификатор: тугой случай на predecessor считаем неосуществимым
    # (прототипная подмена: в продукте это была бы отдельная именованная власть).
    threshold_turn = Fraction(hidden_count + 1, lift.max_subturn_q)
    if threshold_turn < 1:
        cos2 = Fraction(lift.evaluation_turn_cosine_squared.numerator, lift.evaluation_turn_cosine_squared.denominator)
        if angular._compare_turn_cos_squared(cos2, threshold_turn) == 0:
            return False
    return original_lift_ok(lift, hidden_count)


angular._lift_count_is_feasible = lift_ok


def main():
    root = Path(sys.argv[1])
    density = int(sys.argv[2])
    only = None
    if "--only" in sys.argv:
        only = set(sys.argv[sys.argv.index("--only") + 1].split(","))
    prepare = "--prepare" in sys.argv
    manifest = json.loads((root / "manifest.json").read_text(encoding="utf-8"))
    from collections import Counter

    tally = Counter()
    for patch in manifest["domains"]:
        if only is not None and patch not in only:
            continue
        base = root / f"patch_{patch}"
        snapshot = AnalysisSnapshotCodecV1.loads((base / "analysis_snapshot.json").read_bytes())
        request = DecalRequestCodecV1.loads((base / f"decal_request_d{density}.json").read_bytes())
        TRACE["tight_lifts"] = 0
        started = time.perf_counter()
        line = ""
        try:
            if prepare:
                from cftuv_envelope.wavefront import prepare_conveyor

                from cftuv_envelope.wavefront import conveyor_coverage

                prepared = prepare_conveyor(snapshot, request)
                outcome = getattr(prepared.outcome, "value", str(prepared.outcome))
                line = f"prepare={outcome} detail={getattr(prepared, 'detail', '')}"
                if outcome == "EXACT":
                    coverage = conveyor_coverage(prepared, "0.25")
                    outcome = "cov:" + coverage.outcome.value
                    line += f" coverage={coverage.outcome.value} faces={len(getattr(coverage, 'faces', ()) or ())}"
            else:
                result = compile_module.compile_reference_envelopes(snapshot, request)
                outcome = result.outcome.value
                lifted = 0
                if result.compilation is not None:
                    lifted = sum(
                        1
                        for item in result.compilation.envelope_specs
                        if getattr(item, "evaluation_subturn_count_lift", None) is not None
                    )
                line = f"compile={outcome} lifted_fans={lifted} {result.diagnostics[0].message[:160] if result.diagnostics else ''}"
        except Exception as exc:
            outcome = "EXC:" + type(exc).__name__
            tb = traceback.extract_tb(exc.__traceback__)[-3:]
            line = f"{type(exc).__name__}: {str(exc)[:160]} @ " + " <- ".join(f"{Path(f.filename).name}:{f.lineno}" for f in reversed(tb))
        tally[outcome] += 1
        print(f"p{patch} d{density} {time.perf_counter() - started:.2f}s tight_lifts={TRACE['tight_lifts']} {line}")
    print("TALLY", dict(tally))


if __name__ == "__main__":
    main()
