"""Какие ответы меняет прототипная подмена `proto_tight_lift` на ТЕКУЩИХ EXACT-доменах.

  PYTHONSAFEPATH=1 PYTHONPATH=kernel/src python artifacts/plan_not_compiled/proto_diff.py <dir> <density> [--only p1,p2]

Компилирует домен дважды (штатно и с подменой) и сравнивает по каждой угловой
спеке: счёт скрытых рёбер и наличие лифта. Печатает только домены, где ответ
различается, и по каждому различающемуся вееру — (selection H, штатный H/лифт,
подмена H/лифт, угол шагов в градусах, ЕСТЬ ЛИ у штатного пути отказ-кандидат).
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from cftuv_envelope import AnalysisSnapshotCodecV1, DecalRequestCodecV1
from cftuv_envelope.reference import angular, compile as compile_module

import proto_tight_lift as proto  # noqa: E402  (ставит подмену на импорте)

PATCHED_FEASIBLE = compile_module._density_ideal_is_subturn_feasible
PATCHED_LIFT_OK = angular._lift_count_is_feasible
ORIGINAL_FEASIBLE = proto.original_feasible
ORIGINAL_LIFT_OK = proto.original_lift_ok


def _set(patched: bool):
    compile_module._density_ideal_is_subturn_feasible = PATCHED_FEASIBLE if patched else ORIGINAL_FEASIBLE
    angular._lift_count_is_feasible = PATCHED_LIFT_OK if patched else ORIGINAL_LIFT_OK


def _specs(result):
    if result.compilation is None:
        return None
    out = {}
    for item in result.compilation.envelope_specs:
        if hasattr(item, "resolved_hidden_edge_count") and hasattr(item, "owner_sector_id"):
            lift = getattr(item, "evaluation_subturn_count_lift", None)
            out[item.envelope_spec_id.value] = (
                item.resolved_hidden_edge_count,
                None if lift is None else (lift.source_hidden_edge_count, lift.effective_hidden_edge_count),
            )
    return out


def main():
    root = Path(sys.argv[1])
    density = int(sys.argv[2])
    only = None
    if "--only" in sys.argv:
        only = set(sys.argv[sys.argv.index("--only") + 1].split(","))
    manifest = json.loads((root / "manifest.json").read_text(encoding="utf-8"))
    for patch in manifest["domains"]:
        if only is not None and patch not in only:
            continue
        base = root / f"patch_{patch}"
        snapshot = AnalysisSnapshotCodecV1.loads((base / "analysis_snapshot.json").read_bytes())
        request = DecalRequestCodecV1.loads((base / f"decal_request_d{density}.json").read_bytes())
        _set(False)
        before = compile_module.compile_reference_envelopes(snapshot, request)
        _set(True)
        after = compile_module.compile_reference_envelopes(snapshot, request)
        b, a = _specs(before), _specs(after)
        if before.outcome is not after.outcome or b != a:
            changed = [] if a is None or b is None else [(k[-6:], b.get(k), a.get(k)) for k in sorted(set(a) | set(b)) if b.get(k) != a.get(k)]
            print(f"p{patch}: {before.outcome.value} -> {after.outcome.value}; changed specs={len(changed)} of {len(a or {})}; sample={changed[:3]}")
    _set(False)


if __name__ == "__main__":
    main()
