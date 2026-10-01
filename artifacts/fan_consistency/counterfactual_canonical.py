"""Контрфакт (прототип правила, продукт НЕ правится): точно-прямой сырой угол идёт ТЕМ ЖЕ каноническим путём,
что и восстановленный.

  PYTHONSAFEPATH=1 python artifacts/fan_consistency/counterfactual_canonical.py <mesh> <density> [patch,patch...]

В памяти подменяется `_canonical_angle.canonical_reflex_excess_restoration`: для интервала, уже ТОЧНО равного
каноническому отношению (1/2), возвращается восстановление с нулевым отклонением, а не `None`. Тогда ядро сам
записывает сертификат восстановления и (где сырой веер на решётке карты неосуществим) власть канонического
подшага — ровно как для угла 90.0000015°. Всё остальное — штатный код. Цель: показать, какие числа вееров даёт
правило «решает канонический угол, а не шум привязки к решётке» и не ломает ли оно подготовку/покрытие/меш.

РЕЗУЛЬТАТ (mesh `2`, patch 1, d2; 12 углов): КОМПИЛЯЦИЯ плана даёт H=1 у всех 12 углов — три бывших лифтованных
(v33/v34/v29) получают восстановление с нулевым отклонением и власть канонического веера (unbound ordinal
supports), остальные девять прежние. Дальше стоп в `GeometryContext.build`:
`REFERENCE_CANONICAL_SUBTURN_FAN_INVALID` — «canonical subturn fan authority stands on a fan that already
satisfies the guarantee on source supports». Проверяющий власти читает осуществимость сырого веера не на той
геометрии, на которой её решил лифт (почему именно — не раскопано). Вывод: правило достижимо на уровне
компилятора, но требует согласованной правки проверяющего; без неё подготовка отказывает именованно.
"""
from __future__ import annotations

import json
import sys
from collections import Counter
from fractions import Fraction
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import _paths  # noqa: E402

_paths.add_kernel_paths()

import fanlib  # noqa: E402
from cftuv_envelope import AnalysisSnapshotCodecV1, DecalRequestCodecV1, _canonical_angle as ca  # noqa: E402
from cftuv_envelope.wavefront import conveyor_coverage, prepare_conveyor  # noqa: E402

_original = ca.canonical_reflex_excess_restoration


def _prototype(interval):
    found = _original(interval)
    if found is not None:
        return found
    for relation, canonical in ca.CANONICAL_REFLEX_EXCESS_RELATIONS:
        if ca._deviation_over_pi(interval, canonical) == 0:
            return ca.CanonicalAngleRestorationV1(relation, canonical, Fraction(0))
    return None


ca.canonical_reflex_excess_restoration = _prototype
from cftuv_envelope.reference import common as _common  # noqa: E402

_common.canonical_reflex_excess_restoration = _prototype  # проверяющий контекста читает закон по имени


def main() -> None:
    mesh, density = sys.argv[1], int(sys.argv[2])
    manifest = json.loads((_paths.mesh_dir(mesh) / "manifest.json").read_text(encoding="utf-8"))
    patches = sorted(manifest["domains"], key=int)
    if len(sys.argv) > 3:
        patches = [p for p in patches if p in sys.argv[3].split(",")]
    hist: Counter = Counter()
    examples: dict = {}
    outcomes: Counter = Counter()
    for patch in patches:
        base = _paths.mesh_dir(mesh) / f"patch_{patch}"
        snap = AnalysisSnapshotCodecV1.loads((base / "analysis_snapshot.json").read_bytes())
        req = DecalRequestCodecV1.loads((base / f"decal_request_d{density}.json").read_bytes())
        try:
            prep = prepare_conveyor(snap, req)
        except Exception as exc:  # noqa: BLE001
            outcomes[f"EXCEPTION:{type(exc).__name__}:{str(exc)[:80]}"] += 1
            continue
        outcomes[prep.outcome.value + (":" + prep.detail[:240] if prep.outcome.value != "EXACT" else "")] += 1
        if prep.compilation is None or prep.outcome.value != "EXACT":
            continue
        coverage = conveyor_coverage(prep, "0.45")
        outcomes["coverage:" + coverage.outcome.value] += 1
        view = fanlib.DomainView(snap, prep)
        auth = {a.envelope_spec_id for a in view.comp.canonical_subturn_fan_authorities}
        for spec in view.specs:
            vertex, prev, nxt = view.corner_neighbours(spec)
            dot, _c, _, _, ang = view.turn(view.eval_xy, vertex, prev, nxt)
            lift = getattr(spec, "evaluation_subturn_count_lift", None)
            key = (
                "EVAL_EXACT" if dot == 0 else ("EVAL_ABOVE" if abs(ang) > 90.0 else "EVAL_BELOW"),
                f"restored={spec.selection_certificate_id in view.restorations}",
                f"canon_fan={spec.envelope_spec_id in auth}",
                f"specH={spec.resolved_hidden_edge_count}",
                f"lift={lift.lift_law.name[-22:] if lift else '-'}",
            )
            hist[key] += 1
            examples.setdefault(key, (int(patch), fanlib.vertex_index(vertex)))
    print(f"{mesh} d{density} PROTOTYPE canonical-for-exact: outcomes={dict(outcomes)}")
    for key, count in sorted(hist.items()):
        print(f"  {count:4d}  {' | '.join(key)}   e.g. patch {examples[key][0]} v{examples[key][1]}")


main()
