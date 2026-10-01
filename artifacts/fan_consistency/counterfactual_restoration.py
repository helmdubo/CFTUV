"""Контрфакт: те же домены, но сырой угол хоста СДВИНУТ в допуск намерения (restoration срабатывает).

  PYTHONSAFEPATH=1 python artifacts/fan_consistency/counterfactual_restoration.py <mesh> <density> <patch> [shift_decimal]

Единственное изменение входа — у сертификатов с `reflex_excess_over_pi == [1/2, 1/2]` интервал заменяется
на `1/2 + shift` (по умолчанию 5e-7 доли пи = 1.6e-6 рад, ВНУТРИ AUTHOR_ANGULAR_ERROR = 7e-6 рад);
`phi_over_pi` сдвигается на то же. Геометрия, решётка и привязка не тронуты. Вопрос: даёт ли ПУТЬ
ВОССТАНОВЛЕННОГО угла (канонический веер) одинаковое число скрытых опор на тех же углах, где путь
«сырой угол ровно прямой» разошёлся из-за шума решётки. Продукт не правится — вход подменяется в памяти.

РЕЗУЛЬТАТ: отрицательный. Подмена интервала хоста отвергается плановым контролем ДО селектора
(`REFERENCE_ANGLE_SUPPORT_CERTIFICATE_MISMATCH`): сертификат угла обязан содержать истинный угол опор, а он точен.
Честный контрфакт требует сдвига ГЕОМЕТРИИ источника, а не записи; см. `counterfactual_canonical.py`.
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import _paths  # noqa: E402

_paths.add_kernel_paths()

import fanlib  # noqa: E402
from cftuv_envelope import AnalysisSnapshotCodecV1, DecalRequestCodecV1  # noqa: E402
from cftuv_envelope.wavefront import prepare_conveyor  # noqa: E402
from decimal import Decimal  # noqa: E402


def shifted_snapshot_bytes(raw: bytes, shift: Decimal) -> tuple[bytes, int]:
    data = json.loads(raw.decode("utf-8"))
    changed = 0
    for cert in data["reflex_angle_certificates"]:
        payload = cert["measure_payload"]
        interval = payload["reflex_excess_over_pi"]
        if Decimal(interval["lower"]) == Decimal(interval["upper"]) == Decimal("0.5"):
            for key in ("reflex_excess_over_pi", "phi_over_pi"):
                node = payload[key]
                for end in ("lower", "upper"):
                    node[end] = str(Decimal(node[end]) + shift)
            changed += 1
    return json.dumps(data, ensure_ascii=False).encode("utf-8"), changed


def main() -> None:
    mesh, density, patch = sys.argv[1], int(sys.argv[2]), sys.argv[3]
    shift = Decimal(sys.argv[4]) if len(sys.argv) > 4 else Decimal("0.0000005")
    base = _paths.mesh_dir(mesh) / f"patch_{patch}"
    raw, changed = shifted_snapshot_bytes((base / "analysis_snapshot.json").read_bytes(), shift)
    snap = AnalysisSnapshotCodecV1.loads(raw)
    req = DecalRequestCodecV1.loads((base / f"decal_request_d{density}.json").read_bytes())
    prep = prepare_conveyor(snap, req)
    print(f"{mesh} patch {patch} d{density} shift={shift} changed_certs={changed} outcome={prep.outcome.value} {prep.detail[:160]}")
    if prep.compilation is None or prep.outcome.value != "EXACT":
        return
    view = fanlib.DomainView(snap, prep)
    auth = {a.envelope_spec_id for a in view.comp.canonical_subturn_fan_authorities}
    hist: dict = {}
    for spec in view.specs:
        vertex, prev, nxt = view.corner_neighbours(spec)
        _d, _c, _, _, e_ang = view.turn(view.eval_xy, vertex, prev, nxt)
        lift = getattr(spec, "evaluation_subturn_count_lift", None)
        key = (
            "EVAL_EXACT" if _d == 0 else ("EVAL_ABOVE" if abs(e_ang) > 90.0 else "EVAL_BELOW"),
            f"restored={spec.selection_certificate_id in view.restorations}",
            f"canon_fan={spec.envelope_spec_id in auth}",
            f"specH={spec.resolved_hidden_edge_count}",
            f"lift={lift.lift_law.name[-22:] if lift else '-'}",
        )
        hist.setdefault(key, []).append(fanlib.vertex_index(vertex))
    for key, vertices in sorted(hist.items()):
        print(f"  {len(vertices):3d}  {' | '.join(key)}   e.g. v{vertices[0]}")


main()
