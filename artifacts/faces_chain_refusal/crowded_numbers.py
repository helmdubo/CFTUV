"""Числа ветки `crowded` на patch 17 `building`: что отбирает граница 4, что — площадь.

Запуск из корня репозитория:
  PYTHONSAFEPATH=1 PYTHONPATH="artifacts/perf_prepare_diag;artifacts/faces_chain_refusal" \
      python artifacts/faces_chain_refusal/crowded_numbers.py 17 2,3,4

Перебор комбинаций повторён здесь поштучно (продукт печатает только итоги по
детали отказа): для каждой комбинации — дефекты парности ВСЕГО разбиения, дефекты
парности ТОЛЬКО среди граней ветки и их соседей, и тождество площади. Отдельно —
дефекты парности у неизменных граней (должен быть 0, иначе граница 4 была бы
шумом, а не проверкой).
"""
from __future__ import annotations

import sys
from itertools import product
from pathlib import Path

# Каталог стенда с `env` и `big_scene` лежит рядом: ставим его в путь сами, чтобы
# запуск не зависел от переменных окружения.
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "perf_prepare_diag"))
sys.path.insert(0, str(Path(__file__).resolve().parent))

import env  # noqa: E402,F401
from cftuv_envelope import exact_sqrt_sum as canon


def main():
    patch_id = int(sys.argv[1])
    densities = [int(x) for x in sys.argv[2].split(",")]
    from cftuv_envelope.wavefront import faces as F
    from cftuv_envelope.wavefront import prepare_conveyor
    from cftuv_envelope.wavefront.sqrt_sum import SqrtSumV1
    from diag17c import stage

    snap, mk = stage(patch_id)
    original = F.settle_crowded
    captured = {}

    def spy(pending, fixed_total, fixed_segments, polygon_area):
        captured["row"] = (pending, fixed_total, fixed_segments, polygon_area)
        return original(pending, fixed_total, fixed_segments, polygon_area)

    F.settle_crowded = spy
    for d in densities:
        canon.reset_factorization_memory()
        captured.clear()
        prepared = prepare_conveyor(snap, mk(d))
        region = prepared.regions[0]
        print(f"=== d{d}: outcome {prepared.outcome.value} faces={len(region.partition.faces)}"
              f" counters={[c for c in prepared.counters if 'CROWDED' in c[0]]}")
        if "row" not in captured:
            print("  ветка не входилась")
            continue
        pending, fixed_total, fixed_segments, polygon_area = captured["row"]
        print(f"  неизменных сегментов {len(fixed_segments)}; "
              f"дефектов парности только среди неизменных: {F.pairing_defects(fixed_segments)}")
        for item in pending:
            print(f"  грань {item.key}: путей {item.paths}, пригодных {len(item.variants)}, "
                  f"оборвано {item.exhausted}")
        target = SqrtSumV1.rational(polygon_area)
        for number, combo in enumerate(product(*(item.variants for item in pending))):
            segs = fixed_segments + [s for v in combo for s in v.segments]
            total = fixed_total
            for v in combo:
                total = total + v.face.doubled_area
            diff = total - target
            print(f"  комбинация {number}: дефектов парности {F.pairing_defects(segs)}, "
                  f"площадь {'СХОДИТСЯ' if diff.is_zero else 'не сходится'}")


if __name__ == "__main__":
    main()
