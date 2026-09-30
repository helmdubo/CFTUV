"""КТО именно спрашивает `Fraction.__hash__`. Адрес, а не догадка.

Счётчик карточки ставится на счётчиках, а не на секундах (урок ошибки №19),
поэтому первое, что обязано быть измерено, — не «сколько секунд стоит хэш», а
«сколько РАЗ и ИЗ КАКОГО МЕСТА он спрошен». Подмена `Fraction.__hash__`
считающей обёрткой сохраняет значение хэша (она зовёт оригинал), поэтому
ответ домена от замера не двигается ни битом; двигается только цена.

Адрес берётся из кадра ВЫЗЫВАЮЩЕГО: `sys._getframe(1)` в момент вызова хэша —
это либо строка ядра, спросившая словарь, либо `<C>` (кортеж хэшируется
интерпретатором, и тогда виден кадр того, кто хэшировал кортеж).

Прогон из каталога `artifacts/perf_prepare_diag`:
    PYTHONPATH=. python3 ../exact_identity_intern/hash_callers.py <out.json> \
        [snapshot.json] [density] [alpha]
"""

from __future__ import annotations

import fractions
import json
import sys
import time

import env  # noqa: F401

from cftuv_envelope import exact_sqrt_sum as canon

_ORIGINAL_HASH = fractions.Fraction.__hash__
_CALLERS: dict[str, int] = {}
_TOTAL = [0]


def _site(frame) -> str:
    return (
        f"{frame.f_code.co_filename.rsplit('/', 1)[-1]}"
        f":{frame.f_lineno}:{frame.f_code.co_name}"
    )


def _counting_hash(self):
    _TOTAL[0] += 1
    frame = sys._getframe(1)
    key = _site(frame)
    # `<string>` — это сгенерированный `__hash__` frozen-датакласса: он хэширует
    # кортеж полей, и Fraction внутри попадает сюда через C-уровень кортежа.
    # Такой адрес ничего не называет, поэтому поднимаемся до первого кадра,
    # который лежит в файле, а не в exec-строке.
    while frame.f_code.co_filename == "<string>" and frame.f_back is not None:
        frame = frame.f_back
        key = f"<dataclass>/{_site(frame)}"
    _CALLERS[key] = _CALLERS.get(key, 0) + 1
    return _ORIGINAL_HASH(self)


def main() -> None:
    out = sys.argv[1]
    which = sys.argv[2] if len(sys.argv) > 2 else "wall_2_001_snapshot.json"
    density = int(sys.argv[3]) if len(sys.argv) > 3 else 0
    alpha = sys.argv[4] if len(sys.argv) > 4 else "0.45"

    from run_domain import bundle_from_field_snapshot, snapshot_and_request
    from snapshot_bmesh import load_snapshot
    from cftuv.envelope_queue_export import run_queue_domain

    path = env.SNAPSHOTS / which
    payload = load_snapshot(path)
    selected = frozenset(payload["raw"]["selected_edges"])
    _, _, bundle = bundle_from_field_snapshot(path)
    rows = snapshot_and_request(bundle, selected, density=density)

    results = {}
    for patch_id, domain_id, snapshot, request in rows:
        _CALLERS.clear()
        _TOTAL[0] = 0
        canon.reset_factorization_memory()
        canon.reset_unbudgeted_work()
        fractions.Fraction.__hash__ = _counting_hash
        started = time.perf_counter()
        try:
            prepared, domain = run_queue_domain(
                patch_id, domain_id, snapshot, request, alpha
            )
        finally:
            fractions.Fraction.__hash__ = _ORIGINAL_HASH
        seconds = time.perf_counter() - started
        top = sorted(_CALLERS.items(), key=lambda item: -item[1])[:25]
        results[f"{which}:d{density}:a{alpha}:patch{patch_id}"] = {
            "seconds_with_counting_hash": round(seconds, 3),
            "fraction_hash_calls": _TOTAL[0],
            "outcome": f"{domain.preparation_outcome}/{domain.coverage_outcome}",
            "top_callers": [{"site": site, "calls": n} for site, n in top],
        }
        print(json.dumps(results, ensure_ascii=False, indent=1), flush=True)
    with open(out, "w", encoding="utf-8") as handle:
        json.dump(results, handle, ensure_ascii=False, indent=2, sort_keys=True)


if __name__ == "__main__":
    main()
