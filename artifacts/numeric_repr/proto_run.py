"""Прогон одного домена с runtime-шимами `proto_shims` против baseline.

    python proto_run.py <patch> <density> <shims|none> <baseline.json> [<out.json>]

`shims` — список через запятую из `sign,compare,mul,floorcache` либо `none`. Печатает одну
строку JSON: секунды (после прогрева импортов малым доменом), равенство ОТВЕТА (все отпечатки
`gate`), равенство статей бюджета и счётчиков знаков, список различающихся ключей.
"""

from __future__ import annotations

import json
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import gate  # noqa: E402
import pool_sweep  # noqa: E402


def main():
    patch, density = int(sys.argv[1]), int(sys.argv[2])
    shims = [] if sys.argv[3] == "none" else sys.argv[3].split(",")
    baseline = json.loads(Path(sys.argv[4]).read_text(encoding="utf-8"))
    pool_sweep.init_worker(quiet=True)
    gate.compute_row(100, density)  # прогрев импортов
    import proto_shims

    installed = proto_shims.install(shims)
    started = time.perf_counter()
    row = gate.compute_row(patch, density)
    wall = time.perf_counter() - started
    base = baseline["runs"][str(density)]["domains"][str(patch)]
    answer_diff = sorted(
        key
        for key in set(row["answer"]) | set(base["answer"])
        if row["answer"].get(key) != base["answer"].get(key)
    )
    budget_keys = [key for key in base["price"] if key.startswith("EXACT_WORK")]
    budget_equal = all(row["price"].get(k) == base["price"].get(k) for k in budget_keys)
    record = {
        "patch": patch,
        "density": density,
        "shims": installed,
        "seconds": round(wall, 2),
        "baseline_pool_seconds": base["price"]["seconds"],
        "skeleton_seconds": row["price"].get("stage_seconds", {}).get("SKELETON"),
        "answer_equal": not answer_diff,
        "answer_diff_keys": answer_diff,
        "budget_articles_equal": budget_equal,
        "sign_counts_equal": row["price"].get("sign_counts") == base["price"].get("sign_counts"),
        "sign_counts": row["price"].get("sign_counts"),
        "sign_counts_baseline": base["price"].get("sign_counts"),
    }
    print(json.dumps(record, ensure_ascii=False))
    if len(sys.argv) > 5:
        Path(sys.argv[5]).write_text(json.dumps(record, ensure_ascii=False), encoding="utf-8")


if __name__ == "__main__":
    main()
