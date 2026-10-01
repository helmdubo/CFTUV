"""Сколько стоит старт пула: время до «готов» всех воркеров (вне Blender).

    python artifacts/host_export_parallel/pool_start_probe.py [workers]
"""

from __future__ import annotations

import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
for entry in (str(ROOT), str(ROOT / "kernel" / "src")):
    sys.path.insert(0, entry)

from cftuv.envelope_domain_pool import DomainPool  # noqa: E402


def main() -> None:
    workers = int(sys.argv[1]) if len(sys.argv) > 1 else 8
    pool = DomainPool(workers)
    started = time.perf_counter()
    pool.ensure_started()
    print(f"{workers} workers ready in {time.perf_counter() - started:.2f} s")
    pool.close()


if __name__ == "__main__":
    main()
