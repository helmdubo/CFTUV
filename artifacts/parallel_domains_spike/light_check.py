"""Запись домена (`EnvelopeQueueDomainV1` без `preparation`), пришедшая из воркера.

    PYTHONPATH=. python3 ../parallel_domains_spike/light_check.py <pool_pickle.json> <pkl_dir>

ЧИСТЫЙ процесс грузит `patchN.domain_light.pkl` (их записал воркер при
`pool_sweep.py --pickle-dir`) и сверяет отпечаток геометрии/счётчиков/меты с
эталоном, снятым в воркере до сериализации.
"""

from __future__ import annotations

import json
import pickle
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import pool_sweep  # noqa: E402

pool_sweep.init_worker(quiet=True)  # те же импорты/пути, что у воркера

record = json.loads(Path(sys.argv[1]).read_text(encoding="utf-8"))
out = {}
for row in record["passes"][0]["rows"]:
    patch_id = row["patch_id"]
    blob = (Path(sys.argv[2]) / f"patch{patch_id}.domain_light.pkl").read_bytes()
    started = time.perf_counter()
    domain = pickle.loads(blob)
    seconds = time.perf_counter() - started
    got = pool_sweep._fingerprints(domain)
    want = row["pickle_probe"]["expected_fp"]
    out[patch_id] = {
        "bytes": len(blob),
        "loads_seconds_fresh_process": round(seconds, 4),
        "fingerprint_equal": got == want,
        "preparation_is_none": domain.preparation is None,
    }
print(json.dumps(out, indent=1))
