"""Заморозить абсолютные P0-3 projections после принятого P0-2b."""

from __future__ import annotations

import json
from pathlib import Path

import test_wavefront_weighted_wall_differential as differential


HERE = Path(__file__).resolve().parent
OUTPUT = HERE / "p0_3_post_p0_2b_absolute_digests.json"
P0_2B_PRODUCT_COMMIT = "852059a34062fd5c5dd18b1833c2908904a08362"
P0_2B_KERNEL_SOURCE_TREE = "d87b96e08fc54353ba330689da1895acd114064a"
P0_3_ACCEPTANCE_COMMIT = "2851e85053fc1047fc938a51dcc3b5d8dceed88b"
P0_3_DIFFERENTIAL_RECEIPT_BLOB = "ade0ca7448b305a9a2a565ffd89697260d49693b"


def main() -> None:
    by_case = {}
    by_search = {}
    for case in differential.CORPUS:
        readings = differential._readings(case.name)
        search_digests = {
            search.value: differential._projection_sha256(reading)
            for search, reading in readings.items()
        }
        if len(set(search_digests.values())) != 1:
            raise RuntimeError(
                "WF_WEIGHTED_WALL_DIFFERENTIAL_MISMATCH "
                f"case={case.name} digests={search_digests}"
            )
        by_search[case.name] = search_digests
        by_case[case.name] = next(iter(search_digests.values()))

    receipt = {
        "schema": "cftuv.p0_3.post_p0_2b_absolute_digests.v1",
        "projection_schema": "TYPED_CANONICAL_JSON_V1",
        "projection_axes": list(differential.AXES),
        "case_count": len(by_case),
        "searches": ["MOTORCYCLE", "EXHAUSTIVE"],
        "case_projection_sha256": by_case,
        "search_projection_sha256": by_search,
        "corpus_sha256": differential._digest_map_sha256(by_case),
        "p0_2b_product_commit": P0_2B_PRODUCT_COMMIT,
        "p0_2b_kernel_source_tree": P0_2B_KERNEL_SOURCE_TREE,
        "p0_3_acceptance_commit": P0_3_ACCEPTANCE_COMMIT,
        "p0_3_differential_receipt_blob_at_acceptance": (
            P0_3_DIFFERENTIAL_RECEIPT_BLOB
        ),
        "runtime_identity_excluded": ["proof_obligation.vertex_ids"],
        "canonical_types": [
            "bool",
            "counter",
            "dict",
            "event_time_v1",
            "fraction",
            "int",
            "list",
            "none",
            "str",
            "tuple",
        ],
        "reproduce": (
            "$env:PYTHONPATH='kernel/src;kernel/tests'; python "
            "artifacts/kernel_audit_exact_proof/"
            "generate_p0_3_post_p0_2b_absolute_digests.py"
        ),
    }
    OUTPUT.write_text(
        json.dumps(receipt, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )


if __name__ == "__main__":
    main()
