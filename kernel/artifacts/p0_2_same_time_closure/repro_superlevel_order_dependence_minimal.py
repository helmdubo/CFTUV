"""Minimal frozen P0-2 D repro for one exact partial-source polygon."""

from __future__ import annotations

import json
import sys

from wavefront_cases import partial_source_corpus
from repro_superlevel_order_dependence import _run


def main() -> None:
    polygon = dict(partial_source_corpus())["ell_12_source_edges_0_1"]
    split_first = _run(polygon, edge_first=False)
    edge_first = _run(polygon, edge_first=True)
    payload = {
        "outcome": "SUPERLEVEL_ORDER_DEPENDENCE_FOUND",
        "case": "ell_12_source_edges_0_1",
        "split_first": split_first,
        "edge_first": edge_first,
    }
    if split_first == edge_first:
        payload["outcome"] = "REPRO_DID_NOT_DIVERGE"
    json.dump(payload, sys.stdout, indent=2, sort_keys=True)
    sys.stdout.write("\n")


if __name__ == "__main__":
    main()
