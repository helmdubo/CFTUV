"""SHADOW of `RadicalSumV1` by `RadicalSumV2` (slice S2) on the tracked seed of the effective-alpha corpus: the executable part of the S2 evidence.

`tools/radical_v2_shadow.py` replays the pairs of `tests/data/native_effective_alpha_kernel_suite.recs.xz` on the python side with every V1 operation
recorded (operands, result), then checks each one in V2 by value, and replays the pairs again with the float prefilter reading the V2 form. The full
runs (kernel suite, owner's field corpus) are the tool's `kernel` / `field` commands; this test keeps the claim and the tool alive in CI.
"""

from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
for _path in (ROOT / "kernel" / "src", ROOT / "tools"):
    if str(_path) not in sys.path:
        sys.path.insert(0, str(_path))

import native_effective_alpha_corpus as nec  # noqa: E402
import radical_v2_shadow as shadow  # noqa: E402

SEED = ROOT / "tests" / "data" / "native_effective_alpha_kernel_suite.recs.xz"


def test_every_v1_operation_of_the_seed_has_the_same_value_in_v2_and_the_prefilter_does_not_move():
    recorder, report = shadow.shadow_pairs(nec.read_document(SEED))
    assert report["replay_vs_recorded_mismatch"] == 0, "the recorder changed what it records"
    analysis = shadow.analyse(shadow.loaded_of(recorder))
    kinds = analysis["kinds"]
    assert {"add", "sub", "mul", "inverse", "signum", "measure"} <= set(kinds), sorted(kinds)
    differing = {kind: row["value_differs"] for kind, row in kinds.items() if row["value_differs"]}
    assert not differing, (differing, analysis["examples"][:3])
    # The prefilter's binary64 measures of the V2 form differ in the last bits for some values, and no pair's counters or answer move.
    assert report["v2_floats"] == {"pairs_with_other_counts": 0, "pairs_with_other_answer": 0, "counter_moves": {}}


def test_the_v2_form_of_every_recorded_value_is_one_form_under_random_operation_orders():
    recorder, _ = shadow.shadow_pairs(nec.read_document(SEED))
    tally = shadow.probe(shadow.loaded_of(recorder), routes=4, limit=400)
    assert tally["values"] > 0 and tally["v2_value_differs"] == 0 and tally["v2_form_differs"] == 0, tally
