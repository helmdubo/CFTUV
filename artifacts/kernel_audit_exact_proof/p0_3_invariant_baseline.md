# P0-3 invariant baseline

- schema: `cftuv.p0_3.invariant_baseline.v1`
- exact base: `8d93e79b6affca4b5b1032e91b1514d1a21f3b94`
- case universe: 63 entries from `named_corpus()` plus
  `partial_source_corpus()`
- canonical projection SHA-256:
  `cf2d03aface1500d17c09ea29b210681fa9c03b640fa0201222024d2da46f634`

The projection was measured before adding the P0-3 differential corpus.  For
each case it contains the skeleton outcome, `semantic_digest`, SHA-256 of the
complete sorted counter map, proof status, and the count of every proof
disposition.  Its executable per-case authority is the frozen 63-case oracle
in `kernel/tests/test_wavefront_proof_obligations.py`; the projection above is
the hash of those values as produced by the exact base.

Summary:

- outcomes: 46 `EXACT`, 17 `WAVEFRONT_LEFT_UNRESOLVED`;
- proof: 39 `COMPLETE`, 24 `INCOMPLETE`;
- obligations: 420 total = 354 discharged, 36 fallback, 26 survived,
  4 accepted with unproven span;
- old invariant axes: semantic digests, outcomes, and all existing counter
  maps are frozen before the new test-only corpus.

Reproduction environment: CPython 3.10, working directory `kernel`,
`PYTHONPATH=src;tests`.  No `kernel/src` file is part of this snapshot commit.
