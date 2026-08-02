# P0-2 phase B receipt

Base: `8d93e79b6affca4b5b1032e91b1514d1a21f3b94`  
Snapshot gate: `bb003b6`  
Phase A: `58d2e82`  
Phase B: `b42b6fd`

## Measured reachability

- Exact corpus: 63 cases.
- Duplicate canonical `(time, point)` records before B: 11 cases, one excess
  record in each.
- Original kinds: 10 `EDGE + SPLIT`, one `SPLIT + SPLIT`.
- Cross at A: 4 exact-time events enqueued during packet application; residual
  event sizes after the six packets `[2, 0, 2, 0, 0, 0]`; residual processing
  unit occurrences 2.
- After B: `duplicate_exact_time_point_nodes == 0` and
  `mixed_kind_exact_time_point_nodes == 0` on all 63 cases.

## B invariants

- Accumulator identity is exact `(time, point)` plus transitive incidence
  connectivity by shared participant keys. A synthetic same-place case proves
  that disconnected participant components remain separate.
- Merged participants equal the frozen pre-card union in every one of the 11
  cases. `converging_vertices` is the cardinality of the internal union of
  converged runtime vertex IDs, not a sum stored in the public digest.
- A merged component is `MULTIWAY`; `kinds` is the unique value-sorted set of
  original kinds. The `SPLIT + SPLIT` case therefore records `['SPLIT']`.
- Unmerged `node_record` has no `kinds` member and all 52 nonduplicate semantic
  digests remain byte-identical to A. The 11 intentional digest changes are
  listed explicitly beside the preserved A oracle in the P0 proof test.
- The full proof-obligation projection, face partition, and coverage at alpha
  `1/4`, `1`, and `3` remain byte-identical to the frozen A projection for all
  11 cases. Original per-participant incidences preserve the zero-length face
  contour records without reintroducing duplicate skeleton nodes.
- Legacy raw-emission counters and the `23 / 17 / 1` partial-source summary are
  unchanged. `FROZEN_DIGESTS` is untouched; the named corpus has no duplicate
  exact-time point nodes.

## Executed kind-consumer trace

Production searches found three reads of node-kind semantics:

1. `digest.node_record` emits `kind=MULTIWAY` plus `kinds`, and refuses an
   unannotated MULTIWAY node as `MULTIWAY_NODE_KINDS_UNAVAILABLE`.
2. `faces.build_faces` expands stored original incidences for MULTIWAY and
   refuses missing incidence data as
   `MULTIWAY_NODE_INCIDENCE_UNAVAILABLE`.
3. `superlevel.accumulate_nodes` flattens already-merged originals explicitly
   when composing a connected component.

Test-side SPLIT filters in event-queue and motorcycle differential helpers
recognize `SPLIT` inside MULTIWAY `kinds`; the differential kind counter unfolds
original kinds instead of counting MULTIWAY as a candidate event.

## Verification

- `test_wavefront_same_time_closure.py`, `test_wavefront_proof_obligations.py`,
  `test_wavefront_event_queue.py`, `test_wavefront_motorcycle_graph.py`:
  **358 passed** in 113.26 s.
- `test_wavefront_faces.py`, `test_wavefront_coverage.py`: **185 passed** in
  49.67 s.
- An earlier combined eight-file command reached the 304 s process limit and
  was terminated without a pytest verdict; it is not counted as evidence.
- `skeleton.py`: 1950 lines; public imports remain available through the same
  `cftuv_envelope.wavefront.skeleton` aliases.
