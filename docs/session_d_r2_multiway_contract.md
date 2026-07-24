# D-R2 exact atomic ownerless point-contact contract

Status: `PRODUCT_OWNER_ACCEPTED`

Contract ID: `ATOMIC_OWNERLESS_POINT_CONTACT_V1`

Accepted base:
`361102a6539ffbbe6fa8957a04bf12a9bae42bd8`

## Decision

One `SameAlphaInteractionBatchId` is one atomic semantic batch. Candidate
order, component order, arrival-model storage order, IDs, and pairwise
execution order do not select its result.

The v1 supported multiway subset is deliberately narrow. A connected
same-alpha participant hypergraph is admitted by
`ATOMIC_OWNERLESS_POINT_CONTACT_V1` only when all of the following are
proved exactly:

1. every admitted equality locus is zero-dimensional: it has one or more
   exact event anchors and no positive-length segment;
2. every anchor is an accepted `PointContactRecordV1` fact whose participating
   envelope instances and FrontComponents agree with the mutual-arrival
   certificates;
3. the positive-area interiors of every pair of participating
   InteractionComponents are disjoint;
4. the pre-batch positive-area union is already the exact RawCoverage union;
5. all front-reading and construction provenance is complete;
6. no ownership decision is required to preserve the existing matter.

The atomic result is the identity on all positive-area contributions. It does
not apply Policy B half-plane clipping. It emits the exact point anchors as
ownerless equality loci, records
`InteractionCoverageEffect.ATOMIC_OWNERLESS_POINT_CONTACT_IDENTITY`, and sets
`creates_new_matter = false`.

Consequently:

- no matter is created, removed, moved, or repaired;
- no gap is introduced;
- the resolved positive-area union equals RawCoverage exactly;
- all participant permutations produce the same semantic projection;
- a shared InteractionComponent may participate in several point contacts in
  one batch without imposing a pair order.

## Before, at, and after

Before the first exact mutual-arrival alpha, the batch emits no interaction
application or equality locus and leaves contributions unchanged.

At the exact alpha, a qualifying point-only hypergraph is recorded atomically.
Its positive-area result remains unchanged and its equality anchors are
ownerless.

After the exact alpha, the same identity result remains legal only while the
current exact inputs still prove all admission conditions above. The
first-contact anchors remain frozen. If positive-area overlap or a
positive-length equality locus appears, this contract no longer applies.

## Named unsupported boundary

The existing outcome `MULTIWAY_INTERACTION_POLICY_UNPROVEN` remains mandatory
when any connected same-alpha participant hypergraph has:

- a positive-length equality locus;
- positive-area participant overlap;
- coincident arrival laws without a unique exact equality locus;
- incomplete or conflicting participant/provenance facts;
- an exact predicate that cannot decide admission;
- any need for ownership to define interaction geometry.

The existing synthetic three-front meeting has three positive-length equality
loci. It remains named unsupported. This decision does not define a general
multiway Policy B partition.

## Pairwise sequencing is forbidden

`A` versus `B` followed by the winner versus `C` is not an implementation of
this contract. In the reproduced field case, bypassing the current multiway
guard and feeding the two pair proofs to existing Policy B reduces the
resolved area to zero, emits
`INTERACTION_POLICY_B_PARTITION_UNPROVEN`, and produces order-dependent
contribution digests. The supported result is instead an atomic point-contact
identity operation over immutable pre-batch inputs.

## Reproduced field case

The accepted portable `building.002` fixture produces:

- three InteractionComponents and three arrival models;
- two exact mutual-arrival proofs at `Integer(0)`;
- one shared central component;
- two distinct exact anchors and no equality segments;
- pairwise positive-area intersection `0`, `0`, `0`;
- RawCoverage digest
  `622e1f6eec09e64bc1294c37643af19f630086005f9af21f10ac4cd6ed0e987a`;
- exact RawCoverage area
  `Rational(122766786560, 373821260323)`.

The two anchors are:

```text
(Rational(232403250106, 373821260323),
 Rational(240403797009, 373821260323))

(Rational(2651330823508, 373821260323),
 Rational(-452766086928, 373821260323))
```

The case satisfies the admitted subset. Its contractual result is exact
identity preservation plus two ownerless point equality loci in one atomic
same-alpha batch.

## Scope boundary

This gate defines the accepted semantic contract and its executable fixtures.
It does not change Envelope geometry, ownership, station/UV, tessellation,
host code, or the interaction resolver implementation files outside the
D-R2 allowlist. Until a later implementation card consumes this contract, the
current resolver continues to fail closed with
`MULTIWAY_INTERACTION_POLICY_UNPROVEN`.
