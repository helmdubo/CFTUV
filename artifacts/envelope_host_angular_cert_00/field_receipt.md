# HOST-ANG-00 field receipt

Implementation: `b943ccb6cdaa0d970b3bbcd4dbeccecf75f93b7e`

Blender 4.3.2 rebuilt the deterministic `building.002` fixture and exported
selected physical edges 2, 3 and 7 twice. Both exports were byte-identical.
The source mesh fingerprint was
`6eb41c1d243367a2424edc3f664a8f9c61d185e0866be827fa28bf8c44d55f3a`
before and after analysis.

The current exporter emitted four exact Angular owner sectors, relations and
certificates. Two ratios were exactly `0.5`; the non-rational intervals were:

- `[0.500000240757182542983974037,
  0.500000240757182542983974042]`;
- `[0.50010689650843996861663167,
  0.500106896508439968616631675]`.

Both bounds of every non-rational interval were proved against the exact
support cosine. The regression suite also proves that an undecidable exact
predicate still returns
`ENVELOPE_DEBUG_EXACT_ANGULAR_CERTIFICATE_UNAVAILABLE`.

Contract validation reported zero snapshot, request and cross-reference
issues. The kernel result was `RAW_READY / EXACT`, semantic digest
`1ae4404ece65704f092fc87e9b28d11941995d1c0b7607a8e1324f4ff42cfee8`,
with 14 boundary occurrences, one loop, one region and zero point contacts.

## FIX-00 differential

The `DecalRequestV1` bytes are unchanged. After removing
`angular_owner_sectors`, `corner_relations` and
`reflex_angle_certificates`, every snapshot record equals FIX-00. The complete
snapshot differs because the accepted older exporter omitted all Angular
records.

Consequently, the exact current host path does not reproduce the older
3-region/2-point-contact RawCoverage. That portable FIX-00 pair remains a
valid kernel interaction fixture, but it is not the current exact host export
oracle.
