"""P0-4: embedding is certified separately from the near-planar budget."""

from __future__ import annotations

from fractions import Fraction

import cftuv_envelope
from cftuv_envelope.contracts.metric import (
    PRODUCT_SKIRT_ABSOLUTE_BUDGET,
    NearPlanarProjectionEmbeddingCertificateV1,
    SourceSnapEmbeddingCertificateV1,
)


def test_the_two_embedding_certificate_types_are_additive_public_contracts():
    assert cftuv_envelope.SourceSnapEmbeddingCertificateV1 is (
        SourceSnapEmbeddingCertificateV1
    )
    assert cftuv_envelope.NearPlanarProjectionEmbeddingCertificateV1 is (
        NearPlanarProjectionEmbeddingCertificateV1
    )


def test_embedding_does_not_take_authority_over_the_product_skirt_budget():
    assert PRODUCT_SKIRT_ABSOLUTE_BUDGET == Fraction(1, 80)
