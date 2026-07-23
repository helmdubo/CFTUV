"""Standalone semantic digest API for exact RawCoverage results."""

from __future__ import annotations

from dataclasses import dataclass, replace
from hashlib import sha256

from ..codec import canonical_json_bytes
from .contracts import RawCoverageResultV1


@dataclass(frozen=True, slots=True)
class RawCoverageSemanticDigest:
    sha256_hex: str


def raw_coverage_semantic_projection(
    result: RawCoverageResultV1,
) -> RawCoverageResultV1:
    """Return the canonical self-digest-free RawCoverage projection."""

    return replace(result, semantic_digest="")


def raw_coverage_semantic_digest(
    result: RawCoverageResultV1,
) -> RawCoverageSemanticDigest:
    """Digest semantic RawCoverage content without self-hashing the digest field."""

    projection = raw_coverage_semantic_projection(result)
    return RawCoverageSemanticDigest(
        sha256(canonical_json_bytes(projection)).hexdigest()
    )


def validate_raw_coverage_semantic_digest(result: RawCoverageResultV1) -> bool:
    return result.semantic_digest == raw_coverage_semantic_digest(result).sha256_hex


def validate_raw_coverage_digest(result: RawCoverageResultV1) -> bool:
    """Contract spelling retained for downstream Session D callers."""

    return validate_raw_coverage_semantic_digest(result)
