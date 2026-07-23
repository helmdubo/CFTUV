"""Canonical digest for exact ResolvedCoverage without self-hashing."""

from __future__ import annotations

from dataclasses import dataclass, replace
from hashlib import sha256

from ..codec import canonical_json_bytes
from .contracts import ResolvedCoverageResultV1


@dataclass(frozen=True, slots=True)
class ResolvedCoverageSemanticDigest:
    sha256_hex: str


def resolved_coverage_semantic_projection(
    result: ResolvedCoverageResultV1,
) -> ResolvedCoverageResultV1:
    return replace(result, semantic_digest="")


def resolved_coverage_semantic_digest(
    result: ResolvedCoverageResultV1,
) -> ResolvedCoverageSemanticDigest:
    return ResolvedCoverageSemanticDigest(
        sha256(canonical_json_bytes(resolved_coverage_semantic_projection(result))).hexdigest()
    )


def validate_resolved_coverage_digest(result: ResolvedCoverageResultV1) -> bool:
    return result.semantic_digest == resolved_coverage_semantic_digest(result).sha256_hex
