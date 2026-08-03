"""Read-only compatibility for historical planar-normal wire records.

The writer owns one representation: the canonical primitive normal.  One
frozen fixture predates that writer rule and stores an exact positive rational
multiple instead.  This module admits that representation only while the
explicit sunset ratchet is non-zero, reports every hit, and hands downstream
code a canonical in-memory record.

This is a wire-compatibility rule.  It is deliberately not an embedding or
orientation rule: chart orientation has its own authoritative field.
"""

from __future__ import annotations

from dataclasses import dataclass, replace
from enum import Enum
from fractions import Fraction

from .contracts.analysis import AnalysisSnapshotV1
from .contracts.metric import (
    EmbeddingCertifiedRationalAffinePlanarMetricV1,
    ExactRationalV1,
    ExactVector3V1,
    RationalAffinePlanarMetricV2,
)
from .numeric import canonical_primitive_normal


class MetricNormalWireDispositionV1(str, Enum):
    CANONICAL_PRIMITIVE = "CANONICAL_PRIMITIVE"
    POSITIVE_SCALED_COMPATIBILITY_APPLIED = (
        "POSITIVE_SCALED_COMPATIBILITY_APPLIED"
    )
    POSITIVE_SCALED_COMPATIBILITY_EXHAUSTED = (
        "POSITIVE_SCALED_COMPATIBILITY_EXHAUSTED"
    )
    ZERO_NORMAL_FORBIDDEN_BY_WIRE_POLICY = (
        "ZERO_NORMAL_FORBIDDEN_BY_WIRE_POLICY"
    )
    NONPARALLEL_NORMAL_FORBIDDEN_BY_WIRE_POLICY = (
        "NONPARALLEL_NORMAL_FORBIDDEN_BY_WIRE_POLICY"
    )
    NEGATIVE_NORMAL_SCALE_FORBIDDEN_BY_WIRE_POLICY = (
        "NEGATIVE_NORMAL_SCALE_FORBIDDEN_BY_WIRE_POLICY"
    )
    DEGENERATE_AFFINE_BASIS_FORBIDDEN_BY_WIRE_POLICY = (
        "DEGENERATE_AFFINE_BASIS_FORBIDDEN_BY_WIRE_POLICY"
    )


@dataclass(frozen=True, slots=True)
class PositiveScaledNormalCompatibilityPolicyV1:
    positive_scaled_fixtures_remaining: int

    def __post_init__(self) -> None:
        if self.positive_scaled_fixtures_remaining < 0:
            raise ValueError("positive-scaled fixture ratchet cannot be negative")


@dataclass(frozen=True, slots=True)
class MetricNormalWireFactsV1:
    disposition: MetricNormalWireDispositionV1
    canonical_normal: tuple[Fraction, Fraction, Fraction] | None
    shared_scale: Fraction | None

    @property
    def accepted(self) -> bool:
        return self.disposition in (
            MetricNormalWireDispositionV1.CANONICAL_PRIMITIVE,
            MetricNormalWireDispositionV1.POSITIVE_SCALED_COMPATIBILITY_APPLIED,
        )

    @property
    def compatibility_hits(self) -> int:
        return int(
            self.disposition
            is MetricNormalWireDispositionV1.POSITIVE_SCALED_COMPATIBILITY_APPLIED
        )


@dataclass(frozen=True, slots=True)
class MetricWireCompatibilityReceiptV1:
    positive_scaled_normal_compat_hits: int
    dispositions: tuple[MetricNormalWireDispositionV1, ...]


def positive_scaled_normal_compatibility_policy(
) -> PositiveScaledNormalCompatibilityPolicyV1:
    """Current monotone sunset ratchet; lowering to zero removes the exception."""

    positive_scaled_fixtures_remaining = 1
    return PositiveScaledNormalCompatibilityPolicyV1(
        positive_scaled_fixtures_remaining=positive_scaled_fixtures_remaining
    )


def _vector(record) -> tuple[Fraction, Fraction, Fraction]:
    return tuple(
        Fraction(item.numerator, item.denominator)
        for item in (record.x, record.y, record.z)
    )


def _cross(left, right):
    return (
        left[1] * right[2] - left[2] * right[1],
        left[2] * right[0] - left[0] * right[2],
        left[0] * right[1] - left[1] * right[0],
    )


def classify_metric_normal_wire(
    declared,
    basis_normal,
    *,
    policy: PositiveScaledNormalCompatibilityPolicyV1 | None = None,
) -> MetricNormalWireFactsV1:
    """Classify one exact shared scale without a float or component shortcut."""

    active_policy = policy or PositiveScaledNormalCompatibilityPolicyV1(0)
    declared = tuple(Fraction(item) for item in declared)
    basis_normal = tuple(Fraction(item) for item in basis_normal)
    if not any(basis_normal):
        return MetricNormalWireFactsV1(
            (
                MetricNormalWireDispositionV1.DEGENERATE_AFFINE_BASIS_FORBIDDEN_BY_WIRE_POLICY
            ),
            None,
            None,
        )
    canonical = canonical_primitive_normal(basis_normal)
    if not any(declared):
        return MetricNormalWireFactsV1(
            MetricNormalWireDispositionV1.ZERO_NORMAL_FORBIDDEN_BY_WIRE_POLICY,
            canonical,
            None,
        )
    if any(_cross(declared, canonical)):
        return MetricNormalWireFactsV1(
            (
                MetricNormalWireDispositionV1.NONPARALLEL_NORMAL_FORBIDDEN_BY_WIRE_POLICY
            ),
            canonical,
            None,
        )
    pivot = next(index for index, item in enumerate(canonical) if item)
    shared_scale = declared[pivot] / canonical[pivot]
    if any(
        declared[index] != shared_scale * canonical[index]
        for index in range(3)
    ):
        return MetricNormalWireFactsV1(
            (
                MetricNormalWireDispositionV1.NONPARALLEL_NORMAL_FORBIDDEN_BY_WIRE_POLICY
            ),
            canonical,
            None,
        )
    if shared_scale < 0:
        return MetricNormalWireFactsV1(
            (
                MetricNormalWireDispositionV1.NEGATIVE_NORMAL_SCALE_FORBIDDEN_BY_WIRE_POLICY
            ),
            canonical,
            shared_scale,
        )
    if declared == canonical:
        disposition = MetricNormalWireDispositionV1.CANONICAL_PRIMITIVE
    elif active_policy.positive_scaled_fixtures_remaining:
        disposition = (
            MetricNormalWireDispositionV1.
            POSITIVE_SCALED_COMPATIBILITY_APPLIED
        )
    else:
        disposition = (
            MetricNormalWireDispositionV1.
            POSITIVE_SCALED_COMPATIBILITY_EXHAUSTED
        )
    return MetricNormalWireFactsV1(disposition, canonical, shared_scale)


def metric_normal_wire_facts(
    metric: RationalAffinePlanarMetricV2,
    *,
    policy: PositiveScaledNormalCompatibilityPolicyV1 | None = None,
) -> MetricNormalWireFactsV1:
    a = _vector(metric.exact_basis_a)
    b = _vector(metric.exact_basis_b)
    declared = _vector(metric.planarity_certificate.exact_plane_normal)
    return classify_metric_normal_wire(
        declared,
        _cross(a, b),
        policy=policy,
    )


def metric_normal_wire_rejection_message(facts: MetricNormalWireFactsV1) -> str:
    messages = {
        MetricNormalWireDispositionV1.ZERO_NORMAL_FORBIDDEN_BY_WIRE_POLICY: (
            "plane normal must be non-zero"
        ),
        MetricNormalWireDispositionV1.NONPARALLEL_NORMAL_FORBIDDEN_BY_WIRE_POLICY: (
            "plane normal must be parallel to the canonical A/B-basis normal"
        ),
        MetricNormalWireDispositionV1.NEGATIVE_NORMAL_SCALE_FORBIDDEN_BY_WIRE_POLICY: (
            "negative plane-normal scale is forbidden by wire policy"
        ),
        MetricNormalWireDispositionV1.DEGENERATE_AFFINE_BASIS_FORBIDDEN_BY_WIRE_POLICY: (
            "affine basis vectors must define a non-zero plane normal"
        ),
        MetricNormalWireDispositionV1.POSITIVE_SCALED_COMPATIBILITY_EXHAUSTED: (
            "positive-scaled plane-normal compatibility is exhausted"
        ),
    }
    message = messages.get(facts.disposition)
    if message is None:
        raise ValueError("accepted normal-wire facts have no rejection message")
    return f"{facts.disposition.value}: {message}"


def _rational(value: Fraction) -> ExactRationalV1:
    return ExactRationalV1(value.numerator, value.denominator)


def _canonicalized_metric(metric, facts):
    if not facts.compatibility_hits:
        return metric
    normal = ExactVector3V1(*(_rational(item) for item in facts.canonical_normal))
    certificate = replace(metric.planarity_certificate, exact_plane_normal=normal)
    return replace(metric, planarity_certificate=certificate)


def canonicalize_metric_wire_record(
    record,
    *,
    policy: PositiveScaledNormalCompatibilityPolicyV1 | None = None,
):
    """Canonicalize supported metric carriers and return an immutable receipt."""

    dispositions = []
    hits = 0
    ratchet = policy or PositiveScaledNormalCompatibilityPolicyV1(0)

    def canonical_metric(metric):
        nonlocal hits
        remaining = max(0, ratchet.positive_scaled_fixtures_remaining - hits)
        facts = metric_normal_wire_facts(
            metric,
            policy=PositiveScaledNormalCompatibilityPolicyV1(remaining),
        )
        dispositions.append(facts.disposition)
        hits += facts.compatibility_hits
        return _canonicalized_metric(metric, facts)

    if type(record) is RationalAffinePlanarMetricV2:
        canonical = canonical_metric(record)
    elif type(record) is EmbeddingCertifiedRationalAffinePlanarMetricV1:
        canonical = replace(record, metric=canonical_metric(record.metric))
    elif type(record) is AnalysisSnapshotV1:
        ordered = sorted(
            record.surface_metric_descriptors,
            key=lambda item: (
                type(item).__name__,
                getattr(getattr(item, "patch_domain_id", None), "value", ""),
            ),
        )
        descriptors = frozenset(
            canonical_metric(item)
            if type(item) is RationalAffinePlanarMetricV2
            else item
            for item in ordered
        )
        canonical = replace(record, surface_metric_descriptors=descriptors)
    else:
        canonical = record
    return canonical, MetricWireCompatibilityReceiptV1(
        positive_scaled_normal_compat_hits=hits,
        dispositions=tuple(dispositions),
    )
