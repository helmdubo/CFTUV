"""Fail-closed JunctionEnvelope geometry boundary for Session C-R1.

Typed route topology is compiled and preserved, but EC1/AM11 did not approve a
support-based Junction shape law.  The former convex-hull construction added
material without contractual authority, so geometry evaluation remains named
unsupported until that law is separately accepted.
"""

from __future__ import annotations

import sympy as sp

from ..contracts.envelopes import JunctionEnvelopeSpec
from ..numeric import LocalLengthV1
from .common import GeometryContext, ReferenceGeometryError
from .contracts import ReferenceEnvelopeInstanceV1, ReferenceOutcome


def evaluate_junction_envelope(
    context: GeometryContext,
    spec: JunctionEnvelopeSpec,
    alpha_value: LocalLengthV1,
    effective_alpha: sp.Expr,
) -> ReferenceEnvelopeInstanceV1:
    del context, alpha_value, effective_alpha
    raise ReferenceGeometryError(
        ReferenceOutcome.REFERENCE_JUNCTION_ENVELOPE_LAW_UNPROVEN,
        "JunctionEnvelope route topology is compiled, but no support-based "
        f"geometry law is approved for {spec.envelope_spec_id}",
    )
