"""Thin public facade for the staged Envelope host adapter.

Implementation ownership is split between topology export, metric export,
request export and the explicit debug-session controller.  This module keeps
the accepted V0 API stable for callers and tests.
"""

from __future__ import annotations

from .envelope_request_export import (
    EnvelopeDebugDomainEvaluationV1,
    EnvelopeDebugEvaluationV1,
    EnvelopeDebugHostDiagnosticV1,
    EnvelopeDebugHostOutcome,
    EnvelopeDebugHostSeverity,
    EnvelopeDebugStagedEvaluationV1,
    EnvelopeDebugTimingV1,
    EnvelopeHostAdapterError,
    _canonical_chain,
    _host_edge_number,
    _load_kernel,
    build_envelope_analysis_snapshot,
    build_envelope_decal_request,
    evaluate_envelope_debug,
)
from .envelope_topology_export import build_envelope_topology_debug_scene
from .envelope_debug_session import evaluate_envelope_debug_staged


__all__ = (
    "EnvelopeDebugDomainEvaluationV1",
    "EnvelopeDebugEvaluationV1",
    "EnvelopeDebugHostDiagnosticV1",
    "EnvelopeDebugHostOutcome",
    "EnvelopeDebugHostSeverity",
    "EnvelopeDebugStagedEvaluationV1",
    "EnvelopeDebugTimingV1",
    "EnvelopeHostAdapterError",
    "build_envelope_analysis_snapshot",
    "build_envelope_decal_request",
    "build_envelope_topology_debug_scene",
    "evaluate_envelope_debug",
    "evaluate_envelope_debug_staged",
)
