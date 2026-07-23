"""Exact EC2.5 resolved-coverage interaction stage."""

from .arrival import compile_arrival_models
from .candidates import generate_interaction_candidates
from .components import compile_interaction_components
from .contracts import *  # noqa: F401,F403
from .digest import (
    ResolvedCoverageSemanticDigest,
    resolved_coverage_semantic_digest,
    resolved_coverage_semantic_projection,
    validate_resolved_coverage_digest,
)
from .mutual_arrival import ProvenInteractionV1, prove_mutual_arrivals
from .resolved_coverage import resolve_coverage_interactions
from .validation import validate_interaction_inputs
