"""Permanent full-recompute EC2 planar reference path."""

from .contracts import *  # noqa: F401,F403
from .compile import (
    compile_reference_envelopes,
    declare_reference_self_contacts,
)
from .digest import (
    RawCoverageSemanticDigest,
    raw_coverage_semantic_projection,
    raw_coverage_semantic_digest,
    validate_raw_coverage_digest,
    validate_raw_coverage_semantic_digest,
)
from .domain_geometry import (
    BlockingBoundarySegment,
    BoundaryRole,
    SparsePatchDomainGeometryV1,
    build_sparse_patch_domain_geometry,
)
from .planar_types import *  # noqa: F401,F403
from .metric import *  # noqa: F401,F403
from .provenance import *  # noqa: F401,F403
from .raw_coverage import (
    REFERENCE_BOUNDARY_CAPABILITIES_V1,
    evaluate_reference_raw_coverage,
)
from .validation import validate_reference_geometry_payload
