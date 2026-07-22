"""Public API for the Blender-free CFTUV Envelope contract package."""

from .canonical import (
    GeometryBatchSemanticDigest,
    SemanticPlanDigest,
    SnapshotDigest,
    geometry_batch_semantic_digest,
    semantic_plan_digest,
    snapshot_digest,
)
from .codec import (
    AnalysisSnapshotCodecV1,
    CompiledPlanCodecV1,
    ContractCodecError,
    DecalRequestCodecV1,
    GeometryBatchCodecV1,
    canonical_json_bytes,
)
from .contracts import *  # noqa: F401,F403
from .ids import *  # noqa: F401,F403
from .numeric import *  # noqa: F401,F403
from .outcomes import NamedOutcome
from .schema import ContractSchemaError, json_schema_for
from .validation import (
    ContractValidationError,
    ValidationCode,
    ValidationIssue,
    raise_for_issues,
    validate_analysis_snapshot,
    validate_compiled_plan,
    validate_cross_contract_references,
    validate_decal_request,
    validate_geometry_batch,
)
from .version import __version__
