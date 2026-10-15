"""Closed raw request/result types for the owned whole-operation CANDIDATE.

The Python facade accepts a compilation before Density attachment. Native core
input contains declaration facts only: no normalized normals, ideal rays,
feasibility, lifted flag or derived binding reasons. Construction, shell and
angle identities are explicit and must also identify an invocation-local memo.
"""

from __future__ import annotations

from dataclasses import dataclass
from fractions import Fraction
from typing import TYPE_CHECKING, TypeAlias

from .density_construct_v1 import ExpandLimitsV1
from .density_scalar_v2 import DENSITY_SOURCE_CONSTRUCTION_V1
from .density_shell_v2 import DENSITY_SHELL_CANON_V2_CANDIDATE

if TYPE_CHECKING:
    from .contracts import ReferenceEnvelopeCompilationV1

DENSITY_PRINCIPAL_ANGLE_A2_FIXED160_V1 = "DENSITY_PRINCIPAL_ANGLE_A2_FIXED160_V1"
DENSITY_SOURCE_CONSTRUCTION_V1_APPLIED = "DENSITY_SOURCE_CONSTRUCTION_V1_APPLIED"
PrimitiveFactV1: TypeAlias = bool | int | str | None | tuple["PrimitiveFactV1", ...]


def _q(value: Fraction | int) -> Fraction:
    if type(value) is int or isinstance(value, Fraction):
        return Fraction(value)
    raise TypeError("Raw Density numeric facts require exact Python int/Fraction")


def _primitive(value: PrimitiveFactV1) -> None:
    if value is None or type(value) in (bool, int, str):
        return
    if type(value) is tuple:
        for child in value:
            _primitive(child)
        return
    raise TypeError("Core facts must be closed immutable primitives, never arbitrary Python objects")


def _rows(rows: tuple[tuple[str, PrimitiveFactV1], ...]) -> None:
    if type(rows) is not tuple:
        raise TypeError("Closed declaration rows must be immutable tuples")
    names = set()
    for row in rows:
        if type(row) is not tuple or len(row) != 2:
            raise TypeError("Each declaration row must be a (name,value) tuple")
        name, value = row
        if type(name) is not str or name in names:
            raise ValueError("Declaration keys must be unique text names")
        names.add(name)
        _primitive(value)


def _text_ids(*values: str | None) -> None:
    if any(value is not None and type(value) is not str for value in values):
        raise TypeError("Optional IDs must be plain text, never symbolic objects")


@dataclass(frozen=True, slots=True)
class DensityOracleCanonV1:
    construction: str
    shell: str
    angle: str

    def validate_candidate(self) -> None:
        expected = (DENSITY_SOURCE_CONSTRUCTION_V1, DENSITY_SHELL_CANON_V2_CANDIDATE,
                    DENSITY_PRINCIPAL_ANGLE_A2_FIXED160_V1)
        if (self.construction, self.shell, self.angle) != expected:
            raise ValueError("DENSITY_ORACLE_V1_CANON_MISMATCH")


@dataclass(frozen=True, slots=True)
class DensityOracleLimitsV1:
    preparation_cap: int
    search_cap: int
    box_cap: int
    height_cap: int
    finite_expand: ExpandLimitsV1

    def __post_init__(self) -> None:
        for cap in (self.preparation_cap, self.search_cap, self.box_cap, self.height_cap):
            if type(cap) is not int or cap < 0:
                raise ValueError("Raw Density caps must be explicit nonnegative integers")
        if not isinstance(self.finite_expand, ExpandLimitsV1):
            raise TypeError("An explicit finite expansion profile is required")


@dataclass(frozen=True, slots=True)
class RawDensityPointV1:
    x: Fraction
    y: Fraction

    def __post_init__(self) -> None:
        object.__setattr__(self, "x", _q(self.x))
        object.__setattr__(self, "y", _q(self.y))


@dataclass(frozen=True, slots=True)
class RawDensityMetricV1:
    gram: tuple[Fraction, Fraction, Fraction]
    inverse_gram: tuple[Fraction, Fraction, Fraction]
    owner_orientation_sign: int
    metric_id: str | None = None

    def __post_init__(self) -> None:
        for name in ("gram", "inverse_gram"):
            values = getattr(self, name)
            if type(values) is not tuple or len(values) != 3:
                raise TypeError("Symmetric Gram facts require the three entries (00,01,11)")
            object.__setattr__(self, name, tuple(_q(value) for value in values))
        if type(self.owner_orientation_sign) is not int or self.owner_orientation_sign not in (-1, 1):
            raise ValueError("Raw owner orientation must be an explicit +/-1 fact")
        _text_ids(self.metric_id)


@dataclass(frozen=True, slots=True)
class RawDensityIncidentV1:
    """Selected anchor-adjacent raw segment; its unit normal is a core output."""

    start: RawDensityPointV1
    end: RawDensityPointV1
    owner_left: bool | None
    anchor_vertex_id: str | None = None
    source_vertex_start_id: str | None = None
    source_vertex_end_id: str | None = None
    support_id: str | None = None
    chain_use_orientation: str | None = None
    source_direction_declaration: tuple[tuple[str, PrimitiveFactV1], ...] = ()
    owner_face_cycles: tuple[tuple[RawDensityPointV1, ...], ...] = ()
    owner_face_vertex_id_cycles: tuple[tuple[str, ...], ...] = ()
    owner_face_ids: tuple[str, ...] = ()
    owner_face_edge_ordinals: tuple[int, ...] = ()
    physical_edge_id: str | None = None
    chain_use_id: str | None = None

    def __post_init__(self) -> None:
        if not isinstance(self.start, RawDensityPointV1) or not isinstance(self.end, RawDensityPointV1):
            raise TypeError("Incident raw endpoints must be QPoint facts")
        if self.owner_left is not None and type(self.owner_left) is not bool:
            raise TypeError("Incident owner side must be a declaration fact")
        _text_ids(self.anchor_vertex_id, self.source_vertex_start_id, self.source_vertex_end_id,
                  self.support_id, self.chain_use_orientation, self.physical_edge_id, self.chain_use_id)
        _rows(self.source_direction_declaration)
        if type(self.owner_face_cycles) is not tuple:
            raise TypeError("Owner face cycles must be immutable QPoint sequences")
        for cycle in self.owner_face_cycles:
            if type(cycle) is not tuple or len(cycle) < 3 or any(not isinstance(point, RawDensityPointV1) for point in cycle):
                raise TypeError("Each raw owner face cycle requires at least three QPoints")
        if self.owner_face_vertex_id_cycles:
            if type(self.owner_face_vertex_id_cycles) is not tuple or len(self.owner_face_vertex_id_cycles) != len(self.owner_face_cycles):
                raise ValueError("Parallel owner vertex-id cycles must match raw face cycles")
            for ids, points in zip(self.owner_face_vertex_id_cycles, self.owner_face_cycles):
                if type(ids) is not tuple or len(ids) != len(points):
                    raise ValueError("Owner cycle vertex IDs must match QPoint cardinality")
                _text_ids(*ids)
        if type(self.owner_face_ids) is not tuple or (self.owner_face_ids and len(self.owner_face_ids) != len(self.owner_face_cycles)):
            raise ValueError("Optional face IDs must match owner face cycles")
        _text_ids(*self.owner_face_ids)
        if type(self.owner_face_edge_ordinals) is not tuple:
            raise TypeError("Owner face edge ordinals must be immutable integers")
        if self.owner_face_edge_ordinals and len(self.owner_face_edge_ordinals) != len(self.owner_face_cycles):
            raise ValueError("Edge ordinals must match owner face cycles")
        for ordinal, cycle in zip(self.owner_face_edge_ordinals, self.owner_face_cycles):
            if type(ordinal) is not int or not 0 <= ordinal < len(cycle):
                raise ValueError("Owner face edge ordinal is outside its raw cycle")


@dataclass(frozen=True, slots=True)
class RawDensityGeometryV1:
    metric: RawDensityMetricV1
    incoming: RawDensityIncidentV1
    outgoing: RawDensityIncidentV1

    def __post_init__(self) -> None:
        if not isinstance(self.metric, RawDensityMetricV1):
            raise TypeError("Raw geometry requires a RawDensityMetricV1")
        if not isinstance(self.incoming, RawDensityIncidentV1) or not isinstance(self.outgoing, RawDensityIncidentV1):
            raise TypeError("Raw geometry requires two incident declaration records")


@dataclass(frozen=True, slots=True)
class RawDensitySelectionV1:
    q: int
    source_hidden_count: int
    selection_policy_id: str
    max_subturn_value_id: str
    bucket_c: int
    selection_certificate_id: str | None = None

    def __post_init__(self) -> None:
        if type(self.q) is not int or self.q not in range(2, 7):
            raise ValueError("Raw Density q must be in the declared 2..6 profile")
        if type(self.source_hidden_count) is not int or not 0 <= self.source_hidden_count < self.q:
            raise ValueError("Source count must satisfy 0 <= H < q")
        if type(self.bucket_c) is not int or not 1 <= self.bucket_c <= self.q:
            raise ValueError("Declared selection bucket must satisfy 1 <= C <= q")
        # Первый bucket также имеет один скрытый луч: H=max(1,C-1).
        expected = 0 if self.source_hidden_count == 0 and self.bucket_c == 1 else max(1, self.bucket_c - 1)
        if self.source_hidden_count != expected:
            raise ValueError("Positive source count must satisfy H=max(1,C-1)")
        _text_ids(self.selection_policy_id, self.max_subturn_value_id, self.selection_certificate_id)


@dataclass(frozen=True, slots=True)
class RawDensityCanonicalFactsV1:
    """Primitive declarations only; all geometric noise effects are recomputed.

    Source interval is the declared reflex excess / pi. The restoration row
    uses exact rational pair tuples and names copied from its immutable source
    certificate. It never contains evaluation dot/cos², a lift, or bound rays.
    """

    source_reflex_interval: tuple[Fraction, Fraction]
    canonical_excess_over_pi: Fraction | None
    restoration_declaration: tuple[tuple[str, PrimitiveFactV1], ...]
    noise_policy_declaration: tuple[tuple[str, PrimitiveFactV1], ...]
    source_reflex_declaration: tuple[tuple[str, PrimitiveFactV1], ...] = ()

    def __post_init__(self) -> None:
        interval = self.source_reflex_interval
        if type(interval) is not tuple or len(interval) != 2:
            raise TypeError("A raw declared reflex interval requires two exact endpoints")
        object.__setattr__(self, "source_reflex_interval", tuple(_q(value) for value in interval))
        if self.source_reflex_interval[0] > self.source_reflex_interval[1]:
            raise ValueError("Declared reflex interval endpoints are reversed")
        if self.canonical_excess_over_pi is not None:
            object.__setattr__(self, "canonical_excess_over_pi", _q(self.canonical_excess_over_pi))
        _rows(self.restoration_declaration)
        _rows(self.noise_policy_declaration)
        _rows(self.source_reflex_declaration)
        forbidden = {"evaluation_turn_sign", "evaluation_turn_cosine_squared", "binding_lateral_offset_gram_squared_bound",
                     "edge_direction_sine_squared_bound", "noise_applicable", "lifted", "binding_reasons", "ideal"}
        if any(name in forbidden for name, _ in self.noise_policy_declaration):
            raise ValueError("Noise input must contain raw cell/grid/offset facts, not derived geometric decisions")


@dataclass(frozen=True, slots=True)
class RawDensitySpecInput:
    source_geometry: RawDensityGeometryV1
    evaluation_geometry: RawDensityGeometryV1
    turn_orientation_sign: int
    selection: RawDensitySelectionV1
    canonical: RawDensityCanonicalFactsV1
    spec_id: str | None = None
    source_relation_id: str | None = None
    owner_sector_id: str | None = None

    def __post_init__(self) -> None:
        if type(self.turn_orientation_sign) is not int or self.turn_orientation_sign not in (-1, 1):
            raise ValueError("Turn orientation is a separate explicit +/-1 source fact")
        if not isinstance(self.source_geometry, RawDensityGeometryV1) or not isinstance(self.evaluation_geometry, RawDensityGeometryV1):
            raise TypeError("Raw spec requires source and evaluation geometry records")
        if not isinstance(self.selection, RawDensitySelectionV1) or not isinstance(self.canonical, RawDensityCanonicalFactsV1):
            raise TypeError("Raw spec requires selection and canonical declaration records")
        _text_ids(self.spec_id, self.source_relation_id, self.owner_sector_id)


@dataclass(frozen=True, slots=True)
class DensityOracleRequestV1:
    compilation: ReferenceEnvelopeCompilationV1
    canon_ids: DensityOracleCanonV1
    limits: DensityOracleLimitsV1
    window_law: str

    def __post_init__(self) -> None:
        from .contracts import ReferenceEnvelopeCompilationV1

        if not isinstance(self.compilation, ReferenceEnvelopeCompilationV1):
            raise TypeError("Oracle facade requires the immutable pre-attachment compilation")
        if not isinstance(self.canon_ids, DensityOracleCanonV1) or not isinstance(self.limits, DensityOracleLimitsV1):
            raise TypeError("Explicit canon identities and resource limits are required")
        self.canon_ids.validate_candidate()
        if self.window_law not in ("VORONOI_OF_NEIGHBOUR_MIDPOINTS", "NARROW_ROTATION_BAND"):
            raise ValueError("Unknown declared Density window law")


@dataclass(frozen=True, slots=True)
class RawDensityBatchInputV1:
    """Одна чистая compile-транзакция; общий memo живёт только внутри вызова."""

    specs: tuple[RawDensitySpecInput, ...]
    canon_ids: DensityOracleCanonV1
    limits: DensityOracleLimitsV1
    window_law: str

    def __post_init__(self) -> None:
        if type(self.specs) is not tuple or any(type(item) is not RawDensitySpecInput for item in self.specs):
            raise TypeError("Density batch requires an immutable tuple of raw specs")
        ids = tuple(item.spec_id for item in self.specs)
        if any(type(item) is not str for item in ids) or ids != tuple(sorted(set(ids))):
            raise ValueError("Density batch spec identities must be present, unique and sorted")
        if type(self.canon_ids) is not DensityOracleCanonV1 or type(self.limits) is not DensityOracleLimitsV1:
            raise TypeError("Density batch requires explicit canon and resource limits")
        self.canon_ids.validate_candidate()
        if self.window_law not in ("VORONOI_OF_NEIGHBOUR_MIDPOINTS", "NARROW_ROTATION_BAND"):
            raise ValueError("Unknown declared Density window law")


@dataclass(frozen=True, slots=True)
class DensityPriceArticleV1:
    phase: str
    cap: int
    counters: tuple[tuple[str, int], ...]


@dataclass(frozen=True, slots=True)
class DecimalValueFactsV1:
    """Exact finite Decimal tuple; exponent preserves authored quantum."""

    coefficient: int
    exponent: int

    def __post_init__(self) -> None:
        if type(self.coefficient) is not int or type(self.exponent) is not int:
            raise TypeError("Decimal facts require integer coefficient/exponent")


@dataclass(frozen=True, slots=True)
class DecimalIntervalFactsV1:
    lower: DecimalValueFactsV1
    upper: DecimalValueFactsV1
    lower_kind: str
    upper_kind: str
    absolute_error_bound: DecimalValueFactsV1


@dataclass(frozen=True, slots=True)
class BindingCertificateFactsV1:
    bound_primitive_integer_vector: tuple[int, int]
    ideal_window_lower_slope_envelope: DecimalIntervalFactsV1
    ideal_window_upper_slope_envelope: DecimalIntervalFactsV1
    certified_window_width_lower_bound: DecimalValueFactsV1
    proven_predicates: tuple[str, ...]
    binding_reason: str | None = None


@dataclass(frozen=True, slots=True)
class LocalWindowFactsV1:
    ordinal: int
    use_x_denominator: bool
    denominator_sign: int
    full_lower_slope_envelope: DecimalIntervalFactsV1
    full_upper_slope_envelope: DecimalIntervalFactsV1
    termination_lower_slope: Fraction
    termination_upper_slope: Fraction
    certified_termination_width: Fraction
    admissible_lower_outward: Fraction
    admissible_lower_inward: Fraction
    admissible_upper_inward: Fraction
    admissible_upper_outward: Fraction


@dataclass(frozen=True, slots=True)
class ChartPieceFactsV1:
    piece_index: int
    use_x_denominator: bool
    denominator_sign: int
    lower_slope_envelope: DecimalIntervalFactsV1
    upper_slope_envelope: DecimalIntervalFactsV1
    lower_endpoint_included: bool
    upper_endpoint_included: bool
    slope_increases_in_ordinal_order: bool
    pole_ownership: str


@dataclass(frozen=True, slots=True)
class AtlasWindowFactsV1:
    ordinal: int
    pieces: tuple[ChartPieceFactsV1, ...]
    termination_piece_index: int
    termination_lower_slope: Fraction
    termination_upper_slope: Fraction
    certified_termination_width: Fraction
    admissible_lower_outward: Fraction
    admissible_lower_inward: Fraction
    admissible_upper_inward: Fraction
    admissible_upper_outward: Fraction


@dataclass(frozen=True, slots=True)
class FareyHeightWitnessFactsV1:
    first_height: int
    last_height: int
    primitive_candidate_counts: tuple[int, ...]


@dataclass(frozen=True, slots=True)
class AdaptiveFanAuthorityFactsV1:
    max_subturn_q: int
    minimal_common_height: int
    exhaustive_previous_height: int
    termination_height_upper_bound: int
    bound_primitive_integer_vectors: tuple[tuple[int, int], ...]
    binding_reasons: tuple[str | None, ...]
    ordinal_windows: tuple[LocalWindowFactsV1 | AtlasWindowFactsV1, ...]
    previous_height_witness: FareyHeightWitnessFactsV1
    proven_predicates: tuple[str, ...]
    authority_id: str | None = None


@dataclass(frozen=True, slots=True)
class CanonicalFanAuthorityFactsV1:
    ray_law: str
    canonical_relation: str
    canonical_reflex_excess_over_pi: Fraction
    hidden_edge_count: int
    max_subturn_q: int
    ray_rotation_pairs: tuple[tuple[int, int], ...]
    bound_primitive_integer_vectors: tuple[tuple[int, int], ...]
    proven_predicates: tuple[str, ...]
    authority_id: str | None = None


@dataclass(frozen=True, slots=True)
class CountLiftFactsV1:
    lift_law: str
    source_hidden_edge_count: int
    effective_hidden_edge_count: int
    max_subturn_q: int
    evaluation_turn_sign: str
    evaluation_turn_cosine_squared: Fraction
    minimality_predecessor_hidden_edge_count: int
    proven_predicates: tuple[str, ...]


@dataclass(frozen=True, slots=True)
class CanonUsageFactsV1:
    """Usage bundled with success/failure; collector applies per-spec identity."""

    canon: str
    applied_event: str


@dataclass(frozen=True, slots=True)
class NoiseRecordFactsV1:
    noise_law: str
    effect: str
    canonical_relation: str
    canonical_reflex_excess_over_pi: Fraction
    source_hidden_edge_count: int
    effective_hidden_edge_count: int
    max_subturn_q: int
    evaluation_turn_sign: str
    evaluation_turn_cosine_squared: Fraction
    binding_lateral_offset_gram_squared_bound: Fraction
    edge_direction_sine_squared_bound: Fraction
    direction_sine_bound: Fraction
    proven_predicates: tuple[str, ...]


@dataclass(frozen=True, slots=True)
class CanonicalSubturnAuthorityFactsV1:
    guarantee_law: str
    canonical_relation: str
    canonical_reflex_excess_over_pi: Fraction
    hidden_edge_count: int
    max_subturn_q: int
    canonical_subturn_over_pi: Fraction
    raw_residual_upper_bound_radians: Fraction
    proven_predicates: tuple[str, ...]


@dataclass(frozen=True, slots=True)
class CanonicalLawRefusalFactsV1:
    # Только исходы, которые старые diagnostics действительно записали бы.
    noise_refusal: str | None = None
    rays_refusal: str | None = None


@dataclass(frozen=True, slots=True)
class DensityShellEventFactsV1:
    value_wire: tuple[int, ...]
    codes: tuple[str, ...]
    rational_numerator: int | None
    rational_denominator: int | None
    rational_grid_units: int | None
    legacy_projected_min_width_units: int | None


@dataclass(frozen=True, slots=True)
class DensityFailureFactsV1:
    # Сообщение уже соответствует compile._failure, без repr символьного AST.
    exception_type: str
    outcome: str
    message: str


@dataclass(frozen=True, slots=True)
class RawDensitySpecResultV1:
    """Core gives complete facts; formatter assigns IDs without geometry math.

    Predicate names use deterministic sorted order, geometric rows use ordinal
    order, chart pieces use piece_index order, work articles retain call order.
    A warm success memo replays canon usage/events but creates no new articles.
    The input declarations supply source/selection/relation IDs to the formatter.
    """

    outcome: str
    effective_hidden_count: int | None
    binding_reasons: tuple[str | None, ...]
    binding_certificates: tuple[BindingCertificateFactsV1 | None, ...]
    authority: AdaptiveFanAuthorityFactsV1 | CanonicalFanAuthorityFactsV1 | None
    count_lift: CountLiftFactsV1 | None
    canon_usage: tuple[CanonUsageFactsV1, ...]
    ordered_work_articles: tuple[DensityPriceArticleV1, ...]
    named_events: tuple[tuple[str, PrimitiveFactV1], ...]
    named_refusals: tuple[tuple[str, PrimitiveFactV1], ...]
    rust_work_diagnostic: tuple[tuple[str, int], ...] = ()
    noise_record: NoiseRecordFactsV1 | None = None
    canonical_subturn_authority: CanonicalSubturnAuthorityFactsV1 | None = None
    law_refusals: CanonicalLawRefusalFactsV1 = CanonicalLawRefusalFactsV1()
    shell_events: tuple[DensityShellEventFactsV1, ...] = ()
    failure: DensityFailureFactsV1 | None = None
