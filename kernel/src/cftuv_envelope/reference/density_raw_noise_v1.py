"""Raw declaration → exact rational noise facts, without symbolic geometry."""

from dataclasses import dataclass
from fractions import Fraction

from .._authoring_intent import CANONICAL_RESTORATION_ARTIST_ERROR
from .._canonical_angle import CANONICAL_REFLEX_EXCESS_RELATIONS, PI_RATIONAL_UPPER_BOUND
from .contracts import EvaluationBindingNoiseRefusalV1


@dataclass(frozen=True, slots=True)
class RawNoiseDecisionV1:
    relation: str | None
    canonical: Fraction | None
    lateral_squared: Fraction | None = None
    edge_sine_squared: Fraction | None = None
    turn_cosine_squared: Fraction | None = None
    refusal: str | None = None

    @property
    def applicable(self):
        return self.canonical is not None and self.lateral_squared is not None and self.refusal is None


def canonical_selector_fact(raw, laws):
    lower, upper = raw.canonical.source_reflex_interval
    matches = []
    for relation, canonical in CANONICAL_REFLEX_EXCESS_RELATIONS:
        deviation = max(abs(lower-canonical), abs(upper-canonical))
        if laws.sign(laws.scalar(deviation*PI_RATIONAL_UPPER_BOUND-CANONICAL_RESTORATION_ARTIST_ERROR)) <= 0:
            matches.append((relation.value, canonical))
    if len(matches) != 1:
        if raw.canonical.canonical_excess_over_pi is not None:
            raise ValueError("DENSITY_RAW_CANONICAL_DECLARATION_UNPROVEN")
        return None, None
    relation, canonical = matches[0]
    if raw.canonical.canonical_excess_over_pi not in (None, canonical):
        raise ValueError("DENSITY_RAW_CANONICAL_DECLARATION_MISMATCH")
    return relation, canonical


def evaluate_raw_noise(raw, laws, source_metric):
    relation, canonical = canonical_selector_fact(raw, laws)
    result = RawNoiseDecisionV1(relation, canonical)
    rows = dict(raw.canonical.noise_policy_declaration)
    if canonical is None or not rows["binding_present"]:
        return result
    refusals = EvaluationBindingNoiseRefusalV1

    def refused(reason):
        return RawNoiseDecisionV1(relation, canonical, refusal=reason.value)

    if not rows["source_coordinates_available"]:
        return refused(refusals.SOURCE_COORDINATES_UNAVAILABLE)
    incoming, outgoing = rows["incoming_directed_vertex_ids"], rows["outgoing_directed_vertex_ids"]
    anchor = raw.source_geometry.incoming.anchor_vertex_id
    if len(incoming) < 2 or len(outgoing) < 2 or incoming[-1] != anchor or outgoing[0] != anchor:
        return refused(refusals.CORNER_VERTICES_UNAVAILABLE)
    ids = incoming[-2], anchor, outgoing[1]
    source = {name: (Fraction(*x), Fraction(*y)) for name, x, y in rows["source_coordinates"]}
    evaluation = {name: (Fraction(*x), Fraction(*y)) for name, x, y in rows["evaluation_coordinates"]}
    if any(name not in source or name not in evaluation for name in ids):
        return refused(refusals.CORNER_VERTICES_UNAVAILABLE)
    g00, g01, g11 = raw.source_geometry.metric.gram
    determinant = g00*g11-g01*g01

    def norm(vector):
        owned = laws.vector(*vector)
        return laws.fraction(laws.dot_g(source_metric, owned, owned))

    edges = []
    for start, end in zip(ids, ids[1:]):
        edge = tuple(source[end][i]-source[start][i] for i in (0, 1))
        moved = tuple(evaluation[end][i]-evaluation[start][i] for i in (0, 1))
        original_squared, moved_squared = norm(edge), norm(moved)
        if original_squared == 0 or moved_squared == 0:
            return refused(refusals.DEGENERATE_CORNER_EDGE)
        area_squared = determinant*(edge[0]*moved[1]-edge[1]*moved[0])**2
        edges.append((area_squared/original_squared, area_squared/(original_squared*moved_squared), moved))
    first, second = (item[2] for item in edges)
    dot = laws.fraction(laws.dot_g(source_metric, laws.vector(*first), laws.vector(*second)))
    turn_squared = dot*dot/(norm(first)*norm(second))
    scale = rows["base_lattice_scale"]
    cell_bound = (g00+2*abs(g01)+g11)/(scale*scale)
    lateral, edge_sine = max(item[0] for item in edges), max(item[1] for item in edges)
    bound = Fraction(*rows["noise_direction_sine_bound"])
    if canonical*2 != 1:
        return refused(refusals.CANONICAL_RELATION_HAS_NO_ANGULAR_BOUND)
    for value, limit, reason in (
        (lateral, cell_bound, refusals.LATERAL_SHIFT_EXCEEDS_ONE_LATTICE_CELL),
        (edge_sine, bound*bound, refusals.EDGE_DIRECTION_NOISE_EXCEEDS_DECLARED_BOUND),
        (turn_squared, bound*bound, refusals.TURN_NOISE_EXCEEDS_DECLARED_BOUND),
    ):
        if laws.sign(laws.scalar(value-limit)) > 0:
            return refused(reason)
    return RawNoiseDecisionV1(relation, canonical, lateral, edge_sine, turn_squared)
