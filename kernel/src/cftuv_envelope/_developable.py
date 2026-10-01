"""Ступень DEVELOPABLE: домен как ПРИВЯЗАННАЯ К РЕШЁТКЕ развёртка, судимая точным растяжением.

Модуль внутренний и ничем не владеет: допуск растяжения — `contracts.metric.
DEVELOPABLE_STRETCH_BUDGET`, топологию диска, предложение и решётку делает `_unfold`,
суд — `_stretch`, ярлыки вершин — `_fan_closure`. Здесь они складываются в
сертификат и в целочисленную карту, а отказы называются теми именами, что
перечислены в `outcomes`.

ПОРЯДОК, и в нём вся идея.
1. Диск владельца (`owner_topology`): иначе отказ по топологии.
2. Предложение шарнирной развёртки в binary64 (`hinge_proposal`).
3. СУД ДО ПРИВЯЗКИ: растяжение точных дробей предложения. Вне бюджета — отказ
   `DEVELOPABLE_STRETCH_BUDGET_EXCEEDED` с худшим треугольником и худшей вершиной:
   привязка не спасает карту, которая плоха сама. Простота границы до привязки —
   тоже здесь: перекрытие спирали живёт при любом масштабе решётки.
4. СТУПЕНИ РЕШЁТКИ КАРТЫ: `S' = k · S` для `k` из `UNFOLD_CHART_SCALE_FACTORS`
   (`S` — масштаб решётки источника). Первая ступень, на которой привязанная карта
   в бюджете растяжения, без перевёрнутых треугольников и с простой границей, и
   есть карта. Ни одна не годна, хотя предложение до привязки годно, — не
   допуск, а имя: `DEVELOPABLE_CHART_LATTICE_TOO_COARSE`.

Решётка очереди этой карты — единица (целые узлы): `chart_grid_for` даёт масштаб
`1` при `S' >= 2 S`, так что координаты карты — целые, а метрическая единица —
`1/S'` метра.
"""

from __future__ import annotations

from fractions import Fraction
from hashlib import sha256

from ._embedding import _NONE, _OVERLAP, _segment_relation2
from ._fan_closure import classify_interior_vertices, worst_defect_vertex
from ._stretch import measure_stretch, stretch_refusal_text, stretch_violations
from ._unfold import (
    UnfoldTopologyV1,
    chart_metres,
    exact_metres,
    hinge_proposal,
    owner_topology,
    refusal,
    snap_to_chart_lattice,
)
from .contracts.metric import (
    DEVELOPABLE_STRETCH_BUDGET,
    AffineReconstructionLawV1,
    DevelopableLiftLawV1,
    DevelopableProposalLawV1,
    DevelopableUnfoldCertificateV1,
    DevelopableUnfoldTreeLawV1,
    ExactPoint3V1,
    ExactRationalV1,
    ExactVector3V1,
    PlanarityAdmissionLawV1,
    SnappedSourcePositionV1,
)
from .ids import PlanarityCertificateId
from .outcomes import NamedOutcome

#: Ступени решётки карты как кратные масштаба решётки источника: ячейка карты
#: `1/(k S)` метра. `2` — наименьшая, при которой решётка очереди равна единице.
UNFOLD_CHART_SCALE_FACTORS = (2, 8, 32)


def _rational(value: Fraction | int) -> ExactRationalV1:
    item = Fraction(value)
    return ExactRationalV1(item.numerator, item.denominator)


def stable_unfold_id(source_revision, patch_domain_id) -> str:
    payload = "\x1f".join(
        ("developable-unfold", source_revision.value, patch_domain_id.value)
    )
    return f"developable-unfold:{sha256(payload.encode('utf-8')).hexdigest()[:24]}"


def boundary_overlaps(topology: UnfoldTopologyV1, points) -> tuple[int, tuple]:
    """Пары граничных рёбер карты, нарушающие простоту границы: точно, на целых/дробях.

    Непримыкающие рёбра не вправе иметь ни одной общей точки; примыкающие — не
    вправе накладываться (возврат вдоль той же прямой). Возвращает число нарушенных
    пар и имена вершин первой пары (четыре имени либо пусто).
    """

    edges = []
    for triangle_id, ordinal in topology.boundary_sides:
        triangle = topology.by_id[triangle_id]
        edges.append(
            (triangle.vertex_ids[ordinal], triangle.vertex_ids[(ordinal + 1) % 3])
        )
    edges.sort(key=lambda pair: (pair[0].value, pair[1].value))
    count, first = 0, ()
    for left_index, left in enumerate(edges):
        for right in edges[left_index + 1 :]:
            ends = (left[0], left[1], right[0], right[1])
            relation = _segment_relation2(*(points[item] for item in ends))
            adjacent = not {left[0], left[1]}.isdisjoint(right)
            violated = relation == _OVERLAP if adjacent else relation != _NONE
            if not violated:
                continue
            count += 1
            first = first or ends
    return count, first


def _overlap_text(count: int, first: tuple) -> str:
    named = "->".join(item.value for item in first[:2]) + " vs " + "->".join(
        item.value for item in first[2:]
    )
    return f"{count} boundary edge pairs of the chart meet or overlap (first: {named})"


def _stretch_failure(facts, classes, prefix: str):
    """Отказ растяжения/переворота с числами; имя — по порядку `stretch_violations`."""

    outcome = stretch_violations(facts.certificate)[0]
    text = stretch_refusal_text(
        facts.certificate, worst_vertex=_vertex_name(worst_defect_vertex(classes))
    )
    return refusal(outcome, f"{prefix}{text}")


def _vertex_name(vertex_id):
    return None if vertex_id is None else vertex_id.value


def _check_inputs(owner_triangles, snapped, required_ids) -> None:
    missing = sorted(
        {
            vertex.value
            for item in owner_triangles
            for vertex in item.vertex_ids
            if vertex not in snapped
        }
    )
    if missing:
        raise refusal(
            NamedOutcome.NEAR_PLANAR_OWNER_SURFACE_TRIANGLES_UNAVAILABLE,
            f"owner surface triangles name vertices outside the owner Patch: {missing}",
        )
    covered = {vertex for item in owner_triangles for vertex in item.vertex_ids}
    uncovered = sorted(item.value for item in required_ids if item not in covered)
    if uncovered:
        raise refusal(
            NamedOutcome.DEVELOPABLE_ADJACENCY_UNAVAILABLE,
            f"owner face vertices carry no surface triangle: {uncovered[:3]}",
        )


def _certificate(
    *,
    source_revision,
    patch_domain_id,
    snapped,
    required_ids,
    topology,
    proposal,
    chart_scale,
    trials,
    facts,
    proposal_band,
    snapped_chart,
    classes,
    previous_refusals,
):
    return DevelopableUnfoldCertificateV1(
        certificate_id=PlanarityCertificateId(
            stable_unfold_id(source_revision, patch_domain_id)
        ),
        patch_domain_id=patch_domain_id,
        source_revision=source_revision,
        admission_law=PlanarityAdmissionLawV1.DEVELOPABLE_UNFOLD_V1,
        exact=False,
        exact_plane_normal=ExactVector3V1(
            _rational(0), _rational(0), _rational(1)
        ),
        source_vertex_ids=frozenset(required_ids),
        reconstruction_law=AffineReconstructionLawV1.O_PLUS_U_A_PLUS_V_B_V1,
        tree_law=DevelopableUnfoldTreeLawV1.CANONICAL_BFS_SMALLEST_TRIANGLE_ID_V1,
        proposal_law=DevelopableProposalLawV1.BINARY64_HINGE_V1,
        lift_law=DevelopableLiftLawV1.UNFOLDED_SOURCE_TRIANGLES_V1,
        root_triangle_id=proposal.root_triangle_id,
        chart_scale=chart_scale,
        chart_scale_trials=trials,
        stretch=facts.certificate,
        proposal_worst_band_squared_upper=proposal_band,
        snapped_vertex_count=snapped_chart.snapped_vertex_count,
        snap_residual=_rational(snapped_chart.snap_residual),
        vertex_classes=frozenset(classes),
        boundary_loop_count=topology.boundary_loop_count,
        chart_boundary_overlap_count=0,
        previous_refusals=tuple(previous_refusals),
        snapped_source_positions=frozenset(
            SnappedSourcePositionV1(
                vertex_id,
                ExactPoint3V1(*(_rational(item) for item in snapped[vertex_id])),
            )
            for vertex_id in required_ids
        ),
    )


class DevelopableChartV1:
    """Итог ступени: сертификат и целые узлы карты (единицы решётки `1/S'`)."""

    __slots__ = ("certificate", "nodes", "chart_scale")

    def __init__(self, certificate, nodes, chart_scale: int) -> None:
        self.certificate = certificate
        self.nodes = nodes
        self.chart_scale = chart_scale


def build_developable_chart(
    *,
    source_revision,
    patch_domain_id,
    snapped,
    owner_triangles,
    required_ids,
    source_scale: int | None,
    previous_refusals: tuple[str, ...] = (),
    budget: Fraction = DEVELOPABLE_STRETCH_BUDGET,
) -> DevelopableChartV1:
    """Карта и сертификат развёртки домена, либо именованный отказ.

    `snapped` — точные привязанные 3D-позиции вершин владельца (до проекции),
    `owner_triangles` — его треугольники, `source_scale` — масштаб решётки
    источника (`None` — привязки источника не было, и карта не определена).
    """

    if source_scale is None:
        raise refusal(
            NamedOutcome.DEVELOPABLE_REQUIRES_SOURCE_SNAP,
            "the unfolded chart is measured in steps of the source grid, and the "
            "grid law did not snap the source",
        )
    _check_inputs(owner_triangles, snapped, required_ids)
    topology = owner_topology(owner_triangles, snapped)
    proposal = hinge_proposal(topology, snapped)
    classes = classify_interior_vertices(topology, snapped, patch_domain_id.value)
    exact = exact_metres(proposal.coordinates)
    raw = measure_stretch(topology.triangles, snapped, exact, budget)
    proposal_overlap = boundary_overlaps(topology, exact)
    if proposal_overlap[0]:
        raise refusal(
            NamedOutcome.DEVELOPABLE_CHART_SELF_OVERLAP,
            "the hinge unfolding covers itself before any snapping: "
            + _overlap_text(*proposal_overlap),
        )
    if raw.certificate.triangles_outside_budget or raw.certificate.chart_flipped_triangle_count:
        raise _stretch_failure(raw, classes, "the hinge proposal before snapping: ")
    last = None
    for trial, factor in enumerate(UNFOLD_CHART_SCALE_FACTORS, start=1):
        chart_scale = factor * source_scale
        snapped_chart = snap_to_chart_lattice(exact, chart_scale)
        points = chart_metres(snapped_chart.nodes, chart_scale)
        facts = measure_stretch(topology.triangles, snapped, points, budget)
        overlap = boundary_overlaps(topology, snapped_chart.nodes)
        if not stretch_violations(facts.certificate) and not overlap[0]:
            return DevelopableChartV1(
                _certificate(
                    source_revision=source_revision,
                    patch_domain_id=patch_domain_id,
                    snapped=snapped,
                    required_ids=required_ids,
                    topology=topology,
                    proposal=proposal,
                    chart_scale=chart_scale,
                    trials=trial,
                    facts=facts,
                    proposal_band=raw.certificate.worst_band_squared_upper,
                    snapped_chart=snapped_chart,
                    classes=classes,
                    previous_refusals=previous_refusals,
                ),
                snapped_chart.nodes,
                chart_scale,
            )
        last = (chart_scale, facts, overlap)
    chart_scale, facts, overlap = last
    band = raw.certificate.worst_band_squared_upper
    steps = tuple(factor * source_scale for factor in UNFOLD_CHART_SCALE_FACTORS)
    stretch = stretch_refusal_text(
        facts.certificate, worst_vertex=_vertex_name(worst_defect_vertex(classes))
    )
    boundary = _overlap_text(*overlap) if overlap[0] else "boundary simple"
    raise refusal(
        NamedOutcome.DEVELOPABLE_CHART_LATTICE_TOO_COARSE,
        "the unsnapped proposal is within budget "
        f"(worst_band_squared<={band.numerator / band.denominator:.9e}), but no chart "
        f"lattice step in {steps} kept it; at S'={chart_scale}: {stretch}; {boundary}",
    )
