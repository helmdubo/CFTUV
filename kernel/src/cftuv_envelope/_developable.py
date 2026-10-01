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
`1` при `S' >= 2 S`, так что координаты карты — целые (кроме внутренностей объявленных
прямыми цепей), а метрическая единица — `1/S'` метра.

ОБЪЯВЛЕННЫЕ ПРЯМЫМИ ЦЕПИ (`declared_straight_chains`). Очередь требует их точной
коллинеарности в карте, поэтому внутренние вершины такой цепи кладутся на хорду между
привязанными концами (`_straight_chain`), а судья растяжения судит уже эту карту.
Цепь, которую карта не терпит прямой, — отказ `DEVELOPABLE_DECLARED_STRAIGHT_CHAIN_BENT`
с цепью, худшей вершиной и оболочкой боковой суммы углов против `π`: отказ называет
именно прямизну, потому что карта со свободными цепями годна (иначе называется её
собственная причина). Без объявленных цепей карта — байт в байт прежняя.
"""

from __future__ import annotations

from fractions import Fraction
from hashlib import sha256

from ._embedding import _NONE, _OVERLAP, _segment_relation2
from ._fan_closure import classify_interior_vertices, worst_defect_vertex
from ._straight_chain import (
    bent_chain_text,
    chain_coordinates,
    declared_chain_records,
    lattice_displacement,
)
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
    DevelopableStraightChainLawV1,
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
    displacement,
    classes,
    previous_refusals,
    chain_records,
):
    moved, residual = displacement
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
        snapped_vertex_count=moved,
        snap_residual=_rational(residual),
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
        straight_chain_law=(
            DevelopableStraightChainLawV1.INTERIOR_NODES_ON_ENDPOINT_SEGMENT_V1
        ),
        declared_straight_chains=chain_records,
    )


class DevelopableChartV1:
    """Итог ступени: сертификат и узлы карты (единицы решётки `1/S'`).

    Узел — целое, кроме внутренних вершин объявленных прямыми цепей, не лежавших на хорде
    после привязки: те положены на хорду между концами цепи и рациональны.
    """

    __slots__ = ("certificate", "nodes", "chart_scale")

    def __init__(self, certificate, nodes, chart_scale: int) -> None:
        self.certificate = certificate
        self.nodes = nodes
        self.chart_scale = chart_scale


class _Unfolding:
    """Всё, что не зависит от ступени решётки: топология, предложение, ярлыки, суд до привязки."""

    def __init__(
        self,
        *,
        source_revision,
        patch_domain_id,
        snapped,
        owner_triangles,
        required_ids,
        source_scale,
        previous_refusals,
        budget,
        declared_straight_chains,
    ) -> None:
        _check_inputs(owner_triangles, snapped, required_ids)
        self.source_revision = source_revision
        self.patch_domain_id = patch_domain_id
        self.snapped = snapped
        self.required_ids = required_ids
        self.source_scale = source_scale
        self.previous_refusals = previous_refusals
        self.budget = budget
        self.topology = owner_topology(owner_triangles, snapped)
        self.proposal = hinge_proposal(self.topology, snapped)
        self.classes = classify_interior_vertices(
            self.topology, snapped, patch_domain_id.value
        )
        self.exact = exact_metres(self.proposal.coordinates)
        self.raw = measure_stretch(self.topology.triangles, snapped, self.exact, budget)
        self.chains = tuple(
            item for item in declared_straight_chains if all(v in snapped for v in item)
        )
        self.records: tuple = ()

    def refuse_unsound_proposal(self) -> None:
        """Суд ДО привязки: самонакрытие и растяжение предложения не лечатся решёткой.

        Записи объявленных цепей строятся уже после него: карту, которой не будет, они не нужны.
        """

        overlap = boundary_overlaps(self.topology, self.exact)
        if overlap[0]:
            raise refusal(
                NamedOutcome.DEVELOPABLE_CHART_SELF_OVERLAP,
                "the hinge unfolding covers itself before any snapping: "
                + _overlap_text(*overlap),
            )
        facts = self.raw.certificate
        if facts.triangles_outside_budget or facts.chart_flipped_triangle_count:
            raise _stretch_failure(self.raw, self.classes, "the hinge proposal before snapping: ")
        self.records = declared_chain_records(self.chains, self.topology, self.snapped)

    def certificate(self, trial, chart_scale, facts, displacement):
        return _certificate(
            source_revision=self.source_revision,
            patch_domain_id=self.patch_domain_id,
            snapped=self.snapped,
            required_ids=self.required_ids,
            topology=self.topology,
            proposal=self.proposal,
            chart_scale=chart_scale,
            trials=trial,
            facts=facts,
            proposal_band=self.raw.certificate.worst_band_squared_upper,
            displacement=displacement,
            classes=self.classes,
            previous_refusals=self.previous_refusals,
            chain_records=self.records,
        )

    def failure_text(self, last) -> str:
        """Числа последней неудачи ступеней: растяжение, граница либо причина размещения цепи."""

        chart_scale, facts, overlap, reason = last
        if reason is not None:
            return f"at S'={chart_scale}: {reason}"
        stretch = stretch_refusal_text(
            facts.certificate, worst_vertex=_vertex_name(worst_defect_vertex(self.classes))
        )
        boundary = _overlap_text(*overlap) if overlap[0] else "boundary simple"
        return f"at S'={chart_scale}: {stretch}; {boundary}"


def _search_lattice(unfolding: _Unfolding, chains):
    """Ступени решётки с цепями `chains`: `(карта | None, последняя неудача)`."""

    last = None
    for trial, factor in enumerate(UNFOLD_CHART_SCALE_FACTORS, start=1):
        chart_scale = factor * unfolding.source_scale
        snapped_chart = snap_to_chart_lattice(unfolding.exact, chart_scale)
        coordinates, reason = (
            chain_coordinates(chains, unfolding.exact, snapped_chart.nodes, chart_scale)
            if chains
            else (snapped_chart.nodes, None)
        )
        if reason is not None:
            last = (chart_scale, None, (0, ()), reason)
            continue
        facts = measure_stretch(
            unfolding.topology.triangles,
            unfolding.snapped,
            chart_metres(coordinates, chart_scale),
            unfolding.budget,
        )
        overlap = boundary_overlaps(unfolding.topology, coordinates)
        if not stretch_violations(facts.certificate) and not overlap[0]:
            displacement = (
                lattice_displacement(unfolding.exact, coordinates, chart_scale)
                if chains
                else (snapped_chart.snapped_vertex_count, snapped_chart.snap_residual)
            )
            return (
                DevelopableChartV1(
                    unfolding.certificate(trial, chart_scale, facts, displacement),
                    coordinates,
                    chart_scale,
                ),
                None,
            )
        last = (chart_scale, facts, overlap, None)
    return None, last


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
    declared_straight_chains: tuple = (),
) -> DevelopableChartV1:
    """Карта и сертификат развёртки домена, либо именованный отказ.

    `snapped` — точные привязанные 3D-позиции вершин владельца (до проекции),
    `owner_triangles` — его треугольники, `source_scale` — масштаб решётки
    источника (`None` — привязки источника не было, и карта не определена),
    `declared_straight_chains` — упорядоченные вершины цепей, объявленных прямыми.
    """

    if source_scale is None:
        raise refusal(
            NamedOutcome.DEVELOPABLE_REQUIRES_SOURCE_SNAP,
            "the unfolded chart is measured in steps of the source grid, and the "
            "grid law did not snap the source",
        )
    unfolding = _Unfolding(
        source_revision=source_revision,
        patch_domain_id=patch_domain_id,
        snapped=snapped,
        owner_triangles=owner_triangles,
        required_ids=required_ids,
        source_scale=source_scale,
        previous_refusals=previous_refusals,
        budget=budget,
        declared_straight_chains=declared_straight_chains,
    )
    unfolding.refuse_unsound_proposal()
    chart, last = _search_lattice(unfolding, unfolding.chains)
    if chart is not None:
        return chart
    if unfolding.chains:
        free_chart, free_last = _search_lattice(unfolding, ())
        if free_chart is not None:
            raise refusal(
                NamedOutcome.DEVELOPABLE_DECLARED_STRAIGHT_CHAIN_BENT,
                "the chart is within the stretch budget with free chain lines, but the "
                "declared straight chains cannot be straight in it ("
                + bent_chain_text(unfolding.records)
                + "); "
                + unfolding.failure_text(last),
            )
        last = free_last
    band = unfolding.raw.certificate.worst_band_squared_upper
    steps = tuple(factor * source_scale for factor in UNFOLD_CHART_SCALE_FACTORS)
    raise refusal(
        NamedOutcome.DEVELOPABLE_CHART_LATTICE_TOO_COARSE,
        "the unsnapped proposal is within budget "
        f"(worst_band_squared<={band.numerator / band.denominator:.9e}), but no chart "
        f"lattice step in {steps} kept it; {unfolding.failure_text(last)}",
    )
