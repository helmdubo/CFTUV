"""Ступень DEVELOPABLE: домен как ПРИВЯЗАННАЯ К РЕШЁТКЕ развёртка, судимая точным растяжением.

Модуль внутренний и ничем не владеет: допуск растяжения — политика ЗАПРОСА
(`DecalRequestV1.developable_stretch_budget`; параметр `budget`, значение по умолчанию —
`contracts.metric.DEFAULT_DEVELOPABLE_STRETCH_BUDGET`), топологию диска, предложение и
решётку делает `_unfold`, суд — `_stretch`, ярлыки вершин — `_fan_closure`. Здесь они
складываются в сертификат и в целочисленную карту, а отказы называются теми именами, что
перечислены в `outcomes`.

ПОРЯДОК, и в нём вся идея.
1. Диск владельца (`owner_topology`): иначе отказ по топологии.
2. Предложение шарнирной развёртки в binary64 (`hinge_proposal`).
3. СУД ДО ПРИВЯЗКИ: растяжение точных дробей предложения. Вне бюджета — отказ
   `DEVELOPABLE_STRETCH_BUDGET_EXCEEDED` с худшим треугольником и худшей вершиной:
   привязка не спасает карту, которая плоха сама. Простота границы до привязки —
   тоже здесь: перекрытие спирали живёт при любом масштабе решётки.
3б. ВТОРОЕ ПРЕДЛОЖЕНИЕ (`ARAP_TRIGGER_OUTCOMES`). Только после ИМЕНОВАННОГО отказа
   шарнира на шаге 3 (растяжение, переворот, самонакрытие), и только когда искажение
   шарнира само за бюджетом (растяжение либо переворот: `_arap_can_help`), пробуется ARAP
   (`_arap.arap_proposal`, старт — положения шарнира); его положения судит ТОТ ЖЕ суд
   (тот же бюджет, те же предикаты, та же простота границы). Принят домен, который
   шарнир принимал, — его положения прежние; ARAP пробуется на нём лишь как соперник по
   шагу 3в. Самонакрытие у шарнира
   В БЮДЖЕТЕ и без переворотов (спираль, кольцо без разреза: поверхность поворачивает
   больше оборота) ARAP не лечит: он только снижает искажение и из развёртки с нулевым
   искажением не выходит, — такой отказ остаётся прежним и по тексту. Отказ ARAP несёт
   числа обоих предложений; принявший ARAP сертификат называет закон (`proposal_law`) и
   отказ шарнира (`previous_refusals[-1]`).
3в. ЛУЧШЕЕ ПРЕДЛОЖЕНИЕ (`DevelopableProposalSelectionLawV1`). Карта шарнира, ПРИНЯТАЯ в бюджете
   запроса, раньше уходила в сертификат, даже если ARAP растянул бы её меньше. Теперь: если
   сертифицированное растяжение принятой карты шарнира выше `DEVELOPABLE_ISOMETRIC_ENOUGH`
   (1/50), ARAP тоже строит карту (`_Unfolding.competing_arap`, тот же суд, бюджет и решётка),
   и остаётся карта с МЕНЬШИМ сертифицированным растяжением; равенство решает шарнир. Оба
   числа и победитель пишутся в сертификат; ARAP, которому не дали положений либо чью карту
   отказали, оставляет карту шарнира с именем причины (`HINGE_KEPT_ARAP_*`). Карта шарнира не
   выше порога остаётся побитово прежней: ARAP не пробуется.
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

from collections import OrderedDict
from copy import copy
from dataclasses import replace
from fractions import Fraction
from hashlib import sha256

from ._arap import ArapProposalUnavailable, arap_proposal
from ._embedding import _NONE, _OVERLAP, _segment_relation2
from ._fan_closure import classify_interior_vertices, worst_defect_vertex
from ._straight_chain import (
    bent_chain_text,
    chain_coordinates,
    declared_chain_records,
    lattice_displacement,
)
from ._stretch import band_bounds, measure_stretch, stretch_refusal_text, stretch_violations
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
    DEFAULT_DEVELOPABLE_STRETCH_BUDGET,
    DEVELOPABLE_ISOMETRIC_ENOUGH,
    AffineReconstructionLawV1,
    DevelopableLiftLawV1,
    DevelopableProposalLawV1,
    DevelopableProposalSelectionLawV1,
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

#: Отказы шарнирного предложения, после которых пробуется ARAP: растяжение, переворот и
#: самонакрытие границы лечатся другим положением вершин. Остальные отказы (топология
#: диска, вырожденный треугольник, нет привязки источника) стоят ДО предложения и
#: положениями не лечатся.
ARAP_TRIGGER_OUTCOMES = frozenset(
    {
        NamedOutcome.DEVELOPABLE_STRETCH_BUDGET_EXCEEDED,
        NamedOutcome.DEVELOPABLE_CHART_TRIANGLE_FLIPPED,
        NamedOutcome.DEVELOPABLE_CHART_SELF_OVERLAP,
    }
)


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
    proposal_law,
    chart_scale,
    trials,
    facts,
    proposal_band,
    displacement,
    classes,
    previous_refusals,
    chain_records,
    selection,
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
        proposal_law=proposal_law,
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
        proposal_selection_law=selection[0],
        hinge_chart_worst_band_squared_upper=selection[1],
        arap_chart_worst_band_squared_upper=selection[2],
        arap_refusal=selection[3],
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
        self.proposal_law = DevelopableProposalLawV1.BINARY64_HINGE_V1
        self.proposal_name = "hinge"
        self.selection_law = DevelopableProposalSelectionLawV1.HINGE_ISOMETRIC_ENOUGH_V1
        #: Хвост текста отказов ПОСЛЕ привязки у ARAP (у шарнира пусто: тексты прежние).
        self.proposal_note = ""
        self.classes = classify_interior_vertices(
            self.topology, snapped, patch_domain_id.value
        )
        self.exact = exact_metres(self.proposal.coordinates)
        self.raw = measure_stretch(self.topology.triangles, snapped, self.exact, budget)
        self.chains = tuple(
            item for item in declared_straight_chains if all(v in snapped for v in item)
        )
        self.records: tuple = ()

    def unsound_proposal_refusal(self):
        """Суд ДО привязки: отказ (самонакрытие, растяжение, переворот) либо `None`.

        Решётка этого не лечит: плохая сама карта плоха при любом масштабе.
        """

        overlap = boundary_overlaps(self.topology, self.exact)
        if overlap[0]:
            return refusal(
                NamedOutcome.DEVELOPABLE_CHART_SELF_OVERLAP,
                f"the {self.proposal_name} unfolding covers itself before any snapping: "
                + _overlap_text(*overlap),
            )
        facts = self.raw.certificate
        if facts.triangles_outside_budget or facts.chart_flipped_triangle_count:
            return _stretch_failure(
                self.raw, self.classes, f"the {self.proposal_name} proposal before snapping: "
            )
        return None

    def settle_proposal(self) -> None:
        """Предложение, прошедшее суд до привязки: шарнир, а после его ИМЕНОВАННОГО отказа — ARAP.

        Записи объявленных цепей строятся уже после суда: карту, которой не будет, они не нужны.
        """

        refused = self.unsound_proposal_refusal()
        if refused is not None:
            self.adopt_arap(refused)
        self.records = declared_chain_records(self.chains, self.topology, self.snapped)

    def _arap_can_help(self, hinge_refusal) -> bool:
        """ARAP лечит ИСКАЖЕНИЕ: отказ шарнира названный и растяжение либо переворот шарнира за бюджетом.

        Самонакрытие границы при растяжении в бюджете и без переворотов — свойство самой
        поверхности (развёртка уже не искажена, ARAP из неё не выходит): отказ остаётся
        прежним, без второго предложения.
        """

        facts = self.raw.certificate
        distorted = bool(facts.triangles_outside_budget or facts.chart_flipped_triangle_count)
        return hinge_refusal.outcome in ARAP_TRIGGER_OUTCOMES and distorted

    def adopt_arap(self, hinge_refusal) -> None:
        """Второе предложение: ARAP от положений шарнира; тот же суд, отказ несёт оба предложения.

        Отказ шарнира, которого ARAP не лечит (`_arap_can_help`), остаётся как есть. Нехватка
        самого ARAP (потолок работы, матрица не положительна) называется в том же отказе:
        тихого пропуска второго предложения нет.
        """

        if not self._arap_can_help(hinge_refusal):
            raise hinge_refusal
        try:
            arap = arap_proposal(self.topology, self.proposal, self.snapped)
        except ArapProposalUnavailable as unavailable:
            raise refusal(hinge_refusal.outcome, f"{hinge_refusal}; {unavailable}") from hinge_refusal
        # У отказа шарнира по самонакрытию в тексте нет чисел растяжения, а «лучше ли ARAP
        # шарнира» без них не видно: берём их из записи измерения шарнира.
        numbers = ""
        if hinge_refusal.outcome is NamedOutcome.DEVELOPABLE_CHART_SELF_OVERLAP:
            numbers = "; the hinge proposal's " + stretch_refusal_text(
                self.raw.certificate, worst_vertex=_vertex_name(worst_defect_vertex(self.classes))
            )
        self._swap_in_arap(arap)
        self.selection_law = DevelopableProposalSelectionLawV1.ARAP_AFTER_HINGE_REFUSED_V1
        self.previous_refusals = (*self.previous_refusals, hinge_refusal.outcome.value)
        self.proposal_note = (
            f" [{self.proposal_law.value} after the hinge proposal was refused: "
            f"{hinge_refusal.outcome.value}: {hinge_refusal}{numbers}]"
        )
        refused = self.unsound_proposal_refusal()
        if refused is not None:
            raise refusal(refused.outcome, f"{refused}{self.proposal_note}") from hinge_refusal

    def _swap_in_arap(self, arap) -> None:
        """Положения ARAP вместо положений шарнира: предложение, закон, точные дроби и измерение."""

        self.proposal = replace(self.proposal, coordinates=arap.coordinates)
        self.proposal_law = DevelopableProposalLawV1.ARAP_LOCAL_GLOBAL_80_BINARY64_V1
        self.proposal_name = "ARAP"
        self.exact = exact_metres(arap.coordinates)
        self.raw = measure_stretch(self.topology.triangles, self.snapped, self.exact, self.budget)

    def competing_arap(self):
        """Копия с положениями ARAP при ПРИНЯТОМ шарнире: соперник, а не замена.

        Шарнир не отказан, поэтому `previous_refusals` копии — след лестницы без записи об
        отказе шарнира. `ArapProposalUnavailable` (потолок работы, матрица не положительна)
        поднимается как есть; предложение, которое не прошло суд до привязки, — именованный
        отказ с текстом, называющим соперника. Сам `self` не меняется.
        """

        arap = arap_proposal(self.topology, self.proposal, self.snapped)
        rival = copy(self)
        rival._swap_in_arap(arap)
        rival.selection_law = DevelopableProposalSelectionLawV1.BEST_ARAP_WON_V1
        rival.proposal_note = (
            f" [{rival.proposal_law.value} as the competing proposal: the hinge chart is "
            "accepted but stretched beyond the isometric threshold]"
        )
        refused = rival.unsound_proposal_refusal()
        if refused is not None:
            raise refusal(refused.outcome, f"{refused}{rival.proposal_note}") from None
        return rival

    def certificate(self, trial, chart_scale, facts, displacement):
        arap = self.proposal_law is DevelopableProposalLawV1.ARAP_LOCAL_GLOBAL_80_BINARY64_V1
        return _certificate(
            source_revision=self.source_revision,
            patch_domain_id=self.patch_domain_id,
            snapped=self.snapped,
            required_ids=self.required_ids,
            topology=self.topology,
            proposal=self.proposal,
            proposal_law=self.proposal_law,
            chart_scale=chart_scale,
            trials=trial,
            facts=facts,
            proposal_band=self.raw.certificate.worst_band_squared_upper,
            displacement=displacement,
            classes=self.classes,
            previous_refusals=self.previous_refusals,
            chain_records=self.records,
            selection=(
                self.selection_law,
                None if arap else facts.certificate.worst_band_squared_upper,
                facts.certificate.worst_band_squared_upper if arap else None,
                "",
            ),
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


def _chart_of(unfolding: _Unfolding) -> DevelopableChartV1:
    """Карта предложения `unfolding`, прошедшего суд до привязки, либо именованный отказ."""

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
                + unfolding.failure_text(last)
                + unfolding.proposal_note,
            )
        last = free_last
    band = unfolding.raw.certificate.worst_band_squared_upper
    steps = tuple(factor * unfolding.source_scale for factor in UNFOLD_CHART_SCALE_FACTORS)
    raise refusal(
        NamedOutcome.DEVELOPABLE_CHART_LATTICE_TOO_COARSE,
        "the unsnapped proposal is within budget "
        f"(worst_band_squared<={band.numerator / band.denominator:.9e}), but no chart "
        f"lattice step in {steps} kept it; {unfolding.failure_text(last)}"
        f"{unfolding.proposal_note}",
    )


def _band(chart: DevelopableChartV1) -> ExactRationalV1:
    return chart.certificate.stretch.worst_band_squared_upper


def _value(rational: ExactRationalV1) -> Fraction:
    return Fraction(rational.numerator, rational.denominator)


def _with_selection(chart, law, hinge_band, arap_band, arap_refusal="") -> DevelopableChartV1:
    certificate = replace(
        chart.certificate,
        proposal_selection_law=law,
        hinge_chart_worst_band_squared_upper=hinge_band,
        arap_chart_worst_band_squared_upper=arap_band,
        arap_refusal=arap_refusal,
    )
    return DevelopableChartV1(certificate, chart.nodes, chart.chart_scale)


def _best_proposal(unfolding: _Unfolding, chart: DevelopableChartV1) -> DevelopableChartV1:
    """Закон «лучшее предложение»: шарнир выше порога изометрии соревнуется с ARAP.

    Карта ARAP, которой нет (не дали положений либо отказана), не теряется: остаётся карта
    шарнира, а причина записана в сертификате. Равенство сертифицированных границ решает шарнир.
    """

    from .planar_metric import PlanarMetricAdmissionError

    law = DevelopableProposalSelectionLawV1
    if unfolding.proposal_name != "hinge":
        return chart
    hinge_band = _band(chart)
    if _value(hinge_band) <= band_bounds(DEVELOPABLE_ISOMETRIC_ENOUGH)[1]:
        return chart
    try:
        rival = _chart_of(unfolding.competing_arap())
    except ArapProposalUnavailable:
        return _with_selection(chart, law.HINGE_KEPT_ARAP_UNAVAILABLE_V1, hinge_band, None)
    except PlanarMetricAdmissionError as error:
        return _with_selection(
            chart, law.HINGE_KEPT_ARAP_REFUSED_V1, hinge_band, None, error.outcome.value
        )
    arap_band = _band(rival)
    if _value(arap_band) < _value(hinge_band):
        return _with_selection(rival, law.BEST_ARAP_WON_V1, hinge_band, arap_band)
    return _with_selection(chart, law.BEST_HINGE_WON_V1, hinge_band, arap_band)


#: Память построителя в ПРОЦЕССЕ. Карта домена — чистая функция своих входов (binary64 с фиксированным порядком
#: операций, точные дроби), а собирается она около пяти раз: построение метрики и четыре проверки снапшота
#: (выгрузка хоста, декодированные байты, запрос, компиляция), и соперник ARAP платился бы в каждой сборке. Ключ — ВСЕ
#: входы построителя (позиции, треугольники, допуск, след лестницы, прямые цепи), поэтому хит возвращает ровно ту
#: карту, что построил бы вызов; успехи запоминаются, отказы нет (они пересборки не порождают). Размер ограничен
#: давностью обращения; тесты, подменяющие внутренности построителя, сбрасывают память
#: (`clear_developable_chart_memory`).
CHART_MEMORY_ENTRIES = 16
_chart_memory: OrderedDict = OrderedDict()


def clear_developable_chart_memory() -> None:
    _chart_memory.clear()


def _memory_key(
    source_revision,
    patch_domain_id,
    snapped,
    owner_triangles,
    required_ids,
    source_scale,
    previous_refusals,
    budget,
    declared_straight_chains,
) -> tuple:
    return (
        source_revision,
        patch_domain_id,
        tuple(sorted((vertex.value, tuple(position)) for vertex, position in snapped.items())),
        tuple(sorted(owner_triangles, key=lambda item: item.triangle_id.value)),
        tuple(required_ids),
        source_scale,
        tuple(previous_refusals),
        Fraction(budget),
        tuple(tuple(chain) for chain in declared_straight_chains),
    )


def _detached(chart: DevelopableChartV1) -> DevelopableChartV1:
    """Тот же результат с собственным словарём узлов: вызывающий не портит память."""

    return DevelopableChartV1(chart.certificate, dict(chart.nodes), chart.chart_scale)


def build_developable_chart(
    *,
    source_revision,
    patch_domain_id,
    snapped,
    owner_triangles,
    required_ids,
    source_scale: int | None,
    previous_refusals: tuple[str, ...] = (),
    budget: Fraction = DEFAULT_DEVELOPABLE_STRETCH_BUDGET,
    declared_straight_chains: tuple = (),
) -> DevelopableChartV1:
    """Карта и сертификат развёртки домена (из памяти процесса, если входы те же), либо именованный отказ."""

    key = _memory_key(
        source_revision,
        patch_domain_id,
        snapped,
        owner_triangles,
        required_ids,
        source_scale,
        previous_refusals,
        budget,
        declared_straight_chains,
    )
    kept = _chart_memory.get(key)
    if kept is None:
        kept = _chart_memory[key] = _build_developable_chart(
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
        while len(_chart_memory) > CHART_MEMORY_ENTRIES:
            _chart_memory.popitem(last=False)
    else:
        _chart_memory.move_to_end(key)
    return _detached(kept)


def _build_developable_chart(
    *,
    source_revision,
    patch_domain_id,
    snapped,
    owner_triangles,
    required_ids,
    source_scale: int | None,
    previous_refusals: tuple[str, ...],
    budget: Fraction,
    declared_straight_chains: tuple,
) -> DevelopableChartV1:
    """Карта и сертификат развёртки домена, либо именованный отказ.

    `budget` — допуск растяжения запроса (законность `(0, 1/2]` проверяют публичные строители метрики
    и валидатор, а не этот внутренний: тесты изолируют переворот от растяжения большим допуском).
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
    unfolding.settle_proposal()
    return _best_proposal(unfolding, _chart_of(unfolding))
