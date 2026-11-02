"""Неизменяемые UV-фазы физических цепей одного полного запроса.

Факты собираются из существующего station oracle ДО среза материализации.
Сдвиг хранится в метрах и одинаков у обоих Patch-side uses цепи. Ни station
IR, ни подготовка геометрии не меняются; alpha в план не входит.
"""

from __future__ import annotations

import hashlib
import json
from dataclasses import dataclass, field, replace
from fractions import Fraction

from ..exact_sqrt_sum import SqrtSumV1, ExactCanonicalizationWorkBudgetExhausted, exact_work_budget
from .stations import chain_station_table, source_vertex_name, station_of
from .uv_uniform import UniformUvRefusal, ZERO, _join, _point, _supports


def _canonical(value):
    if isinstance(value, SqrtSumV1):
        return ["sqrt", [[root, _canonical(coefficient)] for root, coefficient in value.terms]]
    if isinstance(value, Fraction):
        return ["rat", value.numerator, value.denominator]
    if isinstance(value, (tuple, list)):
        return [_canonical(item) for item in value]
    return value


def _digest(value):
    return hashlib.sha256(json.dumps(_canonical(value), ensure_ascii=True, separators=(",", ":")).encode("utf-8")).hexdigest()


def _text(value):
    return getattr(value, "value", value)


def station_basis(prepared, table) -> tuple[str, str]:
    """Идентичность точного alpha-независимого station/metric basis конкретного домена."""

    key = _text(prepared.compilation.plan_key.patch_domain_id)
    supports = tuple((region.region_id, tuple(face.owner), face.line.a, face.line.b, face.line.c, Fraction(face.line.q))
                     for region in sorted(prepared.regions, key=lambda item: item.region_id)
                     for face in sorted(region.partition.faces, key=lambda face: tuple(face.owner)))
    runs = tuple((run.run_id, run.chain_id, run.origin, run.direction, run.s_origin, run.inverse_length, run.covector)
                 for run in sorted(table.runs.values(), key=lambda run: run.run_id))
    return key, _digest(("UV_STATION_BASIS_V1", table.scale, table.gram, supports, runs, table.cuts, tuple(sorted(table.joins.items()))))


def is_source_cut(prepared, table, rid, node, left, right):
    """Только фактический closer→opener endpoint из station table, не вся пара регионов."""

    uses = { _text(key): use for key, use in prepared.context.uses_by_id.items() }
    edge_a, edge_b = table.edge_of_owner(rid, left), table.edge_of_owner(rid, right)
    pair = {edge_a.chain_use_id, edge_b.chain_use_id}
    corners = table.corners.get((rid, node), ())
    vertices = {source_vertex_name(corner.vertex_id) for corner in corners}
    for _flow, closer, opener, _chain in table.cuts:
        if pair != {closer, opener}:
            continue
        before = prepared.context.directed_chain_vertices(uses[closer])
        after = prepared.context.directed_chain_vertices(uses[opener])
        if before and after and source_vertex_name(_text(before[-1])) == source_vertex_name(_text(after[0])) and source_vertex_name(_text(before[-1])) in vertices:
            return True
    return False


@dataclass(frozen=True, slots=True)
class UniformUvPhaseLinkV1:
    key: tuple
    left: str
    right: str
    delta_metres: SqrtSumV1
    fan: bool
    old_seam: bool
    source_cut: bool = False


@dataclass(frozen=True, slots=True)
class UniformUvPhaseFactsV1:
    basis: tuple[str, str]
    chains: tuple[str, ...]
    links: tuple[UniformUvPhaseLinkV1, ...]


def collect_phase_facts(prepared, table, budget) -> UniformUvPhaseFactsV1:
    """Истинные endpoint станции всех активных соседних опор, включая скрытые на текущей alpha."""

    links, chains = [], set()
    for region in sorted(prepared.regions, key=lambda item: item.region_id):
        rid, poly = region.region_id, region.bridge.polygon
        records = _supports(region, table, budget)
        chains.update(table.runs[support.node[-1]].chain_id for support in records.values() if not support.fan)
        fans = {}
        for node, ordinal, _line in poly.fan_edges():
            owner = (*node, *node, ordinal)
            if owner in records:
                fans.setdefault(node, []).append(owner)
        for loop_number, loop in enumerate(poly.loops):
            for index, node in enumerate(loop.points):
                left = (*loop.points[index-1], *node)
                right = (*node, *loop.points[(index+1) % len(loop.points)])
                if left not in records or right not in records:
                    continue
                a = table.runs[records[left].node[-1]]
                b = table.runs[records[right].node[-1]]
                hidden = sorted(fans.get(node, ()), key=lambda owner: owner[-1])
                # Разрез при конфликте фазы допустим лишь на доказанном старом шве: fan→outgoing, а не внутри веера.
                boundary = _join(records[hidden[-1] if hidden else left], records[right], (rid, loop_number, *node), budget, check_orientation=False)
                old_seam = boundary is not None and boundary.old_seam
                source_cut = is_source_cut(prepared, table, rid, node, left, right)
                delta = (station_of(a, _point(node))-station_of(b, _point(node))).scaled(Fraction(1, table.scale))
                links.append(UniformUvPhaseLinkV1((rid, loop_number, *node), a.chain_id, b.chain_id, delta,
                                                 bool(hidden), old_seam, source_cut))
    return UniformUvPhaseFactsV1(station_basis(prepared, table), tuple(sorted(chains)), tuple(sorted(links, key=lambda link: link.key)))


@dataclass(frozen=True, slots=True)
class UniformUvPhasePlanV1:
    offsets_metres: tuple
    cut_ids: tuple
    basis: tuple
    digest: str
    work_counters: tuple = field(default=(), compare=False, kw_only=True)
    #: Домены, чья подготовка в базисе пришла из повтора лотереи привязки (UNIFORM_UV_PER_DOMAIN_REFUSAL_V1): их
    #: несовпадение базиса на обычной подготовке хост вправе отправить на тот же повтор. Пусто - прежний digest.
    retried_domain_ids: tuple = ()

    def offsets(self):
        return dict(self.offsets_metres)

    def excluded_reason(self, domain_id):
        """Имя, под которым домен исключён из плана фаз, или `None`."""

        return dict(self.basis).get(f"excluded:{domain_id}")

    def validate_basis(self, prepared, table):
        self.validate_basis_value(*station_basis(prepared, table))

    def validate_basis_value(self, key, expected):
        """`validate_basis` по готовой паре `station_basis` (её помнит память подготовки)."""

        found = dict(self.basis).get(key)
        if found is None and self.excluded_reason(key) is not None:
            # Домен исключён из контекста по имени: его отказ - собственный, а не «чужой базис».
            raise UniformUvRefusal("UV_UNIFORM_PHASE_DOMAIN_EXCLUDED", f"{key}: {self.excluded_reason(key)}")
        if found != expected:
            raise UniformUvRefusal("UV_UNIFORM_PHASE_BASIS_MISMATCH", key)


def _refusal_of(name, detail, domain_id):
    refusal = UniformUvRefusal(name, detail)
    refusal.domain_id = domain_id
    return refusal


def compile_uniform_phase_plan(facts, *, excluded_domain_ids=(), retried_domain_ids=()) -> UniformUvPhasePlanV1:
    """Точный weighted forest: P_right-P_left=delta; циклы режутся только по доказанным старым швам.

    Конфликт называет домен-виновник (`UniformUvRefusal.domain_id`): `compile_uniform_uv_context` исключает его и
    собирает план заново, поэтому один домен не роняет весь запрос.
    """

    bases, links, chains, owner = {}, {}, set(), {}
    for item in facts:
        key, digest = item.basis
        if key in bases and bases[key] != digest:
            raise _refusal_of("UV_UNIFORM_PHASE_BASIS_CONFLICT", key, key)
        bases[key] = digest
        chains.update(item.chains)
        for link in item.links:
            if link.key in links and links[link.key] != link:
                raise _refusal_of("UV_UNIFORM_PHASE_LINK_CONFLICT", repr(link.key), key)
            links[link.key] = link
            owner.setdefault(link.key, key)
    for domain_id, outcome in sorted(excluded_domain_ids):
        key = f"excluded:{domain_id}"
        if key in bases and bases[key] != outcome:
            raise _refusal_of("UV_UNIFORM_PHASE_BASIS_CONFLICT", key, None)
        bases[key] = str(outcome)
    parents = {chain: chain for chain in chains}
    weights = {chain: ZERO for chain in chains}

    def find(chain):
        shift = ZERO
        while parents[chain] != chain:
            shift = shift+weights[chain]
            chain = parents[chain]
        return chain, shift

    cuts = []
    for link in sorted(links.values(), key=lambda link: (link.old_seam, not link.fan, link.key)):
        if link.source_cut:
            cuts.append(link.key)
            continue
        root_a, a = find(link.left)
        root_b, b = find(link.right)
        if root_a == root_b:
            if b-a != link.delta_metres:
                if not link.old_seam:
                    raise _refusal_of("UV_UNIFORM_PHASE_CYCLE_WITHOUT_OLD_CUT", repr(link.key), owner.get(link.key))
                cuts.append(link.key)
            continue
        delta = link.delta_metres+a-b
        if root_a < root_b:
            parents[root_b], weights[root_b] = root_a, delta
        else:
            parents[root_a], weights[root_a] = root_b, -delta
    offsets = tuple((chain, find(chain)[1]) for chain in sorted(chains))
    cut_ids, basis = tuple(sorted(cuts)), tuple(sorted(bases.items()))
    retried = tuple(sorted({str(item) for item in retried_domain_ids}))
    identity = ("UV_UNIFORM_PHASE_PLAN_V1", offsets, cut_ids, basis) + ((("retried", retried),) if retried else ())
    return UniformUvPhasePlanV1(offsets, cut_ids, basis, _digest(identity), retried_domain_ids=retried)


def _add_counters(total, counters):
    for name, value in counters:
        total[name] = total.get(name, 0) + value


def compile_uniform_uv_context(preparations, *, excluded_domain_ids=(), work_budget=None, retried_domain_ids=()) -> UniformUvPhasePlanV1:
    """План ВСЕХ статических подготовок запроса до нарезки задач: alpha и видимых граней здесь нет.

    UNIFORM_UV_PER_DOMAIN_REFUSAL_V1: отказ одного домена (опора без имени, исчерпанный бюджет, конфликт фаз) не бросается
    наружу - домен входит в `excluded` под своим именем, и его материализация откажет именованно
    (`UV_UNIFORM_PHASE_DOMAIN_EXCLUDED`), остальные строятся. Бюджет - свой у каждого домена, если вызывающий не дал общий.
    """

    excluded = {str(domain): str(name) for domain, name in excluded_domain_ids}
    facts, totals = {}, {}
    for prepared in sorted(preparations, key=lambda item: _text(item.compilation.plan_key.patch_domain_id)):
        domain_id = _text(prepared.compilation.plan_key.patch_domain_id)
        if domain_id in excluded:
            continue
        budget = exact_work_budget(stage="UV_PHASE", domain_id=domain_id) if work_budget is None else work_budget
        try:
            table = chain_station_table(prepared, budget, withdrawn=frozenset())
            found = collect_phase_facts(prepared, table, budget)
        except ExactCanonicalizationWorkBudgetExhausted:
            excluded[domain_id] = "UV_UNIFORM_PHASE_WORK_BUDGET_EXHAUSTED"
            found = None
        except UniformUvRefusal as refusal:
            excluded[domain_id] = refusal.name
            found = None
        if work_budget is None:
            _add_counters(totals, budget.counters())
        if found is None:
            facts.pop(domain_id, None)
        elif domain_id in facts and facts[domain_id] != found:
            excluded[domain_id] = "UV_UNIFORM_PHASE_BASIS_CONFLICT"
            facts.pop(domain_id)
        else:
            facts[domain_id] = found
    while True:
        try:
            context = compile_uniform_phase_plan(
                [item for domain, item in sorted(facts.items()) if domain not in excluded],
                excluded_domain_ids=tuple(excluded.items()), retried_domain_ids=retried_domain_ids,
            )
            break
        except UniformUvRefusal as refusal:
            guilty = getattr(refusal, "domain_id", None)
            if guilty is None or guilty not in facts or guilty in excluded:
                raise
            excluded[guilty] = refusal.name
    counters = tuple(sorted(totals.items())) if work_budget is None else work_budget.counters()
    return replace(context, work_counters=counters)
