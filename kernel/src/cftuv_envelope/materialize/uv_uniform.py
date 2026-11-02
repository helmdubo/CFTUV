"""Непрерывные аффинные UV-карты полных семантических опор запроса.

План не читает покрытие, alpha, номера регионов Layout или видимые контуры.
Физические станции не меняются: U-числитель имеет отдельную формулу
`N = K + Tx*x + Ty*y + B*r`, V-числитель остаётся физическим r.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass, replace
from fractions import Fraction

from ..exact_sqrt_sum import SqrtSumV1
from .stations import station_of


ZERO = SqrtSumV1.zero()


class UniformUvRefusal(Exception):
    """Именованный отказ UV-плана; DIRECT под новым именем не подставляется."""

    def __init__(self, name: str, detail: str = "", counters: tuple = ()):
        self.name, self.detail = name, detail
        self.counters = counters
        super().__init__(f"{name}: {detail}")


def _dot(a, b):
    return a[0] * b[0] + a[1] * b[1]


def _point(point):
    return tuple(SqrtSumV1.rational(item) for item in point)


def _reverse(owner):
    return owner[2], owner[3], owner[0], owner[1]


@dataclass(frozen=True, slots=True)
class AffineUvChartV1:
    key: tuple
    constant: SqrtSumV1
    tangent: tuple
    shear: SqrtSumV1

    def value(self, point, transverse):
        return self.constant + _dot(self.tangent, point) + self.shear * transverse

    @property
    def coefficients(self):
        return self.constant, *self.tangent, self.shear


@dataclass(frozen=True, slots=True)
class UniformUvPlanV1:
    charts: dict
    cuts: tuple
    joins: tuple

    def charts_of(self, frame):
        rid = frame.face.region_id
        owners = (frame.face.owner, *frame.face.merged_owners)
        found = {}
        for owner in owners:
            chart = self.charts.get((rid, tuple(owner)))
            if chart is None:
                raise UniformUvRefusal("UV_UNIFORM_SUPPORT_UNAVAILABLE", repr((rid, owner)))
            found[chart.key] = chart
        return tuple(found[key] for key in sorted(found))


@dataclass(frozen=True, slots=True)
class _Support:
    node: tuple
    line: object
    tangent: tuple
    anchor: SqrtSumV1 | None
    direct_constant: SqrtSumV1
    direct_tangent: tuple
    frame: str
    fan: bool


@dataclass(frozen=True, slots=True)
class _Join:
    key: tuple
    first: tuple
    second: tuple
    constant_delta: SqrtSumV1
    shear_delta: SqrtSumV1
    old_seam: bool
    source_cut: bool = False


def _unit_tangent(line, gram, direction, budget):
    a, b, q = line.a, line.b, Fraction(line.q)
    g, h, k = gram
    determinant = g * k - h * h
    if determinant <= 0 or q <= 0 or q != (k*a*a - 2*h*a*b + g*b*b) / determinant:
        raise UniformUvRefusal("UV_UNIFORM_SUPPORT_NOT_UNIT", repr((a, b, q)))
    length = g*b*b - 2*h*a*b + k*a*a
    inverse = SqrtSumV1.radical(1, 1 / length, budget)
    return (inverse.scaled(direction * (g*b-h*a)), inverse.scaled(direction * (h*b-k*a)))


def _real_support(rid, owner, line, table):
    edge = table.edge_of_owner(rid, owner)
    if edge is None:
        raise UniformUvRefusal("UV_UNIFORM_SOURCE_UNNAMED", repr((rid, owner)))
    run = table.runs[edge.run_id]
    tangent = tuple(run.inverse_length.scaled(item) for item in run.covector)
    constant = run.s_origin - _dot(tangent, _point(run.origin))
    return _Support((rid, "run", run.run_id), line, tangent, constant, constant, tangent,
                    table.frame_of_run.get(run.run_id, run.run_id), False)


def _fan_parent(rid, node, records, table):
    for corner in table.corners.get((rid, node), ()):
        for owner in (corner.incoming, corner.outgoing):
            key = owner if owner in records else _reverse(owner)
            record = records.get(key)
            if record is not None and not record.fan:
                edge = table.edge_of_owner(rid, key)
                forward = (edge.start, edge.end) == (key[:2], key[2:])
                direction = 1 if edge.along_chain_use == forward else -1
                run = table.runs[edge.run_id]
                return record, direction, station_of(run, _point(node)), corner
    raise UniformUvRefusal("UV_UNIFORM_FAN_SOURCE_UNNAMED", repr((rid, node)))


def _supports(region, table, budget):
    rid = region.region_id
    lines = {tuple(face.owner): face.line for face in region.partition.faces}
    records = {}
    for owner, line in sorted(lines.items()):
        if len(owner) == 4:
            _unit_tangent(line, table.gram, 1, budget)
            records[owner] = _real_support(rid, owner, line, table)
    for owner, line in sorted(lines.items()):
        if len(owner) != 4:
            node = owner[:2]
            parent, direction, station, corner = _fan_parent(rid, node, records, table)
            tangent = _unit_tangent(line, table.gram, direction, budget)
            records[owner] = _Support((rid, "fan", *owner), line, tangent, None, station,
                                     (ZERO, ZERO), f"fan:{corner.vertex_id}:{parent.node[-1]}", True)
    return records


def _join(first, second, key, budget, source_cut=False, check_orientation=True):
    if first.node == second.node:
        return None
    a, b = first.line, second.line
    if check_orientation:
        orientation_a = (first.tangent[0].scaled(a.b)-first.tangent[1].scaled(a.a)).sign(budget=budget)
        orientation_b = (second.tangent[0].scaled(b.b)-second.tangent[1].scaled(b.a)).sign(budget=budget)
        if orientation_a != orientation_b:
            raise UniformUvRefusal("UV_UNIFORM_SUPPORT_ORIENTATION_CONFLICT", repr(key))
    determinant = a.a*b.b - b.a*a.b
    if not determinant:
        if first.tangent != second.tangent or a.a*b.c != b.a*a.c or a.b*b.c != b.b*a.c:
            raise UniformUvRefusal("UV_UNIFORM_PARALLEL_JOIN", repr(key))
        old_seam = first.frame != second.frame and first.direct_constant != second.direct_constant
        return _Join(key, first.node, second.node, ZERO, ZERO, old_seam, source_cut)
    p0 = _point((Fraction(a.c*b.b-b.c*a.b, determinant), Fraction(a.a*b.c-b.a*a.c, determinant)))
    qa = SqrtSumV1.radical(1, Fraction(a.q), budget)
    qb = SqrtSumV1.radical(1, Fraction(b.q), budget)
    p1 = ((qa.scaled(b.b)-qb.scaled(a.b)).scaled(Fraction(1, determinant)),
          (qb.scaled(a.a)-qa.scaled(b.a)).scaled(Fraction(1, determinant)))
    delta = (first.tangent[0]-second.tangent[0], first.tangent[1]-second.tangent[1])
    direct = (first.direct_tangent[0]-second.direct_tangent[0], first.direct_tangent[1]-second.direct_tangent[1])
    old_seam = first.frame != second.frame and (
        not (first.direct_constant-second.direct_constant+_dot(direct, p0)).is_zero
        or not _dot(direct, p1).is_zero
    )
    return _Join(key, first.node, second.node, _dot(delta, p0), _dot(delta, p1),
                 old_seam and not (first.fan and second.fan), source_cut)


def _semantic_joins(prepared, region, records, table, budget):
    from .uv_phase import is_source_cut

    rid, poly = region.region_id, region.bridge.polygon
    fans = {}
    for node, ordinal, _line in poly.fan_edges():
        owner = (*node, *node, ordinal)
        if owner in records:
            fans.setdefault(node, []).append(owner)
    joins = []
    for loop_number, loop in enumerate(poly.loops):
        points = loop.points
        for number, node in enumerate(points):
            incoming = (*points[number-1], *node)
            outgoing = (*node, *points[(number+1) % len(points)])
            hidden = sorted(fans.get(node, ()), key=lambda owner: owner[-1])
            sequence = [owner for owner in (incoming, *hidden, outgoing) if owner in records]
            for left, right in zip(sequence, sequence[1:]):
                key = (rid, loop_number, *node, left, right)
                source_cut = right == outgoing and incoming in records and outgoing in records and is_source_cut(prepared, table, rid, node, incoming, outgoing)
                found = _join(records[left], records[right], key, budget, source_cut)
                if found is not None:
                    joins.append(found)
    return joins


def _path(node, parents):
    edges = []
    while parents[node] is not None:
        node, edge = parents[node]
        edges.append(edge)
    return edges


def _walk(nodes, joins, removed):
    adjacent = {node: [] for node in nodes}
    for index, join in enumerate(joins):
        if index not in removed:
            adjacent[join.first].append((join.second, index, 1))
            adjacent[join.second].append((join.first, index, -1))
    values, components = {}, []
    for root in sorted(nodes, key=lambda key: (nodes[key].anchor is None, key)):
        if root in values:
            continue
        if nodes[root].anchor is None:
            raise UniformUvRefusal("UV_UNIFORM_COMPONENT_WITHOUT_SOURCE", repr(root))
        values[root] = nodes[root].anchor, ZERO
        parents, todo, component = {root: None}, deque([root]), []
        while todo:
            current = todo.popleft()
            component.append(current)
            for other, index, sign in sorted(adjacent[current]):
                join = joins[index]
                k, b = values[current]
                proposed = (k+join.constant_delta.scaled(sign), b+join.shear_delta.scaled(sign))
                if other in values:
                    if values[other] != proposed:
                        path = set(_path(current, parents)) ^ set(_path(other, parents))
                        return None, None, ("HOLONOMY", path | {index})
                elif nodes[other].anchor is not None and proposed[0] != nodes[other].anchor:
                    return None, None, ("PHASE", set(_path(current, parents)) | {index})
                else:
                    values[other] = proposed
                    parents[other] = current, index
                    todo.append(other)
        components.append(component)
    return values, components, None


def uniform_support_graph(prepared, table, budget):
    """Полный семантический граф без alpha: вход для согласования фаз физических цепей."""
    owners, nodes, joins = {}, {}, []
    for region in sorted(prepared.regions, key=lambda item: item.region_id):
        records = _supports(region, table, budget)
        for owner, support in records.items():
            owners[(region.region_id, owner)] = support.node
            previous = nodes.setdefault(support.node, support)
            if previous.tangent != support.tangent or previous.anchor != support.anchor:
                raise UniformUvRefusal("UV_UNIFORM_SOURCE_FRAME_CONFLICT", repr(support.node))
        joins.extend(_semantic_joins(prepared, region, records, table, budget))
    joins.sort(key=lambda join: join.key)
    return owners, nodes, joins


def build_uniform_plan(prepared, table, budget, *, phase_offsets_metres, cut_ids=()) -> UniformUvPlanV1:
    """Карты с согласованными между патчами UV-фазами; физические станции и их память не меняются."""

    owners, nodes, joins = uniform_support_graph(prepared, table, budget)
    for node, support in tuple(nodes.items()):
        if support.anchor is not None:
            chain_id = table.runs[node[-1]].chain_id
            if chain_id not in phase_offsets_metres:
                raise UniformUvRefusal("UV_UNIFORM_PHASE_PLAN_MISSING", chain_id)
            nodes[node] = replace(support, anchor=support.anchor + phase_offsets_metres[chain_id].scaled(table.scale))
    removed = {index for index, join in enumerate(joins) if join.source_cut}
    reasons = {index: "UV_UNIFORM_EXISTING_SOURCE_CUT" for index in removed}
    for corner in cut_ids:
        candidates = [index for index, join in enumerate(joins) if join.key[:4] == corner and len(join.key[-1]) == 4 and join.old_seam]
        if candidates:
            chosen = min(candidates, key=lambda index: joins[index].key)
            removed.add(chosen)
            reasons[chosen] = "UV_UNIFORM_EXISTING_PHASE_CYCLE_CUT"
    while True:
        values, components, conflict = _walk(nodes, joins, removed)
        if conflict is None:
            break
        kind, conflict = conflict
        if kind == "PHASE":
            raise UniformUvRefusal("UV_UNIFORM_PHASE_PLAN_INCONSISTENT", repr(tuple(joins[index].key for index in sorted(conflict))))
        candidates = [index for index in conflict if joins[index].old_seam]
        if not candidates:
            raise UniformUvRefusal("UV_UNIFORM_HOLONOMY_OR_PHASE_CONFLICT", repr(tuple(joins[index].key for index in sorted(conflict))))
        chosen = min(candidates, key=lambda index: (nodes[joins[index].first].fan or nodes[joins[index].second].fan, joins[index].key))
        removed.add(chosen)
        reasons[chosen] = "UV_UNIFORM_EXISTING_DIRECT_SEAM_RETAINED"
    for component in components:
        low = high = values[component[0]][1]
        for node in component[1:]:
            b = values[node][1]
            if (b-low).sign(budget=budget) < 0:
                low = b
            if (b-high).sign(budget=budget) > 0:
                high = b
        shift = (low+high).scaled(Fraction(-1, 2))
        for node in component:
            k, b = values[node]
            values[node] = k, b+shift
    charts = {owner: AffineUvChartV1(node, values[node][0], nodes[node].tangent, values[node][1]) for owner, node in owners.items()}
    return UniformUvPlanV1(charts, tuple((reasons[index], joins[index].key) for index in sorted(removed)),
                           tuple(join.key for index, join in enumerate(joins) if index not in removed))


def plan_from_context(prepared, table, budget, uv_context):
    if uv_context is None:
        raise UniformUvRefusal("UV_UNIFORM_PHASE_PLAN_MISSING")
    uv_context.validate_basis(prepared, table)
    return build_uniform_plan(prepared, table, budget, phase_offsets_metres=uv_context.offsets(), cut_ids=uv_context.cut_ids)


def map_uniform_uv(plan, frame_faces, layout, polygons, points, facts, *, source_loci=None):
    """UV-числители итоговых вершин и формы всех их владельцев; физическая таблица не меняется."""

    values, forms = {}, {}
    for frame, rings in zip(frame_faces, polygons):
        charts = plan.charts_of(frame)
        region = layout.region_of(frame)
        for key in dict.fromkeys(key for ring in rings for key in ring):
            slot = region, key
            transverse = facts[slot][1]
            source = key.startswith("src:") and transverse.is_zero
            point = points[key]
            if source:
                if source_loci is None or slot not in source_loci:
                    raise UniformUvRefusal("UV_UNIFORM_SOURCE_LOCUS_MISSING", key)
                point = source_loci[slot]
            answers = tuple(chart.value(point, transverse) for chart in charts)
            if any(answer != answers[0] for answer in answers[1:]):
                raise UniformUvRefusal("UV_UNIFORM_MERGED_OWNER_CONFLICT", repr(slot))
            pair = answers[0], transverse
            if slot in values and values[slot] != pair:
                raise UniformUvRefusal("UV_UNIFORM_SLOT_CONFLICT", repr(slot))
            values[slot] = pair
            saved = forms.setdefault(slot, {})
            # Геометрическая привязка src может сдвинуть точку резки, но не фазу исходной вершины.
            saved.update((chart.key, (answers[0], ZERO, ZERO, ZERO) if source else chart.coefficients) for chart in charts)
    return values, {slot: tuple(saved[key] for key in sorted(saved)) for slot, saved in forms.items()}


def uniform_uv_slopes(slots, forms, key_slope, fact_slope, clip_points, clip_facts):
    """UV-наклоны ПОСЛЕ физической резки: новая форма никогда не перезаписывается старыми clip s-наклонами."""

    physical = {**fact_slope, **clip_facts}
    result = {}
    for slot in slots:
        label = f"region:{slot[0]}/{slot[1]}"
        if slot not in forms:
            raise UniformUvRefusal("UV_UNIFORM_SLOPE_FORM_MISSING", label)
        if slot not in physical:
            raise UniformUvRefusal("UV_UNIFORM_PHYSICAL_SLOPE_MISSING", label)
        x1, y1 = clip_points.get(slot[1], key_slope.get(slot[1])) or (ZERO, ZERO)
        r1 = physical[slot][1]
        values = tuple(tx*x1 + ty*y1 + b*r1 for _k, tx, ty, b in forms[slot])
        if not values or any(value != values[0] for value in values[1:]):
            raise UniformUvRefusal("UV_UNIFORM_SLOPE_OWNER_CONFLICT", label)
        result[slot] = values[0], r1
    return result
