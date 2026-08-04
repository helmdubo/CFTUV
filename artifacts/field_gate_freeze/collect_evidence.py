"""Свидетельство отказа: то, чего НЕТ в строке «граней 0».

Исследовательский слой. Production `wavefront/faces.py` не тронут ни строкой:
здесь ЧИТАЮТСЯ его же публичные помощники (`polygon_fronts`,
`edge_neighbours`, `face_chain`, `validate_multiway_node`) и повторяется цикл
`build_faces` — но БЕЗ его правила «первый неудавшийся регион кончает всё».
Production остаётся fail-closed; исследование обязано досчитать до конца.

Что собирается и зачем именно это.

* СОБРАННЫЕ И УПАВШИЕ OWNERS. `build_faces` возвращается на ПЕРВОМ несобранном
  фронте и выбрасывает уже собранный префикс, поэтому наружу выходит «0
  граней» — число, которое читается как «не собралось ничего», хотя собралось
  35 из 47. Ремонт, начатый с этого числа, искал бы глобальную поломку там,
  где живут четыре симметричные локальные.
* PLACE-ГРАФ КАЖДОГО УПАВШЕГО OWNER: точки, партнёры, степени вершин, heads и
  tails. `face_chain` называет ПРИЧИНУ строкой («участников в трёх и более
  точках»), но не показывает граф, на котором это видно.
* MULTIWAY-УЗЛЫ с их `kinds` и `incidences` — потому что именно в них
  транзакция сворачивает смешанный контакт, и кратность incidences здесь
  измеряется, а не предполагается.
* ПЛАН ТРАНЗАКЦИИ В ТЕХ ЖЕ ЛОКУСАХ: `birth_wiring`, `existing_port_rewrites`,
  цепочки смертей, `birth_components`, `terminal_birth_cycles`. Транзакция это
  УЖЕ вычислила точно и выбросила, оставив сборщику граней только множества
  участников; свидетельство фиксирует, что именно было выброшено.

CLI: `collect_evidence.py <каталог вывода>`.
"""

from __future__ import annotations

import json
from pathlib import Path
import sys

HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

from field_route import (  # noqa: E402
    SNAPSHOTS,
    bundle_from_field_snapshot,
    selected_edge_ids,
    snapshot_sha256,
    staged_domains,
    terms,
)

FIELD_ALPHA = 0.254
FIELD_DENSITY = 0


def dump_point(point) -> dict:
    return {"x": terms(point.x), "y": terms(point.y)}


def dump_time(time) -> dict:
    time = time.canonical()
    return {
        "dividend": [
            int(time.dividend.numerator),
            int(time.dividend.denominator),
        ],
        "divisor": terms(time.divisor),
    }


def approx(value) -> float:
    import math

    return sum(
        (c.numerator / c.denominator) * math.sqrt(radicand)
        for radicand, c in value.terms
    )


def approx_time(time) -> float:
    time = time.canonical()
    return float(time.dividend) / approx(time.divisor)


def prepare(snapshot_name: str, *, spy_plans: bool):
    """Полный маршрут до `prepare_conveyor`, с записью планов транзакции."""

    payload, _bm, bundle = bundle_from_field_snapshot(
        SNAPSHOTS / snapshot_name
    )
    entries = staged_domains(
        bundle,
        selected_edge_ids(payload),
        alpha=FIELD_ALPHA,
        density=FIELD_DENSITY,
    )
    ready = [entry for entry in entries if entry["ready"] is not None]
    patch_id, domain_id, snapshot, request = ready[0]["ready"]

    plans: list = []
    if spy_plans:
        from cftuv_envelope.wavefront import superlevel

        original = superlevel._emit_component_nodes

        def spy(builder, plan):
            before = len(builder.nodes)
            original(builder, plan)
            plans.append((plan, tuple(range(before, len(builder.nodes)))))

        superlevel._emit_component_nodes = spy
        try:
            from cftuv_envelope.wavefront import prepare_conveyor

            prepared = prepare_conveyor(snapshot, request)
        finally:
            superlevel._emit_component_nodes = original
    else:
        from cftuv_envelope.wavefront import prepare_conveyor

        prepared = prepare_conveyor(snapshot, request)
    return domain_id, prepared, plans


def face_assembly_evidence(snapshot_name: str) -> dict:
    """По-owner'ный разбор сборки граней. Досчитывает ВСЕ фронты."""

    from cftuv_envelope.wavefront.digest import validate_multiway_node
    from cftuv_envelope.wavefront.events import EventKind
    from cftuv_envelope.wavefront.faces import (
        edge_neighbours,
        polygon_fronts,
        face_chain,
    )

    domain_id, prepared, plans = prepare(snapshot_name, spy_plans=True)
    region = prepared.regions[0]
    skeleton, polygon = region.skeleton, region.bridge.polygon

    nodes_by_key: dict = {}
    multiway: list = []
    for index, node in enumerate(skeleton.nodes):
        if node.kind is EventKind.MULTIWAY:
            _, incidences = validate_multiway_node(node)
            multiway.append(
                {
                    "node_index": index,
                    "time_approx": round(approx_time(node.time), 4),
                    "point_approx": [
                        round(approx(node.point.x), 3),
                        round(approx(node.point.y), 3),
                    ],
                    "time": dump_time(node.time),
                    "point": dump_point(node.point),
                    "kinds": [kind.value for kind in node.kinds],
                    "participants": [
                        [int(v) for v in key] for key in node.participants
                    ],
                    "incidences": [
                        [[int(v) for v in key] for key in incidence]
                        for incidence in incidences
                    ],
                    "distinct_incidences": len(
                        {repr(incidence) for incidence in incidences}
                    ),
                    "converging_vertices": int(node.converging_vertices),
                }
            )
        else:
            incidences = (node.participants,)
        for incidence in incidences:
            for key in incidence:
                nodes_by_key.setdefault(key, []).append((index, node))

    neighbours = edge_neighbours(polygon)
    owner_spec = dict(region.owner_by_edge)
    owners = []
    for key, start, end, line in polygon_fronts(polygon):
        row = {
            "owner": [int(v) for v in key],
            "start": [int(v) for v in start],
            "end": [int(v) for v in end],
            "owner_spec": owner_spec.get(key),
        }
        if line.is_stationary:
            row["verdict"] = "WALL_NO_FACE"
            owners.append(row)
            continue
        seats = nodes_by_key.get(key, ())
        row["node_indices"] = [index for index, _ in seats]
        if not seats:
            row["verdict"] = "FACE_HAS_NO_SKELETON_NODE"
            owners.append(row)
            continue
        previous, following = neighbours[key]
        row["previous"] = [int(v) for v in previous]
        row["following"] = [int(v) for v in following]
        chain, why = face_chain(
            key, tuple(node for _, node in seats), previous, following
        )
        row["verdict"] = (
            "ASSEMBLED" if chain is not None else "FACE_CHAIN_DOES_NOT_CLOSE"
        )
        row["why"] = why

        places: dict = {}
        for index, node in seats:
            places.setdefault(
                json.dumps(dump_point(node.point), sort_keys=True), []
            ).append((index, node))
        order = list(places)
        partners = [
            sorted(
                {
                    participant
                    for _, node in places[place]
                    for participant in node.participants
                    if participant != key
                }
            )
            for place in order
        ]
        shared: dict = {}
        for place_index, group in enumerate(partners):
            for participant in group:
                shared.setdefault(participant, []).append(place_index)
        links = {index: set() for index in range(len(order))}
        for occupied in shared.values():
            if len(occupied) == 2:
                first, second = occupied
                links[first].add(second)
                links[second].add(first)
        heads = [i for i, group in enumerate(partners) if following in group]
        tails = [i for i, group in enumerate(partners) if previous in group]
        # Точные термы точек выписываются у УПАВШИХ owners; у собравшихся
        # хватает приближения — их локусы уже заморожены якорной таблицей
        # (`anchor_loci.json`), а полный дамп всех 47 фронтов в точных суммах
        # корней перевалил бы за 512 КБ и упёрся в правило крупных файлов.
        exact_points = chain is None
        row["places"] = [
            {
                "place_index": index,
                "point": (
                    dump_point(places[order[index]][0][1].point)
                    if exact_points
                    else None
                ),
                "point_approx": [
                    round(approx(places[order[index]][0][1].point.x), 3),
                    round(approx(places[order[index]][0][1].point.y), 3),
                ],
                "time_approx": round(
                    approx_time(places[order[index]][0][1].time), 4
                ),
                "node_indices": [i for i, _ in places[order[index]]],
                "kinds": sorted(
                    {node.kind.value for _, node in places[order[index]]}
                ),
                "partners": [
                    [int(v) for v in participant]
                    for participant in partners[index]
                ],
                "degree": len(links[index]),
                "linked_to": sorted(links[index]),
                "is_head": index in heads,
                "is_tail": index in tails,
            }
            for index in range(len(order))
        ]
        row["heads"] = heads
        row["tails"] = tails
        row["crowded_participants"] = [
            {"participant": [int(v) for v in participant], "seats": occupied}
            for participant, occupied in sorted(shared.items())
            if len(occupied) > 2
        ]
        row["degree_histogram"] = {
            str(degree): sum(
                1 for index in links if len(links[index]) == degree
            )
            for degree in sorted({len(v) for v in links.values()})
        }
        owners.append(row)

    def reference(value) -> dict:
        return {"existing": value.existing, "birth_key": repr(value.birth_key)}

    # ПОЛНЫЙ wiring выписывается только у планов В ТЕХ ЖЕ ЛОКУСАХ, что упавшие
    # owners: `birth_key` — символьная запись длиной в сотни знаков, и полный
    # дамп всех 25 планов раздувает свидетельство до полумегабайта, где 90 %
    # относятся к фронтам, которые собрались. У остальных планов остаётся
    # сводка — состав, смерти, размеры wiring, — по которой видно, что они
    # есть и чем отличаются.
    failing_keys = {
        tuple(row["owner"])
        for row in owners
        if row["verdict"] not in ("ASSEMBLED", "WALL_NO_FACE")
    }

    def touches_failure(plan) -> bool:
        return any(
            tuple(int(v) for v in key) in failing_keys
            for key in plan.participants
        )

    plan_rows = [
        {
            "raw_node_indices": list(node_range),
            "wiring_is_full": touches_failure(plan),
            "time_approx": round(approx_time(plan.time), 4),
            "point_approx": [
                round(approx(plan.point.x), 3),
                round(approx(plan.point.y), 3),
            ],
            "time": dump_time(plan.time),
            "point": dump_point(plan.point),
            "resolution": plan.resolution.value,
            "event_kinds": [kind.value for kind in plan.event_kinds],
            "participants": [
                [int(v) for v in key] for key in plan.participants
            ],
            "target_participants": [
                [int(v) for v in key] for key in plan.target_participants
            ],
            "dead_vertex_ids": [int(v) for v in plan.dead_vertex_ids],
            "death_chains": [[int(v) for v in c] for c in plan.chains],
            "birth_wiring_count": len(plan.birth_wiring),
            "existing_port_rewrite_count": len(plan.existing_port_rewrites),
            "birth_component_sizes": [
                len(component) for component in plan.birth_components
            ],
            "terminal_birth_cycle_sizes": [
                len(cycle) for cycle in plan.terminal_birth_cycles
            ],
            "birth_wiring": (
                [
                    [repr(key), reference(first), reference(second)]
                    for key, first, second in plan.birth_wiring
                ]
                if touches_failure(plan)
                else None
            ),
            "existing_port_rewrites": (
                [
                    [int(ident), repr(before), repr(after)]
                    for ident, before, after in plan.existing_port_rewrites
                ]
                if touches_failure(plan)
                else None
            ),
            "birth_components": (
                [
                    [repr(key) for key in component]
                    for component in plan.birth_components
                ]
                if touches_failure(plan)
                else None
            ),
            "terminal_birth_cycles": (
                [
                    [repr(key) for key in cycle]
                    for cycle in plan.terminal_birth_cycles
                ]
                if touches_failure(plan)
                else None
            ),
            "edge_contacts": len(plan.edge_contacts),
            "split_cuts": len(plan.split_cuts),
            "vertex_meetings": len(plan.vertex_meetings),
            "closed_chain_count": int(plan.closed_chain_count),
        }
        for plan, node_range in plans
    ]

    assembled = [r for r in owners if r["verdict"] == "ASSEMBLED"]
    failed = [
        r for r in owners if r["verdict"] not in ("ASSEMBLED", "WALL_NO_FACE")
    ]
    walls = [r for r in owners if r["verdict"] == "WALL_NO_FACE"]
    return {
        "snapshot": snapshot_name,
        "sha256": snapshot_sha256(snapshot_name),
        "alpha": FIELD_ALPHA,
        "density": FIELD_DENSITY,
        "domain_id": domain_id,
        "region_id": region.region_id,
        "conveyor_outcome": prepared.outcome.value,
        "conveyor_detail": prepared.detail,
        "faces_reported_by_production": (
            0 if region.partition is None else len(region.partition.faces)
        ),
        "face_outcome": (
            None if region.partition is None else region.partition.outcome.value
        ),
        "face_detail_dropped_by_conveyor": (
            None if region.partition is None else region.partition.detail
        ),
        "owner_totals": {
            "fronts": len(owners),
            "assembled": len(assembled),
            "failed": len(failed),
            "walls": len(walls),
        },
        "failed_owners": [
            {"owner": r["owner"], "why": r["why"]} for r in failed
        ],
        "owners": owners,
        "multiway_nodes": multiway,
        "transaction_plans": plan_rows,
    }


def walls_012_break(snapshot_name: str = "walls_012_snapshot.json") -> dict:
    """Точка обрыва распространения: уровни, причина, конфликтующая пара."""

    from cftuv_envelope.wavefront import skeleton as skeleton_module
    from cftuv_envelope.wavefront import superlevel_fixed_point as fixed_point

    levels: list = []
    conflicts: list = []

    original_level = skeleton_module._Builder._apply_level
    original_merge = fixed_point.merge_symbolic_split_contacts

    def spy_level(self, level):
        entry = {
            "time": dump_time(level[0].time),
            "time_approx": approx_time(level[0].time),
            "size": len(level),
            "event_kinds": sorted(event.kind.value for event in level),
            "event_points": sorted(
                json.dumps(dump_point(event.point), sort_keys=True)
                for event in level
            ),
            "event_points_approx": sorted(
                [round(approx(event.point.x), 3), round(approx(event.point.y), 3)]
                for event in level
            ),
            "nodes_before": len(self.nodes),
            "alive_before": sum(1 for v in self.vertices if v.alive),
        }
        result = original_level(self, level)
        entry["nodes_after"] = len(self.nodes)
        entry["alive_after"] = sum(1 for v in self.vertices if v.alive)
        entry["refusal"] = None if self.refusal is None else self.refusal.value
        levels.append(entry)
        return result

    def spy_merge(*groups):
        seen: dict = {}
        for contact in (item for group in groups for item in group):
            previous = seen.get(contact.key)
            if previous is not None and previous != contact:
                differing = [
                    field
                    for field in ("time", "point", "projection", "leaf")
                    if getattr(previous, field) != getattr(contact, field)
                ]
                conflicts.append(
                    {
                        "key_time": repr(contact.key.time_key),
                        "key_point": repr(contact.key.point_key),
                        "emitter": repr(contact.key.emitter),
                        "family": repr(contact.key.family),
                        "participants": [
                            [int(v) for v in key]
                            for key in contact.key.participants
                        ],
                        "differing_fields": differing,
                        "first": {
                            field: repr(getattr(previous, field))
                            for field in differing
                        },
                        "second": {
                            field: repr(getattr(contact, field))
                            for field in differing
                        },
                    }
                )
            seen[contact.key] = contact
        return original_merge(*groups)

    # Слежку надо ставить у КАЖДОГО импортёра, а не только в модуле-владельце:
    # `merge_symbolic_split_contacts` втянут по имени (`from ... import ...`),
    # поэтому подмена одного `superlevel_fixed_point` вызовы не перехватывает.
    # Это не тонкость стиля: без второй и третьей подмены свидетельство
    # молча вернуло бы «конфликтов 0» на вершине, которая ими и падает.
    from cftuv_envelope.wavefront import symbolic_mixed_generation
    from cftuv_envelope.wavefront import symbolic_superlevel_coordinator

    importers = (
        fixed_point,
        symbolic_mixed_generation,
        symbolic_superlevel_coordinator,
    )
    skeleton_module._Builder._apply_level = spy_level
    for module in importers:
        module.merge_symbolic_split_contacts = spy_merge
    try:
        domain_id, prepared, _ = prepare(snapshot_name, spy_plans=False)
    finally:
        skeleton_module._Builder._apply_level = original_level
        for module in importers:
            module.merge_symbolic_split_contacts = original_merge

    region = prepared.regions[0]
    skeleton = region.skeleton
    return {
        "snapshot": snapshot_name,
        "sha256": snapshot_sha256(snapshot_name),
        "alpha": FIELD_ALPHA,
        "density": FIELD_DENSITY,
        "domain_id": domain_id,
        "region_id": region.region_id,
        "conveyor_outcome": prepared.outcome.value,
        "conveyor_detail": prepared.detail,
        "skeleton_outcome": skeleton.outcome.value,
        "skeleton_nodes": len(skeleton.nodes),
        "skeleton_counters": {
            name: int(value) for name, value in skeleton.counters
        },
        "levels": levels,
        "merge_conflicts": conflicts,
        "conflict_site": (
            "kernel/src/cftuv_envelope/wavefront/superlevel_fixed_point.py:"
            "merge_symbolic_split_contacts"
        ),
    }


def _main() -> None:
    out = Path(sys.argv[1] if len(sys.argv) > 1 else HERE)
    out.mkdir(parents=True, exist_ok=True)

    evidence = face_assembly_evidence("wall_2_001_snapshot.json")
    (out / "face_assembly_evidence.json").write_text(
        json.dumps(evidence, ensure_ascii=False, indent=1), encoding="utf-8"
    )
    print(
        "face_assembly_evidence:",
        evidence["owner_totals"],
        "|",
        evidence["face_detail_dropped_by_conveyor"],
    )

    broken = walls_012_break()
    (out / "walls_012_break.json").write_text(
        json.dumps(broken, ensure_ascii=False, indent=1), encoding="utf-8"
    )
    print(
        "walls_012_break:",
        broken["skeleton_outcome"],
        broken["skeleton_nodes"],
        "узла, конфликтов:",
        len(broken["merge_conflicts"]),
    )


if __name__ == "__main__":
    _main()
