"""One order-free topology delta per mixed junction component."""

from __future__ import annotations

from math import gcd

from .symbolic_component import (
    SymbolicComponentDeltaV1,
    apply_component_deltas,
    delta_resources,
    normalize_dead_component,
)
from .symbolic_edge_fixed_point import _valid_contact as _valid_edge
from .symbolic_junction_contacts import contact_identity
from .symbolic_overlay import JunctionRefV1


def _time_point(contact):
    return contact.key.time_key, contact.key.point_key


def _resources(overlay, contact):
    result = {("VERTEX_WRITE", ref) for ref in contact.dead_refs}
    for ref in contact.dead_refs:
        vertex = overlay.vertices.get(ref)
        if vertex is not None:
            result.update((
                ("VERTEX_WRITE", vertex.prev),
                ("VERTEX_WRITE", vertex.next),
            ))
    result.update(("FAMILY", family) for family in contact.families)
    return result


def _components(overlay, contacts):
    pending = list(sorted(contacts, key=lambda item: repr(contact_identity(item))))
    result = []
    while pending:
        component = [pending.pop(0)]
        resources = _resources(overlay, component[0])
        time_point = _time_point(component[0])
        changed = True
        while changed:
            changed, rest = False, []
            for contact in pending:
                if (_time_point(contact) == time_point
                        and resources.intersection(_resources(overlay, contact))):
                    component.append(contact)
                    resources.update(_resources(overlay, contact))
                    changed = True
                else:
                    rest.append(contact)
            pending = rest
        result.append(tuple(sorted(
            component, key=lambda item: repr(contact_identity(item))
        )))
    return tuple(result)


def _valid_endpoint(overlay, key):
    emitter = overlay.vertices.get(key.emitter)
    endpoint = overlay.vertices.get(key.endpoint)
    if emitter is None or endpoint is None or not emitter.alive or not endpoint.alive:
        return False
    return any(
        leaf.family == key.family
        and key.endpoint in (binding.start, binding.end)
        for leaf, binding in overlay.spans.items()
    )


def _ray(builder, overlay, leaf, *, incoming):
    line = builder.edges[overlay.spans[leaf].physical_edge_id].line
    dx, dy = line.b, -line.a
    common = gcd(abs(dx), abs(dy))
    ray = (dx // common, dy // common)
    return (-ray[0], -ray[1]) if incoming else ray


def _multi_delta(builder, overlay, component):
    dead = {ref for item in component for ref in item.dead_refs}
    if any(ref not in overlay.vertices or not overlay.vertices[ref].alive
           for ref in dead):
        return None, "SYMBOLIC_JUNCTION_CONTACT_STALE"
    incoming = [
        (overlay.vertices[ref].prev, overlay.vertices[ref].prev_leaf, ref)
        for ref in dead if overlay.vertices[ref].prev not in dead
    ]
    outgoing = [
        (overlay.vertices[ref].next, overlay.vertices[ref].next_leaf, ref)
        for ref in dead if overlay.vertices[ref].next not in dead
    ]
    if any(
        ref not in overlay.vertices or not overlay.vertices[ref].alive
        for ref, _, _ in (*incoming, *outgoing)
    ):
        return None, "SYMBOLIC_JUNCTION_PORT_MATCHING_AMBIGUOUS"
    keys = tuple(sorted((contact_identity(item) for item in component), key=repr))
    if (len(incoming), len(outgoing)) in ((0, 0), (1, 1)):
        return normalize_dead_component(
            overlay, contact_keys=keys, dead_refs=dead,
            point_key=component[0].key.point_key, birth_kind="JUNCTION",
            stale_reason="SYMBOLIC_JUNCTION_CONTACT_STALE",
            ambiguity_reason="SYMBOLIC_JUNCTION_PORT_MATCHING_AMBIGUOUS",
        )
    if len(incoming) != len(outgoing):
        return None, "SYMBOLIC_JUNCTION_PORT_MATCHING_AMBIGUOUS"
    pairs = []
    if len(dead) == 2 and len(incoming) == 2:
        outgoing_by_dead = {item[2]: item for item in outgoing}
        if len(outgoing_by_dead) != 2:
            return None, "SYMBOLIC_JUNCTION_PORT_MATCHING_AMBIGUOUS"
        for item in incoming:
            peer = next(ref for ref in dead if ref != item[2])
            pairs.append((item, outgoing_by_dead[peer]))
    else:
        incoming_by_ray, outgoing_by_ray = {}, {}
        for item in incoming:
            incoming_by_ray.setdefault(
                _ray(builder, overlay, item[1], incoming=True), []
            ).append(item)
        for item in outgoing:
            outgoing_by_ray.setdefault(
                _ray(builder, overlay, item[1], incoming=False), []
            ).append(item)
        if any(len(items) != 1 for items in (*incoming_by_ray.values(),
                                             *outgoing_by_ray.values())):
            return None, "SYMBOLIC_JUNCTION_PORT_MATCHING_AMBIGUOUS"
        for ray in sorted(set(incoming_by_ray) & set(outgoing_by_ray)):
            pairs.append((incoming_by_ray.pop(ray)[0], outgoing_by_ray.pop(ray)[0]))
        rest_in = [item for items in incoming_by_ray.values() for item in items]
        rest_out = [item for items in outgoing_by_ray.values() for item in items]
        if len(rest_in) != len(rest_out) or len(rest_in) > 1:
            return None, "SYMBOLIC_JUNCTION_PORT_MATCHING_AMBIGUOUS"
        if rest_in:
            pairs.append((rest_in[0], rest_out[0]))
    rewires = tuple(sorted((
        ((first[0], first[1]), (second[0], second[1]), JunctionRefV1(
            "JUNCTION", (keys, first[1], second[1])
        ))
        for first, second in pairs
    ), key=repr))
    leaves = frozenset(
        leaf for ref in dead for leaf in (
            overlay.vertices[ref].prev_leaf, overlay.vertices[ref].next_leaf
        )
    )
    return SymbolicComponentDeltaV1(
        keys, tuple(sorted(dead, key=repr)), None, None, None,
        component[0].key.point_key, leaves, rewires,
    ), None


def normalize_junction_generation(builder, overlay, contacts, generation_type):
    contacts = tuple(contacts)
    unique = {contact_identity(contact): contact for contact in contacts}
    if len(unique) != len(contacts):
        return None, "SYMBOLIC_JUNCTION_DUPLICATE_CONTACT"
    ordered = tuple(sorted(
        unique.values(), key=lambda item: repr(contact_identity(item))
    ))
    deltas, occupied = [], set()
    for component in _components(overlay, ordered):
        if any(
            (item.kind == "EDGE" and not _valid_edge(overlay, item.edge))
            or (item.kind == "ENDPOINT" and not _valid_endpoint(overlay, item.key))
            for item in component
        ):
            return None, "SYMBOLIC_JUNCTION_CONTACT_STALE"
        delta, reason = _multi_delta(builder, overlay, component)
        if reason is not None:
            return None, reason
        resources = {
            item for item in delta_resources(delta) if item[0] == "DEAD"
        }
        if occupied.intersection(resources):
            return None, "SYMBOLIC_JUNCTION_COMPONENT_DELTAS_OVERLAP"
        occupied.update(resources)
        deltas.append(delta)
    return generation_type(
        ordered, tuple(sorted(deltas, key=lambda item: repr(item.contact_keys)))
    ), None


def apply_junction_generation(overlay, generation):
    return apply_component_deltas(
        overlay, generation.deltas,
        collision_reason="SYMBOLIC_JUNCTION_BIRTH_COLLISION",
    )
