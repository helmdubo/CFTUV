"""Stable symbolic normal form for pre-commit exact-time contacts."""

from __future__ import annotations

from dataclasses import dataclass
from functools import cmp_to_key

from .events import EventKind
from . import superlevel as base


@dataclass(frozen=True, slots=True)
class SpanFamilyRefV1:
    """Frozen owner-distinct span before any symbolic subdivision."""

    occurrence: tuple
    participant_keys: tuple[tuple[int, ...], ...]


@dataclass(frozen=True, slots=True)
class SplitContactKeyV1:
    """Stable contact identity; it never points at a mutable child leaf."""

    time_key: tuple
    point_key: tuple
    emitter_key: tuple
    family: SpanFamilyRefV1
    participants: tuple[tuple[int, ...], ...]


@dataclass(frozen=True, slots=True)
class SplitContactV1:
    key: SplitContactKeyV1
    projection: object


@dataclass(frozen=True, slots=True)
class SegmentRefV1:
    """Canonical child leaf between adjacent stable contacts."""

    family: SpanFamilyRefV1
    start: SplitContactKeyV1 | None
    end: SplitContactKeyV1 | None
    occurrence: tuple


@dataclass(frozen=True, slots=True)
class OccurrenceRefV1:
    existing: tuple | None = None
    segment: SegmentRefV1 | None = None


@dataclass(frozen=True, slots=True)
class BirthRefV1:
    """Symbolic junction; runtime allocation is deliberately absent."""

    contact: SplitContactKeyV1
    prev: OccurrenceRefV1
    next: OccurrenceRefV1
    key: tuple


@dataclass(frozen=True, slots=True)
class SplitFamilyNormalFormV1:
    family: SpanFamilyRefV1
    contacts: tuple[SplitContactV1, ...]
    segments: tuple[SegmentRefV1, ...]
    births: tuple[BirthRefV1, ...]


@dataclass(frozen=True, slots=True)
class SymbolicMaterializationPlanV1:
    """One commit payload plus an ID-free stable symbolic receipt."""

    plans: tuple[base.SuperlevelComponentPlanV1, ...]
    families: tuple[SplitFamilyNormalFormV1, ...]
    signature: tuple
    unresolved_reason: str | None = None


def _contact_compare(
    first: SplitContactV1, second: SplitContactV1, budget=None
) -> int:
    difference = first.projection - second.projection
    sign = difference.sign(budget=budget)
    if sign:
        return sign
    if first.key == second.key:
        return 0
    return -1 if repr(first.key) < repr(second.key) else 1


def _event_incident_map(snapshot):
    grouped = {}
    for incident in snapshot.incidents:
        grouped.setdefault(incident.event, []).append(incident)
    resolved = {}
    for event, group in grouped.items():
        identities = {base._incident_sort_key(item) for item in group}
        if len(identities) != 1:
            return None
        resolved[event] = min(group, key=base._incident_sort_key)
    return resolved


def _family_contacts(cut, incident_by_event, budget=None):
    incidents = []
    for event in cut.events:
        incident = incident_by_event.get(event)
        if incident is None or incident.target_projection is None:
            return None, None
        incidents.append(incident)
    family = SpanFamilyRefV1(
        cut.target_occurrence,
        (cut.target_occurrence[0],),
    )
    contacts = tuple(
        SplitContactV1(
            SplitContactKeyV1(
                base._time_key(incident.event.time),
                incident.point_key,
                incident.emitter_key,
                family,
                incident.participants,
            ),
            incident.target_projection,
        )
        for incident in incidents
    )
    ordered = tuple(sorted(
        contacts,
        key=cmp_to_key(
            lambda first, second: _contact_compare(first, second, budget)
        ),
    ))
    if len({contact.key for contact in ordered}) != len(ordered):
        return None, None
    return family, ordered


def _segment_refs(family, contacts, occurrences):
    if len(occurrences) != len(contacts) + 1:
        return None
    boundaries = (None, *(contact.key for contact in contacts), None)
    segments = tuple(
        SegmentRefV1(family, start, end, occurrence)
        for start, end, occurrence in zip(
            boundaries,
            boundaries[1:],
            occurrences,
        )
    )
    for index, contact in enumerate(contacts):
        if (
            segments[index].occurrence[2] != contact.key.point_key
            or segments[index + 1].occurrence[1]
            != contact.key.point_key
        ):
            return None
    return segments


def _occurrence_ref(occurrence, segments_by_occurrence):
    segment = segments_by_occurrence.get(occurrence)
    if segment is not None:
        return OccurrenceRefV1(segment=segment)
    return OccurrenceRefV1(existing=occurrence)


def _birth_refs(cut, contacts, segments):
    contacts_by_point = {contact.key.point_key: contact for contact in contacts}
    segments_by_occurrence = {
        segment.occurrence: segment for segment in segments
    }
    births = []
    for birth in cut.births:
        contact = contacts_by_point.get(birth.point_key)
        if contact is None:
            return None
        births.append(
            BirthRefV1(
                contact.key,
                _occurrence_ref(
                    birth.prev_occurrence,
                    segments_by_occurrence,
                ),
                _occurrence_ref(
                    birth.next_occurrence,
                    segments_by_occurrence,
                ),
                birth.key,
            )
        )
    return tuple(sorted(births, key=lambda birth: repr(birth.key)))


def _normal_form(cut, incident_by_event, budget=None):
    family, contacts = _family_contacts(cut, incident_by_event, budget)
    if family is None:
        return None
    segments = _segment_refs(family, contacts, cut.segment_occurrences)
    if segments is None:
        return None
    births = _birth_refs(cut, contacts, segments)
    if births is None:
        return None
    return SplitFamilyNormalFormV1(family, contacts, segments, births)


def _signature(families):
    return tuple(
        (
            family.family,
            tuple(contact.key for contact in family.contacts),
            family.segments,
            family.births,
        )
        for family in families
    )


def plan_split_materialization(
    snapshot, budget=None
) -> SymbolicMaterializationPlanV1:
    """Rebuild split-family leaves once from F0 plus the stable contact set."""

    plans = base.plan_superlevel_components(snapshot, budget)
    incident_by_event = _event_incident_map(snapshot)
    if incident_by_event is None:
        return SymbolicMaterializationPlanV1(
            plans,
            (),
            (),
            "SPLIT_CONTACT_IDENTITY_AMBIGUOUS",
        )
    families = []
    for plan in plans:
        for cut in plan.split_cuts:
            normal = _normal_form(cut, incident_by_event, budget)
            if normal is None:
                return SymbolicMaterializationPlanV1(
                    plans,
                    tuple(families),
                    _signature(tuple(families)),
                    "SPLIT_FAMILY_NORMAL_FORM_UNRESOLVABLE",
                )
            families.append(normal)
    ordered = tuple(sorted(families, key=lambda item: repr(item.family)))
    if len({family.family for family in ordered}) != len(ordered):
        return SymbolicMaterializationPlanV1(
            plans,
            ordered,
            _signature(ordered),
            "SPLIT_FAMILY_OWNER_AMBIGUOUS",
        )
    return SymbolicMaterializationPlanV1(
        plans,
        ordered,
        _signature(ordered),
    )


def materialize_split_plan(builder, snapshot, materialization) -> bool:
    """Allocate segment twins and births only inside this single commit."""

    edge_by_occurrence = base._commit_prestate(
        snapshot,
        materialization.plans,
    )
    if edge_by_occurrence is None:
        return False
    base._commit_plans(builder, materialization.plans, edge_by_occurrence)
    return True
