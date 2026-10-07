//! The stable symbolic normal form of the pre-commit contacts of an exact time (`wavefront/superlevel_closure.py`): the families of cuts of the packet, their contacts in
//! the order of their projections, the segments between neighbouring contacts and the births in terms of both.
//!
//! A family is one frozen span of one owner before any symbolic subdivision; a contact is a stable key (time, point, emitter, family, participants) that never points at a
//! mutable child leaf; a segment is the canonical child leaf between two adjacent contacts. The contacts of a family are ordered by the sign of the difference of their
//! projections on the span (CPython 3.11's sequence of questions, each comparison a paid sign), then by the `repr` of their keys. A packet whose incidents or families
//! cannot be told apart is answered with a NAMED reason, never an arbitrary choice.

use std::collections::HashMap;

use cftuv_canon::fxhash::FxBuild;
use cftuv_core::exact::{self, ExactCtx};
use cftuv_core::sqrt_sum::{SqrtSum, SIGN_FILTER_BITS};

use crate::error::{SkelError, SkelResult};
use crate::plans::{plan_superlevel_components, ComponentPlan, PlanVal, SplitCutPlan};
use crate::pyval::{sorted_by_repr, Val};
use crate::snapshot::{event_identity, time_key, EventIdentity, Incident, Snapshot};

/// `SplitContactV1`: a stable key and the projection of the contact on its span.
#[derive(Debug, Clone)]
pub struct SplitContact {
    pub key: Val,
    pub projection: SqrtSum,
}

/// `SplitFamilyNormalFormV1`: every field but `contacts` is a [`Val`] dataclass instance (`SpanFamilyRefV1`, `SegmentRefV1`, `BirthRefV1`).
#[derive(Debug, Clone)]
pub struct FamilyNormalForm {
    pub family: Val,
    pub contacts: Vec<SplitContact>,
    pub segments: Vec<Val>,
    pub births: Vec<Val>,
}

/// `SymbolicMaterializationPlanV1`: the plans, the families, the ID-free stable receipt and the named reason when something was unresolved.
#[derive(Debug, Clone)]
pub struct Materialization {
    pub plans: Vec<ComponentPlan>,
    pub families: Vec<FamilyNormalForm>,
    pub signature: Val,
    pub unresolved_reason: Option<&'static str>,
}

fn field(value: &Val, name: &str) -> SkelResult<Val> {
    value.field(name).cloned().ok_or_else(|| SkelError::Unsupported(format!("a key without the field {name}")))
}

/// `_contact_compare(first, second, budget)` as `< 0`: the sign of the difference of the projections (paid), then the equality of the keys, then their `repr`.
fn contact_less(ctx: &mut ExactCtx<'_>, first: (&Val, &SqrtSum), second: (&Val, &SqrtSum)) -> SkelResult<bool> {
    let difference = first.1.sub(second.1);
    let sign = exact::sign(ctx, &difference, SIGN_FILTER_BITS)?;
    if sign != 0 {
        return Ok(sign < 0);
    }
    if first.0 == second.0 {
        return Ok(false);
    }
    Ok(first.0.repr().as_bytes() < second.0.repr().as_bytes())
}

/// `sorted_as_cpython311(items, lambda a, b: _contact_compare(a, b, budget))` over anything that has a key and a projection (the contacts of a family, the interior contacts of a
/// generation): CPython 3.11's sequence of questions, each comparison a paid sign.
pub fn sort_by_projection<T: Clone>(ctx: &mut ExactCtx<'_>, items: &[T], key_and_projection: impl Fn(&T) -> (&Val, &SqrtSum)) -> SkelResult<Vec<T>> {
    let mut failure: Option<SkelError> = None;
    let sorted = {
        let mut less = |left: &usize, right: &usize| -> cftuv_clip::error::ClipResult<bool> {
            match contact_less(ctx, key_and_projection(&items[*left]), key_and_projection(&items[*right])) {
                Ok(answer) => Ok(answer),
                Err(error) => {
                    failure = Some(error);
                    Err(cftuv_clip::error::ClipError::Value("refused"))
                }
            }
        };
        cftuv_clip::cpython311::sort_by_less((0..items.len()).collect(), &mut less)
    };
    match sorted {
        Ok(order) => Ok(order.into_iter().map(|index| items[index].clone()).collect()),
        Err(_) => Err(failure.unwrap_or(SkelError::Unsupported("the comparator sort failed without a cause".to_string()))),
    }
}

/// `sorted_as_cpython311(contacts, lambda a, b: _contact_compare(a, b, budget))`.
pub fn sort_contacts(ctx: &mut ExactCtx<'_>, contacts: &[SplitContact]) -> SkelResult<Vec<SplitContact>> {
    sort_by_projection(ctx, contacts, |contact| (&contact.key, &contact.projection))
}

/// `_event_incident_map(snapshot)`: the incident of every event, or none when two incidents of one event differ in their geometry.
pub fn event_incident_map(snapshot: &Snapshot) -> Option<HashMap<EventIdentity, &Incident, FxBuild>> {
    let mut grouped: HashMap<EventIdentity, Vec<&Incident>, FxBuild> = HashMap::default();
    for incident in &snapshot.incidents {
        grouped.entry(incident.identity().clone()).or_default().push(incident);
    }
    let mut resolved = HashMap::default();
    for (event, group) in grouped {
        let first = group[0].sort_key();
        if group.iter().any(|item| item.sort_key() != first) {
            return None;
        }
        resolved.insert(event, group[0]);
    }
    Some(resolved)
}

pub(crate) fn span_family(occurrence: &Val) -> Val {
    Val::data("SpanFamilyRefV1", vec![("occurrence", occurrence.clone()), ("participant_keys", Val::tuple(vec![occurrence.get(0).cloned().unwrap_or_else(Val::none)]))])
}

fn keys_val(keys: &[Vec<i64>]) -> Val {
    Val::tuple(keys.iter().map(|key| Val::ints(key)).collect())
}

/// `_family_contacts(cut, incident_by_event, budget)`: the family of a cut and its contacts in order; none when an event has no incident or no projection, or two
/// contacts share a key.
pub fn family_contacts(ctx: &mut ExactCtx<'_>, cut: &SplitCutPlan, incident_by_event: &HashMap<EventIdentity, &Incident, FxBuild>) -> SkelResult<Option<(Val, Vec<SplitContact>)>> {
    let mut incidents: Vec<&Incident> = Vec::new();
    for event in &cut.events {
        match incident_by_event.get(&event_identity(event)) {
            Some(incident) if incident.target_projection.is_some() => incidents.push(incident),
            _ => return Ok(None),
        }
    }
    let family = span_family(&cut.target_occurrence);
    let mut contacts = Vec::new();
    for incident in incidents {
        let Some(projection) = incident.target_projection.clone() else {
            return Ok(None);
        };
        let key = Val::data(
            "SplitContactKeyV1",
            vec![
                ("time_key", time_key(&incident.event.time)?),
                ("point_key", incident.point_key.clone()),
                ("emitter_key", incident.emitter_key.clone()),
                ("family", family.clone()),
                ("participants", keys_val(&incident.participants)),
            ],
        );
        contacts.push(SplitContact { key, projection });
    }
    let ordered = sort_contacts(ctx, &contacts)?;
    let distinct: std::collections::HashSet<&Val> = ordered.iter().map(|contact| &contact.key).collect();
    if distinct.len() != ordered.len() {
        return Ok(None);
    }
    Ok(Some((family, ordered)))
}

/// `_segment_refs(family, contacts, occurrences)`: the child leaves between adjacent contacts, each checked against the point of the contact it touches.
pub fn segment_refs(family: &Val, contacts: &[SplitContact], occurrences: &[Val]) -> SkelResult<Option<Vec<Val>>> {
    if occurrences.len() != contacts.len() + 1 {
        return Ok(None);
    }
    let mut boundaries: Vec<Val> = vec![Val::none()];
    boundaries.extend(contacts.iter().map(|contact| contact.key.clone()));
    boundaries.push(Val::none());
    let segments: Vec<Val> = boundaries
        .iter()
        .zip(boundaries.iter().skip(1))
        .zip(occurrences)
        .map(|((start, end), occurrence)| Val::data("SegmentRefV1", vec![("family", family.clone()), ("start", start.clone()), ("end", end.clone()), ("occurrence", occurrence.clone())]))
        .collect();
    for (index, contact) in contacts.iter().enumerate() {
        let point_key = field(&contact.key, "point_key")?;
        let (before, after) = (field(&segments[index], "occurrence")?, field(&segments[index + 1], "occurrence")?);
        if before.get(2) != Some(&point_key) || after.get(1) != Some(&point_key) {
            return Ok(None);
        }
    }
    Ok(Some(segments))
}

fn occurrence_ref(occurrence: &Val, segments_by_occurrence: &HashMap<Val, Val, FxBuild>) -> Val {
    match segments_by_occurrence.get(occurrence) {
        Some(segment) => Val::data("OccurrenceRefV1", vec![("existing", Val::none()), ("segment", segment.clone())]),
        None => Val::data("OccurrenceRefV1", vec![("existing", occurrence.clone()), ("segment", Val::none())]),
    }
}

/// `_birth_refs(cut, contacts, segments)`: the births of a cut in terms of contacts and segments, sorted by the `repr` of the birth key.
pub fn birth_refs(cut: &SplitCutPlan, contacts: &[SplitContact], segments: &[Val]) -> SkelResult<Option<Vec<Val>>> {
    let mut contacts_by_point: HashMap<Val, &SplitContact, FxBuild> = HashMap::default();
    for contact in contacts {
        contacts_by_point.insert(field(&contact.key, "point_key")?, contact);
    }
    let mut segments_by_occurrence: HashMap<Val, Val, FxBuild> = HashMap::default();
    for segment in segments {
        segments_by_occurrence.insert(field(segment, "occurrence")?, segment.clone());
    }
    let mut births = Vec::new();
    for item in &cut.births {
        let Some(contact) = contacts_by_point.get(&item.point_key) else {
            return Ok(None);
        };
        births.push(Val::data(
            "BirthRefV1",
            vec![
                ("contact", contact.key.clone()),
                ("prev", occurrence_ref(&item.prev_occurrence, &segments_by_occurrence)),
                ("next", occurrence_ref(&item.next_occurrence, &segments_by_occurrence)),
                ("key", item.key.clone()),
            ],
        ));
    }
    Ok(Some(sorted_by_repr(&births, |birth| birth.field("key").cloned().unwrap_or_else(Val::none))))
}

/// `_normal_form(cut, incident_by_event, budget)`.
pub fn normal_form(ctx: &mut ExactCtx<'_>, cut: &SplitCutPlan, incident_by_event: &HashMap<EventIdentity, &Incident, FxBuild>) -> SkelResult<Option<FamilyNormalForm>> {
    let Some((family, contacts)) = family_contacts(ctx, cut, incident_by_event)? else {
        return Ok(None);
    };
    let Some(segments) = segment_refs(&family, &contacts, &cut.segment_occurrences)? else {
        return Ok(None);
    };
    let Some(births) = birth_refs(cut, &contacts, &segments)? else {
        return Ok(None);
    };
    Ok(Some(FamilyNormalForm { family, contacts, segments, births }))
}

/// `_signature(families)`: `(family, contact keys, segments, births)` of every family.
pub fn signature(families: &[FamilyNormalForm]) -> Val {
    Val::tuple(
        families
            .iter()
            .map(|family| {
                Val::tuple(vec![
                    family.family.clone(),
                    Val::tuple(family.contacts.iter().map(|contact| contact.key.clone()).collect()),
                    Val::tuple(family.segments.clone()),
                    Val::tuple(family.births.clone()),
                ])
            })
            .collect(),
    )
}

fn unresolved(plans: Vec<ComponentPlan>, families: Vec<FamilyNormalForm>, signature: Val, reason: &'static str) -> Materialization {
    Materialization { plans, families, signature, unresolved_reason: Some(reason) }
}

/// `plan_split_materialization(snapshot, budget)`: rebuild the split-family leaves once from the frozen prestate and the stable set of contacts.
pub fn plan_split_materialization(ctx: &mut ExactCtx<'_>, snapshot: &Snapshot) -> SkelResult<Materialization> {
    let plans = plan_superlevel_components(ctx, snapshot)?;
    let Some(incident_by_event) = event_incident_map(snapshot) else {
        return Ok(unresolved(plans, Vec::new(), Val::tuple(Vec::new()), "SPLIT_CONTACT_IDENTITY_AMBIGUOUS"));
    };
    let mut families: Vec<FamilyNormalForm> = Vec::new();
    for plan in &plans {
        for cut in &plan.split_cuts {
            match normal_form(ctx, cut, &incident_by_event)? {
                Some(normal) => families.push(normal),
                None => {
                    let receipt = signature(&families);
                    return Ok(unresolved(plans.clone(), families, receipt, "SPLIT_FAMILY_NORMAL_FORM_UNRESOLVABLE"));
                }
            }
        }
    }
    let ordered = {
        let mut keyed: Vec<FamilyNormalForm> = families;
        keyed.sort_by_cached_key(|item| item.family.repr().to_string());
        keyed
    };
    let distinct: std::collections::HashSet<&Val> = ordered.iter().map(|family| &family.family).collect();
    let receipt = signature(&ordered);
    if distinct.len() != ordered.len() {
        return Ok(unresolved(plans, ordered, receipt, "SPLIT_FAMILY_OWNER_AMBIGUOUS"));
    }
    Ok(Materialization { plans, families: ordered, signature: receipt, unresolved_reason: None })
}

impl PlanVal for SplitContact {
    fn to_val(&self) -> Val {
        Val::data("SplitContactV1", vec![("key", self.key.clone()), ("projection", Val::data("SqrtSumV1", vec![("terms", Val::terms_of(&self.projection))]))])
    }
}

impl PlanVal for FamilyNormalForm {
    fn to_val(&self) -> Val {
        Val::data(
            "SplitFamilyNormalFormV1",
            vec![
                ("family", self.family.clone()),
                ("contacts", Val::tuple(self.contacts.iter().map(PlanVal::to_val).collect())),
                ("segments", Val::tuple(self.segments.clone())),
                ("births", Val::tuple(self.births.clone())),
            ],
        )
    }
}

impl PlanVal for Materialization {
    fn to_val(&self) -> Val {
        Val::data(
            "SymbolicMaterializationPlanV1",
            vec![
                ("plans", Val::tuple(self.plans.iter().map(PlanVal::to_val).collect())),
                ("families", Val::tuple(self.families.iter().map(PlanVal::to_val).collect())),
                ("signature", self.signature.clone()),
                ("unresolved_reason", self.unresolved_reason.map_or_else(Val::none, Val::str)),
            ],
        )
    }
}
