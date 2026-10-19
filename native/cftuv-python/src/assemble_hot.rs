//! `assemble.assemble_batch` behind the persistent session (the ASSEMBLE stage of the materializer): every record of the batch but the diagnostics, the contract
//! versions and the batch itself, which the shim adds (`materialize_hot_op.assemble_batch`).
//!
//! The oracle builds frozen `slots` dataclasses through their `__init__`; here they are built through the slot layout the shim's class table names (`Raw` where the
//! probe confirms it, the generic setter otherwise), with the values the `__post_init__` of the oracle would accept checked first: an id text that `str.strip`
//! empties, a face of fewer than three vertices, a UV that is not finite. Such an input, a key missing from a table (the oracle's `KeyError`) or the oracle's
//! `HALF_EDGE_SHARED_IN_ONE_DIRECTION` of `chains_of` is declined: nothing is built and the shim runs the oracle on the same input. No exact work, no budget, no memory.
//!
//! The numbers: `uv_direct_strip_v1` and `decimal_of` (`cftuv_silhouette::assemble_law`); the chains when no silhouette pass built them: `chains_of`
//! (`cftuv_silhouette::silhouette::chains_of_cycles`).

use std::collections::{BTreeSet, HashMap};
use std::panic::{catch_unwind, AssertUnwindSafe};

use pyo3::exceptions::PyRuntimeError;
use pyo3::prelude::*;
use pyo3::types::{PyFloat, PyFrozenSet, PyInt, PyString, PyTuple};

use cftuv_canon::fxhash::FxBuild;
use cftuv_core::num::IBig;
use cftuv_core::rat::Rat;
use cftuv_core::sqrt_sum::SqrtSum;
use cftuv_silhouette::assemble_law::{decimal_of, decimal_text, uv_coordinate};
use cftuv_silhouette::silhouette::{chains_of_cycles, Key, SilError, SilInput};

use crate::materialize_hot::{exact_dict, pair_of, rat_of, sequence, small_int, unsupported, Classes, Fail, Host, Keys, Read};
use crate::pyobj::{alloc, ibig_from_int, note_attr_build, set_slot, Raw, SlotMode};

/// The classes of the table, in the order the shim hands them over (`materialize_hot_op.ASSEMBLE_CLASSES`).
pub const CLASS_NAMES: [&str; 22] = [
    "SourceFaceId",
    "PhysicalEdgeId",
    "ChainUseId",
    "LineageId",
    "VertexKey",
    "SemanticLocationId",
    "GeometryFaceId",
    "SemanticRegionId",
    "OwnershipClaimId",
    "GeometryStationFactId",
    "SemanticBoundaryId",
    "SemanticInterfaceId",
    "GeometryProvenanceV1",
    "GeometryVertexV1",
    "GeometryUvFactV1",
    "UvPoint2V1",
    "GeometryStationFactV1",
    "LocalCoordinateV1",
    "GeometryFaceV1",
    "GeometrySemanticRegionV1",
    "GeometryBoundaryChainV1",
    "GeometryInterfaceChainV1",
];

const SOURCE_FACE: usize = 0;
const PHYSICAL_EDGE: usize = 1;
const CHAIN_USE: usize = 2;
const LINEAGE: usize = 3;
const VERTEX_KEY: usize = 4;
const LOCATION: usize = 5;
const FACE_ID: usize = 6;
const REGION_ID: usize = 7;
const CLAIM_ID: usize = 8;
const STATION_ID: usize = 9;
const BOUNDARY_ID: usize = 10;
const INTERFACE_ID: usize = 11;
const PROVENANCE: usize = 12;
const VERTEX: usize = 13;
const UV_FACT: usize = 14;
const UV_POINT: usize = 15;
const STATION_FACT: usize = 16;
const COORDINATE: usize = 17;
const FACE: usize = 18;
const REGION: usize = 19;
const BOUNDARY_CHAIN: usize = 20;
const INTERFACE_CHAIN: usize = 21;

/// One class of the table: the class, its field names (the dataclass order), the raw layout when the probe confirms it.
struct Kind {
    class: Py<PyAny>,
    names: Vec<Py<PyString>>,
    raw: Option<Raw>,
}

impl Kind {
    fn make<'py>(&self, py: Python<'py>, values: Vec<Bound<'py, PyAny>>) -> PyResult<Bound<'py, PyAny>> {
        if let Some(raw) = &self.raw {
            return raw.build_values(py, &self.class, values);
        }
        note_attr_build();
        let object = alloc(py, &self.class)?;
        for (name, value) in self.names.iter().zip(&values) {
            set_slot(&object, name, value)?;
        }
        Ok(object)
    }
}

pub struct AssembleClasses {
    kinds: Vec<Kind>,
    decimal: Py<PyAny>,
    metric_space: Py<PyAny>,
}

impl Host {
    /// The class table (`(name, class, field names)` in the order of [`CLASS_NAMES`]), `decimal.Decimal` and the default metric space of `LocalCoordinateV1`.
    pub fn bind_assemble(&mut self, py: Python<'_>, table: &Bound<'_, PyAny>, decimal: &Bound<'_, PyAny>, metric_space: &Bound<'_, PyAny>, mode: SlotMode) -> PyResult<()> {
        let rows: Vec<Bound<'_, PyAny>> = table.try_iter()?.collect::<PyResult<_>>()?;
        if rows.len() != CLASS_NAMES.len() {
            return Err(PyRuntimeError::new_err(format!("cftuv_native: the assemble class table has {} rows, not {}", rows.len(), CLASS_NAMES.len())));
        }
        let mut kinds = Vec::with_capacity(rows.len());
        for (row, wanted) in rows.iter().zip(CLASS_NAMES) {
            let (name, class, fields): (String, Bound<'_, PyAny>, Vec<String>) = row.extract()?;
            if name != wanted {
                return Err(PyRuntimeError::new_err(format!("cftuv_native: the assemble class table names {name} where {wanted} is expected")));
            }
            let names: Vec<Py<PyString>> = fields.iter().map(|field| PyString::intern(py, field).unbind()).collect();
            let class = class.unbind();
            let refs: Vec<&Py<PyString>> = names.iter().collect();
            let raw = Raw::select(py, mode, wanted, &class, &refs)?;
            kinds.push(Kind { class, names, raw });
        }
        self.assemble = Some(AssembleClasses { kinds, decimal: decimal.clone().unbind(), metric_space: metric_space.clone().unbind() });
        Ok(())
    }

    pub fn disable_assemble_raw(&mut self) {
        if let Some(classes) = self.assemble.as_mut() {
            for kind in &mut classes.kinds {
                kind.raw = None;
            }
        }
    }
}

/// What the shim hands over for one `assemble_batch` (see `materialize_hot_op.assemble_batch`).
pub struct AssembleArgs<'r, 'py> {
    /// One row per frame face: `(region, claim number, lineage texts, physical edge ids, chain use ids, run lineage ids, station model)`.
    pub frames: &'r Bound<'py, PyAny>,
    pub edge_faces: &'r Bound<'py, PyAny>,
    pub cycles: &'r Bound<'py, PyAny>,
    pub vertex_cycles: &'r Bound<'py, PyAny>,
    pub positions: &'r Bound<'py, PyAny>,
    pub polygons: &'r Bound<'py, PyAny>,
    pub facts: &'r Bound<'py, PyAny>,
    pub merged: &'r Bound<'py, PyAny>,
    /// `None`: the chains are `chains_of(frame_faces, cycles, ...)`, built here; else `(boundary, interface)` handed back as they are.
    pub chains: &'r Bound<'py, PyAny>,
    pub cut_pairs: &'r Bound<'py, PyAny>,
    pub mark: &'r str,
    pub lattice_alpha: &'r Bound<'py, PyAny>,
    pub scale: &'r Bound<'py, PyAny>,
    pub material: &'r Bound<'py, PyAny>,
}

/// `str.strip()` of Python empties the text: every character is whitespace in Python's sense (`str.isspace`: Unicode White_Space plus the four separators U+001C..U+001F).
fn blank(text: &str) -> bool {
    text.chars().all(|c| c.is_whitespace() || ('\u{1c}'..='\u{1f}').contains(&c))
}

fn text_of(object: &Bound<'_, PyAny>, what: &str) -> Read<String> {
    if !object.is_exact_instance_of::<PyString>() {
        return unsupported(format!("{what} is not a str"));
    }
    let text: String = object.extract()?;
    Ok(text)
}

fn texts_of(object: &Bound<'_, PyAny>, what: &str) -> Read<Vec<String>> {
    let mut found = Vec::new();
    for item in object.try_iter()? {
        found.push(text_of(&item?, what)?);
    }
    Ok(found)
}

struct Frame<'py> {
    region: u32,
    claim: u32,
    /// The four provenance sets of the face: source faces, physical edges, chain uses, lineage.
    sets: [BTreeSet<String>; 4],
    run_lineage: Vec<String>,
    station_model: Bound<'py, PyAny>,
}

/// The objects of one call: ids interned by `(class, text)` (equal values, one object each: the oracle builds one per use, equal and of equal hash).
struct Builder<'c, 'py> {
    py: Python<'py>,
    classes: &'c AssembleClasses,
    ids: HashMap<(usize, String), Bound<'py, PyAny>, FxBuild>,
}

impl<'c, 'py> Builder<'c, 'py> {
    fn id(&mut self, kind: usize, text: String) -> Read<Bound<'py, PyAny>> {
        if let Some(found) = self.ids.get(&(kind, text.clone())) {
            return Ok(found.clone());
        }
        if blank(&text) {
            return unsupported(format!("{} of a blank text (the oracle raises ValueError)", CLASS_NAMES[kind]));
        }
        let object = self.classes.kinds[kind].make(self.py, vec![PyString::new(self.py, &text).into_any()])?;
        self.ids.insert((kind, text), object.clone());
        Ok(object)
    }

    fn make(&self, kind: usize, values: Vec<Bound<'py, PyAny>>) -> Read<Bound<'py, PyAny>> {
        Ok(self.classes.kinds[kind].make(self.py, values)?)
    }

    fn frozenset(&mut self, kind: usize, texts: &BTreeSet<String>) -> Read<Bound<'py, PyAny>> {
        let mut items = Vec::with_capacity(texts.len());
        for text in texts {
            items.push(self.id(kind, text.clone())?);
        }
        Ok(PyFrozenSet::new(self.py, &items)?.into_any())
    }

    fn provenance(&mut self, sets: &[BTreeSet<String>; 4]) -> Read<Bound<'py, PyAny>> {
        let values = vec![self.frozenset(SOURCE_FACE, &sets[0])?, self.frozenset(PHYSICAL_EDGE, &sets[1])?, self.frozenset(CHAIN_USE, &sets[2])?, self.frozenset(LINEAGE, &sets[3])?];
        self.make(PROVENANCE, values)
    }
}

fn union(frames: &[Frame<'_>], members: &[usize]) -> [BTreeSet<String>; 4] {
    let mut sets: [BTreeSet<String>; 4] = Default::default();
    for &member in members {
        for (into, from) in sets.iter_mut().zip(&frames[member].sets) {
            into.extend(from.iter().cloned());
        }
    }
    sets
}

fn read_frames<'py>(args: &AssembleArgs<'_, 'py>) -> Read<Vec<Frame<'py>>> {
    let edge_faces = exact_dict(args.edge_faces, "edge_faces")?;
    let mut frames = Vec::new();
    for row in sequence(args.frames, "frames")? {
        let items = sequence(&row, "a frame row")?;
        if items.len() != 7 {
            return unsupported("a frame row is not seven items");
        }
        let physical = texts_of(&items[3], "a physical edge id")?;
        let mut source_faces = BTreeSet::new();
        for edge in &physical {
            if let Some(names) = edge_faces.get_item(edge.as_str())? {
                source_faces.extend(texts_of(&names, "a source face id")?);
            }
        }
        frames.push(Frame {
            region: small_int(&items[0], "a region")?,
            claim: small_int(&items[1], "a claim number")?,
            sets: [source_faces, physical.into_iter().collect(), texts_of(&items[4], "a chain use id")?.into_iter().collect(), texts_of(&items[2], "a lineage id")?.into_iter().collect()],
            run_lineage: texts_of(&items[5], "a lineage id")?,
            station_model: items[6].clone(),
        });
    }
    Ok(frames)
}

/// `(key, ...)` of the first items of every cycle (the vertex cycles are cycles of `(key, point)` as well).
fn read_cycles<'a, 'py>(py: Python<'py>, object: &Bound<'py, PyAny>, keys: &mut Keys<'a, 'py>) -> Read<Vec<Vec<Key>>> {
    let mut cycles = Vec::new();
    for cycle in sequence(object, "cycles")? {
        let mut found = Vec::new();
        for item in sequence(&cycle, "a cycle")? {
            let Ok(item) = item.cast_exact::<PyTuple>() else {
                return unsupported("a cycle item is not a tuple");
            };
            if item.len() != 2 {
                return unsupported("a cycle item is not (key, point) (the oracle unpacks two)");
            }
            found.push(keys.key(py, &item.get_item(0)?)?);
        }
        cycles.push(found);
    }
    Ok(cycles)
}

fn located(bytes: &[u8], mark: &[u8]) -> Read<String> {
    let place = if !mark.is_empty() && bytes.ends_with(mark) { &bytes[..bytes.len() - mark.len()] } else { bytes };
    match std::str::from_utf8(place) {
        Ok(text) => Ok(format!("location:{text}")),
        Err(_) => unsupported("a vertex key is not UTF-8 after its copy mark"),
    }
}

fn key_text(keys: &Keys<'_, '_>, key: Key) -> Read<String> {
    match std::str::from_utf8(keys.bytes[key as usize]) {
        Ok(text) => Ok(text.to_string()),
        Err(_) => unsupported("a vertex key is not UTF-8"),
    }
}

/// The records: `(vertices, faces, station facts, regions, boundary chains, interface chains)`.
fn assemble<'py>(py: Python<'py>, hot: &Classes, classes: &AssembleClasses, args: &AssembleArgs<'_, 'py>) -> Read<Bound<'py, PyAny>> {
    let pool = &hot.pool;
    let frames = read_frames(args)?;
    let mut keys = Keys::new();
    let cycles = read_cycles(py, args.cycles, &mut keys)?;
    if cycles.len() != frames.len() {
        return unsupported("cycles and frame faces of different lengths");
    }
    let vertex_cycles = if args.vertex_cycles.is_none() { None } else { Some(read_cycles(py, args.vertex_cycles, &mut keys)?) };
    let polygon_objects = sequence(args.polygons, "polygons")?;
    let mut polygons: Vec<Vec<Vec<Key>>> = Vec::with_capacity(polygon_objects.len());
    for face in &polygon_objects {
        let mut rings = Vec::new();
        for ring in sequence(face, "the polygons of a face")? {
            let mut found = Vec::new();
            for key in sequence(&ring, "a polygon")? {
                found.push(keys.key(py, &key)?);
            }
            rings.push(found);
        }
        polygons.push(rings);
    }
    let mut position_rows = Vec::new();
    for (key, value) in exact_dict(args.positions, "positions")?.iter() {
        position_rows.push((keys.key(py, &key)?, value));
    }
    let mut fact_rows: Vec<(u32, Key, (SqrtSum, SqrtSum))> = Vec::new();
    let mut facts: HashMap<(u32, Key), usize, FxBuild> = HashMap::default();
    for (slot, value) in exact_dict(args.facts, "facts")?.iter() {
        let Ok(slot) = slot.cast_exact::<PyTuple>() else {
            return unsupported("a fact slot is not a tuple");
        };
        if slot.len() != 2 {
            return unsupported("a fact slot is not (region, key)");
        }
        let region = small_int(&slot.get_item(0)?, "a fact region")?;
        let key = keys.key(py, &slot.get_item(1)?)?;
        facts.insert((region, key), fact_rows.len());
        fact_rows.push((region, key, pair_of(pool, &value, "a fact")?));
    }
    let mut merged: HashMap<(u32, Vec<Key>), Vec<usize>, FxBuild> = HashMap::default();
    if !args.merged.is_none() {
        for (slot, others) in exact_dict(args.merged, "merged_frames")?.iter() {
            let items = sequence(&slot, "a merged slot")?;
            if items.len() != 2 {
                return unsupported("a merged slot is not (face, polygon)");
            }
            let index = small_int(&items[0], "a merged face")?;
            let mut ring = Vec::new();
            for key in sequence(&items[1], "a merged polygon")? {
                ring.push(keys.key(py, &key)?);
            }
            let mut found = Vec::new();
            for other in sequence(&others, "the other merged faces")? {
                let other = small_int(&other, "a merged face")? as usize;
                if other >= frames.len() {
                    return unsupported("a merged face beyond the frame faces (the oracle raises IndexError)");
                }
                found.push(other);
            }
            merged.insert((index, ring), found);
        }
    }
    let alpha = rat_of(pool, args.lattice_alpha, "lattice_alpha")?;
    if alpha.is_zero() {
        return unsupported("lattice_alpha is zero");
    }
    let inverse_alpha = Rat::one().div(&alpha).map_err(|_| Fail::Unsupported("lattice_alpha is zero".to_string()))?;
    if !args.scale.is_exact_instance_of::<PyInt>() {
        return unsupported("the scale is not an int");
    }
    let scale: IBig = ibig_from_int(args.scale)?;
    let mark = args.mark.as_bytes();
    let mut builder = Builder { py, classes, ids: HashMap::default() };

    // the provenance of each frame face, and of each region (the frame faces of the region, in order)
    let mut face_prov = Vec::with_capacity(frames.len());
    for frame in &frames {
        face_prov.push(builder.provenance(&frame.sets)?);
    }
    let mut region_members: Vec<(u32, Vec<usize>)> = Vec::new();
    let mut region_last: HashMap<u32, usize, FxBuild> = HashMap::default();
    for (index, frame) in frames.iter().enumerate() {
        match region_members.iter_mut().find(|(region, _)| *region == frame.region) {
            Some((_, members)) => members.push(index),
            None => region_members.push((frame.region, vec![index])),
        }
        region_last.insert(frame.region, index);
    }

    // vertices: the provenance of the faces whose cycles hold the key
    let vertex_source = vertex_cycles.as_ref().unwrap_or(&cycles);
    let mut holders: HashMap<Key, Vec<usize>, FxBuild> = HashMap::default();
    for (index, cycle) in vertex_source.iter().enumerate().take(frames.len()) {
        for key in cycle {
            let entry = holders.entry(*key).or_default();
            if !entry.contains(&index) {
                entry.push(index);
            }
        }
    }
    let mut merged_prov: HashMap<Vec<usize>, Bound<'py, PyAny>, FxBuild> = HashMap::default();
    let mut vertices = Vec::with_capacity(position_rows.len());
    for (key, position) in &position_rows {
        let Some(members) = holders.get(key) else {
            return unsupported("a lifted vertex in no cycle (the oracle raises KeyError)");
        };
        let mut members = members.clone();
        members.sort_unstable();
        let provenance = match merged_prov.get(&members) {
            Some(found) => found.clone(),
            None => {
                let built = builder.provenance(&union(&frames, &members))?;
                merged_prov.insert(members, built.clone());
                built
            }
        };
        let vertex_key = builder.id(VERTEX_KEY, key_text(&keys, *key)?)?;
        let location = builder.id(LOCATION, located(keys.bytes[*key as usize], mark)?)?;
        vertices.push(builder.make(VERTEX, vec![vertex_key, position.clone(), location, provenance])?);
    }

    // regions
    let material = args.material.clone();
    let claim_of_region = |region: u32| frames[region_last[&region]].claim;
    let mut regions = Vec::with_capacity(region_members.len());
    for (region, members) in &region_members {
        let region_id = builder.id(REGION_ID, format!("region:{region}"))?;
        let claim_id = builder.id(CLAIM_ID, format!("claim:{}", claim_of_region(*region)))?;
        let provenance = builder.provenance(&union(&frames, members))?;
        regions.push(builder.make(REGION, vec![region_id, claim_id, material.clone(), provenance])?);
    }

    // faces, in the order of the frame faces and their polygons
    let mut uv_facts: HashMap<(u32, Key), Bound<'py, PyAny>, FxBuild> = HashMap::default();
    let mut faces = Vec::new();
    for (index, (frame, rings)) in frames.iter().zip(&polygons).enumerate() {
        let region_id = builder.id(REGION_ID, format!("region:{}", frame.region))?;
        let claim_id = builder.id(CLAIM_ID, format!("claim:{}", frame.claim))?;
        for ring in rings {
            if ring.len() < 3 {
                return unsupported("a face of fewer than three vertices (GeometryFaceV1 raises ValueError)");
            }
            let face_id = builder.id(FACE_ID, format!("face:{}", faces.len()))?;
            let mut ordered = Vec::with_capacity(ring.len());
            let mut uvs = Vec::with_capacity(ring.len());
            for key in ring {
                let vertex_key = builder.id(VERTEX_KEY, key_text(&keys, *key)?)?;
                ordered.push(vertex_key.clone());
                let slot = (frame.region, *key);
                let fact = match uv_facts.get(&slot) {
                    Some(found) => found.clone(),
                    None => {
                        let Some(row) = facts.get(&slot) else {
                            return unsupported("a face vertex without a fact (the oracle raises KeyError)");
                        };
                        let (s, r) = &fact_rows[*row].2;
                        let (Some(u), Some(v)) = (uv_coordinate(s, &inverse_alpha), uv_coordinate(r, &inverse_alpha)) else {
                            return unsupported("a UV that is not finite (UvPoint2V1 raises ValueError)");
                        };
                        let point = builder.make(UV_POINT, vec![PyFloat::new(py, u).into_any(), PyFloat::new(py, v).into_any()])?;
                        let built = builder.make(UV_FACT, vec![vertex_key.clone(), point])?;
                        uv_facts.insert(slot, built.clone());
                        built
                    }
                };
                uvs.push(fact);
            }
            let provenance = match merged.get(&(index as u32, ring.clone())) {
                Some(others) if !others.is_empty() => {
                    let mut members = vec![index];
                    members.extend(others.iter().copied());
                    builder.provenance(&union(&frames, &members))?
                }
                _ => face_prov[index].clone(),
            };
            let values = vec![face_id, PyTuple::new(py, ordered)?.into_any(), PyTuple::new(py, uvs)?.into_any(), region_id.clone(), claim_id.clone(), provenance, material.clone()];
            faces.push(builder.make(FACE, values)?);
        }
    }

    // station facts
    let decimal = classes.decimal.bind(py);
    let metric = classes.metric_space.bind(py);
    let mut lineage_of: HashMap<u32, Bound<'py, PyAny>, FxBuild> = HashMap::default();
    let mut station_facts = Vec::with_capacity(fact_rows.len());
    for (region, key, (s, r)) in &fact_rows {
        let Some(&last) = region_last.get(region) else {
            return unsupported("a fact of a region no frame face has (the oracle raises KeyError)");
        };
        let lineage = match lineage_of.get(region) {
            Some(found) => found.clone(),
            None => {
                let set: BTreeSet<String> = frames[last].run_lineage.iter().cloned().collect();
                let built = builder.frozenset(LINEAGE, &set)?;
                lineage_of.insert(*region, built.clone());
                built
            }
        };
        let mut coordinates = Vec::with_capacity(2);
        for value in [s, r] {
            let Some((negative, coefficient, exponent)) = decimal_of(value, &scale) else {
                return unsupported("a zero scale (the oracle raises ZeroDivisionError)");
            };
            let number = decimal.call1((decimal_text(negative, &coefficient, exponent),))?;
            coordinates.push(builder.make(COORDINATE, vec![number, metric.clone()])?);
        }
        let key_name = key_text(&keys, *key)?;
        let values = vec![
            builder.id(STATION_ID, format!("station:{region}:{key_name}"))?,
            builder.id(VERTEX_KEY, key_name)?,
            builder.id(REGION_ID, format!("region:{region}"))?,
            builder.id(CLAIM_ID, format!("claim:{}", frames[last].claim))?,
            coordinates[0].clone(),
            coordinates[1].clone(),
            frames[last].station_model.clone(),
            lineage,
        ];
        station_facts.push(builder.make(STATION_FACT, values)?);
    }

    // chains
    let (boundary, interface) = if args.chains.is_none() {
        let cut_pairs_set = {
            let mut set = std::collections::HashSet::<(u32, u32), FxBuild>::default();
            for pair in sequence(args.cut_pairs, "cut_pairs")? {
                let items = sequence(&pair, "a cut pair")?;
                if items.len() != 2 {
                    return unsupported("a cut pair is not two regions");
                }
                let (a, b) = (small_int(&items[0], "a cut region")?, small_int(&items[1], "a cut region")?);
                set.insert((a.min(b), a.max(b)));
            }
            set
        };
        let count = keys.bytes.len();
        let mut fact_table = HashMap::with_capacity_and_hasher(fact_rows.len(), FxBuild::default());
        for (region, key, pair) in &fact_rows {
            fact_table.insert((*region, *key), pair.clone());
        }
        let source = SilInput {
            rank: keys.ranks(),
            place: keys.places(mark),
            polygons: Vec::new(),
            cycles: cycles.clone(),
            regions: frames.iter().map(|frame| frame.region).collect(),
            cut_pairs: cut_pairs_set,
            positions: vec![None; count],
            points: vec![None; count],
            facts: fact_table,
            front: SqrtSum::rational(&alpha),
            inverse_alpha: inverse_alpha.clone(),
            uv_slide: Rat::zero(),
            chord_budget: Rat::zero(),
            reverse: false,
            free: vec![false; count],
            free_any: false,
        };
        let (boundary, interface) = match chains_of_cycles(&source) {
            Ok(found) => found,
            Err(SilError::HalfEdgeShared(..)) => return unsupported("HALF_EDGE_SHARED_IN_ONE_DIRECTION: the oracle names it"),
            Err(SilError::Exhausted) => return unsupported("an exhaustion in chains_of"),
            Err(SilError::Unsupported(text)) => return unsupported(text),
        };
        let mut sets = Vec::with_capacity(2);
        for (chains, id_kind, chain_kind) in [(boundary, BOUNDARY_ID, BOUNDARY_CHAIN), (interface, INTERFACE_ID, INTERFACE_CHAIN)] {
            let mut items = Vec::with_capacity(chains.len());
            for (id, path) in chains {
                let mut ordered = Vec::with_capacity(path.len());
                for key in path {
                    ordered.push(builder.id(VERTEX_KEY, key_text(&keys, key)?)?);
                }
                let values = vec![builder.id(id_kind, id)?, PyTuple::new(py, ordered)?.into_any()];
                items.push(builder.make(chain_kind, values)?);
            }
            sets.push(PyFrozenSet::new(py, &items)?.into_any());
        }
        (sets[0].clone(), sets[1].clone())
    } else {
        let items = sequence(args.chains, "chains")?;
        if items.len() != 2 {
            return unsupported("chains is not (boundary, interface)");
        }
        (items[0].clone(), items[1].clone())
    };

    Ok(PyTuple::new(
        py,
        [
            PyFrozenSet::new(py, &vertices)?.into_any(),
            PyTuple::new(py, faces)?.into_any(),
            PyFrozenSet::new(py, &station_facts)?.into_any(),
            PyFrozenSet::new(py, &regions)?.into_any(),
            boundary,
            interface,
        ],
    )?
    .into_any())
}

impl Host {
    /// One `assemble_batch` (see the module note): `Ok(records)` or `Err(why the port declined)`.
    pub fn assemble<'py>(&self, py: Python<'py>, args: &AssembleArgs<'_, 'py>) -> PyResult<Result<Bound<'py, PyAny>, String>> {
        let hot = self.classes()?;
        let Some(classes) = self.assemble.as_ref() else {
            return Err(PyRuntimeError::new_err("cftuv_native: the assemble classes were not bound (`bind_assemble`)"));
        };
        let outcome = catch_unwind(AssertUnwindSafe(|| assemble(py, hot, classes, args))).map_err(|panic| PyRuntimeError::new_err(format!("Session.hot_assemble: native panic: {}", crate::panic_message(&panic))))?;
        match outcome {
            Ok(found) => Ok(Ok(found)),
            Err(Fail::Py(error)) => Err(error),
            Err(Fail::Unsupported(text)) => Ok(Err(text)),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::blank;

    #[test]
    fn blank_is_what_python_strip_empties() {
        assert!(blank(""));
        assert!(blank(" \t\n\u{1c}\u{1f}\u{a0}\u{2003}"));
        assert!(!blank(" a "));
        assert!(!blank("face:0"));
    }
}
