//! `frame` at the boundary: the owner faces and vertices of `build_rational_affine_planar_metric` read from the Python graph, the frame computed by `cftuv-frame`, the metric built back.
//!
//! The op is ACCEPT-OR-DECLINE (see the crate note): the extension returns the `RationalAffinePlanarMetricV2` the Python builder would return for a patch that is exactly planar after the
//! source snap, or it declines by name BEFORE anything else happened and the shim runs the Python builder. The reading is strict by construction:
//!
//! * every record is read only from an instance of EXACTLY the class the contract names (a subclass, a mock, a record of another class is a decline), every leaf is an exact `str`, `float`,
//!   `tuple`, `frozenset`; an id is an instance of exactly its own id class;
//! * the slots are read and written at the offsets the probe of a real instance found (`pyobj::Raw`, the slot mode `CFTUV_NATIVE_SLOTS`: `auto`, `raw`, `attr`); a class whose slots are not
//!   exactly the ones the port writes is a refusal when the classes are bound (`NativePortStale` in the shim: the contract changed under the port);
//! * the result is built from Rust (`PyType_GenericAlloc` and slot writes, no `__init__` and no `__post_init__`): the numbers the oracle's `__post_init__` checks (reduced rationals, positive
//!   denominators, a power-of-two scale, the consistent search record) hold by construction of `cftuv-frame` and are held to the oracle by the corpora.
//!
//! Answers: `(0, metric)` or `(2, reason code, detail)`. A panic is a `RuntimeError`.

use std::collections::HashMap;
use std::panic::{catch_unwind, AssertUnwindSafe};
use std::time::Instant;

use cftuv_core::num::{IBig, UBig};
use cftuv_core::rat::Rat;
use cftuv_frame::grid::{AUTHOR_ANGULAR_ERROR, DECAL_DETAIL, MAGNITUDE_BOUND};
use cftuv_frame::model::{Decline, FaceIn, Input, Reason};
use pyo3::exceptions::PyRuntimeError;
use pyo3::prelude::*;
use pyo3::types::{PyBool, PyDict, PyFloat, PyFrozenSet, PyString, PyTuple};
use pyo3::types::PyFrozenSetMethods;
use pyo3::{ffi, intern, IntoPyObjectExt};
use sha2::{Digest, Sha256};
use std::sync::atomic::{AtomicU64, Ordering};

use crate::pyobj::{alloc, int_of_ibig, int_of_ubig, note_attr_build, note_attr_reads, process_mode, refuse, set_slot, Raw, SlotMode};

/// A failure of reading: a Python error (surfaces as an exception), or a decline (the input is not the canonical shape of the contract).
enum Fail {
    Py(PyErr),
    Decline(Decline),
}

impl From<PyErr> for Fail {
    fn from(error: PyErr) -> Fail {
        Fail::Py(error)
    }
}

type Read<T> = Result<T, Fail>;

fn shape<T>(text: impl Into<String>) -> Read<T> {
    Err(Fail::Decline(Decline { reason: Reason::Shape, detail: text.into() }))
}

// --------------------------------------------------------------------------
// classes
// --------------------------------------------------------------------------

/// The slots of a class in the order of its MRO (`__slots__` of every class that has them).
fn all_slots(py: Python<'_>, class: &Bound<'_, PyAny>) -> PyResult<Vec<String>> {
    let mut found: Vec<String> = Vec::new();
    for klass in class.getattr(intern!(py, "__mro__"))?.try_iter()? {
        let klass = klass?;
        if klass.as_ptr() == py.get_type::<pyo3::types::PyAny>().as_ptr() {
            continue;
        }
        let dictionary = klass.getattr(intern!(py, "__dict__"))?;
        let Ok(slots) = dictionary.get_item("__slots__") else { continue };
        if slots.is_exact_instance_of::<PyString>() {
            found.push(slots.extract::<String>()?);
            continue;
        }
        for slot in slots.try_iter()? {
            let slot: String = slot?.extract()?;
            if slot != "__dict__" && slot != "__weakref__" {
                found.push(slot);
            }
        }
    }
    Ok(found)
}

/// A class the port reads or writes: its slots, the position of each field the port asks for, and the raw access when the probe confirms the layout.
struct Cls {
    class: Py<PyAny>,
    class_ptr: usize,
    names: Vec<Py<PyString>>,
    position: Vec<usize>,
    raw: Option<Raw>,
    label: String,
}

impl Cls {
    /// `wanted`: the fields the port reads (`get(object, k)` is the k-th) or, for a class it writes (`exact`), ALL its slots in the order of its definition: anything else is the contract
    /// changing under the port.
    fn new(py: Python<'_>, mode: SlotMode, label: &str, class: &Bound<'_, PyAny>, wanted: &[&str], exact: bool) -> PyResult<Cls> {
        let slots = all_slots(py, class)?;
        if exact && slots.iter().map(String::as_str).ne(wanted.iter().copied()) {
            return Err(refuse(format!("{label} has the slots {slots:?}, the port was written for {wanted:?} (the contract changed under the port)")));
        }
        let mut position = Vec::with_capacity(wanted.len());
        for field in wanted {
            match slots.iter().position(|slot| slot == field) {
                Some(index) => position.push(index),
                None => return Err(refuse(format!("{label} has no slot `{field}` (the contract changed under the port)"))),
            }
        }
        let names: Vec<Py<PyString>> = slots.iter().map(|name| PyString::new(py, name).unbind()).collect();
        let class_owned = class.clone().unbind();
        let name_refs: Vec<&Py<PyString>> = names.iter().collect();
        let raw = Raw::select(py, mode, label, &class_owned, &name_refs)?;
        Ok(Cls { class_ptr: class.as_ptr() as usize, class: class_owned, names, position, raw, label: label.to_string() })
    }

    fn is(&self, object: &Bound<'_, PyAny>) -> bool {
        object.get_type().as_ptr() as usize == self.class_ptr
    }

    /// The k-th requested field of an instance of exactly this class.
    fn get<'py>(&self, object: &Bound<'py, PyAny>, k: usize) -> Read<Bound<'py, PyAny>> {
        let slot = self.position[k];
        if let Some(raw) = &self.raw {
            if let Some(found) = raw.read(object, slot) {
                return Ok(found.to_owned());
            }
        }
        note_attr_reads(1);
        match object.getattr(self.names[slot].bind(object.py())) {
            Ok(value) => Ok(value),
            Err(_) => shape(format!("a slot of {} is empty", self.label)),
        }
    }

    /// A new instance with every slot set (the slots in the order of the class).
    fn build<'py>(&self, py: Python<'py>, values: Vec<Bound<'py, PyAny>>) -> PyResult<Bound<'py, PyAny>> {
        if values.len() != self.names.len() {
            return Err(refuse(format!("{}: {} values for {} slots", self.label, values.len(), self.names.len())));
        }
        if let Some(raw) = &self.raw {
            macro_rules! arm {
                ($n:literal) => {{
                    let array: [Bound<'py, PyAny>; $n] = values.try_into().map_err(|_| refuse("raw build: the number of values changed"))?;
                    raw.build(py, &self.class, array)
                }};
            }
            return match self.names.len() {
                1 => arm!(1),
                2 => arm!(2),
                3 => arm!(3),
                4 => arm!(4),
                7 => arm!(7),
                14 => arm!(14),
                other => Err(refuse(format!("{}: no raw builder for {other} slots", self.label))),
            };
        }
        note_attr_build();
        let object = alloc(py, &self.class)?;
        for (name, value) in self.names.iter().zip(values.iter()) {
            set_slot(&object, name, value)?;
        }
        Ok(object)
    }
}

fn class_of<'py>(classes: &Bound<'py, PyDict>, name: &str) -> PyResult<Bound<'py, PyAny>> {
    match classes.get_item(name)? {
        Some(found) => Ok(found),
        None => Err(refuse(format!("the shim did not hand over `{name}`"))),
    }
}

struct Bindings {
    vertex: Cls,
    local_point: Cls,
    source_face: Cls,
    vertex_id: Cls,
    edge_id: Cls,
    face_id: Cls,
    patch_id: Cls,
    // the result
    metric: Cls,
    plane_certificate: Cls,
    grid_certificate: Cls,
    trial: Cls,
    rational: Cls,
    point2: Cls,
    point3: Cls,
    vector3: Cls,
    matrix2: Cls,
    coordinate: Cls,
    metric_id: Cls,
    certificate_id: Cls,
    // the members of the enums the result carries, by identity
    chart_ccw: Py<PyAny>,
    chart_cw: Py<PyAny>,
    frame_law: Py<PyAny>,
    admission_law: Py<PyAny>,
    reconstruction_law: Py<PyAny>,
    window_outcome: Py<PyAny>,
    search_order: Py<PyAny>,
    trial_restored: Py<PyAny>,
    trial_not_restored: Py<PyAny>,
}

impl Bindings {
    fn new(py: Python<'_>, classes: &Bound<'_, PyDict>, members: &Bound<'_, PyDict>) -> PyResult<Bindings> {
        let mode = process_mode();
        let reads = |name: &str, wanted: &[&str]| -> PyResult<Cls> { Cls::new(py, mode, name, &class_of(classes, name)?, wanted, false) };
        let writes = |name: &str, slots: &[&str]| -> PyResult<Cls> { Cls::new(py, mode, name, &class_of(classes, name)?, slots, true) };
        let member = |name: &str| -> PyResult<Py<PyAny>> { Ok(class_of(members, name)?.unbind()) };
        Ok(Bindings {
            vertex: reads("SourceVertexV1", &["vertex_id", "position"])?,
            local_point: reads("LocalPoint3V1", &["x", "y", "z"])?,
            source_face: reads("SourceFaceV1", &["face_id", "patch_id", "vertex_cycle", "edge_cycle"])?,
            vertex_id: reads("SourceVertexId", &["value"])?,
            edge_id: reads("PhysicalEdgeId", &["value"])?,
            face_id: reads("SourceFaceId", &["value"])?,
            patch_id: reads("PatchId", &["value"])?,
            metric: writes(
                "RationalAffinePlanarMetricV2",
                &[
                    "reference_metric_id", "patch_domain_id", "source_revision", "exact_origin", "exact_basis_a", "exact_basis_b", "exact_gram_matrix", "exact_inverse_gram_matrix",
                    "exact_source_vertex_coordinates", "chart_orientation", "frame_selection_law", "planarity_certificate", "source_lineage", "grid_certificate",
                ],
            )?,
            plane_certificate: writes(
                "ExactSourcePlaneCertificateV1",
                &["certificate_id", "patch_domain_id", "admission_law", "exact", "exact_plane_normal", "source_vertex_ids", "reconstruction_law"],
            )?,
            grid_certificate: writes(
                "IntegerGridCertificateV1",
                &[
                    "snapping_law", "window_outcome", "patch_extent", "author_angular_error", "decal_detail", "window_lower_bound", "window_upper_bound", "window_step", "source_scale",
                    "magnitude_bound", "intended_right_corners", "restored_right_corners", "search_order", "scale_trials",
                ],
            )?,
            trial: writes("GridScaleTrialV1", &["scale", "step", "restored_right_corners", "outcome"])?,
            rational: writes("ExactRationalV1", &["numerator", "denominator"])?,
            point2: writes("ExactPoint2V1", &["x", "y"])?,
            point3: writes("ExactPoint3V1", &["x", "y", "z"])?,
            vector3: writes("ExactVector3V1", &["x", "y", "z"])?,
            matrix2: writes("ExactMatrix2V1", &["m00", "m01", "m10", "m11"])?,
            coordinate: writes("ExactSourceVertexCoordinateV2", &["source_vertex_id", "domain_coordinate"])?,
            metric_id: writes("ReferenceMetricId", &["value"])?,
            certificate_id: writes("PlanarityCertificateId", &["value"])?,
            chart_ccw: member("chart_ccw")?,
            chart_cw: member("chart_cw")?,
            frame_law: member("frame_law")?,
            admission_law: member("admission_law")?,
            reconstruction_law: member("reconstruction_law")?,
            window_outcome: member("window_outcome")?,
            search_order: member("search_order")?,
            trial_restored: member("trial_restored")?,
            trial_not_restored: member("trial_not_restored")?,
        })
    }
}

// --------------------------------------------------------------------------
// the reader
// --------------------------------------------------------------------------

/// What the numbered input needs from the Python graph besides the numbers: the id objects the result reuses, by rank.
struct Numbered<'py> {
    input: Input,
    vertex_objects: Vec<Bound<'py, PyAny>>,
    vertex_values: Vec<String>,
}

fn text_of<'a>(object: &'a Bound<'_, PyAny>, what: &str) -> Read<&'a str> {
    if !object.is_exact_instance_of::<PyString>() {
        return shape(format!("{what} is not a str"));
    }
    let text = object.cast::<PyString>().map_err(|_| Fail::Decline(Decline { reason: Reason::Shape, detail: format!("{what} is not a str") }))?;
    Ok(text.to_str()?)
}

/// The value of an id of exactly `class`.
fn id_value<'py>(class: &Cls, object: &Bound<'py, PyAny>, what: &str) -> Read<Bound<'py, PyAny>> {
    if !class.is(object) {
        return shape(format!("{what} is not exactly a {}", class.label));
    }
    class.get(object, 0)
}

fn tuple_of<'py>(object: &Bound<'py, PyAny>, what: &str) -> Read<Bound<'py, PyTuple>> {
    if !object.is_exact_instance_of::<PyTuple>() {
        return shape(format!("{what} is not a tuple"));
    }
    object.clone().cast_into::<PyTuple>().map_err(|_| Fail::Decline(Decline { reason: Reason::Shape, detail: format!("{what} is not a tuple") }))
}

fn frozenset_of<'py>(object: &Bound<'py, PyAny>, what: &str) -> Read<Bound<'py, PyFrozenSet>> {
    if !object.is_exact_instance_of::<PyFrozenSet>() {
        return shape(format!("{what} is not a frozenset"));
    }
    object.clone().cast_into::<PyFrozenSet>().map_err(|_| Fail::Decline(Decline { reason: Reason::Shape, detail: format!("{what} is not a frozenset") }))
}

fn float_of(object: &Bound<'_, PyAny>, what: &str) -> Read<f64> {
    if !object.is_exact_instance_of::<PyFloat>() {
        return shape(format!("{what} is not a float"));
    }
    // SAFETY: an exact float.
    let value = unsafe { ffi::PyFloat_AsDouble(object.as_ptr()) };
    if !value.is_finite() {
        return shape(format!("{what} is not finite"));
    }
    Ok(value)
}

impl Bindings {
    /// The owner faces and the positions of their vertices, numbered the way `cftuv-frame` wants them (the oracle's `_source_scope`).
    fn read<'py>(&self, source_vertices: &Bound<'py, PyAny>, source_faces: &Bound<'py, PyAny>, owner_patch_id: &Bound<'py, PyAny>) -> Read<Numbered<'py>> {
        let owner_value_object = id_value(&self.patch_id, owner_patch_id, "the owner patch id")?;
        let owner_value = text_of(&owner_value_object, "the owner patch id value")?.to_owned();
        // the faces of the owner patch, sorted by the value of their id
        let mut selected: Vec<(String, Bound<'py, PyTuple>, Bound<'py, PyTuple>)> = Vec::new();
        for face in frozenset_of(source_faces, "source_faces")?.iter() {
            if !self.source_face.is(&face) {
                return shape("a face that is not exactly a SourceFaceV1");
            }
            let patch = id_value(&self.patch_id, &self.source_face.get(&face, 1)?, "the patch id of a face")?;
            if text_of(&patch, "the patch id value of a face")? != owner_value {
                continue;
            }
            let key_object = id_value(&self.face_id, &self.source_face.get(&face, 0)?, "the id of a face")?;
            let key = text_of(&key_object, "the face id value")?.to_owned();
            let vertices = tuple_of(&self.source_face.get(&face, 2)?, "a vertex_cycle")?;
            let edges = tuple_of(&self.source_face.get(&face, 3)?, "an edge_cycle")?;
            if vertices.len() != edges.len() || vertices.len() < 3 {
                return shape("a face whose cycles are shorter than three or differ in length");
            }
            selected.push((key, vertices, edges));
        }
        selected.sort_by(|left, right| left.0.as_bytes().cmp(right.0.as_bytes()));
        // the distinct vertices of those faces
        let mut by_pointer: HashMap<usize, usize> = HashMap::new();
        let mut by_value: HashMap<String, usize> = HashMap::new();
        let mut distinct: Vec<(String, Bound<'py, PyAny>)> = Vec::new();
        for (_, vertices, _) in &selected {
            for vertex in vertices.iter() {
                if by_pointer.contains_key(&(vertex.as_ptr() as usize)) {
                    continue;
                }
                let value_object = id_value(&self.vertex_id, &vertex, "a vertex id of a face")?;
                let value = text_of(&value_object, "a vertex id value")?;
                let slot = match by_value.get(value) {
                    Some(slot) => *slot,
                    None => {
                        distinct.push((value.to_owned(), vertex.clone()));
                        by_value.insert(value.to_owned(), distinct.len() - 1);
                        distinct.len() - 1
                    }
                };
                by_pointer.insert(vertex.as_ptr() as usize, slot);
            }
        }
        if distinct.len() < 3 {
            return shape("fewer than three source vertices in the faces of the patch");
        }
        // ranks: the order of the values (UTF-8 bytes, which is Python's str order)
        let mut order: Vec<usize> = (0..distinct.len()).collect();
        order.sort_by(|left, right| distinct[*left].0.as_bytes().cmp(distinct[*right].0.as_bytes()));
        let mut rank_of = vec![0u32; distinct.len()];
        for (rank, slot) in order.iter().enumerate() {
            rank_of[*slot] = rank as u32;
        }
        // the positions of the required vertices
        let mut positions: Vec<Option<[f64; 3]>> = vec![None; distinct.len()];
        for vertex in frozenset_of(source_vertices, "source_vertices")?.iter() {
            if !self.vertex.is(&vertex) {
                return shape("a vertex that is not exactly a SourceVertexV1");
            }
            let id = self.vertex.get(&vertex, 0)?;
            if !self.vertex_id.is(&id) {
                // an id of another class cannot be one of the face vertices (those are exactly SourceVertexId)
                continue;
            }
            let value_object = self.vertex_id.get(&id, 0)?;
            let value = text_of(&value_object, "a vertex id value")?;
            let Some(slot) = by_value.get(value).copied() else { continue };
            let position = self.vertex.get(&vertex, 1)?;
            if !self.local_point.is(&position) {
                return shape("a required source vertex without a LocalPoint3V1 position");
            }
            if positions[slot].is_some() {
                return shape("two source vertices with the same id value");
            }
            positions[slot] = Some([
                float_of(&self.local_point.get(&position, 0)?, "x")?,
                float_of(&self.local_point.get(&position, 1)?, "y")?,
                float_of(&self.local_point.get(&position, 2)?, "z")?,
            ]);
        }
        let mut ordered_positions = Vec::with_capacity(distinct.len());
        for slot in &order {
            match positions[*slot] {
                Some(found) => ordered_positions.push(found),
                None => return shape("a vertex of the faces is not among the source vertices"),
            }
        }
        // the faces as ranks and edge numbers
        let mut edge_slots: HashMap<String, u32> = HashMap::new();
        let mut edge_by_pointer: HashMap<usize, u32> = HashMap::new();
        let mut edge_names: Vec<String> = Vec::new();
        let mut faces: Vec<FaceIn> = Vec::with_capacity(selected.len());
        for (key, vertices, edges) in selected {
            let mut ranks = Vec::with_capacity(vertices.len());
            for vertex in vertices.iter() {
                ranks.push(rank_of[by_pointer[&(vertex.as_ptr() as usize)]]);
            }
            let mut numbers = Vec::with_capacity(edges.len());
            for edge in edges.iter() {
                if let Some(found) = edge_by_pointer.get(&(edge.as_ptr() as usize)) {
                    numbers.push(*found);
                    continue;
                }
                let value_object = id_value(&self.edge_id, &edge, "an edge id of a face")?;
                let value = text_of(&value_object, "an edge id value")?;
                let slot = match edge_slots.get(value) {
                    Some(slot) => *slot,
                    None => {
                        edge_names.push(value.to_owned());
                        edge_slots.insert(value.to_owned(), edge_names.len() as u32 - 1);
                        edge_names.len() as u32 - 1
                    }
                };
                edge_by_pointer.insert(edge.as_ptr() as usize, slot);
                numbers.push(slot);
            }
            faces.push(FaceIn { key, vertices: ranks, edges: numbers });
        }
        let vertex_values: Vec<String> = order.iter().map(|slot| distinct[*slot].0.clone()).collect();
        let vertex_objects: Vec<Bound<'py, PyAny>> = order.iter().map(|slot| distinct[*slot].1.clone()).collect();
        Ok(Numbered { input: Input { positions: ordered_positions, faces, edge_names, snaps_source: true }, vertex_objects, vertex_values })
    }
}

// --------------------------------------------------------------------------
// the builder
// --------------------------------------------------------------------------

fn stable_id(kind: &str, parts: &[&str]) -> String {
    let mut payload = String::from(kind);
    for part in parts {
        payload.push('\u{1f}');
        payload.push_str(part);
    }
    let digest = Sha256::digest(payload.as_bytes());
    let mut hex = String::with_capacity(24);
    for byte in digest.iter().take(12) {
        hex.push_str(&format!("{byte:02x}"));
    }
    format!("{kind}:{hex}")
}

struct Builder<'b, 'py> {
    b: &'b Bindings,
    py: Python<'py>,
}

impl<'b, 'py> Builder<'b, 'py> {
    fn int(&self, value: &IBig) -> PyResult<Bound<'py, PyAny>> {
        int_of_ibig(self.py, value)
    }

    fn unsigned(&self, value: &UBig) -> PyResult<Bound<'py, PyAny>> {
        int_of_ubig(self.py, value)
    }

    fn small(&self, value: u64) -> PyResult<Bound<'py, PyAny>> {
        value.into_bound_py_any(self.py)
    }

    fn rational(&self, value: &Rat) -> PyResult<Bound<'py, PyAny>> {
        self.b.rational.build(self.py, vec![self.int(value.numerator())?, self.unsigned(value.denominator())?])
    }

    fn rational_of(&self, numerator: u64, denominator: u64) -> PyResult<Bound<'py, PyAny>> {
        self.b.rational.build(self.py, vec![self.small(numerator)?, self.small(denominator)?])
    }

    fn integer_rational(&self, value: &IBig) -> PyResult<Bound<'py, PyAny>> {
        self.b.rational.build(self.py, vec![self.int(value)?, self.small(1)?])
    }

    fn vector3(&self, values: &[Rat; 3]) -> PyResult<Bound<'py, PyAny>> {
        self.b.vector3.build(self.py, vec![self.rational(&values[0])?, self.rational(&values[1])?, self.rational(&values[2])?])
    }

    fn point3(&self, values: &[Rat; 3]) -> PyResult<Bound<'py, PyAny>> {
        self.b.point3.build(self.py, vec![self.rational(&values[0])?, self.rational(&values[1])?, self.rational(&values[2])?])
    }

    fn matrix(&self, m00: &Rat, m01: &Rat, m11: &Rat) -> PyResult<Bound<'py, PyAny>> {
        // `m10` is its own object, as the oracle builds it (`_rational(value[1][0])`)
        self.b.matrix2.build(self.py, vec![self.rational(m00)?, self.rational(m01)?, self.rational(m01)?, self.rational(m11)?])
    }

    fn text(&self, value: &str) -> Bound<'py, PyAny> {
        PyString::new(self.py, value).into_any()
    }

    /// The metric from the frame; `ids`: the vertex id objects by rank; `domain`, `revision`, `lineage`, `law`: the objects of the call.
    #[allow(clippy::too_many_arguments)]
    fn metric(
        &self,
        frame: &cftuv_frame::Frame,
        numbered: &Numbered<'py>,
        domain: &Bound<'py, PyAny>,
        domain_value: &str,
        revision: &Bound<'py, PyAny>,
        revision_value: &str,
        lineage: &Bound<'py, PyAny>,
        law: &Bound<'py, PyAny>,
    ) -> PyResult<Bound<'py, PyAny>> {
        let py = self.py;
        let b = self.b;
        let scale = UBig::ONE << frame.grid.exponent as usize;
        // the certificate of the grid
        let mut trials: Vec<Bound<'py, PyAny>> = Vec::with_capacity(frame.grid.trials.len());
        for trial in &frame.grid.trials {
            let trial_scale = UBig::ONE << trial.exponent as usize;
            let outcome = if trial.passed { &b.trial_restored } else { &b.trial_not_restored };
            trials.push(b.trial.build(
                py,
                vec![
                    self.unsigned(&trial_scale)?,
                    self.rational_pair(&trial_scale)?,
                    self.small(u64::from(trial.restored))?,
                    outcome.bind(py).clone(),
                ],
            )?);
        }
        let restored = frame.grid.trials.last().map_or(0, |trial| trial.restored);
        let grid = b.grid_certificate.build(
            py,
            vec![
                law.clone(),
                b.window_outcome.bind(py).clone(),
                self.rational(&frame.grid.extent)?,
                self.rational_of(AUTHOR_ANGULAR_ERROR.0, AUTHOR_ANGULAR_ERROR.1)?,
                self.rational_of(DECAL_DETAIL.0, DECAL_DETAIL.1)?,
                self.rational(&frame.grid.window_lower)?,
                self.rational(&frame.grid.window_upper)?,
                self.rational_pair(&scale)?,
                self.unsigned(&scale)?,
                self.small(u64::from(MAGNITUDE_BOUND))?,
                self.small(u64::from(frame.grid.intended_right_corners))?,
                self.small(u64::from(restored))?,
                b.search_order.bind(py).clone(),
                PyTuple::new(py, trials)?.into_any(),
            ],
        )?;
        // the vertices and the coordinates, in the order of the ids (the oracle's `required_ids`)
        let mut coordinates: Vec<Bound<'py, PyAny>> = Vec::with_capacity(numbered.vertex_objects.len());
        for (rank, pair) in frame.coordinates.iter().enumerate() {
            let point = b.point2.build(py, vec![self.rational(&pair[0])?, self.rational(&pair[1])?])?;
            coordinates.push(b.coordinate.build(py, vec![numbered.vertex_objects[rank].clone(), point])?);
        }
        let coordinate_set = PyFrozenSet::new(py, coordinates)?.into_any();
        let normal = b.vector3.build(py, vec![self.integer_rational(&frame.normal[0])?, self.integer_rational(&frame.normal[1])?, self.integer_rational(&frame.normal[2])?])?;
        let certificate_id = b.certificate_id.build(py, vec![self.text(&stable_id("exact-source-plane", &[revision_value, domain_value]))])?;
        let certificate = b.plane_certificate.build(
            py,
            vec![
                certificate_id,
                domain.clone(),
                b.admission_law.bind(py).clone(),
                PyBool::new(py, true).to_owned().into_any(),
                normal,
                PyFrozenSet::new(py, numbered.vertex_objects.iter())?.into_any(),
                b.reconstruction_law.bind(py).clone(),
            ],
        )?;
        let mut id_parts: Vec<&str> = vec![revision_value, domain_value];
        id_parts.extend(numbered.vertex_values.iter().map(String::as_str).take(1));
        id_parts.extend(numbered.vertex_values.iter().map(String::as_str));
        let metric_id = b.metric_id.build(py, vec![self.text(&stable_id("reference-metric", &id_parts))])?;
        let orientation = if frame.orientation_sign > 0 { &b.chart_ccw } else { &b.chart_cw };
        b.metric.build(
            py,
            vec![
                metric_id,
                domain.clone(),
                revision.clone(),
                self.point3(&frame.origin)?,
                self.vector3(&frame.basis_a)?,
                self.vector3(&frame.basis_b)?,
                self.matrix(&frame.gram[0], &frame.gram[1], &frame.gram[2])?,
                self.matrix(&frame.inverse[0], &frame.inverse[1], &frame.inverse[2])?,
                coordinate_set,
                orientation.bind(py).clone(),
                b.frame_law.bind(py).clone(),
                certificate,
                lineage.clone(),
                grid,
            ],
        )
    }

    /// `ExactRationalV1(1, scale)` (the step of a power-of-two scale).
    fn rational_pair(&self, scale: &UBig) -> PyResult<Bound<'py, PyAny>> {
        self.b.rational.build(self.py, vec![self.small(1)?, self.unsigned(scale)?])
    }
}

// --------------------------------------------------------------------------
// the entry point
// --------------------------------------------------------------------------

/// The classes of the kernel the reader and the builder are bound on, once; `compute` is the whole operation.
#[pyclass(module = "cftuv_native._core", frozen)]
pub struct FrameOp {
    bindings: Bindings,
    last_read: AtomicU64,
    last_compute: AtomicU64,
    last_build: AtomicU64,
}

fn nanos(since: Instant) -> u64 {
    u64::try_from(since.elapsed().as_nanos()).unwrap_or(u64::MAX)
}

#[pymethods]
impl FrameOp {
    /// `FrameOp(classes, members)`: `classes` maps a class name to the live class of the kernel, `members` an enum role to the live member; a name missing or a slot list that is not exactly
    /// the one the port was written for is a named refusal (`TypeError("cftuv_native: ...")`, which the shim turns into `NativePortStale`).
    #[new]
    fn new(py: Python<'_>, classes: &Bound<'_, PyDict>, members: &Bound<'_, PyDict>) -> PyResult<FrameOp> {
        Ok(FrameOp { bindings: Bindings::new(py, classes, members)?, last_read: AtomicU64::new(0), last_compute: AtomicU64::new(0), last_build: AtomicU64::new(0) })
    }

    /// `compute(source_vertices, source_faces, owner_patch_id, source_revision, patch_domain_id, source_lineage, snapping_law, snaps_source, decimal_precision)`:
    /// `(0, metric)` or `(2, reason code, detail)`.
    #[allow(clippy::too_many_arguments)]
    fn compute<'py>(
        &self,
        py: Python<'py>,
        source_vertices: &Bound<'py, PyAny>,
        source_faces: &Bound<'py, PyAny>,
        owner_patch_id: &Bound<'py, PyAny>,
        source_revision: &Bound<'py, PyAny>,
        patch_domain_id: &Bound<'py, PyAny>,
        source_lineage: &Bound<'py, PyAny>,
        snapping_law: &Bound<'py, PyAny>,
        snaps_source: bool,
    ) -> PyResult<Bound<'py, PyAny>> {
        let started = Instant::now();
        let read = self.bindings.read(source_vertices, source_faces, owner_patch_id).and_then(|mut numbered| {
            numbered.input.snaps_source = snaps_source;
            let revision = id_value_any(source_revision)?;
            let domain = id_value_any(patch_domain_id)?;
            Ok((numbered, revision, domain))
        });
        self.last_read.store(nanos(started), Ordering::Relaxed);
        let (numbered, revision_value, domain_value) = match read {
            Ok(found) => found,
            Err(Fail::Py(error)) => return Err(error),
            Err(Fail::Decline(decline)) => return (2u8, decline.reason.code(), decline.detail).into_bound_py_any(py),
        };
        let started = Instant::now();
        let outcome = catch_unwind(AssertUnwindSafe(|| cftuv_frame::resolve(&numbered.input)));
        self.last_compute.store(nanos(started), Ordering::Relaxed);
        let frame = match outcome {
            Ok(Ok(frame)) => frame,
            Ok(Err(decline)) => return (2u8, decline.reason.code(), decline.detail).into_bound_py_any(py),
            Err(panic) => return Err(PyRuntimeError::new_err(format!("frame: native panic: {}", crate::panic_message(&panic)))),
        };
        let started = Instant::now();
        let builder = Builder { b: &self.bindings, py };
        let metric = builder.metric(&frame, &numbered, patch_domain_id, &domain_value, source_revision, &revision_value, source_lineage, snapping_law);
        self.last_build.store(nanos(started), Ordering::Relaxed);
        (0u8, metric?).into_bound_py_any(py)
    }

    /// `(read ns, compute ns, build ns)` of the last call.
    fn last_timings(&self) -> (u64, u64, u64) {
        (self.last_read.load(Ordering::Relaxed), self.last_compute.load(Ordering::Relaxed), self.last_build.load(Ordering::Relaxed))
    }

    /// Test-only: which classes read or write their slots by offset: `[(class name, raw)]`.
    fn raw_layouts(&self) -> Vec<(String, bool)> {
        let b = &self.bindings;
        [
            &b.vertex, &b.local_point, &b.source_face, &b.vertex_id, &b.edge_id, &b.face_id, &b.patch_id, &b.metric, &b.plane_certificate, &b.grid_certificate, &b.trial, &b.rational, &b.point2, &b.point3,
            &b.vector3, &b.matrix2, &b.coordinate, &b.metric_id, &b.certificate_id,
        ]
        .iter()
        .map(|cls| (cls.label.clone(), cls.raw.is_some()))
        .collect()
    }
}

/// The `value` (a `str`) of an opaque id object, whatever its class (the revision and the domain are handed over by the caller, who built them).
fn id_value_any(object: &Bound<'_, PyAny>) -> Read<String> {
    let value = object.getattr(intern!(object.py(), "value"))?;
    Ok(text_of(&value, "an id value")?.to_owned())
}
