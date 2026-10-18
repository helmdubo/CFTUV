//! `CHAIN_STATION_PLAN_V1` at the boundary: the parts of an `AnalysisSnapshotV1` the law reads, handed to `cftuv-station` in the iteration order of the Python containers, and the plans back.
//!
//! The reading is strict like `snapshot_check.rs` (whose class and member readers it shares): every record of exactly the class the contract names, every id of exactly its id class, every
//! container exactly a `frozenset` or a `tuple`, every coordinate an exact finite `float`. Anything else is a decline (`(2, detail)`): the shim runs the Python law, which then answers or raises
//! as it always did. The answer of the law is `(0, plans)`, `plans = ((chain, ((ordinal, kind, disposition, reason, ((face id, face id), ...)), ...)), ...)` with the chain record and the face id
//! objects of the snapshot itself, and the codes of `cftuv-station` (`StationKind`, `Disposition`, `Reason`). A panic is a `RuntimeError`.

use std::collections::HashMap;
use std::panic::{catch_unwind, AssertUnwindSafe};

use cftuv_canon::fxhash::FxBuild;
use cftuv_core::num::IBig;
use cftuv_station::{chain_station_plans, ChainIn, ChainKind, Input, Ix, NeighbourFace, SurfaceFace};
use pyo3::exceptions::PyRuntimeError;
use pyo3::prelude::*;
use pyo3::types::{PyBool, PyDict, PyFloat, PyFrozenSet, PyInt, PyString, PyTuple};
use pyo3::{ffi, IntoPyObjectExt};

use crate::pyobj::{ibig_from_int, process_mode};
use crate::snapshot_check::{class_of, shape, Cls, Fail, Members, Read};

struct Bindings {
    vertex_id: Cls,
    face_id: Cls,
    patch_id: Cls,
    chain_id: Cls,
    source_face: Cls,
    vertex: Cls,
    local_point: Cls,
    unavailable_position_ptr: usize,
    chain: Cls,
    neighbour: Cls,
    chain_kind: Members,
}

impl Bindings {
    fn new(py: Python<'_>, classes: &Bound<'_, PyDict>) -> PyResult<Bindings> {
        let mode = process_mode();
        let cls = |name: &str, wanted: &[&str]| -> PyResult<Cls> { Cls::new(py, mode, name, &class_of(classes, name)?, wanted) };
        Ok(Bindings {
            vertex_id: cls("SourceVertexId", &["value"])?,
            face_id: cls("SourceFaceId", &["value"])?,
            patch_id: cls("PatchId", &["value"])?,
            chain_id: cls("PhysicalChainId", &["value"])?,
            source_face: cls("SourceFaceV1", &["face_id", "patch_id", "vertex_cycle"])?,
            vertex: cls("SourceVertexV1", &["vertex_id", "position"])?,
            local_point: cls("LocalPoint3V1", &["x", "y", "z"])?,
            unavailable_position_ptr: class_of(classes, "UnavailableSourcePositionV1")?.as_ptr() as usize,
            chain: cls("PhysicalChainV1", &["physical_chain_id", "kind", "is_closed", "ordered_source_vertex_ids"])?,
            neighbour: cls("SeamNeighbourFaceV1", &["face_id", "patch_id", "vertex_ids", "positions"])?,
            chain_kind: Members::new(py, &class_of(classes, "PhysicalChainKind")?, &["PHYSICAL_DECAL_SOURCE", "PHYSICAL_SEAM", "SEAM_SELF"])?,
        })
    }
}

/// The ids of one class, interned by the bytes of their value (borrowed from the strings the snapshot holds for the whole call).
#[derive(Default)]
struct Table<'a> {
    map: HashMap<&'a [u8], u32, FxBuild>,
    values: Vec<&'a [u8]>,
}

impl<'a> Table<'a> {
    fn intern(&mut self, bytes: &'a [u8]) -> u32 {
        if let Some(found) = self.map.get(bytes) {
            return *found;
        }
        let next = self.values.len() as u32;
        self.values.push(bytes);
        self.map.insert(bytes, next);
        next
    }

    /// The rank of each id by its value (`str` order is the byte order of UTF-8).
    fn ranks(&self) -> Vec<u32> {
        let mut order: Vec<u32> = (0..self.values.len() as u32).collect();
        order.sort_by(|a, b| self.values[*a as usize].cmp(self.values[*b as usize]));
        let mut rank = vec![0u32; order.len()];
        for (position, index) in order.into_iter().enumerate() {
            rank[index as usize] = position as u32;
        }
        rank
    }
}

struct Reader<'b, 'a, 'py> {
    b: &'b Bindings,
    py: Python<'py>,
    vertices: Table<'a>,
    faces: Table<'a>,
    patches: Table<'a>,
    chains: Table<'a>,
    face_objects: Vec<Bound<'py, PyAny>>,
}

fn utf8<'a>(py: Python<'_>, object: &Bound<'_, PyAny>) -> Read<&'a [u8]> {
    if !object.is_exact_instance_of::<PyString>() {
        return shape("an id value is not a str");
    }
    let mut size: ffi::Py_ssize_t = 0;
    // SAFETY: an exact `str`; its UTF-8 buffer lives as long as the string, which the snapshot (held by the caller for the whole call) keeps alive.
    let data = unsafe { ffi::PyUnicode_AsUTF8AndSize(object.as_ptr(), &mut size) };
    if data.is_null() {
        return Err(Fail::Py(PyErr::fetch(py)));
    }
    // SAFETY: borrowed from a live immutable string for the lifetime of the call.
    Ok(unsafe { std::slice::from_raw_parts(data.cast::<u8>(), size as usize) })
}

fn frozenset<'py>(object: &Bound<'py, PyAny>, what: &str) -> Read<Vec<Bound<'py, PyAny>>> {
    if !object.is_exact_instance_of::<PyFrozenSet>() {
        return shape(format!("{what} is not a frozenset"));
    }
    Ok(object.try_iter()?.collect::<PyResult<Vec<_>>>()?)
}

fn tuple<'py>(object: &Bound<'py, PyAny>, what: &str) -> Read<Vec<Bound<'py, PyAny>>> {
    if !object.is_exact_instance_of::<PyTuple>() {
        return shape(format!("{what} is not a tuple"));
    }
    let tuple = object.cast::<PyTuple>().map_err(|_| Fail::Py(PyRuntimeError::new_err("tuple cast")))?;
    Ok(tuple.iter().collect())
}

fn float(object: &Bound<'_, PyAny>) -> Read<f64> {
    if !object.is_exact_instance_of::<PyFloat>() {
        return shape("a coordinate is not a float");
    }
    // SAFETY: an exact float.
    let value = unsafe { ffi::PyFloat_AsDouble(object.as_ptr()) };
    if !value.is_finite() {
        return shape("a coordinate is not finite");
    }
    Ok(value)
}

fn boolean(object: &Bound<'_, PyAny>) -> Read<bool> {
    if !object.is_exact_instance_of::<PyBool>() {
        return shape("is_closed is not a bool");
    }
    // SAFETY: an exact bool is Py_True or Py_False.
    Ok(object.as_ptr() == unsafe { ffi::Py_True() })
}

fn rational(object: &Bound<'_, PyAny>) -> PyResult<(IBig, IBig)> {
    let pair = object.cast::<PyTuple>().map_err(|_| crate::pyobj::refuse("a rational is a (numerator, denominator) tuple"))?;
    if pair.len() != 2 {
        return Err(crate::pyobj::refuse("a rational is a (numerator, denominator) tuple"));
    }
    let (numerator, denominator) = (pair.get_item(0)?, pair.get_item(1)?);
    if !numerator.is_exact_instance_of::<PyInt>() || !denominator.is_exact_instance_of::<PyInt>() {
        return Err(crate::pyobj::refuse("a rational of exact ints"));
    }
    let (numerator, denominator) = (ibig_from_int(&numerator)?, ibig_from_int(&denominator)?);
    if denominator <= IBig::ZERO {
        return Err(crate::pyobj::refuse("a rational with a positive denominator"));
    }
    Ok((numerator, denominator))
}

impl<'b, 'a, 'py> Reader<'b, 'a, 'py> {
    fn id(&self, object: &Bound<'py, PyAny>, class: &Cls, what: &str) -> Read<&'a [u8]> {
        if !class.is(object) {
            return shape(format!("{what} is not exactly its id class"));
        }
        utf8(self.py, &class.get(object, 0)?)
    }

    fn vertex(&mut self, object: &Bound<'py, PyAny>) -> Read<Ix> {
        let bytes = self.id(object, &self.b.vertex_id, "a vertex id")?;
        Ok(self.vertices.intern(bytes))
    }

    fn vertex_tuple(&mut self, object: &Bound<'py, PyAny>, what: &str) -> Read<Vec<Ix>> {
        tuple(object, what)?.iter().map(|item| self.vertex(item)).collect()
    }

    fn face(&mut self, object: &Bound<'py, PyAny>) -> Read<Ix> {
        let bytes = self.id(object, &self.b.face_id, "a face id")?;
        let index = self.faces.intern(bytes);
        if index as usize == self.face_objects.len() {
            self.face_objects.push(object.clone());
        }
        Ok(index)
    }

    fn patch(&mut self, object: &Bound<'py, PyAny>) -> Read<Ix> {
        let bytes = self.id(object, &self.b.patch_id, "a patch id")?;
        Ok(self.patches.intern(bytes))
    }

    fn point(&self, object: &Bound<'py, PyAny>) -> Read<[f64; 3]> {
        let class = &self.b.local_point;
        if !class.is(object) {
            return shape("a position that is not exactly a LocalPoint3V1");
        }
        Ok([float(&class.get(object, 0)?)?, float(&class.get(object, 1)?)?, float(&class.get(object, 2)?)?])
    }

    /// The law's input and the chain records, in the snapshot's order. The shim reads the root of the snapshot and evaluates the oracle's own expressions for the domain's chains
    /// and the relation vertices (`item.patch_domain_id == patch_domain_id`, ...): those stay Python, the records are read here.
    fn read(&mut self, parts: &Parts<'_, 'py>) -> Read<(Input, Vec<Bound<'py, PyAny>>)> {
        let b = self.b;
        let mut source_positions = Vec::new();
        for item in frozenset(parts.source_vertices, "source_vertices")? {
            if !b.vertex.is(&item) {
                return shape("a vertex that is not exactly a SourceVertexV1");
            }
            let vertex = self.vertex(&b.vertex.get(&item, 0)?)?;
            let position = b.vertex.get(&item, 1)?;
            if b.local_point.is(&position) {
                source_positions.push((vertex, self.point(&position)?));
            } else if position.get_type().as_ptr() as usize != b.unavailable_position_ptr {
                return shape("a source position of an unknown class");
            }
        }
        let mut surface_faces = Vec::new();
        for item in frozenset(parts.source_faces, "source_faces")? {
            if !b.source_face.is(&item) {
                return shape("a face that is not exactly a SourceFaceV1");
            }
            let id = self.face(&b.source_face.get(&item, 0)?)?;
            let patch = self.patch(&b.source_face.get(&item, 1)?)?;
            let cycle = self.vertex_tuple(&b.source_face.get(&item, 2)?, "vertex_cycle")?;
            surface_faces.push(SurfaceFace { id, patch, cycle });
        }
        let mut neighbours = Vec::new();
        for item in frozenset(parts.neighbours, "seam_neighbour_faces")? {
            if !b.neighbour.is(&item) {
                return shape("a neighbour face that is not exactly a SeamNeighbourFaceV1");
            }
            let id = self.face(&b.neighbour.get(&item, 0)?)?;
            let patch = self.patch(&b.neighbour.get(&item, 1)?)?;
            let vertices = self.vertex_tuple(&b.neighbour.get(&item, 2)?, "vertex_ids")?;
            let positions = tuple(&b.neighbour.get(&item, 3)?, "positions")?.iter().map(|point| self.point(point)).collect::<Read<Vec<_>>>()?;
            neighbours.push(NeighbourFace { id, patch, vertices, positions });
        }
        let corner_vertices = self.vertex_tuple(parts.corner_vertices, "the relation vertices")?;
        let mut chains = Vec::new();
        let mut records = Vec::new();
        for item in frozenset(parts.chains, "physical_chains")? {
            if !b.chain.is(&item) {
                return shape("a chain that is not exactly a PhysicalChainV1");
            }
            let id = self.id(&b.chain.get(&item, 0)?, &b.chain_id, "a chain id")?;
            let kind = match b.chain_kind.code(&b.chain.get(&item, 1)?)? {
                0 => ChainKind::DecalSource,
                1 => ChainKind::Seam,
                2 => ChainKind::SeamSelf,
                _ => return shape("a chain kind the port does not know"),
            };
            let closed = boolean(&b.chain.get(&item, 2)?)?;
            let vertices = self.vertex_tuple(&b.chain.get(&item, 3)?, "ordered_source_vertex_ids")?;
            chains.push(ChainIn { id: self.chains.intern(id), kind, closed, vertices });
            records.push(item);
        }
        let mut domain_chains = Vec::new();
        for item in tuple(parts.domain_chains, "the domain's chain ids")? {
            let chain = self.id(&item, &b.chain_id, "a chain use's chain id")?;
            domain_chains.push(self.chains.intern(chain));
        }
        let input = Input {
            vertex_rank: self.vertices.ranks(),
            face_rank: self.faces.ranks(),
            chain_rank: self.chains.ranks(),
            full: parts.full,
            source_positions,
            surface_faces,
            neighbours,
            corner_vertices,
            chains,
            domain_chains,
            budget: (IBig::ZERO, IBig::ONE),
            bend: (IBig::ZERO, IBig::ONE),
            float_limit: 0.0,
        };
        Ok((input, records))
    }
}

/// What the shim hands over: the containers of the snapshot and the two expressions it evaluated.
struct Parts<'r, 'py> {
    full: bool,
    source_vertices: &'r Bound<'py, PyAny>,
    source_faces: &'r Bound<'py, PyAny>,
    neighbours: &'r Bound<'py, PyAny>,
    corner_vertices: &'r Bound<'py, PyAny>,
    chains: &'r Bound<'py, PyAny>,
    domain_chains: &'r Bound<'py, PyAny>,
}

/// The law bound to the live kernel classes, once per process (`chain_station_op._handle`).
#[pyclass(module = "cftuv_native._core", frozen)]
pub struct ChainStation {
    bindings: Bindings,
}

#[pymethods]
impl ChainStation {
    /// `ChainStation(classes)`: `classes` maps a class name to the live class of the kernel; a missing name or slot is a named refusal (`TypeError("cftuv_native: ...")`).
    #[new]
    fn new(py: Python<'_>, classes: &Bound<'_, PyDict>) -> PyResult<ChainStation> {
        Ok(ChainStation { bindings: Bindings::new(py, classes)? })
    }

    /// `plans(full, source_vertices, source_faces, seam_neighbour_faces, corner_vertices, physical_chains, domain_chain_ids, budget, bend, float_limit)`: `(0, plans)` or `(2, detail)`;
    /// `budget` and `bend` are `(numerator, denominator)`, `corner_vertices` and `domain_chain_ids` tuples of ids.
    #[allow(clippy::too_many_arguments)]
    fn plans<'py>(
        &self,
        py: Python<'py>,
        full: bool,
        source_vertices: &Bound<'py, PyAny>,
        source_faces: &Bound<'py, PyAny>,
        neighbours: &Bound<'py, PyAny>,
        corner_vertices: &Bound<'py, PyAny>,
        chains: &Bound<'py, PyAny>,
        domain_chains: &Bound<'py, PyAny>,
        budget: &Bound<'py, PyAny>,
        bend: &Bound<'py, PyAny>,
        float_limit: f64,
    ) -> PyResult<Bound<'py, PyAny>> {
        let budget = rational(budget)?;
        let bend = rational(bend)?;
        let mut reader = Reader { b: &self.bindings, py, vertices: Table::default(), faces: Table::default(), patches: Table::default(), chains: Table::default(), face_objects: Vec::new() };
        let parts = Parts { full, source_vertices, source_faces, neighbours, corner_vertices, chains, domain_chains };
        let (mut input, records) = match reader.read(&parts) {
            Ok(found) => found,
            Err(Fail::Py(error)) => return Err(error),
            Err(Fail::Decline(decline)) => return (2u8, decline.detail).into_bound_py_any(py),
        };
        input.budget = budget;
        input.bend = bend;
        input.float_limit = float_limit;
        let answer = match catch_unwind(AssertUnwindSafe(|| chain_station_plans(&input))) {
            Ok(Ok(answer)) => answer,
            Ok(Err(decline)) => return (2u8, decline.0).into_bound_py_any(py),
            Err(panic) => return Err(PyRuntimeError::new_err(format!("chain_station: native panic: {}", crate::panic_message(&panic)))),
        };
        let faces = &reader.face_objects;
        let mut plans = Vec::with_capacity(answer.plans.len());
        for plan in &answer.plans {
            let mut stations = Vec::with_capacity(plan.stations.len());
            for station in &plan.stations {
                let mut pairs = Vec::with_capacity(station.pairs.len());
                for (first, second) in &station.pairs {
                    let one = &faces[answer.face_ids[*first] as usize];
                    let two = &faces[answer.face_ids[*second] as usize];
                    pairs.push(PyTuple::new(py, [one, two])?);
                }
                stations.push((station.ordinal, station.kind as u8, station.disposition as u8, station.reason as u8, PyTuple::new(py, pairs)?).into_bound_py_any(py)?);
            }
            plans.push((&records[plan.chain], PyTuple::new(py, stations)?).into_bound_py_any(py)?);
        }
        (0u8, PyTuple::new(py, plans)?).into_bound_py_any(py)
    }
}
