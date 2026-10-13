//! `snapshot_check` at the boundary: the Python graph of an `AnalysisSnapshotV1` read into the model of `cftuv-snapshot`, and the verdict back.
//!
//! The op is CLEAN-OR-DECLINE (see the crate note): the extension never refuses a snapshot, it either attests that the Python validators would report nothing or declines by name and the
//! shim runs them. The reading is half of the proof, so it is strict by construction:
//!
//! * every record is read only from an instance of EXACTLY the class the contract names (a subclass, a mock, a record of another class is a decline), every leaf is an exact `str`, `int`,
//!   `float`, `bool`, `Decimal`, an id is an instance of exactly its own id class; containers are exactly `frozenset` or `tuple` as the contract types them;
//! * the slots of the records are read at the offsets the probe of a real instance found (`pyobj::Raw`, the slot mode `CFTUV_NATIVE_SLOTS`: `auto`, `raw`, `attr`), a field the probe does not
//!   find is a refusal when the classes are bound (`NativePortStale` in the shim: the contract changed under the port);
//! * the strings are interned once per call, borrowed from the Python strings (which live as long as the snapshot the caller holds); nothing here runs Python code while the graph is read.
//!
//! Answers: `(0, domains, corners, certified, straddled)` for a clean snapshot, `(2, reason code, detail)` for a decline. A panic is a `RuntimeError`.

use std::collections::HashMap;
use std::panic::{catch_unwind, AssertUnwindSafe};
use std::sync::atomic::{AtomicU64, Ordering};
use std::time::Instant;

use cftuv_canon::fxhash::FxBuild;
use cftuv_core::num::IBig;
use cftuv_core::rat::Rat;
use cftuv_snapshot::model::{
    capability, Affine, Certificate, Chain, Constraint, Corner, Dec, Direction, Domain, Edge, Endpoint, Face, Grid, Id, Interval, Junction, JunctionKind, Kind, Loop, Measure, Metric, Neighbour,
    PlaneCertificate, Role, RoutePair, Sector, Snapshot, SupportRef, Surface, Target, Terminal, Topology, Triangle, Turn, Use, Vertex,
};
use cftuv_snapshot::{decimal, Context, Decline, Reason, Scope};
use pyo3::exceptions::PyRuntimeError;
use pyo3::prelude::*;
use pyo3::types::{PyBool, PyDict, PyFloat, PyFrozenSet, PyInt, PyString, PyTuple};
use pyo3::types::PyFrozenSetMethods;
use pyo3::{ffi, intern, IntoPyObjectExt};

use crate::pyobj::{ibig_from_int, note_attr_reads, process_mode, refuse, Raw, SlotMode};

/// A failure of reading: a Python error (surfaces as an exception), or a decline (the snapshot is not the canonical shape of the contract).
enum Fail {
    Py(PyErr),
    Decline(Decline),
}

impl From<PyErr> for Fail {
    fn from(error: PyErr) -> Fail {
        Fail::Py(error)
    }
}

impl From<Decline> for Fail {
    fn from(decline: Decline) -> Fail {
        Fail::Decline(decline)
    }
}

type Read<T> = Result<T, Fail>;

fn shape<T>(text: impl Into<String>) -> Read<T> {
    Err(Fail::Decline(Decline { reason: Reason::Shape, detail: text.into() }))
}

// --------------------------------------------------------------------------
// classes
// --------------------------------------------------------------------------

/// The slots of a class in the order of its MRO (`__slots__` of every class that has them), the raw access when the probe confirms it, and the position of each requested field.
struct Cls {
    class: Py<PyAny>,
    class_ptr: usize,
    names: Vec<Py<PyString>>,
    raw: Option<Raw>,
    position: Vec<usize>,
}

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

impl Cls {
    /// `wanted`: the fields the reader asks for, in the order it asks (`get(object, k)` is the k-th).
    fn new(py: Python<'_>, mode: SlotMode, label: &str, class: &Bound<'_, PyAny>, wanted: &[&str]) -> PyResult<Cls> {
        let slots = all_slots(py, class)?;
        let names: Vec<Py<PyString>> = slots.iter().map(|name| PyString::new(py, name).unbind()).collect();
        let mut position = Vec::with_capacity(wanted.len());
        for field in wanted {
            match slots.iter().position(|slot| slot == field) {
                Some(index) => position.push(index),
                None => return Err(refuse(format!("{label} has no slot `{field}` (the contract changed under the port)"))),
            }
        }
        let class_owned = class.clone().unbind();
        let name_refs: Vec<&Py<PyString>> = names.iter().collect();
        let raw = Raw::select(py, mode, label, &class_owned, &name_refs)?;
        Ok(Cls { class_ptr: class.as_ptr() as usize, class: class_owned, names, raw, position })
    }

    fn is(&self, object: &Bound<'_, PyAny>) -> bool {
        object.get_type().as_ptr() as usize == self.class_ptr
    }

    fn name(&self) -> String {
        self.class.bind_borrowed_name()
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
            Err(_) => shape(format!("a slot of {} is empty", self.name())),
        }
    }
}

trait ClassName {
    fn bind_borrowed_name(&self) -> String;
}

impl ClassName for Py<PyAny> {
    fn bind_borrowed_name(&self) -> String {
        Python::attach(|py| self.bind(py).getattr(intern!(py, "__name__")).and_then(|name| name.extract::<String>()).unwrap_or_else(|_| "?".to_string()))
    }
}

/// An enum class: its members by identity.
struct Members {
    class_ptr: usize,
    members: Vec<(usize, u8)>,
    name: String,
}

impl Members {
    /// `names`: the member names the reader distinguishes, in order; a member of the enum not in `names` reads as code 255.
    fn new(py: Python<'_>, class: &Bound<'_, PyAny>, names: &[&str]) -> PyResult<Members> {
        let mut members = Vec::new();
        for member in class.try_iter()? {
            let member = member?;
            let name: String = member.getattr(intern!(py, "name"))?.extract()?;
            let code = names.iter().position(|wanted| *wanted == name).map(|index| index as u8).unwrap_or(255);
            members.push((member.as_ptr() as usize, code));
        }
        for wanted in names {
            let present = class.getattr(*wanted);
            if present.is_err() {
                return Err(refuse(format!("{} has no member {wanted} (the contract changed under the port)", class.getattr(intern!(py, "__name__"))?)));
            }
        }
        Ok(Members { class_ptr: class.as_ptr() as usize, members, name: class.getattr(intern!(py, "__name__"))?.extract()? })
    }

    fn code(&self, object: &Bound<'_, PyAny>) -> Read<u8> {
        if object.get_type().as_ptr() as usize != self.class_ptr {
            return shape(format!("a value that is not a member of {}", self.name));
        }
        let pointer = object.as_ptr() as usize;
        match self.members.iter().find(|(member, _)| *member == pointer) {
            Some((_, code)) => Ok(*code),
            None => shape(format!("an unknown member of {}", self.name)),
        }
    }
}

// --------------------------------------------------------------------------
// the bound classes
// --------------------------------------------------------------------------

struct Bindings {
    ids: Vec<(Kind, Cls)>,
    decimal_ptr: usize,
    snapshot: Cls,
    patch: Cls,
    domain: Cls,
    sector_record: Cls,
    vertex: Cls,
    local_point: Cls,
    unavailable_position_ptr: usize,
    chain: Cls,
    chain_use: Cls,
    launch: Cls,
    boundary_loop: Cls,
    constraint: Cls,
    loop_target: Cls,
    use_target: Cls,
    edges_target: Cls,
    unavailable_target_ptr: usize,
    sector: Cls,
    support: Cls,
    planar_direction: Cls,
    vector2: Cls,
    unavailable_direction_ptr: usize,
    certificate: Cls,
    measure: Cls,
    unavailable_measure_ptr: usize,
    interval: Cls,
    corner: Cls,
    junction: Cls,
    route_pair: Cls,
    t_topology: Cls,
    x_topology: Cls,
    y_topology: Cls,
    declared_topology: Cls,
    unavailable_topology_ptr: usize,
    terminal: Cls,
    neighbour: Cls,
    surface: Cls,
    source_edge: Cls,
    source_face: Cls,
    triangle: Cls,
    // metric descriptors
    affine_metric: Cls,
    plane_certificate: Cls,
    rational: Cls,
    point3: Cls,
    vector3: Cls,
    matrix2: Cls,
    coordinate: Cls,
    point2: Cls,
    affine_direction: Cls,
    vector2_exact: Cls,
    grid: Cls,
    other_certificates: Vec<(usize, &'static str)>,
    other_descriptors: Vec<(usize, &'static str)>,
    // enums
    capabilities: Members,
    regime: Members,
    payload_mode: Members,
    turn: Members,
    endpoint: Members,
    junction_kind: Members,
    role: Members,
    chart_orientation: Members,
    frame_law: Members,
    admission_law: Members,
    reconstruction_law: Members,
    snapping_law: Members,
    none_ptr: usize,
}

fn class_of<'py>(classes: &Bound<'py, PyDict>, name: &str) -> PyResult<Bound<'py, PyAny>> {
    match classes.get_item(name)? {
        Some(found) => Ok(found),
        None => Err(refuse(format!("the shim did not hand over `{name}`"))),
    }
}

const CAPABILITY_NAMES: [&str; 12] = [
    "PATCH_DOMAIN_V1",
    "PHYSICAL_CHAIN_DIRECTED_USES_V1",
    "PATCH_SURFACE_IR_V1",
    "ORIENTED_OWNER_SECTOR_V1",
    "REFLEX_ANGLE_CERTIFICATE_V1",
    "RELATION_LINEAGE_V1",
    "AUTHORITATIVE_SURFACE_METRIC_V1",
    "PHYSICAL_EDGE_TABLE_V1",
    "BOUNDARY_CONSTRAINT_TARGET_V1",
    "TYPED_JUNCTION_ROUTE_TOPOLOGY_V1",
    "TERMINAL_RELATION_V1",
    "EC0_COORDINATE_FREE_FIXTURE_V5",
];

impl Bindings {
    fn new(py: Python<'_>, classes: &Bound<'_, PyDict>) -> PyResult<Bindings> {
        let mode = process_mode();
        let cls = |name: &str, wanted: &[&str]| -> PyResult<Cls> { Cls::new(py, mode, name, &class_of(classes, name)?, wanted) };
        let pointer = |name: &str| -> PyResult<usize> { Ok(class_of(classes, name)?.as_ptr() as usize) };
        let mut ids = Vec::new();
        for kind in Kind::ALL {
            ids.push((kind, cls(kind.class_name(), &["value"])?));
        }
        let other_certificates = vec![
            (pointer("NearPlanarProjectionCertificateV1")?, "NearPlanarProjectionCertificateV1"),
            (pointer("DevelopableUnfoldCertificateV1")?, "DevelopableUnfoldCertificateV1"),
            (pointer("DevelopableBandChartCertificateV1")?, "DevelopableBandChartCertificateV1"),
        ];
        let other_descriptors = vec![
            (pointer("PlanarPatchFrameV1")?, "PlanarPatchFrameV1"),
            (pointer("IntrinsicSurfaceMetricDescriptorV1")?, "IntrinsicSurfaceMetricDescriptorV1"),
            (pointer("UnavailableSurfaceMetricDescriptorV1")?, "UnavailableSurfaceMetricDescriptorV1"),
        ];
        let none_ptr = unsafe { ffi::Py_None() } as usize;
        Ok(Bindings {
            ids,
            decimal_ptr: pointer("Decimal")?,
            snapshot: cls(
                "AnalysisSnapshotV1",
                &[
                    "schema_version", "source_revision", "analysis_capabilities", "patches", "patch_domains", "surface_metric_descriptors", "surface_ir", "source_vertices", "physical_chains",
                    "chain_uses", "boundary_loops", "boundary_constraints", "angular_owner_sectors", "reflex_angle_certificates", "corner_relations", "junction_relations", "terminal_relations",
                    "seam_neighbour_faces",
                ],
            )?,
            patch: cls("PatchDescriptorV1", &["patch_id"])?,
            domain: cls("PatchDomainV1", &["patch_domain_id", "owner_patch_id", "surface_regime", "boundary_constraint_ids", "sectors"])?,
            sector_record: cls("PatchSectorV1", &["owner_sector_id", "chain_use_id"])?,
            vertex: cls("SourceVertexV1", &["vertex_id", "position"])?,
            local_point: cls("LocalPoint3V1", &["x", "y", "z"])?,
            unavailable_position_ptr: pointer("UnavailableSourcePositionV1")?,
            chain: cls("PhysicalChainV1", &["physical_chain_id", "is_closed", "ordered_source_vertex_ids", "ordered_physical_edge_ids"])?,
            chain_use: cls("ChainUseV1", &["chain_use_id", "physical_chain_id", "owner_patch_id", "patch_domain_id", "boundary_loop_id", "launch_locus"])?,
            launch: cls("LaunchLocusV1", &["boundary_constraint_id"])?,
            boundary_loop: cls("BoundaryLoopV1", &["boundary_loop_id", "patch_domain_id", "ordered_chain_use_ids"])?,
            constraint: cls("BoundaryConstraintV1", &["boundary_constraint_id", "patch_domain_id", "target", "originating_chain_use_id"])?,
            loop_target: cls("BoundaryLoopConstraintTargetV1", &["boundary_loop_id"])?,
            use_target: cls("ChainUseConstraintTargetV1", &["chain_use_id"])?,
            edges_target: cls("PhysicalEdgeSequenceConstraintTargetV1", &["ordered_physical_edge_ids"])?,
            unavailable_target_ptr: pointer("UnavailableBoundaryConstraintTargetV1")?,
            sector: cls(
                "OrientedOwnerSectorV1",
                &["owner_sector_id", "owner_patch_id", "patch_domain_id", "ordered_incident_chain_use_ids", "incoming_support_ref", "outgoing_support_ref", "turn_orientation"],
            )?,
            support: cls("SourceSupportRefV1", &["chain_use_id", "source_launch_boundary_id", "direction_payload"])?,
            planar_direction: cls("CertifiedPlanarSupportDirectionV1", &["direction_in_domain"])?,
            vector2: cls("LocalVector2V1", &["x", "y"])?,
            unavailable_direction_ptr: pointer("UnavailableSourceSupportDirectionV1")?,
            certificate: cls("ReflexAngleCertificateV1", &["certificate_id", "owner_sector_id", "measure_payload", "exact_two_pi"])?,
            measure: cls("CertifiedReflexAngleMeasureV1", &["phi_over_pi", "reflex_excess_over_pi", "orientation"])?,
            unavailable_measure_ptr: pointer("UnavailableReflexAngleMeasureV1")?,
            interval: cls("CertifiedDecimalIntervalV1", &["lower", "upper", "lower_kind", "upper_kind"])?,
            corner: cls("CornerRelationV1", &["corner_relation_id", "source_vertex_id", "owner_sector_id", "reflex_angle_certificate_id", "exact_two_pi"])?,
            junction: cls("JunctionRelationV1", &["junction_relation_id", "source_vertex_id", "incident_chain_use_ids", "relation_kind", "route_topology"])?,
            route_pair: cls("JunctionRoutePairV1", &["route_pairing_id", "first_chain_use_id", "second_chain_use_id"])?,
            t_topology: cls("TJunctionRouteTopologyV1", &["through_route_pair", "branch_chain_use_ids"])?,
            x_topology: cls("XJunctionRouteTopologyV1", &["route_pairs"])?,
            y_topology: cls("YJunctionRouteTopologyV1", &["trunk_chain_use_id", "ordered_branch_chain_use_ids", "route_pairs"])?,
            declared_topology: cls("DeclaredRoutePairsTopologyV1", &["route_pairs"])?,
            unavailable_topology_ptr: pointer("UnavailableJunctionRouteTopologyV1")?,
            terminal: cls("TerminalRelationV1", &["terminal_relation_id", "source_vertex_id", "chain_use_id", "owner_patch_id", "patch_domain_id", "endpoint_role", "owner_sector_id"])?,
            neighbour: cls("SeamNeighbourFaceV1", &["face_id", "patch_id", "vertex_ids"])?,
            surface: cls("PatchSurfaceIRV1", &["schema_version", "source_revision", "payload_mode", "source_vertex_ids", "source_edges", "source_faces", "surface_triangles"])?,
            source_edge: cls("SourceEdgeV1", &["edge_id", "vertex_a_id", "vertex_b_id"])?,
            source_face: cls("SourceFaceV1", &["face_id", "patch_id", "vertex_cycle", "edge_cycle", "triangle_ids"])?,
            triangle: cls("SurfaceTriangleV1", &["triangle_id", "source_face_id", "vertex_ids", "physical_edge_ids"])?,
            affine_metric: cls(
                "RationalAffinePlanarMetricV2",
                &[
                    "reference_metric_id", "patch_domain_id", "source_revision", "exact_origin", "exact_basis_a", "exact_basis_b", "exact_gram_matrix", "exact_inverse_gram_matrix",
                    "exact_source_vertex_coordinates", "chart_orientation", "frame_selection_law", "planarity_certificate", "grid_certificate",
                ],
            )?,
            plane_certificate: cls(
                "ExactSourcePlaneCertificateV1",
                &["certificate_id", "patch_domain_id", "admission_law", "exact", "exact_plane_normal", "source_vertex_ids", "reconstruction_law"],
            )?,
            rational: cls("ExactRationalV1", &["numerator", "denominator"])?,
            point3: cls("ExactPoint3V1", &["x", "y", "z"])?,
            vector3: cls("ExactVector3V1", &["x", "y", "z"])?,
            matrix2: cls("ExactMatrix2V1", &["m00", "m01", "m10", "m11"])?,
            coordinate: cls("ExactSourceVertexCoordinateV2", &["source_vertex_id", "domain_coordinate"])?,
            point2: cls("ExactPoint2V1", &["x", "y"])?,
            affine_direction: cls("CertifiedAffineSupportDirectionV2", &["direction_in_domain"])?,
            vector2_exact: cls("ExactVector2V1", &["x", "y"])?,
            grid: cls("IntegerGridCertificateV1", &["snapping_law", "window_step", "source_scale"])?,
            other_certificates,
            other_descriptors,
            capabilities: Members::new(py, &class_of(classes, "AnalysisCapability")?, &CAPABILITY_NAMES)?,
            regime: Members::new(py, &class_of(classes, "SurfaceRegime")?, &["PLANAR"])?,
            payload_mode: Members::new(py, &class_of(classes, "SurfacePayloadMode")?, &["FULL_HOST_SURFACE", "EC0_COORDINATE_FREE_FIXTURE_V5"])?,
            turn: Members::new(py, &class_of(classes, "TurnOrientation")?, &["CCW_IN_OWNER_PATCH_ORIENTATION", "CW_IN_OWNER_PATCH_ORIENTATION"])?,
            endpoint: Members::new(py, &class_of(classes, "IntervalEndpointKind")?, &["OPEN", "CLOSED"])?,
            junction_kind: Members::new(py, &class_of(classes, "JunctionRelationKind")?, &["T", "X", "Y", "CROSS_PATCH", "MIXED_ALPHA"])?,
            role: Members::new(py, &class_of(classes, "TerminalEndpointRole")?, &["START", "END", "NON_CHAIN_ENDPOINT"])?,
            chart_orientation: Members::new(py, &class_of(classes, "AffineChartOrientationV1")?, &["COORDINATE_CCW_MATCHES_OWNER_PATCH", "COORDINATE_CW_MATCHES_OWNER_PATCH"])?,
            frame_law: Members::new(py, &class_of(classes, "AffineFrameSelectionLawV1")?, &["CANONICAL_SOURCE_VERTEX_BASIS_V1"])?,
            admission_law: Members::new(py, &class_of(classes, "PlanarityAdmissionLawV1")?, &["EXACT_SOURCE_PLANE_V1"])?,
            reconstruction_law: Members::new(py, &class_of(classes, "AffineReconstructionLawV1")?, &["O_PLUS_U_A_PLUS_V_B_V1"])?,
            snapping_law: Members::new(py, &class_of(classes, "GridSnappingLawV1")?, &["UNSNAPPED_EXACT_V1", "SOURCE_ONLY_GRID_SNAP_V1", "INTEGER_GRID_SNAP_V1"])?,
            none_ptr,
        })
    }
}

// --------------------------------------------------------------------------
// the reader
// --------------------------------------------------------------------------

struct Reader<'b, 'a, 'py> {
    b: &'b Bindings,
    py: Python<'py>,
    map: HashMap<&'a [u8], u32, FxBuild>,
    strings: Vec<&'a [u8]>,
}

impl<'b, 'a, 'py> Reader<'b, 'a, 'py> {
    fn intern(&mut self, object: &Bound<'py, PyAny>, what: &str) -> Read<u32> {
        if !object.is_exact_instance_of::<PyString>() {
            return shape(format!("{what} is not a str"));
        }
        let mut size: ffi::Py_ssize_t = 0;
        // SAFETY: `object` is an exact `str`; the buffer lives as long as the string, which the snapshot (held by the caller for the whole call) keeps alive.
        let data = unsafe { ffi::PyUnicode_AsUTF8AndSize(object.as_ptr(), &mut size) };
        if data.is_null() {
            return Err(Fail::Py(PyErr::fetch(self.py)));
        }
        // SAFETY: the slice is borrowed from a live immutable string for the lifetime `'a` of the call.
        let bytes: &'a [u8] = unsafe { std::slice::from_raw_parts(data.cast::<u8>(), size as usize) };
        if let Some(found) = self.map.get(bytes) {
            return Ok(*found);
        }
        let next = self.strings.len() as u32;
        self.strings.push(bytes);
        self.map.insert(bytes, next);
        Ok(next)
    }

    fn id(&mut self, object: &Bound<'py, PyAny>, kind: Kind) -> Read<Id> {
        let (_, class) = &self.b.ids[kind as usize];
        if !class.is(object) {
            return shape(format!("an id that is not exactly a {}", kind.class_name()));
        }
        let value = class.get(object, 0)?;
        let s = self.intern(&value, "an id value")?;
        Ok(Id::new(kind, s))
    }

    fn optional_id(&mut self, object: &Bound<'py, PyAny>, kind: Kind) -> Read<Option<Id>> {
        if object.as_ptr() as usize == self.b.none_ptr {
            return Ok(None);
        }
        Ok(Some(self.id(object, kind)?))
    }

    fn boolean(&self, object: &Bound<'py, PyAny>, what: &str) -> Read<bool> {
        if !object.is_exact_instance_of::<PyBool>() {
            return shape(format!("{what} is not a bool"));
        }
        // SAFETY: `object` is a bool; its address is Py_True or Py_False.
        Ok(object.as_ptr() == unsafe { ffi::Py_True() })
    }

    fn float(&self, object: &Bound<'py, PyAny>, what: &str) -> Read<f64> {
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

    fn integer(&self, object: &Bound<'py, PyAny>, what: &str) -> Read<IBig> {
        if !object.is_exact_instance_of::<PyInt>() {
            return shape(format!("{what} is not an int"));
        }
        Ok(ibig_from_int(object)?)
    }

    fn rational(&self, object: &Bound<'py, PyAny>) -> Read<Rat> {
        let b = self.b;
        let class = &b.rational;
        if !class.is(object) {
            return shape("a value that is not exactly an ExactRationalV1");
        }
        let numerator = self.integer(&class.get(object, 0)?, "a numerator")?;
        let denominator = self.integer(&class.get(object, 1)?, "a denominator")?;
        match Rat::new(numerator, denominator) {
            Ok(value) => Ok(value),
            Err(_) => shape("an ExactRationalV1 with a zero denominator"),
        }
    }

    fn decimal(&self, object: &Bound<'py, PyAny>) -> Read<Dec> {
        if object.get_type().as_ptr() as usize != self.b.decimal_ptr {
            return shape("a bound that is not exactly a Decimal");
        }
        let text = object.str()?;
        match decimal::parse(text.to_str()?) {
            Some(value) => Ok(value),
            None => shape("a Decimal that is not finite"),
        }
    }

    fn frozenset(&self, object: &Bound<'py, PyAny>, what: &str) -> Read<Vec<Bound<'py, PyAny>>> {
        if !object.is_exact_instance_of::<PyFrozenSet>() {
            return shape(format!("{what} is not a frozenset"));
        }
        let set = object.cast::<PyFrozenSet>().map_err(|_| Fail::Decline(Decline { reason: Reason::Shape, detail: format!("{what} is not a frozenset") }))?;
        Ok(set.iter().collect())
    }

    fn tuple(&self, object: &Bound<'py, PyAny>, what: &str) -> Read<Vec<Bound<'py, PyAny>>> {
        if !object.is_exact_instance_of::<PyTuple>() {
            return shape(format!("{what} is not a tuple"));
        }
        let tuple = object.cast::<PyTuple>().map_err(|_| Fail::Decline(Decline { reason: Reason::Shape, detail: format!("{what} is not a tuple") }))?;
        Ok(tuple.iter().collect())
    }

    fn id_set(&mut self, object: &Bound<'py, PyAny>, kind: Kind, what: &str) -> Read<Vec<Id>> {
        let items = self.frozenset(object, what)?;
        items.iter().map(|item| self.id(item, kind)).collect()
    }

    fn id_tuple(&mut self, object: &Bound<'py, PyAny>, kind: Kind, what: &str) -> Read<Vec<Id>> {
        let items = self.tuple(object, what)?;
        items.iter().map(|item| self.id(item, kind)).collect()
    }

    fn vec3(&self, object: &Bound<'py, PyAny>, class: &Cls) -> Read<[Rat; 3]> {
        if !class.is(object) {
            return shape("a 3-vector that is not the exact class");
        }
        Ok([self.rational(&class.get(object, 0)?)?, self.rational(&class.get(object, 1)?)?, self.rational(&class.get(object, 2)?)?])
    }

    fn interval(&self, object: &Bound<'py, PyAny>) -> Read<Interval> {
        let b = self.b;
        let class = &b.interval;
        if !class.is(object) {
            return shape("an interval that is not exactly a CertifiedDecimalIntervalV1");
        }
        let kind = |value: u8| if value == 0 { Endpoint::Open } else { Endpoint::Closed };
        let lower_kind = self.b.endpoint.code(&class.get(object, 2)?)?;
        let upper_kind = self.b.endpoint.code(&class.get(object, 3)?)?;
        if lower_kind > 1 || upper_kind > 1 {
            return shape("an unknown interval endpoint kind");
        }
        Ok(Interval { lower: self.decimal(&class.get(object, 0)?)?, upper: self.decimal(&class.get(object, 1)?)?, lower_kind: kind(lower_kind), upper_kind: kind(upper_kind) })
    }

    fn turn(&self, object: &Bound<'py, PyAny>) -> Read<Turn> {
        match self.b.turn.code(object)? {
            0 => Ok(Turn::Ccw),
            1 => Ok(Turn::Cw),
            _ => shape("an unknown turn orientation"),
        }
    }

    // ---- records

    fn read_snapshot(&mut self, object: &Bound<'py, PyAny>) -> Read<Snapshot<'a>> {
        let b = self.b;
        let class = &b.snapshot;
        if !class.is(object) {
            return shape("the snapshot is not exactly an AnalysisSnapshotV1");
        }
        let schema = {
            let value = class.get(object, 0)?;
            self.intern(&value, "the schema version")?
        };
        let revision = self.id(&class.get(object, 1)?, Kind::SourceRevision)?;
        let mut capabilities = 0u32;
        for member in self.frozenset(&class.get(object, 2)?, "analysis_capabilities")? {
            let code = self.b.capabilities.code(&member)?;
            if (code as usize) < CAPABILITY_NAMES.len() {
                capabilities |= capability::NAMES[code as usize].1;
            }
        }
        let patches = {
            let mut out = Vec::new();
            for item in self.frozenset(&class.get(object, 3)?, "patches")? {
                if !self.b.patch.is(&item) {
                    return shape("a patch that is not exactly a PatchDescriptorV1");
                }
                out.push(self.id(&self.b.patch.get(&item, 0)?, Kind::Patch)?);
            }
            out
        };
        let domains = self.frozenset(&class.get(object, 4)?, "patch_domains")?.iter().map(|item| self.read_domain(item)).collect::<Read<Vec<_>>>()?;
        let metrics = self.frozenset(&class.get(object, 5)?, "surface_metric_descriptors")?.iter().map(|item| self.read_metric(item)).collect::<Read<Vec<_>>>()?;
        let surface = self.read_surface(&class.get(object, 6)?)?;
        let vertices = self.frozenset(&class.get(object, 7)?, "source_vertices")?.iter().map(|item| self.read_vertex(item)).collect::<Read<Vec<_>>>()?;
        let chains = self.frozenset(&class.get(object, 8)?, "physical_chains")?.iter().map(|item| self.read_chain(item)).collect::<Read<Vec<_>>>()?;
        let uses = self.frozenset(&class.get(object, 9)?, "chain_uses")?.iter().map(|item| self.read_use(item)).collect::<Read<Vec<_>>>()?;
        let loops = self.frozenset(&class.get(object, 10)?, "boundary_loops")?.iter().map(|item| self.read_loop(item)).collect::<Read<Vec<_>>>()?;
        let constraints = self.frozenset(&class.get(object, 11)?, "boundary_constraints")?.iter().map(|item| self.read_constraint(item)).collect::<Read<Vec<_>>>()?;
        let sectors = self.frozenset(&class.get(object, 12)?, "angular_owner_sectors")?.iter().map(|item| self.read_sector(item)).collect::<Read<Vec<_>>>()?;
        let certificates = self.frozenset(&class.get(object, 13)?, "reflex_angle_certificates")?.iter().map(|item| self.read_certificate(item)).collect::<Read<Vec<_>>>()?;
        let corners = self.frozenset(&class.get(object, 14)?, "corner_relations")?.iter().map(|item| self.read_corner(item)).collect::<Read<Vec<_>>>()?;
        let junctions = self.frozenset(&class.get(object, 15)?, "junction_relations")?.iter().map(|item| self.read_junction(item)).collect::<Read<Vec<_>>>()?;
        let terminals = self.frozenset(&class.get(object, 16)?, "terminal_relations")?.iter().map(|item| self.read_terminal(item)).collect::<Read<Vec<_>>>()?;
        let neighbours = self.frozenset(&class.get(object, 17)?, "seam_neighbour_faces")?.iter().map(|item| self.read_neighbour(item)).collect::<Read<Vec<_>>>()?;
        Ok(Snapshot {
            strings: Vec::new(),
            schema,
            revision,
            capabilities,
            patches,
            domains,
            metrics,
            surface,
            vertices,
            chains,
            uses,
            loops,
            constraints,
            sectors,
            certificates,
            corners,
            junctions,
            terminals,
            neighbours,
        })
    }

    fn read_domain(&mut self, object: &Bound<'py, PyAny>) -> Read<Domain> {
        let b = self.b;
        let class = &b.domain;
        if !class.is(object) {
            return shape("a domain that is not exactly a PatchDomainV1");
        }
        let planar = self.b.regime.code(&class.get(object, 2)?)? == 0;
        let mut sectors = Vec::new();
        for item in self.frozenset(&class.get(object, 4)?, "sectors")? {
            if !self.b.sector_record.is(&item) {
                return shape("a domain sector that is not exactly a PatchSectorV1");
            }
            sectors.push((self.id(&self.b.sector_record.get(&item, 0)?, Kind::OwnerSector)?, self.id(&self.b.sector_record.get(&item, 1)?, Kind::ChainUse)?));
        }
        Ok(Domain {
            id: self.id(&class.get(object, 0)?, Kind::PatchDomain)?,
            owner_patch: self.id(&class.get(object, 1)?, Kind::Patch)?,
            planar,
            boundary_constraints: self.id_set(&class.get(object, 3)?, Kind::BoundaryConstraint, "boundary_constraint_ids")?,
            sectors,
        })
    }

    fn read_metric(&mut self, object: &Bound<'py, PyAny>) -> Read<Metric> {
        let b = self.b;
        let pointer = object.get_type().as_ptr() as usize;
        if self.b.affine_metric.is(object) {
            let class = &b.affine_metric;
            let domain = self.id(&class.get(object, 1)?, Kind::PatchDomain)?;
            let certificate = class.get(object, 11)?;
            let frame_law = self.b.frame_law.code(&class.get(object, 10)?)?;
            if !self.b.plane_certificate.is(&certificate) {
                let certificate_ptr = certificate.get_type().as_ptr() as usize;
                let name = self.b.other_certificates.iter().find(|(known, _)| *known == certificate_ptr).map(|(_, name)| *name);
                return match name {
                    Some(name) => Ok(Metric::Other { domain, class: name }),
                    None => shape("a planarity certificate of an unknown class"),
                };
            }
            if frame_law != 0 {
                return Ok(Metric::Other { domain, class: "RationalAffinePlanarMetricV2 with another frame law" });
            }
            return Ok(Metric::Affine(Box::new(self.read_affine(object)?)));
        }
        for (known, name) in &self.b.other_descriptors {
            if *known == pointer {
                let domain = self.id(&object.getattr(intern!(self.py, "patch_domain_id"))?, Kind::PatchDomain)?;
                return Ok(Metric::Other { domain, class: name });
            }
        }
        shape("a metric descriptor of an unknown class")
    }

    fn read_affine(&mut self, object: &Bound<'py, PyAny>) -> Read<Affine> {
        let b = self.b;
        let class = &b.affine_metric;
        let revision = self.id(&class.get(object, 2)?, Kind::SourceRevision)?;
        let matrix = |reader: &Reader<'_, '_, 'py>, value: Bound<'py, PyAny>| -> Read<[[Rat; 2]; 2]> {
            let matrix = &reader.b.matrix2;
            if !matrix.is(&value) {
                return shape("a matrix that is not exactly an ExactMatrix2V1");
            }
            Ok([
                [reader.rational(&matrix.get(&value, 0)?)?, reader.rational(&matrix.get(&value, 1)?)?],
                [reader.rational(&matrix.get(&value, 2)?)?, reader.rational(&matrix.get(&value, 3)?)?],
            ])
        };
        let origin = self.vec3(&class.get(object, 3)?, &self.b.point3)?;
        let basis_a = self.vec3(&class.get(object, 4)?, &self.b.vector3)?;
        let basis_b = self.vec3(&class.get(object, 5)?, &self.b.vector3)?;
        let gram = matrix(self, class.get(object, 6)?)?;
        let inverse_gram = matrix(self, class.get(object, 7)?)?;
        let mut coordinates = Vec::new();
        for item in self.frozenset(&class.get(object, 8)?, "exact_source_vertex_coordinates")? {
            let record = &self.b.coordinate;
            if !record.is(&item) {
                return shape("a coordinate that is not exactly an ExactSourceVertexCoordinateV2");
            }
            let id = self.id(&record.get(&item, 0)?, Kind::SourceVertex)?;
            let point = record.get(&item, 1)?;
            let point2 = &self.b.point2;
            if !point2.is(&point) {
                return shape("a domain coordinate that is not exactly an ExactPoint2V1");
            }
            coordinates.push((id, [self.rational(&point2.get(&point, 0)?)?, self.rational(&point2.get(&point, 1)?)?]));
        }
        let ccw = match self.b.chart_orientation.code(&class.get(object, 9)?)? {
            0 => true,
            1 => false,
            _ => return shape("an unknown chart orientation"),
        };
        let certificate = self.read_plane_certificate(&class.get(object, 11)?)?;
        let grid = self.read_grid(&class.get(object, 12)?)?;
        Ok(Affine {
            reference_metric: self.id(&class.get(object, 0)?, Kind::ReferenceMetric)?,
            domain: self.id(&class.get(object, 1)?, Kind::PatchDomain)?,
            revision,
            origin,
            basis_a,
            basis_b,
            gram,
            inverse_gram,
            coordinates,
            ccw,
            certificate,
            grid,
        })
    }

    fn read_plane_certificate(&mut self, object: &Bound<'py, PyAny>) -> Read<PlaneCertificate> {
        let b = self.b;
        let class = &b.plane_certificate;
        Ok(PlaneCertificate {
            id: self.id(&class.get(object, 0)?, Kind::PlanarityCertificate)?,
            domain: self.id(&class.get(object, 1)?, Kind::PatchDomain)?,
            exact_law: self.b.admission_law.code(&class.get(object, 2)?)? == 0,
            exact: self.boolean(&class.get(object, 3)?, "exact")?,
            normal: self.vec3(&class.get(object, 4)?, &self.b.vector3)?,
            vertex_ids: self.id_set(&class.get(object, 5)?, Kind::SourceVertex, "source_vertex_ids")?,
            reconstruction_ok: self.b.reconstruction_law.code(&class.get(object, 6)?)? == 0,
        })
    }

    fn read_grid(&mut self, object: &Bound<'py, PyAny>) -> Read<Grid> {
        let b = self.b;
        let class = &b.grid;
        if !class.is(object) {
            return shape("a grid certificate that is not exactly an IntegerGridCertificateV1");
        }
        let law = self.b.snapping_law.code(&class.get(object, 0)?)?;
        if law > 2 {
            return shape("an unknown grid snapping law");
        }
        let step = class.get(object, 1)?;
        let window_step = if step.as_ptr() as usize == self.b.none_ptr { None } else { Some(self.rational(&step)?) };
        let scale = class.get(object, 2)?;
        let source_scale = if scale.as_ptr() as usize == self.b.none_ptr { None } else { Some(self.integer(&scale, "source_scale")?) };
        Ok(Grid { snaps_source: law != 0, snaps_constructions: law == 2, window_step, source_scale })
    }

    fn read_surface(&mut self, object: &Bound<'py, PyAny>) -> Read<Surface> {
        let b = self.b;
        let class = &b.surface;
        if !class.is(object) {
            return shape("the surface is not exactly a PatchSurfaceIRV1");
        }
        let schema = {
            let value = class.get(object, 0)?;
            self.intern(&value, "the surface schema")?
        };
        let revision = self.id(&class.get(object, 1)?, Kind::SourceRevision)?;
        let full = match self.b.payload_mode.code(&class.get(object, 2)?)? {
            0 => true,
            1 => false,
            _ => return shape("an unknown surface payload mode"),
        };
        let vertex_ids = self.id_set(&class.get(object, 3)?, Kind::SourceVertex, "source_vertex_ids")?;
        let mut edges = Vec::new();
        for item in self.frozenset(&class.get(object, 4)?, "source_edges")? {
            let record = &self.b.source_edge;
            if !record.is(&item) {
                return shape("an edge that is not exactly a SourceEdgeV1");
            }
            edges.push(Edge {
                id: self.id(&record.get(&item, 0)?, Kind::PhysicalEdge)?,
                a: self.id(&record.get(&item, 1)?, Kind::SourceVertex)?,
                b: self.id(&record.get(&item, 2)?, Kind::SourceVertex)?,
            });
        }
        let mut faces = Vec::new();
        for item in self.frozenset(&class.get(object, 5)?, "source_faces")? {
            let record = &self.b.source_face;
            if !record.is(&item) {
                return shape("a face that is not exactly a SourceFaceV1");
            }
            faces.push(Face {
                id: self.id(&record.get(&item, 0)?, Kind::SourceFace)?,
                patch: self.id(&record.get(&item, 1)?, Kind::Patch)?,
                vertex_cycle: self.id_tuple(&record.get(&item, 2)?, Kind::SourceVertex, "vertex_cycle")?,
                edge_cycle: self.id_tuple(&record.get(&item, 3)?, Kind::PhysicalEdge, "edge_cycle")?,
                triangles: self.id_tuple(&record.get(&item, 4)?, Kind::SurfaceTriangle, "triangle_ids")?,
            });
        }
        let mut triangles = Vec::new();
        for item in self.frozenset(&class.get(object, 6)?, "surface_triangles")? {
            let record = &self.b.triangle;
            if !record.is(&item) {
                return shape("a triangle that is not exactly a SurfaceTriangleV1");
            }
            let mut edge_ids = Vec::new();
            for edge in self.tuple(&record.get(&item, 3)?, "physical_edge_ids")? {
                edge_ids.push(self.optional_id(&edge, Kind::PhysicalEdge)?);
            }
            triangles.push(Triangle {
                id: self.id(&record.get(&item, 0)?, Kind::SurfaceTriangle)?,
                face: self.id(&record.get(&item, 1)?, Kind::SourceFace)?,
                vertices: self.id_tuple(&record.get(&item, 2)?, Kind::SourceVertex, "vertex_ids")?,
                edges: edge_ids,
            });
        }
        Ok(Surface { schema, revision, full, vertex_ids, edges, faces, triangles })
    }

    fn read_vertex(&mut self, object: &Bound<'py, PyAny>) -> Read<Vertex> {
        let b = self.b;
        let class = &b.vertex;
        if !class.is(object) {
            return shape("a vertex that is not exactly a SourceVertexV1");
        }
        let position = class.get(object, 1)?;
        let point = &self.b.local_point;
        let position = if point.is(&position) {
            Some([self.float(&point.get(&position, 0)?, "x")?, self.float(&point.get(&position, 1)?, "y")?, self.float(&point.get(&position, 2)?, "z")?])
        } else if position.get_type().as_ptr() as usize == self.b.unavailable_position_ptr {
            None
        } else {
            return shape("a source position of an unknown class");
        };
        Ok(Vertex { id: self.id(&class.get(object, 0)?, Kind::SourceVertex)?, position })
    }

    fn read_chain(&mut self, object: &Bound<'py, PyAny>) -> Read<Chain> {
        let b = self.b;
        let class = &b.chain;
        if !class.is(object) {
            return shape("a chain that is not exactly a PhysicalChainV1");
        }
        Ok(Chain {
            id: self.id(&class.get(object, 0)?, Kind::PhysicalChain)?,
            closed: self.boolean(&class.get(object, 1)?, "is_closed")?,
            vertices: self.id_tuple(&class.get(object, 2)?, Kind::SourceVertex, "ordered_source_vertex_ids")?,
            edges: self.id_tuple(&class.get(object, 3)?, Kind::PhysicalEdge, "ordered_physical_edge_ids")?,
        })
    }

    fn read_use(&mut self, object: &Bound<'py, PyAny>) -> Read<Use> {
        let b = self.b;
        let class = &b.chain_use;
        if !class.is(object) {
            return shape("a ChainUse that is not exactly a ChainUseV1");
        }
        let launch = class.get(object, 5)?;
        if !self.b.launch.is(&launch) {
            return shape("a launch locus that is not exactly a LaunchLocusV1");
        }
        Ok(Use {
            id: self.id(&class.get(object, 0)?, Kind::ChainUse)?,
            chain: self.id(&class.get(object, 1)?, Kind::PhysicalChain)?,
            owner_patch: self.id(&class.get(object, 2)?, Kind::Patch)?,
            domain: self.id(&class.get(object, 3)?, Kind::PatchDomain)?,
            boundary_loop: self.optional_id(&class.get(object, 4)?, Kind::BoundaryLoop)?,
            launch_constraint: self.id(&self.b.launch.get(&launch, 0)?, Kind::BoundaryConstraint)?,
        })
    }

    fn read_loop(&mut self, object: &Bound<'py, PyAny>) -> Read<Loop> {
        let b = self.b;
        let class = &b.boundary_loop;
        if !class.is(object) {
            return shape("a loop that is not exactly a BoundaryLoopV1");
        }
        Ok(Loop {
            id: self.id(&class.get(object, 0)?, Kind::BoundaryLoop)?,
            domain: self.id(&class.get(object, 1)?, Kind::PatchDomain)?,
            uses: self.id_tuple(&class.get(object, 2)?, Kind::ChainUse, "ordered_chain_use_ids")?,
        })
    }

    fn read_constraint(&mut self, object: &Bound<'py, PyAny>) -> Read<Constraint> {
        let b = self.b;
        let class = &b.constraint;
        if !class.is(object) {
            return shape("a constraint that is not exactly a BoundaryConstraintV1");
        }
        let target = class.get(object, 2)?;
        let target = if self.b.loop_target.is(&target) {
            Target::Loop(self.id(&self.b.loop_target.get(&target, 0)?, Kind::BoundaryLoop)?)
        } else if self.b.use_target.is(&target) {
            Target::Use(self.id(&self.b.use_target.get(&target, 0)?, Kind::ChainUse)?)
        } else if self.b.edges_target.is(&target) {
            Target::Edges(self.id_tuple(&self.b.edges_target.get(&target, 0)?, Kind::PhysicalEdge, "ordered_physical_edge_ids")?)
        } else if target.get_type().as_ptr() as usize == self.b.unavailable_target_ptr {
            Target::Unavailable
        } else {
            return shape("a boundary target of an unknown class");
        };
        Ok(Constraint {
            id: self.id(&class.get(object, 0)?, Kind::BoundaryConstraint)?,
            domain: self.id(&class.get(object, 1)?, Kind::PatchDomain)?,
            target,
            originating_use: self.optional_id(&class.get(object, 3)?, Kind::ChainUse)?,
        })
    }

    fn read_support(&mut self, object: &Bound<'py, PyAny>) -> Read<SupportRef> {
        let b = self.b;
        let class = &b.support;
        if !class.is(object) {
            return shape("a support that is not exactly a SourceSupportRefV1");
        }
        let payload = class.get(object, 2)?;
        let direction = if self.b.affine_direction.is(&payload) {
            let vector = self.b.affine_direction.get(&payload, 0)?;
            if !self.b.vector2_exact.is(&vector) {
                return shape("a direction that is not exactly an ExactVector2V1");
            }
            Direction::Affine(self.rational(&self.b.vector2_exact.get(&vector, 0)?)?, self.rational(&self.b.vector2_exact.get(&vector, 1)?)?)
        } else if self.b.planar_direction.is(&payload) {
            let vector = self.b.planar_direction.get(&payload, 0)?;
            if !self.b.vector2.is(&vector) {
                return shape("a direction that is not exactly a LocalVector2V1");
            }
            Direction::Planar(self.float(&self.b.vector2.get(&vector, 0)?, "x")?, self.float(&self.b.vector2.get(&vector, 1)?, "y")?)
        } else if payload.get_type().as_ptr() as usize == self.b.unavailable_direction_ptr {
            Direction::Unavailable
        } else {
            return shape("a support direction of an unknown class");
        };
        Ok(SupportRef { chain_use: self.id(&class.get(object, 0)?, Kind::ChainUse)?, launch_boundary: self.id(&class.get(object, 1)?, Kind::BoundaryConstraint)?, direction })
    }

    fn read_sector(&mut self, object: &Bound<'py, PyAny>) -> Read<Sector> {
        let b = self.b;
        let class = &b.sector;
        if !class.is(object) {
            return shape("a sector that is not exactly an OrientedOwnerSectorV1");
        }
        Ok(Sector {
            id: self.id(&class.get(object, 0)?, Kind::OwnerSector)?,
            owner_patch: self.id(&class.get(object, 1)?, Kind::Patch)?,
            domain: self.id(&class.get(object, 2)?, Kind::PatchDomain)?,
            incident_uses: self.id_tuple(&class.get(object, 3)?, Kind::ChainUse, "ordered_incident_chain_use_ids")?,
            incoming: self.read_support(&class.get(object, 4)?)?,
            outgoing: self.read_support(&class.get(object, 5)?)?,
            turn: self.turn(&class.get(object, 6)?)?,
        })
    }

    fn read_certificate(&mut self, object: &Bound<'py, PyAny>) -> Read<Certificate> {
        let b = self.b;
        let class = &b.certificate;
        if !class.is(object) {
            return shape("a certificate that is not exactly a ReflexAngleCertificateV1");
        }
        let payload = class.get(object, 2)?;
        let measure = if self.b.measure.is(&payload) {
            let measure = &self.b.measure;
            Measure::Certified {
                phi: self.interval(&measure.get(&payload, 0)?)?,
                delta: self.interval(&measure.get(&payload, 1)?)?,
                orientation: self.turn(&measure.get(&payload, 2)?)?,
            }
        } else if payload.get_type().as_ptr() as usize == self.b.unavailable_measure_ptr {
            Measure::Unavailable
        } else {
            return shape("an angle measure of an unknown class");
        };
        Ok(Certificate {
            id: self.id(&class.get(object, 0)?, Kind::AngleCertificate)?,
            sector: self.id(&class.get(object, 1)?, Kind::OwnerSector)?,
            measure,
            exact_two_pi: self.boolean(&class.get(object, 3)?, "exact_two_pi")?,
        })
    }

    fn read_corner(&mut self, object: &Bound<'py, PyAny>) -> Read<Corner> {
        let b = self.b;
        let class = &b.corner;
        if !class.is(object) {
            return shape("a corner that is not exactly a CornerRelationV1");
        }
        Ok(Corner {
            id: self.id(&class.get(object, 0)?, Kind::CornerRelation)?,
            vertex: self.id(&class.get(object, 1)?, Kind::SourceVertex)?,
            sector: self.id(&class.get(object, 2)?, Kind::OwnerSector)?,
            certificate: self.id(&class.get(object, 3)?, Kind::AngleCertificate)?,
            exact_two_pi: self.boolean(&class.get(object, 4)?, "exact_two_pi")?,
        })
    }

    fn read_pair(&mut self, object: &Bound<'py, PyAny>) -> Read<RoutePair> {
        let b = self.b;
        let class = &b.route_pair;
        if !class.is(object) {
            return shape("a route pair that is not exactly a JunctionRoutePairV1");
        }
        Ok(RoutePair {
            id: self.id(&class.get(object, 0)?, Kind::RoutePairing)?,
            first: self.id(&class.get(object, 1)?, Kind::ChainUse)?,
            second: self.id(&class.get(object, 2)?, Kind::ChainUse)?,
        })
    }

    fn read_pairs(&mut self, object: &Bound<'py, PyAny>) -> Read<Vec<RoutePair>> {
        self.frozenset(object, "route_pairs")?.iter().map(|item| self.read_pair(item)).collect()
    }

    fn read_junction(&mut self, object: &Bound<'py, PyAny>) -> Read<Junction> {
        let b = self.b;
        let class = &b.junction;
        if !class.is(object) {
            return shape("a junction that is not exactly a JunctionRelationV1");
        }
        let kind = match self.b.junction_kind.code(&class.get(object, 3)?)? {
            0 => JunctionKind::T,
            1 => JunctionKind::X,
            2 => JunctionKind::Y,
            3 => JunctionKind::CrossPatch,
            4 => JunctionKind::MixedAlpha,
            _ => return shape("an unknown junction kind"),
        };
        let topology = class.get(object, 4)?;
        let topology = if self.b.t_topology.is(&topology) {
            let record = &self.b.t_topology;
            Topology::T { through: self.read_pair(&record.get(&topology, 0)?)?, branches: self.id_set(&record.get(&topology, 1)?, Kind::ChainUse, "branch_chain_use_ids")? }
        } else if self.b.x_topology.is(&topology) {
            Topology::X { pairs: self.read_pairs(&self.b.x_topology.get(&topology, 0)?)? }
        } else if self.b.y_topology.is(&topology) {
            let record = &self.b.y_topology;
            Topology::Y {
                trunk: self.id(&record.get(&topology, 0)?, Kind::ChainUse)?,
                branches: self.id_tuple(&record.get(&topology, 1)?, Kind::ChainUse, "ordered_branch_chain_use_ids")?,
                pairs: self.read_pairs(&record.get(&topology, 2)?)?,
            }
        } else if self.b.declared_topology.is(&topology) {
            Topology::Declared { pairs: self.read_pairs(&self.b.declared_topology.get(&topology, 0)?)? }
        } else if topology.get_type().as_ptr() as usize == self.b.unavailable_topology_ptr {
            Topology::Unavailable
        } else {
            return shape("a route topology of an unknown class");
        };
        Ok(Junction {
            id: self.id(&class.get(object, 0)?, Kind::JunctionRelation)?,
            vertex: self.id(&class.get(object, 1)?, Kind::SourceVertex)?,
            incident_uses: self.id_tuple(&class.get(object, 2)?, Kind::ChainUse, "incident_chain_use_ids")?,
            kind,
            topology,
        })
    }

    fn read_terminal(&mut self, object: &Bound<'py, PyAny>) -> Read<Terminal> {
        let b = self.b;
        let class = &b.terminal;
        if !class.is(object) {
            return shape("a terminal that is not exactly a TerminalRelationV1");
        }
        let role = match self.b.role.code(&class.get(object, 5)?)? {
            0 => Role::Start,
            1 => Role::End,
            2 => Role::NonChain,
            _ => return shape("an unknown terminal endpoint role"),
        };
        Ok(Terminal {
            id: self.id(&class.get(object, 0)?, Kind::TerminalRelation)?,
            vertex: self.id(&class.get(object, 1)?, Kind::SourceVertex)?,
            chain_use: self.id(&class.get(object, 2)?, Kind::ChainUse)?,
            owner_patch: self.id(&class.get(object, 3)?, Kind::Patch)?,
            domain: self.id(&class.get(object, 4)?, Kind::PatchDomain)?,
            role,
            sector: self.optional_id(&class.get(object, 6)?, Kind::OwnerSector)?,
        })
    }

    fn read_neighbour(&mut self, object: &Bound<'py, PyAny>) -> Read<Neighbour> {
        let b = self.b;
        let class = &b.neighbour;
        if !class.is(object) {
            return shape("a neighbour face that is not exactly a SeamNeighbourFaceV1");
        }
        Ok(Neighbour {
            face: self.id(&class.get(object, 0)?, Kind::SourceFace)?,
            patch: self.id(&class.get(object, 1)?, Kind::Patch)?,
            vertices: self.id_tuple(&class.get(object, 2)?, Kind::SourceVertex, "vertex_ids")?,
        })
    }
}

// --------------------------------------------------------------------------
// the entry point
// --------------------------------------------------------------------------

/// The classes of the kernel the reader is built on, bound once; `check` is the whole operation.
#[pyclass(module = "cftuv_native._core", frozen)]
pub struct SnapshotCheck {
    bindings: Bindings,
    /// Nanoseconds of the last call: reading the graph, and the check on the model (a diagnostic of the bench, not an answer).
    last_read: AtomicU64,
    last_check: AtomicU64,
}

#[pymethods]
impl SnapshotCheck {
    /// `SnapshotCheck(classes)`: `classes` maps a class name (`AnalysisSnapshotV1`, the ids, the enums, `Decimal`...) to the live class of the kernel; a name missing or a field the classes lack is a
    /// named refusal (`TypeError("cftuv_native: ...")`, which the shim turns into `NativePortStale`).
    #[new]
    fn new(py: Python<'_>, classes: &Bound<'_, PyDict>) -> PyResult<SnapshotCheck> {
        Ok(SnapshotCheck { bindings: Bindings::new(py, classes)?, last_read: AtomicU64::new(0), last_check: AtomicU64::new(0) })
    }

    /// `check(snapshot, scope, native_exact, decimal_precision, analysis_schema)`: `(0, domains, corners, certified, straddled)` or `(2, reason code, detail)`.
    fn check<'py>(&self, py: Python<'py>, snapshot: &Bound<'py, PyAny>, scope: u8, native_exact: bool, decimal_precision: u32, analysis_schema: &str) -> PyResult<Bound<'py, PyAny>> {
        let scope = if scope == 0 { Scope::Analysis } else { Scope::AnalysisAndReference };
        let context = Context { scope, native_exact, decimal_precision, analysis_schema: analysis_schema.as_bytes().to_vec() };
        let mut reader: Reader<'_, '_, 'py> = Reader { b: &self.bindings, py, map: HashMap::default(), strings: Vec::new() };
        let started = Instant::now();
        let read = reader.read_snapshot(snapshot);
        self.last_read.store(u64::try_from(started.elapsed().as_nanos()).unwrap_or(u64::MAX), Ordering::Relaxed);
        let model = match read {
            Ok(mut model) => {
                model.strings = std::mem::take(&mut reader.strings);
                model
            }
            Err(Fail::Py(error)) => return Err(error),
            Err(Fail::Decline(decline)) => return (2u8, decline.reason.code(), decline.detail).into_bound_py_any(py),
        };
        let started = Instant::now();
        let verdict = catch_unwind(AssertUnwindSafe(|| cftuv_snapshot::check(&model, &context)));
        self.last_check.store(u64::try_from(started.elapsed().as_nanos()).unwrap_or(u64::MAX), Ordering::Relaxed);
        match verdict {
            Ok(Ok(attestation)) => (0u8, attestation.domains, attestation.corners, attestation.certified, attestation.straddled).into_bound_py_any(py),
            Ok(Err(decline)) => (2u8, decline.reason.code(), decline.detail).into_bound_py_any(py),
            Err(panic) => Err(PyRuntimeError::new_err(format!("snapshot_check: native panic: {}", crate::panic_message(&panic)))),
        }
    }

    /// `(read ns, check ns)` of the last call.
    fn last_timings(&self) -> (u64, u64) {
        (self.last_read.load(Ordering::Relaxed), self.last_check.load(Ordering::Relaxed))
    }

    /// Test-only: which classes read their slots by offset: `[(class name, raw)]`.
    fn raw_layouts(&self) -> Vec<(String, bool)> {
        let b = &self.bindings;
        let mut rows: Vec<(String, bool)> = b.ids.iter().map(|(_, cls)| (cls.name(), cls.raw.is_some())).collect();
        for cls in [&b.snapshot, &b.domain, &b.vertex, &b.chain, &b.chain_use, &b.sector, &b.certificate, &b.affine_metric, &b.rational, &b.surface, &b.source_face, &b.triangle] {
            rows.push((cls.name(), cls.raw.is_some()));
        }
        rows
    }
}
