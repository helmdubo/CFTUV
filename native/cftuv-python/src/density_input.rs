//! Только чтение закрытых raw carriers; геометрия и вычисление принадлежат Rust batch.
use crate::pyobj;
use cftuv_density::density_raw_types_v1::*;
use cftuv_density::{IBig, Rational, UBig};
use pyo3::prelude::*;
use pyo3::types::{PyBool, PyInt, PyModule, PyString, PyTuple};
use pyo3::PyTypeInfo;

pub(crate) struct Decoder<'py> {
    pub fraction: Py<PyAny>,
    facts: Bound<'py, PyModule>,
}
impl<'py> Decoder<'py> {
    pub fn new(py: Python<'py>, fraction: &Bound<'py, PyAny>) -> PyResult<Self> {
        if !fraction.is(&py.import("fractions")?.getattr("Fraction")?) {
            return Err(pyobj::refuse(
                "density_compute: the exact Fraction class is required",
            ));
        }
        Ok(Self {
            fraction: fraction.clone().unbind(),
            facts: py.import("cftuv_envelope.reference.density_oracle_v1")?,
        })
    }
    pub fn class(&self, value: &Bound<'py, PyAny>, name: &str) -> PyResult<()> {
        if !value.get_type().is(&self.facts.getattr(name)?) {
            return Err(pyobj::refuse(format!(
                "density_compute: exact {name} required"
            )));
        }
        Ok(())
    }
    pub fn rational(&self, value: &Bound<'py, PyAny>) -> PyResult<Rational> {
        if !value.get_type().is(self.fraction.bind(value.py())) {
            return Err(pyobj::refuse(
                "density_compute: exact Fraction carrier required",
            ));
        }
        let numerator = integer(&value.getattr("_numerator")?)?;
        let denominator = unsigned(&value.getattr("_denominator")?)?;
        if denominator == UBig::ZERO {
            return Err(pyobj::refuse(
                "density_compute: Fraction denominator must be positive",
            ));
        }
        Rational::new(numerator, denominator)
            .map_err(|why| pyobj::refuse(format!("density_compute: rational carrier: {why:?}")))
    }
    pub fn point(&self, value: &Bound<'py, PyAny>) -> PyResult<RawDensityPointV1> {
        self.class(value, "RawDensityPointV1")?;
        Ok(RawDensityPointV1 {
            x: self.rational(&value.getattr("x")?)?,
            y: self.rational(&value.getattr("y")?)?,
        })
    }
    fn metric(&self, value: &Bound<'py, PyAny>) -> PyResult<RawDensityMetricV1> {
        self.class(value, "RawDensityMetricV1")?;
        let triple = |name| -> PyResult<[Rational; 3]> {
            let values = tuple(&value.getattr(name)?)?
                .iter()
                .map(|v| self.rational(&v))
                .collect::<PyResult<Vec<_>>>()?;
            values
                .try_into()
                .map_err(|_| pyobj::refuse("density_compute: metric needs three rational entries"))
        };
        Ok(RawDensityMetricV1 {
            gram: triple("gram")?,
            inverse_gram: triple("inverse_gram")?,
            owner_orientation_sign: integer(&value.getattr("owner_orientation_sign")?)?
                .try_into()
                .map_err(|_| pyobj::refuse("density_compute: orientation must fit i8"))?,
            metric_id: optional_text(&value.getattr("metric_id")?)?,
        })
    }
    fn incident(&self, value: &Bound<'py, PyAny>) -> PyResult<RawDensityIncidentV1> {
        self.class(value, "RawDensityIncidentV1")?;
        let owner = value.getattr("owner_left")?;
        let orientation =
            optional_text(&value.getattr("chain_use_orientation")?)?.map(|s| match s.as_str() {
                "A_START_TO_END" => ChainUseOrientationV1::AStartToEnd,
                "B_START_TO_END" => ChainUseOrientationV1::BStartToEnd,
                _ => ChainUseOrientationV1::Other(s),
            });
        let cycles = tuple(&value.getattr("owner_face_cycles")?)?
            .iter()
            .map(|cycle| {
                tuple(&cycle)?
                    .iter()
                    .map(|point| self.point(&point))
                    .collect::<PyResult<Vec<_>>>()
            })
            .collect::<PyResult<Vec<_>>>()?;
        let id_cycles = tuple(&value.getattr("owner_face_vertex_id_cycles")?)?
            .iter()
            .map(|cycle| texts(&cycle))
            .collect::<PyResult<Vec<_>>>()?;
        Ok(RawDensityIncidentV1 {
            start: self.point(&value.getattr("start")?)?,
            end: self.point(&value.getattr("end")?)?,
            owner_left: if owner.is_none() {
                None
            } else {
                Some(boolean(&owner)?)
            },
            anchor_vertex_id: optional_text(&value.getattr("anchor_vertex_id")?)?,
            source_vertex_start_id: optional_text(&value.getattr("source_vertex_start_id")?)?,
            source_vertex_end_id: optional_text(&value.getattr("source_vertex_end_id")?)?,
            support_id: optional_text(&value.getattr("support_id")?)?,
            chain_use_orientation: orientation,
            source_direction_declarations: directions(&rows(
                &value.getattr("source_direction_declaration")?,
            )?)?,
            owner_face_cycles: cycles,
            owner_face_vertex_id_cycles: id_cycles,
            owner_face_ids: texts(&value.getattr("owner_face_ids")?)?,
            owner_face_edge_ordinals: tuple(&value.getattr("owner_face_edge_ordinals")?)?
                .iter()
                .map(|v| size(&v))
                .collect::<PyResult<Vec<_>>>()?,
            physical_edge_id: optional_text(&value.getattr("physical_edge_id")?)?,
            chain_use_id: optional_text(&value.getattr("chain_use_id")?)?,
        })
    }
    fn geometry(&self, value: &Bound<'py, PyAny>) -> PyResult<RawDensityGeometryV1> {
        self.class(value, "RawDensityGeometryV1")?;
        Ok(RawDensityGeometryV1 {
            metric: self.metric(&value.getattr("metric")?)?,
            incoming: self.incident(&value.getattr("incoming")?)?,
            outgoing: self.incident(&value.getattr("outgoing")?)?,
        })
    }
    fn selection(&self, value: &Bound<'py, PyAny>) -> PyResult<RawDensitySelectionV1> {
        self.class(value, "RawDensitySelectionV1")?;
        Ok(RawDensitySelectionV1 {
            q: size(&value.getattr("q")?)?,
            source_hidden_count: size(&value.getattr("source_hidden_count")?)?,
            selection_policy_id: text(&value.getattr("selection_policy_id")?)?,
            max_subturn_value_id: text(&value.getattr("max_subturn_value_id")?)?,
            bucket_c: size(&value.getattr("bucket_c")?)?,
            selection_certificate_id: optional_text(&value.getattr("selection_certificate_id")?)?,
        })
    }
    fn canonical(&self, value: &Bound<'py, PyAny>) -> PyResult<RawDensityCanonicalFactsV1> {
        self.class(value, "RawDensityCanonicalFactsV1")?;
        let bounds = tuple(&value.getattr("source_reflex_interval")?)?
            .iter()
            .map(|v| self.rational(&v))
            .collect::<PyResult<Vec<_>>>()?;
        let canonical = value.getattr("canonical_excess_over_pi")?;
        Ok(RawDensityCanonicalFactsV1 {
            source_reflex_interval: bounds.try_into().map_err(|_| {
                pyobj::refuse("density_compute: reflex interval requires two bounds")
            })?,
            canonical_excess_over_pi: if canonical.is_none() {
                None
            } else {
                Some(self.rational(&canonical)?)
            },
            restoration_declaration: rows(&value.getattr("restoration_declaration")?)?,
            noise_policy_declaration: rows(&value.getattr("noise_policy_declaration")?)?,
            source_reflex_declaration: rows(&value.getattr("source_reflex_declaration")?)?,
        })
    }
    fn spec(&self, value: &Bound<'py, PyAny>) -> PyResult<RawDensitySpecInputV1> {
        self.class(value, "RawDensitySpecInput")?;
        Ok(RawDensitySpecInputV1 {
            source_geometry: self.geometry(&value.getattr("source_geometry")?)?,
            evaluation_geometry: self.geometry(&value.getattr("evaluation_geometry")?)?,
            turn_orientation_sign: integer(&value.getattr("turn_orientation_sign")?)?
                .try_into()
                .map_err(|_| pyobj::refuse("density_compute: turn orientation must fit i8"))?,
            selection: self.selection(&value.getattr("selection")?)?,
            canonical: self.canonical(&value.getattr("canonical")?)?,
            spec_id: optional_text(&value.getattr("spec_id")?)?,
            source_relation_id: optional_text(&value.getattr("source_relation_id")?)?,
            owner_sector_id: optional_text(&value.getattr("owner_sector_id")?)?,
        })
    }
    pub fn batch(&self, value: &Bound<'py, PyAny>) -> PyResult<RawDensityBatchInputV1> {
        self.class(value, "RawDensityBatchInputV1")?;
        let canon = value.getattr("canon_ids")?;
        self.class(&canon, "DensityOracleCanonV1")?;
        let limits = value.getattr("limits")?;
        self.class(&limits, "DensityOracleLimitsV1")?;
        let expand = limits.getattr("finite_expand")?;
        let expand_class = value
            .py()
            .import("cftuv_envelope.reference.density_construct_v1")?
            .getattr("ExpandLimitsV1")?;
        if !expand.get_type().is(&expand_class) {
            return Err(pyobj::refuse(
                "density_compute: exact ExpandLimitsV1 required",
            ));
        }
        let window = match text(&value.getattr("window_law")?)?.as_str() {
            "NARROW_ROTATION_BAND" => cftuv_density::geometry::WindowLaw::NarrowBand,
            "VORONOI_OF_NEIGHBOUR_MIDPOINTS" => cftuv_density::geometry::WindowLaw::Voronoi,
            _ => return Err(pyobj::refuse("density_compute: unknown window law")),
        };
        Ok(RawDensityBatchInputV1 {
            canon_ids: DensityOracleCanonV1 {
                construction: text(&canon.getattr("construction")?)?,
                shell: text(&canon.getattr("shell")?)?,
                angle: text(&canon.getattr("angle")?)?,
            },
            limits: DensityOracleLimitsV1 {
                preparation_cap: unsigned(&limits.getattr("preparation_cap")?)?,
                search_cap: unsigned(&limits.getattr("search_cap")?)?,
                box_cap: size(&limits.getattr("box_cap")?)?,
                height_cap: unsigned(&limits.getattr("height_cap")?)?,
                finite_expand: cftuv_density::density_expand_v1::ExpandLimits {
                    max_terms: size(&expand.getattr("max_terms")?)?,
                    max_rational_power_bits: size(&expand.getattr("max_rational_power_bits")?)?,
                    max_polynomial_exponent: size(&expand.getattr("max_polynomial_exponent")?)?
                        .try_into()
                        .map_err(|_| {
                            pyobj::refuse("density_compute: polynomial cap must fit u32")
                        })?,
                },
            },
            window_law: window,
            specs: tuple(&value.getattr("specs")?)?
                .iter()
                .map(|v| self.spec(&v))
                .collect::<PyResult<Vec<_>>>()?,
        })
    }
}

fn exact(
    value: &Bound<'_, PyAny>,
    cls: Bound<'_, pyo3::types::PyType>,
    name: &str,
) -> PyResult<()> {
    if !value.get_type().is(&cls) {
        return Err(pyobj::refuse(format!(
            "density_compute: exact {name} required"
        )));
    }
    Ok(())
}
pub(crate) fn tuple<'py>(value: &Bound<'py, PyAny>) -> PyResult<Bound<'py, PyTuple>> {
    exact(value, PyTuple::type_object(value.py()), "tuple")?;
    Ok(value.cast::<PyTuple>()?.clone())
}
pub(crate) fn integer(value: &Bound<'_, PyAny>) -> PyResult<IBig> {
    exact(value, PyInt::type_object(value.py()), "int")?;
    pyobj::ibig_from_int(value)
}
pub(crate) fn unsigned(value: &Bound<'_, PyAny>) -> PyResult<UBig> {
    exact(value, PyInt::type_object(value.py()), "int")?;
    pyobj::ubig_from_int(value)
}
pub(crate) fn size(value: &Bound<'_, PyAny>) -> PyResult<usize> {
    unsigned(value)?
        .try_into()
        .map_err(|_| pyobj::refuse("density_compute: host cardinality does not fit usize"))
}
pub(crate) fn text(value: &Bound<'_, PyAny>) -> PyResult<String> {
    exact(value, PyString::type_object(value.py()), "str")?;
    Ok(value.cast::<PyString>()?.to_str()?.into())
}
pub(crate) fn texts(value: &Bound<'_, PyAny>) -> PyResult<Vec<String>> {
    tuple(value)?.iter().map(|v| text(&v)).collect()
}
pub(crate) fn optional_text(value: &Bound<'_, PyAny>) -> PyResult<Option<String>> {
    if value.is_none() {
        Ok(None)
    } else {
        Ok(Some(text(value)?))
    }
}
pub(crate) fn boolean(value: &Bound<'_, PyAny>) -> PyResult<bool> {
    exact(value, PyBool::type_object(value.py()), "bool")?;
    value.extract()
}
fn primitive(value: &Bound<'_, PyAny>) -> PyResult<PrimitiveFactV1> {
    Ok(if value.is_none() {
        PrimitiveFactV1::None
    } else if value.is_instance_of::<PyBool>() {
        PrimitiveFactV1::Bool(boolean(value)?)
    } else if value.is_instance_of::<PyInt>() {
        PrimitiveFactV1::Int(integer(value)?)
    } else if value.is_instance_of::<PyString>() {
        PrimitiveFactV1::Text(text(value)?)
    } else {
        PrimitiveFactV1::Tuple(
            tuple(value)?
                .iter()
                .map(|v| primitive(&v))
                .collect::<PyResult<Vec<_>>>()?,
        )
    })
}
fn rows(value: &Bound<'_, PyAny>) -> PyResult<DeclarationRowsV1> {
    primitive(value)?
        .rows()
        .ok_or_else(|| pyobj::refuse("density_compute: declaration key/value rows required"))
}
fn directions(values: &DeclarationRowsV1) -> PyResult<Vec<RawDirectionDeclarationV1>> {
    if values.is_empty() {
        return Ok(Vec::new());
    }
    let Some(PrimitiveFactV1::Tuple(records)) = declaration_value(values, "records") else {
        return Err(pyobj::refuse(
            "density_compute: source direction records declaration is absent",
        ));
    };
    records
        .iter()
        .map(|record| {
            let row = record.rows().ok_or_else(|| {
                pyobj::refuse("density_compute: direction declaration rows required")
            })?;
            let Some(PrimitiveFactV1::Text(kind)) = declaration_value(&row, "kind") else {
                return Err(pyobj::refuse("density_compute: direction kind is absent"));
            };
            let coordinate = |name| {
                declaration_value(&row, name)
                    .and_then(PrimitiveFactV1::ratio)
                    .ok_or_else(|| {
                        pyobj::refuse(
                            "density_compute: declared direction rational component required",
                        )
                    })
            };
            Ok(match kind.as_str() {
                "CERTIFIED_PLANAR_V1" => RawDirectionDeclarationV1::CertifiedPlanar {
                    x: coordinate("x")?,
                    y: coordinate("y")?,
                },
                "CERTIFIED_AFFINE_V2" => RawDirectionDeclarationV1::CertifiedAffine {
                    x: coordinate("x")?,
                    y: coordinate("y")?,
                    reference_metric_id: match declaration_value(&row, "reference_metric_id") {
                        Some(PrimitiveFactV1::Text(value)) => Some(value.clone()),
                        Some(PrimitiveFactV1::None) | None => None,
                        _ => {
                            return Err(pyobj::refuse(
                                "density_compute: affine metric identity must be optional text",
                            ))
                        }
                    },
                },
                _ => RawDirectionDeclarationV1::Other { kind: kind.clone() },
            })
        })
        .collect()
}
