//! Закрытые facts из Rust; quantum/optional IDs сохраняются, арифметика здесь отсутствует.
use crate::pyobj;
use cftuv_density::density_raw_result_v1::*;
use cftuv_density::density_result_v1::*;
use cftuv_density::{IBig, Rational, UBig};
use pyo3::prelude::*;
use pyo3::types::{PyDict, PyModule, PyString, PyTuple};
use pyo3::IntoPyObjectExt;

macro_rules! record {
    ($e:expr, $name:literal, {$($key:literal : $value:expr),* $(,)?}) => {
        $e.record($name, vec![$(($key, $value?)),*])
    };
}
pub(crate) struct Encoder<'py> {
    pub(crate) py: Python<'py>,
    facts: Bound<'py, PyModule>,
    fraction: Py<PyAny>,
}
impl<'py> Encoder<'py> {
    pub fn new(py: Python<'py>, fraction: &Py<PyAny>) -> PyResult<Self> {
        Ok(Self {
            py,
            facts: py.import("cftuv_envelope.reference.density_oracle_v1")?,
            fraction: fraction.clone_ref(py),
        })
    }
    fn record(
        &self,
        name: &str,
        fields: Vec<(&str, Bound<'py, PyAny>)>,
    ) -> PyResult<Bound<'py, PyAny>> {
        let class = self.facts.getattr(name)?;
        let declared = class.getattr("__dataclass_fields__")?;
        let declared = declared.cast::<PyDict>()?;
        if declared.len() != fields.len()
            || fields
                .iter()
                .any(|(name, _)| !declared.contains(name).unwrap_or(false))
        {
            return Err(pyobj::refuse(format!(
                "density_compute: result class fields changed: {name}"
            )));
        }
        let value = pyobj::alloc(self.py, &class.unbind())?;
        for (name, field) in fields {
            pyobj::set_slot(&value, &PyString::intern(self.py, name).unbind(), &field)?;
        }
        Ok(value)
    }
    pub(crate) fn q(&self, value: &Rational) -> PyResult<Bound<'py, PyAny>> {
        let (n, d) = value.parts();
        let fraction = pyobj::alloc(self.py, &self.fraction)?;
        pyobj::set_slot(
            &fraction,
            &PyString::intern(self.py, "_numerator").unbind(),
            &self.z(n)?,
        )?;
        pyobj::set_slot(
            &fraction,
            &PyString::intern(self.py, "_denominator").unbind(),
            &self.u(d)?,
        )?;
        Ok(fraction)
    }
    pub(crate) fn z(&self, value: &IBig) -> PyResult<Bound<'py, PyAny>> {
        pyobj::int_of_ibig(self.py, value)
    }
    pub(crate) fn u(&self, value: &UBig) -> PyResult<Bound<'py, PyAny>> {
        pyobj::int_of_ubig(self.py, value)
    }
    pub(crate) fn n(&self, value: usize) -> PyResult<Bound<'py, PyAny>> {
        value.into_bound_py_any(self.py)
    }
    pub(crate) fn i(&self, value: i64) -> PyResult<Bound<'py, PyAny>> {
        value.into_bound_py_any(self.py)
    }
    pub(crate) fn flag(&self, value: bool) -> PyResult<Bound<'py, PyAny>> {
        value.into_bound_py_any(self.py)
    }
    pub(crate) fn text(&self, value: &str) -> PyResult<Bound<'py, PyAny>> {
        value.into_bound_py_any(self.py)
    }
    pub(crate) fn none(&self) -> PyResult<Bound<'py, PyAny>> {
        Ok(self.py.None().into_bound(self.py))
    }
    pub(crate) fn optional_text(&self, value: &Option<String>) -> PyResult<Bound<'py, PyAny>> {
        match value {
            Some(value) => self.text(value),
            None => self.none(),
        }
    }
    pub(crate) fn seq(&self, values: Vec<Bound<'py, PyAny>>) -> PyResult<Bound<'py, PyAny>> {
        Ok(PyTuple::new(self.py, values)?.into_any())
    }
    pub(crate) fn strings<'a>(
        &self,
        values: impl IntoIterator<Item = &'a str>,
    ) -> PyResult<Bound<'py, PyAny>> {
        self.seq(
            values
                .into_iter()
                .map(|value| self.text(value))
                .collect::<PyResult<Vec<_>>>()?,
        )
    }
    fn vector(&self, value: &cftuv_density::farey::PrimitiveVector) -> PyResult<Bound<'py, PyAny>> {
        self.seq(vec![self.z(&value.0)?, self.z(&value.1)?])
    }
    fn vectors(
        &self,
        values: &[cftuv_density::farey::PrimitiveVector],
    ) -> PyResult<Bound<'py, PyAny>> {
        self.seq(
            values
                .iter()
                .map(|value| self.vector(value))
                .collect::<PyResult<Vec<_>>>()?,
        )
    }
    fn decimal(&self, value: &DecimalValueFactsV1) -> PyResult<Bound<'py, PyAny>> {
        record!(self, "DecimalValueFactsV1", { "coefficient":self.z(&value.coefficient), "exponent":self.i(value.exponent) })
    }
    fn interval(&self, value: &DecimalIntervalFactsV1) -> PyResult<Bound<'py, PyAny>> {
        use cftuv_density::atlas_plan::EndpointKind;
        let kind = |value| match value {
            EndpointKind::Open => Ok("OPEN"),
            EndpointKind::Closed => Ok("CLOSED"),
            EndpointKind::Other => {
                Err(pyobj::refuse("density_compute: internal endpoint kind gap"))
            }
        };
        record!(self, "DecimalIntervalFactsV1", { "lower":self.decimal(&value.lower), "upper":self.decimal(&value.upper),
            "lower_kind":self.text(kind(value.lower_kind)?), "upper_kind":self.text(kind(value.upper_kind)?),
            "absolute_error_bound":self.decimal(&value.absolute_error_bound) })
    }
    pub(crate) fn certificate(
        &self,
        value: &BindingCertificateFactsV1,
    ) -> PyResult<Bound<'py, PyAny>> {
        record!(self, "BindingCertificateFactsV1", { "bound_primitive_integer_vector":self.vector(&value.bound_primitive_integer_vector),
            "ideal_window_lower_slope_envelope":self.interval(&value.ideal_window_lower_slope_envelope),
            "ideal_window_upper_slope_envelope":self.interval(&value.ideal_window_upper_slope_envelope),
            "certified_window_width_lower_bound":self.decimal(&value.certified_window_width_lower_bound),
            "proven_predicates":self.strings(value.proven_predicates.iter().copied()), "binding_reason":self.optional_text(&value.binding_reason) })
    }
    fn piece(&self, value: &ChartPieceFactsV1) -> PyResult<Bound<'py, PyAny>> {
        use cftuv_density::atlas_plan::PoleOwnership;
        let pole = match value.pole_ownership {
            PoleOwnership::None => "NONE",
            PoleOwnership::XZero => "X_ZERO",
            PoleOwnership::YZero => "Y_ZERO",
        };
        record!(self, "ChartPieceFactsV1", { "piece_index":self.n(value.piece_index), "use_x_denominator":self.flag(value.use_x_denominator),
            "denominator_sign":self.i(value.denominator_sign as i64), "lower_slope_envelope":self.interval(&value.lower_slope_envelope),
            "upper_slope_envelope":self.interval(&value.upper_slope_envelope), "lower_endpoint_included":self.flag(value.lower_endpoint_included),
            "upper_endpoint_included":self.flag(value.upper_endpoint_included), "slope_increases_in_ordinal_order":self.flag(value.slope_increases_in_ordinal_order),
            "pole_ownership":self.text(pole) })
    }
    fn window(&self, value: &OrdinalWindowFactsV1) -> PyResult<Bound<'py, PyAny>> {
        let (bounds, mut fields, name) = match value {
            OrdinalWindowFactsV1::Local {
                bounds,
                use_x_denominator,
                denominator_sign,
                full_lower_slope_envelope,
                full_upper_slope_envelope,
            } => (
                bounds,
                vec![
                    ("use_x_denominator", self.flag(*use_x_denominator)?),
                    ("denominator_sign", self.i(*denominator_sign as i64)?),
                    (
                        "full_lower_slope_envelope",
                        self.interval(full_lower_slope_envelope)?,
                    ),
                    (
                        "full_upper_slope_envelope",
                        self.interval(full_upper_slope_envelope)?,
                    ),
                ],
                "LocalWindowFactsV1",
            ),
            OrdinalWindowFactsV1::Atlas {
                bounds,
                pieces,
                termination_piece_index,
            } => (
                bounds,
                vec![
                    (
                        "pieces",
                        self.seq(
                            pieces
                                .iter()
                                .map(|v| self.piece(v))
                                .collect::<PyResult<Vec<_>>>()?,
                        )?,
                    ),
                    ("termination_piece_index", self.n(*termination_piece_index)?),
                ],
                "AtlasWindowFactsV1",
            ),
        };
        fields.extend([
            ("ordinal", self.n(bounds.ordinal)?),
            (
                "termination_lower_slope",
                self.q(&bounds.termination_lower_slope)?,
            ),
            (
                "termination_upper_slope",
                self.q(&bounds.termination_upper_slope)?,
            ),
            (
                "certified_termination_width",
                self.q(&bounds.certified_termination_width)?,
            ),
            ("admissible_lower_outward", self.q(&bounds.admissible[0])?),
            ("admissible_lower_inward", self.q(&bounds.admissible[1])?),
            ("admissible_upper_inward", self.q(&bounds.admissible[2])?),
            ("admissible_upper_outward", self.q(&bounds.admissible[3])?),
        ]);
        self.record(name, fields)
    }
    fn witness(&self, value: &FareyHeightWitnessFactsV1) -> PyResult<Bound<'py, PyAny>> {
        record!(self, "FareyHeightWitnessFactsV1", { "first_height":self.u(&value.first_height), "last_height":self.u(&value.last_height),
            "primitive_candidate_counts":self.seq(value.primitive_candidate_counts.iter().map(|v| self.n(*v)).collect::<PyResult<Vec<_>>>()?) })
    }
    pub(crate) fn authority(&self, value: &FanAuthorityFactsV1) -> PyResult<Bound<'py, PyAny>> {
        match value {
            FanAuthorityFactsV1::Adaptive(value) => record!(self, "AdaptiveFanAuthorityFactsV1", {
                "max_subturn_q":self.n(value.max_subturn_q), "minimal_common_height":self.u(&value.minimal_common_height),
                "exhaustive_previous_height":self.u(&value.exhaustive_previous_height), "termination_height_upper_bound":self.u(&value.termination_height_upper_bound),
                "bound_primitive_integer_vectors":self.vectors(&value.bound_primitive_integer_vectors),
                "binding_reasons":self.seq(value.binding_reasons.iter().map(|v| self.optional_text(v)).collect::<PyResult<Vec<_>>>()?),
                "ordinal_windows":self.seq(value.ordinal_windows.iter().map(|v| self.window(v)).collect::<PyResult<Vec<_>>>()?),
                "previous_height_witness":self.witness(&value.previous_height_witness), "proven_predicates":self.strings(value.proven_predicates.iter().copied()),
                "authority_id":self.optional_text(&value.authority_id) }),
            FanAuthorityFactsV1::Canonical(value) => {
                record!(self, "CanonicalFanAuthorityFactsV1", { "ray_law":self.text(&value.ray_law),
                "canonical_relation":self.text(&value.canonical_relation), "canonical_reflex_excess_over_pi":self.q(&value.canonical_reflex_excess_over_pi),
                "hidden_edge_count":self.n(value.hidden_edge_count), "max_subturn_q":self.n(value.max_subturn_q),
                "ray_rotation_pairs":self.seq(value.ray_rotation_pairs.iter().map(|v| self.seq(vec![self.z(&v[0])?, self.z(&v[1])?])).collect::<PyResult<Vec<_>>>()?),
                "bound_primitive_integer_vectors":self.vectors(&value.bound_primitive_integer_vectors),
                "proven_predicates":self.strings(value.proven_predicates.iter().map(String::as_str)), "authority_id":self.optional_text(&value.authority_id) })
            }
        }
    }
    fn lift(&self, value: &CountLiftFactsV1) -> PyResult<Bound<'py, PyAny>> {
        record!(self, "CountLiftFactsV1", { "lift_law":self.text(&value.lift_law), "source_hidden_edge_count":self.n(value.source_hidden_edge_count),
            "effective_hidden_edge_count":self.n(value.effective_hidden_edge_count), "max_subturn_q":self.n(value.max_subturn_q),
            "evaluation_turn_sign":self.text(&value.evaluation_turn_sign), "evaluation_turn_cosine_squared":self.q(&value.evaluation_turn_cosine_squared),
            "minimality_predecessor_hidden_edge_count":self.n(value.minimality_predecessor_hidden_edge_count),
            "proven_predicates":self.strings(value.proven_predicates.iter().map(String::as_str)) })
    }
    fn noise(&self, value: &NoiseRecordFactsV1) -> PyResult<Bound<'py, PyAny>> {
        record!(self, "NoiseRecordFactsV1", { "noise_law":self.text(&value.noise_law), "effect":self.text(&value.effect),
            "canonical_relation":self.text(&value.canonical_relation), "canonical_reflex_excess_over_pi":self.q(&value.canonical_reflex_excess_over_pi),
            "source_hidden_edge_count":self.n(value.source_hidden_edge_count), "effective_hidden_edge_count":self.n(value.effective_hidden_edge_count),
            "max_subturn_q":self.n(value.max_subturn_q), "evaluation_turn_sign":self.text(&value.evaluation_turn_sign),
            "evaluation_turn_cosine_squared":self.q(&value.evaluation_turn_cosine_squared),
            "binding_lateral_offset_gram_squared_bound":self.q(&value.binding_lateral_offset_gram_squared_bound),
            "edge_direction_sine_squared_bound":self.q(&value.edge_direction_sine_squared_bound), "direction_sine_bound":self.q(&value.direction_sine_bound),
            "proven_predicates":self.strings(value.proven_predicates.iter().map(String::as_str)) })
    }
    fn subturn(&self, value: &CanonicalSubturnAuthorityFactsV1) -> PyResult<Bound<'py, PyAny>> {
        record!(self, "CanonicalSubturnAuthorityFactsV1", { "guarantee_law":self.text(&value.guarantee_law),
            "canonical_relation":self.text(&value.canonical_relation), "canonical_reflex_excess_over_pi":self.q(&value.canonical_reflex_excess_over_pi),
            "hidden_edge_count":self.n(value.hidden_edge_count), "max_subturn_q":self.n(value.max_subturn_q),
            "canonical_subturn_over_pi":self.q(&value.canonical_subturn_over_pi), "raw_residual_upper_bound_radians":self.q(&value.raw_residual_upper_bound_radians),
            "proven_predicates":self.strings(value.proven_predicates.iter().map(String::as_str)) })
    }
    pub(crate) fn shell(&self, value: &DensityShellEventFactsV1) -> PyResult<Bound<'py, PyAny>> {
        record!(self, "DensityShellEventFactsV1", { "value_wire":self.seq(value.value_wire.iter().map(|v| self.z(v)).collect::<PyResult<Vec<_>>>()?),
            "codes":self.strings(value.codes.iter().map(String::as_str)),
            "rational_numerator":match &value.rational_numerator { Some(v) => self.z(v), None => self.none() },
            "rational_denominator":match &value.rational_denominator { Some(v) => self.u(v), None => self.none() },
            "rational_grid_units":match &value.rational_grid_units { Some(v) => self.z(v), None => self.none() },
            "legacy_projected_min_width_units":match &value.legacy_projected_min_width_units { Some(v) => self.u(v), None => self.none() } })
    }
    fn failure(&self, value: &DensityFailureFactsV1) -> PyResult<Bound<'py, PyAny>> {
        record!(self, "DensityFailureFactsV1", { "exception_type":self.text(&value.exception_type), "outcome":self.text(&value.outcome), "message":self.text(&value.message) })
    }
    fn article(&self, value: &DensityPriceArticleV1) -> PyResult<Bound<'py, PyAny>> {
        record!(self, "DensityPriceArticleV1", { "phase":self.text(&value.phase), "cap":self.u(&value.cap),
            "counters":self.seq(value.counters.iter().map(|(k, v)| self.seq(vec![self.text(k)?, self.u(v)?])).collect::<PyResult<Vec<_>>>()?) })
    }
    fn usage(&self, value: &CanonUsageFactsV1) -> PyResult<Bound<'py, PyAny>> {
        record!(self, "CanonUsageFactsV1", { "canon":self.text(&value.canon), "applied_event":self.text(&value.applied_event) })
    }
    fn primitive(
        &self,
        value: &cftuv_density::density_raw_types_v1::PrimitiveFactV1,
    ) -> PyResult<Bound<'py, PyAny>> {
        use cftuv_density::density_raw_types_v1::PrimitiveFactV1;
        match value {
            PrimitiveFactV1::Bool(v) => self.flag(*v),
            PrimitiveFactV1::Int(v) => self.z(v),
            PrimitiveFactV1::Text(v) => self.text(v),
            PrimitiveFactV1::None => self.none(),
            PrimitiveFactV1::Tuple(values) => self.seq(
                values
                    .iter()
                    .map(|v| self.primitive(v))
                    .collect::<PyResult<Vec<_>>>()?,
            ),
        }
    }
    pub fn result(&self, value: &RawDensitySpecResultV1) -> PyResult<Bound<'py, PyAny>> {
        record!(self, "RawDensitySpecResultV1", { "outcome":self.text(&value.outcome),
            "effective_hidden_count":match value.effective_hidden_count { Some(v) => self.n(v), None => self.none() },
            "binding_reasons":self.seq(value.binding_reasons.iter().map(|v| self.optional_text(v)).collect::<PyResult<Vec<_>>>()?),
            "binding_certificates":self.seq(value.binding_certificates.iter().map(|v| match v { Some(v) => self.certificate(v), None => self.none() }).collect::<PyResult<Vec<_>>>()?),
            "authority":match &value.authority { Some(v) => self.authority(v), None => self.none() },
            "count_lift":match &value.count_lift { Some(v) => self.lift(v), None => self.none() },
            "canon_usage":self.seq(value.canon_usage.iter().map(|v| self.usage(v)).collect::<PyResult<Vec<_>>>()?),
            "ordered_work_articles":self.seq(value.ordered_work_articles.iter().map(|v| self.article(v)).collect::<PyResult<Vec<_>>>()?),
            "named_events":self.seq(value.named_events.iter().map(|(k, v)| self.seq(vec![self.text(k)?, self.primitive(v)?])).collect::<PyResult<Vec<_>>>()?),
            "named_refusals":self.seq(value.named_refusals.iter().map(|(k, v)| self.seq(vec![self.optional_text(k)?, self.text(v)?])).collect::<PyResult<Vec<_>>>()?),
            "rust_work_diagnostic":self.seq(value.rust_work_diagnostic.iter().map(|(k, v)| self.seq(vec![self.text(k)?, self.u(v)?])).collect::<PyResult<Vec<_>>>()?),
            "noise_record":match &value.noise_record { Some(v) => self.noise(v), None => self.none() },
            "canonical_subturn_authority":match &value.canonical_subturn_authority { Some(v) => self.subturn(v), None => self.none() },
            "law_refusals":record!(self, "CanonicalLawRefusalFactsV1", { "noise_refusal":self.optional_text(&value.law_refusals.noise_refusal), "rays_refusal":self.optional_text(&value.law_refusals.rays_refusal) }),
            "shell_events":self.seq(value.shell_events.iter().map(|v| self.shell(v)).collect::<PyResult<Vec<_>>>()?),
            "failure":match &value.failure { Some(v) => self.failure(v), None => self.none() } })
    }
    pub fn report(&self, value: &DensityRawBatchReportV1) -> PyResult<Bound<'py, PyAny>> {
        let results = self.seq(
            value
                .results
                .iter()
                .map(|v| self.result(v))
                .collect::<PyResult<Vec<_>>>()?,
        )?;
        let internal = match &value.internal_failure {
            Some(v) => self.seq(vec![
                self.n(v.spec_index)?,
                self.text(&v.detail)?,
                self.result(&v.partial_spec_effects)?,
            ])?,
            None => self.none()?,
        };
        self.seq(vec![results, internal])
    }
}
