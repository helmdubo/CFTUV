//! Test-only wire: fan TAB inv00 TAB inv01 TAB inv11 TAB owner TAB turn TAB q TAB H
//! TAB incoming_x TAB incoming_y TAB outgoing_x TAB outgoing_y TAB V|N TAB auto|adaptive
//! [TAB required_csv TAB preparation_cap TAB search_cap TAB resource_height_cap]. Q=n/d.
use cftuv_density::atlas_plan::{
    BoolField, ChartPiece, DecimalEnvelope, NumericField, PoleField, PoleOwnership,
};
use cftuv_density::density_bind_v1::{self, RawFanGeometryV1};
use cftuv_density::density_construct_v1::ConstructorV1;
use cftuv_density::density_expand_v1::ExpandLimits;
use cftuv_density::density_expression::Expressions;
use cftuv_density::density_laws_v1::OwnedDensityLawsV1;
use cftuv_density::fan_compile::{
    AuthorityWindowBounds, CompileLimits, PreparedAuthorityWindow, PreparedFanAuthority,
};
use cftuv_density::fan_prepare::PrepareRecord;
use cftuv_density::farey::WorkBudget;
use cftuv_density::geometry::WindowLaw;
use cftuv_density::legacy_binding::LegacyCertificate;
use cftuv_density::window_build::PreparedWindowRecord;
use cftuv_density::{IBig, Rational, UBig};
use std::io::{self, BufRead, Write};

fn string(v: &str) -> String {
    format!(
        "\"{}\"",
        v.replace('\\', "\\\\")
            .replace('"', "\\\"")
            .replace('\n', "\\n")
            .replace('\r', "\\r")
            .replace('\t', "\\t")
    )
}
fn q(v: &str) -> Result<Rational, String> {
    let (n, d) = v.split_once('/').unwrap_or((v, "1"));
    Rational::new(
        n.parse().map_err(|_| "numerator")?,
        d.parse().map_err(|_| "denominator")?,
    )
    .map_err(|_| "Q".into())
}
fn rq(v: &Rational) -> String {
    let (n, d) = v.parts();
    format!("[{n},{d}]")
}
fn joined<T>(v: &[T], f: impl Fn(&T) -> Result<String, String>) -> Result<String, String> {
    Ok(v.iter().map(f).collect::<Result<Vec<_>, _>>()?.join(","))
}
fn env(v: &DecimalEnvelope) -> Result<String, String> {
    let t = v.quantum.ok_or("UNKNOWN_DECIMAL_QUANTUM")?;
    Ok(format!(
        "{{\"lower\":{},\"upper\":{},\"absolute_error_bound\":{},\"quantum\":[{},{},{}]}}",
        rq(&v.lower),
        rq(&v.upper),
        rq(&v.absolute_error_bound),
        t.lower,
        t.upper,
        t.absolute_error_bound
    ))
}
fn record(v: &PrepareRecord) -> Result<String, String> {
    Ok(format!(
        "{{\"use_x\":{},\"sign\":{},\"lower\":{},\"upper\":{}}}",
        v.use_x_denominator,
        v.denominator_sign,
        env(&v.lower_slope_envelope)?,
        env(&v.upper_slope_envelope)?
    ))
}
fn int(v: &NumericField) -> Result<String, String> {
    match v {
        NumericField::Int(v) => Ok(v.to_string()),
        _ => Err("NONINTEGER_TYPED_FIELD".into()),
    }
}
fn boolean(v: &BoolField) -> Result<String, String> {
    match v {
        BoolField::Bool(v) => Ok(v.to_string()),
        _ => Err("NONBOOL_TYPED_FIELD".into()),
    }
}
fn piece(v: &ChartPiece) -> Result<String, String> {
    let pole = match v.pole_ownership {
        PoleField::Enum(PoleOwnership::None) => "NONE",
        PoleField::Enum(PoleOwnership::XZero) => "X_ZERO",
        PoleField::Enum(PoleOwnership::YZero) => "Y_ZERO",
        _ => return Err("UNKNOWN_POLE_FIELD".into()),
    };
    Ok(format!("{{\"index\":{},\"use_x\":{},\"sign\":{},\"lower\":{},\"upper\":{},\"lower_included\":{},\"upper_included\":{},\"increases\":{},\"pole\":{}}}",int(&v.piece_index)?,boolean(&v.use_x_denominator)?,int(&v.denominator_sign)?,env(&v.lower_slope_envelope)?,env(&v.upper_slope_envelope)?,boolean(&v.lower_endpoint_included)?,boolean(&v.upper_endpoint_included)?,boolean(&v.slope_increases_in_ordinal_order)?,string(pole)))
}
fn window(v: &PreparedWindowRecord) -> Result<String, String> {
    match v {
        PreparedWindowRecord::SingleChart(v) => {
            Ok(format!("{{\"kind\":\"SINGLE\",\"record\":{}}}", record(v)?))
        }
        PreparedWindowRecord::Atlas(v) => Ok(format!(
            "{{\"kind\":\"ATLAS\",\"record\":{},\"pieces\":[{}],\"termination_piece\":{}}}",
            record(&v.local_record)?,
            joined(&v.pieces, piece)?,
            v.termination_piece_index
        )),
    }
}
fn bounds(v: &AuthorityWindowBounds) -> String {
    format!("{{\"ordinal\":{},\"termination_lower\":{},\"termination_upper\":{},\"width\":{},\"admissible\":[{}]}}",v.ordinal,rq(&v.termination_lower_slope),rq(&v.termination_upper_slope),rq(&v.certified_termination_width),v.admissible.iter().map(rq).collect::<Vec<_>>().join(","))
}
fn authority_window(v: &PreparedAuthorityWindow) -> Result<String, String> {
    match v {
        PreparedAuthorityWindow::SingleChart {
            bounds: b,
            record: r,
        } => Ok(format!(
            "{{\"kind\":\"SINGLE\",\"bounds\":{},\"record\":{}}}",
            bounds(b),
            record(r)?
        )),
        PreparedAuthorityWindow::Atlas {
            bounds: b,
            pieces,
            termination_piece_index,
        } => Ok(format!(
            "{{\"kind\":\"ATLAS\",\"bounds\":{},\"pieces\":[{}],\"termination_piece\":{}}}",
            bounds(b),
            joined(pieces, piece)?,
            termination_piece_index
        )),
    }
}
fn authority(v: &PreparedFanAuthority) -> Result<String, String> {
    Ok(format!("{{\"q\":{},\"height\":{},\"previous_height\":{},\"termination_height\":{},\"vectors\":[{}],\"windows\":[{}],\"previous_counts\":[{}],\"counts\":[{}]}}",v.max_subturn_q,v.minimal_common_height,v.exhaustive_previous_height,v.termination_height_upper_bound,v.bound_primitive_integer_vectors.iter().map(|v|format!("[{},{}]",v.0,v.1)).collect::<Vec<_>>().join(","),joined(&v.ordinal_windows,authority_window)?,v.previous_counts.iter().map(ToString::to_string).collect::<Vec<_>>().join(","),v.counts.iter().map(ToString::to_string).collect::<Vec<_>>().join(",")))
}
fn budget(v: &Option<WorkBudget>) -> String {
    v.as_ref().map_or("null".into(), |v| {
        format!("[{}, {}, {}]", v.cap, v.shell_probes, v.order_steps)
    })
}
fn decimal_units(v: &Rational) -> Result<IBig, String> {
    let scale: IBig = "1000000000000000000000000000".parse().unwrap();
    let (n, d) = v.parts();
    let n = n * scale;
    let d = IBig::from(d.clone());
    if &n % &d != IBig::ZERO {
        Err("NON_GRID_ENDPOINT".into())
    } else {
        Ok(n / d)
    }
}
fn certificate(v: &Option<LegacyCertificate>) -> Result<String, String> {
    let Some(v) = v else { return Ok("null".into()) };
    let lower = decimal_units(&v.ideal_window_lower_slope_envelope.upper)?;
    let upper = decimal_units(&v.ideal_window_upper_slope_envelope.lower)?;
    let (width, _) = cftuv_density::decimal_projection::width_from_units(&lower, &upper, 28)
        .map_err(|e| format!("{e:?}"))?;
    let exponent = width.exponent();
    let p: UBig = format!("1{}", "0".repeat(exponent.unsigned_abs() as usize))
        .parse()
        .unwrap();
    let ratio = if exponent < 0 {
        Rational::new(width.coefficient().clone(), p).unwrap()
    } else {
        Rational::integer(width.coefficient() * IBig::from(p))
    };
    if ratio != v.certified_window_width_lower_bound {
        return Err("CERTIFICATE_WIDTH_PROVENANCE_MISMATCH".into());
    }
    Ok(format!(
        "{{\"vector\":[{},{}],\"lower\":{},\"upper\":{},\"width\":[{},{}]}}",
        v.bound_primitive_integer_vector.0,
        v.bound_primitive_integer_vector.1,
        env(&v.ideal_window_lower_slope_envelope)?,
        env(&v.ideal_window_upper_slope_envelope)?,
        width.coefficient(),
        exponent
    ))
}
pub fn evaluate_line(line: &str) -> Result<String, String> {
    let f: Vec<_> = line.split('\t').collect();
    if f.len() < 14 || f[0] != "fan" {
        return Err("WIRE_PROFILE".into());
    }
    let mut limits = CompileLimits::default();
    if f.len() > 15 {
        limits.preparation_work_cap = f[15].parse().map_err(|_| "preparation cap")?;
    }
    if f.len() > 16 {
        limits.search_work_cap = f[16].parse().map_err(|_| "search cap")?;
    }
    if f.len() > 17 {
        limits.resource_height_cap = f[17].parse().map_err(|_| "height cap")?;
    }
    let owner = f[4].parse().map_err(|_| "owner")?;
    let turn = f[5].parse().map_err(|_| "turn")?;
    let qv = f[6].parse().map_err(|_| "q")?;
    let count = f[7].parse().map_err(|_| "count")?;
    let law = match f[12] {
        "V" => WindowLaw::Voronoi,
        "N" => WindowLaw::NarrowBand,
        _ => return Err("law".into()),
    };
    let adaptive_only = match f[13] {
        "auto" => false,
        "adaptive" => true,
        _ => return Err("mode".into()),
    };
    let required = if f.len() > 14 {
        if f[14] == "_" {
            vec![]
        } else {
            f[14]
                .split(',')
                .map(|v| match v {
                    "1" => Ok(true),
                    "0" => Ok(false),
                    _ => Err("reason flag"),
                })
                .collect::<Result<Vec<_>, _>>()?
        }
    } else {
        vec![true; count]
    };
    let inv00 = q(f[1])?;
    let inv01 = q(f[2])?;
    let inv11 = q(f[3])?;
    let raw = RawFanGeometryV1 {
        inverse_gram: [[inv00, inv01.clone()], [inv01, inv11]],
        incoming: [q(f[8])?, q(f[9])?],
        outgoing: [q(f[10])?, q(f[11])?],
        owner_orientation: owner,
        turn_orientation: turn,
        hidden_count: count,
        q: qv,
        window_law: law,
    };
    let expansion = ExpandLimits {
        max_terms: 4096,
        max_polynomial_exponent: 16,
        max_rational_power_bits: 65536,
    };
    let mut graph = Expressions::new();
    let mut c = ConstructorV1::new(&mut graph);
    let produced = match density_bind_v1::produce_principal_fan(&mut c, &raw, expansion) {
        Ok(v) => v,
        Err(e) => {
            return Ok(format!(
                "{{\"stage\":\"PRODUCER\",\"error\":{}}}",
                string(&format!("{e:?}"))
            ))
        }
    };
    let mut laws = OwnedDensityLawsV1::new(&mut c, produced.geometry, produced.values, expansion)
        .map_err(|e| format!("{e:?}"))?;
    let report = density_bind_v1::bind_owned_fan_once(
        &mut laws,
        turn,
        qv,
        &required,
        adaptive_only,
        &limits,
    );
    let windows = report
        .windows
        .as_ref()
        .map(|v| joined(&v.records, window).map(|v| format!("[{v}]")))
        .transpose()?
        .unwrap_or("null".into());
    let budgets = format!(
        "{{\"legacy\":[{}],\"preparation\":{},\"search\":{}}}",
        report
            .legacy_budgets
            .iter()
            .map(budget)
            .collect::<Vec<_>>()
            .join(","),
        budget(&report.adaptive_budgets.preparation),
        budget(&report.adaptive_budgets.search)
    );
    match report.result {Ok(v)=>Ok(format!("{{\"windows\":{windows},\"certificates\":[{}],\"authority\":{},\"budgets\":{budgets}}}",joined(&v.certificates,certificate)?,v.authority.as_ref().map(authority).transpose()?.unwrap_or("null".into()))),
        Err(e)=>Ok(format!("{{\"stage\":\"CONTROLLER\",\"error\":{},\"windows\":{windows},\"budgets\":{budgets}}}",string(&format!("{e:?}"))))}
}
fn main() {
    let stdin = io::stdin();
    let mut output = io::BufWriter::new(io::stdout().lock());
    for line in stdin.lock().lines() {
        let line = line.expect("stdin");
        let result = evaluate_line(&line)
            .unwrap_or_else(|e| format!("{{\"stage\":\"WIRE\",\"internal\":{}}}", string(&e)));
        writeln!(output, "{result}").unwrap();
        output.flush().unwrap();
    }
}
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn unknown_quantum_is_protocol_error_not_inferred_from_one() {
        let mut e = cftuv_density::window_build::exact_decimal_interval(IBig::ONE);
        e.quantum = None;
        assert_eq!(env(&e), Err("UNKNOWN_DECIMAL_QUANTUM".into()));
    }
}
