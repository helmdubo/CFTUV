//! Test-only closed rational declaration wire. Не импортирует AST/готовые нормали.
use cftuv_density::density_construct_v1::ConstructorV1;
use cftuv_density::density_expand_v1::ExpandLimits;
use cftuv_density::density_expression::Expressions;
use cftuv_density::density_raw_laws_v1::*;
use cftuv_density::density_raw_types_v1::*;
use cftuv_density::Rational;
use std::io::{self, BufRead, Write};
fn string(s: &str) -> String {
    format!(
        "\"{}\"",
        s.replace('\\', "\\\\")
            .replace('"', "\\\"")
            .replace('\n', "\\n")
            .replace('\r', "\\r")
            .replace('\t', "\\t")
    )
}
struct Wire<'a> {
    values: std::str::Split<'a, char>,
}
impl<'a> Wire<'a> {
    fn next(&mut self) -> Result<&'a str, String> {
        self.values.next().ok_or("WIRE_SHORT".into())
    }
    fn number(&mut self) -> Result<usize, String> {
        self.next()?.parse().map_err(|_| "WIRE_INTEGER".into())
    }
    fn q(&mut self) -> Result<Rational, String> {
        let value = self.next()?;
        let (n, d) = value.split_once('/').unwrap_or((value, "1"));
        Rational::new(
            n.parse().map_err(|_| "WIRE_NUMERATOR")?,
            d.parse().map_err(|_| "WIRE_DENOMINATOR")?,
        )
        .map_err(|_| "WIRE_RATIO".into())
    }
    fn text(&mut self) -> Result<Option<String>, String> {
        let value = self.next()?;
        if value == "-" {
            return Ok(None);
        }
        if value.len() % 2 != 0 {
            return Err("WIRE_HEX".into());
        }
        let bytes = (0..value.len())
            .step_by(2)
            .map(|i| u8::from_str_radix(&value[i..i + 2], 16).map_err(|_| "WIRE_HEX".to_string()))
            .collect::<Result<Vec<_>, _>>()?;
        Ok(Some(String::from_utf8(bytes).map_err(|_| "WIRE_UTF8")?))
    }
    fn point(&mut self) -> Result<RawDensityPointV1, String> {
        Ok(RawDensityPointV1 {
            x: self.q()?,
            y: self.q()?,
        })
    }
    fn incident(&mut self) -> Result<RawDensityIncidentV1, String> {
        let start = self.point()?;
        let end = self.point()?;
        let owner_left = match self.next()? {
            "T" => Some(true),
            "F" => Some(false),
            "N" => None,
            _ => return Err("WIRE_SIDE".into()),
        };
        let orientation = self.text()?.map(|s| match s.as_str() {
            "A_START_TO_END" => ChainUseOrientationV1::AStartToEnd,
            "B_START_TO_END" => ChainUseOrientationV1::BStartToEnd,
            _ => ChainUseOrientationV1::Other(s),
        });
        let anchor_vertex_id = self.text()?;
        let source_vertex_start_id = self.text()?;
        let source_vertex_end_id = self.text()?;
        let support_id = self.text()?;
        let physical_edge_id = self.text()?;
        let chain_use_id = self.text()?;
        let mut declarations = vec![];
        for _ in 0..self.number()? {
            let tag = self.next()?;
            declarations.push(match tag {
                "P" => RawDirectionDeclarationV1::CertifiedPlanar {
                    x: self.q()?,
                    y: self.q()?,
                },
                "A" => RawDirectionDeclarationV1::CertifiedAffine {
                    x: self.q()?,
                    y: self.q()?,
                    reference_metric_id: self.text()?,
                },
                "O" => RawDirectionDeclarationV1::Other {
                    kind: self.text()?.ok_or("WIRE_KIND")?,
                },
                _ => return Err("WIRE_DIRECTION".into()),
            });
        }
        let mut cycles = vec![];
        for _ in 0..self.number()? {
            let mut points = vec![];
            for _ in 0..self.number()? {
                points.push(self.point()?)
            }
            cycles.push(points);
        }
        let mut ordinals = vec![];
        for _ in 0..self.number()? {
            ordinals.push(self.number()?)
        }
        let mut vertex_ids = vec![];
        for _ in 0..self.number()? {
            let mut ids = vec![];
            for _ in 0..self.number()? {
                ids.push(self.text()?.ok_or("WIRE_VERTEX_ID")?)
            }
            vertex_ids.push(ids)
        }
        let mut face_ids = vec![];
        for _ in 0..self.number()? {
            face_ids.push(self.text()?.ok_or("WIRE_FACE_ID")?)
        }
        Ok(RawDensityIncidentV1 {
            start,
            end,
            owner_left,
            anchor_vertex_id,
            source_vertex_start_id,
            source_vertex_end_id,
            support_id,
            chain_use_orientation: orientation,
            source_direction_declarations: declarations,
            owner_face_cycles: cycles,
            owner_face_vertex_id_cycles: vertex_ids,
            owner_face_ids: face_ids,
            owner_face_edge_ordinals: ordinals,
            physical_edge_id,
            chain_use_id,
        })
    }
}
fn ratio(q: &Rational) -> String {
    let (n, d) = q.parts();
    format!("[{n},{d}]")
}
fn failure(error: &RawLawsFailureV1) -> String {
    match error {
        RawLawsFailureV1::Geometry(e)=>format!("{{\"exception_type\":\"ReferenceGeometryError\",\"outcome\":{},\"message\":{}}}",string(e.outcome()),string(&e.message())),
        RawLawsFailureV1::Contract(RawContractFailureV1::InvalidGramFacts)=>"{\"exception_type\":\"ValueError\",\"outcome\":null,\"message\":\"DENSITY_ORACLE_V1_INVALID_GRAM_FACTS\"}".into(),
        RawLawsFailureV1::Arithmetic(cftuv_density::density_laws_v1::LawsErrorV1::Sign(cftuv_density::density_expand_v1::LazySignErrorV1::Sign(cftuv_density::density_construct_v1::OwnedSignErrorV1::Sign(cftuv_density::density_sign::SignFailure::Undecidable))))=>"{\"exception_type\":\"ReferenceGeometryError\",\"outcome\":\"REFERENCE_CERTIFIED_PREDICATE_UNDECIDABLE\",\"message\":\"Density A exact sign is not certified without generic factorization\"}".into(),
        _=>format!("{{\"internal\":{}}}",string(&format!("{error:?}"))),
    }
}
fn direction(d: RationalDirectionV1) -> String {
    match d {
        RationalDirectionV1::Rational(v) => {
            format!("{{\"primitive\":[{},{}],\"rational\":true}}", v.0, v.1)
        }
        RationalDirectionV1::Irrational => "{\"primitive\":null,\"rational\":false}".into(),
        RationalDirectionV1::Undetermined => "{\"primitive\":null,\"rational\":null}".into(),
    }
}
fn value_wire(
    c: &ConstructorV1<'_>,
    v: &cftuv_density::density_construct_v1::VectorExprV1,
) -> Result<String, String> {
    let rows = v
        .iter()
        .map(|x| {
            let scalar = c
                .numeric_value(x)
                .map_err(|e| format!("{e:?}"))?
                .ok_or("NON_NUMERIC_RAW_NORMAL")?;
            Ok(format!(
                "[{}]",
                scalar
                    .value()
                    .wire()
                    .iter()
                    .map(ToString::to_string)
                    .collect::<Vec<_>>()
                    .join(",")
            ))
        })
        .collect::<Result<Vec<String>, String>>()?;
    Ok(format!("[{}]", rows.join(",")))
}
fn evaluate(line: &str) -> Result<String, String> {
    let mut w = Wire {
        values: line.split('\t'),
    };
    if w.next()? != "raw" {
        return Err("WIRE_TAG".into());
    }
    let use_declarations = match w.next()? {
        "0" => false,
        "1" => true,
        _ => return Err("WIRE_DECLARATIONS".into()),
    };
    let turn: i8 = w.next()?.parse().map_err(|_| "WIRE_TURN")?;
    let gram = [w.q()?, w.q()?, w.q()?];
    let inverse = [w.q()?, w.q()?, w.q()?];
    let owner = w.next()?.parse().map_err(|_| "WIRE_OWNER")?;
    let metric_id = w.text()?;
    let raw = RawDensityGeometryV1 {
        metric: RawDensityMetricV1 {
            gram,
            inverse_gram: inverse,
            owner_orientation_sign: owner,
            metric_id,
        },
        incoming: w.incident()?,
        outgoing: w.incident()?,
    };
    if w.values.next().is_some() {
        return Err("WIRE_LONG".into());
    }
    let mut graph = Expressions::new();
    let mut c = ConstructorV1::new(&mut graph);
    let mut laws = RawDensityLawsV1::new(
        &mut c,
        ExpandLimits {
            max_terms: 4096,
            max_polynomial_exponent: 16,
            max_rational_power_bits: 65536,
        },
    );
    let produced = match laws.raw_geometry(&raw, use_declarations) {
        Ok(v) => v,
        Err(e) => return Ok(format!("{{\"error\":{}}}", failure(&e))),
    };
    let incoming = value_wire(laws.constructor(), &produced.incoming)?;
    let outgoing = value_wire(laws.constructor(), &produced.outgoing)?;
    let incoming_direction = direction(
        laws.rational_direction(&produced.incoming)
            .map_err(|e| format!("{e:?}"))?,
    );
    let outgoing_direction = direction(
        laws.rational_direction(&produced.outgoing)
            .map_err(|e| format!("{e:?}"))?,
    );
    let incoming_covector = laws
        .covector(&produced.metric, &produced.incoming)
        .map_err(|e| format!("{e:?}"))?;
    let outgoing_covector = laws
        .covector(&produced.metric, &produced.outgoing)
        .map_err(|e| format!("{e:?}"))?;
    let incoming_covector_wire = value_wire(laws.constructor(), &incoming_covector)?;
    let outgoing_covector_wire = value_wire(laws.constructor(), &outgoing_covector)?;
    let turn_result = match laws.turn_witness(
        &produced.metric,
        &incoming_covector,
        &outgoing_covector,
        turn,
    ) {
        Err(e) => format!("{{\"error\":{}}}", failure(&e)),
        Ok(v) => {
            let mut counts = vec![];
            for q in 2..=6 {
                for count in 0..q {
                    let feasible = laws
                        .turn_count_feasible(v.dot_sign, &v.cosine_squared, count, q)
                        .map_err(|e| format!("{e:?}"))?;
                    let exact = laws
                        .turn_count_exact_limit(v.dot_sign, &v.cosine_squared, count, q)
                        .map_err(|e| format!("{e:?}"))?;
                    let comparison = laws
                        .turn_count_comparison(&v.cosine_squared, count, q)
                        .map_err(|e| format!("{e:?}"))?
                        .map(|s| s.to_string())
                        .unwrap_or("null".into());
                    counts.push(format!("[{q},{count},{feasible},{exact},{comparison}]"));
                }
            }
            format!(
                "{{\"dot_sign\":{},\"cosine_squared\":{},\"counts\":[{}]}}",
                v.dot_sign,
                ratio(&v.cosine_squared),
                counts.join(",")
            )
        }
    };
    Ok(format!("{{\"incoming\":{incoming},\"outgoing\":{outgoing},\"incoming_covector\":{incoming_covector_wire},\"outgoing_covector\":{outgoing_covector_wire},\"incoming_direction\":{incoming_direction},\"outgoing_direction\":{outgoing_direction},\"turn\":{turn_result}}}"))
}
fn main() {
    let stdin = io::stdin();
    let mut output = io::BufWriter::new(io::stdout().lock());
    for line in stdin.lock().lines() {
        let line = line.expect("stdin");
        let result =
            evaluate(&line).unwrap_or_else(|e| format!("{{\"wire_error\":{}}}", string(&e)));
        writeln!(output, "{result}").unwrap();
        output.flush().unwrap();
    }
}
