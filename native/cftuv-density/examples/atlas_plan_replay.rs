//! TAB replay подготовки Atlas: VALIDATE/TERM/END/CELL/CONTAINS/HEIGHT/REACH/
//! BRACKET/PROVEN/ORDINAL/SEARCH. Wire descriptors сохраняют все source fields.
//! Рациональные callback fixtures изолируют порядок и отказы, без Sympy/float.

use std::io::{self, BufRead};

use cftuv_density::atlas_plan::*;
use cftuv_density::atlas_search::GlobalHeightSegment;
use cftuv_density::farey::PrimitiveVector;
use cftuv_density::{IBig, Rational};

fn r(text: &str) -> Rational {
    let (n, d) = text.split_once('/').unwrap();
    Rational::new(n.parse().unwrap(), d.parse().unwrap()).unwrap()
}
fn rt(value: &Rational) -> String {
    let (n, d) = value.parts();
    format!("{n}/{d}")
}
fn add(a: &Rational, b: &Rational) -> Rational {
    let (an, ad) = a.parts();
    let (bn, bd) = b.parts();
    Rational::new(an * bd + bn * ad, ad * bd).unwrap()
}
fn negate(a: &Rational) -> Rational {
    let (n, d) = a.parts();
    Rational::new(-n, d.clone()).unwrap()
}
fn div(a: &Rational, b: &Rational) -> Rational {
    let (an, ad) = a.parts();
    let (bn, bd) = b.parts();
    let n = an * bd;
    let n = if bn < &IBig::ZERO { -n } else { n };
    Rational::new(n, ad * cftuv_radical::arith::magnitude(bn)).unwrap()
}
fn n(text: &str) -> NumericField {
    match &text[..1] {
        "i" => NumericField::Int(text[1..].parse().unwrap()),
        "n" => NumericField::OtherNumber(r(&text[1..])),
        "x" => NumericField::NonNumeric,
        _ => panic!("number descriptor"),
    }
}
fn b(text: &str) -> BoolField {
    match &text[..1] {
        "b" => BoolField::Bool(text.ends_with('1')),
        "t" => BoolField::OtherTruthiness(text.ends_with('1')),
        _ => panic!("bool descriptor"),
    }
}
fn k(text: &str) -> EndpointKind {
    match text {
        "O" => EndpointKind::Open,
        "C" => EndpointKind::Closed,
        _ => EndpointKind::Other,
    }
}
fn e(text: &str) -> DecimalEnvelope {
    let f: Vec<_> = text.split('~').collect();
    DecimalEnvelope {
        lower: r(f[0]),
        upper: r(f[1]),
        lower_kind: k(f[2]),
        upper_kind: k(f[3]),
        absolute_error_bound: r(f[4]),
        quantum: None,
    }
}
fn et(value: &DecimalEnvelope) -> String {
    let kt = |kind| match kind {
        EndpointKind::Open => "O",
        EndpointKind::Closed => "C",
        EndpointKind::Other => "X",
    };
    format!(
        "{}~{}~{}~{}~{}",
        rt(&value.lower),
        rt(&value.upper),
        kt(value.lower_kind),
        kt(value.upper_kind),
        rt(&value.absolute_error_bound)
    )
}
fn ratio(text: &str) -> RatioField {
    if text == "_" {
        RatioField::Other
    } else {
        let (a, b) = text.split_once('*').unwrap();
        RatioField::Pair(n(a), n(b))
    }
}
fn piece(text: &str) -> ChartPiece {
    let f: Vec<_> = text.split(',').collect();
    ChartPiece {
        is_piece_v1: f[0] == "1",
        piece_index: n(f[1]),
        use_x_denominator: b(f[2]),
        denominator_sign: n(f[3]),
        lower_slope_envelope: e(f[4]),
        upper_slope_envelope: e(f[5]),
        lower_endpoint_included: b(f[6]),
        upper_endpoint_included: b(f[7]),
        slope_increases_in_ordinal_order: b(f[8]),
        pole_ownership: match f[9] {
            "N" => PoleField::Enum(PoleOwnership::None),
            "X" => PoleField::Enum(PoleOwnership::XZero),
            "Y" => PoleField::Enum(PoleOwnership::YZero),
            _ => PoleField::Other,
        },
    }
}
fn window(text: &str) -> Window {
    if text == "-" {
        return Window::SingleChart;
    }
    let f: Vec<_> = text.split('|').collect();
    Window::Atlas(AtlasWindow {
        kind: match f[0] {
            "A" => WindowRecordKind::AtlasV1,
            "P" => WindowRecordKind::Prepared,
            _ => WindowRecordKind::Other,
        },
        ordinal: n(f[1]),
        termination_piece_index: n(f[2]),
        termination_lower_slope: ratio(f[3]),
        termination_upper_slope: ratio(f[4]),
        certified_termination_width: ratio(f[5]),
        admissible_lower_outward: ratio(f[6]),
        admissible_lower_inward: ratio(f[7]),
        admissible_upper_inward: ratio(f[8]),
        admissible_upper_outward: ratio(f[9]),
        pieces: if f[10] == "_" {
            vec![]
        } else {
            f[10].split(';').map(piece).collect()
        },
    })
}
fn atlas(value: &Window) -> &AtlasWindow {
    if let Window::Atlas(value) = value {
        value
    } else {
        panic!("atlas expected")
    }
}
fn record(text: &str) -> LocalRecord {
    let f: Vec<_> = text.split(',').collect();
    LocalRecord {
        use_x_denominator: b(f[0]),
        denominator_sign: n(f[1]),
        lower_slope_envelope: e(f[2]),
        upper_slope_envelope: e(f[3]),
    }
}
fn bracket(text: &str) -> [Rational; 4] {
    text.split(',')
        .map(r)
        .collect::<Vec<_>>()
        .try_into()
        .unwrap()
}
fn bt(value: &[Rational; 4]) -> String {
    value.iter().map(rt).collect::<Vec<_>>().join(",")
}
fn segment(text: &str) -> GlobalHeightSegment {
    let f: Vec<_> = text.split(',').collect();
    GlobalHeightSegment {
        use_x_denominator: f[0] == "1",
        denominator_sign: f[1].parse().unwrap(),
        outer_lower: r(f[2]),
        inner_lower: r(f[3]),
        inner_upper: r(f[4]),
        outer_upper: r(f[5]),
    }
}
fn st(value: &GlobalHeightSegment) -> String {
    format!(
        "{},{},{},{},{},{}",
        u8::from(value.use_x_denominator),
        value.denominator_sign,
        rt(&value.outer_lower),
        rt(&value.inner_lower),
        rt(&value.inner_upper),
        rt(&value.outer_upper)
    )
}
fn segments(values: &[GlobalHeightSegment]) -> String {
    if values.is_empty() {
        "_".into()
    } else {
        values.iter().map(st).collect::<Vec<_>>().join(";")
    }
}
type Vector = (Rational, Rational);
fn vector(text: &str) -> Vector {
    let (a, b) = text.split_once(',').unwrap();
    (r(a), r(b))
}
fn vt(v: &Vector) -> String {
    format!("{},{}", rt(&v.0), rt(&v.1))
}
fn vectors(text: &str) -> Vec<Vector> {
    if text == "_" {
        vec![]
    } else {
        text.split(';').map(vector).collect()
    }
}
fn vlist(values: &[Vector]) -> String {
    if values.is_empty() {
        "_".into()
    } else {
        values.iter().map(vt).collect::<Vec<_>>().join(";")
    }
}
fn error(error: AtlasPlanFailure<String>) -> String {
    match error {
        AtlasPlanFailure::Invalid(error) => error.message().into(),
        AtlasPlanFailure::Callback(error) => error,
        AtlasPlanFailure::UnsupportedChartDescriptor => {
            "INTERNAL:UnsupportedChartDescriptor".into()
        }
        AtlasPlanFailure::UnsupportedDenominatorDescriptor => {
            "INTERNAL:UnsupportedDenominatorDescriptor".into()
        }
        AtlasPlanFailure::StrictZip { argument, relation } => {
            let adjective = if relation == ZipRelation::Shorter {
                "shorter"
            } else {
                "longer"
            };
            let compared = if argument == 2 {
                "argument 1"
            } else {
                "arguments 1-2"
            };
            format!("zip() argument {argument} is {adjective} than {compared}")
        }
    }
}

struct Laws {
    trace: Vec<String>,
    failure: usize,
    accept: Vec<bool>,
    locals: usize,
    boundaries: Vec<Vector>,
    width: Rational,
    height: IBig,
}
impl Laws {
    fn begin(&mut self, prefix: &str) -> Result<(), String> {
        if self.failure != 0 && self.trace.len() + 1 == self.failure {
            self.trace.push(format!("{prefix}:!refused"));
            return Err("refused".into());
        }
        Ok(())
    }
    fn emit<T>(&mut self, prefix: String, result: String, value: T) -> Result<T, String> {
        self.begin(&prefix)?;
        self.trace.push(format!("{prefix}:{result}"));
        Ok(value)
    }
    fn local(&mut self, ordinal: usize, v: &Vector, q: usize) -> Result<bool, String> {
        let result = self.accept[self.locals % self.accept.len()];
        self.locals += 1;
        self.emit(
            format!("L:{ordinal}:{q}:{}", vt(v)),
            u8::from(result).to_string(),
            result,
        )
    }
    fn local_height(&mut self, item: &TerminationBox) -> Result<IBig, String> {
        self.emit(
            format!("H:{},{}:1:1", rt(&item.lower), rt(&item.upper)),
            self.height.to_string(),
            self.height.clone(),
        )
    }
}
impl SegmentPlanLaws for Laws {
    type Vector = Vector;
    type Scalar = Rational;
    type Error = String;
    fn denominator(&mut self, v: &Vector, x: bool) -> Result<Rational, String> {
        let result = if x { v.0.clone() } else { v.1.clone() };
        self.emit(format!("D:{}:{}", vt(v), u8::from(x)), rt(&result), result)
    }
    fn sign(&mut self, value: &Rational) -> Result<i8, String> {
        let n = value.parts().0;
        let result = if n < &IBig::ZERO {
            -1
        } else if n > &IBig::ZERO {
            1
        } else {
            0
        };
        self.emit(format!("S:{}", rt(value)), result.to_string(), result)
    }
    fn slope(&mut self, v: &Vector, x: bool) -> Result<Rational, String> {
        let prefix = format!("T:{}:{}", vt(v), u8::from(x));
        self.begin(&prefix)?;
        let (a, b) = if x { (&v.1, &v.0) } else { (&v.0, &v.1) };
        if b.parts().0.is_zero() {
            self.trace.push(format!("{prefix}:!zero_denominator"));
            return Err("zero_denominator".into());
        }
        let result = div(a, b);
        self.trace.push(format!("{prefix}:{}", rt(&result)));
        Ok(result)
    }
    fn decimal_envelope(&mut self, value: &Rational) -> Result<DecimalEnvelope, String> {
        let result = DecimalEnvelope {
            lower: add(value, &negate(&self.width)),
            upper: add(value, &self.width),
            lower_kind: EndpointKind::Closed,
            upper_kind: EndpointKind::Closed,
            absolute_error_bound: self.width.clone(),
            quantum: None,
        };
        self.emit(format!("E:{}", rt(value)), et(&result), result)
    }
    fn local_candidate(&mut self, ordinal: usize, v: &Vector, q: usize) -> Result<bool, String> {
        self.local(ordinal, v, q)
    }
    fn vector_from_slope(&mut self, value: &Rational, x: bool, sign: i8) -> Result<Vector, String> {
        let (n, d) = value.parts();
        let one = Rational::integer(cftuv_radical::arith::signed(d) * IBig::from(sign));
        let number = Rational::integer(n * IBig::from(sign));
        let result = if x { (one, number) } else { (number, one) };
        self.emit(
            format!("V:{}:{}:{sign}", rt(value), u8::from(x)),
            vt(&result),
            result,
        )
    }
    fn local_transition(
        &mut self,
        ordinal: usize,
        v: &PrimitiveVector,
        q: usize,
    ) -> Result<bool, String> {
        self.local(
            ordinal,
            &(
                Rational::integer(v.0.clone()),
                Rational::integer(v.1.clone()),
            ),
            q,
        )
    }
    fn subturn_boundary_vectors(
        &mut self,
        ordinal: usize,
        orientation: i8,
        q: usize,
    ) -> Result<Vec<Vector>, String> {
        self.emit(
            format!("B:{ordinal}:{orientation}:{q}"),
            vlist(&self.boundaries),
            self.boundaries.clone(),
        )
    }
}

fn run(f: &[&str], laws: &mut Laws) -> Result<String, AtlasPlanFailure<String>> {
    let invalid = AtlasPlanFailure::Invalid;
    Ok(match f[0] {
        "VALIDATE" => {
            validate_atlas_structure(atlas(&window(f[7]))).map_err(invalid)?;
            "VALID".into()
        }
        "TERM" => {
            let w = window(f[7]);
            let item = atlas(&w);
            let p = termination_piece(item).map_err(invalid)?;
            format!(
                "TERM:{}",
                item.pieces
                    .iter()
                    .position(|p2| std::ptr::eq(p, p2))
                    .unwrap()
            )
        }
        "END" => {
            let p = piece(f[7]);
            let (a, b) = piece_ordered_endpoints(&p);
            format!("END:{};{}", et(a), et(b))
        }
        "CELL" => format!("CELL:{}", piece_cell_index(&piece(f[7]))),
        "CONTAINS" => {
            let v = vector(f[8]);
            let v = PrimitiveVector(v.0.parts().0.clone(), v.1.parts().0.clone());
            format!(
                "CONTAINS:{}",
                u8::from(piece_contains_vector(&piece(f[7]), &v))
            )
        }
        "HEIGHT" => {
            let boxes = if f[7] == "_" {
                vec![]
            } else {
                f[7].split(';')
                    .map(|t| {
                        let p: Vec<_> = t.split(',').collect();
                        TerminationBox {
                            lower: r(p[0]),
                            upper: r(p[1]),
                            use_x_denominator: b(p[2]),
                            denominator_sign: n(p[3]),
                        }
                    })
                    .collect()
            };
            let height = global_termination_height(&boxes, |b| laws.local_height(b))
                .map_err(AtlasPlanFailure::Callback)?;
            format!("HEIGHT:{height}")
        }
        "REACH" => {
            let w = window(f[7]);
            let reached = reachable_pieces(atlas(&w), |v| laws.local_transition(1, v, 2))?;
            format!(
                "REACH:{}",
                reached
                    .iter()
                    .map(usize::to_string)
                    .collect::<Vec<_>>()
                    .join(",")
            )
        }
        "BRACKET" => {
            let boundaries = laws.boundaries.clone();
            format!(
                "BRACKET:{}",
                bt(&piece_bracket(&boundaries, &piece(f[7]), laws)?)
            )
        }
        "PROVEN" => format!(
            "SEGMENTS:{}",
            segments(&[proven_segment(
                f[8].parse().unwrap(),
                f[9].parse().unwrap(),
                &segment(f[7]),
                laws
            )?])
        ),
        "ORDINAL" => {
            let out = ordinal_segments(
                f[10].parse().unwrap(),
                f[11].parse().unwrap(),
                f[12].parse().unwrap(),
                &window(f[7]),
                &record(f[8]),
                &bracket(f[9]),
                laws,
            )?;
            format!("SEGMENTS:{}", segments(&out))
        }
        "SEARCH" => {
            let windows: Vec<_> = if f[7] == "_" {
                vec![]
            } else {
                f[7].split('@').map(window).collect()
            };
            let records: Vec<_> = if f[8] == "_" {
                vec![]
            } else {
                f[8].split('@').map(record).collect()
            };
            let brackets: Vec<_> = if f[9] == "_" {
                vec![]
            } else {
                f[9].split('@').map(bracket).collect()
            };
            let result = search_segments(
                f[10].parse().unwrap(),
                f[11].parse().unwrap(),
                &windows,
                &records,
                &brackets,
                laws,
            )?;
            format!(
                "PLANS:{}",
                if result.is_empty() {
                    "-".into()
                } else {
                    result
                        .iter()
                        .map(|v| segments(v))
                        .collect::<Vec<_>>()
                        .join("#")
                }
            )
        }
        _ => panic!("operation"),
    })
}
fn main() {
    for line in io::stdin().lock().lines() {
        let line = line.unwrap();
        let f: Vec<_> = line.split('\t').collect();
        let mut laws = Laws {
            trace: vec![],
            failure: f[2].parse().unwrap(),
            accept: f[3].chars().map(|c| c == '1').collect(),
            locals: 0,
            boundaries: vectors(f[4]),
            width: r(f[5]),
            height: f[6].parse().unwrap(),
        };
        let result = run(&f, &mut laws).unwrap_or_else(|e| format!("ERROR:{}", error(e)));
        println!("{result}\tTRACE:{}", laws.trace.join("@"));
    }
}
