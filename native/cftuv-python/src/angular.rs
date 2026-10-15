//! ANGULAR raw single-patch wire evaluator (PyO3 0.29, abi3); integration draft, compilation pending.
//! Input is EXACT dict with 12 keys from angular_wire.encode(). Each sequence is an
//! exact tuple: points=(host,(rat,rat)); chains=(ref,vertices,edges);
//! faces=(id,vertices,edges,triangles); sites=(opaque_key,cut,anchor,prev?,next?).
//! rat=(exact int numerator, exact positive int denominator); ref/key=(i64,i64,i64).
//! No host objects, geometry in Python, callbacks, contract constructors or effects.
//! Output (0, plans), plans=(ordinal,key,decision,stage_before,stage_after_success).
//! decision=("SKIP",reason) or ("REFLEX",n,m,orientation,A,B,D,C,T,events).
//! All vectors/scalars are rat tuples; events/stages are ordered string tuples.
//! Decline=(2,"ANGULAR_WIRE"|"ANGULAR_CORE_<Reason>",detail). Panic=>RuntimeError.
//! Shim MUST validate bindings/mode and preflight ALL patches before effects;
//! a decline reruns independent WHOLE oracle. Registration alone does not install a backend operation.
use std::panic::{catch_unwind, AssertUnwindSafe};
use cftuv_angular::{evaluate_patch, Chart, Chain, Decision, Face, PatchInput, Point, Ref, Site, Skip, SymbolicEvent, Vec2};
use cftuv_core::{num, rat::Rat};
use pyo3::prelude::*;
use pyo3::exceptions::PyRuntimeError;
use pyo3::types::{PyBool, PyDict, PyInt, PyString, PyTuple};
use pyo3::IntoPyObjectExt;
use crate::pyobj::{ibig_from_int, int_of_ibig, int_of_ubig};
const MAX_ITEMS:usize=100_000;
const MAX_BITS:usize=16_384;
enum Fail { Py(PyErr), Shape(String) }
impl From<PyErr> for Fail {fn from(e:PyErr)->Self {Self::Py(e)}}
type Read<T>=Result<T,Fail>;
fn shape<T>(why:&str)->Read<T>{Err(Fail::Shape(why.to_owned()))}
fn tuple<'py>(x:&Bound<'py,PyAny>)->Read<Bound<'py,PyTuple>> {
    if !x.is_exact_instance_of::<PyTuple>() {return shape("exact tuple required");}
    let t=x.cast::<PyTuple>().map_err(|_|Fail::Shape("tuple cast".into()))?.clone();
    if t.len()>MAX_ITEMS {return shape("sequence limit");} Ok(t)
}
fn fixed<'py>(x:&Bound<'py,PyAny>,n:usize)->Read<Bound<'py,PyTuple>> {
    let t=tuple(x)?;if t.len()!=n {return shape("tuple cardinality");}Ok(t)
}
fn int64(x:&Bound<'_,PyAny>)->Read<i64> {
    if !x.is_exact_instance_of::<PyInt>() {return shape("exact i64 required (bool rejected)");}
    x.extract::<i64>().map_err(|_|Fail::Shape("integer outside i64".into()))
}
fn boolean(x:&Bound<'_,PyAny>)->Read<bool> {
    if !x.is_exact_instance_of::<PyBool>() {return shape("exact bool required");}Ok(x.extract()?)
}
fn text(x:&Bound<'_,PyAny>)->Read<String> {
    if !x.is_exact_instance_of::<PyString>() {return shape("exact str required");}
    let s:String=x.extract()?;if s.len()>128 {return shape("wire tag too long");}Ok(s)
}
fn rational(x:&Bound<'_,PyAny>)->Read<Rat> {
    let t=fixed(x,2)?;let n=t.get_item(0)?;let d=t.get_item(1)?;
    for part in [&n,&d] {
        if !part.is_exact_instance_of::<PyInt>() {return shape("exact rat integer required (bool rejected)");}
        // Exact builtin int only: no subclass/host callback can replace bit_length.
        let bits:usize=part.call_method0("bit_length")?.extract()?;
        if bits>MAX_BITS {return shape("rational bit limit");}
    }
    let numerator=ibig_from_int(&n)?;let denominator=ibig_from_int(&d)?;
    if num::signum(&denominator)<=0 {return shape("positive rational denominator required");}
    Ok(Rat::reduced(numerator,num::magnitude(&denominator)))
}
fn vector(x:&Bound<'_,PyAny>)->Read<Vec2> {
    let t=fixed(x,2)?;Ok([rational(&t.get_item(0)?)?,rational(&t.get_item(1)?)?])
}
fn reference(x:&Bound<'_,PyAny>)->Read<Ref> {
    let t=fixed(x,3)?;Ok((int64(&t.get_item(0)?)?,int64(&t.get_item(1)?)?,int64(&t.get_item(2)?)?))
}
fn optional_ref(x:&Bound<'_,PyAny>)->Read<Option<Ref>> {if x.is_none(){Ok(None)}else{Ok(Some(reference(x)?))}}
fn ints(x:&Bound<'_,PyAny>)->Read<Vec<i64>> {tuple(x)?.iter().map(|x|int64(&x)).collect()}
fn item<'py>(d:&Bound<'py,PyDict>,name:&str)->Read<Bound<'py,PyAny>> {
    d.get_item(name)?.ok_or_else(||Fail::Shape(format!("missing {name}")))
}
fn read_patch(x:&Bound<'_,PyAny>)->Read<PatchInput> {
    if !x.is_exact_instance_of::<PyDict>() {return shape("exact patch dict required");}
    let d=x.cast::<PyDict>().map_err(|_|Fail::Shape("dict cast".into()))?;
    const KEYS:[&str;12]=["version","patch_id","owner_sign","gram","chart","band_or_ring_cut","points","chains","faces","sites","present_vertices","support_triangles"];
    if d.len()!=KEYS.len() {return shape("patch key cardinality");}
    for (key,_) in d.iter() {if !KEYS.contains(&text(&key)?.as_str()){return shape("unknown patch key");}}
    let version=u32::try_from(int64(&item(d,"version")?)?).map_err(|_|Fail::Shape("version range".into()))?;
    let patch_id=int64(&item(d,"patch_id")?)?;
    let owner=int64(&item(d,"owner_sign")?)?;
    if owner!=1 && owner!=-1 {return shape("owner orientation must be +/-1");}
    if text(&item(d,"chart")?)? != "RationalAffine" {return shape("first wire only admits RationalAffine");}
    let g=fixed(&item(d,"gram")?,2)?;
    let gram=[vector(&g.get_item(0)?)?,vector(&g.get_item(1)?)?];
    let mut points=Vec::new();
    for row in tuple(&item(d,"points")?)?.iter() {
        let row=fixed(&row,2)?;points.push(Point{id:int64(&row.get_item(0)?)?,xy:vector(&row.get_item(1)?)?});
    }
    let mut chains=Vec::new();
    for row in tuple(&item(d,"chains")?)?.iter() {
        let row=fixed(&row,3)?;chains.push(Chain{reference:reference(&row.get_item(0)?)?,vertices:ints(&row.get_item(1)?)?,edges:ints(&row.get_item(2)?)?});
    }
    let mut faces=Vec::new();
    for row in tuple(&item(d,"faces")?)?.iter() {
        let row=fixed(&row,4)?;faces.push(Face{id:int64(&row.get_item(0)?)?,vertices:ints(&row.get_item(1)?)?,edges:ints(&row.get_item(2)?)?,triangles:ints(&row.get_item(3)?)?});
    }
    let mut sites=Vec::new();
    for row in tuple(&item(d,"sites")?)?.iter() {
        let row=fixed(&row,5)?;sites.push(Site{key:reference(&row.get_item(0)?)?,cut:boolean(&row.get_item(1)?)?,anchor:int64(&row.get_item(2)?)?,incoming:optional_ref(&row.get_item(3)?)?,outgoing:optional_ref(&row.get_item(4)?)?});
    }
    let support=item(d,"support_triangles")?;
    Ok(PatchInput{version,patch_id,owner_sign:owner as i8,gram,chart:Chart::RationalAffine,
        band_or_ring_cut:boolean(&item(d,"band_or_ring_cut")?)?,points,chains,faces,sites,
        present_vertices:ints(&item(d,"present_vertices")?)?,
        support_triangles:if support.is_none(){None}else{Some(ints(&support)?)} })
}
fn rat_out<'py>(py:Python<'py>,q:&Rat)->PyResult<Bound<'py,PyAny>> {
    Ok(PyTuple::new(py,[int_of_ibig(py,q.numerator())?,int_of_ubig(py,q.denominator())?])?.into_any())
}
fn vec_out<'py>(py:Python<'py>,v:&Vec2)->PyResult<Bound<'py,PyAny>> {
    Ok(PyTuple::new(py,[rat_out(py,&v[0])?,rat_out(py,&v[1])?])?.into_any())
}
fn decision_out<'py>(py:Python<'py>,decision:&Decision)->PyResult<Bound<'py,PyAny>> {
    match decision {
        Decision::Skip(reason)=> {
            let reason=match reason {Skip::OffJunction=>"OffJunction",Skip::BeyondChartReach=>"BeyondChartReach",Skip::ForwardCollinear=>"ForwardCollinear",Skip::NonReflex=>"NonReflex"};
            ("SKIP",reason).into_bound_py_any(py)
        },
        Decision::Reflex(r)=> {
            let events=PyTuple::new(py,r.before_measure.iter().map(|x|match x {SymbolicEvent::Canonical=>"canonical",SymbolicEvent::Normalize=>"normalize"}))?;
            ("REFLEX",vec_out(py,&r.incoming_normal)?,vec_out(py,&r.outgoing_normal)?,r.turn_orientation,
                rat_out(py,&r.norm_left)?,rat_out(py,&r.norm_right)?,rat_out(py,&r.dot)?,rat_out(py,&r.owner_cross)?,rat_out(py,&r.gram_det)?,events).into_bound_py_any(py)
        }
    }
}
fn run<'py>(py:Python<'py>,patch:&Bound<'py,PyAny>)->PyResult<Bound<'py,PyAny>> {
    let input=match read_patch(patch) {
        Ok(input)=>input,Err(Fail::Py(error))=>return Err(error),
        Err(Fail::Shape(why))=>return (2u8,"ANGULAR_WIRE",why).into_bound_py_any(py),
    };
    let plan=match evaluate_patch(&input) {
        Ok(plan)=>plan,
        Err(decline)=>return (2u8,format!("ANGULAR_CORE_{:?}",decline.reason),format!("site={:?}",decline.site)).into_bound_py_any(py),
    };
    let mut output=Vec::with_capacity(plan.sites.len());
    for site in &plan.sites {
        let before=PyTuple::new(py,site.stage_before.iter().copied())?;
        let after=PyTuple::new(py,site.stage_after_success.iter().copied())?;
        output.push((site.ordinal,site.key,decision_out(py,&site.decision)?,before,after).into_bound_py_any(py)?);
    }
    (0u8,PyTuple::new(py,output)?).into_bound_py_any(py)
}
/// Pure single-patch entry; the future angular_compute shim owns whole-operation admission.
#[pyfunction]
pub fn angular_patch<'py>(py:Python<'py>,patch:&Bound<'py,PyAny>)->PyResult<Bound<'py,PyAny>> {
    match catch_unwind(AssertUnwindSafe(||run(py,patch))) {
        Ok(result)=>result,
        Err(panic)=>Err(PyRuntimeError::new_err(format!("angular: native panic: {}",crate::panic_message(&panic)))),
    }
}
