use crate::model::*;
use cftuv_core::rat::Rat;

pub(crate) fn sub(a: &Vec2, b: &Vec2) -> Vec2 { [a[0].sub(&b[0]), a[1].sub(&b[1])] }
pub(crate) fn dot(p: &PatchInput, a: &Vec2, b: &Vec2) -> Rat {
    a[0].mul(&p.gram[0][0].mul(&b[0]).add(&p.gram[0][1].mul(&b[1])))
        .add(&a[1].mul(&p.gram[1][0].mul(&b[0]).add(&p.gram[1][1].mul(&b[1]))))
}
pub(crate) fn cross(p: &PatchInput, a: &Vec2, b: &Vec2) -> Rat {
    a[0].mul(&b[1]).sub(&a[1].mul(&b[0])).mul(&Rat::from_i64(p.owner_sign as i64))
}
fn point(p: &PatchInput, id: i64) -> Result<&Vec2, Reason> {
    p.points.iter().find(|x| x.id == id).map(|x| &x.xy).ok_or(Reason::MissingPoint)
}
pub(crate) enum Support { Ready(Vec2, Vec2), Beyond }
pub(crate) fn support(p: &PatchInput, reference: Ref, anchor: i64) -> Result<Support, Reason> {
    let chain = p.chains.iter().find(|c| c.reference == reference).ok_or(Reason::MissingChain)?;
    let n = chain.vertices.len();
    let (a,b,e) = if anchor == chain.vertices[0] {
        (chain.vertices[0],chain.vertices[1],chain.edges[0])
    } else if anchor == chain.vertices[n-1] {
        (chain.vertices[n-2],chain.vertices[n-1],chain.edges[n-2])
    } else { return Err(Reason::NotEndpoint); };
    let tangent = sub(point(p,b)?, point(p,a)?);
    if dot(p,&tangent,&tangent).is_zero() { return Err(Reason::DegenerateSupport); }
    let mut candidates: Vec<Vec2> = Vec::new();
    for face in &p.faces {
        let Some(ordinal) = face.edges.iter().position(|id| *id == e) else { continue; };
        if let Some(charted) = &p.support_triangles {
            if !face.triangles.iter().any(|id| charted.contains(id)) { return Ok(Support::Beyond); }
        }
        let cycle = face.vertices.iter().map(|id| point(p,*id)).collect::<Result<Vec<_>,_>>()?;
        let mut area = Rat::zero();
        for i in 0..cycle.len() {
            let a=cycle[i]; let b=cycle[(i+1)%cycle.len()];
            area=area.add(&a[0].mul(&b[1]).sub(&a[1].mul(&b[0])));
        }
        if area.is_zero() { continue; }
        let direction=sub(cycle[(ordinal+1)%cycle.len()],cycle[ordinal]);
        let same=dot(p,&tangent,&direction).signum()>0;
        let coordinate_left=(area.signum()>0)==same;
        let owner_left=coordinate_left==(p.owner_sign>0);
        let x=p.gram[0][0].mul(&tangent[0]).add(&p.gram[0][1].mul(&tangent[1]));
        let y=p.gram[1][0].mul(&tangent[0]).add(&p.gram[1][1].mul(&tangent[1]));
        let s=Rat::from_i64(if owner_left {p.owner_sign as i64} else {-(p.owner_sign as i64)});
        let normal=[y.neg().mul(&s), x.mul(&s)];
        candidates.push(normal);
    }
    // Как в oracle: сначала весь scan (поздний Beyond имеет приоритет),
    // только затем сравнение направлений кандидатов.
    let first=candidates.first().ok_or(Reason::NoOwnerFace)?;
    for item in &candidates[1..] {
        if !cross(p,first,item).is_zero() || dot(p,first,item).signum()<=0 {
            return Err(Reason::OwnerDirectionsDisagree);
        }
    }
    Ok(Support::Ready(first.clone(),tangent))
}
