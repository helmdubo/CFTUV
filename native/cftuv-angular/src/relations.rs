use std::collections::BTreeSet;
use cftuv_core::{num, rat::Rat};
use crate::{model::*, owner::{self,Support}};

fn square(q: &Rat) -> bool {
    if q.signum()<0 { return false; }
    let n=num::magnitude(q.numerator()); let d=q.denominator();
    let a=num::isqrt(&n); let b=num::isqrt(d);
    &a*&a==n && &b*&b==*d
}
fn prices(n: &Vec2,m: &Vec2,a: &Rat,b: &Rat,d: &Rat,c: &Rat,t: &Rat)->Vec<SymbolicEvent> {
    let mut out=Vec::new();
    for (v,q) in [(n,a),(m,b)] {
        if !square(q) { for x in v { if !x.is_zero() {out.push(SymbolicEvent::Canonical);} } }
    }
    let ab=a.mul(b);
    if !d.is_zero() && !square(&ab) {out.push(SymbolicEvent::Normalize);}
    if !c.is_zero() && !square(&ab) {out.push(SymbolicEvent::Normalize);}
    if !c.is_zero() && !square(&t.div(&ab).expect("SPD nonzero norms")) {out.push(SymbolicEvent::Normalize);}
    out
}
fn admission(p: &PatchInput)->Result<Rat,Reason> {
    if p.version!=1 {return Err(Reason::Version);}
    if p.chart==Chart::Unsupported {return Err(Reason::Chart);}
    if p.band_or_ring_cut {return Err(Reason::BandOrRingCut);}
    let g=&p.gram;
    let det=g[0][0].mul(&g[1][1]).sub(&g[0][1].mul(&g[1][0]));
    if ![-1,1].contains(&p.owner_sign) || g[0][1]!=g[1][0] || g[0][0].signum()<=0 || det.signum()<=0 {
        return Err(Reason::InvalidMetric);
    }
    let unique=|v:Vec<i64>| v.iter().copied().collect::<BTreeSet<_>>().len()==v.len();
    if !unique(p.points.iter().map(|x|x.id).collect()) || !unique(p.faces.iter().map(|x|x.id).collect())
        || p.chains.iter().map(|x|x.reference).collect::<BTreeSet<_>>().len()!=p.chains.len() {
        return Err(Reason::DuplicateId);
    }
    if p.chains.iter().any(|x|x.reference.0!=p.patch_id || x.vertices.len()<2 || x.edges.len()+1!=x.vertices.len())
        || p.faces.iter().any(|x|x.vertices.len()<3 || x.vertices.len()!=x.edges.len()) {
        return Err(Reason::InvalidTopology);
    }
    Ok(det)
}
pub fn evaluate_patch(p: &PatchInput)->Result<PatchPlan,Decline> {
    let det=admission(p).map_err(|reason|Decline{site:None,reason})?;
    let mut result=PatchPlan{sites:Vec::new()};
    for (ordinal,s) in p.sites.iter().enumerate() {
        let fail=|reason|Decline{site:Some(ordinal),reason};
        let mut before=vec![if s.cut {"ANGULAR_CUT_VERTICES_CONSIDERED"} else {"ANGULAR_CORNERS_CONSIDERED"}];
        let decision=if s.incoming.is_none() {
            before.push("ANGULAR_CORNERS_OFF_JUNCTION"); Decision::Skip(Skip::OffJunction)
        } else {
            if !p.present_vertices.contains(&s.anchor) {return Err(fail(Reason::MissingAnchor));}
            let incoming=owner::support(p,s.incoming.unwrap(),s.anchor).map_err(fail)?;
            // Исходник не вычисляет вторую опору, если первая вне карты.
            let supports=match incoming {
                Support::Beyond=>None,
                Support::Ready(n,v)=>match owner::support(p,s.outgoing.ok_or_else(||fail(Reason::MissingChain))?,s.anchor).map_err(fail)? {
                    Support::Beyond=>None, Support::Ready(m,w)=>Some((n,v,m,w)),
                },
            };
            if let Some((n,v,m,w))=supports {
                let turn=owner::cross(p,&v,&w);
                if turn.is_zero() {
                    if owner::dot(p,&v,&w).signum()<=0 {return Err(fail(Reason::FullTurn));}
                    before.push("ANGULAR_COLLINEAR_SKIPPED");
                    if s.cut {before.push("ANGULAR_CUT_CHART_COLLINEAR");}
                    Decision::Skip(Skip::ForwardCollinear)
                } else {
                    let l=owner::cross(p,&v,&n); let r=owner::cross(p,&w,&m);
                    if l.is_zero() || r.is_zero() || l.signum()!=r.signum() {return Err(fail(Reason::InteriorSidesDisagree));}
                    if turn.signum()==l.signum() {Decision::Skip(Skip::NonReflex)} else {
                        before.push("ANGULAR_REFLEX_CORNERS");
                        if s.cut {before.push("ANGULAR_CUT_REFLEX_RELATIONS");}
                        let a=owner::dot(p,&n,&n); let b=owner::dot(p,&m,&m);
                        let d=owner::dot(p,&n,&m); let c=owner::cross(p,&n,&m);
                        let events=prices(&n,&m,&a,&b,&d,&c,&det);
                        Decision::Reflex(Reflex{incoming_normal:n,outgoing_normal:m,
                            turn_orientation:turn.signum(),norm_left:a,norm_right:b,
                            dot:d,owner_cross:c,gram_det:det.clone(),before_measure:events})
                    }
                }
            } else {before.push("ANGULAR_CORNERS_BEYOND_CHART_REACH");Decision::Skip(Skip::BeyondChartReach)}
        };
        let after=if matches!(&decision,Decision::Reflex(_)) {vec!["ANGULAR_RELATIONS_BUILT"]} else {vec![]};
        result.sites.push(SitePlan{ordinal,key:s.key,decision,stage_before:before,stage_after_success:after});
    }
    Ok(result)
}
