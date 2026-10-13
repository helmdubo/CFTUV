"""Opt-in test bridge: genuine raw rationals -> Python/Rust leaf facts, never AST import."""
import argparse
from dataclasses import replace
from fractions import Fraction as F
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import time
if hasattr(sys, "set_int_max_str_digits"):
    sys.set_int_max_str_digits(0)

def main():
    parser=argparse.ArgumentParser()
    parser.add_argument("--root",type=Path,required=True)
    parser.add_argument("--replay",type=Path,required=True)
    parser.add_argument("--corpus",type=Path)
    parser.add_argument("--out",type=Path,required=True)
    parser.add_argument("--oracle-root",type=Path)
    parser.add_argument("--tools-root",type=Path)
    parser.add_argument("--limit",type=int)
    args=parser.parse_args()
    oracle_root=args.oracle_root or args.root/"kernel/src"
    tools_root=args.tools_root or args.root/"tools"
    sys.path[:0]=[str(oracle_root),str(tools_root)]
    from cftuv_envelope.reference import density_oracle_v1 as facts
    from cftuv_envelope.reference.density_owned_laws_v1 import OwnedDensityLawsV1
    from cftuv_envelope.reference.density_construct_v1 import DensityConstructArenaV1,ExpandLimitsV1
    from density_raw_wire import read_raw_spec
    args.out.mkdir(parents=True,exist_ok=True)
    sources=[oracle_root/"cftuv_envelope/reference"/name for name in ("density_owned_laws_v1.py","density_construct_v1.py","density_scalar_v2.py","density_oracle_v1.py","density_angle_value_v1.py")]
    sources += [tools_root/"density_raw_wire.py",Path(__file__),args.replay]
    if args.corpus:sources.append(args.corpus)
    hashes={str(path):hashlib.sha256(path.read_bytes()).hexdigest() for path in sources}
    limits=facts.DensityOracleLimitsV1(65536,131072,96,1000000,ExpandLimitsV1(4096,16,65536))
    def error(exc):
        outcome=getattr(exc,"outcome",None)
        return {"exception_type":type(exc).__name__,"outcome":None if outcome is None else outcome.value,"message":str(exc)}
    def value_wire(laws,vector):
        return [list(laws.arena.scalar(v.value).value.wire()) for v in vector.expressions()]
    def direction(laws,vector):
        primitive,rational=laws.rational_direction(vector)
        return {"primitive":None if primitive is None else list(primitive),"rational":rational}
    def expected(raw,use,turn):
        laws=OwnedDensityLawsV1(DensityConstructArenaV1(),limits)
        try:metric,incoming,outgoing=laws.raw_geometry(raw,use_direction_declarations=use)
        except Exception as exc:return {"error":error(exc)}
        incoming_covector=laws.covector(metric,incoming);outgoing_covector=laws.covector(metric,outgoing)
        result={"incoming":value_wire(laws,incoming),"outgoing":value_wire(laws,outgoing),"incoming_covector":value_wire(laws,incoming_covector),"outgoing_covector":value_wire(laws,outgoing_covector),"incoming_direction":direction(laws,incoming),"outgoing_direction":direction(laws,outgoing)}
        try:
            sign,squared=laws.turn_witness(metric,incoming_covector,outgoing_covector,turn)
            counts=[[q,h,laws.turn_count_feasible(sign,squared,h,q),laws.turn_count_exact_limit(sign,squared,h,q),laws.turn_count_comparison(squared,h,q)] for q in range(2,7) for h in range(q)]
            result["turn"]={"dot_sign":sign,"cosine_squared":[squared.numerator,squared.denominator],"counts":counts}
        except Exception as exc:result["turn"]={"error":error(exc)}
        return result
    def text(value):return "-" if value is None else value.encode("utf-8").hex()
    def incident_wire(value):
        rows=[str(value.start.x),str(value.start.y),str(value.end.x),str(value.end.y),"N" if value.owner_left is None else "T" if value.owner_left else "F",text(value.chain_use_orientation)]
        rows += [text(getattr(value,key)) for key in ("anchor_vertex_id","source_vertex_start_id","source_vertex_end_id","support_id","physical_edge_id","chain_use_id")]
        records=dict(value.source_direction_declaration).get("records",())
        rows.append(str(len(records)))
        for declaration in records:
            record=dict(declaration);kind=record["kind"]
            if kind=="CERTIFIED_PLANAR_V1":rows += ["P",str(F(*record["x"])),str(F(*record["y"]))]
            elif kind=="CERTIFIED_AFFINE_V2":rows += ["A",str(F(*record["x"])),str(F(*record["y"])),text(record["reference_metric_id"])]
            else:rows += ["O",text(kind)]
        rows.append(str(len(value.owner_face_cycles)))
        for cycle in value.owner_face_cycles:
            rows.append(str(len(cycle)))
            for point in cycle:rows += [str(point.x),str(point.y)]
        rows += [str(len(value.owner_face_edge_ordinals)),*map(str,value.owner_face_edge_ordinals),str(len(value.owner_face_vertex_id_cycles))]
        for ids in value.owner_face_vertex_id_cycles:rows += [str(len(ids)),*map(text,ids)]
        rows += [str(len(value.owner_face_ids)),*map(text,value.owner_face_ids)]
        return rows
    def wire(raw,use,turn):
        m=raw.metric
        return "\t".join(["raw","1" if use else "0",str(turn),*map(str,m.gram),*map(str,m.inverse_gram),str(m.owner_orientation_sign),text(m.metric_id),*incident_wire(raw.incoming),*incident_wire(raw.outgoing)])
    def point(x,y):return facts.RawDensityPointV1(F(x),F(y))
    def declarations(*rows):return (("records",tuple(tuple(row.items()) for row in rows)),)
    def synthetics():
        for skew in (False,True):
            for owner in (-1,1):
                metric=facts.RawDensityMetricV1((F(2),F(1,2),F(1)) if skew else (F(1),F(0),F(1)),(F(4,7),F(-2,7),F(8,7)) if skew else (F(1),F(0),F(1)),owner,"metric")
                incoming=facts.RawDensityIncidentV1(point(0,0),point(1,0),True,physical_edge_id="edge",chain_use_id="use")
                outgoing=replace(incoming,end=point(0,1))
                base=facts.RawDensityGeometryV1(metric,incoming,outgoing)
                yield "base",base,True,owner
                faces=replace(incoming,owner_left=None,owner_face_cycles=((point(0,0),point(1,0),point(1,1),point(0,1)),),owner_face_edge_ordinals=(0,))
                yield "face",replace(base,incoming=faces),True,owner
                tangent=replace(incoming,source_direction_declaration=declarations({"kind":"CERTIFIED_PLANAR_V1","x":(1,1),"y":(0,1)}))
                yield "tangent",replace(base,incoming=tangent),True,owner
                normal=replace(incoming,owner_left=None,source_direction_declaration=declarations({"kind":"CERTIFIED_PLANAR_V1","x":(-1 if skew else 0,1),"y":(4 if skew else 1,1)}))
                yield "normal",replace(base,incoming=normal),True,owner
                wrong=replace(incoming,source_direction_declaration=declarations({"kind":"CERTIFIED_AFFINE_V2","x":(0,1),"y":(1,1),"reference_metric_id":"other"}))
                yield "affine_metric",replace(base,incoming=wrong),True,owner
                yield "disabled_affine",replace(base,incoming=wrong),False,owner
                conflict=replace(incoming,source_direction_declaration=declarations({"kind":"CERTIFIED_PLANAR_V1","x":(0,1),"y":(1,1)},{"kind":"CERTIFIED_PLANAR_V1","x":(0,1),"y":(-1,1)}))
                yield "conflict",replace(base,incoming=conflict),True,owner
                neither=replace(incoming,source_direction_declaration=declarations({"kind":"CERTIFIED_PLANAR_V1","x":(1,1),"y":(1,1)}))
                yield "neither",replace(base,incoming=neither),True,owner
                ambiguous=replace(incoming,owner_left=None)
                yield "ambiguous",replace(base,incoming=ambiguous),True,owner
                yield "orientation",replace(base,incoming=replace(ambiguous,chain_use_orientation="A_START_TO_END")),True,owner
                yield "face_no_ordinals",replace(base,incoming=replace(faces,owner_face_edge_ordinals=())),True,owner
                yield "invalid_gram",replace(base,metric=replace(metric,inverse_gram=(F(2),metric.inverse_gram[1],metric.inverse_gram[2]))),True,owner
    def inputs():
        yield from synthetics()
        if args.corpus:
            with args.corpus.open(encoding="utf-8") as stream:
                for compilation_index,line in enumerate(stream):
                    for spec_index,data in enumerate(json.loads(line).get("raw",())):
                        spec=read_raw_spec(data)
                        present=dict(spec.canonical.noise_policy_declaration)["binding_present"]
                        yield f"field/{compilation_index}/{spec_index}/source",spec.source_geometry,True,spec.turn_orientation_sign
                        yield f"field/{compilation_index}/{spec_index}/evaluation",spec.evaluation_geometry,not present,spec.turn_orientation_sign
    started=time.time();counts={"records":0,"equal":0,"mismatch":0}
    kwargs={"creationflags":subprocess.CREATE_NO_WINDOW} if os.name=="nt" else {}
    process=subprocess.Popen([str(args.replay)],stdin=subprocess.PIPE,stdout=subprocess.PIPE,text=True,encoding="utf-8",**kwargs)
    (args.out/"running.json").write_text(json.dumps({"python_pid":os.getpid(),"replay_pid":process.pid}),encoding="utf-8")
    try:
        with (args.out/"records.jsonl").open("w",encoding="utf-8") as output:
            for index,(name,raw,use,turn) in enumerate(inputs()):
                if args.limit is not None and index>=args.limit:break
                if any(shutil.disk_usage(drive).free<1500000000 for drive in ("C:/","E:/")):raise RuntimeError("DISK_BELOW_1_5GB")
                line=wire(raw,use,turn);process.stdin.write(line+"\n");process.stdin.flush()
                actual=json.loads(process.stdout.readline());oracle=expected(raw,use,turn)
                different=sorted(key for key in actual.keys()|oracle.keys() if actual.get(key)!=oracle.get(key))
                counts["records"]+=1;counts["mismatch" if different else "equal"]+=1
                output.write(json.dumps({"index":index,"name":name,"different":different,"wire":line,"python":oracle,"rust":actual})+"\n");output.flush()
                if index%20==0:
                    (args.out/"progress.json").write_text(json.dumps(counts),encoding="utf-8");print(json.dumps(counts),flush=True)
    finally:process.stdin.close();process.wait(timeout=10)
    changed=[str(path) for path in sources if hashlib.sha256(path.read_bytes()).hexdigest()!=hashes[str(path)]]
    receipt={"status":"ACCEPTED_RAW_LEAF_BOUNDED" if not counts["mismatch"] and not changed else "FAILED_RAW_LEAF_DIFFERENTIAL","scope":"Raw typed geometry/normals V2 value wires, primal/covectors, directions, turn and count gates q2..6; no principal, bind, noise or whole-operation acceptance","counts":counts,"source_hashes":hashes,"source_changed":changed,"elapsed_seconds":time.time()-started,"python_pid":os.getpid(),"replay_pid":process.pid}
    (args.out/"receipt.json").write_text(json.dumps(receipt,indent=2)+"\n",encoding="utf-8");print(json.dumps(receipt),flush=True)
    return bool(counts["mismatch"] or changed)
if __name__=="__main__":raise SystemExit(main())
