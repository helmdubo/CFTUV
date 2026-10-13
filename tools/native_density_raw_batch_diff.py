"""Typed pure batch A/B; исходные compile scope и ordinal order сохраняются."""
from __future__ import annotations
import argparse
from collections import OrderedDict
from dataclasses import replace
from fractions import Fraction
import hashlib
import json
import lzma
import os
from pathlib import Path
import sys
import time

sys.set_int_max_str_digits(0)
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "kernel/src"))
sys.path.insert(0, str(ROOT / "tools"))
from density_raw_wire import read_raw_spec
from density_raw_compile_audit import raw_wire
from density_raw_compile_diff import CANONS, _stored_result, limits_for
from density_shell_compile_diff import exact_tree, tree_deltas
from cftuv_envelope.reference import density_result_v1 as formatter
from cftuv_envelope.reference.density_construct_v1 import ExpandLimitsV1
from cftuv_envelope.reference.density_oracle_v1 import RawDensityBatchInputV1, DensityOracleLimitsV1
from cftuv_envelope.reference.density_raw_frontend_v1 import evaluate_raw_density_batch
from cftuv_envelope.ids import EnvelopeSpecId, SelectionCertificateId
from cftuv_envelope.reference import density_oracle_v1 as facts

def skew_undecidable_batch():
    point=lambda x,y:facts.RawDensityPointV1(Fraction(x),Fraction(y))
    incident=lambda x,y:facts.RawDensityIncidentV1(point(0,0),point(x,y),True)
    metric=facts.RawDensityMetricV1((Fraction(2),Fraction(1,2),Fraction(1)),(Fraction(4,7),Fraction(-2,7),Fraction(8,7)),1,"metric")
    geometry=facts.RawDensityGeometryV1(metric,incident(1,0),incident(0,1))
    selection=facts.RawDensitySelectionV1(5,3,"HUBER_EMANATED_COUNT_DENSITY_A_V1","LINEAR_REFLEX_DENSITY_3_V1",4,None)
    canonical=facts.RawDensityCanonicalFactsV1((Fraction(7,10),Fraction(7,10)),None,(),(("binding_present",False),),())
    raw=facts.RawDensitySpecInput(geometry,geometry,1,selection,canonical,"a")
    caps=DensityOracleLimitsV1(4096,4096,4096,1000000,ExpandLimitsV1(4096,16,65536))
    return RawDensityBatchInputV1((raw,),CANONS,caps,"VORONOI_OF_NEIGHBOUR_MIDPOINTS")

def source_hashes():
    paths=list((ROOT/"kernel/src/cftuv_envelope").rglob("*.py"))
    paths.extend((ROOT/"tools"/name) for name in ("native_density_raw_batch_diff.py","density_raw_wire.py","density_raw_compile_audit.py","density_raw_compile_diff.py","density_shell_compile_diff.py"))
    return {str(path):hashlib.sha256(path.read_bytes()).hexdigest() for path in paths}

def observed(result, raw):
    # Optional native ID — внутренний carrier; общий formatter проверяет/строит final ID.
    normalized = replace(result, rust_work_diagnostic=(), authority=None if result.authority is None else replace(result.authority, authority_id=None))
    sid = SelectionCertificateId(raw.selection.selection_certificate_id or "synthetic-selection")
    specid = EnvelopeSpecId(raw.spec_id)
    return {"facts": raw_wire(normalized), "formatted": exact_tree({
        "authority": formatter.format_authority(result.authority, sid),
        "certificates": tuple(formatter.format_certificate(item) for item in result.binding_certificates),
        "lift": formatter.format_count_lift(result.count_lift, sid),
        "noise": formatter.format_noise_record(result.noise_record, specid, sid),
        "subturn": formatter.format_canonical_subturn_authority(result.canonical_subturn_authority, specid, sid),
    })}

def main():
    parser=argparse.ArgumentParser()
    selection=parser.add_mutually_exclusive_group(required=True)
    selection.add_argument("--eligible",type=Path)
    selection.add_argument("--accepted-records",type=Path)
    parser.add_argument("--package",type=Path,required=True)
    parser.add_argument("--output",type=Path,required=True)
    parser.add_argument("--limit",type=int)
    parser.add_argument("--indices",help="Explicit comma-separated original batch indices")
    parser.add_argument("--window-law",default="NARROW_ROTATION_BAND")
    parser.add_argument("--candidate",action="store_true",help="Explicit interim candidate entrypoint; default uses public pinned density_compute")
    args=parser.parse_args()
    selected=None if args.indices is None else {int(value) for value in args.indices.split(",")}
    sys.path.insert(0,str(args.package))
    import cftuv_native
    operation=cftuv_native.density_compute_candidate if args.candidate else cftuv_native.density_compute
    entrypoint="density_compute_candidate" if args.candidate else "density_compute"
    guard_before=source_hashes()
    groups=OrderedDict()
    seed=args.eligible or args.accepted_records
    for index,line in enumerate(seed.read_text(encoding="utf-8").splitlines()):
        item=json.loads(line)
        if args.eligible:
            groups.setdefault((item["source"],item["compile_row"]),[]).append(item)
        else:
            path=seed.parent/"blobs"/(item["after"]["typed_facts_sha256"]+".json.xz")
            blob=lzma.decompress(path.read_bytes())
            if hashlib.sha256(blob).hexdigest()!=item["after"]["typed_facts_sha256"]:raise ValueError("accepted typed facts blob SHA mismatch")
            facts=json.loads(blob)
            groups[(str(seed),index)]={"settings":item["settings"],"facts":facts}
    args.output.mkdir(parents=True,exist_ok=False)
    binaries=list((args.package/"cftuv_native").glob("_core*.pyd"))
    if len(binaries)!=1:raise ValueError("one native binary required")
    binary=binaries[0]
    binary_sha=hashlib.sha256(binary.read_bytes()).hexdigest()
    (args.output/"running.json").write_text(json.dumps({"pid":os.getpid(),"seed":str(seed),"native_binary_sha256":binary_sha,"native_entrypoint":entrypoint}),encoding="utf-8")
    started=time.monotonic();passed=failed=0
    with (args.output/"records.jsonl").open("w",encoding="utf-8") as output:
        for index,(key,items) in enumerate(groups.items()):
            if args.limit is not None and index>=args.limit:break
            if selected is not None and index not in selected:continue
            if args.eligible:
                items.sort(key=lambda item:item["raw_ordinal"])
                settings=items[0]["settings"]
                if any(item["settings"]!=settings for item in items):raise ValueError("batch settings disagree")
                raws=[item["raw"] for item in items]
                expected=None
                caps=DensityOracleLimitsV1(settings["preparation_cap"],settings["search_cap"],settings["box_cap"],settings["height_cap"],ExpandLimitsV1(4096,16,65536))
            else:
                settings=items["settings"];raws=[item["input"] for item in items["facts"]]
                expected=tuple(_stored_result(item["result"]) for item in items["facts"])
                caps=limits_for(settings)
            specs=tuple(read_raw_spec(raw) for raw in raws)
            batch=RawDensityBatchInputV1(specs,CANONS,caps,args.window_law)
            record={"index":index,"source":key[0],"compile_row":key[1],"settings":settings,"raw":raws}
            if args.eligible:
                record.update(raw_ordinals=[item["raw_ordinal"] for item in items],memo_supplied=[item["memo_supplied"] for item in items])
                record["provenance"]=[{key:value for key,value in item.items() if key not in ("raw","settings")} for item in items]
            try:
                python=evaluate_raw_density_batch(batch) if expected is None else expected
                native=operation(batch)
                if len(native)>len(specs):raise ValueError("native returned more results than input specs")
                record["python"]=[raw_wire(item) for item in python];record["native"]=[raw_wire(item) for item in native]
                record["differences"]=tree_deltas([observed(item,raw) for item,raw in zip(python,specs)],[observed(item,raw) for item,raw in zip(native,specs)])
            except Exception as error:
                record["error"]={"type":type(error).__name__,"message":str(error)};record["differences"]=[{"path":"batch.exception"}]
                for name in ("completed_results","spec_index","partial_effects"):
                    if hasattr(error,name):record["error"][name]=raw_wire(getattr(error,name))
                if "python" not in record:
                    # Python input/profile rejection не отменяет независимый native вызов.
                    try:
                        record["native"]=raw_wire(operation(batch))
                    except Exception as native_error:
                        record["native_error"]={"type":type(native_error).__name__,"message":str(native_error)}
                        for name in ("completed_results","spec_index","partial_effects"):
                            if hasattr(native_error,name):record["native_error"][name]=raw_wire(getattr(native_error,name))
            if record["differences"]:failed+=1
            else:passed+=1
            output.write(json.dumps(record,sort_keys=True)+"\n");output.flush()
            (args.output/"progress.json").write_text(json.dumps({"processed":passed+failed,"passed":passed,"failed":failed,"elapsed_seconds":time.monotonic()-started}),encoding="utf-8")
    guard_after=source_hashes()
    changed=[path for path in sorted(set(guard_before)|set(guard_after)) if guard_before.get(path)!=guard_after.get(path)]
    receipt={"state":"PASS_SCOPED" if not failed else "FAIL","passed":passed,"failed":failed,"elapsed_seconds":time.monotonic()-started,"seed_sha256":hashlib.sha256(seed.read_bytes()).hexdigest(),"native_binary_sha256":binary_sha,"binary_changed":hashlib.sha256(binary.read_bytes()).hexdigest()!=binary_sha,"source_changed":changed,"source_sha256":guard_before,"window_law":args.window_law,"oracle":"accepted grouped source results" if args.accepted_records else "fresh Python raw batch","scope":"fresh pure batch only; caller cross-domain memo not supplied"}
    receipt["native_entrypoint"]=entrypoint
    receipt["native_build_parts"]=cftuv_native.native_build_parts()
    if receipt["binary_changed"]:receipt["state"]="FAIL_BINARY_CHANGED"
    if changed:receipt["state"]="FAIL_SOURCE_CHANGED"
    (args.output/"receipt.json").write_text(json.dumps(receipt,indent=2),encoding="utf-8")
    print(json.dumps({key:value for key,value in receipt.items() if key!="source_sha256"}));return receipt["state"]!="PASS_SCOPED"

if __name__=="__main__":raise SystemExit(main())
