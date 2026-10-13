"""Portable inputs-only raw batch seed; live oracle, без сохранённых AST/ответов."""
from __future__ import annotations
import hashlib
import json
import lzma
from pathlib import Path
import sys
from dataclasses import replace
from fractions import Fraction

sys.set_int_max_str_digits(0)
ROOT=Path(__file__).resolve().parents[1]
for directory in (ROOT/'kernel/src',ROOT/'tools'):
    if str(directory) not in sys.path:sys.path.insert(0,str(directory))
from cftuv_envelope.reference import density_oracle_v1 as facts
from cftuv_envelope.reference.density_construct_v1 import ExpandLimitsV1
from cftuv_envelope.reference.density_raw_frontend_v1 import evaluate_raw_density_batch
from density_raw_wire import read_raw_spec
from density_raw_compile_audit import raw_wire
from density_raw_compile_diff import CANONS, limits_for
from native_density_raw_batch_diff import observed
from density_shell_compile_diff import tree_deltas

SCHEMA='cftuv.native.density_batch.inputs.v1'
LAWS=('NARROW_ROTATION_BAND','VORONOI_OF_NEIGHBOUR_MIDPOINTS')
SEED=ROOT/'tests/data/native_density_batch_inputs.json.xz'
MANIFEST=SEED.with_name('native_density_batch_inputs.manifest.json')
ASSERTIONS={'synthetic-warm-success':'warm-success','synthetic-repeat-failed-band':'repeat-failed-band','synthetic-final-failure':'final-failure','synthetic-skew-undecidable':'undecidable'}

def packed(value):return json.dumps(value,sort_keys=True,separators=(',',':'),ensure_ascii=True).encode('ascii')
def digest(value):return hashlib.sha256(packed(value)).hexdigest()

def read_seed(path=SEED,*,_require_host_high_angle=True):
    path=Path(path);manifest=json.loads(path.with_name('native_density_batch_inputs.manifest.json').read_text())
    encoded=path.read_bytes();decoded=lzma.decompress(encoded)
    if hashlib.sha256(encoded).hexdigest()!=manifest['compressed_sha256'] or hashlib.sha256(decoded).hexdigest()!=manifest['payload_sha256']:
        raise ValueError('DENSITY_BATCH_SEED_SHA_MISMATCH')
    document=json.loads(decoded)
    if set(document)!={'schema','cases'} or document['schema']!=SCHEMA:raise ValueError('DENSITY_BATCH_SEED_SCHEMA')
    identities=[]
    for case in document['cases']:
        if set(case)!={'id','specs','limits','provenance','input_sha256'}:raise ValueError('DENSITY_BATCH_SEED_CASE_FIELDS')
        if digest({'specs':case['specs'],'limits':case['limits']})!=case['input_sha256']:raise ValueError('DENSITY_BATCH_CASE_SHA_MISMATCH')
        for law in LAWS:batch_for(case,law)
        identities.append(case['id'])
    if len(set(identities))!=len(identities) or len(identities)!=manifest['cases']:raise ValueError('DENSITY_BATCH_SEED_CARDINALITY')
    raws=[raw for case in document['cases'] for raw in case['specs']]
    if {raw['selection']['q'] for raw in raws}!={2,3,4,5,6}:raise ValueError('DENSITY_BATCH_SEED_Q_COVERAGE')
    if not any(raw['selection']['source_hidden_count']==0 for raw in raws):raise ValueError('DENSITY_BATCH_SEED_ZERO_MISSING')
    if {raw['turn_orientation_sign'] for raw in raws}!={-1,1} or {raw['source_geometry']['metric']['owner_orientation_sign'] for raw in raws}!={-1,1}:raise ValueError('DENSITY_BATCH_SEED_ORIENTATION_COVERAGE')
    kinds={case['provenance']['kind'] for case in document['cases']}
    required={'UNPATCHED_KERNEL_RAW_INPUT','FIELD_RAW_INPUT_ONLY','SYNTHETIC_RAW_RATIONAL_DECLARATIONS','SYNTHETIC_RAW_ZERO_COUNT','EXPLICIT_MALFORMED_SELECTION_REGRESSION'}
    if not required<=kinds or not (set(ASSERTIONS)|{'synthetic-malformed-selection'})<=set(identities):raise ValueError('DENSITY_BATCH_SEED_REQUIRED_CASE_MISSING')
    required_field={'grid-straw:1657/5120','grid-straw:3647/5120','multispec','successful-memo-reuse','noise-record','count-lift'}
    if not required_field<=set(manifest['field_coverage']):raise ValueError('DENSITY_BATCH_SEED_FIELD_COVERAGE')
    if not {item['id'] for item in manifest['field_selection']}<=set(identities):raise ValueError('DENSITY_BATCH_SEED_FIELD_SELECTION_ID')
    if _require_host_high_angle:
        high=[case for case in document['cases'] if case['provenance']['kind']=='HOST_FLOAT_HIGH_ANGLE_RAW_INPUT']
        if (len(high)!=5 or {case['provenance']['q'] for case in high}!=set(range(2,7))
                or {case['id'] for case in high}!={f'host-float-high-angle-q{q}' for q in range(2,7)}
                or manifest.get('host_high_angle_q_coverage')!=list(range(2,7))):
            raise ValueError('DENSITY_BATCH_SEED_HOST_HIGH_ANGLE_COVERAGE')
    return document['cases']

def batch_for(case,law):
    if law not in LAWS:raise ValueError('DENSITY_BATCH_WINDOW_LAW')
    caps=case['limits'];profile=caps['finite_expand']
    limits=facts.DensityOracleLimitsV1(caps['preparation_cap'],caps['search_cap'],caps['box_cap'],caps['height_cap'],ExpandLimitsV1(**profile))
    return facts.RawDensityBatchInputV1(tuple(read_raw_spec(raw) for raw in case['specs']),CANONS,limits,law)

def compare_case(case,law,compute):
    batch=batch_for(case,law)
    if case['id']=='synthetic-malformed-selection':
        try:evaluate_raw_density_batch(batch)
        except ValueError as error:
            if str(error)!='DENSITY_RAW_SELECTION_BUCKET_UNPROVEN':raise AssertionError(str(error)) from error
        else:raise AssertionError('Python accepted malformed selection declaration')
        try:compute(batch)
        except RuntimeError as error:
            assert type(error).__name__=='DensityNativeInternalErrorV1'
            assert str(error)=='density_compute: internal/profile failure: Raw(Contract(SelectionBucketUnproven))'
            assert error.spec_index==0 and error.completed_results==()
            effects=error.partial_effects
            assert effects.failure is None and effects.named_refusals==()
            assert effects.ordered_work_articles==() and effects.shell_events==()
            return {'rejection':'internal/profile','differences':[]}
        raise AssertionError('native accepted malformed selection or named it geometric')
    python=evaluate_raw_density_batch(batch);native=compute(batch)
    assert len(native)<=len(batch.specs)
    before=[observed(result,raw) for result,raw in zip(python,batch.specs)]
    after=[observed(result,raw) for result,raw in zip(native,batch.specs)]
    differences=tree_deltas(before,after)
    assert not differences,differences[:10]
    if case['provenance']['kind']=='FIELD_RAW_INPUT_ONLY' and law==LAWS[0]:
        metadata=json.loads(MANIFEST.read_text())
        selected=next(item['selected_for'] for item in metadata['field_selection'] if item['id']==case['id']) if any(item['id']==case['id'] for item in metadata['field_selection']) else ()
        observed_ratios={(abs(event.rational_numerator),event.rational_denominator) for result in python for event in result.shell_events if event.rational_numerator is not None}
        for label in selected:
            if label.startswith('grid-straw:'):
                numerator,denominator=map(int,label.split(':')[1].split('/'))
                assert (numerator,denominator) in observed_ratios
    assertion=ASSERTIONS.get(case['id'])
    if case['provenance']['kind']=='HOST_FLOAT_HIGH_ANGLE_RAW_INPUT':
        assert native and all(result.failure is None and result.outcome=='OK' for result in native), 'ordinary accepted host float carrier must remain successful'
        assert {raw.selection.q for raw in batch.specs}=={case['provenance']['q']}
    if assertion=='warm-success' and law==LAWS[1]:
        assert len(native)==2 and native[0].ordered_work_articles and not native[1].ordered_work_articles
    elif assertion=='repeat-failed-band' and law==LAWS[0]:
        assert len(native)==2 and native[0].named_refusals and native[1].named_refusals
        assert native[0].named_refusals[0][1]==native[1].named_refusals[0][1]
        assert tuple(article.phase for article in native[0].ordered_work_articles)==('PREPARATION','SEARCH','PREPARATION','PREPARATION','SEARCH')
        assert tuple(article.phase for article in native[1].ordered_work_articles)==('PREPARATION','SEARCH')
        assert native[1].ordered_work_articles==native[0].ordered_work_articles[:2]
    elif assertion=='final-failure':
        assert native[-1].failure is not None and not native[-1].named_refusals
        assert native[-1].ordered_work_articles
    elif assertion=='undecidable' and law==LAWS[1]:
        assert native[-1].failure.exception_type=='ReferenceGeometryError'
        assert native[-1].failure.outcome=='REFERENCE_CERTIFIED_PREDICATE_UNDECIDABLE'
    return {'results':len(native),'differences':[]}

def case_of(identity,specs,limits,provenance):
    inputs={'specs':raw_wire(tuple(specs)),'limits':raw_wire(limits)}
    return {'id':identity,**inputs,'provenance':provenance,'input_sha256':digest(inputs)}

def append_host_high_angle(inputs,path=SEED):
    """Добавляет реальные host-extracted inputs; сохранённые ответы не входят в seed."""
    path=Path(path);inputs=Path(inputs)
    cases=read_seed(path,_require_host_high_angle=False)
    document=json.loads(inputs.read_text())
    if document.get('schema')!='density_high_angle_actual_host_raw_v1' or document.get('source_changed')!=[]:
        raise ValueError('DENSITY_HOST_HIGH_ANGLE_EXPORT_INVALID')
    if len(document['cases'])!=5 or {case['q'] for case in document['cases']}!=set(range(2,7)):
        raise ValueError('DENSITY_HOST_HIGH_ANGLE_Q_COVERAGE')
    for relative,expected in document['source_hashes'].items():
        source=ROOT/relative.replace('\\','/')
        if hashlib.sha256(source.read_bytes()).hexdigest()!=expected:raise ValueError('DENSITY_HOST_HIGH_ANGLE_SOURCE_CHANGED')
    source_payload_sha=hashlib.sha256(inputs.read_bytes()).hexdigest()
    added=[]
    for exported in sorted(document['cases'],key=lambda case:case['q']):
        q=exported['q'];batch=exported['batch'];caps=batch['limits']
        if batch['canon_ids']!=raw_wire(CANONS):raise ValueError('DENSITY_HOST_HIGH_ANGLE_CANON_MISMATCH')
        provenance={'kind':'HOST_FLOAT_HIGH_ANGLE_RAW_INPUT','q':q,'source_origin':inputs.name,
            'source_payload_sha256':source_payload_sha,'source_sha256':document['source_hashes'],
            'host_checkpoint':exported['origin'],'float_carrier_law':'RATIONAL_FROM_DECIMAL_TEXT_V1'}
        limits=facts.DensityOracleLimitsV1(caps['preparation_cap'],caps['search_cap'],caps['box_cap'],caps['height_cap'],ExpandLimitsV1(**caps['finite_expand']))
        case=case_of(f'host-float-high-angle-q{q}',tuple(read_raw_spec(raw) for raw in batch['specs']),limits,provenance)
        old=next((old for old in cases if old['id']==case['id']),None)
        if old is not None and old!=case:raise ValueError('DENSITY_HOST_HIGH_ANGLE_SEED_CONFLICT')
        if old is None:cases.append(case);added.append(case['id'])
    manifest_path=path.with_name('native_density_batch_inputs.manifest.json')
    manifest=json.loads(manifest_path.read_text())
    payload=packed({'schema':SCHEMA,'cases':cases});encoded=lzma.compress(payload,preset=6)
    manifest.update(cases=len(cases),compressed_sha256=hashlib.sha256(encoded).hexdigest(),
        payload_sha256=hashlib.sha256(payload).hexdigest(),compressed_bytes=len(encoded),
        host_high_angle_q_coverage=list(range(2,7)),host_high_angle_source_payload_sha256=source_payload_sha)
    path.write_bytes(encoded);manifest_path.write_text(json.dumps(manifest,indent=2)+'\n')
    read_seed(path)
    return {'added':added,'manifest':manifest}

def field_cases(records):
    records=Path(records)
    for row,line in enumerate(records.read_text().splitlines()):
        record=json.loads(line);sha=record['after']['typed_facts_sha256']
        payload=lzma.decompress((records.parent/'blobs'/(sha+'.json.xz')).read_bytes())
        if hashlib.sha256(payload).hexdigest()!=sha:raise ValueError('FIELD_TYPED_FACT_SHA_MISMATCH')
        rows=json.loads(payload)
        yield case_of(f'field-{row:04}',tuple(read_raw_spec(item['input']) for item in rows),limits_for(record['settings']),{'kind':'FIELD_RAW_INPUT_ONLY','source_payload_sha256':sha,'compile_row':row,'run':record['run'],'source_input_sha256':record['input']})

def synthetic_spec(q=2,h=0,owner=1,turn=1,skew=False):
    point=lambda x,y:facts.RawDensityPointV1(Fraction(x),Fraction(y))
    incident=lambda x,y:facts.RawDensityIncidentV1(point(0,0),point(x,y),True)
    gram=(Fraction(2),Fraction(1,2),Fraction(1)) if skew else (Fraction(1),Fraction(0),Fraction(1))
    inverse=(Fraction(4,7),Fraction(-2,7),Fraction(8,7)) if skew else gram
    metric=facts.RawDensityMetricV1(gram,inverse,owner,'synthetic-metric')
    geometry=facts.RawDensityGeometryV1(metric,incident(1,0),incident(0,turn*owner))
    bucket=1 if h==0 else h+1
    declared=Fraction(2*bucket-1,2*q)
    canonical=facts.RawDensityCanonicalFactsV1((declared,declared),None,(),(('binding_present',False),),())
    selection=facts.RawDensitySelectionV1(q,h,'HUBER_EMANATED_COUNT_DENSITY_A_V1',f'LINEAR_REFLEX_DENSITY_{q-2}_V1',bucket,'synthetic-selection')
    return facts.RawDensitySpecInput(geometry,geometry,turn,selection,canonical,'a')

def generate(initial,accepted):
    """Sources only; input hash/provenance stored, expected answers never stored."""
    # Повторная генерация сохраняет отдельный замороженный host-carrier regression set.
    high_angles=[case for case in read_seed() if case['provenance']['kind']=='HOST_FLOAT_HIGH_ANGLE_RAW_INPUT']
    prior_manifest=json.loads(MANIFEST.read_text())
    caps=facts.DensityOracleLimitsV1(4096,4096,96,1000000,ExpandLimitsV1(4096,16,65536))
    cases=[]
    for q in range(2,7):
        for owner in (-1,1):
            for turn in (-1,1):
                for skew in (False,True):
                    h=max(1,(q+1)//2-1)
                    raw=synthetic_spec(q,h,owner,turn,skew)
                    cases.append(case_of(f'synthetic-q{q}-owner{owner}-turn{turn}-skew{int(skew)}',(raw,),caps,{'kind':'SYNTHETIC_RAW_RATIONAL_DECLARATIONS'}))
    cases.append(case_of('synthetic-zero',(synthetic_spec(),),caps,{'kind':'SYNTHETIC_RAW_ZERO_COUNT'}))
    raw=synthetic_spec(5,1);outgoing=replace(raw.source_geometry.outgoing,end=facts.RawDensityPointV1(Fraction(1),Fraction(2)))
    geometry=replace(raw.source_geometry,outgoing=outgoing)
    raw=replace(raw,source_geometry=geometry,evaluation_geometry=geometry,canonical=replace(raw.canonical,source_reflex_interval=(Fraction(2,5),)*2))
    pair=(raw,replace(raw,spec_id='b'))
    cases.append(case_of('synthetic-warm-success',pair,caps,{'kind':'SYNTHETIC_BATCH_MEMO'}))
    cases.append(case_of('synthetic-repeat-failed-band',pair,replace(caps,search_cap=6),{'kind':'SYNTHETIC_BATCH_MEMO'}))
    cases.append(case_of('synthetic-final-failure',pair,replace(caps,preparation_cap=0,search_cap=0),{'kind':'SYNTHETIC_RAW_RESOURCE_LIMIT'}))
    from native_density_raw_batch_diff import skew_undecidable_batch
    witness=skew_undecidable_batch()
    cases.append(case_of('synthetic-skew-undecidable',witness.specs,witness.limits,{'kind':'SYNTHETIC_RAW_SIGN_COUNTEREXAMPLE'}))
    malformed=replace(raw,selection=replace(raw.selection,bucket_c=3,source_hidden_count=2))
    cases.append(case_of('synthetic-malformed-selection',(malformed,),caps,{'kind':'EXPLICIT_MALFORMED_SELECTION_REGRESSION'}))
    grouped={}
    for line in Path(initial).read_text().splitlines():
        record=json.loads(line)
        if record['original_outcome_profile_equivalence']=='DIRECT_UNPATCHED_FIXTURE_COMPILE':grouped.setdefault((record['source'],record['compile_row']),[]).append(record)
    for index,((source,row),records) in enumerate(grouped.items()):
        records.sort(key=lambda record:record['raw_ordinal']);settings=records[0]['settings']
        limits=facts.DensityOracleLimitsV1(settings['preparation_cap'],settings['search_cap'],settings['box_cap'],settings['height_cap'],ExpandLimitsV1(4096,16,65536))
        provenance={'kind':'UNPATCHED_KERNEL_RAW_INPUT','source_origin':Path(source).parent.name+'/'+Path(source).name,'source_sha256':hashlib.sha256(Path(source).read_bytes()).hexdigest(),'compile_row':row,'tests':sorted({record['test'] for record in records})}
        cases.append(case_of(f'kernel-unpatched-{index:02}',tuple(read_raw_spec(record['raw']) for record in records),limits,provenance))
    covered=set();selected=[];field_coverage=set();selection_metadata=[]
    for row,line in enumerate(Path(accepted).read_text().splitlines()):
        record=json.loads(line);sha=record['after']['typed_facts_sha256']
        payload=lzma.decompress((Path(accepted).parent/'blobs'/(sha+'.json.xz')).read_bytes())
        if hashlib.sha256(payload).hexdigest()!=sha:raise ValueError('FIELD_TYPED_FACT_SHA_MISMATCH')
        rows=json.loads(payload);qs={item['input']['selection']['q'] for item in rows}
        features={'run:'+record['run']}
        if not rows:features.add('empty-compile-scope')
        if len(rows)>1:features.add('multispec')
        for item in rows:
            raw,result=item['input'],item['result']
            features.update(('q:'+str(raw['selection']['q']),'source-H:'+str(raw['selection']['source_hidden_count']),'owner:'+str(raw['source_geometry']['metric']['owner_orientation_sign'])))
            features.add('metric-skew' if raw['source_geometry']['metric']['gram'][1][0] else 'metric-diagonal')
            if result['noise_record'] is not None:features.add('noise-record')
            if result['count_lift'] is not None:features.add('count-lift')
            if result['failure'] is not None:features.add('named-failure:'+result['failure']['outcome'])
            if result['authority'] is not None:
                features.add('canonical-authority' if 'ray_law' in result['authority'] else 'adaptive-authority')
            if not result['ordered_work_articles'] and (result['authority'] is not None or any(result['binding_certificates'])):features.add('successful-memo-reuse')
            for event in result['shell_events']:
                features.update('shell:'+code for code in event['codes'])
                if event['rational_denominator']==5120 and abs(event['rational_numerator'] or 0) in (1657,3647):
                    features.add('grid-straw:'+str(abs(event['rational_numerator']))+'/5120')
        new_features=features-field_coverage
        if new_features:
            if len(selected)>=64:raise ValueError('FIELD_SEED_REPRESENTATIVE_LIMIT')
            field_coverage.update(features);covered.update(qs);selected.append(row)
            identity=f'field-representative-{row:04}'
            selection_metadata.append({'id':identity,'compile_row':row,'run':record['run'],'selected_for':sorted(new_features)})
            cases.append(case_of(identity,tuple(read_raw_spec(item['input']) for item in rows),limits_for(record['settings']),{'kind':'FIELD_RAW_INPUT_ONLY','source_origin':'raw-candidate-compile/full2432-v1/records.jsonl','source_payload_sha256':sha,'compile_row':row,'run':record['run'],'source_input_sha256':record['input']}))
    cases.extend(high_angles)
    document={'schema':SCHEMA,'cases':cases};payload=packed(document);encoded=lzma.compress(payload,preset=6)
    SEED.write_bytes(encoded)
    manifest={'schema':SCHEMA,'cases':len(cases),'batch_laws':list(LAWS),'compressed_sha256':hashlib.sha256(encoded).hexdigest(),'payload_sha256':hashlib.sha256(payload).hexdigest(),'compressed_bytes':len(encoded),'field_q_coverage':sorted(covered),'field_compile_rows':selected,'field_selection_window_law':LAWS[0],'field_coverage':sorted(field_coverage),'field_selection':selection_metadata,'inputs_only':True}
    manifest.update(host_high_angle_q_coverage=list(range(2,7)),host_high_angle_source_payload_sha256=prior_manifest['host_high_angle_source_payload_sha256'])
    MANIFEST.write_text(json.dumps(manifest,indent=2)+'\n')
    read_seed()
    return manifest

if __name__=='__main__':
    import argparse
    parser=argparse.ArgumentParser();parser.add_argument('--initial',type=Path);parser.add_argument('--accepted',type=Path)
    parser.add_argument('--append-host-high-angle',type=Path)
    arguments=parser.parse_args()
    if arguments.append_host_high_angle:
        if arguments.initial or arguments.accepted:parser.error('append mode cannot regenerate the existing corpus')
        result=append_host_high_angle(arguments.append_host_high_angle)
    else:
        if not arguments.initial or not arguments.accepted:parser.error('--initial and --accepted are required for generation')
        result=generate(arguments.initial,arguments.accepted)
    print(json.dumps(result,indent=2))
