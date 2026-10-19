"""Компактная квитанция независимого аудита; продукт и исходные evidence не меняются."""
import hashlib
import json
import shutil
import subprocess
from fractions import Fraction as F
from pathlib import Path

ROOT = Path('D:/CFTUV-lazydrag')
SCRATCH = Path('D:/lazydrag-scratch')
DEST = ROOT/'artifacts/lazy_drag_20261019'
BASE = 'f8f52ed75dce0ecb650d8f1f45dcb586597b1de0'


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


fine = json.loads((SCRATCH/'out/independent_focus_fine.json').read_text())
samples = sorted([
    {'width':w,'accepted':v['merge']['accepted_by_uv'],
     'faces':v['faces'],'vertices':v['vertices'],'residual_hex':v['merge']['residual_hex']}
    for w,v in fine['widths'].items()
    if abs(F(w)-F('0.0485189140073579'))<=F('4e-15')
],key=lambda v:F(v['width']))
transitions = [[a['width'],b['width']] for a,b in zip(samples,samples[1:]) if a['accepted']!=b['accepted']]
assert len(samples)==81 and len(transitions)==24
assert all(F(str(float(v['width'])))==F(v['width']) for v in samples)
assert fine['clean_no_hooks_checked']==87 and not fine['clean_no_hooks_mismatches']
compact = {
    'samples':samples,'transitions':transitions,'sample_count':len(samples),
    'transition_count':len(transitions),'all_widths_roundtrip_through_host_str_float':True,
    'clean_full_materializations_without_hooks':fine['clean_no_hooks_checked'],
    'clean_full_count_mismatches':fine['clean_no_hooks_mismatches'],
    'scene_sha256_before':fine['scene_sha256_before'],
    'scene_sha256_after':fine['scene_sha256_after'],
    'native_build_id':fine['native']['build_id'],
    'scope':'Observed binary64 nonmonotonic toggles, independently rechecked without hooks. This sampled width range is NOT a proved bounding band.'
}
(SCRATCH/'out/float_switch_observations.json').write_text(json.dumps(compact,indent=2),encoding='utf-8', newline='\n')
for name in ('independent_focus.json','exact_event.json','float_switch_observations.json'):
    shutil.copyfile(SCRATCH/'out'/name,DEST/name)
for name in ('independent_focus.py','exact_event.py','bundle_independent_evidence.py'):
    shutil.copyfile(SCRATCH/name,DEST/'scripts'/name)

scenes = {
    'building':Path('C:/Users/helmd/AppData/Local/Temp/cftuv_field_parity_20261008/frozen/testscene.blend'),
    'cover008':Path('C:/Users/helmd/AppData/Local/Temp/cftuv_cover008_frozen_v2/buildings2_2.blend'),
}
expected = {
    'building':'750baefc50ddbccad2ff090dae135c55e7b0d1584ca62697261a32b249e15fb4',
    'cover008':'a044b5aa8d5156bcf64167d7181557fb563b2d1472d8f837b4c6a51ce414ab32',
}
hashes = {name:sha(p) for name,p in scenes.items()}
assert hashes==expected
files = ['INDEPENDENT_DESIGN_RU.txt','independent_focus.json','exact_event.json',
         'float_switch_observations.json','scripts/independent_focus.py',
         'scripts/exact_event.py','scripts/bundle_independent_evidence.py']
changed = subprocess.check_output(['git','diff','--name-only',BASE,'--','cftuv','kernel','native','cftuv_native','tests','tools'],cwd=ROOT,text=True)
assert not changed.strip(),changed
receipt = {
    'base':BASE,'previous_design_commit':'1d89542a9fcf62007d23c6335b986e112a1ed20b',
    'date':'2026-10-19','product_diff_vs_base':[],
    'scene_sha256':hashes,'native_build_id':fine['native']['build_id'],
    'captured_preparation_sha256':fine['pickle_sha256'],
    'evidence_files_sha256':{name:sha(DEST/name) for name in files},
    'executed':['headless narrow saved-preparation materialization, 7 named widths',
                'headless 81-width fine grid plus 87 clean no-hook count checks',
                'exact degree-10 polynomial and rational interval sign/derivative proof',
                'source audit and scene hash checks'],
    'not_executed':['new full-scene capture','new whole-corpus sweep','new tree_ab',
                    'Blender timing benchmark','GUI/perceptual acceptance','product implementation'],
    'replay':['Blender -b <frozen cover008> --python-exit-code 1 --python independent_focus.py',
              'python exact_event.py',
              'Blender -b <frozen cover008> --python-exit-code 1 --python independent_focus.py -- --fine',
              'python bundle_independent_evidence.py'],
    'limits':['Polynomial is exact conditional on recorded affine operands; three-width check is not a global law proof.',
              'Sampled FP oscillation is not a certified enclosing band.',
              'Performance and complete certification remain implementation acceptance work.'],
    'free_bytes_end':{drive:shutil.disk_usage(drive).free for drive in ('C:/','D:/','E:/','F:/','G:/','H:/') if Path(drive).exists()}
}
assert min(receipt['free_bytes_end'].values()) >= 1.5*1024**3
(DEST/'INDEPENDENT_RECEIPT.json').write_text(json.dumps(receipt,indent=2),encoding='utf-8', newline='\n')
print('EVIDENCE_BUNDLE_OK',len(files),'files; no product diff; hashes match')
