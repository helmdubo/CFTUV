"""Actual Blender full-profile pin AUTHORING only (candidate, approved=false). No save, geometry, native import or approval."""
import hashlib, json, runpy, sys, traceback
from pathlib import Path
ROOT = Path('D:/CFTUV-export-angular-20261016')
SCENE = Path('C:/Users/helmd/AppData/Local/Temp/cftuv_field_parity_20261008/frozen/testscene.blend')
SCENE_SHA = '750baefc50ddbccad2ff090dae135c55e7b0d1584ca62697261a32b249e15fb4'
OUT = Path(sys.argv[sys.argv.index('--') + 1])
DEPS = 'C:/Users/helmd/.cftuv-native/venv-corpus/Lib/site-packages'
sha = lambda p: hashlib.sha256(Path(p).read_bytes()).hexdigest()
report = dict(status='INCOMPLETE', scope='offline full-profile candidate only; no pin approval')
code = 1
try:
    import bpy
    assert bpy.app.background and Path(bpy.data.filepath).resolve() == SCENE.resolve()
    assert sha(SCENE) == SCENE_SHA
    cached = {n: getattr(m, '__file__', None) for n, m in tuple(sys.modules.items()) if n == 'cftuv' or n.startswith('cftuv.')}
    for n, f in cached.items():
        assert type(f) is str and Path(f).resolve().is_relative_to(ROOT / 'cftuv'), 'WRONG_CACHED_HOST ' + n
    assert 'cftuv_native' not in sys.modules
    sys.path[:0] = [str(ROOT), str(ROOT / 'kernel/src'), str(ROOT / 'tools'), DEPS]
    argv = list(sys.argv)
    try:
        sys.argv = [str(ROOT / 'tools/native_angular_pins.py'), '--profile', 'full', '--out', str(OUT)]
        try:
            runpy.run_path(sys.argv[0], run_name='__main__')
        except SystemExit as done:
            if done.code not in (None, 0):
                raise
    finally:
        sys.argv = argv
    value = json.loads(OUT.read_text())
    assert value['profile'] == 'CPython-3.11:full' and value['source_status'] == '' and value['approved'] is False
    assert 'cftuv_native' not in sys.modules
    report.update(status='FULL_PROFILE_CANDIDATE_COMPLETE_NOT_APPROVED', candidate=str(OUT), candidate_sha256=sha(OUT),
                  source_head=value['source_head'], hashes=len(value['pins']['hashes']),
                  class_shapes=len(value['pins']['wire_metadata']['class_shapes']))
    code = 0
except BaseException:
    report.update(status='FAILED_NOT_ACCEPTED', failure=traceback.format_exc())
finally:
    report['frozen_sha256_after'] = sha(SCENE)
    if report['frozen_sha256_after'] != SCENE_SHA:
        code = 1; report['status'] = 'FAILED_FROZEN_CHANGED'
    Path(str(OUT) + '.receipt.json').write_text(json.dumps(report, indent=2))
sys.exit(code)
