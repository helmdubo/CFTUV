"""Spot-check mutants of the touched Python shim code (bind once / check per call / source scope). Mutates a dev site copy only."""
import json, shutil, subprocess, sys
from pathlib import Path
S = Path('D:/cftuv-angular-gates-20261017')
REPO = Path('D:/CFTUV-export-angular-20261016')
BP = 'C:/Program Files/Blender Foundation/Blender 4.5/4.5/python/bin/python.exe'
B, W, O = 'angular_bindings.py', 'angular_wire.py', 'angular_operation.py'
MUTANTS = [
    ('Y01_state_check_skipped', B, "            if not _state_unchanged(live): raise NativePortStale('FUNCTION_STATE_CHANGED:'+name)\n", ''),
    ('Y02_members_check_skipped', B, "            if table is not None and not _members_unchanged(*table): raise NativePortStale('CLASS_MEMBERS_CHANGED:'+name)\n", ''),
    ('Y03_identity_check_skipped', B, "            if _identity(current) is not identity: raise NativePortStale('CALLABLE_OR_CLASS_CHANGED:'+name)\n", ''),
    ('Y04_member_values_unchecked', B, "if len(namespace)!=count or not all(map(is_,map(namespace.get,names),values)): return False", "if len(namespace)!=count: return False"),
    ('Y05_member_count_unchecked', B, "if len(namespace)!=count or not all(", "if not all("),
    ('Y06_state_always_unchanged', B, "    fn,code,defaults,kw,items=live\n    if code is None: return True\n", "    return True\n"),
    ('Y07_validate_never_checks_anchors', B, "        if not anchors_checked: self.check_anchors()\n", ''),
    ('Y08_early_gate_removed', O, "    early=assembly_binding.validate(SimpleNamespace(binding_metadata=wire_binding.metadata,site_metadata=()),maps)\n", "    early=None\n"),
    ('Y09_class_line_off_by_one', W, "    return ''.join(_inspect.getblock(lines[lnum:]))", "    return ''.join(_inspect.getblock(lines[lnum+1:]))"),
    ('Y10_scope_cache_ignores_identity', W, "    if entry is not None and entry[0] is value:return entry[1]", "    if entry is not None:return entry[1]"),
    ('Y11_last_class_wins', W, "        found.setdefault('.'.join(inner),", "        found.__setitem__('.'.join(inner),"),
    ('Y12_bind_snapshot_not_checked', B, "            if members is not None and _class_members(value)!=members: raise NativePortStale('CLASS_MEMBERS_CHANGED:'+name)\n", ''),
]
TESTS = ['tests/test_native_angular_bind_once.py', 'tests/test_native_angular_facade.py', 'tests/test_native_angular_operation.py']
results = []
for mid, rel, old, new in MUTANTS:
    site = S / 'pymut' / mid
    if site.exists():
        shutil.rmtree(site)
    shutil.copytree(S / 'wheel-dev/site', site)
    target = site / 'cftuv_native' / rel
    raw = target.read_bytes().decode('utf-8')
    crlf = '\r\n' in raw
    text = raw.replace('\r\n', '\n')
    assert text.count(old) == 1, (mid, text.count(old))
    text = text.replace(old, new)
    target.write_bytes((text.replace('\n', '\r\n') if crlf else text).encode('utf-8'))
    env = {k: v for k, v in __import__('os').environ.items() if not k.startswith('CFTUV_')}
    env['PYTHONPATH'] = str(site) + ';C:/Users/helmd/.cftuv-native/venv-corpus/Lib/site-packages'
    proc = subprocess.run([BP, '-B', '-m', 'pytest', '-q', '-p', 'no:cacheprovider', '--basetemp', str(S / 'pymut-pt' / mid), *TESTS], cwd=REPO, env=env, capture_output=True, text=True)
    last = proc.stdout.strip().splitlines()[-1] if proc.stdout.strip() else proc.stderr[-200:]
    results.append(dict(id=mid, file=rel, killed=proc.returncode != 0, summary=last))
    print(mid, 'KILLED' if proc.returncode else 'SURVIVED', last, flush=True)
    shutil.rmtree(site, ignore_errors=True)
(S / 'pymut-results.json').write_text(json.dumps(results, indent=2))
