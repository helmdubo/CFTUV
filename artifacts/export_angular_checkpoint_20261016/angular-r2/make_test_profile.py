"""TEST-ONLY isolated-parity profile from a fresh offline candidate. NOT production approval.

Copies the previous isolated profile's schema; replaces only pins/source/identity fields with the
candidate's (pins copied verbatim, class_shapes stay JSON lists; replay load_pins converts them).
"""
import hashlib, json, sys
from pathlib import Path
old_path, cand_path, build_id, kernel_identity, out = sys.argv[1:6]
sha = lambda p: hashlib.sha256(Path(p).read_bytes()).hexdigest()
old = json.loads(Path(old_path).read_text()); cand = json.loads(Path(cand_path).read_text())
assert cand['approved'] is False and cand['profile'] == old['profile'] and cand['source_status'] == ''
assert cand['runtime']['version'] == old['python']
new = dict(old)
new.update(hashes=cand['pins']['hashes'], runtime=cand['pins']['runtime'], wire_metadata=cand['pins']['wire_metadata'],
           source_files=cand['source_files'], source_head=cand['source_head'], source_status=cand['source_status'],
           native_build_id=build_id, kernel_identity=kernel_identity,
           candidate_path=str(Path(cand_path).resolve()), candidate_sha256=sha(cand_path),
           closure_receipts={str(Path(cand_path).resolve()): sha(cand_path)},
           approved=True, production_approved=False,
           approval_scope='isolated parity evaluation only; not production acceptance',
           approval_note='TEST-ONLY re-issue after certificate admission change (4ef3c403); replay injection only; no product constants')
for key in ('actual_oracle_identity',):
    assert cand[key] == old[key]
Path(out).write_text(json.dumps(new, indent=2, sort_keys=True))
print(sha(out))
