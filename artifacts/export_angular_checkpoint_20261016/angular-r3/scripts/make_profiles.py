"""TEST-ONLY isolated-parity profiles (light and full) for one native build. NOT production approval; injected only by harnesses."""
import hashlib, json, sys
from pathlib import Path
template_path, build_id, kernel_identity, out_dir, *candidates = sys.argv[1:]
sha = lambda p: hashlib.sha256(Path(p).read_bytes()).hexdigest()
template = json.loads(Path(template_path).read_text())
for cand_path in candidates:
    cand = json.loads(Path(cand_path).read_text())
    assert cand['approved'] is False and cand['source_status'] == '' and cand['runtime']['version'] == template['python']
    assert cand['actual_oracle_identity'] == template['actual_oracle_identity']
    new = dict(template)
    new.update(profile=cand['profile'], hashes=cand['pins']['hashes'], runtime=cand['pins']['runtime'], wire_metadata=cand['pins']['wire_metadata'],
               source_files=cand['source_files'], source_head=cand['source_head'], source_status=cand['source_status'],
               native_build_id=build_id, kernel_identity=kernel_identity,
               candidate_path=str(Path(cand_path).resolve()), candidate_sha256=sha(cand_path), closure_receipts={str(Path(cand_path).resolve()): sha(cand_path)},
               approved=True, production_approved=False, approval_scope='isolated parity evaluation only; not production acceptance',
               approval_note='TEST-ONLY profile for harness injection (replay, field A/B, timing); no product constants; APPROVED_PINS stays empty')
    out = Path(out_dir) / ('profile-test-' + cand['profile'].replace(':', '-') + '-' + build_id[:8] + '.json')
    out.write_text(json.dumps(new, indent=2, sort_keys=True))
    print(out, sha(out))
