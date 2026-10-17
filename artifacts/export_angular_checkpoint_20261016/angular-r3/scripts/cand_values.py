import json, hashlib, sys
for f in sys.argv[1:]:
    raw = open(f, 'rb').read()
    d = json.loads(raw)
    p = d['pins']
    h = hashlib.sha256(json.dumps(p, sort_keys=True, separators=(',', ':')).encode()).hexdigest()
    print(f, 'file_sha', hashlib.sha256(raw).hexdigest())
    print('  ', d['profile'], 'approved', d['approved'], 'head', d['source_head'], 'clean', d['source_status'] == '',
          'hashes', len(p['hashes']), 'runtime', len(p['runtime']['fingerprints']), 'shapes', len(p['wire_metadata']['class_shapes']),
          'sources', len(d['source_files']), 'pins_sha', h)
