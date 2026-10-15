"""Материализует конечную immutable atan таблицу из проверенной калибровки."""
import argparse
import json
from pathlib import Path

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--anchors',type=Path,required=True)
    parser.add_argument('--out',type=Path,required=True)
    args=parser.parse_args()
    data=json.loads(args.anchors.read_text(encoding='utf-8'))
    assert not data['reverse_differences']
    prefixes={row['n']:int(row['certified_floor']) for row in data['records'] if row['precision']==532}
    assert sorted(prefixes)==list(range(256))
    for row in data['records']:
        assert row['delta']==0
        assert prefixes[row['n']]>>(532-row['precision'])==int(row['legacy'])
    body='//! floor(atan(n/128) * 2^532), n=0..255; certified rational-series bounds.\n'
    body+='//! Генератор tools/density_atan_table.py; runtime не использует Newton/libm.\n'
    body+='pub(crate) const ATAN_FLOOR_532: [&str; 256] = [\n'
    body+=''.join(f'    "{prefixes[n]:x}",\n' for n in range(256))+'];\n'
    args.out.write_text(body,encoding='utf-8')
    print('256 certified prefixes;1792 legacy comparisons preserved',flush=True)

if __name__=='__main__': main()
