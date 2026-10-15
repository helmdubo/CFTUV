"""Pinned whole raw batch equals live Python, включая quantum/IDs/ordered price/events."""
from __future__ import annotations
import os
from pathlib import Path
import sys
import pytest

ROOT=Path(__file__).resolve().parents[1]
for directory in (ROOT/'tools',ROOT/'kernel/src'):
    if str(directory) not in sys.path:sys.path.insert(0,str(directory))
try:
    import cftuv_native
except ModuleNotFoundError as error:
    if error.name!='cftuv_native':raise
    pytest.skip('cftuv_native is not built; whole raw Density batch cannot be compared',allow_module_level=True)

from native_gate import field_tier,skip_unless_available
skip_unless_available(cftuv_native,'density')
import native_density_batch_corpus as corpus

CASES=corpus.read_seed()
PARAMETERS=[pytest.param(case,law,id=case['id']+':'+law) for case in CASES for law in corpus.LAWS]

@pytest.mark.parametrize('case,law',PARAMETERS)
def test_native_density_batch_seed_matches_live_oracle(case,law):
    corpus.compare_case(case,law,cftuv_native.density_compute)

FIELD=os.environ.get('CFTUV_NATIVE_DENSITY_BATCH_FIELD','')

@field_tier(bool(FIELD),'owner whole raw Density field records: set CFTUV_NATIVE_DENSITY_BATCH_FIELD to accepted records.jsonl')
def test_native_density_batch_field_matches_live_oracle():
    count=0
    for case in corpus.field_cases(FIELD):
        corpus.compare_case(case,'NARROW_ROTATION_BAND',cftuv_native.density_compute)
        count+=1
    assert count==2432,'declared whole field acceptance must contain all 2432 compile-scoped batches'
