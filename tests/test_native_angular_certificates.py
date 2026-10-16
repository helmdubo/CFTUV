"""ANGULAR wire admits exactly the certificates whose oracle path equals the exact-plane path.

The frozen oracle (`_build_angular_relations_reference`) reads the planarity certificate only through the
band-chart accessors of `envelope_chart_band` (band type check, `support_triangle_ids`, `cut`, and
`strip_boundary` behind `cut`). A certificate class without those fields therefore takes the same oracle
path as `ExactSourcePlaneCertificateV1`; the band chart does not and stays a named wire decline.
"""
import dataclasses
import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
BAND_FIELDS = {'support_triangle_ids', 'cut', 'strip_boundary'}


def _admitted():
    import cftuv_envelope as kernel
    from cftuv import envelope_angular_binding as host
    return host.build_binding_inputs(kernel)['classes']['certificate']


def test_admitted_certificates_are_exact_near_planar_and_whole_unfold():
    from cftuv_envelope.contracts import metric
    assert _admitted() == (metric.ExactSourcePlaneCertificateV1, metric.NearPlanarProjectionCertificateV1,
                           metric.DevelopableUnfoldCertificateV1)
    assert metric.DevelopableBandChartCertificateV1 not in _admitted()


def test_admitted_certificates_carry_no_band_or_cut_field():
    from cftuv_envelope.contracts import metric
    for kind in _admitted():
        assert not BAND_FIELDS & {field.name for field in dataclasses.fields(kind)}, kind.__name__
    assert BAND_FIELDS <= {field.name for field in dataclasses.fields(metric.DevelopableBandChartCertificateV1)}


def test_oracle_reads_certificate_only_through_band_accessors():
    # Any new certificate read in the oracle or chart helpers invalidates the admission argument above.
    reads = set()
    for name in ('cftuv/envelope_angular_relations.py', 'cftuv/envelope_chart_band.py'):
        text = (ROOT / name).read_text(encoding='utf-8')
        reads |= set(re.findall(r'planarity_certificate\.(\w+)', text))
        reads |= set(re.findall(r'getattr\(\s*(?:frame\.|metric\.)?planarity_certificate,\s*"(\w+)"', text))
        reads |= set(re.findall(r'getattr\(frame\.planarity_certificate,\s*"(\w+)"', text))
    oracle = (ROOT / 'cftuv/envelope_angular_relations.py').read_text(encoding='utf-8')
    assert 'getattr(frame.planarity_certificate, "support_triangle_ids", None)' in oracle
    assert reads <= BAND_FIELDS | {'reach_cap', 'tightened', 'excluded_triangle_count'}, reads


def test_wire_declines_band_support_or_cut_by_name():
    text = (ROOT / 'native/cftuv-python/python/cftuv_native/angular_wire.py').read_text(encoding='utf-8')
    assert "getattr(certificate,'support_triangle_ids',None) is None" in text
    assert "getattr(certificate,'cut',None) is None" in text
    assert "'BAND_OR_RING_CUT_UNSUPPORTED'" in text
