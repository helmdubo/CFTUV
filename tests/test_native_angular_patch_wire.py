"""The PyO3 single-patch ANGULAR reader/writer (the shim's test-only `cftuv_native.angular_patch`) on hand-made wires: the exact output contract.

The shim's assembly reads the native plan only through products and validations (`angular_assemble`), and the recorded corpus has no
off-junction or beyond-chart site, so a swapped skip name, swapped norms, a dropped band flag or support-triangle list, or a reader that
accepts a non-positive denominator would leave the corpus replay equal. These witnesses pin the wire itself (ANGULAR mutation gate).
The geometry is the L-shaped fixture of `native/cftuv-angular/tests/reference.rs`. No pins, no host objects, no oracle.
"""
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
for _path in (ROOT / 'kernel' / 'src', ROOT):
    if str(_path) not in sys.path:
        sys.path.insert(0, str(_path))

try:
    import cftuv_native  # noqa: F401
except ModuleNotFoundError as error:
    if error.name != 'cftuv_native':
        raise
    pytest.skip('the cftuv_native extension is not built: the ANGULAR wire checks are skipped', allow_module_level=True)

native = cftuv_native  # the shim; angular_patch is its test-only raw entry

if not hasattr(native, 'angular_patch'):
    pytest.skip('this build of the shim has no angular_patch', allow_module_level=True)


def r(n, d=1):
    return (n, d)


def wire(**changes):
    xy = ((-2, -2), (0, -2), (0, 0), (2, 0), (2, 2), (-2, 2))
    patch = {
        'version': 1, 'patch_id': 7, 'owner_sign': 1,
        'gram': ((r(2), r(1)), (r(1), r(3))), 'chart': 'RationalAffine', 'band_or_ring_cut': False,
        'points': tuple((i, (r(x), r(y))) for i, (x, y) in enumerate(xy)),
        'chains': (((7, 0, 0), (1, 2), (11,)), ((7, 0, 1), (2, 3), (12,))),
        'faces': ((9, (0, 1, 2, 3, 4, 5), (10, 11, 12, 13, 14, 15), (90,)),),
        'sites': (((0, 0, 0), False, 2, (7, 0, 0), (7, 0, 1)),),
        'present_vertices': (0, 1, 2, 3, 4, 5), 'support_triangles': None,
    }
    patch.update(changes)
    return patch


REFLEX = ('REFLEX', (r(-6), r(2)), (r(-2), r(4)), -1, r(60), r(40), r(20), r(-20), r(5),
          ('canonical',) * 4 + ('normalize',) * 3)


def test_reflex_plan_is_the_exact_tuple():
    assert native.angular_patch(wire()) == (0, ((0, (0, 0, 0), REFLEX, ('ANGULAR_CORNERS_CONSIDERED', 'ANGULAR_REFLEX_CORNERS'), ('ANGULAR_RELATIONS_BUILT',)),))


def test_owner_sign_reaches_the_core():
    status, ((_, _, decision, _, _),) = native.angular_patch(wire(owner_sign=-1))
    assert status == 0 and decision[3] == 1 and decision[7] == r(20)


def test_cut_site_and_second_ordinal():
    off = ((5, 5, 5), True, 999, None, None)
    status, plans = native.angular_patch(wire(sites=(off, ((0, 0, 1), True, 2, (7, 0, 0), (7, 0, 1)))))
    assert status == 0
    assert plans[0] == (0, (5, 5, 5), ('SKIP', 'OffJunction'), ('ANGULAR_CUT_VERTICES_CONSIDERED', 'ANGULAR_CORNERS_OFF_JUNCTION'), ())
    assert plans[1] == (1, (0, 0, 1), REFLEX, ('ANGULAR_CUT_VERTICES_CONSIDERED', 'ANGULAR_REFLEX_CORNERS', 'ANGULAR_CUT_REFLEX_RELATIONS'), ('ANGULAR_RELATIONS_BUILT',))


def test_support_triangles_reach_the_core():
    assert native.angular_patch(wire(support_triangles=(90,)))[1][0][2] == REFLEX
    assert native.angular_patch(wire(support_triangles=(999,))) == (
        0, ((0, (0, 0, 0), ('SKIP', 'BeyondChartReach'), ('ANGULAR_CORNERS_CONSIDERED', 'ANGULAR_CORNERS_BEYOND_CHART_REACH'), ()),))


def test_band_flag_reaches_the_core():
    assert native.angular_patch(wire(band_or_ring_cut=True)) == (2, 'ANGULAR_CORE_BandOrRingCut', 'site=None')


def test_gram_rows_are_read_in_order():
    # Несимметричная метрика: строки не переставлены и не симметризованы читателем.
    assert native.angular_patch(wire(gram=((r(2), r(1)), (r(2), r(3)))))[:2] == (2, 'ANGULAR_CORE_InvalidMetric')
    status, ((_, _, decision, _, _),) = native.angular_patch(wire(gram=((r(2), r(0)), (r(0), r(3)))))
    assert status == 0 and (decision[4], decision[5], decision[8]) == (r(72), r(48), r(6))


@pytest.mark.parametrize('denominator', [0, -1])
def test_non_positive_denominator_is_a_named_wire_decline(denominator):
    points = list(wire()['points'])
    points[0] = (0, ((-2, denominator), r(-2)))
    assert native.angular_patch(wire(points=tuple(points))) == (2, 'ANGULAR_WIRE', 'positive rational denominator required')


def test_rationals_are_reduced_on_read():
    points = tuple((i, ((x[0] * 3, 3), (y[0] * 3, 3))) for i, (x, y) in wire()['points'])
    assert native.angular_patch(wire(points=points)) == native.angular_patch(wire())
