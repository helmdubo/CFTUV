"""Ограниченная проверка одного inverse-growth recipe; не новый scalar fallback."""
from __future__ import annotations

import argparse
from contextlib import contextmanager
from dataclasses import replace
from decimal import Decimal
from fractions import Fraction
import hashlib
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "kernel/src"))

import sympy as sp  # noqa: E402
import cftuv_envelope as kernel  # noqa: E402
from cftuv_envelope.codec import to_canonical_data  # noqa: E402
from cftuv_envelope._density_policy import canonical_rotation_rays  # noqa: E402
from cftuv_envelope.reference import adaptive_density_band as band, adaptive_density_fan as fan  # noqa: E402
from cftuv_envelope.reference import compile as compile_module, canonical_fan_rays as rays  # noqa: E402
from cftuv_envelope.reference.density_fan_binding import bind_density_fan  # noqa: E402
from cftuv_envelope.reference.density_turn import _huber_density_interpolated_normals  # noqa: E402
from cftuv_envelope.reference.density_vectors import _density_runtime_vector  # noqa: E402
from cftuv_envelope.reference.direction_binding import has_rational_density_support_direction  # noqa: E402
from cftuv_envelope.reference.metric import ExactPlanarMetric  # noqa: E402
from cftuv_envelope.reference.subturn_exact_limit import density_count_is_feasible  # noqa: E402
from cftuv_envelope.numeric import CertifiedDecimalIntervalV1, IntervalEndpointKind  # noqa: E402
from density_scalar_v2_diff import _NumericReader  # noqa: E402
from density_shell_compile_diff import observed_budgets, ordered_budgets  # noqa: E402
from native_density_wire import Wire  # noqa: E402

ORIENTATION = kernel.TurnOrientation.CCW_IN_OWNER_PATCH_ORIENTATION


def raw_metric(g):
    zero, one = sp.Integer(0), sp.Integer(1)
    return ExactPlanarMetric(((one, zero), (zero, sp.Integer(g))), ((one, zero), (zero, sp.Rational(1, g))), 1)


def scalar_profile(expression):
    wire = Wire()
    root = wire.expression(expression)
    record = dict(nodes=wire.nodes, root=root)
    try:
        scalar = _NumericReader(wire.nodes).read(root)
        record["value_wire"] = scalar.value.wire()
        record["class_count"] = len(scalar.value.terms)
        inverse = scalar.inverse()
        record["inverse_wire"] = inverse.value.wire()
    except Exception as error:
        record["profile"] = dict(type=type(error).__name__, code=getattr(getattr(error, "code", None), "value",
                                  getattr(error, "code", None)), detail=str(error))
    return record


@contextmanager
def observed_slopes(metric):
    from cftuv_envelope.reference.density_legacy_laws_v1 import DensityLegacyLawsV1
    original, rows = DensityLegacyLawsV1.slope, []
    def slope(vector, use_x, owner=None):
        x, y = metric.density_expressions(vector)
        denominator = x if use_x else y
        row = dict(use_x=use_x, denominator=scalar_profile(denominator))
        rows.append(row)
        result = original(vector, use_x, owner)
        wire = Wire()
        row["old_slope"] = dict(nodes=wire.nodes, root=wire.expression(result))
        return result
    DensityLegacyLawsV1.slope = staticmethod(slope)
    try:
        yield rows
    finally:
        DensityLegacyLawsV1.slope = staticmethod(original)


def window_recipe(g, outgoing, count, table, law):
    metric = raw_metric(g)
    rotation = canonical_rotation_rays(Fraction(1, 2), count + 1, 6) if table else None
    ideal = _huber_density_interpolated_normals(
        metric, _density_runtime_vector(1, 1), _density_runtime_vector(*outgoing), count, 1,
        rational_rotation=rotation)
    covectors = fan._covectors(metric, ideal, law, ORIENTATION)
    with observed_slopes(metric) as slopes, observed_budgets() as budgets:
        try:
            windows, records = fan._window_envelopes(covectors)
            answer = dict(status="returned", windows=len(windows),
                          records=[repr(record) for record in records])
        except Exception as error:
            answer = dict(status="raised", type=type(error).__name__, detail=str(error),
                          outcome=getattr(getattr(error, "outcome", None), "value", None))
    return dict(g=g, outgoing=outgoing, count=count, table=table, law=law, answer=answer,
                slopes=slopes, budgets=ordered_budgets(budgets))


def selection_and_count(g, outgoing):
    metric = raw_metric(g)
    interval = CertifiedDecimalIntervalV1(Decimal("0.5"), Decimal("0.5"),
                                         IntervalEndpointKind.CLOSED, IntervalEndpointKind.CLOSED, Decimal(0))
    bucket = compile_module._resolve_huber_density_bucket_interval(interval, 6)
    source_count = max(1, bucket - 1)
    facts = []
    for count in (source_count, source_count + 1):
        ideal = _huber_density_interpolated_normals(
            metric, _density_runtime_vector(1, 1), _density_runtime_vector(*outgoing), count, 1)
        facts.append(dict(count=count, feasible=density_count_is_feasible(metric, ideal, 6),
                          rational_hidden=[has_rational_density_support_direction(metric, n) for n in ideal[1:-1]],
                          source_normal_rows=[sp.srepr(value) for n in ideal for value in metric.density_expressions(n)]))
    table = _huber_density_interpolated_normals(
        metric, _density_runtime_vector(1, 1), _density_runtime_vector(*outgoing), 3, 1,
        rational_rotation=canonical_rotation_rays(Fraction(1, 2), 4, 6))
    primitive = [rays._primitive_covector(metric, normal) for normal in table[1:-1]]
    return dict(g=g, outgoing=outgoing, canonical=Fraction(1, 2).as_integer_ratio(),
                bucket=bucket, source_count=source_count, actual_dot=1 * outgoing[0] + g * outgoing[1],
                counts=facts, table_primitive_covectors=primitive,
                table_refusal="CANONICAL_RAYS_IRRATIONAL_IN_CHART" if None in primitive else None)


def old_bind(g, outgoing, count, law):
    metric = raw_metric(g)
    ideal = _huber_density_interpolated_normals(
        metric, _density_runtime_vector(1, 1), _density_runtime_vector(*outgoing), count, 1)
    refusals = []
    with observed_budgets() as budgets:
        try:
            result = bind_density_fan(metric, ideal, ORIENTATION, 6, (None,) * count,
                                      lifted=count > 2, law=law, spec_id="research-recipe", refusals=refusals)
            answer = dict(status="returned", result=to_canonical_data(result))
        except Exception as error:
            answer = dict(status="raised", type=type(error).__name__, detail=str(error),
                          outcome=getattr(getattr(error, "outcome", None), "value", None))
    return dict(g=g, outgoing=outgoing, count=count, law=law, answer=answer,
                refusals=refusals, budgets=ordered_budgets(budgets))


def producer_g5():
    from cftuv_envelope.planar_metric import build_rational_affine_planar_metric
    from cftuv_envelope.contracts.analysis import SourceVertexV1
    from cftuv_envelope.contracts.surface import SourceFaceV1
    from cftuv_envelope.numeric import LocalPoint3V1, LocalVector3V1
    from cftuv_envelope.ids import SourceRevision, SourceVertexId, SourceFaceId, PatchId, PatchDomainId, PhysicalEdgeId
    from cftuv_envelope.contracts.metric import GridSnappingLawV1
    ids, patch = tuple(SourceVertexId("v" + str(i)) for i in range(3)), PatchId("p")
    vertices = tuple(SourceVertexV1(k, LocalPoint3V1(*xyz))
                     for k, xyz in zip(ids, ((0, 0, 0), (1, 0, 0), (0, 1, 2)), strict=True))
    face = SourceFaceV1(SourceFaceId("f"), patch, ids, tuple(PhysicalEdgeId("e" + str(i)) for i in range(3)),
                        LocalVector3V1(0, -2, 1), ())
    metric = build_rational_affine_planar_metric(
        source_revision=SourceRevision("raw-producer-g5"), patch_domain_id=PatchDomainId("d"), owner_patch_id=patch,
        source_vertices=vertices, source_faces=(face,), grid_policy=GridSnappingLawV1.SOURCE_ONLY_GRID_SNAP_V1)
    return dict(gram=to_canonical_data(metric.exact_gram_matrix), basis_a=to_canonical_data(metric.exact_basis_a),
                basis_b=to_canonical_data(metric.exact_basis_b), planarity=metric.planarity_certificate.admission_law.value,
                stretch="EXACT_SOURCE_PLANE; developable stretch rung not used; no Gram-eigenvalue stretch inference")


def invalid_gram_compile():
    from cftuv_envelope.contracts.metric import ExactMatrix2V1, ExactRationalV1
    root = ROOT / "kernel/fixtures/building_patch17_crowded_v1"
    snapshot = kernel.AnalysisSnapshotCodecV1.loads((root / "analysis_snapshot.json").read_bytes())
    request = kernel.DecalRequestCodecV1.loads((root / "decal_request_density4.json").read_bytes())
    descriptor, = snapshot.surface_metric_descriptors
    def matrix(k, inverse=False):
        return ExactMatrix2V1(ExactRationalV1(1, 1), ExactRationalV1(0, 1), ExactRationalV1(0, 1),
                             ExactRationalV1(1, k) if inverse else ExactRationalV1(k, 1))
    changed = replace(descriptor, exact_gram_matrix=matrix(3), exact_inverse_gram_matrix=matrix(3, True))
    with observed_budgets() as budgets:
        result = kernel.compile_reference_envelopes(replace(snapshot, surface_metric_descriptors=frozenset({changed})), request)
    return dict(scope="invalid raw Gram injection, not a valid producer witness", outcome=result.outcome.value,
                diagnostics=to_canonical_data(result.diagnostics), budgets=ordered_budgets(budgets))


def evidence():
    choices = [selection_and_count(3, (-3, 1)), selection_and_count(5, (-3, 1)), selection_and_count(5, (-5, 1))]
    windows = [window_recipe(g, (-g, 1), 3, table, law) for g in (3, 5) for table in (True, False)
               for law in (band.WINDOW_LAW_VORONOI, band.WINDOW_LAW_NARROW_BAND)]
    binds = [old_bind(g, (-g, 1), count, law) for g, count in ((3, 2), (5, 3))
             for law in (band.WINDOW_LAW_VORONOI, band.WINDOW_LAW_NARROW_BAND)]
    return dict(schema="cftuv.inverse-growth-reachability-research.v1", choices=choices, windows=windows, binds=binds,
                producer_g5=producer_g5(), invalid_gram_compile=invalid_gram_compile(),
                source_guard_map=[
                    "validation_metric.py:1306 _check_gram: declared Gram equals rational 3D basis dot products",
                    "compile.py:266 _resolve_huber_density_bucket_interval: u=1/2,q6 -> C3,H2",
                    "compile.py:686 _evaluation_density_spec: feasible source count returns without lift",
                    "canonical_fan_rays.py:263 _decide: TABLE irrational direction returns refusal",
                    "compile.py:810..873: refusal keeps precomputed actual ideal; rotation_ideal is not forwarded to bind",
                    "adaptive_density_band.py:124 window_neighbours: real or phantom neighbors depend on named law"],
                conclusions=["The original diag(1,3),u=1/2,H3 table witness is not this product recipe.",
                             "diag(1,3) has no rational 3D basis Gram: rational Householder sends unit A to e1, giving y^2+z^2=3; mod3 forces numerator/denominator common factor3.",
                             "Even abstract diag(1,3) source H2 is feasible with rational hidden directions; no H3 lift.",
                             "diag(1,5) is producer-realizable, but unchanged outgoing(-3,1) is not quarter; adjusted(-5,1) lifts H2->H3.",
                             "Adjusted diag(1,5) TABLE is refused as irrational in chart, and actual bind receives regular pi/8 ideal, whose scalar-DAG first profile is nonrational root, not inverse cap.",
                             "No universal bound of <=3 classes is inferred. Earlier observed corpus maximum2 remains corpus-only evidence."],
                proposed_path="If a future actual four-term denominator is proven reachable, a separately admitted exact biquadratic conjugation may multiply d by one generator conjugate, invert the resulting <=2-class norm with existing inverse, then multiply by the conjugate. Require explicit generator/coset proof and nonzero norm; do not raise radical inversecap or map candidate profile to product UNDECIDABLE. Not implemented.")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args(argv)
    result = evidence()
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(result, indent=2) + "\n", encoding="utf8")
    print(json.dumps(dict(output=str(args.out),choices=[(x["g"],x["outgoing"],x["counts"][0]["feasible"]) for x in result["choices"]])))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
