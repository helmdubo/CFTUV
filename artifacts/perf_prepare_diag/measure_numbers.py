"""Битовые длины ТОЧНЫХ ЧИСЕЛ чарта и законов прихода по вариантам меша."""

from __future__ import annotations

import json
import statistics
import time
from fractions import Fraction

import env  # noqa: F401
from run_domain import bundle_from_field_snapshot, snapshot_and_request
from snapshot_bmesh import load_snapshot

import cftuv_envelope as kernel
from cftuv_envelope.reference import adaptive_density_fan as fan
from cftuv.surface_ir import HOST_GRID_POLICY, HOST_PLANARITY_POLICY

FAN_CALLS = []


def _spy_fan():
    original = fan._certify_adaptive_density_fan_prepared

    def spy(metric, ideal, orientation, max_subturn_q, records, **kwargs):
        entry = {"q": max_subturn_q, "ideal_count": len(ideal)}
        started = time.perf_counter()
        try:
            result = original(
                metric, ideal, orientation, max_subturn_q, records, **kwargs
            )
        except fan.DensityRationalAuthorityExhausted as exc:
            entry["seconds"] = round(time.perf_counter() - started, 4)
            entry["outcome"] = "EXHAUSTED"
            entry["detail"] = str(exc)
            FAN_CALLS.append(entry)
            raise
        entry["seconds"] = round(time.perf_counter() - started, 4)
        entry["outcome"] = "OK"
        entry["D_star"] = result.minimal_common_height
        entry["termination_height_upper_bound"] = result.termination_height_upper_bound
        entry["window_width_floats"] = [
            float(Fraction(*w.certified_termination_width))
            for w in result.ordinal_windows
        ]
        entry["window_slope_bits"] = [
            max(
                Fraction(*w.termination_lower_slope).denominator.bit_length(),
                Fraction(*w.termination_upper_slope).denominator.bit_length(),
            )
            for w in result.ordinal_windows
        ]
        FAN_CALLS.append(entry)
        return result

    fan._certify_adaptive_density_fan_prepared = spy


_spy_fan()


def rat(value) -> Fraction:
    return Fraction(value.numerator, value.denominator)


def bits(value) -> tuple[int, int]:
    frac = Fraction(value)
    return (frac.numerator.bit_length(), frac.denominator.bit_length())


def summarise(pairs):
    if not pairs:
        return {}
    nums = [item[0] for item in pairs]
    dens = [item[1] for item in pairs]
    return {
        "count": len(pairs),
        "num_bits_max": max(nums),
        "num_bits_median": int(statistics.median(nums)),
        "den_bits_max": max(dens),
        "den_bits_median": int(statistics.median(dens)),
    }


def metric_facts(snapshot, patch_id, domain_id):
    metric = kernel.build_rational_affine_planar_metric(
        source_revision=snapshot.source_revision,
        patch_domain_id=next(
            item.patch_domain_id
            for item in snapshot.patch_domains
        ),
        owner_patch_id=next(
            item.patch_id
            for item in snapshot.patches
        ),
        source_vertices=snapshot.source_vertices,
        source_faces=snapshot.surface_ir.source_faces,
        planarity_policy=kernel.PlanarityAdmissionLawV1(HOST_PLANARITY_POLICY.value),
        grid_policy=kernel.GridSnappingLawV1(HOST_GRID_POLICY.value),
    )
    certificate = metric.planarity_certificate
    near_planar = type(certificate).__name__ == "NearPlanarProjectionCertificateV1"
    origin = [rat(getattr(metric.exact_origin, a)) for a in "xyz"]
    basis_a = [rat(getattr(metric.exact_basis_a, a)) for a in "xyz"]
    basis_b = [rat(getattr(metric.exact_basis_b, a)) for a in "xyz"]
    gram = [rat(getattr(metric.exact_gram_matrix, m)) for m in ("m00", "m01", "m10", "m11")]
    inverse = [
        rat(getattr(metric.exact_inverse_gram_matrix, m))
        for m in ("m00", "m01", "m10", "m11")
    ]
    coordinates = [
        (rat(item.domain_coordinate.x), rat(item.domain_coordinate.y))
        for item in metric.exact_source_vertex_coordinates
    ]
    flat_coordinates = [value for pair in coordinates for value in pair]
    facts = {
        "planarity": type(certificate).__name__,
        "near_planar": near_planar,
        "projected_vertices": (
            len(certificate.projected_source_vertex_ids) if near_planar else 0
        ),
        "grid_law": metric.grid_certificate.snapping_law.value,
        "grid_window_step": (
            str(metric.grid_certificate.window_step)
            if metric.grid_certificate.window_step is not None
            else None
        ),
        "origin_bits": [bits(v) for v in origin],
        "basis_a_bits": [bits(v) for v in basis_a],
        "basis_b_bits": [bits(v) for v in basis_b],
        "basis_a_len2_float": float(sum(v * v for v in basis_a)),
        "basis_b_len2_float": float(sum(v * v for v in basis_b)),
        "gram_bits": [bits(v) for v in gram],
        "inverse_gram_bits": [bits(v) for v in inverse],
        "gram_condition_float": None,
        "chart_coordinates": summarise([bits(v) for v in flat_coordinates]),
    }
    g00, g01, _, g11 = [float(v) for v in gram]
    trace = g00 + g11
    determinant = g00 * g11 - g01 * g01
    if determinant > 0:
        disc = max(trace * trace / 4.0 - determinant, 0.0) ** 0.5
        big, small = trace / 2.0 + disc, trace / 2.0 - disc
        facts["gram_condition_float"] = big / small if small > 0 else None
    if near_planar:
        facts["max_residual"] = float(rat(certificate.max_residual_squared)) ** 0.5
        facts["residual_budget"] = float(rat(certificate.residual_budget))
    return metric, facts


def law_facts(prepared):
    """Битовые длины коэффициентов законов прихода после подготовки."""

    pairs = []
    scale_pairs = []
    for region in prepared.regions:
        for owner, law in region.owner_by_edge:
            for value in owner:
                if isinstance(value, int):
                    pairs.append((value.bit_length(), 1))
            del law
    lattice = _lattice_alpha_bits(prepared)
    return {
        "owner_span_ints": summarise(pairs) if pairs else {},
        "lattice": lattice,
        "scale_pairs": scale_pairs,
    }


def _lattice_alpha_bits(prepared):
    try:
        scale = prepared.counter("CONVEYOR_LATTICE_SCALE")
    except Exception:
        scale = None
    return {"lattice_scale": scale}


VARIANTS = {}


def variant(name):
    def wrap(builder):
        VARIANTS[name] = builder
        return builder

    return wrap


def walls_012_positions():
    payload = load_snapshot(env.SNAPSHOTS / "walls_012_snapshot.json")
    return payload, [tuple(v) for v in payload["raw"]["vertices"]]


def run_variant(name, snapshot_path, vertex_override, selected):
    FAN_CALLS.clear()
    _, _, bundle = bundle_from_field_snapshot(
        snapshot_path, vertex_override=vertex_override
    )
    rows = snapshot_and_request(bundle, selected)
    out = []
    for patch_id, domain_id, snapshot, request in rows:
        _, facts = metric_facts(snapshot, patch_id, domain_id)
        FAN_CALLS.clear()
        started = time.perf_counter()
        compiled = kernel.compile_reference_envelopes(snapshot, request)
        compile_seconds = time.perf_counter() - started
        record = {
            "variant": name,
            "patch_id": patch_id,
            "domain_id": domain_id,
            "metric": facts,
            "compile_outcome": compiled.outcome.value,
            "compile_seconds": round(compile_seconds, 3),
            "fan_calls": list(FAN_CALLS),
            "faces": len(snapshot.surface_ir.source_faces),
            "source_vertices": len(snapshot.source_vertices),
        }
        out.append(record)
    return out
