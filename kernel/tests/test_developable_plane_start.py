"""ARAP_FROM_PLANE_PROJECTION_V1: третье предложение — ARAP, стартующий с проекции на плоскость Ньюэлла.

Верх изогнутой стены с шумом (синтетика `bumpy_ring`: сектор 345°, гладкие волны и белый шум) шарнир
складывает (накрытие и перевороты), и ARAP80 от шарнира из этого не выходит: отказ
`DEVELOPABLE_CHART_SELF_OVERLAP`, и исход ещё и зависит от нумерации (корень шарнира — треугольник с
наименьшим именем). Тот же ARAP80 со старта с проекции (она сама вложение) даёт карту в 2.9 % растяжения.

Здесь: карта строится законом `ARAP_FROM_PLANE_PROJECTION_80_BINARY64_V1` при 20 % и получает
именованный отказ растяжения при 2 %, называя все три предложения; исход не зависит от нумерации; старт,
который не вложение (винтовая лента 400°), — именованный пропуск при прежнем имени отказа; валидатор
пересчитывает сертификат и ловит подмену следа и закона.
"""

from __future__ import annotations

import random
from dataclasses import replace
from fractions import Fraction

import pytest

from cftuv_envelope._developable import clear_developable_chart_memory
from cftuv_envelope._stretch import stretch_violations
from cftuv_envelope.contracts.metric import (
    CurvatureLadderPolicyV1,
    DevelopableProposalLawV1,
    DevelopableProposalSelectionLawV1,
)
from cftuv_envelope.outcomes import NamedOutcome
from cftuv_envelope.planar_metric import PlanarMetricAdmissionError
from cftuv_envelope.validation_metric import (
    validate_embedding_certified_rational_affine_planar_metric,
)

import developable_factories as factories
from developable_factories import DOMAIN, PATCH, REVISION, developable_chart
from developable_route import build_metric

PLANE = DevelopableProposalLawV1.ARAP_FROM_PLANE_PROJECTION_80_BINARY64_V1
OVERLAP = NamedOutcome.DEVELOPABLE_CHART_SELF_OVERLAP
STRETCH = NamedOutcome.DEVELOPABLE_STRETCH_BUDGET_EXCEEDED
ON = CurvatureLadderPolicyV1.NEAR_PLANAR_THEN_DEVELOPABLE_UNFOLD_V1


def _ring():
    return factories.surface(*factories.bumpy_ring())


def _relabelled(seed):
    """Та же геометрия под другими именами вершин, другим порядком граней и другим началом каждого цикла."""

    points, cycles = factories.bumpy_ring()
    rng = random.Random(seed)
    names = list(points)
    shuffled = names[:]
    rng.shuffle(shuffled)
    rename = dict(zip(names, (f"n{index:03d}x{name}" for index, name in enumerate(shuffled)), strict=True))
    order = cycles[:]
    rng.shuffle(order)
    turned = []
    for cycle in order:
        shift = rng.randrange(len(cycle))
        turned.append([rename[name] for name in cycle[shift:] + cycle[:shift]])
    return factories.surface({rename[name]: point for name, point in points.items()}, turned)


def _issues(record, parts):
    vertices, faces, triangles = parts
    return validate_embedding_certified_rational_affine_planar_metric(
        record,
        source_vertices=vertices,
        source_faces=faces,
        owner_patch_id=PATCH,
        expected_source_revision=REVISION,
        expected_patch_domain_id=DOMAIN,
        expected_source_lineage=frozenset(),
        surface_triangles=triangles,
    )


def _tampered(record, **changes):
    certificate = replace(record.metric.planarity_certificate, **changes)
    return replace(record, metric=replace(record.metric, planarity_certificate=certificate))


def test_the_noisy_ring_is_built_by_arap_from_the_plane_projection_at_twenty_percent():
    """Шарнир и ARAP от шарнира отказаны накрытием (оба в следе); карта — ARAP со старта с проекции."""

    certificate = developable_chart(_ring()).certificate
    band = certificate.stretch.worst_band_squared_upper
    assert certificate.proposal_law is PLANE
    assert certificate.proposal_selection_law is DevelopableProposalSelectionLawV1.ARAP_AFTER_HINGE_REFUSED_V1
    assert certificate.previous_refusals == (OVERLAP.value, OVERLAP.value)
    assert 1.0 < band.numerator / band.denominator < 1.07
    assert certificate.chart_boundary_overlap_count == 0


def test_a_tight_budget_is_a_named_stretch_refusal_naming_all_three_proposals():
    with pytest.raises(PlanarMetricAdmissionError) as failure:
        developable_chart(_ring(), budget=Fraction(2, 100))
    text = str(failure.value)
    assert failure.value.outcome is STRETCH
    assert text.startswith("the plane-start ARAP proposal before snapping: developable stretch")
    assert f"{PLANE.value} after the ARAP proposal was refused: {OVERLAP.value}" in text
    assert "the ARAP unfolding covers itself before any snapping" in text
    assert "ARAP_LOCAL_GLOBAL_80_BINARY64_V1 after the hinge proposal was refused" in text
    assert "the hinge unfolding covers itself before any snapping" in text


def test_the_outcome_does_not_depend_on_the_numbering():
    """Исход «карта есть» не зависит от нумерации: на главной ветке 6 из 21 нумерации отказаны (накрытие или
    растяжение ARAP от шарнира), здесь строятся все. Закон победителя от нумерации зависит по-прежнему: где
    ARAP от шарнира сошёлся, он и остаётся (третье предложение пробуется только после его отказа)."""

    laws = set()
    for seed in range(0, 9):
        clear_developable_chart_memory()
        certificate = developable_chart(_ring() if seed == 0 else _relabelled(seed)).certificate
        assert certificate.proposal_law in {PLANE, DevelopableProposalLawV1.ARAP_LOCAL_GLOBAL_80_BINARY64_V1}, seed
        assert not stretch_violations(certificate.stretch), seed
        assert certificate.chart_boundary_overlap_count == 0, seed
        laws.add(certificate.proposal_law)
    assert PLANE in laws


def test_a_projection_that_is_not_an_embedding_is_a_named_skip_under_the_previous_name():
    """Винтовая лента 400° с шумом: проекция на среднюю плоскость накрывает себя — третье предложение не
    пробуется, отказ ARAP от шарнира остаётся прежним по имени (растяжение) и называет пропуск."""

    parts = factories.surface(*factories.bumpy_ring(steps=28, sweep=400.0, amplitude=0.2, seed=4, pitch=1.0))
    with pytest.raises(PlanarMetricAdmissionError) as failure:
        developable_chart(parts)
    text = str(failure.value)
    assert failure.value.outcome is STRETCH
    assert text.startswith("the ARAP proposal before snapping: developable stretch")
    assert text.endswith(
        f"; {PLANE.value} skipped: the mean-plane projection is not an embedding "
        "(0 flipped triangles, 2 boundary edge pairs meet or overlap)"
    )


def test_the_isometric_spiral_never_reaches_the_plane_start():
    """Спираль без шума: шарнир в бюджете накрывает себя, ARAP ему не дан — и старт с проекции тоже."""

    with pytest.raises(PlanarMetricAdmissionError) as failure:
        developable_chart(factories.spiral_strip())
    assert failure.value.outcome is OVERLAP
    assert PLANE.value not in str(failure.value)


def test_the_validator_recomputes_the_plane_start_certificate_and_catches_tampering():
    parts = _ring()
    record = build_metric(parts, ladder=ON)
    certificate = record.metric.planarity_certificate
    assert certificate.proposal_law is PLANE
    assert certificate.previous_refusals[-2:] == (OVERLAP.value, OVERLAP.value)
    assert _issues(record, parts) == ()
    # Без отказа ARAP в следе: старт с проекции пробуется только после него.
    short = _tampered(record, previous_refusals=certificate.previous_refusals[:-1])
    assert _issues(short, parts)
    # Закон подменён на ARAP от шарнира: пересчёт строит другую карту.
    swapped = _tampered(record, proposal_law=DevelopableProposalLawV1.ARAP_LOCAL_GLOBAL_80_BINARY64_V1)
    assert _issues(swapped, parts)
