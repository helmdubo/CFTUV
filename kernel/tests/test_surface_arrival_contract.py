"""Формы `SurfaceArrivalComplexV1`, которых планарная редукция НЕ производит.

Ворота плоской редукции доказывают, что планарный ответ выражается контрактом
без потерь. Они не могут доказать ничего о формах, которых у планарной очереди
нет вовсе: bary-регион, точечный посев, радиальный закон прихода,
сертифицированная точность, сохранённая ничья, станции обхода конусной вершины.
Форма без единого производителя и без единого отрицательного контроля — слух,
поэтому здесь у каждой из них есть и построенный экземпляр, и дверь, которую
можно хлопнуть.

Разделение файлов то же, что у S0 (`test_surface_adjacency_contract.py`,
`test_surface_metric_v2_contract.py`): ворота проверяют СВЕРКУ двух колонок,
контрактный тест — САМ контракт.
"""

from __future__ import annotations

from dataclasses import replace
from decimal import Decimal
from fractions import Fraction

import pytest

from cftuv_envelope.contracts.analysis import SurfaceRegime
from cftuv_envelope.contracts.metric import ExactPoint2V1, ExactRationalV1
from cftuv_envelope.contracts.surface_adjacency import (
    SurfaceAdjacencyDigestValue,
    SurfaceAdjacencyScopeV1,
)
from cftuv_envelope.contracts.surface_arrival import (
    SURFACE_ARRIVAL_COMPLEX_V1_SCHEMA,
    ArrivalCandidateV1,
    ArrivalCellId,
    ArrivalCellLawV1,
    ArrivalCellV1,
    ArrivalCounterV1,
    ArrivalDomainLawV1,
    ArrivalDomainV1,
    ArrivalLawV1,
    ArrivalMetricDigestValue,
    ArrivalOwnerKeyV1,
    ArrivalPathHistoryKeyV1,
    ArrivalSeedKindV1,
    ArrivalSeedV1,
    ArrivalSupportLineV1,
    ArrivalTieResolutionV1,
    BarycentricHalfPlaneV1,
    CertifiedArrivalPrecisionV1,
    CutLocusPointV1,
    ExactAlgebraicPointV1,
    ExactAlgebraicSumV1,
    ExactAlgebraicTimeV1,
    ExactArrivalPrecisionV1,
    ExactRadicalTermV1,
    SeedStationIntervalV1,
    StationWitnessKindV1,
    StationWitnessV1,
    SurfaceArrivalBackendV1,
    SurfaceArrivalComplexV1,
    SurfaceArrivalContractError,
    SurfaceMetricRefV1,
    exact_rational,
    validate_surface_arrival_complex,
)
from cftuv_envelope.contracts.surface_metric_v2 import (
    NamedEpsilonV1,
    SurfaceAdjacencyRefV1,
    SurfaceMetricPrecisionTierV1,
)
from cftuv_envelope.ids import LawId, PatchDomainId, SourceRevision


DOMAIN = PatchDomainId("arrival:domain")
REVISION = SourceRevision("arrival:revision")
ADJACENCY = SurfaceAdjacencyRefV1(
    REVISION,
    SurfaceAdjacencyScopeV1.FULL_SOURCE_MESH,
    SurfaceAdjacencyDigestValue("a" * 64),
)
EPSILON = NamedEpsilonV1(LawId("UNFOLD_CHAIN_ENCLOSURE_V1"), Decimal("1E-12"))
TRIANGLE = ArrivalCellId("synthetic:triangle:0001")


def _rational(value) -> ExactRationalV1:
    return exact_rational(Fraction(value))


def _sum(value) -> ExactAlgebraicSumV1:
    ratio = Fraction(value)
    if not ratio:
        return ExactAlgebraicSumV1(())
    return ExactAlgebraicSumV1((ExactRadicalTermV1(1, exact_rational(ratio)),))


def _place(x, y) -> ExactAlgebraicPointV1:
    return ExactAlgebraicPointV1(_sum(x), _sum(y))


def _time(value) -> ExactAlgebraicTimeV1:
    return ExactAlgebraicTimeV1(_rational(value), _sum(1))


def _point(x, y) -> ExactPoint2V1:
    return ExactPoint2V1(_rational(x), _rational(y))


SEGMENT_OWNER = ArrivalOwnerKeyV1("seed:segment")
POINT_OWNER = ArrivalOwnerKeyV1("seed:point")
WALL_OWNER = ArrivalOwnerKeyV1("seed:wall")

SEEDS = frozenset(
    {
        ArrivalSeedV1(
            ArrivalSeedKindV1.PARALLEL_SEGMENT,
            SEGMENT_OWNER,
            SeedStationIntervalV1(_point(0, 0), _point(1, 0)),
        ),
        ArrivalSeedV1(
            ArrivalSeedKindV1.POINT,
            POINT_OWNER,
            SeedStationIntervalV1(_point(1, 1), _point(1, 1)),
        ),
        ArrivalSeedV1(
            ArrivalSeedKindV1.BOUNDARY_CONDITION,
            WALL_OWNER,
            SeedStationIntervalV1(_point(0, 1), _point(1, 1)),
        ),
    }
)


def _bary_region() -> ArrivalDomainV1:
    """`b0 - b1 <= 0` — половина треугольника, точная и рациональная."""

    return ArrivalDomainV1(
        ArrivalDomainLawV1.BARYCENTRIC_REGION_V1,
        (
            BarycentricHalfPlaneV1(
                _rational(1), _rational(-1), _rational(0), _rational(0)
            ),
        ),
    )


def _segment_candidate() -> ArrivalCandidateV1:
    return ArrivalCandidateV1(
        SEGMENT_OWNER,
        ArrivalPathHistoryKeyV1("unfold:side:0->1"),
        ArrivalLawV1.PARALLEL_OFFSET_V1,
        (
            StationWitnessV1(StationWitnessKindV1.SIDE_CROSSING, _place(1, 0)),
            StationWitnessV1(
                StationWitnessKindV1.CUT_LOCUS_CONTACT, _place(1, 1)
            ),
        ),
        _bary_region(),
        ExactArrivalPrecisionV1(),
    )


def _radial_candidate() -> ArrivalCandidateV1:
    return ArrivalCandidateV1(
        POINT_OWNER,
        ArrivalPathHistoryKeyV1("unfold:cone:2"),
        ArrivalLawV1.RADIAL_FROM_STATION_V1,
        (StationWitnessV1(StationWitnessKindV1.CONE_VERTEX, _place(1, 1)),),
        _bary_region(),
        CertifiedArrivalPrecisionV1(_enclosure(), EPSILON),
    )


def _enclosure():
    from cftuv_envelope.numeric import (
        CertifiedDecimalIntervalV1,
        IntervalEndpointKind,
    )

    return CertifiedDecimalIntervalV1(
        Decimal("0.25"),
        Decimal("0.75"),
        IntervalEndpointKind.CLOSED,
        IntervalEndpointKind.CLOSED,
        Decimal("1E-12"),
    )


def _complex(**overrides) -> SurfaceArrivalComplexV1:
    cell = ArrivalCellV1(
        TRIANGLE,
        (_segment_candidate(), _radial_candidate()),
        ArrivalTieResolutionV1.MULTIWAY_PRESERVED,
    )
    fields = {
        "schema_version": SURFACE_ARRIVAL_COMPLEX_V1_SCHEMA,
        "patch_domain_id": DOMAIN,
        "source_revision": REVISION,
        "backend": SurfaceArrivalBackendV1.DEVELOPABLE_UNFOLD,
        "metric_ref": SurfaceMetricRefV1(
            REVISION,
            DOMAIN,
            SurfaceRegime.DEVELOPABLE,
            ArrivalMetricDigestValue("b" * 64),
            ADJACENCY,
        ),
        "adjacency_ref": ADJACENCY,
        "cell_law": ArrivalCellLawV1.MESH_TRIANGLE_V1,
        "alpha_horizon": _rational(2),
        "seeds": SEEDS,
        "cells": frozenset({cell}),
        "cut_locus": frozenset(
            {
                CutLocusPointV1(
                    _place(1, 1),
                    _time(1),
                    tuple(
                        sorted(
                            (SEGMENT_OWNER, POINT_OWNER),
                            key=lambda item: item.value,
                        )
                    ),
                )
            }
        ),
        "owner_fragments": frozenset(),
        "events": (),
        "domain_doubled_area": _sum(1),
        "front_outcome": _skeleton_outcome(),
        "ownership_outcome": LawId("EXACT"),
        "proof_status": _proof_status(),
        "proof_obligations": (),
        "precision_tier": SurfaceMetricPrecisionTierV1.REFERENCE_CERTIFIED,
        "named_epsilon": EPSILON,
        "counters": (
            ArrivalCounterV1(LawId("ARRIVAL_CELLS"), 1),
            ArrivalCounterV1(LawId("ARRIVAL_SEEDS"), 3),
        ),
    }
    fields.update(overrides)
    return SurfaceArrivalComplexV1(**fields)


def _skeleton_outcome():
    from cftuv_envelope.wavefront.superlevel import SkeletonOutcome

    return SkeletonOutcome.EXACT


def _proof_status():
    from cftuv_envelope.wavefront.proof import ProofStatus

    return ProofStatus.COMPLETE


def _refusal(callable_, *args, **kwargs) -> str:
    with pytest.raises(SurfaceArrivalContractError) as refusal:
        callable_(*args, **kwargs)
    return refusal.value.invariant


# --------------------------------------------------------------------------
# Положительная сторона: формы, которых у планарной очереди нет
# --------------------------------------------------------------------------


def test_a_preserved_tie_with_a_radial_candidate_on_a_bary_region_is_accepted():
    complex_ = _complex()
    validate_surface_arrival_complex(complex_)
    cell = next(iter(complex_.cells))
    assert cell.tie_resolution is ArrivalTieResolutionV1.MULTIWAY_PRESERVED
    laws = {item.arrival_law for item in cell.candidates}
    assert laws == {
        ArrivalLawV1.PARALLEL_OFFSET_V1,
        ArrivalLawV1.RADIAL_FROM_STATION_V1,
    }
    assert {item.domain.law for item in cell.candidates} == {
        ArrivalDomainLawV1.BARYCENTRIC_REGION_V1
    }
    assert {item.kind for item in complex_.seeds} == {
        ArrivalSeedKindV1.PARALLEL_SEGMENT,
        ArrivalSeedKindV1.POINT,
        ArrivalSeedKindV1.BOUNDARY_CONDITION,
    }


def test_the_law_of_arrival_is_not_tied_to_the_kind_of_the_seed():
    """Посев-отрезок после разворота даёт РАДИАЛЬНЫЙ приход, и это не отказ."""

    cell = next(iter(_complex().cells))
    radial = replace(
        cell.candidates[0], arrival_law=ArrivalLawV1.RADIAL_FROM_STATION_V1
    )
    validate_surface_arrival_complex(
        _complex(
            cells=frozenset(
                {
                    ArrivalCellV1(
                        TRIANGLE,
                        (radial, cell.candidates[1]),
                        ArrivalTieResolutionV1.MULTIWAY_PRESERVED,
                    )
                }
            )
        )
    )


def test_one_seed_reaches_one_cell_by_two_different_paths():
    """Ключ истории пути — не украшение: без него две развёртки слились бы."""

    cell = next(iter(_complex().cells))
    twin = replace(
        cell.candidates[0],
        path_history_key=ArrivalPathHistoryKeyV1("unfold:side:0->2"),
    )
    validate_surface_arrival_complex(
        _complex(
            cells=frozenset(
                {
                    ArrivalCellV1(
                        TRIANGLE,
                        (cell.candidates[0], twin),
                        ArrivalTieResolutionV1.MULTIWAY_PRESERVED,
                    )
                }
            )
        )
    )


# --------------------------------------------------------------------------
# Двери fail-closed
# --------------------------------------------------------------------------


def test_two_candidates_with_one_identity_are_indistinguishable():
    cell = next(iter(_complex().cells))
    with pytest.raises(ValueError, match="неразличимы"):
        ArrivalCellV1(
            TRIANGLE,
            (cell.candidates[0], cell.candidates[0]),
            ArrivalTieResolutionV1.MULTIWAY_PRESERVED,
        )


def test_a_cell_without_candidates_is_absence_not_emptiness():
    with pytest.raises(ValueError, match="без кандидатов"):
        ArrivalCellV1(TRIANGLE, (), ArrivalTieResolutionV1.RESOLVED_EXACT)


def test_a_degenerate_segment_seed_is_a_directional_support_not_a_segment():
    with pytest.raises(ValueError, match="DIRECTIONAL_PARALLEL"):
        ArrivalSeedV1(
            ArrivalSeedKindV1.PARALLEL_SEGMENT,
            SEGMENT_OWNER,
            SeedStationIntervalV1(_point(0, 0), _point(0, 0)),
        )


def test_a_point_seed_does_not_occupy_a_segment():
    with pytest.raises(ValueError, match="одну точку"):
        ArrivalSeedV1(
            ArrivalSeedKindV1.POINT,
            POINT_OWNER,
            SeedStationIntervalV1(_point(0, 0), _point(1, 0)),
        )


def test_the_whole_cell_domain_is_not_cut_by_half_planes():
    with pytest.raises(ValueError, match="не режется"):
        ArrivalDomainV1(
            ArrivalDomainLawV1.WHOLE_CELL_V1, _bary_region().half_planes
        )


def test_a_bary_region_without_half_planes_is_not_a_region():
    with pytest.raises(ValueError, match="полуплоскости"):
        ArrivalDomainV1(ArrivalDomainLawV1.BARYCENTRIC_REGION_V1, ())


def test_a_cut_locus_point_is_a_meeting_of_at_least_two():
    with pytest.raises(ValueError, match="минимум двое"):
        CutLocusPointV1(_place(0, 0), _time(1), (SEGMENT_OWNER,))


def test_cut_locus_owners_are_ordered_and_unique():
    with pytest.raises(ValueError, match="упорядочены"):
        CutLocusPointV1(
            _place(0, 0), _time(1), (SEGMENT_OWNER, SEGMENT_OWNER)
        )


def test_the_canonical_set_of_radicals_does_not_repeat_a_radicand():
    term = ExactRadicalTermV1(2, _rational(1))
    with pytest.raises(ValueError, match="возрастает"):
        ExactAlgebraicSumV1((term, term))


def test_a_zero_coefficient_is_not_stored_in_the_canonical_set():
    with pytest.raises(ValueError, match="Нулевой|нулевой"):
        ExactRadicalTermV1(2, ExactRationalV1(0, 1))


def test_a_time_with_a_zero_divisor_is_refused():
    with pytest.raises(ValueError, match="знаменатель"):
        ExactAlgebraicTimeV1(_rational(1), ExactAlgebraicSumV1(()))


def test_a_negative_speed_square_is_refused():
    with pytest.raises(ValueError, match="неотрицателен"):
        ArrivalSupportLineV1(1, 0, 0, ExactRationalV1(-1, 1))


def test_a_non_integer_support_line_is_refused():
    with pytest.raises(ValueError, match="целые"):
        ArrivalSupportLineV1(1, 0, Fraction(1, 2), _rational(1))


def test_counters_are_ordered_by_name_and_never_repeat():
    assert (
        _refusal(
            _complex,
            counters=(
                ArrivalCounterV1(LawId("ARRIVAL_SEEDS"), 3),
                ArrivalCounterV1(LawId("ARRIVAL_CELLS"), 1),
            ),
        )
        == "COUNTERS_ORDER"
    )


def test_a_certified_candidate_under_the_exact_tier_is_refused():
    """Класс точности комплекса не может быть выше точности его кандидатов."""

    assert (
        _refusal(
            _complex,
            precision_tier=SurfaceMetricPrecisionTierV1.REFERENCE_EXACT_SMALL,
            named_epsilon=None,
        )
        == "CERTIFIED_CANDIDATE_UNDER_EXACT_TIER"
    )


def test_a_planar_regime_never_reaches_the_v2_arm():
    """Закон S0: у PLANAR своя решётко-точная метрика, и V2 её не подменяет."""

    with pytest.raises(ValueError, match="PLANAR"):
        SurfaceMetricRefV1(
            REVISION,
            DOMAIN,
            SurfaceRegime.PLANAR,
            ArrivalMetricDigestValue("b" * 64),
            ADJACENCY,
        )


def test_a_foreign_schema_version_is_refused():
    assert _refusal(_complex, schema_version="cftuv.envelope.other.v1") == (
        "SCHEMA_VERSION"
    )


def test_the_complex_and_its_metric_ref_name_one_revision_and_one_domain():
    assert (
        _refusal(_complex, source_revision=SourceRevision("other"))
        == "SOURCE_REVISION"
    )
    assert (
        _refusal(_complex, patch_domain_id=PatchDomainId("other"))
        == "PATCH_DOMAIN"
    )


def test_one_owner_key_names_one_seed():
    assert (
        _refusal(
            _complex,
            seeds=SEEDS
            | {
                ArrivalSeedV1(
                    ArrivalSeedKindV1.BOUNDARY_CONDITION,
                    SEGMENT_OWNER,
                    SeedStationIntervalV1(_point(2, 2), _point(3, 3)),
                )
            },
        )
        == "SEED_OWNER_UNIQUE"
    )


def test_one_cell_address_names_one_cell():
    """Два набора кандидатов на один адрес — это выбор, сделанный читателем."""

    cell = next(iter(_complex().cells))
    twin = ArrivalCellV1(
        cell.cell_id,
        (cell.candidates[0],),
        ArrivalTieResolutionV1.RESOLVED_EXACT,
    )
    assert (
        _refusal(_complex, cells=frozenset({cell, twin})) == "CELL_ID_UNIQUE"
    )


def test_resolved_exact_keeps_exactly_one_candidate():
    """«Разрешено точно» с двумя выжившими — это неразрешённая ничья."""

    cell = next(iter(_complex().cells))
    assert (
        _refusal(
            _complex,
            cells=frozenset(
                {
                    ArrivalCellV1(
                        cell.cell_id,
                        cell.candidates,
                        ArrivalTieResolutionV1.RESOLVED_EXACT,
                    )
                }
            ),
        )
        == "RESOLVED_EXACT_KEEPS_ONE_CANDIDATE"
    )


def test_a_fragment_owner_outside_the_seeds_is_refused():
    from cftuv_envelope.contracts.surface_arrival import OwnerFragmentV1

    fragment = OwnerFragmentV1(
        ArrivalOwnerKeyV1("seed:stranger"),
        TRIANGLE,
        ArrivalSupportLineV1(1, 0, 0, _rational(1)),
        _point(0, 0),
        _point(1, 0),
        (_place(0, 0), _place(1, 0), _place(1, 1)),
        _sum(1),
    )
    assert (
        _refusal(_complex, owner_fragments=frozenset({fragment}))
        == "FRAGMENT_OWNER_IS_NOT_A_SEED"
    )
