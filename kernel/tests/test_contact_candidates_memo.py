"""Контакты источника с границей домена считаются один раз на подготовку, а ответ тот же.

`resolve_component_alphas` на каждом покрытии пересчитывал `_contact_candidates` для
каждой пары `(источник, граница)`, хотя alpha в эти контакты не входит: она стоит лишь в
сравнениях ПОСЛЕ них. На центральном патче `building` 1332 вызова давали 2.0 с из 2.2 с
покрытия. `ContactCandidatesMemoV1` держит их значения в подготовке и возит с ней.

Что проверяется:

1. ОТВЕТ ПОБИТОВО ТОТ ЖЕ с памятью и без неё — на доменах, где контакты решают дело
   (дыра в кольце, вогнутый внешний контур: `BARRIER_SPLIT_REQUIRED`), и на полевой
   фикстуре при alpha от малой до большой. Сравниваются сами резолюции вместе с
   диагностиками, то есть событиями и конструкциями, а не только исходом;
2. память НАСТОЯЩАЯ: после первого покрытия она не пуста, второе не считает ничего;
3. память ЕДЕТ с подготовкой: копия после пикла знает те же контакты, счётчики копии
   нули, а покрытие копии при другой alpha ничего не пересчитывает и отвечает так же;
4. ключ несёт геометрию: отрезок границы и его `reversed()` носят одно `segment_id`, а
   контакты у них разные, и память отдаёт каждому его собственные.
"""

from __future__ import annotations

import pickle
from dataclasses import replace
from decimal import Decimal

import cftuv_envelope as kernel
from cftuv_envelope.contracts.envelopes import StripEnvelopeSpec
from cftuv_envelope.numeric import LocalLengthV1
from cftuv_envelope.reference import compile_reference_envelopes
from cftuv_envelope.reference.boundary import (
    ContactCandidatesMemoV1,
    _contact_candidates,
    _contacts_of,
    _continuous_support_intervals,
    build_domain_geometry,
    resolve_component_alphas,
)
from cftuv_envelope.reference.common import GeometryContext
from cftuv_envelope.reference.domain_geometry import BlockingBoundarySegment
from cftuv_envelope.reference.validation import validate_compilation_geometry_payload
from cftuv_envelope.wavefront import (
    ConveyorOutcome,
    conveyor_coverage,
    prepare_conveyor,
)

from reference_factories import straight_snapshot
from wavefront_cases import FIELD_FIXTURE


HOLE_RING = (
    ((0.0, 0.0), (4.0, 0.0), (4.0, 3.0), (4.0, 5.0), (4.0, 10.0), (0.0, 10.0)),
    ((6.0, 0.0), (10.0, 0.0), (10.0, 10.0), (6.0, 10.0), (6.0, 5.0), (6.0, 3.0)),
    ((4.0, 0.0), (6.0, 0.0), (6.0, 3.0), (4.0, 3.0)),
    ((4.0, 5.0), (6.0, 5.0), (6.0, 10.0), (4.0, 10.0)),
)
CONCAVE = (
    (
        (0.0, 0.0),
        (10.0, 0.0),
        (10.0, 10.0),
        (6.0, 10.0),
        (6.0, 3.0),
        (4.0, 3.0),
        (4.0, 10.0),
        (0.0, 10.0),
    ),
)
ALPHAS = ("0.5", "1", "2", "3", "4", "5", "7")


def _geometry(snapshot, request):
    compiled = compile_reference_envelopes(snapshot, request)
    assert compiled.compilation is not None
    frame, _ = validate_compilation_geometry_payload(compiled.compilation)
    context = GeometryContext.build(compiled.compilation, frame)
    return context, build_domain_geometry(context)


def _hole_ring():
    return straight_snapshot(
        faces=HOLE_RING,
        source_routes=(
            {
                "name": "bottom",
                "points": ((0.0, 0.0), (4.0, 0.0), (6.0, 0.0), (10.0, 0.0)),
            },
            {
                "name": "top",
                "points": ((10.0, 10.0), (6.0, 10.0), (4.0, 10.0), (0.0, 10.0)),
            },
        ),
        alpha="4",
    )


def _concave():
    return straight_snapshot(
        faces=CONCAVE,
        source_routes=({"name": "source", "points": ((0.0, 0.0), (10.0, 0.0))},),
        alpha="4",
    )


def _sources(context):
    """Опорные интервалы всех Strip-спек — ровно то, что обходит резолвер."""

    for spec in sorted(
        (
            item
            for item in context.compilation.envelope_specs
            if isinstance(item, StripEnvelopeSpec)
        ),
        key=lambda item: item.envelope_spec_id.value,
    ):
        seed = next(
            item
            for item in context.compilation.seeds
            if getattr(item, "seed_id", None) == spec.source_seed_id
        )
        yield from _continuous_support_intervals(
            context,
            context.support_segments_for_use(
                seed.chain_use_id, spec.envelope_spec_id.value
            ),
        )


def test_the_answer_is_bitwise_the_same_with_and_without_the_memo():
    for build in (_hole_ring, _concave):
        context, domain = _geometry(*build())
        memo = ContactCandidatesMemoV1()
        outcomes = set()
        for alpha in ALPHAS:
            value = LocalLengthV1(Decimal(alpha))
            plain = resolve_component_alphas(context, value, domain)
            remembered = resolve_component_alphas(context, value, domain, memo)
            again = resolve_component_alphas(context, value, domain, memo)
            assert remembered == plain, (build.__name__, alpha)
            assert again == plain, (build.__name__, alpha)
            outcomes.update(
                item.capacity_outcome for item in plain[0].values()
            )
        # Контрольная проверка различения: контакты в этих случаях РЕШАЮТ — без
        # названного исхода ёмкости равенство выше не отличило бы память от пустышки.
        assert any(item is not None for item in outcomes), build.__name__


def test_the_memo_is_real_and_the_second_pass_computes_nothing():
    context, domain = _geometry(*_hole_ring())
    memo = ContactCandidatesMemoV1()
    resolve_component_alphas(context, LocalLengthV1(Decimal("4")), domain, memo)

    assert memo.entries
    assert memo.computed == len(memo.entries) > 0

    # Меньшая alpha берёт подмножество контактов и длин, посчитанных при большей.
    first, reused = memo.computed, memo.reused
    resolve_component_alphas(context, LocalLengthV1(Decimal("2")), domain, memo)

    assert memo.computed == first
    assert memo.reused > reused


def test_the_key_carries_the_geometry_not_only_the_segment_name():
    context, domain = _geometry(*_hole_ring())
    source = next(_sources(context))
    memo = ContactCandidatesMemoV1()
    checked = 0
    for boundary in domain.blocking_segments:
        reverse = BlockingBoundarySegment(
            boundary.segment.reversed(), boundary.role, boundary.concave_vertex_keys
        )
        assert reverse.segment.segment_id == boundary.segment.segment_id
        direct = _contacts_of(context, source, boundary, memo)
        mirrored = _contacts_of(context, source, reverse, memo)
        assert direct == _contact_candidates(context, source, boundary)
        assert mirrored == _contact_candidates(context, source, reverse)
        checked += 1
    assert checked > 4
    # Каждый отрезок и его зеркало — две записи, а не одна на имя.
    assert memo.computed == 2 * checked


def _field_preparation():
    root = FIELD_FIXTURE.parent
    snapshot = kernel.AnalysisSnapshotCodecV1.loads(
        (root / "analysis_snapshot.json").read_bytes()
    )
    request = kernel.DecalRequestCodecV1.loads(
        (root / "decal_request.json").read_bytes()
    )
    return prepare_conveyor(snapshot, request)


def _contact_entries(memo) -> int:
    return sum(1 for key in memo.entries if key[0] == "contacts")


def _answer(coverage):
    return (
        coverage.outcome,
        coverage.alpha,
        coverage.doubled_area,
        coverage.counters,
        tuple(
            (
                region.region_id,
                region.outcome,
                region.doubled_area,
                tuple(
                    (face.owner, face.envelope_spec_id, face.envelope_instance_id)
                    for face in region.faces
                ),
            )
            for region in coverage.regions
        ),
    )


def test_the_field_preparation_answers_the_same_and_fills_once():
    prepared = _field_preparation()
    assert prepared.outcome is ConveyorOutcome.EXACT, prepared.detail
    assert prepared.contact_memo is not None and not prepared.contact_memo.entries
    bare = replace(prepared, contact_memo=None)

    first = conveyor_coverage(prepared, "0.25")
    assert _answer(first) == _answer(conveyor_coverage(bare, "0.25"))
    memo = prepared.contact_memo
    contacts = _contact_entries(memo)
    assert contacts > 0

    for alpha in ("0.5", "2", "7"):
        assert _answer(conveyor_coverage(prepared, alpha)) == _answer(
            conveyor_coverage(bare, alpha)
        )
    # Контакты `(источник, граница)` алфа-независимы: все посчитаны на первом
    # покрытии. Длины источников идут лениво (нужны лишь контактам внутри alpha).
    assert _contact_entries(memo) == contacts
    assert memo.reused > 0
    computed = memo.computed
    conveyor_coverage(prepared, "7")
    assert memo.computed == computed


def test_the_memo_travels_with_the_preparation_and_the_copy_computes_nothing():
    prepared = _field_preparation()
    before = _answer(conveyor_coverage(prepared, "0.25"))
    assert prepared.contact_memo.entries

    clone = pickle.loads(pickle.dumps(prepared, protocol=pickle.HIGHEST_PROTOCOL))

    assert clone.contact_memo is not prepared.contact_memo
    assert len(clone.contact_memo.entries) == len(prepared.contact_memo.entries)
    # Счётчики не едут: копия считает свои, а не выдаёт чужие числа за свои.
    assert (clone.contact_memo.computed, clone.contact_memo.reused) == (0, 0)
    assert _answer(conveyor_coverage(clone, "0.25")) == before
    after_other = _answer(conveyor_coverage(clone, "0.5"))
    assert clone.contact_memo.computed == 0
    assert clone.contact_memo.reused > 0
    assert after_other == _answer(conveyor_coverage(prepared, "0.5"))
    assert after_other != before
