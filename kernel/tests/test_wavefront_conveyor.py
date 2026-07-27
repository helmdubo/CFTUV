"""Публичный вход очереди: те же числа, что у стенда, но без союза.

Срез отвечает на один вопрос: можно ли отдать хосту покрытие с owner'ами, НЕ
платя за союз юбок. Ответ — да, и он разложен по тестам так, чтобы каждое
утверждение стояло на своём числе:

- `test_the_public_entry_reproduces_the_field_numbers_without_the_union`
  — те же 3 источника / 9 стен / 10 узлов / 3 грани `[2, 2, 8]` / сумма
  108 901 947 644, что мерил стенд;
- `test_the_public_entry_never_calls_the_exact_union`
  — союз не вызывается НИ РАЗУ, и это счёт вызовов, а не отсутствие стадии в
  таймингах: стадию можно забыть объявить, вызов — нет;
- `test_the_arrival_laws_are_the_same_set_as_the_clipped_path_produces`
  — разрез до клипа законен: множество законов совпадает с тем, что даёт
  `compile_arrival_models` ПОСЛЕ клипа и союза;
- `test_the_preparation_does_not_depend_on_alpha_and_the_instance_name_does`
  — гипотеза alpha-независимости подтверждена для геометрии и ОПРОВЕРГНУТА
  для имени экземпляра, с числами по обеим половинам;
- `test_the_rational_translation_is_exact_where_nsimplify_silently_truncated`
  — перевод в дроби чинится не только по цене: прежний рецепт на
  иррациональном входе врал молча.
"""

from __future__ import annotations

from dataclasses import replace
from decimal import Decimal
from fractions import Fraction
from pathlib import Path

import pytest
import sympy as sp

import cftuv_envelope as kernel
from cftuv_envelope.interactions.arrival import compile_arrival_models
from cftuv_envelope.interactions.components import compile_interaction_components
from cftuv_envelope.numeric import LocalLengthV1, MetricSpace
from cftuv_envelope.reference.arrangement import ExactSegmentArrangementBackend
from cftuv_envelope.reference.boundary import build_domain_geometry
from cftuv_envelope.reference.common import GeometryContext
from cftuv_envelope.reference.validation import (
    validate_reference_geometry_payload,
)
from cftuv_envelope.wavefront import (
    ConveyorOutcome,
    chart_lattice_for_frame,
    conveyor_coverage,
    evaluate_conveyor_coverage,
    prepare_conveyor,
)
from cftuv_envelope.wavefront.bridge import BridgeOutcome
from cftuv_envelope.wavefront.coverage import CoverageOutcome
from cftuv_envelope.wavefront.conveyor import exact_rational
from cftuv_envelope.wavefront.digest import semantic_digest
from cftuv_envelope.wavefront.faces import FaceOutcome
from cftuv_envelope.wavefront.skeleton import SkeletonOutcome
from cftuv_envelope.wavefront.sqrt_sum import SqrtSumV1

from reference_factories import straight_snapshot
from wavefront_cases import FIELD_FIXTURE


FIXTURE = FIELD_FIXTURE.parent


def _field_input():
    snapshot = kernel.AnalysisSnapshotCodecV1.loads(
        (FIXTURE / "analysis_snapshot.json").read_bytes()
    )
    request = kernel.DecalRequestCodecV1.loads(
        (FIXTURE / "decal_request.json").read_bytes()
    )
    return snapshot, request


@pytest.fixture(scope="module")
def field_preparation():
    """Подготовка `bf6` один раз на модуль — она стоит 59 мс и не зависит от alpha.

    Кэш здесь не удобство теста, а само проверяемое свойство: если бы
    подготовка от alpha зависела, разделять её между тестами с разными alpha
    было бы нельзя. Независимость доказывается отдельным тестом, а этот кэш
    падал бы вместе с ней.
    """

    snapshot, request = _field_input()
    return prepare_conveyor(snapshot, request)


# --------------------------------------------------------------------------
# 1. Полевые числа воспроизводятся, и союз при этом не считается
# --------------------------------------------------------------------------


def test_the_public_entry_reproduces_the_field_numbers_without_the_union(
    field_preparation,
):
    """Отображение и счёт `bf6` те же, что мерил стенд через `RawCoverage`.

    Каждое число здесь уже стояло в `DECISIONS.md` за 2026-07-27 и получено
    там ЧЕРЕЗ союз. Совпадение показывает, что союз в этот ответ не входил:

    | величина | число |
    |---|---:|
    | регионов домена | 1 |
    | законов прихода | 3 |
    | рёбер домена / источников / стен | 12 / 3 / 9 |
    | неопределённых владельцев | 0 |
    | масштаб решётки карты | 65536 |
    | узлов скелета | 10 |
    | граней | 3, по узлам `[2, 2, 8]` |
    | удвоенная площадь домена | 108 901 947 644 |
    """

    prepared = field_preparation
    assert prepared.outcome is ConveyorOutcome.EXACT, prepared.detail
    assert dict(prepared.counters) == {
        "CONVEYOR_DOMAIN_REGIONS": 1,
        "CONVEYOR_ARRIVAL_LAWS": 3,
        "CONVEYOR_LATTICE_SCALE": 65536,
        "CONVEYOR_DOMAIN_EDGES": 12,
        "CONVEYOR_SOURCE_EDGES": 3,
        "CONVEYOR_WALL_EDGES": 9,
        "CONVEYOR_AMBIGUOUS_OWNER_EDGES": 0,
        "CONVEYOR_WEIGHTED_SOURCE_EDGES": 3,
        "CONVEYOR_SKELETON_NODES": 10,
        "CONVEYOR_FACES": 3,
    }

    (region,) = prepared.regions
    assert region.bridge_outcome is BridgeOutcome.EXACT
    assert region.skeleton_outcome is SkeletonOutcome.EXACT
    assert region.face_outcome is FaceOutcome.EXACT
    assert region.bridge.findings == ()
    assert len(region.skeleton.nodes) == 10
    assert sorted(face.node_count for face in region.partition.faces) == [2, 2, 8]
    assert region.partition.polygon_doubled_area == 108901947644
    assert region.partition.area_defect.terms == ()
    # Стены названы поимённо, а не только сосчитаны: девять рёбер без грани.
    assert region.wall_edge_count == 9
    assert len(region.wall_spans) == 9
    assert region.ambiguous_owner_spans == ()
    # Ни одно ребро не бывает разом источником и стеной.
    owners = {edge for edge, _ in region.owner_by_edge}
    assert len(owners) == 3
    assert owners.isdisjoint(set(region.wall_spans))


def test_the_public_entry_never_calls_the_exact_union(field_preparation):
    """Союз не вызывается НИ РАЗУ — счётом вызовов, а не отсутствием стадии.

    Отсутствие стадии в таймингах доказывало бы только то, что её забыли
    объявить. Здесь подменяется сам метод КЛАССА `ExactSegmentArrangementBackend`,
    поэтому под счётчик попадают все четыре его экземпляра в ядре
    (`reference/raw_coverage.py`, `interactions/arrival.py`,
    `interactions/policy_b.py`, `reference/domain_oracle.py`) — то есть
    единственный способ посчитать союз в этом процессе.

    Положительный контроль обязателен: без него «ноль вызовов» неотличимо от
    «счётчик не работает». Эталонный путь на том же входе вызывает союз ЧЕТЫРЕ
    раза: по одному на клип каждой из трёх юбок о домен плюс сам `RAW_UNION`.
    """

    snapshot, request = _field_input()
    calls: list[str] = []
    original = ExactSegmentArrangementBackend.exact_union

    def counting(self, *args, **kwargs):
        calls.append(type(self).__name__)
        return original(self, *args, **kwargs)

    ExactSegmentArrangementBackend.exact_union = counting
    try:
        covered = evaluate_conveyor_coverage(snapshot, request)
        conveyor_calls = len(calls)
        calls.clear()
        compiled = kernel.compile_reference_envelopes(snapshot, request)
        kernel.evaluate_reference_raw_coverage(
            compiled.compilation, request.requested_alpha
        )
        reference_calls = len(calls)
    finally:
        ExactSegmentArrangementBackend.exact_union = original

    assert covered.outcome is ConveyorOutcome.EXACT, covered.detail
    assert conveyor_calls == 0
    # Положительный контроль: счётчик ловит союз там, где он есть.
    assert reference_calls == 4
    # И то же самое видно с другой стороны: у входа нет стадии союза вовсе.
    stages = {stage for stage, _ in field_preparation.timings}
    assert "RAW_UNION" not in stages and "DOMAIN_CLIP" not in stages
    assert stages == {
        "PLAN_COMPILE",
        "DOMAIN_BUILD",
        "ARRIVAL_LAWS",
        "CHART_LATTICE",
        "DOMAIN_LOOPS",
        "BRIDGE",
        "SKELETON",
        "FACES",
    }


# --------------------------------------------------------------------------
# 2. Законность разреза: те же законы, что даёт путь после клипа
# --------------------------------------------------------------------------


def test_the_arrival_laws_are_the_same_set_as_the_clipped_path_produces():
    """Разрез ДО клипа законен: множество законов не меняется, цена меняется.

    Проверяется именно МНОЖЕСТВО четвёрок `(n_x, n_y, c, s^2)`, а не их число
    и не сумма: совпадение суммы совпадения множества не доказывает. Полный
    путь при этом проходит через `resolve_component_alphas`, построение юбок,
    их клип о домен и союз (`compile_arrival_models` зовёт `exact_union` на
    каждый компонент), а вход очереди — ни через что из этого.

    Разница между путями всё-таки есть, и она названа здесь же: полный путь
    ОТБИРАЕТ законы по признаку «у юбки есть сегмент на фронте при эффективной
    alpha», вход очереди берёт все опорные сегменты. На `bf6` отбор не
    отбрасывает ничего — по одному опорному сегменту на каждую из трёх
    Strip-спек, — и это проверяется числом, а не предполагается.
    """

    snapshot, request = _field_input()
    compiled = kernel.compile_reference_envelopes(snapshot, request)
    compilation = compiled.compilation
    raw = kernel.evaluate_reference_raw_coverage(
        compilation, request.requested_alpha
    ).raw_coverage
    boundary_resolved = tuple(
        sorted(
            raw.boundary_resolved_envelopes,
            key=lambda item: item.envelope_instance.envelope_instance_id,
        )
    )
    components = compile_interaction_components(compilation, boundary_resolved)
    models, diagnostics = compile_arrival_models(
        compilation, components, boundary_resolved
    )
    assert diagnostics == ()
    clipped_laws = {
        (
            exact_rational(model.arrival_law.normal.expressions()[0]),
            exact_rational(model.arrival_law.normal.expressions()[1]),
            exact_rational(model.arrival_law.source_constant.as_expr()),
            exact_rational(model.arrival_law.normal_speed.as_expr()) ** 2,
        )
        for model in models
    }

    frame, _ = validate_reference_geometry_payload(
        compilation.analysis_snapshot, compilation.plan_key.patch_domain_id
    )
    context = GeometryContext.build(compilation, frame)
    # Вход очереди строит законы этой же функцией — она одна на оба пути.
    from cftuv_envelope.wavefront.conveyor import _arrival_laws

    laws, issue = _arrival_laws(context)
    assert issue is None
    queue_laws = {
        (law.normal_x, law.normal_y, law.constant, law.speed_squared)
        for law in laws
    }

    assert len(models) == 3
    assert len(laws) == 3
    assert queue_laws == clipped_laws


# --------------------------------------------------------------------------
# 3. Покрытие при alpha запроса
# --------------------------------------------------------------------------


def test_the_coverage_at_the_requested_alpha_is_exact_and_owned(
    field_preparation,
):
    """Покрытие `bf6` при alpha запроса, и у каждого куска назван владелец.

    Числа те же, что стенд получил через союз:

    | величина | число |
    |---|---|
    | alpha запроса | 1/4, в единицах решётки 16384 |
    | исход | `EXACT` |
    | членов в каноническом наборе | 6 |
    | оболочка удвоенной площади | строго в (3018411397, 3018411399) |
    | граней с владельцем | 3, все имена различны |
    | стен | 9 |

    Иррациональное число читается ЦЕЛИКОМ оболочкой, а не по частям: сумма
    членов совпадения множества не доказала бы, а оболочка — свойство самого
    значения.
    """

    covered = conveyor_coverage(field_preparation)
    assert covered.outcome is ConveyorOutcome.EXACT, covered.detail
    assert covered.alpha == Fraction(1, 4)
    assert covered.lattice_alpha == 16384
    (region,) = covered.regions
    assert region.outcome is CoverageOutcome.EXACT
    assert len(covered.doubled_area.terms) == 6
    low, high = covered.doubled_area.enclosure(80)
    assert 3018411397 < low <= high < 3018411399
    assert covered.doubled_area.sign() > 0
    # Покрытие строго внутри домена.
    assert (
        SqrtSumV1.rational(covered.polygon_doubled_area) - covered.doubled_area
    ).sign() > 0
    assert covered.polygon_doubled_area == 108901947644

    assert dict(covered.counters) == {
        "CONVEYOR_COVERED_REGIONS": 1,
        "CONVEYOR_COVERED_FACES": 3,
        "CONVEYOR_COVERAGE_TERMS": 6,
        "CONVEYOR_NAMED_OWNERS": 3,
        "CONVEYOR_WALL_EDGES": 9,
    }
    faces = covered.faces
    assert len(faces) == 3
    assert len({face.envelope_instance_id for face in faces}) == 3
    assert len({face.envelope_spec_id for face in faces}) == 3
    assert all(
        face.envelope_instance_id.startswith("envelope-instance:")
        for face in faces
    )


def test_the_owner_names_are_the_very_instances_the_reference_path_builds():
    """Имена владельцев — НЕ свои, а те самые, что строит эталонный путь.

    Без этой сверки «три различных имени» доказывало бы только различность.
    Имя экземпляра выводится `strip_envelope_instance_id` — одной функцией на
    оба пути, — поэтому совпадение здесь проверяет проводку, а не совпадение
    двух хешей по случайности.
    """

    snapshot, request = _field_input()
    covered = evaluate_conveyor_coverage(snapshot, request)
    compiled = kernel.compile_reference_envelopes(snapshot, request)
    raw = kernel.evaluate_reference_raw_coverage(
        compiled.compilation, request.requested_alpha
    ).raw_coverage

    reference_ids = {
        item.envelope_instance.envelope_instance_id
        for item in raw.boundary_resolved_envelopes
    }
    queue_ids = {face.envelope_instance_id for face in covered.faces}
    assert len(queue_ids) == 3
    assert queue_ids <= reference_ids
    # Эталон строит девять экземпляров (3 Strip + 6 Cap), источником фронта
    # служат три. Разница названа числом, чтобы «подмножество» не читалось как
    # «а вдруг там пусто».
    assert len(reference_ids) == 9


# --------------------------------------------------------------------------
# 4. Обе гипотезы карточки: подтверждение и опровержение, каждое числом
# --------------------------------------------------------------------------


def test_the_preparation_does_not_depend_on_alpha_and_the_instance_name_does(
    field_preparation,
):
    """Гипотеза alpha-независимости: подтверждена для геометрии, опровергнута
    для имени экземпляра.

    ПОДТВЕРЖДЕНО. При alpha запроса 0.25 и 0.5 подготовка совпадает ПОБИТОВО:
    `semantic_digest` скелета, состав граней, `owner_by_edge`, все счётчики.
    Поэтому `prepare` и вынесен в отдельный вход и кэшируем.

    ОПРОВЕРГНУТО, и вот где именно. `envelope_instance_id` стоит на
    ЭФФЕКТИВНОЙ alpha (`strip_envelope_instance_id`), поэтому при 0.25 и 0.5
    все три имени различны — три из трёх. Значит имя владельца принадлежит
    alpha-зависимой ступени, а alpha-независимая знает владельца только по
    `envelope_spec_id`. Стоимость этого вывода — стадия `EFFECTIVE_ALPHA`
    внутри `coverage`, и она названа отдельной строкой таймингов.
    """

    snapshot, request = _field_input()
    other = replace(
        request,
        requested_alpha=LocalLengthV1(
            Decimal("0.5"), MetricSpace.SOURCE_LOCAL_INTRINSIC
        ),
    )
    prepared_quarter = field_preparation
    prepared_half = prepare_conveyor(snapshot, other)
    assert prepared_half.outcome is ConveyorOutcome.EXACT

    (quarter,) = prepared_quarter.regions
    (half,) = prepared_half.regions
    assert semantic_digest(quarter.skeleton) == semantic_digest(half.skeleton)
    assert quarter.owner_by_edge == half.owner_by_edge
    assert quarter.wall_spans == half.wall_spans
    assert prepared_quarter.counters == prepared_half.counters
    assert [face.owner for face in quarter.partition.faces] == [
        face.owner for face in half.partition.faces
    ]
    assert (
        quarter.partition.polygon_doubled_area
        == half.partition.polygon_doubled_area
    )

    # Опровержение: одна и та же подготовка, две alpha, ТРИ различных имени.
    at_quarter = conveyor_coverage(prepared_quarter, "0.25")
    at_half = conveyor_coverage(prepared_quarter, "0.5")
    assert at_quarter.outcome is ConveyorOutcome.EXACT
    assert at_half.outcome is ConveyorOutcome.EXACT
    quarter_names = {face.envelope_instance_id for face in at_quarter.faces}
    half_names = {face.envelope_instance_id for face in at_half.faces}
    assert len(quarter_names) == len(half_names) == 3
    assert quarter_names.isdisjoint(half_names)
    # Владелец по спеке при этом ОДИН И ТОТ ЖЕ: alpha-независимая половина
    # имени существует, и она названа.
    assert {face.envelope_spec_id for face in at_quarter.faces} == {
        face.envelope_spec_id for face in at_half.faces
    }
    # Разбиение одно на обе alpha, а покрытие растёт.
    assert at_quarter.polygon_doubled_area == at_half.polygon_doubled_area
    assert (at_half.doubled_area - at_quarter.doubled_area).sign() > 0
    # Цена имени названа стадией, а не спрятана в общем времени.
    assert {stage for stage, _ in at_quarter.timings} == {
        "EFFECTIVE_ALPHA",
        "COVERAGE_CLIP",
    }


def test_the_rational_translation_is_exact_where_nsimplify_silently_truncated():
    """Перевод в дроби чинится не только по цене: прежний рецепт врал молча.

    Прежний хелпер стенда переводил через `nsimplify`:

        numerator, denominator = sp.fraction(sp.nsimplify(expression))
        Fraction(int(numerator), int(denominator))

    На `sqrt(2)/2` он не падает, а возвращает `1/2`: `sp.fraction` даёт
    `(sqrt(2), 2)`, а `int(sqrt(2))` есть 1. Ошибка 29 % и ни одного признака,
    что что-то произошло. `exact_rational` на том же входе отвечает `None`, и
    вызывающий обязан назвать исход.

    Цена тоже проверяется, но не секундомером — секундомер в тесте мерил бы
    машину. Проверяется РАВЕНСТВО на всех рациональных значениях фикстуры:
    именно оно и позволило `nsimplify` убрать (304 мс на 36 значений против
    0.1 мс, замер в отчёте среза).
    """

    def by_nsimplify(expression) -> Fraction:
        numerator, denominator = sp.fraction(sp.nsimplify(expression))
        return Fraction(int(numerator), int(denominator))

    # 1. Молчаливое усечение прежнего рецепта — воспроизведено, а не заявлено.
    assert by_nsimplify(sp.sqrt(2) / 2) == Fraction(1, 2)
    assert exact_rational(sp.sqrt(2) / 2) is None
    assert exact_rational(sp.sqrt(2)) is None
    assert exact_rational(sp.pi) is None

    # 2. На рациональных значениях фикстуры оба пути дают одно и то же.
    snapshot, request = _field_input()
    compiled = kernel.compile_reference_envelopes(snapshot, request)
    frame, _ = validate_reference_geometry_payload(
        compiled.compilation.analysis_snapshot,
        compiled.compilation.plan_key.patch_domain_id,
    )
    context = GeometryContext.build(compiled.compilation, frame)
    domain = build_domain_geometry(context)
    (region,) = domain.domain_regions
    expressions = [point.x.as_expr() for point in region.outer.points]
    expressions += [point.y.as_expr() for point in region.outer.points]
    assert len(expressions) == 24
    assert [exact_rational(item) for item in expressions] == [
        by_nsimplify(item) for item in expressions
    ]
    assert all(exact_rational(item) is not None for item in expressions)


# --------------------------------------------------------------------------
# 5. Именованные исходы всех ступеней. Тихих нет.
# --------------------------------------------------------------------------


def test_every_domain_region_goes_through_the_queue_and_the_count_is_named(
    field_preparation,
):
    """Все регионы домена, а не первый, — и сколько их, следует из контракта.

    Хелпер стенда брал `domain_regions[0]`, и проверить это было нечем.
    Измерено: `SparsePatchDomainGeometryV1.domain_regions` возвращает РОВНО
    ОДИН регион по построению и отказывает `REFERENCE_INPUT_CONTRACT_INVALID`
    при нескольких внешних петлях. То есть второго региона в v1 не бывает
    вовсе — и это утверждение о контракте, а не о фикстуре.

    Цикл во входе от этого не лишний: он снимает молчаливую зависимость от
    контракта, который может измениться. Проверяется здесь и то, и другое:
    контракт — прямо, число регионов — счётчиком.
    """

    assert field_preparation.counter("CONVEYOR_DOMAIN_REGIONS") == 1
    assert len(field_preparation.regions) == 1

    covered = conveyor_coverage(field_preparation)
    assert covered.counter("CONVEYOR_COVERED_REGIONS") == 1
    # Сумма по регионам собирается сложением, а не берётся у первого.
    total = SqrtSumV1.zero()
    for region in covered.regions:
        total = total + region.doubled_area
    assert (total - covered.doubled_area).is_zero
    assert covered.polygon_doubled_area == sum(
        region.polygon_doubled_area for region in covered.regions
    )


@pytest.mark.parametrize(
    "name,face,expected",
    (
        (
            "triangle",
            ((0, 0), (12, 0), (0, 12)),
            ConveyorOutcome.ARRIVAL_LAW_IS_NOT_RATIONAL,
        ),
        (
            "square",
            ((0, 0), (8, 0), (8, 8), (0, 8)),
            ConveyorOutcome.CHART_LATTICE_IS_NOT_DECLARED,
        ),
    ),
)
def test_a_step_that_cannot_proceed_answers_by_name_not_by_crash(
    name, face, expected
):
    """Отказы достижимы на настоящем входе корпуса и приходят ИМЕНЕМ.

    Оба случая найдены замером, а не придуманы:

    | вход | исход | почему |
    |---|---|---|
    | треугольник `straight_snapshot` | `ARRIVAL_LAW_IS_NOT_RATIONAL` | нормаль гипотенузы иррациональна |
    | квадрат `straight_snapshot` | `CHART_LATTICE_IS_NOT_DECLARED` | кадр `PlanarPatchFrameV1` без сертификата решётки |

    Второй до этого теста был не отказом, а `AttributeError` — то есть падением
    без имени. Это и есть та причина, по которой перечень исходов пишется от
    прогона, а не от чтения.
    """

    routes = tuple(
        {
            "name": f"e{index}",
            "points": (face[index], face[(index + 1) % len(face)]),
        }
        for index in range(len(face))
    )
    snapshot, request = straight_snapshot(
        faces=(face,),
        source_routes=routes,
        alpha="1",
        revision_name=f"conveyor-{name}",
    )
    prepared = prepare_conveyor(snapshot, request)
    assert prepared.outcome is expected
    assert prepared.detail
    # Ступень покрытия не притворяется, что покрытие есть.
    covered = conveyor_coverage(prepared)
    assert covered.outcome is ConveyorOutcome.PREPARATION_IS_NOT_EXACT
    assert covered.detail == expected.value
    assert covered.faces == ()
    assert covered.doubled_area.is_zero
    # Время отказавшей ступени всё равно записано: стадия без числа — это
    # стадия, про которую нечего сказать.
    assert prepared.timings
    assert all(elapsed >= 0.0 for _, elapsed in prepared.timings)


def test_the_chart_lattice_is_derived_from_the_certificate_and_not_chosen():
    """Масштаб решётки выведен законом; отсутствие закона — `None`, не догадка."""

    snapshot, request = _field_input()
    compiled = kernel.compile_reference_envelopes(snapshot, request)
    frame, _ = validate_reference_geometry_payload(
        compiled.compilation.analysis_snapshot,
        compiled.compilation.plan_key.patch_domain_id,
    )
    lattice = chart_lattice_for_frame(frame)
    assert lattice is not None
    assert lattice.scale == 65536

    class _FrameWithoutCertificate:
        pass

    assert chart_lattice_for_frame(_FrameWithoutCertificate()) is None
