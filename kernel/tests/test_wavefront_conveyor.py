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

Второй полевой домен (`building_002_weighted_normals_v1`) добавлен позже и
отвечает на другой вопрос: что делать с законом, у которого нормаль
ИРРАЦИОНАЛЬНА при единичной скорости. Ответ — переписать запись, а не задачу:

- `test_the_weighted_normals_domain_needs_the_rescale_and_then_closes`
  — домен доходит до `EXACT`, спасено масштабом 2 закона из 4;
- `test_the_weighted_normals_coverage_matches_an_independent_closed_form`
  — покрытие сверено с замкнутой формулой площади, а не с самим собой;
- `test_the_rescale_changes_the_record_and_not_the_geometry`
  — класс прямой и отношение `(s/|n|)^2` переживают смену записи точно;
- `test_a_record_no_single_factor_can_rationalise_is_still_refused`
  — прежний отказ достижим и после починки.

Сильнейший критерий — равенство с ЭТАЛОННЫМ `RawCoverage`, и он разобран на два
теста, потому что ответ у него разный и оба ответа измерены:

- `test_the_queue_equals_the_reference_raw_coverage_when_the_source_is_whole`
  — на `building_002_weighted_normals_v1` (4 источника из 4, стен ноль)
  разность ДОКАЗАННО нулевая;
- `test_the_queue_exceeds_the_reference_raw_coverage_on_a_partial_source`
  — на `bf6` (3 источника из 12, девять стен) очередь накрывает НА 7.00 %
  больше, и это объявленная разница моделей: юбка обрезана концами своего
  ребра, отрезок фронта едет вдоль неподвижной прямой стены дальше.
"""

from __future__ import annotations

from contextlib import contextmanager
from dataclasses import replace
from decimal import Decimal
from fractions import Fraction

import pytest
import sympy as sp

import cftuv_envelope as kernel
from cftuv_envelope.contracts.envelopes import StripEnvelopeSpec
from cftuv_envelope.interactions import arrival as arrival_module
from cftuv_envelope.interactions import policy_b as policy_b_module
from cftuv_envelope.interactions.arrival import (
    STRIP_FRONT_NORMAL_SPEED,
    compile_arrival_models,
    strip_front_support_line,
)
from cftuv_envelope.interactions.components import compile_interaction_components
from cftuv_envelope.numeric import LocalLengthV1, MetricSpace
from cftuv_envelope.reference import domain_oracle as domain_oracle_module
from cftuv_envelope.reference import raw_coverage as raw_coverage_module
from cftuv_envelope.reference.arrangement import ExactSegmentArrangementBackend
from cftuv_envelope.reference.boundary import build_domain_geometry
from cftuv_envelope.reference.common import GeometryContext
from cftuv_envelope.reference.planar_types import ExactScalar, exact_sign
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
from cftuv_envelope.wavefront import conveyor as conveyor_module
from cftuv_envelope.wavefront.bridge import BridgeOutcome, line_class
from cftuv_envelope.wavefront.coverage import CoverageOutcome
from cftuv_envelope.wavefront.conveyor import (
    _arrival_laws,
    _rational_after_scaling,
    _read_arrival_law,
    exact_rational,
)
from cftuv_envelope.wavefront.digest import semantic_digest
from cftuv_envelope.wavefront.faces import FaceOutcome
from cftuv_envelope.wavefront.skeleton import SkeletonOutcome
from cftuv_envelope.wavefront.sqrt_sum import SqrtSumV1

from reference_factories import straight_snapshot
from wavefront_cases import FIELD_FIXTURE


FIXTURE = FIELD_FIXTURE.parent
# Второй полевой домен того же меша: у его опор нормаль ИРРАЦИОНАЛЬНА при
# единичной скорости, и до пере-масштабирования записи вход на нём отказывал.
WEIGHTED_FIXTURE = FIXTURE.parent / "building_002_weighted_normals_v1"


def _load(root):
    snapshot = kernel.AnalysisSnapshotCodecV1.loads(
        (root / "analysis_snapshot.json").read_bytes()
    )
    request = kernel.DecalRequestCodecV1.loads(
        (root / "decal_request.json").read_bytes()
    )
    return snapshot, request


def _field_input():
    return _load(FIXTURE)


def _weighted_input():
    return _load(WEIGHTED_FIXTURE)


def _union_singletons():
    """Все экземпляры бэкенда союза, живущие в ядре модульными одиночками.

    Список явный, а не найденный обходом: обход по `gc` поймал бы и чужие
    объекты, а пропуск здесь виден сразу — положительным контролем, который
    перестал бы считать.
    """

    return (
        raw_coverage_module.REFERENCE_ARRANGEMENT_BACKEND,
        arrival_module._ARRANGEMENT,
        policy_b_module._ARRANGEMENT,
        domain_oracle_module._ORACLE_BACKEND,
    )


@contextmanager
def _counting_unions():
    """Счётчик вызовов союза, который ловит и класс, и уже созданных одиночек.

    Подмены одного метода КЛАССА мало, и это не осторожность, а измеренная
    причина падения. `test_reference_envelopes.py` подменяет `exact_union` на
    ЭКЗЕМПЛЯРЕ `REFERENCE_ARRANGEMENT_BACKEND` через `monkeypatch.setattr`;
    отмена такой подмены не удаляет атрибут, а ЗАПИСЫВАЕТ в `__dict__`
    экземпляра связанный метод, снятый до подмены. Тень остаётся на весь
    процесс, и с этого момента подмена класса тому экземпляру не видна:
    полный прогон давал `0 == 4` при зелёном одиночном.

    Поэтому здесь подменяются оба уровня, а прежнее содержимое `__dict__`
    восстанавливается ровно таким, каким было, — включая его отсутствие.
    """

    calls: list[str] = []
    original = ExactSegmentArrangementBackend.exact_union

    def counting(self, *args, **kwargs):
        calls.append(type(self).__name__)
        return original(self, *args, **kwargs)

    shadows = [
        (backend, vars(backend).get("exact_union", _MISSING))
        for backend in _union_singletons()
    ]
    ExactSegmentArrangementBackend.exact_union = counting
    for backend, _ in shadows:
        backend.exact_union = counting.__get__(backend)
    try:
        yield calls
    finally:
        ExactSegmentArrangementBackend.exact_union = original
        for backend, previous in shadows:
            if previous is _MISSING:
                del backend.exact_union
            else:
                backend.exact_union = previous


_MISSING = object()


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


@pytest.fixture(scope="module")
def weighted_preparation():
    """Подготовка второго полевого домена — того, что требует масштаба записи."""

    snapshot, request = _weighted_input()
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
        # Ни один закон этой фикстуры не пришлось переписывать: все три
        # записи рациональны сами. Ноль здесь — половина утверждения о
        # пере-масштабировании, вторая половина (двойка) на фикстуре
        # `building_002_weighted_normals_v1`.
        "CONVEYOR_RESCALED_ARRIVAL_LAWS": 0,
        # ВЕЕРОВ У ЭТОГО ДОМЕНА НЕТ, и все четыре нуля означают одно и то же
        # измеренное обстоятельство: снапшот `building.002` не объявляет ни
        # одного `CornerRelation`, поэтому компилятор не строит ни одной
        # `AngularEnvelopeSpec`. Не «веер отклонён» (тогда стоял бы
        # `CONVEYOR_IRRATIONAL_VERTEX_FANS`) и не «угол митрованный по профилю»
        # (тогда `CONVEYOR_MITERED_CORNERS`), а «угла во входе нет вовсе».
        # Отсюда и числа ниже не сдвинулись после появления мягкого угла:
        # см. `test_wavefront_vertex_fan.py`.
        "CONVEYOR_RATIONAL_VERTEX_FANS": 0,
        "CONVEYOR_IRRATIONAL_VERTEX_FANS": 0,
        "CONVEYOR_MITERED_CORNERS": 0,
        "CONVEYOR_FAN_SUPPORTS": 0,
        "CONVEYOR_FAN_EDGES": 0,
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
    объявить. Здесь считаются ВЫЗОВЫ: подменяется и метод класса
    `ExactSegmentArrangementBackend`, и `exact_union` у всех четырёх его
    экземпляров-одиночек в ядре (`reference/raw_coverage.py`,
    `interactions/arrival.py`, `interactions/policy_b.py`,
    `reference/domain_oracle.py`). Почему обоих уровней, а не одного —
    в `_counting_unions`: это не запас, а починенное падение полного прогона.

    Положительный контроль обязателен: без него «ноль вызовов» неотличимо от
    «счётчик не работает». Эталонный путь на том же входе вызывает союз ЧЕТЫРЕ
    раза: по одному на клип каждой из трёх юбок о домен плюс сам `RAW_UNION`.
    """

    snapshot, request = _field_input()
    with _counting_unions() as calls:
        covered = evaluate_conveyor_coverage(snapshot, request)
        conveyor_calls = len(calls)
        calls.clear()
        compiled = kernel.compile_reference_envelopes(snapshot, request)
        kernel.evaluate_reference_raw_coverage(
            compiled.compilation, request.requested_alpha
        )
        reference_calls = len(calls)

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
    # Законы очереди берутся у самой очереди, а не пересобираются здесь:
    # пересборка проверяла бы формулу теста, а не формулу входа.
    reading = _arrival_laws(context)
    assert reading.detail is None
    # Записи этой фикстуры рациональны сами, поэтому сравнивать есть что:
    # пере-масштабирование сюда не вмешалось ни разу.
    assert reading.rescaled_count == 0
    laws = reading.laws
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
    "name,face,expected,rescaled",
    (
        (
            "triangle",
            ((0, 0), (12, 0), (0, 12)),
            ConveyorOutcome.CHART_LATTICE_IS_NOT_DECLARED,
            1,
        ),
        (
            "square",
            ((0, 0), (8, 0), (8, 8), (0, 8)),
            ConveyorOutcome.CHART_LATTICE_IS_NOT_DECLARED,
            0,
        ),
    ),
)
def test_a_step_that_cannot_proceed_answers_by_name_not_by_crash(
    name, face, expected, rescaled
):
    """Отказы достижимы на настоящем входе корпуса и приходят ИМЕНЕМ.

    Оба случая найдены замером, а не придуманы. Квадрат до появления этого
    теста отвечал не отказом, а `AttributeError` — падением без имени.

    ТРЕУГОЛЬНИК ПЕРЕЕХАЛ, и это измеренная находка, а не правка ожидания под
    зелёный. Он отвечал `ARRIVAL_LAW_IS_NOT_RATIONAL`, потому что нормаль
    гипотенузы иррациональна; пере-масштабирование записи чинит ровно это, и
    один его закон теперь спасается (`CONVEYOR_RESCALED_ARRIVAL_LAWS = 1`), а
    отказ сдвигается на СЛЕДУЮЩУЮ ступень — решётку карты. То есть починка,
    снятая с полевого домена, оказалась не частным случаем этого домена.

    | вход | исход | спасено масштабом |
    |---|---|---:|
    | треугольник | `CHART_LATTICE_IS_NOT_DECLARED` | 1 из 3 |
    | квадрат | `CHART_LATTICE_IS_NOT_DECLARED` | 0 из 4 |

    Счётчик на отказавшей подготовке виден потому, что `_refused` несёт то,
    что успел узнать: иначе «законы были рациональны» и «законы спасены, а
    споткнулись мы дальше» выглядели бы одинаково.

    `ARRIVAL_LAW_IS_NOT_RATIONAL` от этого не осиротел — он остался
    достижимым и проверяется прямо, на записи, которую не спасает никакой
    множитель: `test_a_record_no_single_factor_can_rationalise_is_still_refused`.
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
    assert prepared.counter("CONVEYOR_RESCALED_ARRIVAL_LAWS") == rescaled
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


@pytest.mark.parametrize(
    "alpha,kind",
    (
        (Fraction(1, 4), "Fraction"),
        (0.25, "float"),
    ),
    ids=("fraction", "float"),
)
def test_an_alpha_outside_the_declared_type_is_refused_by_name(
    field_preparation, alpha, kind
):
    """Чужой тип alpha отвергается ИМЕНЕМ, а не трассой из `decimal`.

    До этой проверки `conveyor_coverage(prepared, Fraction(1, 4))` падал голым
    `decimal.InvalidOperation` из `Decimal(str(alpha))` — сообщением про
    десятичный модуль вместо ответа про контракт. Соседний `coverage_at` в
    ЭТОМ ЖЕ пакете `Fraction` принимает, поэтому контраст обязан объясняться
    сам: адаптер хоста спотыкается о него первым.

    Оба типа отвергаются по одной причине, и это не вкус:

    | тип | что подменяется молча |
    |---|---|
    | `Fraction` | `1/3` в `Decimal` округляется до двадцати восьми девяток |
    | `float` | `0.1` есть `0.1000000000000000055511151231257827`, а `Decimal(str(0.1))` — ровно `1/10` |

    Дальше ядро считает ТОЧНО, поэтому подменённое значение уже не отличить от
    задуманного ничем.
    """

    with pytest.raises(TypeError) as refusal:
        conveyor_coverage(field_preparation, alpha)
    message = str(refusal.value)
    assert kind in message
    # Сообщение называет и контракт, и способ передать то же число без потерь.
    assert "LocalLengthV1, Decimal, int или str" in message
    assert '"0.25"' in message


def test_the_alpha_type_is_checked_before_the_preparation_is_paid_for():
    """Проверка типа стоит ДО подготовки, и это доказано счётом, а не временем.

    Секундомер здесь мерил бы машину. Считается другое: сколько раз
    `evaluate_conveyor_coverage` успел позвать компиляцию плана — первую же
    ступень подготовки. Ноль означает, что 53 мс скелета за чужой тип alpha не
    заплачены.
    """

    snapshot, request = _field_input()
    calls: list[int] = []
    original = conveyor_module.compile_reference_envelopes

    def counting(*args, **kwargs):
        calls.append(1)
        return original(*args, **kwargs)

    conveyor_module.compile_reference_envelopes = counting
    try:
        with pytest.raises(TypeError):
            evaluate_conveyor_coverage(snapshot, request, alpha=Fraction(1, 4))
        refused_calls = len(calls)
        # Положительный контроль: с объявленным типом компиляция ВЫЗЫВАЕТСЯ.
        covered = evaluate_conveyor_coverage(snapshot, request, alpha="0.25")
    finally:
        conveyor_module.compile_reference_envelopes = original

    assert refused_calls == 0
    assert len(calls) == 1
    assert covered.outcome is ConveyorOutcome.EXACT


def test_an_unreadable_alpha_string_is_refused_by_name_too():
    """Строка объявленного типа, но нечитаемая, — тоже именованный отказ.

    `Decimal("шире")` бросает `InvalidOperation`, и на границе это снова
    трасса про десятичный модуль. Тип здесь верный, поэтому исключение другое
    (`ValueError`, не `TypeError`), а требование то же.
    """

    snapshot, request = _field_input()
    with pytest.raises(ValueError) as refusal:
        evaluate_conveyor_coverage(snapshot, request, alpha="шире")
    message = str(refusal.value)
    assert "не читается десятичной записью" in message
    assert "LocalLengthV1, Decimal, int или str" in message


def test_every_declared_alpha_type_gives_the_very_same_coverage(
    field_preparation,
):
    """Положительный контроль контракта: объявленные типы принимаются и равны.

    Без него «отвергает Fraction и float» проходило бы и у входа, который
    отвергает вообще всё. Четыре записи одного числа — `None` (alpha запроса),
    строка, `Decimal` и `LocalLengthV1` — обязаны дать ПОБИТОВО одну площадь,
    а не близкую.
    """

    baseline = conveyor_coverage(field_preparation)
    assert baseline.outcome is ConveyorOutcome.EXACT
    for alpha in (
        "0.25",
        Decimal("0.25"),
        LocalLengthV1(Decimal("0.25"), MetricSpace.SOURCE_LOCAL_INTRINSIC),
    ):
        covered = conveyor_coverage(field_preparation, alpha)
        assert covered.outcome is ConveyorOutcome.EXACT
        assert covered.alpha == baseline.alpha
        assert covered.lattice_alpha == baseline.lattice_alpha
        assert covered.doubled_area.terms == baseline.doubled_area.terms
        assert [face.envelope_instance_id for face in covered.faces] == [
            face.envelope_instance_id for face in baseline.faces
        ]
    # `int` тоже объявлен, и он не должен отвергаться заодно с дробью.
    at_zero = conveyor_coverage(field_preparation, 0)
    assert at_zero.outcome is ConveyorOutcome.EXACT
    assert at_zero.lattice_alpha == 0
    assert at_zero.doubled_area.is_zero


# --------------------------------------------------------------------------
# 6. Иррациональная единичная нормаль: та же прямая в рациональной записи
# --------------------------------------------------------------------------


def test_the_weighted_normals_domain_needs_the_rescale_and_then_closes(
    weighted_preparation,
):
    """Полевой домен с иррациональной нормалью доходит до конца — через масштаб.

    ЧТО БЫЛО. `prepare_conveyor` отвечал
    `ARRIVAL_LAW_IS_NOT_RATIONAL` на `strip-spec:1e831f0c…`: полевой закон
    приходит с единичной скоростью (`s = 1`), и нормаль при этом
    `n = (-sqrt(426753013)/8192, 0)`, константа `c = -sqrt(426753013)/8192`.
    Два домена из тринадцати в поле падали ровно здесь.

    ЧТО СТАЛО. Деление записи на `|λ| = sqrt(426753013)/8192` даёт
    `n' = (-1, 0)`, `c' = -1`, `s'^2 = 67108864/426753013`. Прямая, движение и
    сторона те же; рациональны все четыре поля.

    | величина | число |
    |---|---:|
    | законов / спасено масштабом | 4 / **2** |
    | рёбер домена / источников / стен | 4 / 4 / 0 |
    | взвешенных рёбер-источников | 2 |
    | масштаб решётки | 65536 |
    | узлов скелета | 2 |
    | граней, по узлам | 4, `[1, 1, 2, 2]` |
    | удвоенная площадь домена | 8 589 934 592 |

    Гипотеза карточки «после рескейла домен решается до конца» —
    ПОДТВЕРЖДЕНА: следующей именованной ступени за снятым отказом не нашлось,
    мост, скелет и сборщик все три `EXACT`.
    """

    prepared = weighted_preparation
    assert prepared.outcome is ConveyorOutcome.EXACT, prepared.detail
    assert dict(prepared.counters) == {
        "CONVEYOR_DOMAIN_REGIONS": 1,
        "CONVEYOR_ARRIVAL_LAWS": 4,
        "CONVEYOR_RESCALED_ARRIVAL_LAWS": 2,
        # Вееров нет по той же причине, что и у соседней фикстуры: у домена нет
        # ни одного объявленного вогнутого угла. Ноль здесь — измерение входа,
        # а не тишина очереди.
        "CONVEYOR_RATIONAL_VERTEX_FANS": 0,
        "CONVEYOR_IRRATIONAL_VERTEX_FANS": 0,
        "CONVEYOR_MITERED_CORNERS": 0,
        "CONVEYOR_FAN_SUPPORTS": 0,
        "CONVEYOR_FAN_EDGES": 0,
        "CONVEYOR_LATTICE_SCALE": 65536,
        "CONVEYOR_DOMAIN_EDGES": 4,
        "CONVEYOR_SOURCE_EDGES": 4,
        "CONVEYOR_WALL_EDGES": 0,
        "CONVEYOR_AMBIGUOUS_OWNER_EDGES": 0,
        "CONVEYOR_WEIGHTED_SOURCE_EDGES": 2,
        "CONVEYOR_SKELETON_NODES": 2,
        "CONVEYOR_FACES": 4,
    }

    (region,) = prepared.regions
    assert region.bridge_outcome is BridgeOutcome.EXACT
    assert region.skeleton_outcome is SkeletonOutcome.EXACT
    assert region.face_outcome is FaceOutcome.EXACT
    assert region.bridge.findings == ()
    assert len(region.skeleton.nodes) == 2
    assert sorted(face.node_count for face in region.partition.faces) == [
        1,
        1,
        2,
        2,
    ]
    assert region.partition.polygon_doubled_area == 8589934592
    assert region.partition.area_defect.terms == ()
    # Источником служит КАЖДОЕ ребро домена, стен нет ни одной, и владелец у
    # каждого ребра свой — на квадрате это ровно четыре различных имени.
    assert region.wall_spans == ()
    assert len({name for _, name in region.owner_by_edge}) == 4
    assert region.ambiguous_owner_spans == ()


def test_the_weighted_normals_coverage_matches_an_independent_closed_form(
    weighted_preparation,
):
    """Покрытие домена сверено НЕ с самим собой, а с замкнутой формулой.

    Домен — квадрат со стороной 65536 (единица карты), источник на каждой
    стороне. Перпендикулярный ход фронта равен `alpha * s/|n|`: у верхней и
    нижней сторон это `16384`, у левой и правой — `16384 * 8192/sqrt(N)`,
    где `N = 426753013`. Непокрытым остаётся прямоугольник между фронтами,
    поэтому

        покрыто = L^2 - (L - 2*alpha) * (L - 2*d),  d = alpha*8192/sqrt(N)

    Это утверждение геометрии, а не очереди: оно выведено из четырёх
    полуплоскостей и не знает ни про скелет, ни про грани. Совпадение точное —
    разность канонических наборов пуста, а не мала.

    Числа: `EXACT`, **2 члена**, удвоенная площадь
    `4294967296 + 17592186044416/sqrt(426753013)`, то есть 59.91 % домена.
    """

    covered = conveyor_coverage(weighted_preparation)
    assert covered.outcome is ConveyorOutcome.EXACT, covered.detail
    assert covered.alpha == Fraction(1, 4)
    assert covered.lattice_alpha == 16384
    assert covered.polygon_doubled_area == 8589934592
    assert len(covered.doubled_area.terms) == 2

    radicand = 426753013
    side = 65536
    alpha = 16384
    rational_part = side * side - (side - 2 * alpha) * side
    radical_numerator = 2 * alpha * 8192 * (side - 2 * alpha)
    expected = SqrtSumV1.rational(2 * rational_part) + SqrtSumV1.radical(
        Fraction(2 * radical_numerator, radicand), radicand
    )
    assert (covered.doubled_area - expected).is_zero

    # Покрытие строго внутри домена и строго положительно.
    assert covered.doubled_area.sign() > 0
    assert (
        SqrtSumV1.rational(covered.polygon_doubled_area) - covered.doubled_area
    ).sign() > 0
    # Владелец назван у каждого из четырёх кусков, и все имена различны.
    assert len(covered.faces) == 4
    assert len({face.envelope_instance_id for face in covered.faces}) == 4
    assert len({face.envelope_spec_id for face in covered.faces}) == 4


def _chart_area(covered, prepared):
    """Площадь покрытия очереди в единицах КАРТЫ, символьным выражением.

    Очередь считает удвоенную площадь на решётке масштаба `lattice.scale`,
    эталон — площадь в координатах карты. Пересчёт делением на `2*scale^2` —
    это смена единиц, а не приближение: множитель точный и рациональный.
    """

    scale = prepared.lattice.scale
    total = sp.Integer(0)
    for radicand, coefficient in covered.doubled_area.terms:
        total += sp.Rational(
            coefficient.numerator, coefficient.denominator
        ) * sp.sqrt(radicand)
    return total / (2 * scale * scale)


def test_the_queue_equals_the_reference_raw_coverage_when_the_source_is_whole(
    weighted_preparation,
):
    """ТОЧНОЕ равенство очереди с эталонным `RawCoverage` — на целом источнике.

    Это сильнейший критерий среза, и он проверяется точным предикатом на
    РАЗНОСТИ, а не оболочкой: `sp.simplify(radsimp(...)).is_zero`, то есть
    доказанный ноль, а не «совпало на 30 знаках».

    Совпадение здесь не случайность и не совпадение чисел: у этого домена
    ИСТОЧНИКОМ ЯВЛЯЕТСЯ ВСЯ ГРАНИЦА (4 ребра из 4, стен ноль), а на целом
    источнике союз юбок и заметание фронтом описывают одно и то же множество.
    Что бывает при частичном источнике — соседний тест, и там разность НЕ ноль.
    """

    covered = conveyor_coverage(weighted_preparation)
    assert weighted_preparation.counter("CONVEYOR_WALL_EDGES") == 0
    snapshot, request = _weighted_input()
    evaluated = kernel.evaluate_reference_raw_coverage(
        kernel.compile_reference_envelopes(snapshot, request).compilation,
        request.requested_alpha,
    )
    difference = sp.simplify(
        sp.radsimp(
            _chart_area(covered, weighted_preparation)
            - sp.sympify(evaluated.raw_coverage.exact_area_expression)
        )
    )
    assert difference.is_zero is True


def test_the_queue_exceeds_the_reference_raw_coverage_on_a_partial_source(
    field_preparation,
):
    """ГИПОТЕЗА «множества совпадают целиком» на `bf6` ОПРОВЕРГНУТА, и числом.

    У `bf6` источником является 3 ребра из 12, остальные девять — СТЕНЫ. Юбка
    эталона обрезана концами СВОЕГО ребра; отрезок фронта очереди концами ребра
    не ограничен — он едет вдоль неподвижной прямой стены дальше, чем стена
    существует как отрезок границы. Это объявленная разница двух моделей
    (`mitered_standard.py`, «ОБЪЯВЛЕННАЯ ГРАНИЦА ЭТОЙ МОДЕЛИ»), а не дефект
    одной из них, и здесь она измерена:

    | величина | число |
    |---|---:|
    | очередь, площадь карты | `0.351389334264933965` |
    | эталон `RawCoverage`   | `0.328410391784360909` |
    | ИЗБЫТОК очереди        | `0.0229789424805730560`, то есть **+7.00 %** |

    Прежние митрованные числа `bf6` при этом НЕ СДВИНУЛИСЬ, и это тоже
    измерение, а не совпадение: снапшот не объявляет ни одного вогнутого угла,
    поэтому веера в плане нет (`test_wavefront_vertex_fan.py`), и покрытие
    осталось прежним — 6 членов, оболочка внутри `(3018411397, 3018411399)`,
    2.77 % площади домена.
    """

    covered = conveyor_coverage(field_preparation)
    assert field_preparation.counter("CONVEYOR_WALL_EDGES") == 9
    assert field_preparation.counter("CONVEYOR_SOURCE_EDGES") == 3
    assert len(covered.doubled_area.terms) == 6
    low, high = covered.doubled_area.enclosure(64)
    assert 3018411397 < low <= high < 3018411399
    assert covered.polygon_doubled_area == 108901947644

    snapshot, request = _field_input()
    evaluated = kernel.evaluate_reference_raw_coverage(
        kernel.compile_reference_envelopes(snapshot, request).compilation,
        request.requested_alpha,
    )
    reference = sp.sympify(evaluated.raw_coverage.exact_area_expression)
    excess = sp.simplify(
        sp.radsimp(_chart_area(covered, field_preparation) - reference)
    )
    assert excess.is_zero is not True
    # Знак доказан численно с запасом в 20 знаков: избыток, а не недобор.
    assert sp.N(excess, 20) > 0
    assert abs(sp.N(excess / reference * 100, 8) - sp.Float("6.9970205")) < sp.Float(
        "1e-6"
    )


def test_the_rescale_changes_the_record_and_not_the_geometry():
    """Пере-масштабирование доказано инвариантами, а не осмотром результата.

    Мост берёт у закона ровно две вещи, и обе обязаны пережить смену записи:

    1. **класс несущей прямой** — он и решает, какому ребру закон достался;
    2. **отношение `(s/|n|)^2`** — из него считается `q` ребра, то есть
       скорость фронта.

    Второе проверяется против ИСХОДНОЙ иррациональной записи: отношение
    считается прямо из `sqrt(426753013)`-нормали и сравнивается с рациональным
    отношением пере-масштабированного закона точным нулём разности. Совпадение
    здесь и означает, что переписана запись, а не задача.

    Знак проверяется тем же сравнением: деление шло на `|λ|`, поэтому нормаль
    смотрит туда же. Делили бы на `λ`, у двух законов из четырёх нормаль
    перевернулась бы, класс прямой сменился бы на противоположный, и это
    сравнение бы упало.
    """

    snapshot, request = _weighted_input()
    compiled = kernel.compile_reference_envelopes(snapshot, request)
    compilation = compiled.compilation
    frame, _ = validate_reference_geometry_payload(
        compilation.analysis_snapshot, compilation.plan_key.patch_domain_id
    )
    context = GeometryContext.build(compilation, frame)
    reading = _arrival_laws(context)
    assert reading.detail is None
    assert len(reading.laws) == 4
    assert reading.rescaled_count == 2

    by_name = {law.name: law for law in reading.laws}
    irrational_names = set()
    for spec in sorted(
        compilation.envelope_specs, key=lambda item: item.envelope_spec_id.value
    ):
        if not isinstance(spec, StripEnvelopeSpec):
            continue
        seed = next(
            item
            for item in compilation.seeds
            if getattr(item, "seed_id", None) == spec.source_seed_id
        )
        for source in context.support_segments_for_use(
            seed.chain_use_id, spec.envelope_spec_id.value
        ):
            normal, constant = strip_front_support_line(context, source)
            normal_x, normal_y = normal.expressions()
            speed_squared = (
                STRIP_FRONT_NORMAL_SPEED.as_expr()
                * STRIP_FRONT_NORMAL_SPEED.as_expr()
            )
            if exact_rational(normal_x) is None or exact_rational(normal_y) is None:
                irrational_names.add(spec.envelope_spec_id.value)
            law = by_name[spec.envelope_spec_id.value]

            # 1. Отношение (s/|n|)^2 исходной записи и переписанной — одно.
            raw_ratio = speed_squared / (
                normal_x * normal_x + normal_y * normal_y
            )
            rewritten = sp.Rational(
                law.speed_ratio_squared.numerator,
                law.speed_ratio_squared.denominator,
            )
            assert sp.simplify(raw_ratio - rewritten).is_zero is True, law.name

            # 2. Класс прямой исходной записи и переписанной — один. Считается
            #    той же функцией моста, которой он считается в работе.
            raw_class = _sympy_line_class(
                normal_x, normal_y, constant.as_expr()
            )
            assert raw_class is not None, law.name
            rewritten_class = line_class(
                (law.normal_x, law.normal_y, law.constant)
            )
            assert all(
                sp.simplify(
                    left - sp.Rational(right.numerator, right.denominator)
                ).is_zero
                is True
                for left, right in zip(raw_class, rewritten_class, strict=True)
            ), law.name

    # Переписаны ровно те два закона, у которых запись была иррациональна.
    assert len(irrational_names) == 2
    assert all(
        not by_name[name].is_unit_speed for name in irrational_names
    )


def _sympy_line_class(normal_x, normal_y, constant):
    """Класс несущей прямой в SymPy — тот же закон, что у `bridge.line_class`.

    Считается здесь, а не берётся у моста, ровно потому, что вход тут
    иррациональный: `line_class` объявлен на дробях. Нормировка та же —
    деление на модуль первой ненулевой компоненты.
    """

    for value in (normal_x, normal_y):
        if exact_sign(value) != 0:
            scale = sp.Abs(value)
            return (normal_x / scale, normal_y / scale, constant / scale)
    return None


def test_a_record_no_single_factor_can_rationalise_is_still_refused():
    """`ARRIVAL_LAW_IS_NOT_RATIONAL` не осиротел: он достижим и проверен.

    Пере-масштабирование делит всю запись на ОДНО число, поэтому оно спасает
    только запись с ОБЩИМ множителем. Нормаль `(1, sqrt(2))` общего множителя
    не имеет: рационализировать первую компоненту и вторую одновременно нечем,
    и отказ обязан остаться прежним, а не превратиться в приближение.

    Три случая, и каждый отвечает своей причиной:

    | запись | ответ |
    |---|---|
    | `n = (1, sqrt(2))` | масштаб не помогает |
    | `n = (sqrt(3), sqrt(2))` | масштаб не помогает |
    | `n = (0, 0)`, `c = sqrt(2)` | нормали нет вовсе, делить не на что |
    """

    zero = ExactScalar.from_value(0)
    one = sp.Integer(1)

    law, rescaled, issue = _read_arrival_law(
        "no-common-factor", _Normal(one, sp.sqrt(2)), zero, one
    )
    assert law is None and rescaled is False
    assert "не рациональна и после масштаба" in issue

    law, rescaled, issue = _read_arrival_law(
        "two-radicals", _Normal(sp.sqrt(3), sp.sqrt(2)), zero, one
    )
    assert law is None and rescaled is False
    assert "не рациональна и после масштаба" in issue

    law, rescaled, issue = _read_arrival_law(
        "zero-normal",
        _Normal(sp.Integer(0), sp.Integer(0)),
        ExactScalar.from_value(sp.sqrt(2)),
        one,
    )
    assert law is None and rescaled is False
    assert "нормаль закона нулевая" in issue

    # Положительный контроль: запись С общим множителем спасается, и запись
    # уже рациональная проходит БЕЗ масштаба.
    law, rescaled, issue = _read_arrival_law(
        "common-factor",
        _Normal(-sp.sqrt(7) / 4, sp.Integer(0)),
        ExactScalar.from_value(-sp.sqrt(7) / 4),
        one,
    )
    assert issue == "" and rescaled is True
    assert (law.normal_x, law.normal_y, law.constant) == (
        Fraction(-1),
        Fraction(0),
        Fraction(-1),
    )
    assert law.speed_squared == Fraction(16, 7)

    law, rescaled, issue = _read_arrival_law(
        "already-rational", _Normal(sp.Integer(0), sp.Integer(3)), zero, one
    )
    assert issue == "" and rescaled is False
    assert (law.normal_x, law.normal_y, law.speed_squared) == (
        Fraction(0),
        Fraction(3),
        Fraction(1),
    )


class _Normal:
    """Нормаль-ковектор синтетического закона: только то, что читает вход."""

    def __init__(self, x, y) -> None:
        self._values = (x, y)

    def expressions(self):
        return self._values


def test_a_rescale_that_cannot_be_proven_is_not_applied():
    """Дробь принимается только с доказательством, и это проверяется подменой.

    `_rational_after_scaling` возвращает дробь лишь тогда, когда
    `value - r*scale` обращается в ноль ТОЧНО. Без положительного контроля
    «доказательство есть» неотличимо от «проверка не выполняется», поэтому
    здесь стоят обе половины: доказуемая пара даёт дробь, недоказуемая — `None`.
    """

    radical = sp.sqrt(426753013) / 8192
    # Доказуемо: -radical поделённое на radical есть ровно -1.
    assert _rational_after_scaling(-radical, radical) == Fraction(-1)
    assert _rational_after_scaling(sp.Integer(0), radical) == Fraction(0)
    # Квадрат множителя рационален, даже когда сам множитель нет.
    assert _rational_after_scaling(sp.Integer(1), radical * radical) == Fraction(
        67108864, 426753013
    )
    # Недоказуемо: другой радикал общего множителя не имеет.
    assert _rational_after_scaling(sp.sqrt(2), radical) is None
    assert _rational_after_scaling(radical + 1, radical) is None


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
