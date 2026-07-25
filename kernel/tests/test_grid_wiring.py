"""Проводка целочисленной решётки: источник, конструкции и обе проверки.

Здесь доказывается не решётка — она доказана в `test_robust_grid.py` и
`test_robust_snapping.py` — а то, что она подключена: что закон доходит до
сертификата, сертификат до дайджеста, привязка до `offset_support_g` и
`segment_intersections`, а обе объявленные проверки исполняются и отказывают.
"""

from __future__ import annotations

from dataclasses import replace
from fractions import Fraction
import math

import pytest
import sympy as sp

from reference_factories import straight_snapshot

from cftuv_envelope import (
    GridScaleSearchOrderV1,
    GridScaleTrialOutcomeV1,
    GridScaleTrialV1,
    GridSnappingLawV1,
    GridWindowOutcomeV1,
    IntegerGridCertificateV1,
    build_rational_affine_planar_metric,
    compile_reference_envelopes,
    evaluate_reference_raw_coverage,
)
from cftuv_envelope.contracts.metric import ExactRationalV1
from cftuv_envelope.numeric import LocalPoint3V1
from cftuv_envelope.contracts.analysis import SourceVertexV1
from cftuv_envelope.outcomes import NamedOutcome
from cftuv_envelope.planar_metric import PlanarMetricAdmissionError
from cftuv_envelope.reference.arrangement import ExactSegmentArrangementBackend
from cftuv_envelope.reference.common import make_segment
from cftuv_envelope.reference.contracts import (
    ReachabilityCertificateV1,
    ReferenceOutcome,
)
from cftuv_envelope.reference.metric import (
    ExactPlanarMetric,
    SnapDisplacementExceedsBudget,
    snap_exact_point,
)
from cftuv_envelope.reference.planar_types import (
    SYMBOLIC_FALLBACK_COUNTS,
    ConstructionCertificate,
    ConstructionKind,
    ExactPlanarPoint,
    PlanarLoop,
    PlanarRegion,
)
from cftuv_envelope.reference.provenance import make_reference_provenance
from cftuv_envelope.robust.grid import (
    SNAP_COUNTS,
    GridSpecV1,
    reset_snap_counts,
    set_active_grid,
)
from cftuv_envelope.robust.snapping import grid_window_for_patch
from cftuv_envelope.source_grid import (
    AUTHOR_ANGULAR_ERROR,
    DECAL_DETAIL,
    chart_grid_for,
    intended_right_corners,
)


SQUARE = (((0.0, 0.0), (6.0, 0.0), (6.0, 4.0), (0.0, 4.0)),)
BOTTOM = ({"name": "bottom", "points": ((0.0, 0.0), (6.0, 0.0))},)

# Косой четырёхугольник: базис (2,1,0) и (3,3,0). Метрика не осевая, и,
# главное, `det(G)*g00 = 9*5 = 45` — не полный квадрат, поэтому `unit_g` даёт
# настоящий радикал. Осевые и параллелограммные фикстуры для этой роли не
# годятся: у них `|n|^2_g = det(G)*|t|^2_g` оказывается точным квадратом, и
# алгебра не рождается вовсе — счётчик показал бы ноль ДО привязки тоже. Вершины при этом целые,
# то есть лежат в узлах любой решётки: повёрнутый прямоугольник для этой роли
# не годится, у него задуманные координаты иррациональны и в узел не попадают
# ни при каком шаге. Это не обход теста, а следствие механизма: привязка
# восстанавливает отношение только там, где задуманное положение — узел.
SKEWED = (((0.0, 0.0), (2.0, 1.0), (3.0, 3.0), (1.0, 2.0)),)
SKEWED_BOTTOM = ({"name": "bottom", "points": ((0.0, 0.0), (2.0, 1.0))},)


def _evaluated(law, *, alpha="1", faces=SQUARE, routes=BOTTOM, transform=None):
    snapshot, request = straight_snapshot(
        faces=faces,
        source_routes=routes,
        alpha=alpha,
        revision_name=f"grid-{law.value}",
    )
    if transform is not None:
        snapshot = replace(
            snapshot,
            source_vertices=frozenset(
                SourceVertexV1(
                    item.vertex_id,
                    LocalPoint3V1(
                        *transform(
                            item.position.x, item.position.y, item.position.z
                        )
                    ),
                )
                for item in snapshot.source_vertices
            ),
        )
    domain = next(iter(snapshot.patch_domains))
    metric = build_rational_affine_planar_metric(
        source_revision=snapshot.source_revision,
        patch_domain_id=domain.patch_domain_id,
        owner_patch_id=domain.owner_patch_id,
        source_vertices=snapshot.source_vertices,
        source_faces=snapshot.surface_ir.source_faces,
        grid_policy=law,
    )
    snapshot = replace(snapshot, surface_metric_descriptors=frozenset({metric}))
    compilation = compile_reference_envelopes(snapshot, request).compilation
    result = evaluate_reference_raw_coverage(compilation, request.requested_alpha)
    return metric, result


# --------------------------------------------------------------------------
# Закон доходит до сертификата, сертификат — до дайджеста
# --------------------------------------------------------------------------


def test_every_metric_records_which_grid_law_produced_it():
    for law in GridSnappingLawV1:
        metric, _ = _evaluated(law)
        certificate = metric.grid_certificate
        assert certificate.snapping_law is law
        # Обе границы окна названы — обе и записаны.
        assert certificate.window_lower_bound.numerator > 0
        assert certificate.window_upper_bound.numerator > 0
        assert certificate.window_outcome is GridWindowOutcomeV1.WINDOW_AVAILABLE


def test_the_grid_specification_reaches_the_digest_and_distinguishes_scales():
    """Одна спецификация — один дайджест; разные спецификации — разные.

    Второе — тоже проверка: если дайджест не различает решётки, значит
    спецификация в сертификат не попала, и воспроизвести результат нечем.
    """

    first = _evaluated(GridSnappingLawV1.INTEGER_GRID_SNAP_V1)[1]
    second = _evaluated(GridSnappingLawV1.INTEGER_GRID_SNAP_V1)[1]
    assert first.raw_coverage.semantic_digest == second.raw_coverage.semantic_digest

    metric, _ = _evaluated(GridSnappingLawV1.INTEGER_GRID_SNAP_V1)
    certificate = metric.grid_certificate
    winner = certificate.scale_trials[-1]
    coarse = replace(
        metric,
        grid_certificate=replace(
            certificate,
            window_step=ExactRationalV1(1, 1024),
            source_scale=1024,
            scale_trials=certificate.scale_trials[:-1]
            + (replace(winner, scale=1024, step=ExactRationalV1(1, 1024)),),
        ),
    )
    from cftuv_envelope import canonical_json_bytes

    assert canonical_json_bytes(coarse) != canonical_json_bytes(metric)


# --------------------------------------------------------------------------
# Дифференциальный: топология та же, координаты — не дальше половины ячейки
# --------------------------------------------------------------------------


def _points(result):
    return {
        (vertex.point.x.expression, vertex.point.y.expression)
        for vertex in result.raw_coverage.vertices
    }


def test_snapping_keeps_the_topology_it_found_without_the_grid():
    unsnapped_metric, unsnapped = _evaluated(
        GridSnappingLawV1.UNSNAPPED_EXACT_V1, faces=SKEWED, routes=SKEWED_BOTTOM, alpha="0.25"
    )
    snapped_metric, snapped = _evaluated(
        GridSnappingLawV1.INTEGER_GRID_SNAP_V1, faces=SKEWED, routes=SKEWED_BOTTOM, alpha="0.25"
    )
    assert unsnapped.outcome is ReferenceOutcome.EXACT
    assert snapped.outcome is ReferenceOutcome.EXACT
    assert len(snapped.raw_coverage.regions) == len(unsnapped.raw_coverage.regions)
    assert len(snapped.raw_coverage.loops) == len(unsnapped.raw_coverage.loops)
    assert len(snapped.raw_coverage.edges) == len(unsnapped.raw_coverage.edges)

    grid = ExactPlanarMetric.from_descriptor(snapped_metric).chart_grid
    half_cell = Fraction(1, 2 * grid.scale)
    before = sorted(
        (float(sp.sympify(x)), float(sp.sympify(y))) for x, y in _points(unsnapped)
    )
    after = sorted(
        (float(sp.sympify(x)), float(sp.sympify(y))) for x, y in _points(snapped)
    )
    assert len(before) == len(after)
    for (bx, by), (ax, ay) in zip(before, after):
        assert abs(ax - bx) <= float(half_cell)
        assert abs(ay - by) <= float(half_cell)


def _canonicalization_sites(law, monkeypatch):
    """Откуда именно приходят алгебраические канонизации. Не «сколько», а «где».

    «Ноль» тут недостижим и не был бы правдой: `unit_g` несёт `sqrt` по
    определению, и привязать направление некуда — привязывается точка. Вопрос,
    ради которого счётчик заведён, другой: течёт ли радикал ДАЛЬШЕ конструкции.
    Ответ на него даёт место вызова, а не сумма.
    """

    import traceback
    from collections import Counter
    from pathlib import Path as _Path

    from cftuv_envelope.reference import planar_types

    original = planar_types._canonical_expr
    sites = Counter()

    def traced(value):
        if not value.is_Rational:
            stack = traceback.extract_stack()[:-1]
            frame = next(
                (
                    item
                    for item in reversed(stack)
                    if "planar_types" not in item.filename
                ),
                stack[-1],
            )
            sites[_Path(frame.filename).name] += 1
        return original(value)

    # Патч снимается СВОИМ же вызовом, а не в конце теста. Иначе второй замер
    # оборачивает трассировщик первого, первый продолжает считать чужой прогон,
    # и его число оказывается суммой всех последующих — то есть замер меряет
    # порядок вызовов, а не закон.
    monkeypatch.setattr(planar_types, "_canonical_expr", traced)
    try:
        _evaluated(law, faces=SKEWED, routes=SKEWED_BOTTOM, alpha="0.25")
    finally:
        monkeypatch.undo()
    return sites


def test_snapping_keeps_the_radical_inside_the_construction_that_makes_it(
    monkeypatch,
):
    """Ядро среза: после привязки радикал не течёт из конструкции дальше.

    Замер на этой фикстуре: 901 алгебраическая канонизация без решётки против
    41 с ней, и все 41 — внутри `reference/metric.py` (`unit_g`,
    `owner_normal_g` и вход самой привязки в `offset_support_g`). В arrangement
    их ноль, а именно он платит за них дороже всех.
    """

    unsnapped = _canonicalization_sites(
        GridSnappingLawV1.UNSNAPPED_EXACT_V1, monkeypatch
    )
    snapped = _canonicalization_sites(
        GridSnappingLawV1.INTEGER_GRID_SNAP_V1, monkeypatch
    )
    assert unsnapped["arrangement.py"] > 0, (
        "без решётки радикал обязан дотекать до arrangement, иначе фикстура "
        "не проверяет ничего"
    )
    assert snapped["arrangement.py"] == 0, (
        f"радикал всё ещё течёт в arrangement: {dict(snapped)}"
    )
    assert set(snapped) == {"metric.py"}, dict(snapped)
    assert sum(snapped.values()) < sum(unsnapped.values()) / 10


# --------------------------------------------------------------------------
# Проверка 1 — alpha против шага
# --------------------------------------------------------------------------


def test_a_band_thinner_than_four_cells_is_a_named_refusal():
    metric, result = _evaluated(
        GridSnappingLawV1.INTEGER_GRID_SNAP_V1, alpha="0.0001"
    )
    step = Fraction(
        metric.grid_certificate.window_step.numerator,
        metric.grid_certificate.window_step.denominator,
    )
    assert Fraction("0.0001") < 4 * step
    assert result.outcome is ReferenceOutcome.ENVELOPE_BAND_BELOW_GRID_RESOLUTION


def test_the_same_band_passes_without_a_grid():
    """Отказ принадлежит решётке, а не полосе: без решётки та же alpha проходит."""

    _, result = _evaluated(GridSnappingLawV1.UNSNAPPED_EXACT_V1, alpha="0.0001")
    assert result.outcome is not ReferenceOutcome.ENVELOPE_BAND_BELOW_GRID_RESOLUTION


# --------------------------------------------------------------------------
# Проверка 2 — привязка обязана доказать, что сработала
# --------------------------------------------------------------------------


def _positions(snapshot, faces):
    return {
        item.vertex_id: (
            Fraction(*float(item.position.x).as_integer_ratio()),
            Fraction(*float(item.position.y).as_integer_ratio()),
            Fraction(*float(item.position.z).as_integer_ratio()),
        )
        for item in snapshot.source_vertices
    }


def test_a_nearly_right_corner_is_named_intended_right():
    """Классификация угла записана, а не спрятана: она видна в сертификате."""

    snapshot, _ = straight_snapshot(
        faces=SQUARE, source_routes=BOTTOM, revision_name="corners"
    )
    faces = tuple(snapshot.surface_ir.source_faces)
    square = _positions(snapshot, faces)
    assert intended_right_corners(square, faces) == ()

    tilted = dict(square)
    key = next(k for k, v in square.items() if v[0] == 6 and v[1] == 0)
    # Сдвиг на 3 мкм по вертикали при плече 4 м — это 7.5e-7 рад, внутри
    # объявленной авторской ошибки.
    tilted[key] = (square[key][0] + Fraction(3, 10**6), square[key][1], square[key][2])
    named = intended_right_corners(tilted, faces)
    assert named, "угол в пределах объявленной ошибки обязан быть назван"
    assert len(named) <= 4 * len(faces)


def _field_certificate(**overrides):
    """Сертификат с числами полевого патча; переопределяется по полю."""

    facts = dict(
        snapping_law=GridSnappingLawV1.INTEGER_GRID_SNAP_V1,
        window_outcome=GridWindowOutcomeV1.WINDOW_AVAILABLE,
        patch_extent=ExactRationalV1(11, 1),
        author_angular_error=ExactRationalV1(7, 10**6),
        decal_detail=ExactRationalV1(1, 100),
        window_lower_bound=ExactRationalV1(77, 500000),
        window_upper_bound=ExactRationalV1(1, 100),
        window_step=ExactRationalV1(1, 2048),
        source_scale=2048,
        magnitude_bound=25,
        intended_right_corners=3,
        restored_right_corners=3,
        search_order=GridScaleSearchOrderV1.FINEST_ADMISSIBLE_FIRST_V1,
        scale_trials=(
            GridScaleTrialV1(
                scale=4096,
                step=ExactRationalV1(1, 4096),
                restored_right_corners=1,
                outcome=GridScaleTrialOutcomeV1.RELATIONS_NOT_RESTORED,
            ),
            GridScaleTrialV1(
                scale=2048,
                step=ExactRationalV1(1, 2048),
                restored_right_corners=3,
                outcome=GridScaleTrialOutcomeV1.RELATIONS_RESTORED,
            ),
        ),
    )
    facts.update(overrides)
    return IntegerGridCertificateV1(**facts)


def test_the_snap_refuses_when_it_restores_nothing():
    """Отказ строится там же, где привязка, и он именованный.

    Сертификат, объявляющий восстановление, не может существовать с
    невосстановленным углом: это запрещено конструктивно.
    """

    with pytest.raises(ValueError) as failure:
        _field_certificate(
            restored_right_corners=1,
            scale_trials=(
                GridScaleTrialV1(
                    scale=2048,
                    step=ExactRationalV1(1, 2048),
                    restored_right_corners=1,
                    outcome=GridScaleTrialOutcomeV1.RELATIONS_NOT_RESTORED,
                ),
            ),
        )
    assert "не восстановил" in str(failure.value)


def test_the_certificate_refuses_a_search_that_contradicts_its_own_law():
    """Записанный перебор — доказательство выбора, а не украшение при нём.

    Закон объявляет четыре свойства (перебор непуст, победил ПЕРВЫЙ прошедший,
    победитель и есть выбранный шаг, порядок объявленный) — и на каждое здесь
    приходится проба. Без этого «перебор записан» проверяло бы наличие поля,
    а не то, что в нём записан ТОТ перебор.
    """

    passed = GridScaleTrialV1(
        scale=2048,
        step=ExactRationalV1(1, 2048),
        restored_right_corners=3,
        outcome=GridScaleTrialOutcomeV1.RELATIONS_RESTORED,
    )
    failed = GridScaleTrialV1(
        scale=4096,
        step=ExactRationalV1(1, 4096),
        restored_right_corners=1,
        outcome=GridScaleTrialOutcomeV1.RELATIONS_NOT_RESTORED,
    )
    cases = {
        "перебора": dict(scale_trials=()),
        "прошедшей": dict(scale_trials=(passed, failed)),
        "не совпадает": dict(scale_trials=(failed, replace(passed, scale=1024,
                                                           step=ExactRationalV1(1, 1024)))),
        "порядком": dict(
            search_order=GridScaleSearchOrderV1.COARSEST_ADMISSIBLE_FIRST_V1,
            scale_trials=(failed, passed),
        ),
        "границы окна": dict(
            scale_trials=(
                GridScaleTrialV1(
                    scale=2**20,
                    step=ExactRationalV1(1, 2**20),
                    restored_right_corners=1,
                    outcome=GridScaleTrialOutcomeV1.RELATIONS_NOT_RESTORED,
                ),
                passed,
            )
        ),
        "расходится": dict(
            scale_trials=(replace(failed, restored_right_corners=3), passed)
        ),
    }
    for fragment, overrides in cases.items():
        with pytest.raises(ValueError) as failure:
            _field_certificate(**overrides)
        assert fragment in str(failure.value), (fragment, str(failure.value))


def test_an_unsnapped_certificate_cannot_carry_a_search_it_never_ran():
    with pytest.raises(ValueError) as failure:
        _field_certificate(
            snapping_law=GridSnappingLawV1.UNSNAPPED_EXACT_V1,
            restored_right_corners=1,
        )
    assert "не перебирает" in str(failure.value)


def _field_snapshot():
    from pathlib import Path

    import cftuv_envelope as kernel

    fixture = (
        Path(__file__).resolve().parents[1]
        / "fixtures"
        / "building_002_point_contact_v1"
    )
    return (
        kernel.AnalysisSnapshotCodecV1.loads(
            (fixture / "analysis_snapshot.json").read_bytes()
        ),
        kernel.DecalRequestCodecV1.loads((fixture / "decal_request.json").read_bytes()),
    )


def _field_metric(snapshot, law):
    import cftuv_envelope as kernel

    descriptor = next(iter(snapshot.surface_metric_descriptors))
    domain = next(
        item
        for item in snapshot.patch_domains
        if item.patch_domain_id == descriptor.patch_domain_id
    )
    return build_rational_affine_planar_metric(
        source_revision=snapshot.source_revision,
        patch_domain_id=descriptor.patch_domain_id,
        owner_patch_id=domain.owner_patch_id,
        source_vertices=snapshot.source_vertices,
        source_faces=snapshot.surface_ir.source_faces,
        planarity_policy=kernel.PlanarityAdmissionLawV1.NEAR_PLANAR_PROJECTION_V1,
        grid_policy=law,
        source_lineage=descriptor.source_lineage,
    )


def test_the_field_mesh_restores_every_intended_right_corner():
    """Главные ворота среза, на реальном меше, а не на синтетике.

    `building.002`, патч 0: три угла объявленная ошибка числит задуманно
    прямыми. Прежний закон брал шаг у нижней границы окна — 1/4096 — и
    восстанавливал ОДИН из трёх, потому что координата вершины `…:3` по Y
    равна 27.4752197265625 и при этом масштабе ложится ровно в середину
    ячейки. Закон перебора пробует 4096, получает 1 из 3, идёт дальше и берёт
    2048, где восстановлены все три.

    Проверяется и то, и другое: и что выбран 2048, и что 4096 действительно
    пробовали и он действительно отказал. Иначе «перебор работает» нельзя
    отличить от «повезло с первым кандидатом».
    """

    snapshot, _ = _field_snapshot()
    metric = _field_metric(snapshot, GridSnappingLawV1.INTEGER_GRID_SNAP_V1)
    certificate = metric.grid_certificate

    assert certificate.intended_right_corners == 3
    assert certificate.restored_right_corners == 3
    assert certificate.source_scale == 2048
    assert certificate.search_order is (
        GridScaleSearchOrderV1.FINEST_ADMISSIBLE_FIRST_V1
    )
    assert [
        (item.scale, item.restored_right_corners, item.outcome)
        for item in certificate.scale_trials
    ] == [
        (4096, 1, GridScaleTrialOutcomeV1.RELATIONS_NOT_RESTORED),
        (2048, 3, GridScaleTrialOutcomeV1.RELATIONS_RESTORED),
    ]


def test_the_field_search_records_every_scale_it_tried_not_only_the_winner():
    """Перебор записан целиком, иначе выбор невоспроизводим.

    Отказавшая проба — это причина, по которой выбран следующий масштаб.
    Записать только победителя значило бы записать следствие без причины: по
    сертификату было бы не отличить «4096 не пробовали» от «4096 отказал».
    """

    snapshot, _ = _field_snapshot()
    certificate = _field_metric(
        snapshot, GridSnappingLawV1.INTEGER_GRID_SNAP_V1
    ).grid_certificate

    assert len(certificate.scale_trials) == 2
    assert certificate.scale_trials[0].scale != certificate.source_scale
    assert certificate.scale_trials[-1].scale == certificate.source_scale
    # Каждая проба несёт свой шаг дробью, а не только масштаб: сертификат
    # читается без пересчёта, и шаг в нём тот же, что в окне.
    assert [item.step for item in certificate.scale_trials] == [
        ExactRationalV1(1, 4096),
        ExactRationalV1(1, 2048),
    ]


def test_no_scale_in_the_window_is_a_named_refusal_not_a_silent_choice():
    """Пустой перебор кончается именем, а не «ближайшим» масштабом.

    Предикат приёмки подменяется на заведомо непроходимый — так проверяется
    именно ветка «ни один не прошёл», а не удача полевого меша. Строится тот
    же вызов, что и в продукте, поэтому проверяется и то, что отказ доходит
    наружу, а не гасится по дороге.
    """

    from cftuv_envelope import source_grid

    snapshot, _ = _field_snapshot()
    original = source_grid.restored_right_corners
    source_grid.restored_right_corners = lambda positions, corners: 0
    try:
        with pytest.raises(PlanarMetricAdmissionError) as failure:
            _field_metric(snapshot, GridSnappingLawV1.INTEGER_GRID_SNAP_V1)
    finally:
        source_grid.restored_right_corners = original

    assert (
        failure.value.outcome is NamedOutcome.NO_GRID_SCALE_RESTORES_RELATIONS
    )
    # Отказ называет ВЕСЬ перебор, а не только последнюю пробу: иначе по
    # сообщению не отличить «окно пусто» от «одна проба отказала».
    message = str(failure.value)
    for scale in (128, 256, 512, 1024, 2048, 4096):
        assert f"{scale}→0" in message


def test_the_declared_order_starts_where_the_old_law_stopped():
    """Новый закон — расширение прежнего, а не другой закон.

    Прежний брал шаг у нижней границы окна и на этом заканчивал. Объявленный
    порядок начинает ровно с него, поэтому на всяком патче, где прежний закон
    работал, выбирается тот же масштаб и ничего не двигается; перебор виден
    только там, где прежний закон отказывал.
    """

    from cftuv_envelope.robust.snapping import admissible_scales

    for extent in (Fraction(1), Fraction(4), Fraction(11), Fraction(40)):
        window = grid_window_for_patch(
            extent=extent,
            author_angular_error=AUTHOR_ANGULAR_ERROR,
            decal_detail=DECAL_DETAIL,
        )
        candidates = admissible_scales(window)
        assert candidates[-1] == window.grid.scale


def test_the_search_order_changes_the_scale_and_the_cost_of_that_is_measured():
    """Оба конца окна проходят проверку 2 — и стоят разного. Замер, не вкус.

    На полевом патче объявленный порядок (от мелкого шага) берёт 2048 со
    сдвигом вершины 2.29e-04 м, обратный — 128 со сдвигом 3.76e-03 м, в 16.4
    раза больше при том же результате проверки. Во столько же раз поднимается
    и минимально допустимая alpha (4 x шаг). Тест держит оба числа, чтобы
    выбор порядка нельзя было поменять, не заметив цены.
    """

    from cftuv_envelope.source_grid import (
        select_grid_scale,
        snap_positions,
        source_extent,
    )

    snapshot, _ = _field_snapshot()
    descriptor = next(iter(snapshot.surface_metric_descriptors))
    domain = next(
        item
        for item in snapshot.patch_domains
        if item.patch_domain_id == descriptor.patch_domain_id
    )
    faces = tuple(
        sorted(
            (
                item
                for item in snapshot.surface_ir.source_faces
                if item.patch_id == domain.owner_patch_id
            ),
            key=lambda item: item.face_id.value,
        )
    )
    positions = _positions(snapshot, faces)
    positions = {
        vertex_id: value
        for vertex_id, value in positions.items()
        if any(vertex_id in face.vertex_cycle for face in faces)
    }
    window = grid_window_for_patch(
        extent=source_extent(positions),
        author_angular_error=AUTHOR_ANGULAR_ERROR,
        decal_detail=DECAL_DETAIL,
    )
    intended = intended_right_corners(positions, faces)
    assert len(intended) == 3

    chosen = {}
    for order in GridScaleSearchOrderV1:
        grid, trials = select_grid_scale(
            positions=positions,
            intended=intended,
            window=window,
            search_order=order,
        )
        snapped = snap_positions(positions, grid)
        displacement = max(
            max(abs(a - b) for a, b in zip(snapped[key], positions[key]))
            for key in positions
        )
        chosen[order] = (grid.scale, displacement, len(trials))

    fine = chosen[GridScaleSearchOrderV1.FINEST_ADMISSIBLE_FIRST_V1]
    coarse = chosen[GridScaleSearchOrderV1.COARSEST_ADMISSIBLE_FIRST_V1]
    assert (fine[0], fine[2]) == (2048, 2)
    assert (coarse[0], coarse[2]) == (128, 1)
    assert fine[1] == Fraction(15, 65536) and coarse[1] == Fraction(493, 131072)
    # 16.4x — цена одной сэкономленной пробы, и она платится геометрией.
    assert coarse[1] / fine[1] > 16


def _field_evaluated(law):
    """Полный прогон полевого меша под названным законом решётки."""

    import cftuv_envelope as kernel

    snapshot, request = _field_snapshot()
    metric = _field_metric(snapshot, law)
    snapshot = replace(snapshot, surface_metric_descriptors=frozenset({metric}))
    compiled = kernel.compile_reference_envelopes(snapshot, request)
    assert compiled.compilation is not None
    return metric, kernel.evaluate_reference_raw_coverage(
        compiled.compilation, request.requested_alpha
    )


def test_the_field_mesh_still_refuses_downstream_of_the_restored_corners():
    """Почему хост НЕ доходит до `INTEGER_GRID_SNAP_V1` — числом, а не прозой.

    Проверка 2 на полевом меше проходит (3 из 3), но полный прогон с привязкой
    КОНСТРУКЦИЙ отказывает: `REFERENCE_ARRANGEMENT_ROTATION_SYSTEM_UNPROVEN`,
    «boundary point does not have balanced incoming/outgoing half-edges», и
    так на КАЖДОМ масштабе окна (128, 256, 512, 1024, 2048 — 4096 отсеивается
    раньше, проверкой 2).

    Отказ локализован разрезом, а не рассуждением: при привязке одного только
    ИСТОЧНИКА (`SOURCE_ONLY_GRID_SNAP_V1`, решётка карты выключена) тот же
    прогон даёт `EXACT` и ту же топологию, что и без решётки — 3 петли, 3
    региона, 2 точечных контакта; это держит соседний тест. Ломает привязка
    КОНСТРУКЦИЙ — `offset_support_g` и `segment_intersections`, то есть то
    самое слияние вычисленных точек, которое карточка R1b сама называет
    лотереей, а не механизмом. На полевом меше оно сливает 3 точки и оставляет
    две висячие полурёбра вместо замкнутой границы.

    Тест держит этот факт исполняемым: пока он красный по-другому, доводить
    политику хоста до привязки конструкций нельзя, а когда он покраснеет —
    блокер снят. Именно поэтому `INTEGER_GRID_SNAP_V1` не удалён вместе с
    переключением хоста на `SOURCE_ONLY_GRID_SNAP_V1`: без него сторожить
    блокер было бы нечем.
    """

    metric, result = _field_evaluated(GridSnappingLawV1.INTEGER_GRID_SNAP_V1)
    assert metric.grid_certificate.restored_right_corners == 3
    assert (
        result.outcome
        is ReferenceOutcome.REFERENCE_ARRANGEMENT_ROTATION_SYSTEM_UNPROVEN
    )


# --------------------------------------------------------------------------
# Третий закон: привязан источник, конструкции — нет
# --------------------------------------------------------------------------


def _topology(result):
    raw = result.raw_coverage
    return (len(raw.loops), len(raw.regions), len(raw.point_contacts))


def test_the_field_mesh_keeps_its_topology_when_only_the_source_is_snapped():
    """То, ради чего срез: привязка источника не ломает того, что уже работало.

    Сравнивается не с записанным числом, а с прогоном ПОД УМОЛЧАНИЕМ в этом же
    тесте: «та же топология» — утверждение об отношении двух законов, и держать
    его константой значило бы проверять, что константа не изменилась, а не что
    законы согласны. Литеральные (3, 3, 2) стоят рядом, чтобы совпадение двух
    сломанных прогонов не читалось как согласие.
    """

    _, default = _field_evaluated(GridSnappingLawV1.UNSNAPPED_EXACT_V1)
    _, source_only = _field_evaluated(GridSnappingLawV1.SOURCE_ONLY_GRID_SNAP_V1)

    assert default.outcome is ReferenceOutcome.EXACT
    assert source_only.outcome is ReferenceOutcome.EXACT
    assert _topology(default) == (3, 3, 2)
    assert _topology(source_only) == _topology(default)


def test_the_source_only_law_restores_every_intended_right_corner():
    """Выигрыш нового закона: 3 из 3 там, где умолчание даёт 0 из 3.

    Оба числа в одном тесте намеренно. «Восстановлено 3» само по себе ничего
    не говорит: если бы умолчание тоже давало 3, восстанавливать было бы
    нечего и закон не покупал бы ничего.
    """

    snapshot, _ = _field_snapshot()
    default = _field_metric(
        snapshot, GridSnappingLawV1.UNSNAPPED_EXACT_V1
    ).grid_certificate
    snapped = _field_metric(
        snapshot, GridSnappingLawV1.SOURCE_ONLY_GRID_SNAP_V1
    ).grid_certificate

    assert default.intended_right_corners == 3
    assert default.restored_right_corners == 0
    assert snapped.intended_right_corners == 3
    assert snapped.restored_right_corners == 3
    # Тот же перебор, что и у привязки конструкций: источник они двигают
    # одинаково, и расходятся ниже.
    assert snapped.source_scale == 2048
    assert [
        (item.scale, item.restored_right_corners) for item in snapped.scale_trials
    ] == [(4096, 1), (2048, 3)]


def test_the_source_only_law_moves_the_source_and_leaves_constructions_exact():
    """Разрез назван двумя величинами — значит и проверяется двумя.

    `source_step` принадлежит привязке ИСТОЧНИКА (в метрах, её читает проверка
    `alpha >= 4 x шаг`), `chart_grid` — привязке КОНСТРУКЦИЙ. Ровно этим
    `SOURCE_ONLY_GRID_SNAP_V1` и отличается от `INTEGER_GRID_SNAP_V1`, поэтому
    одной проверки «решётка есть/нет» тут недостаточно: она не отличила бы
    снятую привязку конструкций от снятой привязки вовсе.
    """

    snapshot, _ = _field_snapshot()
    expected = {
        GridSnappingLawV1.UNSNAPPED_EXACT_V1: (None, None),
        GridSnappingLawV1.SOURCE_ONLY_GRID_SNAP_V1: (Fraction(1, 2048), None),
        # Решётка карты мельче решётки источника (32768 против 2048): её шаг
        # выводится из метрического, а не берётся равным, и ячейка карты по
        # доказанной границе никогда не крупнее ячейки источника.
        GridSnappingLawV1.INTEGER_GRID_SNAP_V1: (Fraction(1, 2048), 32768),
    }
    for law, (step, chart_scale) in expected.items():
        exact = ExactPlanarMetric.from_descriptor(_field_metric(snapshot, law))
        assert exact.source_step == step, law
        assert (
            None if exact.chart_grid is None else exact.chart_grid.scale
        ) == chart_scale, law
        # Бюджет сдвига объявляется только там, где есть что сдвигать.
        assert (exact.squared_snap_budget is None) is (chart_scale is None), law


def test_the_source_only_snap_moves_a_field_vertex_by_a_measured_amount():
    """Сдвиг вершины — цена закона, и она названа точной дробью, а не оценкой.

    15/65536 = 2.288818e-04 м на `building.002`. Это ровно половина ячейки
    выбранного шага 1/2048 = 4.882812e-04 м, то есть максимум, который привязка
    вообще может стоить: функция объявляет границу, и её собственный результат
    ей удовлетворяет.
    """

    from cftuv_envelope.source_grid import resolve_source_grid

    snapshot, _ = _field_snapshot()
    descriptor = next(iter(snapshot.surface_metric_descriptors))
    domain = next(
        item
        for item in snapshot.patch_domains
        if item.patch_domain_id == descriptor.patch_domain_id
    )
    faces = tuple(
        sorted(
            (
                item
                for item in snapshot.surface_ir.source_faces
                if item.patch_id == domain.owner_patch_id
            ),
            key=lambda item: item.face_id.value,
        )
    )
    before = {
        vertex_id: value
        for vertex_id, value in _positions(snapshot, faces).items()
        if any(vertex_id in face.vertex_cycle for face in faces)
    }
    facts = resolve_source_grid(
        positions=before,
        faces=faces,
        snapping_law=GridSnappingLawV1.SOURCE_ONLY_GRID_SNAP_V1,
    )
    after = facts.positions
    shift = max(
        max(abs(a - b) for a, b in zip(after[key], before[key], strict=True))
        for key in before
    )
    step = Fraction(1, facts.certificate.source_scale)
    assert shift == Fraction(15, 65536)
    assert shift <= step / 2


def test_snapping_the_source_alone_buys_no_algebraic_reduction(monkeypatch):
    """Замер, отвечающий «нет»: производительности привязка источника не даёт.

    Вопрос был открыт — покупает ли новый закон хоть сколько-нибудь алгебры,
    ради которой решётка затевалась. Ответ измерен и он отрицательный:
    901 канонизация без решётки, 901 с привязкой ОДНОГО источника, 41 с
    привязкой конструкций. Механизм объясняет ровно это: радикал рождается в
    `unit_g`, то есть у НАПРАВЛЕНИЯ, а привязать можно точку; источник
    двигается, но ни одна алгебраическая конструкция от этого не исчезает.
    Убирает их привязка конструкций — та самая, которая ломает полевую
    топологию.

    Отсюда честная формулировка среза: `SOURCE_ONLY_GRID_SNAP_V1` покупается
    КОРРЕКТНОСТЬЮ (3 восстановленных прямых угла из 3), а не скоростью.
    Счётчик считает вызовы `_canonical_expr`, а он не кэширован, поэтому три
    закона в одном процессе дают три независимых числа.
    """

    unsnapped = _canonicalization_sites(
        GridSnappingLawV1.UNSNAPPED_EXACT_V1, monkeypatch
    )
    source_only = _canonicalization_sites(
        GridSnappingLawV1.SOURCE_ONLY_GRID_SNAP_V1, monkeypatch
    )
    snapped = _canonicalization_sites(
        GridSnappingLawV1.INTEGER_GRID_SNAP_V1, monkeypatch
    )

    assert sum(source_only.values()) == sum(unsnapped.values())
    assert dict(source_only) == dict(unsnapped)
    # Радикал по-прежнему дотекает до arrangement: его останавливает не
    # источник, а конструкции.
    assert source_only["arrangement.py"] > 0
    assert snapped["arrangement.py"] == 0
    assert sum(snapped.values()) < sum(source_only.values()) / 10


# --------------------------------------------------------------------------
# Бюджет сдвига: объявленная граница проверяется на собственном результате
# --------------------------------------------------------------------------


def test_the_snap_result_satisfies_the_budget_the_metric_declares():
    metric, _ = _evaluated(
        GridSnappingLawV1.INTEGER_GRID_SNAP_V1,
        faces=SKEWED,
        routes=SKEWED_BOTTOM,
        alpha="0.25",
    )
    exact = ExactPlanarMetric.from_descriptor(metric)
    assert exact.chart_grid is not None
    # Привязка не двигает точку дальше половины ячейки по каждой оси, поэтому
    # объявленный бюджет обязан быть не меньше этого максимума.
    maximum = Fraction(1, 2) ** 2 * 2 / Fraction(exact.chart_grid.scale**2)
    assert exact.squared_snap_budget >= maximum


def test_the_exact_node_solver_agrees_with_the_proven_rounding_rule():
    """Дифференциально против R1a: то же правило, вычисленное там, где дроби нет.

    `snap_value` округляет `floor(x + 1/2)` на дробях; `snap_exact_point` решает
    тот же узел для алгебраического значения — сначала интервальной оболочкой,
    потом символьным `sp.floor`. Совпадение обязано быть точным, а не похожим,
    иначе привязка означает разное в разных местах конвейера. 400 рациональных
    и 200 алгебраических проб, расхождений 0.
    """

    import random

    from cftuv_envelope.robust.grid import snap_point

    random.seed(1)
    grid = GridSpecV1(scale=256)
    for _ in range(400):
        values = [
            Fraction(
                random.randint(-10**4, 10**4),
                random.choice((1, 2, 3, 256, 512, 7)),
            )
            for _ in range(2)
        ]
        reset_snap_counts()
        rational = snap_point(values[0], values[1], grid)
        reset_snap_counts()
        exact = snap_exact_point(
            ExactPlanarPoint.from_values(
                *(sp.Rational(item.numerator, item.denominator) for item in values)
            ),
            grid,
            None,
            "differential",
        )
        assert (rational.x, rational.y) == (
            int(sp.sympify(exact.x.expression) * grid.scale),
            int(sp.sympify(exact.y.expression) * grid.scale),
        )

    for _ in range(200):
        radicand = random.choice((2, 3, 5, 7, 45))
        offset = Fraction(random.randint(-500, 500), random.choice((1, 4, 16)))
        weight = Fraction(random.randint(-50, 50), random.choice((1, 3, 8)))
        expression = sp.Rational(
            offset.numerator, offset.denominator
        ) + sp.Rational(weight.numerator, weight.denominator) * sp.sqrt(radicand)
        reset_snap_counts()
        point = snap_exact_point(
            ExactPlanarPoint.from_values(expression, expression),
            grid,
            None,
            "differential",
        )
        assert int(sp.sympify(point.x.expression) * grid.scale) == int(
            sp.floor(expression * grid.scale + sp.Rational(1, 2))
        )


def test_a_budget_smaller_than_the_cell_fires_a_named_refusal():
    grid = GridSpecV1(scale=4)
    reset_snap_counts()
    point = ExactPlanarPoint.from_values(sp.Rational(1, 3), sp.Rational(1, 3))
    with pytest.raises(SnapDisplacementExceedsBudget):
        snap_exact_point(point, grid, Fraction(1, 10**9), "test")


# --------------------------------------------------------------------------
# Регион с дырой: путь, который поле не проходит ни разу
# --------------------------------------------------------------------------


def _loop(name, coordinates, provenance):
    points = tuple(ExactPlanarPoint.from_values(*item) for item in coordinates)
    certificates = tuple(
        ConstructionCertificate(
            ConstructionKind.SOURCE_VERTEX,
            source_vertex_ids=frozenset({f"vertex:{name}:{index}"}),
        )
        for index in range(len(points))
    )
    return PlanarLoop(
        f"loop:{name}",
        tuple(
            make_segment(
                f"segment:{name}:{index}",
                points[index],
                points[(index + 1) % len(points)],
                support_ids=provenance.support_ids,
                provenance=provenance,
                start_certificates=frozenset({certificates[index]}),
                end_certificates=frozenset(
                    {certificates[(index + 1) % len(points)]}
                ),
                boundary_constraint_ids=provenance.boundary_constraint_ids,
            )
            for index in range(len(points))
        ),
    )


def _holed_region(name, outer, holes, *, domain=False):
    provenance = make_reference_provenance(
        envelope_spec_ids=frozenset() if domain else frozenset({f"spec:{name}"}),
        envelope_instance_ids=(
            frozenset() if domain else frozenset({f"instance:{name}"})
        ),
        support_ids=frozenset({f"support:{name}"}),
        physical_edge_ids=frozenset({f"edge:{name}"}),
        chain_use_ids=frozenset({f"use:{name}"}),
        patch_domain_ids=frozenset({"domain"}),
        boundary_constraint_ids=(
            frozenset({f"constraint:{name}"}) if domain else frozenset()
        ),
    )
    return PlanarRegion(
        region_id=f"region:{name}",
        outer=_loop(f"{name}:outer", outer, provenance),
        holes=tuple(
            _loop(f"{name}:hole:{index}", item, provenance)
            for index, item in enumerate(holes)
        ),
        contributor_instance_ids=(
            frozenset() if domain else frozenset({f"instance:{name}"})
        ),
        contributor_spec_ids=(
            frozenset() if domain else frozenset({f"spec:{name}"})
        ),
    )


@pytest.mark.parametrize("grid", (None, GridSpecV1(scale=64)))
def test_a_region_with_a_hole_survives_the_grid(grid):
    """`DOMAIN_HOLE_LOOPS = 0` во всех семи полевых доменах — поле этот путь
    не проходит ни разу, поэтому синтетика обязана его пройти до поля."""

    region = _holed_region(
        "holed",
        ((0, 0), (8, 0), (8, 8), (0, 8)),
        (((2, 2), (2, 4), (4, 4), (4, 2)),),
    )
    domain = _holed_region(
        "domain", ((-20, -20), (20, -20), (20, 20), (-20, 20)), (), domain=True
    )
    reachability = {
        "instance:holed": ReachabilityCertificateV1(
            front_component_ids=frozenset({"front:holed"}),
            source_launch_reachable=True,
            initial_branch_count=1,
            effective_branch_count=1,
            boundary_event_keys=(),
            bypass_used=False,
        )
    }
    reset_snap_counts()
    set_active_grid(grid)
    try:
        result = ExactSegmentArrangementBackend().exact_union(
            (region,), (domain,), reachability
        )
    finally:
        set_active_grid(None)
    # 64 - 4 = 60: площадь квадрата за вычетом дыры, и дыра осталась петлёй.
    assert sp.sympify(result.exact_area_expression) == 60
    assert len(result.loops) == 2


# --------------------------------------------------------------------------
# Слияние вырождения, которое binary64 разнёс
# --------------------------------------------------------------------------


def test_the_grid_merges_points_binary64_split_apart():
    """Три отрезка, задуманно сходящиеся в одну точку, разнесены на 1e-9.

    Без решётки пересечения различны; с решёткой они ложатся в один узел, и
    `merged_points` это показывает. Слияние здесь — наблюдаемое следствие, а не
    механизм: механизм — привязка источника, и он проверяется отдельно.
    """

    provenance = make_reference_provenance(
        envelope_spec_ids=frozenset({"spec:x"}),
        envelope_instance_ids=frozenset({"instance:x"}),
        support_ids=frozenset({"support:x"}),
        physical_edge_ids=frozenset({"edge:x"}),
        chain_use_ids=frozenset({"use:x"}),
        patch_domain_ids=frozenset({"domain"}),
        boundary_constraint_ids=frozenset(),
    )
    drift = sp.Rational(1, 10**9)
    certificate = ConstructionCertificate(
        ConstructionKind.SOURCE_VERTEX, source_vertex_ids=frozenset({"vertex:x"})
    )

    def segment(name, start, end):
        return make_segment(
            name,
            ExactPlanarPoint.from_values(*start),
            ExactPlanarPoint.from_values(*end),
            support_ids=provenance.support_ids,
            provenance=provenance,
            start_certificates=frozenset({certificate}),
            end_certificates=frozenset({certificate}),
        )

    horizontal = segment("h", (-2, 0), (2, 0))
    from cftuv_envelope.reference.arrangement import segment_intersections

    crossings = [
        segment(f"leg{index}", (offset - 1, -1), (offset + 1, 1))
        for index, offset in enumerate((0, drift, 2 * drift))
    ]

    reset_snap_counts()
    set_active_grid(None)
    exact = {
        segment_intersections(horizontal, leg)[0].x.expression for leg in crossings
    }
    assert len(exact) == 3, "без решётки три задуманно совпавшие точки различны"
    assert SNAP_COUNTS["merged_points"] == 0

    reset_snap_counts()
    set_active_grid(GridSpecV1(scale=1024))
    try:
        merged = {
            segment_intersections(horizontal, leg)[0].x.expression
            for leg in crossings
        }
    finally:
        set_active_grid(None)
    assert len(merged) == 1, "с решёткой они обязаны стать одной"
    assert SNAP_COUNTS["merged_points"] == 2


# --------------------------------------------------------------------------
# Решётка карты: объявленная граница проверяется на собственном результате
# --------------------------------------------------------------------------


def test_a_chart_cell_is_never_coarser_than_the_source_cell():
    """Координаты карты — не метры, поэтому шаг источника переносится с
    доказанной верхней границей метрической длины, а не как есть."""

    step = Fraction(1, 4096)
    for gram in (
        ((sp.Integer(1), sp.Integer(0)), (sp.Integer(0), sp.Integer(1))),
        ((sp.Integer(36), sp.Integer(0)), (sp.Integer(0), sp.Integer(16))),
        ((sp.Rational(25), sp.Rational(7)), (sp.Rational(7), sp.Rational(9))),
    ):
        grid = chart_grid_for(gram, step)
        cell = Fraction(1, grid.scale)
        longest = math.sqrt(
            float(gram[0][0]) + 2 * abs(float(gram[0][1])) + float(gram[1][1])
        )
        assert float(cell) * longest <= float(step)


def test_the_declared_author_error_is_the_one_the_certificate_carries():
    metric, _ = _evaluated(GridSnappingLawV1.INTEGER_GRID_SNAP_V1)
    certificate = metric.grid_certificate
    assert Fraction(
        certificate.author_angular_error.numerator,
        certificate.author_angular_error.denominator,
    ) == AUTHOR_ANGULAR_ERROR
