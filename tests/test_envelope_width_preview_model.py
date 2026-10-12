"""Модель превью меша ширины (`envelope_width_preview_model`, `envelope_width_mesh_preview`): математика и жизнь сессии, без Blender.

Модель строится из АФФИННОГО СЕРТИФИКАТА одного точного прогона (`cftuv_envelope.materialize.parametric`): наклон по ширине каждой вершины и каждой петли UV. Никаких затравочных прогонов
и подгонки по образцам; утверждения (каждое названо и проверено числами, а не словом):

1. ВНУТРИ ИНТЕРВАЛА ПРЕВЬЮ РАВНО ТОЧНОМУ. На настоящих доменах (ряд квадратов `quad_row_bundle`, их сертификаты выпускает продуктовый путь) и на синтетической складке из двух доменов
   с общими вершинами позиции меша и UV кадра модели на ширине внутри интервала равны точному прогону с отклонением не больше `EXACT_TOLERANCE` (1e-9) — по ОДНОМУ образцу.
2. ВНЕ ИНТЕРВАЛА ОТКАЗАНО ИМЕНЕМ. Ширина за интервалом всех доменов — кадр отказан `PREVIEW_ALPHA_OUTSIDE_EVERY_INTERVAL` и равен базе;
   ширина за интервалом одного домена — домен придержан (`PARAM_INTERVAL_END`) на геометрии базы, остальные двигаются.
3. МОДЕЛЬ ДОМЕНА ИМЕЕТ ПРИЧИНЫ. Причина сертификата ядра (`PARAM_*`), нет сертификата, не сошлись размеры, нет интервала, не материализован, сварка дала два наклона одной вершины, сосед по стене —
   развёртка: у каждого свой исход, и общая вершина двух доменов не двигается, пока не смоделирован каждый.
4. БАЗА БИТОВО. Кадр на ширине базы — сама база; отмена возвращает меш побитово.
5. САМОПРОВЕРКА И КАРАНТИН. Неверная модель (сертификат, который не держится на окне) ловится сверкой с точным прогоном (`deviation`), домен идёт в карантин журнала доверия и
   возвращается по правилу (чистая независимая сверка в тени, сжатая область, третье опровержение снимает модель домена). Дробно-линейный закон из аудита ломает аффинность, и это названо.
6. МЕШ ПРИНИМАЕТ КАДР ТОЛЬКО СВОЙ. Владение (тождество объекта и датаблока, размеры, поколение раскладки, отпечаток раскладки точной записи),
   ширина и ревизия сверяются перед кадром; подмена датаблока с теми же свойствами, те же счётчики при другой диагонали, Edit, внешнее обновление
   геометрии в depsgraph и шаг истории снимают модель с названной причиной и не пишут в меш.
7. ЖИЗНЬ СЕССИИ. Кнопка кладёт образец, модель строит первый кадр (без затравок); точный результат заводит новую модель в потоке счёта, `finish_live_run`.
8. ИМЯ. Модель названа аффинным сертификатом одного прогона: структура домена на другой ширине проверяется апостериори, это сказано в строке статуса.
"""

from __future__ import annotations

import dataclasses
import itertools
import sys
import types
from array import array
from fractions import Fraction
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

KERNEL_SRC = Path(__file__).resolve().parents[1] / "kernel" / "src"
if str(KERNEL_SRC) not in sys.path:
    sys.path.insert(0, str(KERNEL_SRC))

from cftuv import envelope_production_mesh as production_mesh  # noqa: E402
from cftuv import envelope_width_preview_model as pm  # noqa: E402
from cftuv import envelope_width_mesh_preview as preview  # noqa: E402
from cftuv.envelope_debug_session import EnvelopeDebugSessionController  # noqa: E402
from cftuv.envelope_domain_pool import shutdown_domain_pool  # noqa: E402
from cftuv.envelope_production_export import MATERIALIZED, ProductionDomainResultV1, run_production  # noqa: E402
from cftuv_envelope.materialize.interval import CERTIFIED, NOT_CERTIFIED, AlphaIntervalV1  # noqa: E402
from envelope_fixture_bundles import quad_row_bundle  # noqa: E402

ROW = 4
OFFSET = 0.02
#: Допуск равенства превью и точного прогона на ширине внутри интервала (метры и единицы UV): округление `binary64` и разность ширин.
EXACT_TOLERANCE = 1e-9
FLOOR_SLOPE = (0.0, 1.0, 0.0)
WALL_SLOPE = (0.0, 0.0, 1.0)
STILL = (0.0, 0.0, 0.0)


@pytest.fixture(scope="module", autouse=True)
def _no_pool_outlives_the_module():
    yield
    shutdown_domain_pool()


# --------------------------------------------------------------------------
# Образцы: настоящие домены и синтетическая складка
# --------------------------------------------------------------------------


def _row_sample(controller, bundle, alpha, key=("row",)):
    run = run_production(
        controller,
        bundle,
        frozenset(range(ROW)),
        alpha,
        source_object_key="o",
        source_data_key="m",
        density=None,
        workers=0,
    )
    arrays = production_mesh.build_mesh_arrays(run.results, OFFSET)
    return preview.build_sample(run.results, arrays, key, alpha), run, arrays


@pytest.fixture(scope="module")
def row():
    bundle, controller = quad_row_bundle(ROW), EnvelopeDebugSessionController()
    return lambda alpha, key=("row",): _row_sample(controller, bundle, alpha, key)


def _synthetic_batch(alpha, *, free, fixed, faces, refs, uv_axis):
    """Батч минимального вида (то, что читает писатель): вершины `free` движутся с шириной, `fixed` стоят; UV — `(x, y) / alpha`."""

    def position(key):
        base = fixed.get(key)
        return base if base is not None else free[key](alpha)

    def vertex(key):
        x, y, z = position(key)
        item = SimpleNamespace(vert_key=SimpleNamespace(value=key), position=SimpleNamespace(x=x, y=y, z=z))
        if key in refs:
            item.semantic_location_ref = SimpleNamespace(value=refs[key])
        return item

    def face(index, keys):
        return SimpleNamespace(
            face_id=SimpleNamespace(value=f"face:{index}"),
            ordered_vert_keys=tuple(SimpleNamespace(value=k) for k in keys),
            uv_facts=tuple(
                SimpleNamespace(uv=SimpleNamespace(u=position(k)[uv_axis[0]] / alpha, v=position(k)[uv_axis[1]] / alpha)) for k in keys
            ),
            ownership_claim_id=SimpleNamespace(value="claim:0"),
        )

    keys = sorted({k for ring in faces for k in ring})
    return SimpleNamespace(
        vertices=tuple(vertex(k) for k in keys),
        faces=tuple(face(i, ring) for i, ring in enumerate(faces)),
        interface_chains=(),
        source_revision=SimpleNamespace(value="rev"),
    )


def _certificate(batch, slopes, uv_axis, *, alpha, markers=(), approximate=(), rate=0.0):
    """Сертификат домена такого вида, какой выпускает ядро: наклоны позиций в порядке ключей батча и наклоны `u * alpha`, `v * alpha` по петлям граней (`slopes`: ключ -> наклон позиции)."""

    keys = sorted(item.vert_key.value for item in batch.vertices)
    positions, loops = array("d"), array("d")
    count = 0
    for key in keys:
        positions.extend(slopes.get(key, STILL))
    for face in batch.faces:
        for vertex in face.ordered_vert_keys:
            slope = slopes.get(vertex.value, STILL)
            loops.extend((slope[uv_axis[0]], slope[uv_axis[1]]))
            count += 1
    exact_alpha = Fraction(str(float(alpha)))
    if markers:
        return SimpleNamespace(markers=tuple(markers), approximate=(), alpha=exact_alpha, vertex_count=0, loop_count=0, position_slopes=array("d"), loop_slopes=array("d"))
    return SimpleNamespace(
        markers=(), approximate=tuple(approximate), approximate_rate=rate, alpha=exact_alpha, vertex_count=len(keys), loop_count=count, position_slopes=positions, loop_slopes=loops,
    )


def _interval(alpha, low=0.1, high=0.6, status=CERTIFIED):
    return AlphaIntervalV1(status, "", float(alpha), float(low), float(high), 0, 0)


def _fold_results(
    alpha, *, wall_interval=None, wall_token_suffix="", wall_markers=(), floor_markers=(), wall_slope=WALL_SLOPE, certificates=True, floor_approximate=(), wall_approximate=(), floor_rate=0.0
):
    """Пол (`z = 0`) и стена (`y = 0`) делят ребро `a-b` исходника (ссылки `location:src:`): свободные вершины движутся с шириной."""

    refs = {"a": "location:src:A", "b": "location:src:B"}
    floor = _synthetic_batch(
        alpha,
        free={"c": lambda w: (1.0, w, 0.0), "d": lambda w: (0.0, w, 0.0)},
        fixed={"a": (0.0, 0.0, 0.0), "b": (1.0, 0.0, 0.0)},
        faces=(("a", "b", "c"), ("a", "c", "d")),
        refs=refs,
        uv_axis=(0, 1),
    )
    wall = _synthetic_batch(
        alpha,
        free={"e": lambda w: (1.0, 0.0, w), "f": lambda w: (0.0, 0.0, w)},
        fixed={"a": (0.0, 0.0, 0.0), "b": (1.0, 0.0, 0.0)},
        faces=(("b", "a", "f"), ("b", "f", "e")),
        refs=refs,
        uv_axis=(0, 2),
    )
    floor_certificate = (
        _certificate(floor, {"c": FLOOR_SLOPE, "d": FLOOR_SLOPE}, (0, 1), alpha=alpha, markers=floor_markers, approximate=floor_approximate, rate=floor_rate) if certificates else None
    )
    wall_certificate = _certificate(wall, {"e": wall_slope, "f": wall_slope}, (0, 2), alpha=alpha, markers=wall_markers, approximate=wall_approximate) if certificates else None
    return [
        ProductionDomainResultV1(
            0, "floor", MATERIALIZED, floor, normal=(0.0, 0.0, 1.0), source_normal=(0.0, 0.0, 1.0),
            alpha_interval=_interval(alpha), structure_digest="floor-structure", parametric=floor_certificate,
        ),
        ProductionDomainResultV1(
            1, "wall", MATERIALIZED, wall, normal=(0.0, -1.0, 0.0), source_normal=(0.0, -1.0, 0.0),
            alpha_interval=wall_interval or _interval(alpha), structure_digest="wall-structure" + wall_token_suffix, parametric=wall_certificate,
        ),
    ]


def _fold_sample(alpha, key=("fold",), **kwargs):
    results = _fold_results(alpha, **kwargs)
    arrays = production_mesh.build_mesh_arrays(results, OFFSET)
    return preview.build_sample(results, arrays, key, alpha), arrays


def _max_error(model, sample):
    positions, uvs, _live = pm.positions_and_uvs(model, sample.alpha)
    return float(np.max(np.abs(positions - sample.positions))), float(np.max(np.abs(uvs - sample.uvs)))


# --------------------------------------------------------------------------
# 1. Внутри интервала превью равно точному
# --------------------------------------------------------------------------


def test_the_preview_equals_the_exact_run_inside_the_interval_on_real_domains(row):
    base, _, _ = row(0.25)
    assert all(item.status == CERTIFIED for item in base.domains)
    # Продуктовый путь выпускает сертификаты: наклоны лежат в образце, причин нет.
    assert all(item.slopes is not None and item.uv_slopes is not None and item.markers == () for item in base.domains), [item.markers for item in base.domains]
    model = pm.build_model(base)
    assert isinstance(model, pm.PreviewModelV1) and model.modelled_domains == len(base.domains) == model.domain_count
    assert [row_[3:] for row_ in model.rows] == [(1, None)] * len(base.domains)
    # Аффинный сертификат верит себе на весь интервал ядра: ни досягаемости, ни второго образца.
    for width in (0.2501, 0.2503, 0.2507, 0.249, 0.2574, 0.26, 0.3, 0.5):
        exact, _, _ = row(width)
        position_error, uv_error = _max_error(model, exact)
        assert position_error <= EXACT_TOLERANCE and uv_error <= EXACT_TOLERANCE, (width, position_error, uv_error)
        check = pm.deviation(model, exact)
        assert check.refuted == () and check.domains_checked == len(base.domains) and check.max_position <= EXACT_TOLERANCE


def test_a_fold_of_two_domains_with_shared_vertices_is_followed_exactly():
    base, _ = _fold_sample(0.25)
    model = pm.build_model(base)
    assert isinstance(model, pm.PreviewModelV1) and model.modelled_domains == 2
    assert [row_[3:] for row_ in model.rows] == [(1, None), (1, None)]
    for width in (0.26, 0.31, 0.45):
        exact, _ = _fold_sample(width)
        assert max(_max_error(model, exact)) <= EXACT_TOLERANCE, width
    # Общие вершины `a`, `b` (сварка по ссылке источника) стоят: их позиции не зависят от ширины.
    frame_positions, _uvs, _live = pm.positions_and_uvs(model, 0.4)
    shared = [index for index in range(base.vertex_count) if (model.a1[index] == 0).all()]
    assert len(shared) >= 2
    assert np.array_equal(frame_positions[shared], base.positions[shared])


def test_the_model_reads_the_certificate_of_the_sample_and_the_loops_follow_the_view_order():
    base, _ = _fold_sample(0.25)
    floor, wall = base.domains
    assert floor.slopes.shape == (floor.vertices.shape[0], 3) and floor.uv_slopes.shape == (floor.loop_count, 2)
    # Свободные вершины `c`, `d` пола двигаются вдоль `y`: наклон позиции — единица, `v * alpha` — единица, `u * alpha` — нуль.
    assert sorted(set(floor.slopes[:, 1].tolist())) == [0.0, 1.0] and not floor.slopes[:, [0, 2]].any()
    assert sorted(set(floor.uv_slopes[:, 1].tolist())) == [0.0, 1.0] and not floor.uv_slopes[:, 0].any()
    assert sorted(set(wall.slopes[:, 2].tolist())) == [0.0, 1.0]


# --------------------------------------------------------------------------
# 2. Вне интервала отказано именем
# --------------------------------------------------------------------------


def test_a_width_beyond_every_interval_is_refused_by_name_and_equals_the_base():
    base, _ = _fold_sample(0.25)
    model = pm.build_model(base)

    frame = pm.evaluate(model, 0.9)  # интервал `(0.1, 0.6)`

    assert frame.refusal == pm.REFUSED_OUTSIDE_EVERY_INTERVAL == frame.outcome
    assert frame.live_domains == 0 and frame.held_domains == 2
    assert {reason for _patch, reason in frame.held} == {pm.DOMAIN_HELD_OUTSIDE} == {"PARAM_INTERVAL_END"}
    assert np.array_equal(frame.positions, np.asarray(base.positions, dtype=np.float32).reshape(-1))
    assert np.array_equal(frame.uvs, np.asarray(base.uvs, dtype=np.float32).reshape(-1))
    for bad in (0.0, -1.0, float("nan"), float("inf")):
        assert pm.evaluate(model, bad).refusal == pm.REFUSED_ALPHA
    assert pm.evaluate(None, 0.3).refusal == pm.REFUSED_NO_MODEL


def test_a_domain_whose_interval_is_left_is_held_at_the_base_and_the_others_move():
    base, _ = _fold_sample(0.25)
    # Интервал стены кончается на 0.3: на 0.4 пол двигается, стена придержана и названа.
    narrow = dataclasses.replace(base.domains[1], high=0.3)
    narrowed = dataclasses.replace(base, domains=(base.domains[0], narrow))
    model = pm.build_model(narrowed)
    frame = pm.evaluate(model, 0.4)
    assert (frame.live_domains, frame.held_domains) == (1, 1) and frame.outcome == pm.PREVIEW_MESH_FROM_AFFINE_CERTIFICATE_V1
    assert frame.held == ((1, pm.DOMAIN_HELD_OUTSIDE),)
    moved = frame.positions.reshape(-1, 3)
    wall_free = [i for i in narrow.vertices if (model.a1[i] == 0).all()]
    floor_free = [i for i in base.domains[0].vertices if (model.vlow[i] < 0.4 < model.vhigh[i])]
    assert floor_free and not np.array_equal(moved[floor_free], base.positions[floor_free].astype(np.float32))
    assert np.array_equal(moved[wall_free], base.positions[wall_free].astype(np.float32))


# --------------------------------------------------------------------------
# 3. Модель домена имеет причины
# --------------------------------------------------------------------------


def test_every_reason_a_domain_is_not_modelled_is_named():
    base, _ = _fold_sample(0.25)
    assert [row_[2] for row_ in pm.build_model(base).rows] == [pm.DOMAIN_MODELLED] * 2

    # 1. Причина сертификата ядра идёт как есть (`PARAM_*`); домен остаётся на последней точной геометрии.
    marked, _ = _fold_sample(0.25, wall_markers=("PARAM_NON_AFFINE_CLIP_NODE",))
    model = pm.build_model(marked)
    assert model.rows[1][2] == "PARAM_NON_AFFINE_CLIP_NODE" and model.rows[0][2] == pm.DOMAIN_MODELLED
    # 2. У результата нет сертификата вовсе (перенесён на другую ревизию либо шаг ширины выключен).
    bare, _ = _fold_sample(0.25, certificates=False)
    refusal = pm.build_model(bare)
    assert isinstance(refusal, pm.PreviewRefusalV1) and refusal.outcome == pm.REFUSED_NO_MODELLED_DOMAIN and pm.DOMAIN_NO_CERTIFICATE in refusal.detail
    # 3. Размеры сертификата не сошлись с видом домена (проверяет `sample_of`).
    results = _fold_results(0.25)
    results[1] = results[1].with_changes(parametric=SimpleNamespace(markers=(), alpha=Fraction("0.25"), vertex_count=999, loop_count=1, position_slopes=array("d"), loop_slopes=array("d")))
    arrays = production_mesh.build_mesh_arrays(results, OFFSET)
    shape = preview.build_sample(results, arrays, ("fold",), 0.25)
    assert pm.build_model(shape).rows[1][2] == pm.DOMAIN_SHAPE_MISMATCH
    # 4. У домена нет заверенного интервала (`NOT_CERTIFIED`) и домен не материализован (нет токена).
    uncertain, _ = _fold_sample(0.25, wall_interval=_interval(0.25, 0.25, 0.25, NOT_CERTIFIED))
    assert pm.build_model(uncertain).rows[1][2] == pm.DOMAIN_NO_INTERVAL
    tokenless = dataclasses.replace(base, domains=(base.domains[0], dataclasses.replace(base.domains[1], token="")))
    assert pm.build_model(tokenless).rows[1][2] == pm.DOMAIN_NOT_MATERIALIZED
    # 5. Все домены названы - отказ целой модели называет причины числом.
    everything, _ = _fold_sample(0.25, wall_markers=("PARAM_X",), floor_markers=("PARAM_Y",))
    refusal = pm.build_model(everything)
    assert refusal.outcome == pm.REFUSED_NO_MODELLED_DOMAIN and "PARAM_X=1" in refusal.detail and "PARAM_Y=1" in refusal.detail


def test_a_vertex_shared_with_an_unmodelled_domain_does_not_move():
    marked, _ = _fold_sample(0.25, wall_markers=("PARAM_NON_AFFINE_CLIP_NODE",))
    model = pm.build_model(marked)
    shared = sorted(set(marked.domains[0].vertices.tolist()) & set(marked.domains[1].vertices.tolist()))
    assert shared, "the fold has welded vertices"
    for index in shared:
        assert model.vlow[index] == np.inf and model.vhigh[index] == -np.inf and (model.a1[index] == 0).all()
    frame = pm.evaluate(model, 0.4)
    assert frame.held == ((1, "PARAM_NON_AFFINE_CLIP_NODE"),)
    assert (frame.live_domains, frame.held_domains) == (1, 1)


def test_a_certificate_of_another_width_is_named_and_never_read():
    """Сертификат несёт наклоны точки, на которой его выпустили: образец другой ширины (перенос, подмена) его не читает."""

    results = _fold_results(0.25)
    results[1] = results[1].with_changes(parametric=_certificate(results[1].batch, {"e": WALL_SLOPE, "f": WALL_SLOPE}, (0, 2), alpha=0.26))
    arrays = production_mesh.build_mesh_arrays(results, OFFSET)
    sample = preview.build_sample(results, arrays, ("fold",), 0.25)
    assert sample.domains[1].markers == (pm.DOMAIN_ALPHA_MISMATCH,) and sample.domains[1].slopes is None
    model = pm.build_model(sample)
    assert model.rows[1][2] == pm.DOMAIN_ALPHA_MISMATCH == "PARAM_CERTIFICATE_ALPHA_MISMATCH" and model.rows[0][2] == pm.DOMAIN_MODELLED


def test_two_domains_that_give_one_welded_vertex_two_slopes_are_both_named_and_held():
    """Сварка по равным позициям при alpha0 не держится на окне, если наклоны общей вершины разные."""

    results = _fold_results(0.25)
    # Стена несёт наклон на общей вершине `a`: пол держит её неподвижной.
    wall = results[1].batch
    broken = _certificate(wall, {"a": (0.0, 0.0, 0.25), "e": WALL_SLOPE, "f": WALL_SLOPE}, (0, 2), alpha=0.25)
    results[1] = results[1].with_changes(parametric=broken)
    arrays = production_mesh.build_mesh_arrays(results, OFFSET)
    sample = preview.build_sample(results, arrays, ("fold",), 0.25)
    refusal = pm.build_model(sample)
    assert isinstance(refusal, pm.PreviewRefusalV1) and refusal.outcome == pm.REFUSED_NO_MODELLED_DOMAIN
    assert f"{pm.DOMAIN_WELD_DISAGREES}=2" in refusal.detail


def test_a_neighbour_without_a_certificate_makes_the_wall_nodes_of_this_domain_unservable():
    """Закон `WALL_MITER_OFFSET_V1`: узел стены смещается по нормалям СОСЕДА; у соседа без сертификата они неизвестны, и позиция узла не доказана."""

    results = _fold_results(0.25)
    results[0] = results[0].with_changes(parametric=None)
    arrays = production_mesh.build_mesh_arrays(results, OFFSET)
    arrays = dataclasses.replace(arrays, domain_partners=((), (0,)))  # у стены сосед по стене - пол
    sample = preview.build_sample(results, arrays, ("fold",), 0.25)
    assert sample.domains[1].partners == (0,)
    model = pm.build_model(sample)
    assert isinstance(model, pm.PreviewRefusalV1) and f"{pm.DOMAIN_WELD_PARTNER}=1" in model.detail and f"{pm.DOMAIN_NO_CERTIFICATE}=1" in model.detail
    # Плоский сосед ничего не меняет: его нормаль одна, смесь вдоль стены постоянна.
    flat = _fold_results(0.25)
    flat_arrays = dataclasses.replace(production_mesh.build_mesh_arrays(flat, OFFSET), domain_partners=((), (0,)))
    ok = pm.build_model(preview.build_sample(flat, flat_arrays, ("fold",), 0.25))
    assert isinstance(ok, pm.PreviewModelV1) and ok.modelled_domains == 2 and ok.approximate_domains == 0 and ok.approximate is None


# --------------------------------------------------------------------------
# 1b. PREVIEW_DEVELOPABLE_FROZEN_NORMAL_V1: развёртка двигается приближённо, и это названо
# --------------------------------------------------------------------------


def _display_fold(alpha=0.25, **kwargs):
    """Складка, у которой пол - развёртка (ядро назвало его `approximate`, числа есть): плоская стена - точный домен."""

    return _fold_sample(alpha, floor_approximate=(pm.DEVELOPABLE_MARKER,), **kwargs)[0]


def test_a_developable_domain_is_served_as_display_approximate_and_named_apart_from_the_exact_ones():
    base = _display_fold()
    assert base.domains[0].approximate == (pm.DEVELOPABLE_MARKER,) and base.domains[0].markers == () and base.domains[1].approximate == ()
    assert base.domains[0].slopes is not None, "the numbers of a developable domain are the exact affine part of its batch"
    model = pm.build_model(base)
    assert isinstance(model, pm.PreviewModelV1)
    assert [row_[2] for row_ in model.rows] == [pm.DOMAIN_DISPLAY_APPROXIMATE, pm.DOMAIN_MODELLED] == ["PREVIEW_DEVELOPABLE_FROZEN_NORMAL_V1", "MODELLED"]
    assert model.modelled_domains == 2 and model.approximate_domains == 1 and model.reason_counts() == {}
    assert list(model.approximate) == [True, False]
    assert model.rows[0][4] == pm.DISPLAY_APPROXIMATE_REACH_RATIO and model.rows[1][4] is None
    frame = pm.evaluate(model, 0.26)
    assert frame.refusal == "" and frame.live_domains == 2 and frame.approximate_domains == 1 and frame.held == ()
    # Точная аффинная часть: складка синтетики аффинна, смещение у неё постоянно, поэтому кадр равен точному прогону.
    exact, _ = _fold_sample(0.26, floor_approximate=(pm.DEVELOPABLE_MARKER,))
    position_error, uv_error = _max_error(model, exact)
    assert position_error < 1e-12 and uv_error < 1e-12
    # Модель без развёрток не знает приближённых доменов.
    plain = pm.build_model(_fold_sample(0.25)[0])
    assert plain.approximate is None and plain.approximate_domains == 0 and pm.evaluate(plain, 0.26).approximate_domains == 0


def test_the_display_approximate_domain_is_held_beyond_its_reach_by_name_while_the_exact_one_keeps_moving():
    model = pm.build_model(_display_fold())
    inside = 0.25 * (1.0 + 0.8 * pm.DISPLAY_APPROXIMATE_REACH_RATIO)
    beyond = 0.25 * (1.0 + 1.5 * pm.DISPLAY_APPROXIMATE_REACH_RATIO)
    assert pm.evaluate(model, inside).live_domains == 2
    frame = pm.evaluate(model, beyond)
    assert frame.live_domains == 1 and frame.approximate_domains == 0
    assert frame.held == ((0, pm.DOMAIN_HELD_DISPLAY_REACH),) and pm.DOMAIN_HELD_DISPLAY_REACH == "PREVIEW_DEVELOPABLE_FROZEN_NORMAL_REACH_END"
    # Ниже базы то же: досягаемость симметрична.
    assert pm.evaluate(model, 0.25 * (1.0 - 1.5 * pm.DISPLAY_APPROXIMATE_REACH_RATIO)).held == ((0, pm.DOMAIN_HELD_DISPLAY_REACH),)


def test_the_reach_of_a_display_approximate_domain_follows_the_error_estimate_of_its_own_normal_rate():
    """Оценка ошибки кадра - `offset * rate * dt * запас`; досягаемость - доля ширины базы, на которой оценка ещё в пределе (метры), не выше общего потолка."""

    alpha = 0.25
    reach = 0.02
    rate = pm.DISPLAY_APPROXIMATE_ERROR_LIMIT / (pm.DISPLAY_APPROXIMATE_ERROR_SAFETY * OFFSET * reach * alpha)
    base = _fold_sample(alpha, floor_approximate=(pm.DEVELOPABLE_MARKER,), floor_rate=rate)[0]
    assert base.offset == OFFSET and base.domains[0].approximate_rate == rate
    model = pm.build_model(base)
    assert model.rows[0][4] == pytest.approx(reach) and model.rows[1][4] is None
    estimate = lambda share: OFFSET * rate * (alpha * share) * pm.DISPLAY_APPROXIMATE_ERROR_SAFETY  # noqa: E731
    assert estimate(0.99 * reach) < pm.DISPLAY_APPROXIMATE_ERROR_LIMIT < estimate(1.01 * reach)
    assert pm.evaluate(model, alpha * (1.0 + 0.99 * reach)).approximate_domains == 1
    far = pm.evaluate(model, alpha * (1.0 + 1.01 * reach))
    assert far.approximate_domains == 0 and far.held == ((0, pm.DOMAIN_HELD_DISPLAY_REACH),) and far.live_domains == 1
    # Нормали не движутся (скорость ноль) либо смещение неизвестно (ноль): общий потолок; скорость бесконечна либо не число: домен назван и не двигается.
    assert pm._display_reach(OFFSET, 0.0, alpha) == pm.DISPLAY_APPROXIMATE_REACH_RATIO == pm._display_reach(0.0, 5.0, alpha)
    assert pm._display_reach(OFFSET, float("inf"), alpha) == 0.0 and pm._display_reach(OFFSET, float("nan"), alpha) == 0.0
    assert pm._display_reach(OFFSET, 1e-9, alpha) == pm.DISPLAY_APPROXIMATE_REACH_RATIO  # потолок не поднимается
    stopped = pm.build_model(_fold_sample(alpha, floor_approximate=(pm.DEVELOPABLE_MARKER,), floor_rate=float("inf"))[0])
    assert pm.evaluate(stopped, alpha * 1.001).held == ((0, pm.DOMAIN_HELD_DISPLAY_REACH),)
    # Сосед по стене берёт скорость нормали соседа-развёртки.
    results = _fold_results(alpha, floor_approximate=(pm.DEVELOPABLE_MARKER,), floor_rate=rate)
    arrays = dataclasses.replace(production_mesh.build_mesh_arrays(results, OFFSET), domain_partners=((), (0,)))
    partner = pm.build_model(preview.build_sample(results, arrays, ("fold",), alpha))
    assert partner.rows[1][4] == pytest.approx(reach)


def test_a_display_approximate_domain_is_judged_by_its_own_limit_in_metres_and_goes_to_quarantine_by_the_same_ledger():
    approximate_model = pm.build_model(_display_fold())
    exact_model = pm.build_model(_fold_sample(0.25)[0])
    shifted, _ = _fold_sample(0.26, floor_approximate=(pm.DEVELOPABLE_MARKER,))
    own = np.setdiff1d(shifted.domains[0].vertices, shifted.domains[1].vertices)
    assert own.size
    positions = np.array(shifted.positions)
    positions[own, 2] += 3e-4  # больше предела приближённого домена (1e-4 м), меньше общего (0.5 % ширины = 1.3 мм)
    perturbed = dataclasses.replace(shifted, positions=positions)
    assert 3e-4 > pm.DISPLAY_APPROXIMATE_ERROR_LIMIT and 3e-4 < pm.DEVIATION_LIMIT_RATIO * 0.26
    check = pm.deviation(approximate_model, perturbed)
    assert check.refuted == (0,) and check.worst_ratio > 1.0
    assert pm.deviation(exact_model, perturbed).refuted == (), "the same error is within the limit of an exact domain"
    ledger, events = pm.advance_ledger(None, approximate_model, check)
    assert events == ((0, "floor", pm.DOMAIN_QUARANTINED),)
    rebuilt = pm.build_model(shifted, ledger)
    assert rebuilt.rows[0][2] == pm.DOMAIN_QUARANTINED and list(rebuilt.approximate) == [True, False], "a quarantined domain stays display-approximate for its shadow check"
    assert pm.evaluate(rebuilt, 0.26).held == ((0, pm.DOMAIN_QUARANTINED),)


def test_the_partner_of_a_display_approximate_neighbour_is_display_approximate_by_the_same_cause():
    """Узлы стены смещает нормаль соседа-развёртки: их кадр приближён по той же причине, что кадр соседа (`WALL_MITER_OFFSET_V1`)."""

    results = _fold_results(0.25, floor_approximate=(pm.DEVELOPABLE_MARKER,))
    arrays = dataclasses.replace(production_mesh.build_mesh_arrays(results, OFFSET), domain_partners=((), (0,)))
    sample = preview.build_sample(results, arrays, ("fold",), 0.25)
    assert sample.domains[1].partners == (0,) and sample.domains[1].approximate == ()
    model = pm.build_model(sample)
    assert [row_[2] for row_ in model.rows] == [pm.DOMAIN_DISPLAY_APPROXIMATE, pm.DOMAIN_DISPLAY_APPROXIMATE]
    assert list(model.approximate) == [True, True] and model.approximate_domains == 2 and model.modelled_domains == 2
    assert pm.evaluate(model, 0.26).approximate_domains == 2


# --------------------------------------------------------------------------
# 4. База битово
# --------------------------------------------------------------------------


def test_the_frame_at_the_base_width_is_the_base_bitwise():
    base, _ = _fold_sample(0.25)
    model = pm.build_model(base)
    frame = pm.evaluate(model, base.alpha)
    again = pm.base_frame(base)
    assert frame.outcome == pm.PREVIEW_MESH_FROM_AFFINE_CERTIFICATE_V1 and frame.live_domains == 2
    assert np.array_equal(frame.positions, again.positions) and np.array_equal(frame.uvs, again.uvs)
    assert frame.positions.dtype == np.float32 and frame.positions.shape == (base.vertex_count * 3,)
    assert np.array_equal(frame.positions, np.asarray(base.positions, dtype=np.float32).reshape(-1))


# --------------------------------------------------------------------------
# 5. Самопроверка и карантин
# --------------------------------------------------------------------------


def _pole(alpha):
    """Дробно-линейный закон из аудита F4: `f(a) = 0.01 / (1.11 - a)`; шесть процентов за базой 1.0 уже втрое круче хорды."""

    return 0.01 / (1.11 - alpha)


def _pole_sample(alpha):
    """Один домен на ЧЕТЫРЁХ вершинах и шести петлях (два треугольника), свободные вершины двигаются по `_pole`; сертификат честен в точке `alpha` (касательная), но закон неаффинен."""

    slope = 0.01 / (1.11 - alpha) ** 2
    batch = _synthetic_batch(
        alpha,
        free={"c": lambda w: (1.0, _pole(w), 0.0), "d": lambda w: (0.0, _pole(w), 0.0)},
        fixed={"a": (0.0, 0.0, 0.0), "b": (1.0, 0.0, 0.0)},
        faces=(("a", "b", "c"), ("a", "c", "d")),
        refs={},
        uv_axis=(0, 1),
    )
    results = [
        ProductionDomainResultV1(
            0, "pole", MATERIALIZED, batch, normal=(0.0, 0.0, 1.0), source_normal=(0.0, 0.0, 1.0),
            alpha_interval=_interval(alpha, 0.5, 1.1), structure_digest="pole-structure",
            parametric=_certificate(batch, {"c": (0.0, slope, 0.0), "d": (0.0, slope, 0.0)}, (0, 1), alpha=alpha),
        )
    ]
    arrays = production_mesh.build_mesh_arrays(results, 0.0)
    return preview.build_sample(results, arrays, ("pole",), alpha)


def _refuted_fold():
    """Неверная модель складки (ошибка в наклоне), точный прогон на 0.3 и итог сверки: оба домена опровергнуты."""

    base, _ = _fold_sample(0.25)
    good = pm.build_model(base)
    wrong = dataclasses.replace(good, a1=good.a1 + 0.5)  # ошибка в наклоне: превью уходит от точного
    exact, _ = _fold_sample(0.3)
    return base, good, wrong, exact, pm.deviation(wrong, exact)


def test_a_wrong_model_is_refuted_by_the_next_exact_run_and_its_domains_go_to_quarantine():
    base, good, wrong, exact, check = _refuted_fold()
    assert check.refusal == "" and check.domains_checked == 2
    assert check.refuted == (0, 1) and check.max_position > 0.01 and check.worst_ratio > 1.0
    assert check.relative_distance == pytest.approx(0.2)
    ledger, events = pm.advance_ledger(None, wrong, check)
    assert sorted(events) == [(0, "floor", pm.DOMAIN_QUARANTINED), (1, "wall", pm.DOMAIN_QUARANTINED)]
    entry = ledger.get(0, "floor")
    assert (entry.state, entry.refutations, entry.scale, entry.clean_checks) == (pm.TRUST_QUARANTINED, 1, 0.5, 0)
    assert entry.reach_cap == pytest.approx(pm.TRUST_CAP_FRACTION * 0.2) and ledger.counts() == {pm.TRUST_QUARANTINED: 2}
    # Модель нового образца строится под журналом: оба домена построены и придержаны, кадр отказан именем и равен базе.
    rebuilt = pm.build_model(exact, ledger)
    assert [row_[2] for row_ in rebuilt.rows] == [pm.DOMAIN_QUARANTINED, pm.DOMAIN_QUARANTINED]
    assert rebuilt.modelled_domains == 0 and rebuilt.reason_counts() == {pm.DOMAIN_QUARANTINED: 2}
    frame = pm.evaluate(rebuilt, 0.31)
    assert frame.refusal == pm.REFUSED_ALL_QUARANTINED == frame.outcome and frame.live_domains == 0
    assert frame.held == ((0, pm.DOMAIN_QUARANTINED), (1, pm.DOMAIN_QUARANTINED))
    assert np.array_equal(frame.positions, np.asarray(exact.positions, dtype=np.float32).reshape(-1))
    # Верная модель не опровергается; чужой ключ и отсутствие модели названы; без вердикта журнал не меняется.
    assert pm.deviation(good, exact).refuted == () and pm.deviation(good, exact).worst_ratio < 1e-6
    stranger, _ = _fold_sample(0.3, key=("other",))
    assert pm.deviation(good, stranger).refusal == pm.REFUSED_KEY
    assert pm.deviation(None, exact).refusal == pm.REFUSED_NO_MODEL
    assert pm.advance_ledger(ledger, wrong, pm.deviation(good, stranger)) == (ledger, ())


def test_a_sample_at_the_base_width_is_not_an_independent_check():
    base, _good, wrong, exact, check = _refuted_fold()
    ledger, _ = pm.advance_ledger(None, wrong, check)
    rebuilt = pm.build_model(exact, ledger)
    verdict = pm.deviation(rebuilt, exact)
    assert verdict.refusal == pm.REFUSED_NOT_INDEPENDENT and verdict.domains_checked == 0 and verdict.refuted == ()
    assert pm.advance_ledger(ledger, rebuilt, verdict) == (ledger, ())


def test_a_quarantined_domain_comes_back_after_a_clean_independent_check_with_a_reduced_reach():
    base, _good, wrong, exact, check = _refuted_fold()
    ledger, _ = pm.advance_ledger(None, wrong, check)
    rebuilt = pm.build_model(exact, ledger)
    # У самой базы любая модель чиста: сверка слишком близко не считается (расстояние меньше половины будущей досягаемости).
    close, _ = _fold_sample(0.3005)
    shadow = pm.deviation(rebuilt, close)
    assert shadow.domains_checked == 0 and shadow.shadow_clean == () and shadow.shadow_refuted == ()
    assert pm.advance_ledger(ledger, rebuilt, shadow)[1] == ()
    # Ширина внутри НОМИНАЛЬНОЙ области (0.27, 0.33), но за сжатым окном модели: сверка в тени чиста, домены возвращаются.
    independent, _ = _fold_sample(0.325)
    verdict = pm.deviation(rebuilt, independent)
    assert verdict.domains_checked == 0 and verdict.shadow_clean == (0, 1) and verdict.shadow_refuted == ()
    readmitted, events = pm.advance_ledger(ledger, rebuilt, verdict)
    assert sorted(name for _patch, _domain, name in events) == [pm.DOMAIN_READMITTED, pm.DOMAIN_READMITTED]
    assert readmitted.get(0, "floor").state == pm.TRUST_REDUCED and readmitted.get(1, "wall").state == pm.TRUST_REDUCED
    # Возврат со сжатой областью: аффинный домен получает половину номинальной области, но не больше капа опровержения.
    again = pm.build_model(independent, readmitted)
    assert [row_[2] for row_ in again.rows] == [pm.DOMAIN_MODELLED, pm.DOMAIN_MODELLED]
    assert [row_[4] for row_ in again.rows] == [pytest.approx(0.5 * pm.READMIT_NOMINAL_REACH_RATIO)] * 2
    assert pm.evaluate(again, 0.33).live_domains == 2
    beyond = pm.evaluate(again, 0.36)  # 10.8 % от базы: внутри интервала ядра (0.1, 0.6), за сжатой областью
    assert beyond.refusal == pm.REFUSED_OUTSIDE_EVERY_INTERVAL and beyond.held == ((0, pm.DOMAIN_HELD_OUTSIDE), (1, pm.DOMAIN_HELD_OUTSIDE))


def _verdict(*, refuted=(), shadow_clean=(), shadow_refuted=(), distance=0.1):
    return pm.DeviationV1(
        0.3, 0.1, 0.0, len(refuted), 0, 0, tuple(refuted), "", tuple(shadow_clean), tuple(shadow_refuted), distance, 20.0
    )


def test_a_domain_that_fails_again_shrinks_further_and_the_third_refutation_abandons_its_model():
    base, _ = _fold_sample(0.25)
    model = pm.build_model(base)
    ledger, events = pm.advance_ledger(None, model, _verdict(refuted=(0,), distance=0.2))
    first = ledger.get(0, "floor")
    assert [item[2] for item in events] == [pm.DOMAIN_QUARANTINED] and (first.refutations, first.scale) == (1, 0.5)
    assert first.reach_cap == pytest.approx(0.1)
    # Повторное опровержение В ТЕНИ: область сжата ещё вдвое (и кап по ближнему расстоянию), чистых сверок снова нет.
    ledger, events = pm.advance_ledger(ledger, model, _verdict(shadow_refuted=(0,), distance=0.04))
    second = ledger.get(0, "floor")
    assert [item[2] for item in events] == [pm.DOMAIN_QUARANTINED]
    assert (second.state, second.refutations, second.scale, second.clean_checks) == (pm.TRUST_QUARANTINED, 2, 0.25, 0)
    assert second.reach_cap == pytest.approx(0.02)
    ledger, events = pm.advance_ledger(ledger, model, _verdict(refuted=(0,), distance=0.1))
    third = ledger.get(0, "floor")
    assert [item[2] for item in events] == [pm.DOMAIN_ABANDONED] and (third.state, third.refutations) == (pm.TRUST_ABANDONED, 3)
    assert third.reach_cap == pytest.approx(0.02), "the cap only shrinks"
    # Снятый домен никаким числом чистых сверок не возвращается, а модель называет его.
    after, events = pm.advance_ledger(ledger, model, _verdict(shadow_clean=(0,)))
    assert events == () and after.get(0, "floor") == third
    rebuilt = pm.build_model(base, ledger)
    assert rebuilt.rows[0][2] == pm.DOMAIN_ABANDONED and rebuilt.rows[1][2] == pm.DOMAIN_MODELLED
    assert pm.evaluate(rebuilt, 0.3).held == ((0, pm.DOMAIN_ABANDONED),)
    # Журнал другого ключа образца забыт: он говорил про другую геометрию.
    foreign_base, _ = _fold_sample(0.25, key=("other",))
    fresh, events = pm.advance_ledger(ledger, pm.build_model(foreign_base), _verdict(refuted=(0,)))
    assert fresh.key == ("other",) and fresh.get(0, "floor").refutations == 1 and len(events) == 1


def test_a_certificate_that_does_not_hold_on_the_window_is_refuted_and_quarantined():
    """Аудит F4: `f(a) = 0.01 / (1.11 - a)`. Касательная в точке 1.0 — честный наклон в точке, но закон неаффинен, и на 1.09 точное 0.5, а модель говорит ~0.23: сверка это ловит."""

    base = _pole_sample(1.0)
    model = pm.build_model(base)
    assert model.rows[0][2:] == (pm.DOMAIN_MODELLED, 1, None), "an affine certificate believes itself on the whole kernel interval"
    assert pm.evaluate(model, 1.09).live_domains == 1, "and moves the mesh there"
    exact = _pole_sample(1.09)
    check = pm.deviation(model, exact)
    limit = pm.DEVIATION_LIMIT_RATIO * 1.09
    assert check.refuted == (0,) and check.max_position > 40.0 * limit, (check.max_position, limit)
    assert check.worst_ratio > 40.0 and check.relative_distance == pytest.approx(0.09)
    ledger, events = pm.advance_ledger(None, model, check)
    assert [item[2] for item in events] == [pm.DOMAIN_QUARANTINED]
    assert ledger.get(0, "pole").reach_cap == pytest.approx(pm.TRUST_CAP_FRACTION * 0.09)
    rebuilt = pm.build_model(exact, ledger)
    assert rebuilt.rows[0][2] == pm.DOMAIN_QUARANTINED
    held = pm.evaluate(rebuilt, 1.1)
    assert held.refusal == pm.REFUSED_ALL_QUARANTINED and held.held == ((0, pm.DOMAIN_QUARANTINED),)


# --------------------------------------------------------------------------
# Массивы меша: состав домена
# --------------------------------------------------------------------------


def test_the_mesh_arrays_carry_the_domain_layout_the_model_reads(row):
    first, _, arrays = row(0.25)
    second, _, other = row(0.2525)
    assert len(arrays.domain_keys) == len(arrays.domain_vertex_index) == len(arrays.domain_loop_counts) == len(arrays.domain_tokens) == ROW
    assert len(arrays.domain_partners) == ROW
    assert sum(arrays.domain_loop_counts) == len(arrays.uvs)
    assert max(max(item) for item in arrays.domain_vertex_index) < len(arrays.positions)
    # Токен — структура без чисел: тот же при другой ширине; он входит в образец, а не в дайджест массивов.
    assert arrays.domain_tokens == other.domain_tokens and arrays.digest != other.digest
    assert [item.token for item in first.domains] == list(arrays.domain_tokens)


# --------------------------------------------------------------------------
# Байты
# --------------------------------------------------------------------------


def test_the_model_reports_its_bytes(row):
    base, _, _ = row(0.25)
    model = pm.build_model(base)
    assert 0 < model.own_bytes and model.nbytes == model.own_bytes + base.nbytes
    # Слагаемые честные: сумма `nbytes` массивов, а не оценка.
    arrays = (model.a1, model.e1, model.vlow, model.vhigh, model.loop_counts, model.dlow, model.dhigh)
    assert model.own_bytes == sum(item.nbytes for item in arrays)
    assert model.a1.dtype == np.float64 and model.e1.dtype == np.float64


# --------------------------------------------------------------------------
# 6. Меш принимает только свой кадр (владение мешем)
# --------------------------------------------------------------------------


_IDS = itertools.count(10_000)


class _Collection(list):
    def __init__(self, *items):
        super().__init__(items)
        self.sets = []

    def foreach_set(self, name, values):
        self.sets.append((name, np.array(values, copy=True)))

    def get(self, name):
        return next((item for item in self if getattr(item, "name", None) == name), None)


class _Layout:
    """`mesh.loops` и `mesh.polygons`: длина, `foreach_get` и счётчик чтений (отпечаток раскладки перечитывается не каждый кадр)."""

    def __init__(self, values):
        self.values = np.asarray(values, dtype=np.int32)
        self.reads = 0
        self.broken = False

    def __len__(self):
        return len(self.values)

    def foreach_get(self, name, buffer):
        if self.broken:
            raise RuntimeError("synthetic read failure")
        self.reads += 1
        buffer[:] = self.values


class _Mesh(dict):
    def __init__(self, vertices, loops, width, name="CFTUV_Decal", corners=None):
        super().__init__({production_mesh.DECAL_WIDTH_PROPERTY: width})
        self.name = name
        self.vertices = _Collection(*range(vertices))
        layer = SimpleNamespace(name=production_mesh.DECAL_UV_LAYER, data=_Collection(*range(loops)))
        self.uv_layers = _Collection(layer)
        owner = SimpleNamespace(name=production_mesh.DECAL_OWNER_ATTRIBUTE, data=_Collection(*range(loops // 3)))
        self.attributes = _Collection(owner)  # атрибут граней `cftuv_owner`: его пишет точный lean-кадр (WIDTH_REFRESH_LEAN_V1)
        self.loops = _Layout(np.arange(loops) % max(vertices, 1) if corners is None else corners)
        self.polygons = _Layout(np.arange(loops // 3) * 3)
        self.edges = _Collection(*range(vertices + loops // 3))
        self.pointer, self.session_uid = next(_IDS), next(_IDS)
        self.updated = 0

    def as_pointer(self):
        return self.pointer

    def update(self, *args):  # `dict.update` (копия свойств) и `Mesh.update()` (запись геометрии) в одном имени, как у Blender-мока
        if args:
            super().update(*args)
        else:
            self.updated += 1


class _Decal(dict):
    def __init__(self, mesh, revision, mode="OBJECT"):
        super().__init__({production_mesh.DECAL_REVISION_PROPERTY: revision})
        self.data = mesh
        self.mode = mode
        self.pointer, self.session_uid = next(_IDS), next(_IDS)

    def as_pointer(self):
        return self.pointer


def _serve(monkeypatch, decal_for):
    monkeypatch.setitem(
        sys.modules,
        "bpy",
        types.SimpleNamespace(data=types.SimpleNamespace(objects={"src": SimpleNamespace(name="src")})),
    )
    monkeypatch.setattr(production_mesh, "find_decal_object", lambda source: decal_for)


def _owned(monkeypatch, *, samples=None, corners=None, with_owner=True):
    """Образец, модель и меш декали, чьё владение записал «точный путь» (`capture_ownership`): мир одного кадра."""

    if samples is None:
        samples = (_fold_sample(0.25)[0],)
    (base,) = samples
    model = pm.build_model(base)
    mesh = _Mesh(base.vertex_count, base.loop_count, base.alpha, corners=corners)
    decal = _Decal(mesh, base.source_revision)
    controller = EnvelopeDebugSessionController()
    controller.width_build = SimpleNamespace(source_name="src")
    controller.width_displayed, controller.width_model = base, model
    _serve(monkeypatch, decal)
    if with_owner:
        assert preview.capture_ownership(controller, decal, base) is not None
    return SimpleNamespace(base=base, model=model, mesh=mesh, decal=decal, controller=controller, meshes=[mesh])


def _writes(world):
    return sum(len(item.vertices.sets) + len(item.uv_layers[0].data.sets) for item in world.meshes)


def _dropped(controller, reason):
    return controller.width_preview_log.dropped == {f"{preview.MODEL_DROPPED}:{reason}": 1}


def _is_dropped_clean(controller, reason):
    """Модель, образцы и владение сняты, причина названа в журнале и в строке статуса."""

    assert controller.width_model is None and controller.width_displayed is None
    assert controller.width_mesh_owner is None and controller.width_mesh_preview is None
    assert _dropped(controller, reason), controller.width_preview_log.dropped
    assert f"none ({preview.MODEL_DROPPED}:{reason})" in preview.status_lines(controller)[0]
    return True


def test_the_mesh_takes_a_frame_only_when_it_is_the_owned_base_of_the_model(monkeypatch):
    world = _owned(monkeypatch)
    controller, base, good, decal = world.controller, world.base, world.mesh, world.decal

    state = preview.preview_mesh_now(controller, 0.27)

    assert state is not None and state.outcome == pm.PREVIEW_MESH_FROM_AFFINE_CERTIFICATE_V1 and state.live == 2 and state.serial == 1
    assert [name for name, _ in good.vertices.sets] == ["co"] and good.uv_layers[0].data.sets[0][0] == "uv" and good.updated == 1
    assert good.vertices.sets[0][1].dtype == np.float32 and good.vertices.sets[0][1].shape == (base.vertex_count * 3,)
    assert preview.preview_mesh_now(controller, 0.28).serial == 2
    lines = preview.status_lines(controller)
    assert "PREVIEW_MESH_FROM_AFFINE_CERTIFICATE_V1 preview (exact affine certificate; structure verified afterwards)" in lines[0] and "2 domains moving" in lines[0]
    assert lines[1].startswith("Preview model (affine certificate of one exact run, valid inside the certified event interval): 2/2 domains modelled")

    # Отмена: меш назад на базу, побитово то, что записал точный путь.
    assert preview.restore_base_mesh(controller) is True and controller.width_mesh_preview is None
    assert np.array_equal(good.vertices.sets[-1][1], np.asarray(base.positions, dtype=np.float32).reshape(-1))
    assert preview.restore_base_mesh(controller) is False  # меш уже на базе

    # Edit-режим декали: модель снята строго (меш мог быть изменён руками), записи нет, причина названа.
    decal.mode = "EDIT"
    writes = len(good.vertices.sets)
    assert preview.preview_mesh_now(controller, 0.27) is None and len(good.vertices.sets) == writes
    assert controller.width_model is None and _dropped(controller, preview.DECAL_IN_EDIT_MODE)
    assert preview.preview_mesh_now(EnvelopeDebugSessionController(), 0.27) is None  # без модели кадра нет и ничего не названо лишнего


def _grow_a_vertex(world, _monkeypatch):
    world.mesh.vertices.append(len(world.mesh.vertices))


def _other_width(world, _monkeypatch):
    world.mesh[production_mesh.DECAL_WIDTH_PROPERTY] = world.base.alpha + 0.1


def _other_revision(world, _monkeypatch):
    world.decal[production_mesh.DECAL_REVISION_PROPERTY] = "another"


def _another_face_count(world, _monkeypatch):
    world.mesh.polygons = _Layout(np.arange(len(world.mesh.polygons) + 1) * 3)


def _another_datablock_with_the_same_properties(world, _monkeypatch):
    """Копия датаблока: те же счётчики, те же свойства, та же раскладка, другой указатель и `session_uid`."""

    twin = _Mesh(world.base.vertex_count, world.base.loop_count, world.base.alpha, corners=world.mesh.loops.values)
    twin.update(world.mesh)
    world.decal.data = twin
    world.meshes.append(twin)


def _another_object_with_the_same_mesh(world, monkeypatch):
    other = _Decal(world.mesh, world.base.source_revision)
    _serve(monkeypatch, other)


def _no_decal_object(_world, monkeypatch):
    _serve(monkeypatch, None)


def _no_ownership(world, _monkeypatch):
    world.controller.width_mesh_owner = None


def _the_model_of_another_sample(world, _monkeypatch):
    newer, _ = _fold_sample(0.2575)
    world.controller.width_model = pm.build_model(newer)


@pytest.mark.parametrize(
    "mutate, reason",
    [
        (_grow_a_vertex, preview.MESH_NOT_THE_BASE),
        (_other_width, preview.MESH_NOT_THE_BASE),
        (_other_revision, preview.MESH_NOT_THE_BASE),
        (_another_face_count, preview.MESH_NOT_THE_BASE),
        (_another_datablock_with_the_same_properties, preview.MESH_DATABLOCK_REPLACED),
        (_another_object_with_the_same_mesh, preview.DECAL_OBJECT_REPLACED),
        (_no_decal_object, preview.DECAL_GONE),
        (_no_ownership, preview.OWNERSHIP_UNKNOWN),
        (_the_model_of_another_sample, preview.LAYOUT_GENERATION_STALE),
    ],
    ids=lambda item: getattr(item, "__name__", item),
)
def test_every_reason_the_mesh_is_not_the_owned_base_drops_the_model_by_name_and_writes_nothing(monkeypatch, mutate, reason):
    world = _owned(monkeypatch)
    mutate(world, monkeypatch)

    assert preview.preview_mesh_now(world.controller, 0.27) is None
    assert _writes(world) == 0, "a mesh that is not the base never receives a frame"
    assert _is_dropped_clean(world.controller, reason)
    assert preview.restore_base_mesh(world.controller) is False


def test_two_meshes_with_the_same_counts_but_another_triangle_diagonal_are_told_apart(monkeypatch):
    """Аудит F2: четыре вершины, шесть петель, но диагонали двух треугольников разные; счётчики и свойства их не различают."""

    first = (0, 1, 2, 0, 2, 3)
    second = (0, 1, 3, 1, 2, 3)

    # 1. Другой датаблок с теми же счётчиками и свойствами: тождество (указатель и `session_uid`) отличает его от записанного точным путём.
    world = _owned(monkeypatch, samples=(_pole_sample(1.0),), corners=first)
    assert (world.base.vertex_count, world.base.loop_count) == (4, 6)
    rival = _Mesh(4, 6, world.base.alpha, corners=second)
    rival.update(world.mesh)
    world.decal.data = rival
    world.meshes.append(rival)
    assert preview.preview_mesh_now(world.controller, 1.02) is None and _writes(world) == 0
    assert _is_dropped_clean(world.controller, preview.MESH_DATABLOCK_REPLACED)

    # 2. Тот же датаблок, раскладку изменили на месте (ручная правка диагонали): кадры внутри интервала перепроверки не перечитывают
    #    раскладку (цена кадра прежняя), а сеть через интервал её видит и снимает модель, не записав кадр на чужую раскладку.
    world = _owned(monkeypatch, samples=(_pole_sample(1.0),), corners=first)
    owner = world.controller.width_mesh_owner
    reads = world.mesh.loops.reads
    assert reads == 1 and owner.recheck_after() >= preview.LAYOUT_RECHECK_SECONDS
    world.mesh.loops.values[:] = second
    for width in (1.01, 1.02, 1.03):
        assert preview.preview_mesh_now(world.controller, width) is not None
    assert world.mesh.loops.reads == reads, "frames inside the interval do not re-read the layout"
    owner.verified_at -= 10.0
    written = _writes(world)
    assert preview.preview_mesh_now(world.controller, 1.04) is None and _writes(world) == written
    assert world.mesh.loops.reads == reads + 1 and world.controller.width_preview_log.layout_checks == 1
    assert _is_dropped_clean(world.controller, preview.MESH_LAYOUT_CHANGED)

    # 3. Полиномиальная раскладка: другие начала граней при тех же индексах вершин петель тоже другая раскладка.
    world = _owned(monkeypatch, samples=(_pole_sample(1.0),), corners=first)
    world.mesh.polygons.values[:] = (0, 4)
    world.controller.width_mesh_owner.verified_at -= 10.0
    assert preview.preview_mesh_now(world.controller, 1.02) is None
    assert _is_dropped_clean(world.controller, preview.MESH_LAYOUT_CHANGED)


def test_the_layout_recheck_interval_is_never_below_the_floor_and_follows_the_cost_of_a_read(monkeypatch):
    world = _owned(monkeypatch)
    owner = world.controller.width_mesh_owner
    owner.check_seconds = 0.0
    assert owner.recheck_after() == preview.LAYOUT_RECHECK_SECONDS
    owner.check_seconds = 0.1  # большой меш: чтение 100 мс -> проверка не чаще раза в `LAYOUT_RECHECK_COST_RATIO` чтений
    assert owner.recheck_after() == pytest.approx(0.1 * preview.LAYOUT_RECHECK_COST_RATIO)


def _update(identifier, *, geometry=True):
    return SimpleNamespace(id=SimpleNamespace(session_uid=identifier), is_updated_geometry=geometry)


def test_a_geometry_update_of_the_decal_that_we_did_not_cause_drops_the_model_at_once(monkeypatch):
    world = _owned(monkeypatch)
    controller, owner = world.controller, world.controller.width_mesh_owner
    mesh_uid, object_uid = world.mesh.session_uid, world.decal.session_uid

    # Запись точного пути оставила метку: первое обновление геометрии объекта и меша — наше, метка гаснет (оба ID в одном обновлении).
    assert owner.pending_own_update
    assert preview.note_decal_updates(controller, [_update(mesh_uid), _update(object_uid)]) == ""
    assert not owner.pending_own_update and controller.width_model is not None
    # Кадр пишет сам и снова оставляет метку: обновление после него тоже наше.
    assert preview.preview_mesh_now(controller, 0.27) is not None and owner.pending_own_update
    assert preview.note_decal_updates(controller, [_update(object_uid)]) == "" and not owner.pending_own_update
    # Чужие ID и обновления без геометрии не значат ничего.
    assert preview.note_decal_updates(controller, [_update(12345), _update(mesh_uid, geometry=False)]) == ""
    assert controller.width_model is not None
    # Внешнее обновление геометрии (вход в Edit, правка, применение модификатора): модель снята строго, до следующего кадра.
    writes = _writes(world)
    assert preview.note_decal_updates(controller, [_update(mesh_uid)]) == preview.DECAL_CHANGED_EXTERNALLY
    assert _is_dropped_clean(controller, preview.DECAL_CHANGED_EXTERNALLY)
    assert preview.preview_mesh_now(controller, 0.28) is None and _writes(world) == writes
    # Без владения обработчик ничего не просматривает.
    assert preview.note_decal_updates(controller, [_update(mesh_uid)]) == ""


def test_a_depsgraph_update_carries_an_evaluated_copy_and_the_original_is_what_is_compared(monkeypatch):
    """`DepsgraphUpdate.id` в Blender — вычисленная копия с нулевым `session_uid`; тождество берётся у `id.original` (найдено смоком, не угадано)."""

    world = _owned(monkeypatch)
    controller, owner = world.controller, world.controller.width_mesh_owner
    owner.pending_own_update = False

    def evaluated(uid, original):
        return SimpleNamespace(id=SimpleNamespace(session_uid=0, original=original), is_updated_geometry=True)

    stranger = SimpleNamespace(session_uid=424242)
    assert preview.note_decal_updates(controller, [evaluated(0, None), evaluated(0, stranger)]) == ""
    assert controller.width_model is not None
    mine = SimpleNamespace(session_uid=world.mesh.session_uid)
    assert preview.note_decal_updates(controller, [evaluated(0, mine)]) == preview.DECAL_CHANGED_EXTERNALLY
    assert _is_dropped_clean(controller, preview.DECAL_CHANGED_EXTERNALLY)


def test_undo_and_redo_drop_the_model_strictly_and_a_quiet_session_names_nothing(monkeypatch):
    world = _owned(monkeypatch)
    controller = world.controller
    assert preview.preview_mesh_now(controller, 0.27) is not None

    preview.note_history(controller)

    assert _is_dropped_clean(controller, preview.HISTORY_STEP)
    assert controller.width_displayed is None, "without a displayed sample there is nothing to model: the next exact result starts over"
    assert preview.preview_mesh_now(controller, 0.28) is None
    quiet = EnvelopeDebugSessionController()
    preview.note_history(quiet)
    assert quiet.width_preview_log is None and quiet.width_model_refusal is None


def test_the_exact_write_records_a_new_layout_generation_and_the_ownership_belongs_to_its_sample(monkeypatch):
    world = _owned(monkeypatch)
    controller = world.controller
    first = controller.width_mesh_owner
    assert first.generation == 1 and controller.width_layout_generation == 1 and preview.log_of(controller).layout_generation == 1
    assert first.sample is world.base and first.object_pointer == world.decal.pointer and first.mesh_uid == world.mesh.session_uid
    assert (first.vertices, first.loops, first.polygons, first.edges) == (
        len(world.mesh.vertices), len(world.mesh.loops), len(world.mesh.polygons), len(world.mesh.edges)
    )
    # Следующая точная запись — следующее поколение раскладки; тот же меш даёт тот же отпечаток.
    second = preview.capture_ownership(controller, world.decal, world.base)
    assert second.generation == 2 and second.layout == first.layout and controller.width_layout_generation == 2
    # Сброс сессии (кнопка с полным сбросом, загрузка файла) забывает владение и счётчик.
    controller.reset_width_mesh_preview()
    assert controller.width_mesh_owner is None and controller.width_layout_generation == 0


def test_a_failed_ownership_read_is_named_and_the_frame_stays_silent(monkeypatch):
    world = _owned(monkeypatch, with_owner=False)
    controller = world.controller
    world.mesh.loops.broken = True
    assert preview.capture_ownership(controller, world.decal, world.base) is None and controller.width_mesh_owner is None
    assert controller.width_preview_log.refusals == {f"{preview.OWNERSHIP_CAPTURE_FAILED}:RuntimeError": 1}
    world.mesh.loops.broken = False
    assert preview.preview_mesh_now(controller, 0.27) is None and _writes(world) == 0
    assert _is_dropped_clean(controller, preview.OWNERSHIP_UNKNOWN)
    # Объекта нет вовсе (кнопка не создала декаль): владения нет, и называть нечего.
    assert preview.capture_ownership(controller, None, world.base) is None and preview.capture_ownership(controller, world.decal, None) is None


def test_the_preview_writer_names_a_mesh_of_another_shape_and_never_writes_into_it():
    mesh = _Mesh(4, 6, 0.25)
    with pytest.raises(production_mesh.ProductionWriteError) as error:
        production_mesh.write_preview_geometry(mesh, np.zeros(9, dtype=np.float32), np.zeros(12, dtype=np.float32))
    assert error.value.outcome == production_mesh.OUTCOME_PREVIEW_SHAPE_MISMATCH and mesh.vertices.sets == []
    mesh.uv_layers.clear()
    with pytest.raises(production_mesh.ProductionWriteError) as error:
        production_mesh.write_preview_geometry(mesh, np.zeros(12, dtype=np.float32), np.zeros(12, dtype=np.float32))
    assert error.value.outcome == production_mesh.OUTCOME_PREVIEW_NO_UV_LAYER
    fresh = _Mesh(4, 6, 0.25)
    production_mesh.write_preview_geometry(fresh, np.arange(12, dtype=np.float32), np.arange(12, dtype=np.float32))
    assert fresh.updated == 1 and fresh.vertices.sets[0][0] == "co" and fresh.uv_layers[0].data.sets[0][0] == "uv"


# --------------------------------------------------------------------------
# 7. Жизнь сессии
# --------------------------------------------------------------------------


def _live(alpha, displayed, previous, key=("fold",), trust=None):
    results = _fold_results(alpha)
    run = SimpleNamespace(results=results)
    return preview.finish_live_run(run, alpha=alpha, offset=OFFSET, key=key, displayed=displayed, previous=previous, trust=trust)


def test_the_session_builds_the_model_from_the_button_sample_at_the_first_frame_and_from_the_exact_results():
    controller = EnvelopeDebugSessionController()
    button, _ = _fold_sample(0.25)
    preview.note_button_display(controller, button)
    assert controller.width_displayed is button and controller.width_model is None and controller.width_model_refusal is None

    # Модель образца кнопки строится при первом обращении (вход инструмента), без единого прогона.
    assert preview.ensure_model(controller) is True
    assert isinstance(controller.width_model, pm.PreviewModelV1) and controller.width_model.base is button
    assert preview.log_of(controller).models == 1 and preview.ensure_model(controller) is True and preview.log_of(controller).models == 1

    # Точный результат на новой ширине: образец на экране — он, прежняя модель сверена (аффинный сертификат держится), новая построена из ЕГО сертификатов.
    old_model = controller.width_model
    exact_live = _live(0.2575, controller.width_displayed, old_model)
    assert exact_live.check is not None and exact_live.check.refuted == () and exact_live.check.max_position <= EXACT_TOLERANCE
    preview.note_exact_display(controller, exact_live)
    assert controller.width_displayed is exact_live.sample
    assert controller.width_model is exact_live.model and controller.width_model.base is exact_live.sample
    assert controller.width_mesh_preview is None
    assert preview.log_of(controller).checks == 1 and preview.log_of(controller).max_position_error <= EXACT_TOLERANCE

    # Без образца на экране модели строить не из чего, и названо это не отказом, а отсутствием.
    empty = EnvelopeDebugSessionController()
    assert preview.ensure_model(empty) is False and empty.width_model is None and empty.width_model_refusal is None


def test_a_model_that_cannot_be_built_is_named_and_never_fails_the_exact_result(monkeypatch):
    button, _ = _fold_sample(0.25)

    def boom(*_args):
        raise ArithmeticError("synthetic")

    monkeypatch.setattr(preview, "build_model", boom)
    broken = _live(0.2575, button, None)
    assert broken.model.outcome == preview.MODEL_BUILD_FAILED and "ArithmeticError" in broken.model.detail
    controller = EnvelopeDebugSessionController()
    preview.note_exact_display(controller, broken)
    assert controller.width_displayed is broken.sample and controller.width_model is None
    assert preview.log_of(controller).refusals == {preview.MODEL_BUILD_FAILED: 1}
    assert "Preview model: none (PREVIEW_MODEL_BUILD_FAILED)" in preview.status_lines(controller)[0]
    # Отказ назван один раз и второй раз не пробуется (`ensure_model` не крутит сломанное построение на каждом кадре).
    assert preview.ensure_model(controller) is False and preview.log_of(controller).refusals == {preview.MODEL_BUILD_FAILED: 1}


def test_the_key_names_everything_the_geometry_depends_on():
    record = SimpleNamespace(
        source_name="src", source_digest="d", kernel_backend="PYTHON", skeleton_backend="PYTHON", embedding_backend="PYTHON", density="2", stretch_percent=20,
        dissolve_percent=0.39, invalidation_count=3,
    )
    key = preview.sample_key(record, 0.02)
    for field_name, value in (
        ("source_name", "other"), ("source_digest", "e"), ("kernel_backend", "NATIVE:abc"), ("skeleton_backend", "NATIVE"), ("embedding_backend", "NATIVE"), ("density", "4"),
        ("stretch_percent", 42), ("dissolve_percent", 0.5), ("invalidation_count", 4),
    ):
        assert preview.sample_key(SimpleNamespace(**{**vars(record), field_name: value}), 0.02) != key, field_name
    assert preview.sample_key(record, 0.03) != key


def test_the_session_carries_the_refutation_into_the_ledger_the_status_and_the_next_model():
    controller = EnvelopeDebugSessionController()
    button = _fold_sample(0.25)[0]
    preview.note_button_display(controller, button)
    good = pm.build_model(button)
    wrong = dataclasses.replace(good, a1=good.a1 + 0.5)
    controller.width_model = wrong

    # Точный результат опровергает прежнюю модель: журнал, события, новая модель с карантином, строки статуса.
    refuting = _live(0.3, button, wrong, trust=controller.width_trust)
    assert refuting.check.refuted == (0, 1) and [item[2] for item in refuting.trust_events] == [pm.DOMAIN_QUARANTINED] * 2
    assert [row_[2] for row_ in refuting.model.rows] == [pm.DOMAIN_QUARANTINED] * 2
    preview.note_exact_display(controller, refuting)
    log = preview.log_of(controller)
    assert controller.width_trust is refuting.trust and controller.width_trust.counts() == {pm.TRUST_QUARANTINED: 2}
    assert log.trust_events == {pm.DOMAIN_QUARANTINED: 2} and log.domains_refuted == 2 and log.max_deviation_ratio > 1.0
    lines = preview.status_lines(controller)
    assert any(line.startswith("Preview trust: 2 quarantined") for line in lines), lines
    assert any(line.startswith("Last exact check:") and "2 refuted" in line for line in lines), lines
    assert any("PREVIEW_DOMAIN_QUARANTINED_AFTER_REFUTATION 2" in line for line in lines), lines

    # Следующее точное обновление: чистая независимая сверка в тени возвращает домены со сжатой областью.
    returning = _live(0.325, controller.width_displayed, controller.width_model, trust=controller.width_trust)
    assert returning.check.shadow_clean == (0, 1) and returning.check.refuted == ()
    assert [item[2] for item in returning.trust_events] == [pm.DOMAIN_READMITTED] * 2
    assert [row_[2] for row_ in returning.model.rows] == [pm.DOMAIN_MODELLED] * 2
    preview.note_exact_display(controller, returning)
    assert controller.width_trust.counts() == {pm.TRUST_REDUCED: 2}
    assert preview.log_of(controller).trust_events == {pm.DOMAIN_QUARANTINED: 2, pm.DOMAIN_READMITTED: 2}

    # Кнопка — явная пересборка: журнал доверия забыт.
    preview.note_button_display(controller, button)
    assert controller.width_trust is None


def test_the_preview_status_names_the_certificate_and_says_the_structure_is_verified_afterwards(monkeypatch):
    world = _owned(monkeypatch)
    controller = world.controller
    preview.preview_mesh_now(controller, 0.27)
    text = "\n".join(preview.status_lines(controller))
    assert "affine certificate" in text and "structure verified afterwards" in text
    assert "approximate" not in text, text  # аффинность внутри интервала заверена ядром точно; оговорена только структура
    controller.width_model = None
    controller.width_model_refusal = pm.PreviewRefusalV1("PREVIEW_NO_MODELLED_DOMAIN")
    assert preview.status_lines(controller)[-1] == "Preview model: none (PREVIEW_NO_MODELLED_DOMAIN) - the line overlay only"


def test_the_status_names_display_approximate_domains_apart_from_the_exact_ones_and_the_state_counts_them(monkeypatch):
    world = _owned(monkeypatch, samples=(_display_fold(),))
    state = preview.preview_mesh_now(world.controller, 0.26)
    assert state.live == 2 and state.approximate == 1 and state.held == 0
    text = "\n".join(preview.status_lines(world.controller))
    assert "(1 exact affine, 1 display-approximate PREVIEW_DEVELOPABLE_FROZEN_NORMAL_V1:" in text and "error limit 0.1 mm" in text
    assert "(1 exact, 1 display-approximate PREVIEW_DEVELOPABLE_FROZEN_NORMAL_V1)" in text
    # Ширина за досягаемостью: приближённый домен придержан под своим именем, точный двигается, в статусе приближённых уже нет.
    far = preview.preview_mesh_now(world.controller, 0.25 * (1.0 + 1.5 * pm.DISPLAY_APPROXIMATE_REACH_RATIO))
    assert far.live == 1 and far.approximate == 0 and far.held == 1
    assert "display-approximate" not in "\n".join(preview.status_lines(world.controller)).split("Preview model")[0]


# --------------------------------------------------------------------------
# 9. Линии превью только там, где меш не движется (PREVIEW_CONTOUR_ONLY_WHERE_MODEL_HELD_V1)
# --------------------------------------------------------------------------


def _contour_world(monkeypatch, *, wall_high=0.3):
    """Мир кадра на сгибе из двух патчей (пол — патч 0, стена — патч 1); стена живёт до `wall_high`. Подмена `compute_width_preview` пишет `skip_patches` вызовов."""

    from cftuv import envelope_width_live as live
    from cftuv.envelope_width_preview import PREVIEW_BINARY64_V1, WidthPreviewV1

    base, _ = _fold_sample(0.25)
    narrowed = dataclasses.replace(base, domains=(base.domains[0], dataclasses.replace(base.domains[1], high=wall_high)))
    world = _owned(monkeypatch, samples=(narrowed,))
    world.controller.width_build = SimpleNamespace(source_name="src", preview_inputs=object())
    world.skips = []

    def fake_contour(inputs, width, *, lift=0.0, skip_patches=frozenset()):
        world.skips.append(frozenset(skip_patches))
        return WidthPreviewV1(PREVIEW_BINARY64_V1, float(width), float(lift), (), (), 0.0)

    monkeypatch.setattr(live, "compute_width_preview", fake_contour)
    world.live = live
    return world


def test_the_contour_is_computed_only_for_patches_the_mesh_model_does_not_move(monkeypatch):
    world = _contour_world(monkeypatch)
    controller, live = world.controller, world.live

    # Обе стороны живы: меш двигает оба патча, линии не нужны нигде.
    state = live.width_event_preview(controller, 0.27, 0.02)
    assert state.moving == frozenset({0, 1}) and state.held == 0 and world.skips[-1] == frozenset({0, 1})
    # Интервал стены кончился на 0.3: стена (патч 1) придержана на последней точной геометрии — линии остаются на ней, пол движется сам.
    state = live.width_event_preview(controller, 0.4, 0.02)
    assert state.moving == frozenset({0}) and state.held_patches == (1,) and world.skips[-1] == frozenset({0})
    assert controller.width_mesh_preview is state and controller.width_preview.preview.width == 0.4


def test_without_a_live_frame_the_contour_covers_every_path_as_before(monkeypatch):
    world = _contour_world(monkeypatch)
    controller, live = world.controller, world.live

    # 1. Ширина за интервалом каждого домена: кадр отказан именем, меш на базе, линии всех путей.
    state = live.width_event_preview(controller, 0.9, 0.02)
    assert state.outcome == pm.REFUSED_OUTSIDE_EVERY_INTERVAL and state.moving == frozenset() and world.skips[-1] == frozenset()
    # 2. Меш не принимает кадр (Edit): модель снята именем, кадра нет, линии всех путей.
    world.decal.mode = "EDIT"
    assert live.width_event_preview(controller, 0.27, 0.02) is None and controller.width_model is None
    assert world.skips[-1] == frozenset() and len(world.skips) == 2
    # 3. Модели нет и построить её не из чего (образец снят вместе с моделью): линии всех путей.
    assert live.width_event_preview(controller, 0.27, 0.02) is None and world.skips[-1] == frozenset()


def test_the_tool_entry_asks_the_model_without_writing_the_mesh(monkeypatch):
    world = _contour_world(monkeypatch)
    controller = world.controller
    writes = _writes(world)

    assert preview.moving_now(controller, 0.4) == frozenset({0}) and preview.moving_now(controller, 0.27) == frozenset({0, 1})
    assert preview.moving_now(controller, 0.9) == frozenset()  # ни один домен не жив: как отказанный кадр
    assert preview.moving_now(controller, float("nan")) == frozenset() and preview.moving_now(controller, -1.0) == frozenset()
    assert _writes(world) == writes and controller.width_mesh_preview is None
    controller.width_model, controller.width_displayed = None, None
    assert preview.moving_now(controller, 0.27) == frozenset()


def test_a_quarantined_domain_keeps_its_contour_and_a_patch_outside_the_model_is_never_called_moving(monkeypatch):
    world = _contour_world(monkeypatch, wall_high=0.6)
    model = world.model
    quarantined = np.array([False, True])
    held_model = dataclasses.replace(model, quarantined=quarantined, vertex_quarantined=np.zeros(model.base.vertex_count, dtype=bool))

    assert pm.moving_patches_at(held_model, 0.27) == frozenset({0})
    assert pm.moving_patches(held_model, [(1, pm.DOMAIN_QUARANTINED)]) == frozenset({0})
    assert 7 not in pm.moving_patches(held_model, ()) and pm.moving_patches(held_model, ()) == frozenset({0, 1})


def test_a_drag_event_goes_through_the_one_event_preview_and_the_header_follows(monkeypatch):
    from cftuv.envelope_width_adjust import KIND_MOVE, WidthAdjustSessionV1, WidthEventV1
    from cftuv.envelope_width_session import AdjustRuntimeV1, apply_step

    world = _contour_world(monkeypatch)
    controller = world.controller
    start = world.base.alpha
    session = WidthAdjustSessionV1(start, pivot=(0.0, 0.0), start_mouse=(10.0, 0.0), metres_per_pixel=1.0)
    runtime = AdjustRuntimeV1(session, controller, 0.02, start)
    context = SimpleNamespace(area=None)

    apply_step(context, runtime, session.handle(WidthEventV1(KIND_MOVE, 10.0 + 0.15, 0.0)))  # 0.40: стена придержана
    assert controller.width_mesh_preview.held_patches == (1,) and world.skips == [frozenset({0})]
    apply_step(context, runtime, session.handle(WidthEventV1(KIND_MOVE, 10.0 + 0.15, 0.0)))  # ширина та же: ни кадра, ни линий
    assert len(world.skips) == 1
