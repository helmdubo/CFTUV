"""Модель превью меша ширины (`envelope_width_preview_model`, `envelope_width_mesh_preview`): математика и жизнь сессии, без Blender.

Утверждения (каждое названо и проверено числами, а не словом):

1. ВНУТРИ ИНТЕРВАЛА ПРЕВЬЮ РАВНО ТОЧНОМУ. На настоящих доменах (ряд квадратов `quad_row_bundle`, полевой домен) и на синтетической складке из двух
   доменов с общими вершинами позиции меша и UV кадра модели на ширине внутри интервала равны точному прогону с отклонением не больше
   `EXACT_TOLERANCE` (1e-9) — по прямой через две точки и по квадрату через три; у кривого домена квадрат точнее прямой.
2. ВНЕ ИНТЕРВАЛА ОТКАЗАНО ИМЕНЕМ. Ширина за интервалом всех доменов — кадр отказан `PREVIEW_ALPHA_OUTSIDE_EVERY_INTERVAL` и равен базе;
   ширина за интервалом одного домена — домен придержан (`PREVIEW_DOMAIN_ALPHA_OUTSIDE_INTERVAL`) на геометрии базы, остальные двигаются.
3. МОДЕЛЬ ДОМЕНА ИМЕЕТ ПРИЧИНЫ. Другой образец за интервалом, смена структуры, пропавший домен, нет интервала, другой ключ, та же ширина —
   у каждого свой исход (`DOMAIN_*`, `REFUSED_*`), и общая вершина двух доменов не двигается, пока не смоделирован каждый.
4. БАЗА БИТОВО. Кадр на ширине базы — сама база; отмена возвращает меш побитово.
5. САМОПРОВЕРКА И КАРАНТИН (аудит F4, F5). Неверная модель ловится сверкой с точным прогоном (`deviation`), домен идёт в карантин журнала доверия и
   возвращается по правилу (чистая независимая сверка в тени, сжатая область, третье опровержение снимает модель домена); дробно-линейный закон
   из аудита ломает фиксированный процент досягаемости, и это названо, а не принято на слово.
6. МЕШ ПРИНИМАЕТ КАДР ТОЛЬКО СВОЙ (аудит F2). Владение (тождество объекта и датаблока, размеры, поколение раскладки, отпечаток раскладки точной записи),
   ширина и ревизия сверяются перед кадром; подмена датаблока с теми же свойствами, те же счётчики при другой диагонали, Edit, внешнее обновление
   геометрии в depsgraph и шаг истории снимают модель с названной причиной и не пишут в меш.
7. ЖИЗНЬ СЕССИИ. Кнопка, точный результат и затравка расставляют образцы так, как описано в модуле; модель и журнал доверия строит `finish_live_run`.
8. ИМЯ. Приблизительная модель нигде не названа сертификатом (стена `tests/test_architecture.py`); сертифицирован только интервал событий ядра.
"""

from __future__ import annotations

import dataclasses
import itertools
import sys
import types
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


def _interval(alpha, low=0.1, high=0.6, status=CERTIFIED):
    return AlphaIntervalV1(status, "", float(alpha), float(low), float(high), 0, 0)


def _fold_results(alpha, *, wall_interval=None, wall_token_suffix=""):
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
    return [
        ProductionDomainResultV1(
            0, "floor", MATERIALIZED, floor, normal=(0.0, 0.0, 1.0), source_normal=(0.0, 0.0, 1.0),
            alpha_interval=_interval(alpha), structure_digest="floor-structure",
        ),
        ProductionDomainResultV1(
            1, "wall", MATERIALIZED, wall, normal=(0.0, -1.0, 0.0), source_normal=(0.0, -1.0, 0.0),
            alpha_interval=wall_interval or _interval(alpha), structure_digest="wall-structure" + wall_token_suffix,
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
    near, _, _ = row(0.2505)
    far, _, _ = row(0.2495)
    assert all(item.status == CERTIFIED for item in base.domains)
    linear = pm.build_model(base, [near])
    quadratic = pm.build_model(base, [near, far])
    assert isinstance(linear, pm.PreviewModelV1) and isinstance(quadratic, pm.PreviewModelV1)
    assert linear.quadratic_domains == 0 and quadratic.quadratic_domains == len(base.domains)
    # Прямая через две точки верит себе на три процента ширины, квадрат у аффинного домена — на весь интервал ядра.
    for width in (0.2501, 0.2503, 0.2507, 0.249, 0.2574, 0.26, 0.3, 0.5):
        exact, _, _ = row(width)
        for model in (linear, quadratic):
            if model is linear and abs(width / base.alpha - 1.0) > pm.CHORD_REACH_RATIO:
                assert pm.evaluate(linear, width).refusal == pm.REFUSED_OUTSIDE_EVERY_INTERVAL, width
                continue
            position_error, uv_error = _max_error(model, exact)
            assert position_error <= EXACT_TOLERANCE and uv_error <= EXACT_TOLERANCE, (width, position_error, uv_error)
            check = pm.deviation(model, exact)
            assert check.refuted == () and check.domains_checked == len(base.domains) and check.max_position <= EXACT_TOLERANCE


def test_a_curved_domain_is_followed_better_by_the_quadratic_than_by_the_chord():
    """Вершина на ребре источника под резкой — дробно-линейная функция ширины: прямая через две точки хорда, квадрат через три точнее."""

    def curved(width):
        # Пересечение поворачивающейся прямой с неподвижной: параметр `t = w / (w + 0.5)` дробно-линеен.
        t = width / (width + 0.5)
        return (1.0, t, 3.0 * t)

    def results(width):
        batch = _synthetic_batch(
            width,
            free={"c": curved, "d": lambda w: (0.0, w, 0.0)},
            fixed={"a": (0.0, 0.0, 0.0), "b": (1.0, 0.0, 0.0)},
            faces=(("a", "b", "c"), ("a", "c", "d")),
            refs={},
            uv_axis=(0, 1),
        )
        return [
            ProductionDomainResultV1(
                0, "bent", MATERIALIZED, batch, normal=(0.0, 0.0, 1.0), source_normal=(0.0, 0.0, 1.0),
                alpha_interval=_interval(width, 0.05, 2.0), structure_digest="s",
            )
        ]

    def sample(width):
        found = results(width)
        arrays = production_mesh.build_mesh_arrays(found, 0.0)
        return preview.build_sample(found, arrays, ("bent",), width)

    base, near, far = sample(0.4), sample(0.405), sample(0.395)
    linear = pm.build_model(base, [near])
    quadratic = pm.build_model(base, [near, far])
    exact = sample(0.41)  # внутри досягаемости обеих моделей
    chord = _max_error(linear, exact)[0]
    parabola = _max_error(quadratic, exact)[0]
    assert chord > 1e-5, "the fixture must be curved enough to tell the two apart"
    assert parabola < chord / 4.0, (chord, parabola)
    # Кривизна значима, и модель это знает: досягаемость десять процентов, а не весь интервал ядра; прямая — три.
    assert quadratic.rows[0][3:] == (2, pm.CURVED_REACH_RATIO) and linear.rows[0][3:] == (1, pm.CHORD_REACH_RATIO)
    assert pm.evaluate(quadratic, 0.43).live_domains == 1  # 7.5 % от базы
    beyond = pm.evaluate(quadratic, 0.46)  # 15 % от базы: внутри интервала ядра `(0.05, 2.0)`, за досягаемостью модели
    assert beyond.refusal == pm.REFUSED_OUTSIDE_EVERY_INTERVAL and beyond.held == ((0, pm.DOMAIN_HELD_REACH),)
    assert pm.evaluate(linear, 0.43).held == ((0, pm.DOMAIN_HELD_REACH),)  # прямая за три процента не верит себе


def test_a_fold_of_two_domains_with_shared_vertices_is_followed_exactly():
    base, _ = _fold_sample(0.25)
    near, _ = _fold_sample(0.2525)
    far, _ = _fold_sample(0.2475)
    model = pm.build_model(base, [near, far])
    assert isinstance(model, pm.PreviewModelV1) and model.modelled_domains == 2
    # Аффинные домены: кривизна в пределах шума нулевая, досягаемость модели не ограничивает ничего, кроме интервала ядра.
    assert [row[3:] for row in model.rows] == [(2, None), (2, None)] and not model.a2.any() and not model.e2.any()
    for width in (0.26, 0.31, 0.45):
        exact, _ = _fold_sample(width)
        assert max(_max_error(model, exact)) <= EXACT_TOLERANCE, width
    # Общие вершины `a`, `b` (сварка по ссылке источника) стоят: их позиции не зависят от ширины.
    frame_positions, _uvs, _live = pm.positions_and_uvs(model, 0.4)
    shared = [index for index in range(base.vertex_count) if (model.a1[index] == 0).all()]
    assert len(shared) >= 2
    assert np.array_equal(frame_positions[shared], base.positions[shared])


# --------------------------------------------------------------------------
# 2. Вне интервала отказано именем
# --------------------------------------------------------------------------


def test_a_width_beyond_every_interval_is_refused_by_name_and_equals_the_base():
    base, _ = _fold_sample(0.25)
    near, _ = _fold_sample(0.2525)
    model = pm.build_model(base, [near])

    frame = pm.evaluate(model, 0.9)  # интервал `(0.1, 0.6)`

    assert frame.refusal == pm.REFUSED_OUTSIDE_EVERY_INTERVAL == frame.outcome
    assert frame.live_domains == 0 and frame.held_domains == 2
    assert {reason for _patch, reason in frame.held} == {pm.DOMAIN_HELD_OUTSIDE}
    assert np.array_equal(frame.positions, np.asarray(base.positions, dtype=np.float32).reshape(-1))
    assert np.array_equal(frame.uvs, np.asarray(base.uvs, dtype=np.float32).reshape(-1))
    for bad in (0.0, -1.0, float("nan"), float("inf")):
        assert pm.evaluate(model, bad).refusal == pm.REFUSED_ALPHA
    assert pm.evaluate(None, 0.3).refusal == pm.REFUSED_NO_MODEL


def test_a_domain_whose_interval_is_left_is_held_at_the_base_and_the_others_move():
    base, _ = _fold_sample(0.25)
    near, _ = _fold_sample(0.2525)
    far, _ = _fold_sample(0.2475)
    # Интервал стены кончается на 0.3: на 0.4 пол двигается, стена придержана и названа.
    narrow = dataclasses.replace(base.domains[1], high=0.3)
    narrowed = dataclasses.replace(base, domains=(base.domains[0], narrow))
    model = pm.build_model(narrowed, [near, far])
    frame = pm.evaluate(model, 0.4)
    assert (frame.live_domains, frame.held_domains) == (1, 1) and frame.outcome == pm.PREVIEW_MESH_FROM_INTERVAL_V1
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
    near, _ = _fold_sample(0.2525)

    # 1. Образец за интервалом базы.
    outside, _ = _fold_sample(0.7)
    refusal = pm.build_model(base, [outside])
    assert isinstance(refusal, pm.PreviewRefusalV1) and refusal.outcome == pm.REFUSED_NO_MODELLED_DOMAIN
    assert pm.DOMAIN_OUTSIDE in refusal.detail
    # 2. Структура домена другая (подпись ядра либо токен локальных массивов).
    switched, _ = _fold_sample(0.2525, wall_token_suffix="-other")
    model = pm.build_model(base, [switched])
    assert model.rows[1][2] == pm.DOMAIN_STRUCTURE_SWITCH and model.rows[0][2] == pm.DOMAIN_MODELLED
    retokened = dataclasses.replace(near, domains=(near.domains[0], dataclasses.replace(near.domains[1], token="x")))
    assert pm.build_model(base, [retokened]).rows[1][2] == pm.DOMAIN_STRUCTURE_SWITCH
    # 3. Домена нет в другом образце.
    missing = dataclasses.replace(near, domains=near.domains[:1])
    assert pm.build_model(base, [missing]).rows[1][2] == pm.DOMAIN_MISSING
    # 4. У домена нет заверенного интервала (`NOT_CERTIFIED`).
    uncertain, _ = _fold_sample(0.25, wall_interval=_interval(0.25, 0.25, 0.25, NOT_CERTIFIED))
    assert pm.build_model(uncertain, [near]).rows[1][2] == pm.DOMAIN_NO_INTERVAL
    # 5. Другой ключ и та же ширина.
    stranger, _ = _fold_sample(0.2525, key=("other",))
    assert pm.build_model(base, [stranger]).outcome == pm.REFUSED_KEY
    assert pm.build_model(base, [base]).outcome == pm.REFUSED_NO_OTHER_SAMPLE
    assert pm.build_model(base, []).outcome == pm.REFUSED_NO_OTHER_SAMPLE


def test_a_vertex_shared_with_an_unmodelled_domain_does_not_move():
    base, _ = _fold_sample(0.25)
    near, _ = _fold_sample(0.2525)
    far, _ = _fold_sample(0.2475)
    switched, _ = _fold_sample(0.2525, wall_token_suffix="-other")
    switched_far, _ = _fold_sample(0.2475, wall_token_suffix="-other")
    model = pm.build_model(base, [switched, switched_far])
    shared = sorted(set(base.domains[0].vertices.tolist()) & set(base.domains[1].vertices.tolist()))
    assert shared, "the fold has welded vertices"
    for index in shared:
        assert model.vlow[index] == np.inf and model.vhigh[index] == -np.inf and (model.a1[index] == 0).all()
    frame = pm.evaluate(model, 0.4)
    assert frame.held == ((1, pm.DOMAIN_STRUCTURE_SWITCH),)
    assert (frame.live_domains, frame.held_domains) == (1, 1)


# --------------------------------------------------------------------------
# 4. База битово
# --------------------------------------------------------------------------


def test_the_frame_at_the_base_width_is_the_base_bitwise():
    base, _ = _fold_sample(0.25)
    near, _ = _fold_sample(0.2525)
    model = pm.build_model(base, [near])
    frame = pm.evaluate(model, base.alpha)
    again = pm.base_frame(base)
    assert frame.outcome == pm.PREVIEW_MESH_FROM_INTERVAL_V1 and frame.live_domains == 2
    assert np.array_equal(frame.positions, again.positions) and np.array_equal(frame.uvs, again.uvs)
    assert frame.positions.dtype == np.float32 and frame.positions.shape == (base.vertex_count * 3,)
    assert np.array_equal(frame.positions, np.asarray(base.positions, dtype=np.float32).reshape(-1))


# --------------------------------------------------------------------------
# 5. Самопроверка и карантин
# --------------------------------------------------------------------------


def _pole(alpha):
    """Дробно-линейный закон из аудита F4: `f(a) = 0.01 / (1.11 - a)`; шесть процентов за базой 1.0 уже втрое круче хорды."""

    return 0.01 / (1.11 - alpha)


def _pole_trio():
    return _pole_sample(1.0), _pole_sample(1.005), _pole_sample(0.995)


def _pole_sample(alpha):
    """Один домен на ЧЕТЫРЁХ вершинах и шести петлях (два треугольника), свободные вершины двигаются по `_pole`."""

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
        )
    ]
    arrays = production_mesh.build_mesh_arrays(results, 0.0)
    return preview.build_sample(results, arrays, ("pole",), alpha)


def _refuted_fold():
    """Неверная модель складки (ошибка в наклоне), точный прогон на 0.3 и итог сверки: оба домена опровергнуты."""

    base, _ = _fold_sample(0.25)
    near, _ = _fold_sample(0.2525)
    far, _ = _fold_sample(0.2475)
    good = pm.build_model(base, [near, far])
    wrong = dataclasses.replace(good, a1=good.a1 + 0.5)  # ошибка в наклоне: превью уходит от точного
    exact, _ = _fold_sample(0.3)
    return base, near, far, good, wrong, exact, pm.deviation(wrong, exact)


def test_a_wrong_model_is_refuted_by_the_next_exact_run_and_its_domains_go_to_quarantine():
    base, near, _far, good, wrong, exact, check = _refuted_fold()
    assert check.refusal == "" and check.domains_checked == 2
    assert check.refuted == (0, 1) and check.max_position > 0.01 and check.worst_ratio > 1.0
    assert check.relative_distance == pytest.approx(0.2)
    ledger, events = pm.advance_ledger(None, wrong, check)
    assert sorted(events) == [(0, "floor", pm.DOMAIN_QUARANTINED), (1, "wall", pm.DOMAIN_QUARANTINED)]
    entry = ledger.get(0, "floor")
    assert (entry.state, entry.refutations, entry.scale, entry.clean_checks) == (pm.TRUST_QUARANTINED, 1, 0.5, 0)
    assert entry.reach_cap == pytest.approx(pm.TRUST_CAP_FRACTION * 0.2) and ledger.counts() == {pm.TRUST_QUARANTINED: 2}
    # Модель нового образца строится под журналом: оба домена построены и придержаны, кадр отказан именем и равен базе.
    rebuilt = pm.build_model(exact, [base, near], ledger)
    assert [row[2] for row in rebuilt.rows] == [pm.DOMAIN_QUARANTINED, pm.DOMAIN_QUARANTINED]
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


def test_a_sample_at_a_fit_width_is_not_an_independent_check():
    base, near, _far, _good, wrong, exact, check = _refuted_fold()
    ledger, _ = pm.advance_ledger(None, wrong, check)
    rebuilt = pm.build_model(exact, [base, near], ledger)
    for fitted in (exact, base, near):
        verdict = pm.deviation(rebuilt, fitted)
        assert verdict.refusal == pm.REFUSED_NOT_INDEPENDENT and verdict.domains_checked == 0 and verdict.refuted == ()
        assert pm.advance_ledger(ledger, rebuilt, verdict) == (ledger, ())


def test_a_quarantined_domain_comes_back_after_a_clean_independent_check_with_a_reduced_reach():
    base, near, _far, _good, wrong, exact, check = _refuted_fold()
    ledger, _ = pm.advance_ledger(None, wrong, check)
    rebuilt = pm.build_model(exact, [base, near], ledger)
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
    # Возврат со сжатой областью: аффинный домен теряет освобождение от досягаемости (10 %) и получает половину её, но не больше капа опровержения.
    again = pm.build_model(independent, [exact, base], readmitted)
    assert [row[2] for row in again.rows] == [pm.DOMAIN_MODELLED, pm.DOMAIN_MODELLED]
    assert [row[4] for row in again.rows] == [pytest.approx(0.5 * pm.CURVED_REACH_RATIO)] * 2
    assert pm.evaluate(again, 0.33).live_domains == 2
    beyond = pm.evaluate(again, 0.36)  # 10.8 % от базы: внутри интервала ядра (0.1, 0.6), за сжатой областью
    assert beyond.refusal == pm.REFUSED_OUTSIDE_EVERY_INTERVAL and beyond.held == ((0, pm.DOMAIN_HELD_REACH), (1, pm.DOMAIN_HELD_REACH))


def _verdict(*, refuted=(), shadow_clean=(), shadow_refuted=(), distance=0.1):
    return pm.DeviationV1(
        0.3, 0.1, 0.0, len(refuted), 0, 0, tuple(refuted), "", tuple(shadow_clean), tuple(shadow_refuted), distance, 20.0
    )


def test_a_domain_that_fails_again_shrinks_further_and_the_third_refutation_abandons_its_model():
    base, _ = _fold_sample(0.25)
    near, _ = _fold_sample(0.2525)
    far, _ = _fold_sample(0.2475)
    model = pm.build_model(base, [near, far])
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
    rebuilt = pm.build_model(base, [near, far], ledger)
    assert rebuilt.rows[0][2] == pm.DOMAIN_ABANDONED and rebuilt.rows[1][2] == pm.DOMAIN_MODELLED
    assert pm.evaluate(rebuilt, 0.3).held == ((0, pm.DOMAIN_ABANDONED),)
    # Журнал другого ключа образца забыт: он говорил про другую геометрию.
    foreign_base, _ = _fold_sample(0.25, key=("other",))
    foreign_near, _ = _fold_sample(0.2525, key=("other",))
    fresh, events = pm.advance_ledger(ledger, pm.build_model(foreign_base, [foreign_near]), _verdict(refuted=(0,)))
    assert fresh.key == ("other",) and fresh.get(0, "floor").refutations == 1 and len(events) == 1


def test_a_fixed_percent_of_reach_is_not_a_guarantee_and_the_audit_example_is_refuted_and_quarantined():
    """Аудит F4: `f(a) = 0.01 / (1.11 - a)`, образцы 1, 1.005, 0.995; на 1.09 (внутри 10 %) точное 0.5, модель говорит ~0.23."""

    base, near, far = _pole_trio()
    model = pm.build_model(base, [near, far])
    assert model.rows[0][2:] == (pm.DOMAIN_MODELLED, 2, pm.CURVED_REACH_RATIO), "the model believes itself up to ten percent"
    assert pm.evaluate(model, 1.09).live_domains == 1, "and moves the mesh there"
    exact = _pole_sample(1.09)
    check = pm.deviation(model, exact)
    limit = pm.DEVIATION_LIMIT_RATIO * 1.09
    assert check.refuted == (0,) and check.max_position > 40.0 * limit, (check.max_position, limit)
    assert check.worst_ratio > 40.0 and check.relative_distance == pytest.approx(0.09)
    ledger, events = pm.advance_ledger(None, model, check)
    assert [item[2] for item in events] == [pm.DOMAIN_QUARANTINED]
    assert ledger.get(0, "pole").reach_cap == pytest.approx(pm.TRUST_CAP_FRACTION * 0.09)
    rebuilt = pm.build_model(exact, [base, near], ledger)
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
    assert sum(arrays.domain_loop_counts) == len(arrays.uvs)
    assert max(max(item) for item in arrays.domain_vertex_index) < len(arrays.positions)
    # Токен — структура без чисел: тот же при другой ширине; он входит в образец, а не в дайджест массивов.
    assert arrays.domain_tokens == other.domain_tokens and arrays.digest != other.digest
    assert [item.token for item in first.domains] == list(arrays.domain_tokens)


# --------------------------------------------------------------------------
# Байты
# --------------------------------------------------------------------------


def test_the_model_reports_its_bytes_and_the_quadratic_costs_more(row):
    base, _, _ = row(0.25)
    near, _, _ = row(0.2505)
    far, _, _ = row(0.2495)
    linear = pm.build_model(base, [near])
    quadratic = pm.build_model(base, [near, far])
    assert linear.a2 is None and linear.e2 is None and quadratic.a2 is not None
    assert 0 < linear.own_bytes < quadratic.own_bytes
    assert quadratic.nbytes == quadratic.own_bytes + base.nbytes
    # Слагаемые честные: сумма `nbytes` массивов, а не оценка.
    arrays = (quadratic.a1, quadratic.a2, quadratic.e1, quadratic.e2, quadratic.vlow, quadratic.vhigh, quadratic.loop_counts, quadratic.dlow, quadratic.dhigh)
    assert quadratic.own_bytes == sum(item.nbytes for item in arrays)
    assert quadratic.a2.dtype == np.float32 and quadratic.a1.dtype == np.float64


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
        samples = (_fold_sample(0.25)[0], _fold_sample(0.2525)[0], _fold_sample(0.2475)[0])
    base, near, far = samples
    model = pm.build_model(base, [near, far])
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

    assert controller.width_model is None and controller.width_displayed is None and controller.width_aux == ()
    assert controller.width_mesh_owner is None and controller.width_mesh_preview is None
    assert _dropped(controller, reason), controller.width_preview_log.dropped
    assert f"none ({preview.MODEL_DROPPED}:{reason})" in preview.status_lines(controller)[0]
    return True


def test_the_mesh_takes_a_frame_only_when_it_is_the_owned_base_of_the_model(monkeypatch):
    world = _owned(monkeypatch)
    controller, base, good, decal = world.controller, world.base, world.mesh, world.decal

    state = preview.preview_mesh_now(controller, 0.27)

    assert state is not None and state.outcome == pm.PREVIEW_MESH_FROM_INTERVAL_V1 and state.live == 2 and state.serial == 1
    assert [name for name, _ in good.vertices.sets] == ["co"] and good.uv_layers[0].data.sets[0][0] == "uv" and good.updated == 1
    assert good.vertices.sets[0][1].dtype == np.float32 and good.vertices.sets[0][1].shape == (base.vertex_count * 3,)
    assert preview.preview_mesh_now(controller, 0.28).serial == 2
    lines = preview.status_lines(controller)
    assert "PREVIEW_MESH_FROM_INTERVAL_V1 preview (approximate, not certified)" in lines[0] and "2 domains moving" in lines[0]
    assert lines[1].startswith("Preview model (approximate, inside the certified event interval): 2/2 domains modelled")

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
    world.controller.width_model = pm.build_model(newer, [world.base])


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
    world = _owned(monkeypatch, samples=_pole_trio(), corners=first)
    assert (world.base.vertex_count, world.base.loop_count) == (4, 6)
    rival = _Mesh(4, 6, world.base.alpha, corners=second)
    rival.update(world.mesh)
    world.decal.data = rival
    world.meshes.append(rival)
    assert preview.preview_mesh_now(world.controller, 1.02) is None and _writes(world) == 0
    assert _is_dropped_clean(world.controller, preview.MESH_DATABLOCK_REPLACED)

    # 2. Тот же датаблок, раскладку изменили на месте (ручная правка диагонали): кадры внутри интервала перепроверки не перечитывают
    #    раскладку (цена кадра прежняя), а сеть через интервал её видит и снимает модель, не записав кадр на чужую раскладку.
    world = _owned(monkeypatch, samples=_pole_trio(), corners=first)
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
    world = _owned(monkeypatch, samples=_pole_trio(), corners=first)
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
    assert not preview.wants_prime(controller), "without a displayed sample nothing is primed: the next exact result starts over"
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


def _live(alpha, displayed, aux, previous, prime, key=("fold",), trust=None):
    results = _fold_results(alpha)
    run = SimpleNamespace(results=results)
    return preview.finish_live_run(
        run, alpha=alpha, offset=OFFSET, key=key, displayed=displayed, aux=aux, previous=previous, prime=prime, trust=trust
    )


def test_the_session_builds_the_model_from_the_button_the_primes_and_the_exact_results():
    controller = EnvelopeDebugSessionController()
    button, _ = _fold_sample(0.25)
    preview.note_button_display(controller, button)
    assert controller.width_displayed is button and controller.width_model is None and preview.wants_prime(controller)

    # Затравка 1: образец вспомогательный, модель прямая. Её ширина — внутри интервалов, рядом с базой, не база.
    first = preview.next_prime_alpha(button, controller.width_aux)
    assert first is not None and first != button.alpha and abs(first / button.alpha - 1.0) <= preview.PRIME_RELATIVE_STEP * 1.0001
    controller.width_prime_attempts += 1
    live = _live(first, button, (), None, True)
    assert preview.note_prime(controller, button, live) == ""
    assert controller.width_model.quadratic_domains == 0 and len(controller.width_aux) == 1
    assert preview.wants_prime(controller), "one other sample gives a chord: a second prime is wanted"

    # Затравка 2: с другой стороны от базы; модель квадратная, больше затравок не нужно.
    second = preview.next_prime_alpha(button, controller.width_aux)
    assert second is not None and (second > button.alpha) != (first > button.alpha)
    controller.width_prime_attempts += 1
    assert preview.note_prime(controller, button, _live(second, button, controller.width_aux, None, True)) == ""
    assert controller.width_model.quadratic_domains == 2 and not preview.wants_prime(controller)
    assert preview.log_of(controller).models == 2 and preview.log_of(controller).refusals == {}

    # Точный результат на новой ширине: образец на экране — он, прежний уходит во вспомогательные, прежняя модель сверена.
    old_model = controller.width_model
    exact_live = _live(0.2575, controller.width_displayed, controller.width_aux, old_model, False)
    assert exact_live.check is not None and exact_live.check.refuted == () and exact_live.check.max_position <= EXACT_TOLERANCE
    preview.note_exact_display(controller, exact_live)
    assert controller.width_displayed is exact_live.sample and controller.width_aux[0] is button
    assert controller.width_model is exact_live.model and controller.width_model.base is exact_live.sample
    assert controller.width_mesh_preview is None and controller.width_prime_attempts == 0
    assert preview.log_of(controller).checks == 1 and preview.log_of(controller).max_position_error <= EXACT_TOLERANCE

    # Затравка, посчитанная для образца, которого на экране уже нет, не принимается.
    stale = _live(0.2578, button, (), None, True)
    assert preview.note_prime(controller, button, stale) == preview.PRIME_BASE_REPLACED
    assert preview.log_of(controller).refusals == {preview.PRIME_BASE_REPLACED: 1}


def test_a_model_that_cannot_be_built_is_named_and_never_fails_the_exact_result(monkeypatch):
    button, _ = _fold_sample(0.25)
    stranger, _ = _fold_sample(0.2525, key=("other",))

    def boom(*_args):
        raise ArithmeticError("synthetic")

    live = _live(0.2575, None, (stranger,), None, False)  # на экране образца нет, а вспомогательный — другого меша
    assert isinstance(live.model, pm.PreviewRefusalV1) and live.model.outcome == pm.REFUSED_KEY
    monkeypatch.setattr(preview, "build_model", boom)
    broken = _live(0.2575, button, (), None, False)
    assert broken.model.outcome == preview.MODEL_BUILD_FAILED and "ArithmeticError" in broken.model.detail
    controller = EnvelopeDebugSessionController()
    preview.note_exact_display(controller, broken)
    assert controller.width_displayed is broken.sample and controller.width_model is None
    assert preview.log_of(controller).refusals == {preview.MODEL_BUILD_FAILED: 1}
    assert "Preview model: none (PREVIEW_MODEL_BUILD_FAILED)" in preview.status_lines(controller)[0]


def test_the_key_names_everything_the_geometry_depends_on():
    record = SimpleNamespace(
        source_name="src", source_digest="d", kernel_backend="PYTHON", density="2", stretch_percent=20,
        dissolve_percent=0.39, invalidation_count=3,
    )
    key = preview.sample_key(record, 0.02)
    for field_name, value in (
        ("source_name", "other"), ("source_digest", "e"), ("kernel_backend", "NATIVE:abc"), ("density", "4"),
        ("stretch_percent", 42), ("dissolve_percent", 0.5), ("invalidation_count", 4),
    ):
        assert preview.sample_key(SimpleNamespace(**{**vars(record), field_name: value}), 0.02) != key, field_name
    assert preview.sample_key(record, 0.03) != key


def test_the_prime_width_stays_inside_the_intervals_and_never_repeats():
    base, _ = _fold_sample(0.25)
    seen = {base.alpha}
    aux = []
    for _ in range(4):
        found = preview.next_prime_alpha(base, tuple(aux))
        if found is None:
            break
        assert found not in seen and 0.0 < found
        assert all(item.interval.contains_exact(Fraction(str(found))) for item in base.domains)
        seen.add(found)
        aux.append(_fold_sample(found)[0])
    assert len(aux) >= 2


def test_the_session_carries_the_refutation_into_the_ledger_the_status_and_the_next_model():
    controller = EnvelopeDebugSessionController()
    button, near = _fold_sample(0.25)[0], _fold_sample(0.2525)[0]
    far = _fold_sample(0.2475)[0]
    preview.note_button_display(controller, button)
    good = pm.build_model(button, [near, far])
    wrong = dataclasses.replace(good, a1=good.a1 + 0.5)
    controller.width_model = wrong

    # Точный результат опровергает прежнюю модель: журнал, события, новая модель с карантином, строки статуса.
    refuting = _live(0.3, button, (near, far), wrong, False, trust=controller.width_trust)
    assert refuting.check.refuted == (0, 1) and [item[2] for item in refuting.trust_events] == [pm.DOMAIN_QUARANTINED] * 2
    assert [row[2] for row in refuting.model.rows] == [pm.DOMAIN_QUARANTINED] * 2
    preview.note_exact_display(controller, refuting)
    log = preview.log_of(controller)
    assert controller.width_trust is refuting.trust and controller.width_trust.counts() == {pm.TRUST_QUARANTINED: 2}
    assert log.trust_events == {pm.DOMAIN_QUARANTINED: 2} and log.domains_refuted == 2 and log.max_deviation_ratio > 1.0
    lines = preview.status_lines(controller)
    assert any(line.startswith("Preview trust: 2 quarantined") for line in lines), lines
    assert any(line.startswith("Last exact check:") and "2 refuted" in line for line in lines), lines
    assert any("PREVIEW_DOMAIN_QUARANTINED_AFTER_REFUTATION 2" in line for line in lines), lines

    # Затравка журнал не двигает (опорная, а не проверочная), но строится под ним.
    primed = _live(0.2512, controller.width_displayed, controller.width_aux, None, True, trust=controller.width_trust)
    assert primed.trust is controller.width_trust and primed.trust_events == () and primed.check is None
    assert primed.model.rows[0][2] == pm.DOMAIN_QUARANTINED

    # Следующее точное обновление: чистая независимая сверка в тени возвращает домены со сжатой областью.
    returning = _live(0.325, controller.width_displayed, controller.width_aux, controller.width_model, False, trust=controller.width_trust)
    assert returning.check.shadow_clean == (0, 1) and returning.check.refuted == ()
    assert [item[2] for item in returning.trust_events] == [pm.DOMAIN_READMITTED] * 2
    assert [row[2] for row in returning.model.rows] == [pm.DOMAIN_MODELLED] * 2
    preview.note_exact_display(controller, returning)
    assert controller.width_trust.counts() == {pm.TRUST_REDUCED: 2}
    assert preview.log_of(controller).trust_events == {pm.DOMAIN_QUARANTINED: 2, pm.DOMAIN_READMITTED: 2}

    # Кнопка — явная пересборка: журнал доверия забыт.
    preview.note_button_display(controller, button)
    assert controller.width_trust is None


def test_the_preview_status_names_the_model_approximate_and_never_a_certificate(monkeypatch):
    world = _owned(monkeypatch)
    controller = world.controller
    preview.preview_mesh_now(controller, 0.27)
    text = "\n".join(preview.status_lines(controller))
    assert "approximate" in text and "not certified" in text
    assert "ertificate" not in text, text  # сертифицирован только интервал событий ядра, а не модель внутри него
    controller.width_model = None
    controller.width_model_refusal = pm.PreviewRefusalV1("PREVIEW_NO_OTHER_EXACT_SAMPLE")
    assert preview.status_lines(controller)[-1] == "Preview model: none (PREVIEW_NO_OTHER_EXACT_SAMPLE) - the line overlay only"
