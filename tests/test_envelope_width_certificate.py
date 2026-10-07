"""Сертификат превью меша ширины (`envelope_width_certificate`, `envelope_width_mesh_preview`): математика и жизнь сессии, без Blender.

Утверждения (каждое названо и проверено числами, а не словом):

1. ВНУТРИ ИНТЕРВАЛА ПРЕВЬЮ РАВНО ТОЧНОМУ. На настоящих доменах (ряд квадратов `quad_row_bundle`, полевой домен) и на синтетической складке из двух
   доменов с общими вершинами позиции меша и UV кадра сертификата на ширине внутри интервала равны точному прогону с отклонением не больше
   `EXACT_TOLERANCE` (1e-9) — по прямой через две точки и по квадрату через три; у кривого домена квадрат точнее прямой.
2. ВНЕ ИНТЕРВАЛА ОТКАЗАНО ИМЕНЕМ. Ширина за интервалом всех доменов — кадр отказан `PREVIEW_ALPHA_OUTSIDE_EVERY_INTERVAL` и равен базе;
   ширина за интервалом одного домена — домен придержан (`PREVIEW_DOMAIN_ALPHA_OUTSIDE_INTERVAL`) на геометрии базы, остальные двигаются.
3. СЕРТИФИКАЦИЯ ДОМЕНА ИМЕЕТ ПРИЧИНЫ. Другой образец за интервалом, смена структуры, пропавший домен, нет интервала, другой ключ, та же ширина —
   у каждого свой исход (`DOMAIN_*`, `REFUSED_*`), и общая вершина двух доменов не двигается, пока не сертифицирован каждый.
4. БАЗА БИТОВО. Кадр на ширине базы — сама база; отмена возвращает меш побитово.
5. САМОПРОВЕРКА. Неверный сертификат ловится сверкой с точным прогоном (`deviation`), домен снимается (`PREVIEW_DOMAIN_REFUTED_BY_EXACT`).
6. МЕШ ПРИНИМАЕТ КАДР ТОЛЬКО СВОЙ. Состав, ширина и ревизия сверяются перед каждым кадром; чужой меш — сертификат снят с названной причиной.
7. ЖИЗНЬ СЕССИИ. Кнопка, точный результат и затравка расставляют образцы так, как описано в модуле; сертификат строит `finish_live_run`.
"""

from __future__ import annotations

import dataclasses
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
from cftuv import envelope_width_certificate as cert  # noqa: E402
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


def _max_error(certificate, sample):
    positions, uvs, _live = cert.positions_and_uvs(certificate, sample.alpha)
    return float(np.max(np.abs(positions - sample.positions))), float(np.max(np.abs(uvs - sample.uvs)))


# --------------------------------------------------------------------------
# 1. Внутри интервала превью равно точному
# --------------------------------------------------------------------------


def test_the_preview_equals_the_exact_run_inside_the_interval_on_real_domains(row):
    base, _, _ = row(0.25)
    near, _, _ = row(0.2505)
    far, _, _ = row(0.2495)
    assert all(item.status == CERTIFIED for item in base.domains)
    linear = cert.build_certificate(base, [near])
    quadratic = cert.build_certificate(base, [near, far])
    assert isinstance(linear, cert.PreviewCertificateV1) and isinstance(quadratic, cert.PreviewCertificateV1)
    assert linear.quadratic_domains == 0 and quadratic.quadratic_domains == len(base.domains)
    # Прямая через две точки верит себе на три процента ширины, квадрат у аффинного домена — на весь интервал ядра.
    for width in (0.2501, 0.2503, 0.2507, 0.249, 0.2574, 0.26, 0.3, 0.5):
        exact, _, _ = row(width)
        for certificate in (linear, quadratic):
            if certificate is linear and abs(width / base.alpha - 1.0) > cert.CHORD_REACH_RATIO:
                assert cert.evaluate(linear, width).refusal == cert.REFUSED_OUTSIDE_EVERY_INTERVAL, width
                continue
            position_error, uv_error = _max_error(certificate, exact)
            assert position_error <= EXACT_TOLERANCE and uv_error <= EXACT_TOLERANCE, (width, position_error, uv_error)
            check = cert.deviation(certificate, exact)
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
    linear = cert.build_certificate(base, [near])
    quadratic = cert.build_certificate(base, [near, far])
    exact = sample(0.41)  # внутри досягаемости обеих моделей
    chord = _max_error(linear, exact)[0]
    parabola = _max_error(quadratic, exact)[0]
    assert chord > 1e-5, "the fixture must be curved enough to tell the two apart"
    assert parabola < chord / 4.0, (chord, parabola)
    # Кривизна значима, и модель это знает: досягаемость десять процентов, а не весь интервал ядра; прямая — три.
    assert quadratic.rows[0][3:] == (2, cert.CURVED_REACH_RATIO) and linear.rows[0][3:] == (1, cert.CHORD_REACH_RATIO)
    assert cert.evaluate(quadratic, 0.43).live_domains == 1  # 7.5 % от базы
    beyond = cert.evaluate(quadratic, 0.46)  # 15 % от базы: внутри интервала ядра `(0.05, 2.0)`, за досягаемостью модели
    assert beyond.refusal == cert.REFUSED_OUTSIDE_EVERY_INTERVAL and beyond.held == ((0, cert.DOMAIN_HELD_REACH),)
    assert cert.evaluate(linear, 0.43).held == ((0, cert.DOMAIN_HELD_REACH),)  # прямая за три процента не верит себе


def test_a_fold_of_two_domains_with_shared_vertices_is_followed_exactly():
    base, _ = _fold_sample(0.25)
    near, _ = _fold_sample(0.2525)
    far, _ = _fold_sample(0.2475)
    certificate = cert.build_certificate(base, [near, far])
    assert isinstance(certificate, cert.PreviewCertificateV1) and certificate.certified_domains == 2
    # Аффинные домены: кривизна в пределах шума нулевая, досягаемость модели не ограничивает ничего, кроме интервала ядра.
    assert [row[3:] for row in certificate.rows] == [(2, None), (2, None)] and not certificate.a2.any() and not certificate.e2.any()
    for width in (0.26, 0.31, 0.45):
        exact, _ = _fold_sample(width)
        assert max(_max_error(certificate, exact)) <= EXACT_TOLERANCE, width
    # Общие вершины `a`, `b` (сварка по ссылке источника) стоят: их позиции не зависят от ширины.
    frame_positions, _uvs, _live = cert.positions_and_uvs(certificate, 0.4)
    shared = [index for index in range(base.vertex_count) if (certificate.a1[index] == 0).all()]
    assert len(shared) >= 2
    assert np.array_equal(frame_positions[shared], base.positions[shared])


# --------------------------------------------------------------------------
# 2. Вне интервала отказано именем
# --------------------------------------------------------------------------


def test_a_width_beyond_every_interval_is_refused_by_name_and_equals_the_base():
    base, _ = _fold_sample(0.25)
    near, _ = _fold_sample(0.2525)
    certificate = cert.build_certificate(base, [near])

    frame = cert.evaluate(certificate, 0.9)  # интервал `(0.1, 0.6)`

    assert frame.refusal == cert.REFUSED_OUTSIDE_EVERY_INTERVAL == frame.outcome
    assert frame.live_domains == 0 and frame.held_domains == 2
    assert {reason for _patch, reason in frame.held} == {cert.DOMAIN_HELD_OUTSIDE}
    assert np.array_equal(frame.positions, np.asarray(base.positions, dtype=np.float32).reshape(-1))
    assert np.array_equal(frame.uvs, np.asarray(base.uvs, dtype=np.float32).reshape(-1))
    for bad in (0.0, -1.0, float("nan"), float("inf")):
        assert cert.evaluate(certificate, bad).refusal == cert.REFUSED_ALPHA
    assert cert.evaluate(None, 0.3).refusal == cert.REFUSED_NO_CERTIFICATE


def test_a_domain_whose_interval_is_left_is_held_at_the_base_and_the_others_move():
    base, _ = _fold_sample(0.25)
    near, _ = _fold_sample(0.2525)
    far, _ = _fold_sample(0.2475)
    # Интервал стены кончается на 0.3: на 0.4 пол двигается, стена придержана и названа.
    narrow = dataclasses.replace(base.domains[1], high=0.3)
    narrowed = dataclasses.replace(base, domains=(base.domains[0], narrow))
    certificate = cert.build_certificate(narrowed, [near, far])
    frame = cert.evaluate(certificate, 0.4)
    assert (frame.live_domains, frame.held_domains) == (1, 1) and frame.outcome == cert.PREVIEW_MESH_FROM_INTERVAL_V1
    assert frame.held == ((1, cert.DOMAIN_HELD_OUTSIDE),)
    moved = frame.positions.reshape(-1, 3)
    wall_free = [i for i in narrow.vertices if (certificate.a1[i] == 0).all()]
    floor_free = [i for i in base.domains[0].vertices if (certificate.vlow[i] < 0.4 < certificate.vhigh[i])]
    assert floor_free and not np.array_equal(moved[floor_free], base.positions[floor_free].astype(np.float32))
    assert np.array_equal(moved[wall_free], base.positions[wall_free].astype(np.float32))


# --------------------------------------------------------------------------
# 3. Сертификация домена имеет причины
# --------------------------------------------------------------------------


def test_every_reason_a_domain_is_not_certified_is_named():
    base, _ = _fold_sample(0.25)
    near, _ = _fold_sample(0.2525)

    # 1. Образец за интервалом базы.
    outside, _ = _fold_sample(0.7)
    refusal = cert.build_certificate(base, [outside])
    assert isinstance(refusal, cert.PreviewRefusalV1) and refusal.outcome == cert.REFUSED_NO_CERTIFIED_DOMAIN
    assert cert.DOMAIN_OUTSIDE in refusal.detail
    # 2. Структура домена другая (подпись ядра либо токен локальных массивов).
    switched, _ = _fold_sample(0.2525, wall_token_suffix="-other")
    certificate = cert.build_certificate(base, [switched])
    assert certificate.rows[1][2] == cert.DOMAIN_STRUCTURE_SWITCH and certificate.rows[0][2] == cert.DOMAIN_CERTIFIED
    retokened = dataclasses.replace(near, domains=(near.domains[0], dataclasses.replace(near.domains[1], token="x")))
    assert cert.build_certificate(base, [retokened]).rows[1][2] == cert.DOMAIN_STRUCTURE_SWITCH
    # 3. Домена нет в другом образце.
    missing = dataclasses.replace(near, domains=near.domains[:1])
    assert cert.build_certificate(base, [missing]).rows[1][2] == cert.DOMAIN_MISSING
    # 4. У домена нет заверенного интервала (`NOT_CERTIFIED`).
    uncertain, _ = _fold_sample(0.25, wall_interval=_interval(0.25, 0.25, 0.25, NOT_CERTIFIED))
    assert cert.build_certificate(uncertain, [near]).rows[1][2] == cert.DOMAIN_NO_INTERVAL
    # 5. Другой ключ и та же ширина.
    stranger, _ = _fold_sample(0.2525, key=("other",))
    assert cert.build_certificate(base, [stranger]).outcome == cert.REFUSED_KEY
    assert cert.build_certificate(base, [base]).outcome == cert.REFUSED_NO_OTHER_SAMPLE
    assert cert.build_certificate(base, []).outcome == cert.REFUSED_NO_OTHER_SAMPLE


def test_a_vertex_shared_with_an_uncertified_domain_does_not_move():
    base, _ = _fold_sample(0.25)
    near, _ = _fold_sample(0.2525)
    far, _ = _fold_sample(0.2475)
    switched, _ = _fold_sample(0.2525, wall_token_suffix="-other")
    switched_far, _ = _fold_sample(0.2475, wall_token_suffix="-other")
    certificate = cert.build_certificate(base, [switched, switched_far])
    shared = sorted(set(base.domains[0].vertices.tolist()) & set(base.domains[1].vertices.tolist()))
    assert shared, "the fold has welded vertices"
    for index in shared:
        assert certificate.vlow[index] == np.inf and certificate.vhigh[index] == -np.inf and (certificate.a1[index] == 0).all()
    frame = cert.evaluate(certificate, 0.4)
    assert frame.held == ((1, cert.DOMAIN_STRUCTURE_SWITCH),)
    assert (frame.live_domains, frame.held_domains) == (1, 1)


# --------------------------------------------------------------------------
# 4. База битово
# --------------------------------------------------------------------------


def test_the_frame_at_the_base_width_is_the_base_bitwise():
    base, _ = _fold_sample(0.25)
    near, _ = _fold_sample(0.2525)
    certificate = cert.build_certificate(base, [near])
    frame = cert.evaluate(certificate, base.alpha)
    again = cert.base_frame(base)
    assert frame.outcome == cert.PREVIEW_MESH_FROM_INTERVAL_V1 and frame.live_domains == 2
    assert np.array_equal(frame.positions, again.positions) and np.array_equal(frame.uvs, again.uvs)
    assert frame.positions.dtype == np.float32 and frame.positions.shape == (base.vertex_count * 3,)
    assert np.array_equal(frame.positions, np.asarray(base.positions, dtype=np.float32).reshape(-1))


# --------------------------------------------------------------------------
# 5. Самопроверка
# --------------------------------------------------------------------------


def test_a_wrong_certificate_is_caught_by_the_next_exact_run_and_the_domain_is_removed():
    base, _ = _fold_sample(0.25)
    near, _ = _fold_sample(0.2525)
    far, _ = _fold_sample(0.2475)
    good = cert.build_certificate(base, [near, far])
    wrong = dataclasses.replace(good, a1=good.a1 + 0.5)  # ошибка в наклоне: превью уходит от точного
    exact, _ = _fold_sample(0.3)
    check = cert.deviation(wrong, exact)
    assert check.refusal == "" and check.domains_checked == 2
    assert check.refuted == (0, 1) and check.max_position > 0.01
    reduced = wrong.without_domains(check.refuted)
    assert [row[2] for row in reduced.rows] == [cert.DOMAIN_REFUTED, cert.DOMAIN_REFUTED]
    frame = cert.evaluate(reduced, 0.3)
    assert frame.refusal == cert.REFUSED_OUTSIDE_EVERY_INTERVAL
    assert np.array_equal(frame.positions, np.asarray(base.positions, dtype=np.float32).reshape(-1))
    assert cert.deviation(good, exact).refuted == ()
    stranger, _ = _fold_sample(0.3, key=("other",))
    assert cert.deviation(good, stranger).refusal == cert.REFUSED_KEY
    assert cert.deviation(None, exact).refusal == cert.REFUSED_NO_CERTIFICATE


# --------------------------------------------------------------------------
# Массивы меша: состав домена
# --------------------------------------------------------------------------


def test_the_mesh_arrays_carry_the_domain_layout_the_certificate_reads(row):
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


def test_the_certificate_reports_its_bytes_and_the_quadratic_costs_more(row):
    base, _, _ = row(0.25)
    near, _, _ = row(0.2505)
    far, _, _ = row(0.2495)
    linear = cert.build_certificate(base, [near])
    quadratic = cert.build_certificate(base, [near, far])
    assert linear.a2 is None and linear.e2 is None and quadratic.a2 is not None
    assert 0 < linear.own_bytes < quadratic.own_bytes
    assert quadratic.nbytes == quadratic.own_bytes + base.nbytes
    # Слагаемые честные: сумма `nbytes` массивов, а не оценка.
    arrays = (quadratic.a1, quadratic.a2, quadratic.e1, quadratic.e2, quadratic.vlow, quadratic.vhigh, quadratic.loop_counts, quadratic.dlow, quadratic.dhigh)
    assert quadratic.own_bytes == sum(item.nbytes for item in arrays)
    assert quadratic.a2.dtype == np.float32 and quadratic.a1.dtype == np.float64


# --------------------------------------------------------------------------
# 6. Меш принимает только свой кадр
# --------------------------------------------------------------------------


class _Collection(list):
    def __init__(self, *items):
        super().__init__(items)
        self.sets = []

    def foreach_set(self, name, values):
        self.sets.append((name, np.array(values, copy=True)))

    def get(self, name):
        return next((item for item in self if getattr(item, "name", None) == name), None)


class _Mesh(dict):
    def __init__(self, vertices, loops, width, name="CFTUV_Decal"):
        super().__init__({production_mesh.DECAL_WIDTH_PROPERTY: width})
        self.name = name
        self.vertices = _Collection(*range(vertices))
        layer = SimpleNamespace(name=production_mesh.DECAL_UV_LAYER, data=_Collection(*range(loops)))
        self.uv_layers = _Collection(layer)
        self.updated = 0

    def update(self):
        self.updated += 1


class _Decal(dict):
    def __init__(self, mesh, revision, mode="OBJECT"):
        super().__init__({production_mesh.DECAL_REVISION_PROPERTY: revision})
        self.data = mesh
        self.mode = mode


def _controller_with(base, certificate, decal_for, monkeypatch):
    controller = EnvelopeDebugSessionController()
    controller.width_build = SimpleNamespace(source_name="src")
    controller.width_displayed, controller.width_certificate = base, certificate
    monkeypatch.setitem(
        sys.modules,
        "bpy",
        types.SimpleNamespace(data=types.SimpleNamespace(objects={"src": SimpleNamespace(name="src")})),
    )
    monkeypatch.setattr(production_mesh, "find_decal_object", lambda source: decal_for)
    return controller


def test_the_mesh_takes_a_frame_only_when_it_is_the_base_of_the_certificate(monkeypatch):
    base, _ = _fold_sample(0.25)
    near, _ = _fold_sample(0.2525)
    far, _ = _fold_sample(0.2475)
    certificate = cert.build_certificate(base, [near, far])
    good = _Mesh(base.vertex_count, base.loop_count, base.alpha)
    decal = _Decal(good, base.source_revision)
    controller = _controller_with(base, certificate, decal, monkeypatch)

    state = preview.preview_mesh_now(controller, 0.27)

    assert state is not None and state.outcome == cert.PREVIEW_MESH_FROM_INTERVAL_V1 and state.live == 2 and state.serial == 1
    assert [name for name, _ in good.vertices.sets] == ["co"] and good.uv_layers[0].data.sets[0][0] == "uv" and good.updated == 1
    assert good.vertices.sets[0][1].dtype == np.float32 and good.vertices.sets[0][1].shape == (base.vertex_count * 3,)
    assert preview.preview_mesh_now(controller, 0.28).serial == 2
    lines = preview.status_lines(controller)
    assert "PREVIEW_MESH_FROM_INTERVAL_V1 preview, not certified" in lines[0] and "2 domains moving" in lines[0]
    assert lines[1].startswith("Preview certificate: 2/2 domains")

    # Отмена: меш назад на базу, побитово то, что записал точный путь.
    assert preview.restore_base_mesh(controller) is True and controller.width_mesh_preview is None
    assert np.array_equal(good.vertices.sets[-1][1], np.asarray(base.positions, dtype=np.float32).reshape(-1))
    assert preview.restore_base_mesh(controller) is False  # меш уже на базе

    # Edit-режим декали: кадры молчат, сертификат цел.
    decal.mode = "EDIT"
    writes = len(good.vertices.sets)
    assert preview.preview_mesh_now(controller, 0.27) is None and len(good.vertices.sets) == writes
    assert controller.width_certificate is certificate
    decal.mode = "OBJECT"

    # Каждая причина, по которой меш не база, снимает сертификат и образцы с названием.
    bad = {
        "other vertex count": (_Mesh(base.vertex_count + 1, base.loop_count, base.alpha), base.source_revision),
        "other width": (_Mesh(base.vertex_count, base.loop_count, base.alpha + 0.1), base.source_revision),
        "other revision": (_Mesh(base.vertex_count, base.loop_count, base.alpha), "another"),
    }
    for label, (mesh, revision) in bad.items():
        controller = _controller_with(base, certificate, _Decal(mesh, revision), monkeypatch)
        assert preview.preview_mesh_now(controller, 0.27) is None, label
        assert controller.width_certificate is None and controller.width_displayed is None, label
        assert controller.width_preview_log.dropped == {f"PREVIEW_CERTIFICATE_DROPPED:{preview.MESH_NOT_THE_BASE}": 1}, label
        assert "none (PREVIEW_CERTIFICATE_DROPPED:PREVIEW_MESH_NOT_THE_BASE)" in preview.status_lines(controller)[0]
    gone = _controller_with(base, certificate, None, monkeypatch)
    assert preview.preview_mesh_now(gone, 0.27) is None and gone.width_certificate is None
    assert preview.preview_mesh_now(EnvelopeDebugSessionController(), 0.27) is None  # без сертификата кадра нет и ничего не названо лишнего


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


def _live(alpha, displayed, aux, previous, prime, key=("fold",)):
    results = _fold_results(alpha)
    run = SimpleNamespace(results=results)
    return preview.finish_live_run(run, alpha=alpha, offset=OFFSET, key=key, displayed=displayed, aux=aux, previous=previous, prime=prime)


def test_the_session_builds_the_certificate_from_the_button_the_primes_and_the_exact_results():
    controller = EnvelopeDebugSessionController()
    button, _ = _fold_sample(0.25)
    preview.note_button_display(controller, button)
    assert controller.width_displayed is button and controller.width_certificate is None and preview.wants_prime(controller)

    # Затравка 1: образец вспомогательный, сертификат прямой. Её ширина — внутри интервалов, рядом с базой, не база.
    first = preview.next_prime_alpha(button, controller.width_aux)
    assert first is not None and first != button.alpha and abs(first / button.alpha - 1.0) <= preview.PRIME_RELATIVE_STEP * 1.0001
    controller.width_prime_attempts += 1
    live = _live(first, button, (), None, True)
    assert preview.note_prime(controller, button, live) == ""
    assert controller.width_certificate.quadratic_domains == 0 and len(controller.width_aux) == 1
    assert preview.wants_prime(controller), "one other sample gives a chord: a second prime is wanted"

    # Затравка 2: с другой стороны от базы; сертификат квадратный, больше затравок не нужно.
    second = preview.next_prime_alpha(button, controller.width_aux)
    assert second is not None and (second > button.alpha) != (first > button.alpha)
    controller.width_prime_attempts += 1
    assert preview.note_prime(controller, button, _live(second, button, controller.width_aux, None, True)) == ""
    assert controller.width_certificate.quadratic_domains == 2 and not preview.wants_prime(controller)
    assert preview.log_of(controller).certificates == 2 and preview.log_of(controller).refusals == {}

    # Точный результат на новой ширине: образец на экране — он, прежний уходит во вспомогательные, прежний сертификат сверен.
    old_certificate = controller.width_certificate
    exact_live = _live(0.2575, controller.width_displayed, controller.width_aux, old_certificate, False)
    assert exact_live.check is not None and exact_live.check.refuted == () and exact_live.check.max_position <= EXACT_TOLERANCE
    preview.note_exact_display(controller, exact_live)
    assert controller.width_displayed is exact_live.sample and controller.width_aux[0] is button
    assert controller.width_certificate is exact_live.certificate and controller.width_certificate.base is exact_live.sample
    assert controller.width_mesh_preview is None and controller.width_prime_attempts == 0
    assert preview.log_of(controller).checks == 1 and preview.log_of(controller).max_position_error <= EXACT_TOLERANCE

    # Затравка, посчитанная для образца, которого на экране уже нет, не принимается.
    stale = _live(0.2578, button, (), None, True)
    assert preview.note_prime(controller, button, stale) == preview.PRIME_BASE_REPLACED
    assert preview.log_of(controller).refusals == {preview.PRIME_BASE_REPLACED: 1}


def test_a_certificate_that_cannot_be_built_is_named_and_never_fails_the_exact_result(monkeypatch):
    button, _ = _fold_sample(0.25)
    stranger, _ = _fold_sample(0.2525, key=("other",))

    def boom(*_args):
        raise ArithmeticError("synthetic")

    live = _live(0.2575, None, (stranger,), None, False)  # на экране образца нет, а вспомогательный — другого меша
    assert isinstance(live.certificate, cert.PreviewRefusalV1) and live.certificate.outcome == cert.REFUSED_KEY
    monkeypatch.setattr(preview, "build_certificate", boom)
    broken = _live(0.2575, button, (), None, False)
    assert broken.certificate.outcome == preview.CERTIFICATE_BUILD_FAILED and "ArithmeticError" in broken.certificate.detail
    controller = EnvelopeDebugSessionController()
    preview.note_exact_display(controller, broken)
    assert controller.width_displayed is broken.sample and controller.width_certificate is None
    assert preview.log_of(controller).refusals == {preview.CERTIFICATE_BUILD_FAILED: 1}
    assert "none (PREVIEW_CERTIFICATE_BUILD_FAILED)" in preview.status_lines(controller)[0]


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
