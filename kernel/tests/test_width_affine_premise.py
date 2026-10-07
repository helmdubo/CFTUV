"""Посылка превью меша на живой ширине: что внутри заверенного интервала аффинно по ширине, а что нет, и чем это тогда описывается.

Превью (`cftuv/envelope_width_certificate.py`) двигает настоящий меш декали между точными пересчётами: по двум либо трём точным
материализациям домена внутри ОДНОГО заверенного интервала (`materialize.interval`) строится многочлен положения вершины и произведения
`u * alpha`, `v * alpha`. Это законно, только если форма зависимости известна, а не «похожа». Здесь она проверена на самом ядре, без хоста,
на ширинах, которых в построении нет:

1. ПОЗИЦИИ И `u * alpha` АФФИННЫ у доменов-развёрток и полевых доменов без резки: прямая через две точки равна точному на третьей ширине с
   отклонением не больше `AFFINE_TOLERANCE` (метры и единицы UV). Закон `UV_DIRECT_STRIP_V1` — `(u, v) = (s, r) / alpha`, станции `(s, r)`
   аффинны, поэтому аффинно произведение, а сам `u` — нет (его превью делит на ширину).
2. ВЕРШИНА НА РЕБРЕ ИСТОЧНИКА ПОД РЕЗКОЙ АФФИННОЙ НЕ БЫВАЕТ (измерено на `wall_noise_top_rung_clip_v1`, законы резки по треугольникам): она лежит на
   пересечении поворачивающейся прямой с неподвижным ребром, её параметр — дробно-линейная функция ширины, и в интервале без событий она
   отходит от хорды на миллиметры при шаге в десятки процентов. Посылка «в интервале всё аффинно» для таких доменов НЕВЕРНА, и превью её не
   принимает на веру: оно берёт квадрат через три точки (ошибка третьего порядка), а запись `rows` называет степень и досягаемость модели
   каждого домена (`CURVED_REACH_RATIO`: десять процентов ширины базы). Здесь проверено, что на этой досягаемости квадрат точнее хорды в разы и
   укладывается в `CURVED_QUADRATIC_TOLERANCE`; на всём интервале ядра отклонение печатается (сантиметры у крайних ширин), а не прячется.
3. СТРУКТУРА В ИНТЕРВАЛЕ ТА ЖЕ, ГДЕ ЭТО ЗАВЕРЕНО. Полевые домены под законами хоста (силуэт, закон по граням) переключают структуру ВНУТРИ интервала
   (решения тесселяции, силуэта и положения вершин в интервал не входят): такие тройки называются в счёте и не судятся; в превью им соответствует
   исход `PREVIEW_DOMAIN_STRUCTURE_SWITCH` (проверен на стороне хоста).
"""

from __future__ import annotations

import pytest

import developable_factories as df
import materialize_factories as factories
from developable_route import developable_domain
from materialize_factories import prepare_and_cover

from cftuv_envelope.contracts.geometry_batch import DecalTopologyLawV1
from cftuv_envelope.contracts.metric import NearPlanarLiftLawV1
from cftuv_envelope.materialize.admit import materialization_request
from cftuv_envelope.materialize.domain import materialize_domain
from cftuv_envelope.materialize.interval import CERTIFIED
from cftuv_envelope.wavefront import conveyor_coverage, prepare_conveyor

ROUTE = ("r0a", "r0b")
BY_TRIANGLES = NearPlanarLiftLawV1.SOURCE_TRIANGLES_CLIPPED_V1
BY_FACES = NearPlanarLiftLawV1.SOURCE_FACES_CLIPPED_V1
POLYGONS = DecalTopologyLawV1.PLANAR_POLYGONS_V1
SILHOUETTE = DecalTopologyLawV1.SILHOUETTE_TOPOLOGY_V1

#: Допуск положения вершины батча (метры) и `u * alpha` (единицы UV) для аффинных доменов: округление `binary64` суммы корней и разность ширин.
AFFINE_TOLERANCE = 1e-9
#: Допуск квадрата через три точки у кривого домена, метры, на ширинах до `CURVED_REACH` от базы (измерено: порядка 1e-4).
CURVED_QUADRATIC_TOLERANCE = 1e-3
#: Досягаемость модели у кривого домена: доля ширины базы (`envelope_width_certificate.CURVED_REACH_RATIO`).
CURVED_REACH = 0.10
#: Доля ширины базы, на которую две опорные ширины отходят от неё (как затравка хоста, `PRIME_RELATIVE_STEP`).
SUPPORT_STEP = 0.005


def developable(make, alpha="1"):
    snapshot, request = developable_domain(make(), ROUTE, alpha=alpha)
    prepared, _coverage = prepare_and_cover(snapshot, request)
    return prepared


def field(name):
    snapshot, request = factories.load_fixture(name)
    prepared = prepare_conveyor(snapshot, request)
    assert prepared.outcome.value == "EXACT", prepared.detail
    return prepared


def materialize(prepared, alpha, lift=BY_TRIANGLES, topology=POLYGONS):
    coverage = conveyor_coverage(prepared, str(alpha))
    assert coverage.outcome.value == "EXACT", coverage.detail
    return materialize_domain(
        prepared,
        coverage,
        request=materialization_request(prepared, uv_policy_id="UV_DIRECT_STRIP_V1"),
        near_planar_lift_law=lift,
        decal_topology_law=topology,
        certify=True,
        digests=False,
    )


#: `(имя, постройка, ширины alpha0, закон подъёма, закон топологии, аффинен ли домен)`.
CASES = (
    ("fold", lambda: developable(df.fold_strip), ("0.3", "1.3"), BY_TRIANGLES, POLYGONS, True),
    ("slant", lambda: developable(df.slant_fold), ("0.7", "2.0"), BY_TRIANGLES, POLYGONS, True),
    ("quarter", lambda: developable(df.quarter_cylinder), ("0.7", "2.0"), BY_TRIANGLES, POLYGONS, True),
    ("weighted_normals", lambda: field("building_002_weighted_normals_v1"), (None,), BY_FACES, SILHOUETTE, True),
    ("point_contact", lambda: field("building_002_point_contact_v1"), (None,), BY_FACES, SILHOUETTE, True),
    ("cut_fans", lambda: field("mesh2_patch0_cut_fans_v1"), (None,), BY_FACES, SILHOUETTE, True),
    ("noise_top", lambda: field("wall_noise_top_rung_clip_v1"), ("0.25", "0.85"), BY_TRIANGLES, POLYGONS, False),
)


def positions(batch) -> dict:
    return {item.vert_key.value: (item.position.x, item.position.y, item.position.z) for item in batch.vertices}


def scaled_corner_uvs(batch, alpha) -> dict:
    """`{(кольцо грани с наименьшего ключа, ключ вершины): (u * alpha, v * alpha)}`: петля определена ключами, а не номерами граней."""

    found = {}
    for face in batch.faces:
        keys = [key.value for key in face.ordered_vert_keys]
        start = keys.index(min(keys))
        ring = tuple(keys[start:] + keys[:start])
        for key, fact in zip(keys, face.uv_facts):
            found[(ring, key)] = (fact.uv.u * alpha, fact.uv.v * alpha)
    return found


def inside(record):
    """Ширины строго внутри интервала, не совпадающие с `alpha0`: у краёв и в середине каждой половины."""

    high = record.high if record.high is not None else record.alpha * 3
    found = []
    for fraction in (0.05, 0.5, 0.95):
        found.append(record.alpha + (record.low - record.alpha) * fraction)
        found.append(record.alpha + (high - record.alpha) * fraction)
    return [value for value in found if record.low < value < high and value > 0 and value != record.alpha]


def line(first, second, alpha_first, alpha_second, alpha):
    t = (alpha - alpha_first) / (alpha_second - alpha_first)
    return [a + t * (b - a) for a, b in zip(first, second)]


def parabola(base, near, far, alpha0, alpha1, alpha2, alpha):
    """Многочлен второй степени через три точки в форме Ньютона: ровно то, что строит `envelope_width_certificate._newton`."""

    slopes = [(b - a) / (alpha1 - alpha0) for a, b in zip(base, near)]
    curves = [(((c - a) / (alpha2 - alpha0)) - s) / (alpha2 - alpha1) for a, c, s in zip(base, far, slopes)]
    dt = alpha - alpha0
    return [a + dt * (s + k * (alpha - alpha1)) for a, s, k in zip(base, slopes, curves)]


@pytest.mark.parametrize("name,build,alphas,lift,topology,affine", CASES, ids=[item[0] for item in CASES])
def test_positions_and_scaled_uv_inside_the_certified_interval(name, build, alphas, lift, topology, affine):
    prepared = build()
    judged = switched = 0
    worst = {"line_position": 0.0, "line_uv": 0.0, "parabola_position": 0.0, "parabola_uv": 0.0}
    reach = {"line_position": 0.0, "parabola_position": 0.0, "parabola_uv": 0.0}  # то же, но только внутри досягаемости модели
    for alpha0 in alphas:
        base = materialize(prepared, alpha0 if alpha0 is not None else prepared.requested_alpha.value, lift, topology)
        assert base.is_materialized and base.interval.status == CERTIFIED, (name, base.detail)
        record = base.interval
        origin = float(record.alpha)
        near_width, far_width = origin * (1.0 + SUPPORT_STEP), origin * (1.0 - SUPPORT_STEP)
        if not (record.low < far_width and (record.high is None or near_width < record.high)):
            continue  # интервал уже опорных ширин: домен не получил бы квадрата (хост назвал бы это сертификацией домена прямой либо отказом)
        near = materialize(prepared, repr(near_width), lift, topology)
        far = materialize(prepared, repr(far_width), lift, topology)
        near_reach = [
            origin * (1.0 + sign * fraction)
            for fraction in (0.02, 0.05, 0.09)
            for sign in (1.0, -1.0)
            if record.low < origin * (1.0 + sign * fraction) and (record.high is None or origin * (1.0 + sign * fraction) < record.high)
        ]
        for width in [*near_reach, *inside(record)[:6]]:
            other = materialize(prepared, repr(width), lift, topology)
            digests = {item.structure.digest for item in (base, near, far, other)}
            if len(digests) != 1:
                switched += 1
                continue
            judged += 1
            within = abs(width - origin) <= CURVED_REACH * origin
            keys = [positions(item.batch) for item in (base, near, far, other)]
            assert set(keys[0]) == set(keys[1]) == set(keys[2]) == set(keys[3]), name
            for key in keys[3]:
                chord = line(keys[0][key], keys[1][key], origin, near_width, width)
                curve = parabola(keys[0][key], keys[1][key], keys[2][key], origin, near_width, far_width, width)
                worst["line_position"] = max(worst["line_position"], max(abs(a - b) for a, b in zip(chord, keys[3][key])))
                worst["parabola_position"] = max(worst["parabola_position"], max(abs(a - b) for a, b in zip(curve, keys[3][key])))
                if within:
                    reach["line_position"] = max(reach["line_position"], max(abs(a - b) for a, b in zip(chord, keys[3][key])))
                    reach["parabola_position"] = max(reach["parabola_position"], max(abs(a - b) for a, b in zip(curve, keys[3][key])))
            corners = [scaled_corner_uvs(item.batch, w) for item, w in ((base, origin), (near, near_width), (far, far_width), (other, width))]
            assert set(corners[0]) == set(corners[3]), name
            for corner in corners[3]:
                chord = line(corners[0][corner], corners[1][corner], origin, near_width, width)
                curve = parabola(corners[0][corner], corners[1][corner], corners[2][corner], origin, near_width, far_width, width)
                worst["line_uv"] = max(worst["line_uv"], max(abs(a - b) for a, b in zip(chord, corners[3][corner])))
                worst["parabola_uv"] = max(worst["parabola_uv"], max(abs(a - b) for a, b in zip(curve, corners[3][corner])))
                if within:
                    reach["parabola_uv"] = max(reach["parabola_uv"], max(abs(a - b) for a, b in zip(curve, corners[3][corner])))
    print(
        f"AFFINE_PREMISE {name}: judged {judged} widths, structure switched in {switched}; "
        + ", ".join(f"{label} {value:.3e}" for label, value in worst.items())
        + " | within the model reach: "
        + ", ".join(f"{label} {value:.3e}" for label, value in reach.items())
    )
    assert judged >= 1, f"{name}: no width inside the interval kept its structure; the premise was not exercised"
    if affine:
        assert worst["line_position"] <= AFFINE_TOLERANCE and worst["line_uv"] <= AFFINE_TOLERANCE, (name, worst)
    else:
        # Домен не аффинен, и это названо: хорда ошибается заметно, квадрат на досягаемости модели — в допуске и точнее хорды в разы,
        # а на всём интервале ядра он тоже лучше хорды (но уже на сантиметры: поэтому у модели своя досягаемость).
        assert worst["line_position"] > 10 * AFFINE_TOLERANCE, (name, worst)
        assert reach["parabola_position"] <= CURVED_QUADRATIC_TOLERANCE, (name, reach)
        assert reach["parabola_position"] < reach["line_position"] / 4, (name, reach)
        assert worst["parabola_position"] < worst["line_position"]
