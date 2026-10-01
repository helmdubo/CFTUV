"""ПОЛЕВАЯ РЕЛИЗ-МАТРИЦА: четыре слепка владельца тем же маршрутом, что Blender.

Карточка FIELD-GATE-FREEZE. Полевой отказ владельца заморожен ВОРОТАМИ ДО
любых правок: правка без замороженного свидетельства — подгонка, а не ремонт.

ДВА ВОРОТА ЗДЕСЬ — БЫВШИЕ КРАСНЫЕ, ТЕПЕРЬ ОБЯЗАНЫ БЫТЬ ЗЕЛЁНЫМИ.
Они были заморожены КРАСНЫМИ как знамя ремонта (полевой отказ владельца на
вершине 7e3c5fa) и стали зелёными от ремонта закона места рождения порта
(слияние ede9916, запись в DECISIONS от 2026-08-07):

* `test_wall_2_001_faces_and_coverage_are_exact` — был: скелет EXACT на
  49 узлах, граней 0, `FACE_CHAIN_DOES_NOT_CLOSE`.
* `test_walls_012_is_exact` — был: скелет обрывается на 2 узлах вместо 12,
  `SUPERLEVEL_COMPONENT_UNRESOLVABLE`.

xfail и skip им ЗАПРЕЩЕНЫ НАВСЕГДА, и запрет исполняется тестом
`test_red_gates_are_not_suppressed`: если ворота снова покраснеют, xfail
превратил бы полевой отказ в зелёную строку — ровно в то, ради отмены чего
эта карточка заведена. Красное снимается ремонтом ядра, не маркером.

ЧТО ЗАМОРОЖЕНО, А ЧТО НЕТ. Заморожены: coverage (исход покрытия), ownership
(спек-владелец на КАЖДОМ ребре-источнике), continuation (каждый нестационарный
фронт обязан замкнуть свою цепочку граней — это и есть `FaceOutcome.EXACT`),
ТОЧНЫЕ ЛОКУСЫ и семантические `participants`.

НЕ заморожен ИСТОРИЧЕСКИЙ СЧЁТЧИК УЗЛОВ. Ни 45, ни 49 не объявлены властью:
какое из двух чисел верно, ещё не доказано, и заморозить любое значило бы
решить открытый вопрос тестом. Вместо счётчика заморожены ЯКОРНЫЕ ЛОКУСЫ —
подмножество, которое ОБЕ математики (рабочая вершина 6ce0227 и отвергнутая
полем 1dbf712) выдают в ПОБИТОВО одних и тех же точных `(t, точка)`. Локус,
который дают обе, не есть историческая случайность ни одной из них. Их 37 из
45/49 на стене 2.001, и на всех 37 множества `participants` СОВПАДАЮТ
побитово — измерено, а не предположено (`artifacts/field_gate_freeze/`).

ПОЧЕМУ ПОДПРОЦЕСС, А НЕ ВЫЗОВ В ТОМ ЖЕ ПРОЦЕССЕ. Две причины, обе
неустранимые. (1) КАП РАБОТЫ: полевой сигнал владельца про `building` —
«висит», а ворота, которые ждут бесконечно, не ворота; внешний предел
превращает зависание в ИМЕНОВАННЫЙ отказ `DOMAIN_WORK_CAP_EXCEEDED`.
Wall-clock назван честно: детерминированный бюджет работы в единицах работы —
отдельная карточка, и подменять её секундомером эти ворота не вправе; секунды
сторожат ворота, а не выносят суждение о математике. (2) ИЗОЛЯЦИЯ: маршрут
поднимает `artifacts/perf_prepare_diag/env.py`, который ДОПОЛНЯЕТ заглушку
`mathutils.Vector` (унарный минус, равенство, хэш) на весь процесс. В общем
процессе хостовой сюиты это была бы невидимая правка окружения соседних
тестов.
"""

from __future__ import annotations

import json
import os
from pathlib import Path
import subprocess
import sys

import pytest


ROOT = Path(__file__).resolve().parents[1]
ROUTE = ROOT / "artifacts" / "field_gate_freeze" / "field_route.py"
ANCHORS = ROOT / "artifacts" / "field_gate_freeze" / "anchor_loci.json"

RED = "FIELD-GATE-FREEZE КРАСНЫЕ ВОРОТА"

WALL_2_001 = "wall_2_001_snapshot.json"
WALLS_012 = "walls_012_snapshot.json"
WALLS_001 = "walls_001_u_route_snapshot.json"
BUILDING = "building_full_snapshot.json"

# Полевые ручки владельца (вершина 7e3c5fa, Fan Density 0, alpha 0.254).
FIELD_ALPHA = 0.254
FIELD_DENSITY = 0
# `building` снят на дефолте харнесса: alpha этого меша владельцем не
# сообщалась, а измеренный вердикт от alpha не зависит (вход очереди
# alpha-независим — это уже записанная находка).
BUILDING_ALPHA = 0.45

# Домены `building`, входящие в ворота: три самых тяжёлых по непланарности из
# несущих выбранные рёбра (89/109/121 — те же, что мерил перф-диагност) плюс
# 91 (максимальная непланарность 2.7e-5 м) и 17 (одиннадцать граней, три
# петли — самый крупный многопетлевой домен с выбранными рёбрами). «Все 122
# домена» намеренно НЕ гоняются: ворота обязаны завершаться.
BUILDING_PATCHES = (17, 89, 91, 109, 121)

# Кап работы на слепок, секунды. Числа взяты как ~3x над измеренным на этой
# вершине временем (2.001 — 16.6 с, building — 22.5 с), чтобы кап ловил
# ЗАВИСАНИЕ, а не медленную машину.
WORK_CAP_SECONDS = {
    WALL_2_001: 180.0,
    WALLS_012: 120.0,
    WALLS_001: 120.0,
    BUILDING: 300.0,
}

# Заранее одобренные ИМЕНОВАННЫЕ отказы. Всё, чего здесь нет, — дефект.
APPROVED_NAMED_REFUSALS = {
    # building, патч 89. До NEAR_PLANAR V2 — `NEAR_PLANAR_RESIDUAL_BUDGET_EXCEEDED`
    # (невязка 1.89 см против 1.25 см); под укладкой на поверхность отказывала ширина
    # (один треугольник из 12, 5.43 м вдоль, перпендикулярен плоскости карты, `min cos²`
    # = 1.891e-06). С лестницей S1 (DEVELOPABLE) домен пробует развёртку и отказывает
    # ТОЧНЕЕ: развёртка по растяжению в бюджете (ступенька трёх плоскостей, 12
    # треугольников, внутренних вершин нет), но вершина `building:34` — граничная с
    # веером 360.167° (> 2π): границы карты у неё перекрываются на 0.167°, и карта —
    # не вложение. Прежнее имя этого отказа держит закрепка лестницы
    # (`test_building_patch_89_still_refuses_by_width_on_the_near_planar_rung`).
    (BUILDING, 89): "HOST_EXPORT_REJECTED:DEVELOPABLE_CHART_SELF_OVERLAP",
}

_CACHE: dict[str, dict] = {}


PIN_LIFT_FLAG = "--pin-lift"
PIN_FRAME_FLAG = "--pin-frame"
PIN_LADDER_FLAG = "--pin-ladder"
LEGACY_LIFT = "CERTIFIED_PLANE_V1"
LEGACY_FRAME = "CANONICAL_ONLY_V1"
NEAR_PLANAR_ONLY = "NEAR_PLANAR_ONLY_V1"


def route(
    snapshot: str,
    pin_lift: str | None = None,
    pin_frame: str | None = None,
    pin_ladder: str | None = None,
) -> dict:
    """Полный маршрут слепка в отдельном процессе под капом работы.

    `pin_lift` и `pin_frame` — ИМЕНОВАННЫЕ закрепки закона укладки и политики репера
    хоста (только для ворот математики фронта, см. `test_walls_012_is_exact` и
    таблицу якорей): по умолчанию маршрут идёт настоящими законами хоста, закрепка
    едет в `substitutions` ответа.
    """

    key = f"{snapshot}|{pin_lift}|{pin_frame}|{pin_ladder}"
    if key in _CACHE:
        return _CACHE[key]
    if snapshot == BUILDING:
        alpha, patches = BUILDING_ALPHA, ",".join(
            str(value) for value in BUILDING_PATCHES
        )
    else:
        alpha, patches = FIELD_ALPHA, "-"
    command = [
        sys.executable,
        str(ROUTE),
        snapshot,
        str(alpha),
        str(FIELD_DENSITY),
        patches,
    ]
    cap = WORK_CAP_SECONDS[snapshot]
    environment = dict(os.environ)
    if pin_lift is not None:
        command += [PIN_LIFT_FLAG, pin_lift]
    if pin_frame is not None:
        command += [PIN_FRAME_FLAG, pin_frame]
    if pin_ladder is not None:
        command += [PIN_LADDER_FLAG, pin_ladder]
    try:
        finished = subprocess.run(
            command,
            capture_output=True,
            text=True,
            timeout=cap,
            cwd=str(ROOT),
            env=environment,
        )
    except subprocess.TimeoutExpired:
        pytest.fail(
            f"DOMAIN_WORK_CAP_EXCEEDED: {snapshot} не завершился за {cap} с. "
            "Это ЗАВИСАНИЕ, а не медленный тест: кап поставлен втрое выше "
            "измеренного времени этого же слепка. Поднимать кап, чтобы "
            "ворота позеленели, запрещено — незавершаемость есть отказ."
        )
    if finished.returncode != 0:
        pytest.fail(
            f"ROUTE_DID_NOT_FINISH: {snapshot} вернул {finished.returncode}.\n"
            f"{finished.stderr[-4000:]}"
        )
    result = json.loads(finished.stdout)
    _CACHE[key] = result
    return result


def domain(
    snapshot: str,
    patch_id: int,
    pin_lift: str | None = None,
    pin_frame: str | None = None,
    pin_ladder: str | None = None,
) -> dict:
    for record in route(snapshot, pin_lift, pin_frame, pin_ladder)["domains"]:
        if record["patch_id"] == patch_id:
            return record
    raise AssertionError(
        f"DOMAIN_ABSENT: у {snapshot} нет домена патча {patch_id}; "
        f"есть {[r['patch_id'] for r in route(snapshot, pin_lift, pin_frame, pin_ladder)['domains']]}"
    )


def anchors(snapshot: str, patch_id: int) -> dict:
    table = json.loads(ANCHORS.read_text(encoding="utf-8"))
    for record in table[snapshot]:
        if record["patch_id"] == patch_id:
            return record
    raise AssertionError(f"ANCHOR_TABLE_MISSING: {snapshot} патч {patch_id}")


def locus_key(locus: dict) -> str:
    return json.dumps([locus["time"], locus["point"]], sort_keys=True)


# ---------------------------------------------------------------------------
# 1. Тождество полевого датума. Ворота, которые обязаны держаться и ДО, и
#    ПОСЛЕ ремонта: если поехали они, измеряется уже не полевой случай.
# ---------------------------------------------------------------------------


def test_wall_2_001_route_reproduces_the_field_datum():
    """Тот же домен, та же решётка, те же законы/веера/ничьи, что в поле."""

    record = domain(WALL_2_001, 0)
    counters = record["counters"]
    assert record["domain_id"] == "host-v0:patch-domain:90ce55618b55029b728c31e7"
    assert record["region_id"] == (
        "sparse-patch-domain-region:2c9f74554f43ff5023f8b8a1"
    )
    assert counters["CONVEYOR_LATTICE_SCALE"] == 32768
    assert counters["CONVEYOR_ARRIVAL_LAWS"] == 35
    assert counters["CONVEYOR_SOURCE_EDGES"] == 35
    assert counters["CONVEYOR_RATIONAL_VERTEX_FANS"] == 12
    assert counters["CONVEYOR_FAN_SUPPORTS"] == 12
    assert counters["CONVEYOR_AMBIGUOUS_OWNER_EDGES"] == 23
    assert record["bridge_outcome"] == "EXACT"


def test_wall_2_001_ownership_accounts_for_every_front():
    """Владение: каждый фронт либо владеемый, либо ничья, либо стена.

    Тихого третьего состояния быть не должно: фронт без владельца и без записи
    в ничьих — это потерянное ребро, а не «просто не покрашено». Ключ веера
    отличается от ключа ребра длиной (пять чисел против четырёх), поэтому две
    половины владения считаются раздельно.
    """

    record = domain(WALL_2_001, 0)
    counters = record["counters"]
    owned_source = {
        tuple(key) for key, _ in record["owner_by_edge"] if len(key) == 4
    }
    owned_fan = {
        tuple(key) for key, _ in record["owner_by_edge"] if len(key) == 5
    }
    ambiguous = {tuple(key) for key in record["ambiguous_owner_spans"]}
    assert all(name for _, name in record["owner_by_edge"])
    # 12 владеемых + 23 ничьи = 35 рёбер-источников; веера все 12 владеемы.
    assert len(owned_source) == 12
    assert len(ambiguous) == 23
    assert not (owned_source & ambiguous)
    assert (
        len(owned_source) + len(ambiguous)
        == counters["CONVEYOR_SOURCE_EDGES"]
        == 35
    )
    assert len(owned_fan) == counters["CONVEYOR_FAN_EDGES"] == 12
    assert record["wall_spans"] == []
    assert counters["CONVEYOR_WALL_EDGES"] == 0
    assert {name.split(":")[0] for _, name in record["owner_by_edge"]} == {
        "strip-spec",
        "angular-spec",
    }


# ---------------------------------------------------------------------------
# 2. БЫВШЕЕ КРАСНОЕ ВОРОТО №1 — стена 2.001; теперь обязано быть зелёным.
# ---------------------------------------------------------------------------


def test_wall_2_001_faces_and_coverage_are_exact():
    """Стена 2.001 обязана собрать грани и покрытие. Было красным, снято
    ремонтом закона места рождения (ede9916); история отказа — ниже.

    Полевой прогон владельца (вершина 7e3c5fa, Fan Density 0, alpha 0.254):
    скелет EXACT, `CONVEYOR_SKELETON_NODES` 49, `CONVEYOR_FACES` 0,
    `preparation_outcome` FACES_DID_NOT_ASSEMBLE, регион
    `sparse-patch-domain-region:2c9f74554f43ff5023f8b8a1` ->
    `FaceOutcome.FACE_CHAIN_DOES_NOT_CLOSE`. Воспроизведено здесь без Blender
    ПОЛНОСТЬЮ, включая номер домена и все счётчики.

    Конкретика отказа лежит рядом
    (`artifacts/field_gate_freeze/face_assembly_evidence.json`): из 47 фронтов
    35 собираются, 12 нет, четырьмя симметричными тройками — по одной на
    каждый оконный вырез. Ведущий отказ: ребро (-9213, 8558) -> (-52583, 32768),
    участник (-65536, 32768, -22166, 8558) сидит в ТРЁХ точках из пяти, все три
    в одно точное время.
    """

    record = domain(WALL_2_001, 0)
    assert record["skeleton_outcome"] == "EXACT"
    assert record["face_outcome"] == "EXACT", (
        f"{RED} / WALL_2_001_FACES_DID_NOT_ASSEMBLE: "
        f"{record['outcome']} — {record['detail']}\n"
        f"грань не сложилась: {record['face_detail']}\n"
        "Свидетельство отказа: artifacts/field_gate_freeze/"
        "face_assembly_evidence.json (собранные и упавшие owners, place-граф, "
        "степени, heads/tails, планы транзакции в тех же локусах)."
    )
    assert record["outcome"] == "EXACT"
    assert record["coverage_outcome"] == "EXACT"
    assert record["counters"]["CONVEYOR_FACES"] > 0


# ---------------------------------------------------------------------------
# 3. БЫВШЕЕ КРАСНОЕ ВОРОТО №2 — walls.012; теперь обязано быть зелёным.
# ---------------------------------------------------------------------------


def test_walls_012_is_exact():
    """walls.012 обязан строиться целиком. Было красным, снято тем же
    ремонтом закона места рождения (ede9916), что и стена 2.001.

    Обрыв (история) короче и потому нагляднее, чем у 2.001: распространение
    останавливается на ЧЕТВЁРТОМ уровне, скелет отдаёт 2 узла вместо 12,
    исход `SUPERLEVEL_COMPONENT_UNRESOLVABLE`, причина
    `SYMBOLIC_INTERIOR_SPLIT_CONTACT_METADATA_CONFLICT`. Пакет этого уровня
    ПОБИТОВО тот же, что на рабочей вершине: четыре SPLIT в одно точное время
    в четырёх точных точках. Расходится не арифметика событий, а их обработка
    (`artifacts/field_gate_freeze/walls_012_break.json`).
    """

    # Закрепка закона укладки: ворота проверяют математику фронта на полевой
    # геометрии, а настоящий закон хоста отказывает этот домен по ширине
    # (`test_walls_012_patch_0_refuses_under_the_surface_law`).
    record = domain(WALLS_012, 0, LEGACY_LIFT)
    assert record["skeleton_outcome"] == "EXACT", (
        f"{RED} / WALLS_012_SKELETON_DID_NOT_CLOSE: "
        f"{record['outcome']} — {record['detail']}\n"
        f"узлов скелета {record['skeleton_nodes']}\n"
        "Свидетельство обрыва: artifacts/field_gate_freeze/walls_012_break.json"
    )
    assert record["face_outcome"] == "EXACT"
    assert record["outcome"] == "EXACT"
    assert record["coverage_outcome"] == "EXACT"
    assert record["counters"]["CONVEYOR_FACES"] > 0


# ---------------------------------------------------------------------------
# 4. Якорные локусы и семантические participants — вместо счётчика узлов.
# ---------------------------------------------------------------------------


@pytest.mark.parametrize(
    "snapshot,patch_id,pin_frame",
    [
        (WALL_2_001, 0, None),
        (WALLS_001, 0, None),
        (BUILDING, 17, None),
        (BUILDING, 91, None),
        # Near-planar домены: якоря записаны в канонических координатах карты, а
        # приведённый базис (NEAR_PLANAR V2, коммит 4) пишет ту же плоскость в других
        # `(u, v)`. Закрепка репера — именованная и едет в `substitutions` маршрута.
        (BUILDING, 109, LEGACY_FRAME),
        (BUILDING, 121, LEGACY_FRAME),
    ],
)
def test_anchor_loci_survive_with_their_participants(snapshot, patch_id, pin_frame):
    """Каждый якорный локус на месте, и его `participants` не изменились.

    Якорный локус — тот, который ОБЕ математики выдают в побитово одинаковых
    точных `(t, точка)`. Число узлов при этом НЕ проверяется: ремонт вправе
    добавить или снять локусы, но не вправе сдвинуть согласованные.
    """

    table = anchors(snapshot, patch_id)
    if pin_frame is not None:
        # Закрепка не немая: прогон с ней несёт её имя в ответе маршрута.
        assert (
            f"HOST_NEAR_PLANAR_FRAME_POLICY_PINNED:{pin_frame}"
            in route(snapshot, None, pin_frame)["substitutions"]
        )
    present = {
        locus_key(locus): locus
        for locus in domain(snapshot, patch_id, None, pin_frame)["loci"]
    }
    missing = []
    drifted = []
    for anchor in table["anchors"]:
        key = json.dumps([anchor["time"], anchor["point"]], sort_keys=True)
        found = present.get(key)
        if found is None:
            missing.append(anchor["point"])
        elif found["participants"] != anchor["participants"]:
            drifted.append((anchor["point"], anchor["participants"],
                            found["participants"]))
    assert not missing, (
        f"ANCHOR_LOCUS_DISAPPEARED: {len(missing)} из {len(table['anchors'])} "
        f"якорных локусов {snapshot} п{patch_id} исчезли."
    )
    assert not drifted, (
        f"ANCHOR_PARTICIPANTS_DRIFTED: у {len(drifted)} якорных локусов "
        f"{snapshot} п{patch_id} поехал состав участников: {drifted[:2]}"
    )


def test_walls_012_anchor_loci_survive():
    """Отдельно от параметризации: у walls.012 якорей всего два, и они живы.

    Ворота слабые НАМЕРЕННО и об этом сказано вслух: на сломанной вершине
    распространение обрывалось ДО остальных десяти локусов, поэтому в якоря
    они не попали — согласия двух математик по ним нет. Сильные ворота
    walls.012 — `test_walls_012_is_exact` (бывшее красное, теперь зелёное).
    """

    table = anchors(WALLS_012, 0)
    assert table["anchor_loci"] == 2
    present = {
        locus_key(locus)
        for locus in domain(WALLS_012, 0, LEGACY_LIFT, LEGACY_FRAME)["loci"]
    }
    for anchor in table["anchors"]:
        key = json.dumps([anchor["time"], anchor["point"]], sort_keys=True)
        assert key in present, f"ANCHOR_LOCUS_DISAPPEARED: {anchor['point']}"


def test_walls_012_patch_0_refuses_under_the_surface_law():
    """Ступень near-planar (NEAR_PLANAR V2) называет этот домен отказом по ширине.

    Тест держит ЭТУ ступень лестницы, закрепив лестницу хоста
    (`NEAR_PLANAR_ONLY_V1`): с настоящей лестницей S1 тот же домен уходит на развёртку
    (`test_walls_012_patch_0_is_unfolded_and_the_straight_chain_law_names_the_refusal`).

    Домен строился: невязка 1.0 см лежала внутри абсолютного бюджета юбки
    1.25 см. Но патч несёт щель в 1 см глубины, и один его треугольник из 10
    (8.09 м вдоль, остальные плоские: `cos²` 0.9999998) наклонён на ~26.6° к
    плоскости карты: `min cos²` = 0.80036 против порога 2500/2601. Так
    записано, а не замолчано: ворота математики идут с названной закрепкой
    закона (`test_walls_012_is_exact`), а этот тест держит настоящий ответ.
    """

    record = domain(WALLS_012, 0, None, None, NEAR_PLANAR_ONLY)
    assert record["stage"] == "HOST_EXPORT"
    assert record["outcome"] == (
        "HOST_EXPORT_REJECTED:NEAR_PLANAR_WIDTH_DISTORTION_BUDGET_EXCEEDED"
    )
    assert "min_cos_squared=8.003640990e-01" in record["detail"]
    assert "threshold=9.611687812e-01" in record["detail"]
    assert route(WALLS_012, LEGACY_LIFT)["substitutions"] == [
        f"HOST_NEAR_PLANAR_LIFT_POLICY_PINNED:{LEGACY_LIFT}"
    ]


def test_walls_012_patch_0_is_unfolded_and_the_straight_chain_law_names_the_refusal():
    """С лестницей хоста (S1) домен проходит метрику развёрткой и отказывает на ОЧЕРЕДИ.

    Щель в 1 см глубины развёртывается (растяжение в бюджете), но выбранная цепь —
    прямая в 3D и пересекает складку щели под углом: в развёртке она ЛОМАНАЯ, а
    закон «объявленная прямая цепь линейна в карте» (`SOURCE_DECLARED_STRAIGHT_CHAIN_
    IS_NOT_LINEAR`) её отвергает именованно. Это не потеря домена и не тихое
    исчезновение: ступень и имя названы.
    """

    record = domain(WALLS_012, 0)
    assert record["stage"] == "QUEUE"
    assert record["outcome"] == "PLAN_IS_NOT_COMPILED"
    assert record["detail"] == "SOURCE_DECLARED_STRAIGHT_CHAIN_IS_NOT_LINEAR"


# ---------------------------------------------------------------------------
# 5. walls.001 — дверь строится, склон отказывает ИМЕНЕМ. Оба — полевой факт.
# ---------------------------------------------------------------------------


def test_walls_001_door_domain_builds():
    """Домен-дверь: EXACT, 12 узлов, 9 граней — как в полевом профиле.

    ПРО СЧЁТЧИК УЗЛОВ ЗДЕСЬ. Запрет морозить исторический счёт узлов
    относится к спорной паре 45/49 у стены 2.001, где неизвестно, какое число
    верно. Здесь ситуация другая и она проверена: 12 — это, во-первых, число
    из ПОЛЕВОГО ПРОФИЛЯ САМОГО ВЛАДЕЛЬЦА
    (`artifacts/field_snapshots/walls_001_door_queue_profile.txt`,
    `CONVEYOR_SKELETON_NODES 12`), во-вторых, число, на котором ОБЕ вершины
    сошлись побитово (12 локусов, 0 расхождений). Это свидетельство, а не
    объявление власти.
    """

    record = domain(WALLS_001, 0)
    assert record["domain_id"].endswith("120901db80b70b6927f754e6")
    assert record["outcome"] == "EXACT"
    assert record["coverage_outcome"] == "EXACT"
    assert record["face_outcome"] == "EXACT"
    assert record["counters"]["CONVEYOR_FACES"] == 9
    assert record["faces"] == 9
    assert record["counters"]["CONVEYOR_SKELETON_NODES"] == 12
    assert record["counters"]["CONVEYOR_LATTICE_SCALE"] == 16384


def test_walls_001_slope_domain_is_unfolded_and_exact():
    """Домен-склон: развёртка (S1) строит его, очередь EXACT.

    Раньше склон отказывал на метрике по ширине: треугольник, перпендикулярный
    плоскости карты (`min cos²` = 0), проекция схлопывала. Склон — разворачиваемая
    поверхность, и с лестницей хоста он проходит метрику, а очередь считает его
    покрытие точно.
    """

    record = domain(WALLS_001, 1)
    assert record["domain_id"].endswith("eb64fc8b4eaaabe6c70159ff")
    assert record["stage"] == "QUEUE"
    assert record["outcome"] == "EXACT"
    assert record["coverage_outcome"] == "EXACT"
    assert record["face_outcome"] == "EXACT"


def test_walls_001_slope_domain_refuses_by_the_field_name_on_the_near_planar_rung():
    """Ступень near-planar (лестница закреплена): прежний ИМЕНОВАННЫЙ отказ по ширине.

    Именованный отказ — не пропуск: он обязан прийти по имени и на той же
    стадии. Тихое исчезновение домена было бы дефектом, а не «ну он же не
    считается».
    """

    record = domain(WALLS_001, 1, None, None, NEAR_PLANAR_ONLY)
    assert record["domain_id"].endswith("eb64fc8b4eaaabe6c70159ff")
    assert record["stage"] == "HOST_EXPORT"
    assert record["outcome"] == (
        "HOST_EXPORT_REJECTED:NEAR_PLANAR_WIDTH_DISTORTION_BUDGET_EXCEEDED"
    )
    # Числа отказа несёт деталь: у склона есть треугольник, перпендикулярный
    # плоскости карты (проекция схлопывает его в отрезок).
    assert "min_cos_squared=0.000000000e+00" in record["detail"]
    assert "INTRINSIC_WIDTH_RELATIVE_V1" in record["detail"]


# ---------------------------------------------------------------------------
# 6. building — каждый выбранный домен либо EXACT, либо одобренный отказ.
# ---------------------------------------------------------------------------


def test_building_patch_89_still_refuses_by_width_on_the_near_planar_rung():
    """Прежнее имя патча 89 — на закреплённой ступени near-planar, с прежними числами."""

    record = domain(BUILDING, 89, None, None, NEAR_PLANAR_ONLY)
    assert record["outcome"] == (
        "HOST_EXPORT_REJECTED:NEAR_PLANAR_WIDTH_DISTORTION_BUDGET_EXCEEDED"
    )
    assert "min_cos_squared=1.891285054e-06" in record["detail"]


def test_building_patch_89_refuses_on_the_unfolding_with_its_numbers():
    """Ступенька развёртывается, но вершина с веером 360.167° перекрывает границу карты."""

    record = domain(BUILDING, 89)
    assert record["stage"] == "HOST_EXPORT"
    assert "boundary edge pairs of the chart meet or overlap" in record["detail"]
    assert "[after near-planar NEAR_PLANAR_WIDTH_DISTORTION_BUDGET_EXCEEDED" in record["detail"]


@pytest.mark.parametrize("patch_id", BUILDING_PATCHES)
def test_building_domain_is_exact_or_an_approved_named_refusal(patch_id):
    record = domain(BUILDING, patch_id)
    approved = APPROVED_NAMED_REFUSALS.get((BUILDING, patch_id))
    if approved is not None:
        assert record["outcome"] == approved, (
            f"BUILDING_REFUSAL_CHANGED_NAME: патч {patch_id} отказал "
            f"{record['outcome']!r} вместо одобренного {approved!r}"
        )
        return
    assert record["outcome"] == "EXACT", (
        f"BUILDING_DOMAIN_REGRESSED: патч {patch_id}: {record['outcome']} — "
        f"{record['detail']}. Новый отказ вносится в "
        "APPROVED_NAMED_REFUSALS решением владельца, а не молча."
    )
    assert record["coverage_outcome"] == "EXACT"
    assert record["face_outcome"] == "EXACT"
    assert record["counters"]["CONVEYOR_FACES"] > 0


def test_building_route_declares_its_blender_substitution():
    """Шаг, которого без Blender НЕ СУЩЕСТВУЕТ, назван в ответе маршрута.

    Классификация OUTER/HOLE у многопетлевых патчей идёт в продакшне через
    временный UV-unwrap внутри Blender. Харнесс подставляет другой модульный
    путь того же файла — и это обязано быть ВИДНО рядом с числами, которые
    подмена сделала возможными, иначе безблендерный прогон выдавал бы себя за
    полный. Три остальных слепка подмен не требуют, и это проверяется тоже:
    пустой список здесь — утверждение, а не умолчание.
    """

    assert route(BUILDING)["substitutions"] == [
        "HOST_MULTI_LOOP_UV_CLASSIFICATION_SUBSTITUTED_BY_NESTING"
    ]
    for name in (WALL_2_001, WALLS_012, WALLS_001):
        assert route(name)["substitutions"] == [], (
            f"UNDECLARED_SUBSTITUTION: {name} прошёл маршрут с подменой "
            f"{route(name)['substitutions']}, а полевой прогон владельца — нет."
        )


def test_building_route_finishes_within_the_work_cap():
    """Ни одного зависания: маршрут вернулся, значит кап не сработал.

    Ворото существует отдельно от исходов доменов потому, что «висит» —
    ОТДЕЛЬНАЯ полевая жалоба владельца, и её нельзя доказать тем, что
    какой-то домен оказался EXACT.
    """

    result = route(BUILDING)
    assert len(result["domains"]) == len(BUILDING_PATCHES)
    assert all(
        record["stage"] in ("QUEUE", "HOST_EXPORT")
        for record in result["domains"]
    )


# ---------------------------------------------------------------------------
# 7. Запрет прятать красное.
# ---------------------------------------------------------------------------


RED_GATES = (
    "test_wall_2_001_faces_and_coverage_are_exact",
    "test_walls_012_is_exact",
)


def test_red_gates_are_not_suppressed():
    """Красные ворота нельзя ни пометить xfail, ни пропустить.

    Правило исполняемое, а не прозаическое: `xfail` на полевом отказе делает
    сюиту зелёной при живом дефекте, и именно так дефект перестают видеть.
    """

    source = Path(__file__).read_text(encoding="utf-8").splitlines()
    for name in RED_GATES:
        index = next(
            i for i, line in enumerate(source) if line.startswith(f"def {name}(")
        )
        head = "\n".join(source[max(0, index - 12):index])
        for banned in ("xfail", "skipif", "pytest.mark.skip"):
            assert banned not in head, (
                f"RED_GATE_SUPPRESSED: {name} помечен {banned}. Полевой отказ "
                "снимается ремонтом ядра, а не маркером."
            )
        # Проверка фальсифицируема: строка `def name(` обязана существовать.
        assert source[index].startswith(f"def {name}(")


def test_snapshot_inputs_are_the_frozen_field_bytes():
    """Слепки не подменены: SHA256 входа — часть расписки поставки."""

    receipt = json.loads(
        (ROOT / "artifacts" / "field_gate_freeze" / "RECEIPT.json").read_text(
            encoding="utf-8"
        )
    )
    frozen = {row["snapshot"]: row["sha256"] for row in receipt["snapshots"]}
    for name in (WALL_2_001, WALLS_012, WALLS_001, BUILDING):
        assert route(name)["sha256"] == frozen[name], (
            f"SNAPSHOT_BYTES_CHANGED: {name} больше не тот вход, на котором "
            "снят полевой отказ; расписка недействительна."
        )
