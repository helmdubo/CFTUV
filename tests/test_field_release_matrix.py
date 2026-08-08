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
    # walls.001, домен-склон: полевой профиль владельца
    # (`artifacts/field_snapshots/walls_001_door_queue_profile.txt`) несёт тот
    # же исход с тем же max_residual_squared=1.178496e+01.
    (WALLS_001, 1): "HOST_EXPORT_REJECTED:NEAR_PLANAR_RESIDUAL_BUDGET_EXCEEDED",
    # building, патч 89: тот же near-planar отказ на обеих вершинах.
    (BUILDING, 89): "HOST_EXPORT_REJECTED:NEAR_PLANAR_RESIDUAL_BUDGET_EXCEEDED",
}

_CACHE: dict[str, dict] = {}


def route(snapshot: str) -> dict:
    """Полный маршрут слепка в отдельном процессе под капом работы."""

    if snapshot in _CACHE:
        return _CACHE[snapshot]
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
    try:
        finished = subprocess.run(
            command,
            capture_output=True,
            text=True,
            timeout=cap,
            cwd=str(ROOT),
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
    _CACHE[snapshot] = result
    return result


def domain(snapshot: str, patch_id: int) -> dict:
    for record in route(snapshot)["domains"]:
        if record["patch_id"] == patch_id:
            return record
    raise AssertionError(
        f"DOMAIN_ABSENT: у {snapshot} нет домена патча {patch_id}; "
        f"есть {[r['patch_id'] for r in route(snapshot)['domains']]}"
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

    record = domain(WALLS_012, 0)
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
    "snapshot,patch_id",
    [
        (WALL_2_001, 0),
        (WALLS_001, 0),
        (BUILDING, 17),
        (BUILDING, 91),
        (BUILDING, 109),
        (BUILDING, 121),
    ],
)
def test_anchor_loci_survive_with_their_participants(snapshot, patch_id):
    """Каждый якорный локус на месте, и его `participants` не изменились.

    Якорный локус — тот, который ОБЕ математики выдают в побитово одинаковых
    точных `(t, точка)`. Число узлов при этом НЕ проверяется: ремонт вправе
    добавить или снять локусы, но не вправе сдвинуть согласованные.
    """

    table = anchors(snapshot, patch_id)
    present = {
        locus_key(locus): locus for locus in domain(snapshot, patch_id)["loci"]
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
    present = {locus_key(locus) for locus in domain(WALLS_012, 0)["loci"]}
    for anchor in table["anchors"]:
        key = json.dumps([anchor["time"], anchor["point"]], sort_keys=True)
        assert key in present, f"ANCHOR_LOCUS_DISAPPEARED: {anchor['point']}"


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


def test_walls_001_slope_domain_refuses_by_the_field_name():
    """Домен-склон: тот же ИМЕНОВАННЫЙ near-planar отказ, что в поле.

    Именованный отказ — не пропуск: он обязан прийти по имени и на той же
    стадии. Тихое исчезновение домена было бы дефектом, а не «ну он же не
    считается».
    """

    record = domain(WALLS_001, 1)
    assert record["domain_id"].endswith("eb64fc8b4eaaabe6c70159ff")
    assert record["stage"] == "HOST_EXPORT"
    assert record["outcome"] == APPROVED_NAMED_REFUSALS[(WALLS_001, 1)]
    assert "max_residual_squared=1.178496e+01" in record["detail"]
    assert "PRODUCT_SKIRT_ABSOLUTE_V1" in record["detail"]


# ---------------------------------------------------------------------------
# 6. building — каждый выбранный домен либо EXACT, либо одобренный отказ.
# ---------------------------------------------------------------------------


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
