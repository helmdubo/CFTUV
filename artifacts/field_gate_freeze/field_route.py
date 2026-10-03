"""Полный безблендерный маршрут полевого слепка — тот же, которым идёт Blender.

decode -> request policy -> source snap -> metric -> angular profile ->
conveyor preparation -> skeleton -> faces -> coverage.

ЧТО ЗДЕСЬ ПЕРЕИСПОЛЬЗОВАНО, А НЕ НАПИСАНО ЗАНОВО. Всё, кроме двух вещей ниже,
берётся из существующих харнессов: `artifacts/perf_prepare_diag/env.py`
(пути + заглушки mathutils/bmesh/bpy), `snapshot_bmesh.py` (BMesh-срез над
слепком владельца), `run_domain.py` (`bundle_from_field_snapshot`,
`timed_queue`, `report`), `big_scene.py` (именованная подмена UV-классификации
многопетлевых патчей для building). Ядро зовётся своими публичными дверями
`prepare_conveyor`/`conveyor_coverage` через хостовый `run_queue_domain` —
ровно как в поле.

ДВЕ ВЕЩИ, КОТОРЫХ В ХАРНЕССАХ НЕ БЫЛО, и почему они здесь.

1. ПО-ДОМЕННЫЙ ЗАХВАТ ИМЕНОВАННОГО ОТКАЗА ХОСТА. `run_domain.snapshot_and_request`
   строит снимок и запрос ДЛЯ ВСЕХ доменов меша одним циклом без try. У
   walls.001 это фатально: домен-склон законно отвергается
   `NEAR_PLANAR_RESIDUAL_BUDGET_EXCEEDED` на стадии экспорта, исключение
   уносит весь меш, и домен-дверь, который в поле СТРОИТСЯ, становится
   невидим. Здесь тот же цикл с тем же порядком вызовов
   (`stage_domain_inputs` -> `build_envelope_analysis_snapshot` ->
   `build_envelope_decal_request`), но отказ каждого домена ловится и
   становится ЗАПИСЬЮ этого домена. Тот же приём уже применён в
   `big_scene.py` (`except EnvelopeHostAdapterError` вокруг пары builder'ов) —
   это перенос его на слепки поменьше, а не новая политика.

2. КАП РАБОТЫ. Полевой сигнал владельца про building — «висит». Ворота,
   которые ждут бесконечно, не ворота. Кап здесь ВНЕШНИЙ и honest: домен
   считается в отдельном процессе с жёстким пределом секунд, превышение
   становится ИМЕНОВАННЫМ исходом `DOMAIN_WORK_CAP_EXCEEDED`, а не зависанием.
   Wall-clock назван явно: детерминированный `ExactWorkBudgetV1` в единицах
   работы — отдельная карточка (ремонт №2 порядка работ), и подменять её
   секундомером эта карточка не вправе. Секунды здесь сторожат ВОРОТА, а не
   выносят суждение о математике.
"""

from __future__ import annotations

import json
from pathlib import Path
import sys
import time

HERE = Path(__file__).resolve().parent
REPO_ROOT = HERE.parents[1]
DIAG = REPO_ROOT / "artifacts" / "perf_prepare_diag"
SNAPSHOTS = REPO_ROOT / "artifacts" / "field_snapshots"

if str(DIAG) not in sys.path:
    sys.path.insert(0, str(DIAG))

import env  # noqa: E402,F401  (пути + заглушки; ставит и kernel/src, и корень)
from run_domain import bundle_from_field_snapshot  # noqa: E402
from snapshot_bmesh import selected_edge_ids  # noqa: E402


HOST_REJECT = "HOST_EXPORT_REJECTED"
STAGE_RAISED = "STAGE_RAISED"
WORK_CAP_EXCEEDED = "DOMAIN_WORK_CAP_EXCEEDED"


#: Закрепки — ЯВНЫЕ аргументы маршрута, а не переменные окружения: унаследованная
#: от оболочки переменная молча меняла бы чужой разовый прогон. Неизвестный флаг —
#: отказ, а не молчаливое игнорирование.
PIN_LIFT_FLAG = "--pin-lift"
PIN_FRAME_FLAG = "--pin-frame"
PIN_LADDER_FLAG = "--pin-ladder"
PIN_FANS_FLAG = "--pin-fans"
PIN_JOIN_FLAG = "--pin-join"
#: Законы вееров ДО RIGHT-ANGLE-STABLE (2026-10-03): допуск восстановления 7e-6 рад,
#: граница шума привязки 1/1000, таблица лучей только лифтованного `(1/2, 4, 6)`, окно луча
#: Вороного. Закрепка веера — ТОЛЬКО красный контроль и переснятие таблицы якорей
#: (`refreeze_anchor_loci.py`): маршрут по умолчанию идёт продуктовыми законами и ничего не закрепляет.
#: Имя без суффикса `_ONLY` — все четыре закона; одиночные имена (через запятую можно несколько) называют
#: каждый закон отдельно, чтобы переснятие записывало, КАКОЙ закон сдвинул якорь.
FAN_LAWS_BEFORE_RIGHT_ANGLE_STABLE = "FAN_LAWS_BEFORE_RIGHT_ANGLE_STABLE_V1"
FAN_RESTORATION_TOLERANCE_PIN = "FAN_RESTORATION_TOLERANCE_BEFORE_RIGHT_ANGLE_STABLE_V1"
FAN_NOISE_BOUND_PIN = "FAN_BINDING_NOISE_BOUND_BEFORE_RIGHT_ANGLE_STABLE_V1"
FAN_ROTATION_TABLE_PIN = "FAN_ROTATION_TABLE_BEFORE_RIGHT_ANGLE_STABLE_V1"
FAN_RAY_WINDOW_PIN = "FAN_RAY_WINDOW_BEFORE_RIGHT_ANGLE_STABLE_V1"
#: Порог мягкого излома JOIN ДО решения 45° (2026-10-03): 30° = π/6.
JOIN_THRESHOLD_BEFORE_45 = "JOIN_SOFT_BEND_THRESHOLD_30_V1"


def split_pins(argv) -> tuple[list[str], dict[str, str]]:
    """Позиционные аргументы и закрепки: `--pin-lift`, `--pin-frame`, `--pin-ladder`, `--pin-fans`, `--pin-join`."""

    positional: list[str] = []
    pins: dict[str, str] = {}
    items = list(argv)
    while items:
        item = items.pop(0)
        if item in (
            PIN_LIFT_FLAG,
            PIN_FRAME_FLAG,
            PIN_LADDER_FLAG,
            PIN_FANS_FLAG,
            PIN_JOIN_FLAG,
        ):
            if not items:
                raise SystemExit(f"{item} needs a policy name")
            pins[item] = items.pop(0)
        elif item.startswith("--"):
            raise SystemExit(f"unknown field_route flag: {item}")
        else:
            positional.append(item)
    return positional, pins


def install_lift_pin(law_name: str) -> None:
    """Закрепить закон укладки хоста на время ЭТОГО процесса (подмена константы)."""

    from cftuv import envelope_request_export as export_module
    from cftuv.surface_ir import HostNearPlanarLiftPolicy

    export_module.HOST_NEAR_PLANAR_LIFT_POLICY = HostNearPlanarLiftPolicy(law_name)


def install_frame_pin(policy_name: str) -> None:
    """Закрепить политику репера near-planar хоста на время ЭТОГО процесса."""

    from cftuv import envelope_request_export as export_module
    from cftuv.surface_ir import HostNearPlanarFramePolicy

    export_module.HOST_NEAR_PLANAR_FRAME_POLICY = HostNearPlanarFramePolicy(
        policy_name
    )


def install_ladder_pin(policy_name: str) -> None:
    """Закрепить лестницу кривизны хоста (S1): `NEAR_PLANAR_ONLY_V1` — как до развёртки."""

    from cftuv import envelope_request_export as export_module
    from cftuv.surface_ir import HostCurvatureLadderPolicy

    export_module.HOST_CURVATURE_LADDER_POLICY = HostCurvatureLadderPolicy(policy_name)


def _pin_restoration_tolerance() -> None:
    from cftuv_envelope import _canonical_angle as canonical_module
    from cftuv_envelope._authoring_intent import AUTHOR_ANGULAR_ERROR

    canonical_module.CANONICAL_RESTORATION_ARTIST_ERROR = AUTHOR_ANGULAR_ERROR
    canonical_module._TOLERANCE_OVER_PI = (
        AUTHOR_ANGULAR_ERROR / canonical_module.PI_RATIONAL_UPPER_BOUND
    )


def _pin_noise_bound() -> None:
    from fractions import Fraction

    from cftuv_envelope.reference import evaluation_binding_noise as noise_module

    noise_module.NOISE_DIRECTION_SINE_BOUND = Fraction(1, 1000)


def _pin_rotation_table() -> None:
    from fractions import Fraction

    from cftuv_envelope import _density_policy as policy_module

    policy_module.CANONICAL_ROTATION_TABLE = {
        key: row
        for key, row in policy_module.CANONICAL_ROTATION_TABLE.items()
        if key == (Fraction(1, 2), 4, 6)
    }


def _pin_ray_window() -> None:
    from cftuv_envelope.reference import compile as compile_module
    from cftuv_envelope.reference.adaptive_density_band import WINDOW_LAW_VORONOI

    compile_module.FAN_WINDOW_LAW = WINDOW_LAW_VORONOI


_FAN_LAW_INSTALLERS = {
    FAN_RESTORATION_TOLERANCE_PIN: _pin_restoration_tolerance,
    FAN_NOISE_BOUND_PIN: _pin_noise_bound,
    FAN_ROTATION_TABLE_PIN: _pin_rotation_table,
    FAN_RAY_WINDOW_PIN: _pin_ray_window,
}


def install_fans_pin(name: str) -> None:
    """Закрепить законы вееров ядра ДО RIGHT-ANGLE-STABLE на время ЭТОГО процесса.

    `name` — `FAN_LAWS_BEFORE_RIGHT_ANGLE_STABLE_V1` (все четыре закона) либо список одиночных имён
    через запятую. Константы ядра возвращаются на прежние значения; сам код ядра не меняется.
    """

    names = (
        list(_FAN_LAW_INSTALLERS)
        if name == FAN_LAWS_BEFORE_RIGHT_ANGLE_STABLE
        else name.split(",")
    )
    for item in names:
        if item not in _FAN_LAW_INSTALLERS:
            raise SystemExit(f"unknown fan-laws pin: {item}")
    for item in names:
        _FAN_LAW_INSTALLERS[item]()


def install_join_pin(name: str) -> None:
    """Закрепить порог JOIN 30° (до решения 45°) на время ЭТОГО процесса.

    Порог читают три модуля: решение угла, проверка плана и его реэкспорт; без подмены всех трёх
    проверка отвергла бы план с прежним порогом.
    """

    if name != JOIN_THRESHOLD_BEFORE_45:
        raise SystemExit(f"unknown join pin: {name}")
    from fractions import Fraction

    from cftuv_envelope import _corner_treatment as treatment_module
    from cftuv_envelope import validation_corner_treatment as validation_module
    from cftuv_envelope.reference import corner_treatment as reference_module

    for module in (treatment_module, validation_module, reference_module):
        module.JOIN_THRESHOLD_OVER_PI = Fraction(1, 6)


def snapshot_sha256(name: str) -> str:
    import hashlib

    return hashlib.sha256((SNAPSHOTS / name).read_bytes()).hexdigest()


def install_building_substitution() -> str:
    """Именованная подмена UV-классификации: только для building_full."""

    import big_scene  # noqa: F401  (подмена ставится на импорте)

    return big_scene.SUBSTITUTION


def staged_domains(bundle, selected, *, alpha: float, density: int):
    """Домены меша: (patch_id, domain_id, snapshot, request) либо именованный отказ.

    Порядок и аргументы вызовов совпадают с `run_domain.snapshot_and_request`;
    отличие ровно одно — отказ домена не уносит остальные (см. докстринг №1).
    """

    from cftuv.envelope_request_export import (
        EnvelopeHostAdapterError,
        _typed_value,
        build_envelope_analysis_snapshot,
        build_envelope_decal_request,
    )
    from cftuv.envelope_topology_export import stage_domain_inputs

    (
        _scene,
        revision,
        patch_ids,
        request_id,
        selected_by_domain,
    ) = stage_domain_inputs(bundle, frozenset(selected))

    out = []
    for patch_id in patch_ids:
        domain_id = _typed_value("patch-domain", revision, patch_id)
        try:
            snapshot = build_envelope_analysis_snapshot(
                bundle, included_patch_ids=frozenset({patch_id})
            )
            request = build_envelope_decal_request(
                snapshot,
                frozenset(selected_by_domain[domain_id]),
                alpha,
                decal_request_id_value=request_id,
                density=density,
            )
        except EnvelopeHostAdapterError as error:
            out.append(
                {
                    "patch_id": int(patch_id),
                    "domain_id": domain_id,
                    "stage": "HOST_EXPORT",
                    "outcome": f"{HOST_REJECT}:{error.outcome.value}",
                    "detail": str(error),
                    "ready": None,
                }
            )
            continue
        out.append(
            {
                "patch_id": int(patch_id),
                "domain_id": domain_id,
                "stage": "QUEUE",
                "outcome": None,
                "detail": "",
                "ready": (patch_id, domain_id, snapshot, request),
            }
        )
    return out


def terms(value) -> list:
    """Каноническая сумма корней в JSON: [[радиканд, [числитель, знаменатель]], ...]."""

    return [
        [int(radicand), [int(c.numerator), int(c.denominator)]]
        for radicand, c in value.terms
    ]


def locus_record(node) -> dict:
    """Локус узла скелета: ТОЧНЫЕ (t, точка) и семантические participants.

    Ни `kind`, ни `converging_vertices`, ни `incidences` в ключ локуса не
    входят: ключ — это ГДЕ и КОГДА, а состав — то, что вокруг ключа может
    законно измениться при ремонте.
    """

    time = node.time.canonical()
    return {
        "time": {
            "dividend": [
                int(time.dividend.numerator),
                int(time.dividend.denominator),
            ],
            "divisor": terms(time.divisor),
        },
        "point": {"x": terms(node.point.x), "y": terms(node.point.y)},
        "participants": sorted(
            [int(v) for v in key] for key in node.participants
        ),
        "kind": node.kind.value,
        "kinds": [kind.value for kind in node.kinds],
        "converging_vertices": int(node.converging_vertices),
        "incidence_count": len(node.incidences),
    }


def run_domain_record(entry, *, alpha: float):
    """Одна ступень очереди на готовом домене. Возвращает плоскую запись."""

    from cftuv.envelope_queue_export import run_queue_domain

    patch_id, domain_id, snapshot, request = entry["ready"]
    started = time.perf_counter()
    prepared, domain = run_queue_domain(
        patch_id, domain_id, snapshot, request, str(alpha)
    )
    total = time.perf_counter() - started
    region = prepared.regions[0] if prepared.regions else None
    skeleton = None if region is None else region.skeleton
    return {
        "patch_id": int(patch_id),
        "domain_id": domain_id,
        "stage": "QUEUE",
        "outcome": domain.preparation_outcome,
        "coverage_outcome": domain.coverage_outcome,
        "detail": domain.detail,
        "faces": len(domain.faces),
        "lattice_scale": domain.lattice_scale,
        "counters": {name: int(value) for name, value in prepared.counters},
        "region_id": None if region is None else region.region_id,
        "bridge_outcome": None if region is None else region.bridge_outcome.value,
        "skeleton_outcome": (
            None if skeleton is None else skeleton.outcome.value
        ),
        "skeleton_nodes": 0 if skeleton is None else len(skeleton.nodes),
        "skeleton_proof_status": (
            None if skeleton is None else skeleton.proof_status.value
        ),
        "skeleton_obligations": (
            0 if skeleton is None else len(skeleton.proof_obligations)
        ),
        "face_outcome": (
            None
            if region is None or region.partition is None
            else region.partition.outcome.value
        ),
        "face_detail": (
            None
            if region is None or region.partition is None
            else region.partition.detail
        ),
        "owner_specs": (
            []
            if region is None
            else sorted({name for _, name in region.owner_by_edge})
        ),
        # Владение: КАЖДОЕ ребро-источник со своим спеком, а не только их набор.
        "owner_by_edge": (
            []
            if region is None
            else sorted(
                [[int(v) for v in key], name]
                for key, name in region.owner_by_edge
            )
        ),
        "wall_spans": (
            []
            if region is None
            else sorted([int(v) for v in key] for key in region.wall_spans)
        ),
        "ambiguous_owner_spans": (
            []
            if region is None
            else sorted(
                [int(v) for v in key] for key in region.ambiguous_owner_spans
            )
        ),
        "loci": (
            []
            if skeleton is None
            else [locus_record(node) for node in skeleton.nodes]
        ),
        "total_ms": round(total * 1000.0, 1),
        "prepare_ms": round(domain.prepare_seconds * 1000.0, 1),
        "coverage_ms": round(domain.coverage_seconds * 1000.0, 1),
        "contour_ms": round(domain.contour_seconds * 1000.0, 1),
    }


def run_snapshot(
    name: str, *, alpha: float, density: int, patch_filter=None,
    substitutions=(),
):
    """Полный маршрут на слепке. Каждый домен — своя запись, отказ тоже.

    `substitutions` едет В ОТВЕТЕ, а не остаётся в голове запускающего:
    подмена шага, которого без Blender не существует, обязана быть видна
    читателю расписки рядом с числами, которые она сделала возможными.
    """

    payload, bm, bundle = bundle_from_field_snapshot(SNAPSHOTS / name)
    selected = selected_edge_ids(payload)
    records = []
    for entry in staged_domains(
        bundle, selected, alpha=alpha, density=density
    ):
        if patch_filter is not None and entry["patch_id"] not in patch_filter:
            continue
        if entry["ready"] is None:
            entry.pop("ready")
            records.append(entry)
            continue
        records.append(run_domain_record(entry, alpha=alpha))
    return {
        "snapshot": name,
        "object_name": payload["object_name"],
        "sha256": snapshot_sha256(name),
        "alpha": alpha,
        "density": density,
        "substitutions": list(substitutions),
        "mesh": {
            "vertices": len(bm.verts),
            "edges": len(bm.edges),
            "faces": len(bm.faces),
            "selected_edges": len(selected),
        },
        "domains": records,
    }


def _main() -> None:
    """CLI: `field_route.py <snapshot> <alpha> <density> [patch,patch] [out] [--pin-lift N] [--pin-frame N]`."""

    arguments, pins = split_pins(sys.argv[1:])
    name = arguments[0]
    alpha = float(arguments[1])
    density = int(arguments[2])
    patches = (
        frozenset(int(v) for v in arguments[3].split(","))
        if len(arguments) > 3 and arguments[3] not in ("", "-")
        else None
    )
    substitutions = []
    pinned = pins.get(PIN_LIFT_FLAG)
    if pinned:
        # ИМЕНОВАННАЯ ЗАКРЕПКА закона укладки. Ворота математики фронта (walls.012)
        # проверяют ЯДРО на полевой геометрии, а не политику укладки хоста; закон
        # NEAR_PLANAR V2 отказывает этот домен по ширине (см. DECISIONS 2026-10-03),
        # и без закрепки сильные ворота перестали бы видеть математику. Закрепка
        # едет в ответе: прогон без неё — прогон по настоящему закону хоста.
        install_lift_pin(pinned)
        substitutions.append(f"HOST_NEAR_PLANAR_LIFT_POLICY_PINNED:{pinned}")
    pinned_frame = pins.get(PIN_FRAME_FLAG)
    if pinned_frame:
        # Вторая ИМЕНОВАННАЯ закрепка. Таблица якорных локусов записана в канонических
        # координатах карты; приведённый целочисленный базис (NEAR_PLANAR V2, коммит 4)
        # даёт ту же плоскость в других `(u, v)`, и якорь, сравниваемый по точным
        # координатам, перестаёт находиться, хотя геометрия не сдвинулась. Ворота
        # проверяют математику фронта на полевой геометрии, а не выбор репера; настоящий
        # репер хоста держат отдельные тесты ядра.
        install_frame_pin(pinned_frame)
        substitutions.append(f"HOST_NEAR_PLANAR_FRAME_POLICY_PINNED:{pinned_frame}")
    pinned_ladder = pins.get(PIN_LADDER_FLAG)
    if pinned_ladder:
        # Третья ИМЕНОВАННАЯ закрепка: ворота прежней математики near-planar (отказ по
        # ширине, имена, числа) держат прежний ответ ПОД РАЗВЁРТКОЙ тоже: с настоящей
        # лестницей хоста те же домены уходят на ступень ниже, и это другой ответ.
        install_ladder_pin(pinned_ladder)
        substitutions.append(f"HOST_CURVATURE_LADDER_POLICY_PINNED:{pinned_ladder}")
    pinned_fans = pins.get(PIN_FANS_FLAG)
    if pinned_fans:
        # Четвёртая ИМЕНОВАННАЯ закрепка — КРАСНЫЙ КОНТРОЛЬ, не режим ворот: таблица якорных
        # локусов снята на продуктовых законах вееров, и этот прогон возвращает прежние
        # (допуск 7e-6 рад, окно Вороного, таблица только поднятого d4). Ворота якорей обязаны
        # на нём краснеть (`test_anchor_gate_goes_red_when_an_old_law_is_re_enabled`), иначе они
        # перестали видеть закон. Имя едет в `substitutions`.
        install_fans_pin(pinned_fans)
        substitutions.append(f"KERNEL_FAN_LAWS_PINNED:{pinned_fans}")
    pinned_join = pins.get(PIN_JOIN_FLAG)
    if pinned_join:
        # Пятая ИМЕНОВАННАЯ закрепка — тоже красный контроль: порог JOIN 30° возвращает
        # локусы, которые закон 45° снял (`retired_by_join` в таблице якорей).
        install_join_pin(pinned_join)
        substitutions.append(f"KERNEL_JOIN_THRESHOLD_PINNED:{pinned_join}")
    if name == "building_full_snapshot.json":
        # Классификация OUTER/HOLE у многопетлевых патчей идёт в продакшне
        # через временный UV-unwrap внутри Blender. Без Blender шага НЕ
        # СУЩЕСТВУЕТ, и подмена другим модульным путём того же файла — не
        # деталь запуска, а факт, который расписка обязана нести.
        substitutions.append(install_building_substitution())
    result = run_snapshot(
        name,
        alpha=alpha,
        density=density,
        patch_filter=patches,
        substitutions=substitutions,
    )
    text = json.dumps(result, ensure_ascii=False, indent=1)
    if len(arguments) > 4:
        Path(arguments[4]).write_text(text, encoding="utf-8")
    print(text)


if __name__ == "__main__":
    _main()
