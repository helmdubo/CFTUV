"""Контракт `tools/field_sweep_detail_report.py`: причины отказов -> группы.

Почему вход синтетический, а не сохранённый артефакт. Сохранённых расписок
очереди в дереве нет вовсе: `preparation_outcome` не встречается ни в одном
файле `artifacts/` (перемерено), то есть ни детали, ни исхода домена очереди
взять там негде. Инструмент читает ФОРМАТ, а не конкретный файл, поэтому
формат здесь и воспроизводится.

Почему фикстура строится кодом ЭКСПОРТЁРА, а не литеральным JSON. Литерал —
это вторая копия формата: он остаётся зелёным ровно до того дня, когда
`queue_domain_payload` переименует ключ, и тогда тест продолжит доказывать
согласие инструмента с собственной выдумкой. Здесь sidecar-раздел собирают
`refused_queue_domain` и `queue_scene_payload` — те самые функции хоста
(`cftuv/envelope_queue_export.py`), поэтому расхождение инструмента с
продуктом становится красным, а не невидимым. Blender для этого не нужен:
экспортёр отображает контракты ядра и bpy не трогает.

Обёртка sidecar'а (`schema`, `object_name`, ключ `queue`) собирается словарём:
её пишет `_staged_sidecar` в `cftuv/envelope_debug_renderer.py:1303-1349`, а
рендерер тянет за собой Grease Pencil и bpy. Источник формата назван здесь
ссылкой, а имена ступеней и ключей, которые инструмент СРАВНИВАЕТ, проверены
против продукта отдельными тестами ниже.
"""

from __future__ import annotations

import ast
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace

import pytest


REPO_ROOT = Path(__file__).resolve().parents[1]
KERNEL_SRC = REPO_ROOT / "kernel" / "src"
if str(KERNEL_SRC) not in sys.path:
    sys.path.insert(0, str(KERNEL_SRC))

TOOL_PATH = REPO_ROOT / "tools" / "field_sweep_detail_report.py"
FIELD_GATE = REPO_ROOT / "tools" / "run_envelope_mr1_building_gate.py"
BUTTON_SWEEP = REPO_ROOT / "tools" / "blender_field_sweep.py"

from cftuv.envelope_debug_profile import EnvelopeDomainStage  # noqa: E402
from cftuv.envelope_queue_export import (  # noqa: E402
    EnvelopeQueueDomainV1,
    _queue_stage,
    build_queue_scene,
    queue_domain_payload,
    queue_scene_payload,
    refused_queue_domain,
)


def _load_tool():
    """Инструмент импортируется файлом: `tools/` не пакет, а он — только stdlib."""

    spec = importlib.util.spec_from_file_location(
        "field_sweep_detail_report", TOOL_PATH
    )
    module = importlib.util.module_from_spec(spec)
    # Модуль кладётся в `sys.modules` ДО исполнения: `dataclass` ищет там
    # собственный модуль класса, и без записи разбор аннотаций падает.
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


REPORT = _load_tool()


def _refused_domain(patch_id: int, domain_id: str, outcome: str, detail: str):
    """Отказавший домен строит сам экспортёр — из подготовки, как в поле.

    `prepared` подставляется минимальный: `refused_queue_domain` читает у него
    исход, деталь, регионы, решётку, имена законов, счётчики и тайминги — и
    ровно это здесь и объявлено. Ни одного поля сверх того, что читает продукт.
    """

    prepared = SimpleNamespace(
        outcome=SimpleNamespace(value=outcome),
        detail=detail,
        regions=(),
        lattice=None,
        law_names=(),
        counters=(),
        timings=(),
    )
    return refused_queue_domain(
        patch_id,
        domain_id,
        prepared,
        None,
        prepare_seconds=0.25,
        coverage_seconds=0.0,
    )


def _exact_domain(patch_id: int, domain_id: str):
    """Прошедший домен — записью экспортёра напрямую.

    `build_queue_domain` требует настоящих регионов и покрытия с гранями; для
    знаменателя таблицы нужны только исходы, и подделывать ради них геометрию
    значило бы проверять подделку. Запись — та же `EnvelopeQueueDomainV1`, и
    сериализует её тот же `queue_domain_payload`.
    """

    return EnvelopeQueueDomainV1(
        patch_id=patch_id,
        patch_domain_id=domain_id,
        preparation_outcome="EXACT",
        coverage_outcome="EXACT",
        detail="",
        lattice_scale=1,
        alpha="1/2",
        lattice_alpha="1/2",
        law_names=(),
        regions=(),
        faces=(),
        segments=(),
        counters=(),
        timings=(),
        prepare_seconds=0.1,
        coverage_seconds=0.1,
        contour_seconds=0.0,
    )


# Детали `PLAN_IS_NOT_COMPILED` — ИМЕНА исходов компиляции эталона: конвейер
# кладёт в деталь `compiled.outcome.value`
# (`kernel/src/cftuv_envelope/wavefront/conveyor.py:1200-1202`). Берутся из
# самого перечисления ядра, а не переписаны строкой.
def _reference_outcome(name: str) -> str:
    from cftuv_envelope.reference.contracts import ReferenceOutcome

    return ReferenceOutcome[name].value


PLAN_DETAIL_A = _reference_outcome("REFERENCE_MULTISECTOR_CHAIN_USE_UNSUPPORTED")
PLAN_DETAIL_B = _reference_outcome("REFERENCE_JUNCTION_ENVELOPE_LAW_UNPROVEN")


def _bridge_detail(region_id: str, name: str) -> str:
    """Деталь `BRIDGE_DID_NOT_MAP` — `«регион: исход моста»`.

    Форма не сочинена: конвейер собирает её как
    `f"{region.region_id}: {region.bridge_outcome.value}"`
    (`kernel/src/cftuv_envelope/wavefront/conveyor.py:1015-1020`). Имя исхода
    берётся из `BridgeOutcome`, а не переписывается строкой.
    """

    from cftuv_envelope.wavefront.bridge import BridgeOutcome

    return f"{region_id}: {BridgeOutcome[name].value}"


BRIDGE_DETAIL = _bridge_detail(
    "4.0.r0", "SOURCE_EDGE_INSIDE_A_LINE_CLASS_IS_UNDETERMINED"
)


def _gate_payload(mesh: str, density: int, rows) -> dict:
    """Расписка прямых ворот полевого цикла — минимальным словарём.

    Полностью её строит `tools/run_envelope_mr1_building_gate.py` (payload на
    `:944-970`, раздел домена — `_queue_payload` на `:474-477`), и вызвать его
    здесь нечем: модуль ворот импортирует `bpy`. Поэтому имена ключей раздела
    `queue` проверяются разбором его исходника (см. тест ниже), а не доверием.
    """

    return {
        "schema": REPORT.GATE_SCHEMA,
        "source_object": mesh,
        "effective_density": density,
        "runs": [
            {
                "scope": "all_seam_chains_l0",
                "domains": [
                    {
                        "patch_id": patch_id,
                        "patch_domain_id": domain_id,
                        "queue": {
                            "preparation_outcome": preparation,
                            "coverage_outcome": coverage,
                            "detail": detail,
                        },
                    }
                    for patch_id, domain_id, preparation, coverage, detail in rows
                ],
            }
        ],
    }


def _sidecar_payload(object_name: str, domains) -> dict:
    return {
        "schema": REPORT.SIDECAR_SCHEMA,
        "object_name": object_name,
        "queue": queue_scene_payload(build_queue_scene(domains)),
    }


@pytest.fixture()
def field_sidecar(tmp_path: Path) -> Path:
    """Полевая раскладка в миниатюре: 3 + 1 + 1 отказ и два прошедших домена."""

    domains = [
        _refused_domain(index, f"{index}.0", "PLAN_IS_NOT_COMPILED", PLAN_DETAIL_A)
        for index in range(3)
    ]
    domains.append(
        _refused_domain(3, "3.0", "PLAN_IS_NOT_COMPILED", PLAN_DETAIL_B)
    )
    domains.append(
        _refused_domain(4, "4.0", "BRIDGE_DID_NOT_MAP", BRIDGE_DETAIL)
    )
    domains.extend((_exact_domain(5, "5.0"), _exact_domain(6, "6.0")))
    path = tmp_path / "sidecar.json"
    path.write_text(
        json.dumps(_sidecar_payload("ENVDBG_2", domains), ensure_ascii=False),
        encoding="utf-8",
    )
    return path


def test_copied_stage_names_equal_the_product_enum():
    """Имена ступеней в инструменте — копии, и копии обязаны совпадать.

    Инструмент только stdlib и продукт не импортирует (расписку читают там, где
    аддон не установлен). Цена такого решения — три строковых литерала, и
    единственное, что не даёт им разъехаться с `EnvelopeDomainStage`, — эта
    проверка.
    """

    assert (
        REPORT.QUEUE_PREPARE_REJECTED
        == EnvelopeDomainStage.QUEUE_PREPARE_REJECTED.value
    )
    assert (
        REPORT.QUEUE_COVERAGE_REJECTED
        == EnvelopeDomainStage.QUEUE_COVERAGE_REJECTED.value
    )
    assert REPORT.QUEUE_RESOLVED == EnvelopeDomainStage.QUEUE_RESOLVED.value


@pytest.mark.parametrize(
    ("preparation", "coverage"),
    (
        ("EXACT", "EXACT"),
        ("EXACT", "COVERAGE_IS_NOT_EXACT"),
        ("PLAN_IS_NOT_COMPILED", ""),
        ("BRIDGE_DID_NOT_MAP", "PREPARATION_IS_NOT_EXACT"),
    ),
)
def test_record_stage_agrees_with_host_stage_rule(preparation, coverage):
    """Ступень домена инструмент считает тем же правилом, что и хост.

    Своё правило разошлось бы с колонкой `stage` расписок, по которой владелец
    и сверяет таблицу; расхождение было бы тихим — обе стороны печатали бы
    правдоподобные имена.
    """

    domain = EnvelopeQueueDomainV1(
        patch_id=0,
        patch_domain_id="0.0",
        preparation_outcome=preparation,
        coverage_outcome=coverage,
        detail="",
        lattice_scale=1,
        alpha="",
        lattice_alpha="",
        law_names=(),
        regions=(),
        faces=(),
        segments=(),
        counters=(),
        timings=(),
        prepare_seconds=0.0,
        coverage_seconds=0.0,
        contour_seconds=0.0,
    )
    record = REPORT.DomainRecord(
        source="",
        mesh="",
        density=None,
        scope="",
        patch_id=0,
        patch_domain_id="0.0",
        preparation_outcome=preparation,
        coverage_outcome=coverage,
        detail="",
    )
    assert record.stage == _queue_stage(domain).value


def test_tool_reads_only_keys_the_exporter_writes():
    """Ключи, которые инструмент берёт у домена, пишет `queue_domain_payload`.

    Иначе опечатка в имени ключа даёт не падение, а пустую колонку: `.get`
    вернёт `None`, и таблица честно покажет «деталей нет».
    """

    written = set(queue_domain_payload(_exact_domain(0, "0.0")))
    assert {
        "patch_id",
        "patch_domain_id",
        "preparation_outcome",
        "coverage_outcome",
        "detail",
    } <= written


def test_sidecar_groups_are_sorted_by_frequency(field_sidecar: Path):
    """Главная работа инструмента: (исход, деталь) -> счёт -> поимённо домены.

    Полевая строка «`PLAN_IS_NOT_COMPILED`: причины не разобраны» закрывается
    именно этой таблицей: одинаковый исход с РАЗНЫМИ деталями — это два разных
    счёта, и в один класс они не сливаются.
    """

    report = REPORT.build_report(REPORT.read_paths([str(field_sidecar)]))
    assert report["schema"] == REPORT.REPORT_SCHEMA
    assert report["domain_count"] == 7
    assert report["refused_count"] == 5
    mesh = report["meshes"][0]
    assert mesh["mesh"] == "ENVDBG_2"
    assert [
        (item["outcome"], item["detail"], item["count"])
        for item in mesh["groups"]
    ] == [
        ("PLAN_IS_NOT_COMPILED", PLAN_DETAIL_A, 3),
        ("EXACT", REPORT.DETAIL_IS_EMPTY, 2),
        ("BRIDGE_DID_NOT_MAP", BRIDGE_DETAIL, 1),
        ("PLAN_IS_NOT_COMPILED", PLAN_DETAIL_B, 1),
    ]
    assert mesh["groups"][0]["patch_domain_ids"] == ["0.0", "1.0", "2.0"]
    assert mesh["groups"][0]["stages"] == [REPORT.QUEUE_PREPARE_REJECTED]


def test_domains_of_a_group_are_listed_by_patch_number(tmp_path: Path):
    """Список доменов группы читается как список патчей поля.

    Строковая сортировка ставит `10.0` между `1.0` и `2.0`; на двенадцати
    доменах меша «2» такой список перестаёт быть пригодным для сверки глазами,
    ради которой он и печатается.
    """

    domains = [
        _refused_domain(index, f"{index}.0", "PLAN_IS_NOT_COMPILED", PLAN_DETAIL_A)
        for index in (2, 10, 1)
    ]
    path = tmp_path / "sidecar.json"
    path.write_text(
        json.dumps(_sidecar_payload("ENVDBG_2", domains), ensure_ascii=False),
        encoding="utf-8",
    )
    report = REPORT.build_report(REPORT.read_paths([str(path)]))
    assert report["meshes"][0]["groups"][0]["patch_domain_ids"] == [
        "1.0",
        "2.0",
        "10.0",
    ]


def test_empty_detail_is_named_and_not_printed_as_a_hole(field_sidecar: Path):
    """Пустая деталь получает имя.

    Пустая ячейка в таблице неотличима от «графа не заполнилась»; у прошедшего
    домена деталь пуста ПО ПОСТРОЕНИЮ (`build_queue_domain`: `coverage.detail
    or prepared.detail`), и это разные утверждения.
    """

    report = REPORT.build_report(REPORT.read_paths([str(field_sidecar)]))
    exact = next(
        item for item in report["totals"] if item["outcome"] == "EXACT"
    )
    assert exact["detail"] == REPORT.DETAIL_IS_EMPTY


def test_gate_receipt_domains_are_read_from_the_queue_section(tmp_path: Path):
    """Вторая читаемая форма — расписка прямых ворот полевого цикла.

    Имена полей взяты не на глаз: тот же тест проверяет, что `_queue_payload`
    в `tools/run_envelope_mr1_building_gate.py` их и кладёт.
    """

    module = ast.parse(FIELD_GATE.read_text(encoding="utf-8"))
    function = next(
        node
        for node in module.body
        if isinstance(node, ast.FunctionDef) and node.name == "_queue_payload"
    )
    returned = next(
        node.value
        for node in ast.walk(function)
        if isinstance(node, ast.Return) and isinstance(node.value, ast.Dict)
    )
    keys = {key.value for key in returned.keys if isinstance(key, ast.Constant)}
    assert {"preparation_outcome", "coverage_outcome", "detail", "stage"} <= keys

    payload = _gate_payload(
        "building",
        4,
        [(11, "11.0", "PLAN_IS_NOT_COMPILED", "", PLAN_DETAIL_A)],
    )
    # Домен, которого в прогоне очереди не было (движок `raw`): раздела
    # `queue` у него нет вовсе, и в таблицу он попасть не должен.
    payload["runs"][0]["domains"].append(
        {"patch_id": 12, "patch_domain_id": "12.0", "stage": "RAW_READY"}
    )
    path = tmp_path / "building_density_4_field_run.json"
    path.write_text(json.dumps(payload, ensure_ascii=False), encoding="utf-8")

    report = REPORT.build_report(REPORT.read_paths([str(path)]))
    assert report["domain_count"] == 1
    mesh = report["meshes"][0]
    assert (mesh["mesh"], mesh["effective_density"]) == ("building", 4)
    assert mesh["groups"][0]["patch_domain_ids"] == ["11.0"]


def test_same_mesh_at_two_densities_stays_two_rows(tmp_path: Path):
    """Один меш на двух плотностях — две строки, а не сумма.

    Полевой цикл кладёт расписки d0/d1/d4 в один каталог, и складывать их в
    один счёт значило бы отвечать «сколько отказов у building» числом, которое
    не соответствует ни одному прогону.
    """

    for density, detail in ((4, PLAN_DETAIL_A), (0, PLAN_DETAIL_B)):
        payload = _gate_payload(
            "building",
            density,
            [(11, "11.0", "PLAN_IS_NOT_COMPILED", "", detail)],
        )
        (tmp_path / f"building_density_{density}_field_run.json").write_text(
            json.dumps(payload, ensure_ascii=False), encoding="utf-8"
        )

    report = REPORT.build_report(REPORT.read_paths([str(tmp_path)]))
    assert [
        (item["mesh"], item["effective_density"], item["domain_count"])
        for item in report["meshes"]
    ] == [("building", 0, 1), ("building", 4, 1)]


def test_sweep_receipt_names_its_missing_detail(tmp_path: Path):
    """Расписка свипа деталей не несёт — и говорит об этом, а не молчит.

    `parity_projection` собирается из исходов и дайджеста
    (`tools/blender_field_sweep.py:504-527`), детали в ней нет по построению.
    Пустая таблица без объяснения читалась бы как «отказов не было».
    """

    sweep = ast.parse(BUTTON_SWEEP.read_text(encoding="utf-8"))
    projection = next(
        node
        for node in sweep.body
        if isinstance(node, ast.FunctionDef)
        and node.name == "_button_parity_projection"
    )
    projected_keys = {
        key.value
        for node in ast.walk(projection)
        if isinstance(node, ast.Dict)
        for key in node.keys
        if isinstance(key, ast.Constant)
    }
    assert "detail" not in projected_keys

    payload = {
        "schema": REPORT.SWEEP_SCHEMA,
        "densities": [
            {
                "effective_density": 4,
                "direct_button_parity": [
                    {"direct_receipt": "receipts/building_density_4_field_run.json"}
                ],
                "reports": [{"mesh": "building", "parity_projection": []}],
            }
        ],
    }
    path = tmp_path / "sweep.json"
    path.write_text(json.dumps(payload, ensure_ascii=False), encoding="utf-8")

    reading = REPORT.read_source(path)
    assert reading.records == ()
    assert reading.note.startswith(REPORT.SWEEP_RECEIPT_CARRIES_NO_DETAIL)
    assert "receipts/building_density_4_field_run.json" in reading.note


def test_directory_input_is_walked_and_totals_add_up(tmp_path: Path):
    """Каталог расписок — один прогон кнопки владельца, а не файл за файлом."""

    for index, mesh in enumerate(("ENVDBG_2", "ENVDBG_building")):
        domains = [
            _refused_domain(index, f"{index}.0", "PLAN_IS_NOT_COMPILED", PLAN_DETAIL_A)
        ]
        (tmp_path / f"{mesh}.json").write_text(
            json.dumps(_sidecar_payload(mesh, domains), ensure_ascii=False),
            encoding="utf-8",
        )

    report = REPORT.build_report(REPORT.read_paths([str(tmp_path)]))
    assert report["domain_count"] == 2
    assert [item["mesh"] for item in report["meshes"]] == [
        "ENVDBG_2",
        "ENVDBG_building",
    ]
    total = report["totals"][0]
    assert (total["outcome"], total["count"]) == ("PLAN_IS_NOT_COMPILED", 2)
    assert total["meshes"] == {"ENVDBG_2": 1, "ENVDBG_building": 1}


def test_unknown_shape_is_named_not_silently_empty(tmp_path: Path):
    """Чужой JSON — именованный ответ, а не пустая таблица."""

    path = tmp_path / "alien.json"
    path.write_text(json.dumps({"schema": "something.else"}), encoding="utf-8")
    reading = REPORT.read_source(path)
    assert reading.records == ()
    assert reading.note == REPORT.SOURCE_IS_NOT_A_FIELD_RECEIPT


def test_cli_prints_table_and_json_and_fails_closed_on_empty_input(
    field_sidecar: Path,
    tmp_path: Path,
):
    """Кнопка владельца — один вызов; вход без доменов не притворяется успехом."""

    table = subprocess.run(
        [sys.executable, str(TOOL_PATH), str(field_sidecar)],
        cwd=REPO_ROOT,
        text=True,
        capture_output=True,
        check=False,
    )
    assert table.returncode == 0, table.stderr
    assert "PLAN_IS_NOT_COMPILED" in table.stdout
    assert PLAN_DETAIL_A in table.stdout
    assert "ИТОГО" in table.stdout

    encoded = subprocess.run(
        [sys.executable, str(TOOL_PATH), "--json", str(field_sidecar)],
        cwd=REPO_ROOT,
        text=True,
        capture_output=True,
        check=False,
    )
    assert encoded.returncode == 0, encoded.stderr
    payload = json.loads(encoded.stdout)
    assert payload["schema"] == REPORT.REPORT_SCHEMA
    assert payload["totals"][0]["count"] == 3

    empty = tmp_path / "empty"
    empty.mkdir()
    nothing = subprocess.run(
        [sys.executable, str(TOOL_PATH), str(empty)],
        cwd=REPO_ROOT,
        text=True,
        capture_output=True,
        check=False,
    )
    assert nothing.returncode == 2
    assert REPORT.NO_QUEUE_DOMAINS_IN_INPUT in nothing.stderr
