"""Контракт раннера оракула R0 — БЕЗ вызова Surfer2.

Почему без бинаря. Бинаря Surfer2 в дереве нет и не будет: GPL-3, clean-room,
в CI он собирается по расписке `artifacts/surfer2_r0_spike/build_receipt.json`.
Тест, требующий его, красил бы чистый клон в красный за отсутствие того, чего
в репозитории нет по решению. Поэтому здесь проверяется ровно то, что раннер
делает САМ: перевод входа в GraphML, перевод нашего `q` в вес Surfer2, разбор
OBJ, вывод допуска, фильтр внутренности с дырами, паросочетание и именование
отказов. Разговор с живым бинарём — `tools/surfer2_corpus_run.py`, и он
именованно пропускается, когда бинаря нет.

Где бинарь всё-таки нужен процессу — там стоит ЗАГЛУШКА: подставной
исполняемый файл, который печатает то, что попросили. Она проверяет обёртку
процесса (rc, tempdir, дайджест stderr, «нет ответа» вместо пустого списка), а
не математику Surfer2, и потому честна.
"""

from __future__ import annotations

import importlib.util
import json
import os
import subprocess
import sys
from fractions import Fraction
from pathlib import Path
from types import SimpleNamespace
from xml.etree import ElementTree

import pytest


REPO_ROOT = Path(__file__).resolve().parents[1]
TOOLS = REPO_ROOT / "tools"
KERNEL_SRC = REPO_ROOT / "kernel" / "src"
KERNEL_TESTS = REPO_ROOT / "kernel" / "tests"
for _path in (KERNEL_SRC, KERNEL_TESTS):
    if str(_path) not in sys.path:
        sys.path.insert(0, str(_path))


def _load(name: str):
    """Инструмент импортируется файлом: `tools/` не пакет."""

    spec = importlib.util.spec_from_file_location(name, TOOLS / f"{name}.py")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


ORACLE = _load("surfer2_oracle")
CORPUS = _load("surfer2_corpus_run")

POSIX_ONLY = pytest.mark.skipif(
    os.name != "posix",
    reason="STUB_BINARY_NEEDS_POSIX_SHEBANG: заглушка исполняется шебангом",
)


def _stub(tmp_path: Path, body: str) -> Path:
    """Подставной «бинарь»: получает те же argv, что настоящий."""

    path = tmp_path / "stub-surfer"
    path.write_text(f"#!{sys.executable}\n{body}", encoding="utf-8")
    path.chmod(0o755)
    return path


def _square(side: int = 8) -> ORACLE.RingV1:
    points = ((0, 0), (side, 0), (side, side), (0, side))
    return ORACLE.RingV1(
        tuple((Fraction(x), Fraction(y)) for x, y in points),
        (Fraction(1),) * 4,
    )


# --------------------------------------------------------------------------
# 1. Вход: рациональные числа строками
# --------------------------------------------------------------------------


@pytest.mark.parametrize(
    ("value", "expected"),
    [
        (8, "8"),
        (-3, "-3"),
        (Fraction(3, 2), "3/2"),
        (Fraction(4, 2), "2"),
        (Fraction(-7, 3), "-7/3"),
    ],
)
def test_rational_text_writes_p_over_q(value, expected):
    """Число уходит в GraphML СТРОКОЙ и без потери разряда.

    Это не косметика формата: при точном ядре Surfer2 делает
    `string_to_maybe_NT(x) = x`, то есть строит CORE::Expr прямо из текста.
    Пройди число через `double`, лосслесс кончился бы на входе.
    """

    assert ORACLE.rational_text(value) == expected


def test_graphml_is_well_formed_and_carries_every_ring():
    """Многокольцевой вход: узлы всех колец, рёбра ЗАМЫКАЮТ каждое кольцо.

    Дыра здесь не надстройка над односвязным генератором: кольца идут подряд,
    и единственное, что их различает, — какие узлы соединены рёбрами. Ребра
    между кольцами быть не должно ни одного.
    """

    hole = ORACLE.RingV1(
        tuple(
            (Fraction(x), Fraction(y))
            for x, y in ((2, 2), (2, 6), (6, 6), (6, 2))
        ),
        (Fraction(1),) * 4,
    )
    document = ORACLE.graphml_document((_square(), hole))
    root = ElementTree.fromstring(document)
    namespace = {"g": "http://graphml.graphdrawing.org/xmlns"}
    nodes = root.findall(".//g:node", namespace)
    edges = root.findall(".//g:edge", namespace)
    assert [node.get("id") for node in nodes] == [str(i) for i in range(8)]
    spans = {
        frozenset((int(edge.get("source")), int(edge.get("target"))))
        for edge in edges
    }
    assert spans == {
        frozenset((0, 1)), frozenset((1, 2)), frozenset((2, 3)), frozenset((3, 0)),
        frozenset((4, 5)), frozenset((5, 6)), frozenset((6, 7)), frozenset((7, 4)),
    }


def test_graphml_declares_string_attributes_and_writes_weights():
    """Веса объявлены строковым типом и печатаются дробью, а не десятичной."""

    ring = ORACLE.RingV1(_square().points, (Fraction(3, 2),) * 4)
    document = ORACLE.graphml_document((ring,))
    assert 'attr.name="edge-weight" attr.type="string"' in document
    assert document.count("<data key=\"w\">3/2</data>") == 4


# --------------------------------------------------------------------------
# 2. Веса: наше `q` против евклидовой скорости Surfer2
# --------------------------------------------------------------------------


@pytest.mark.parametrize(
    ("value", "expected"),
    [
        (Fraction(4), Fraction(2)),
        (Fraction(9, 4), Fraction(3, 2)),
        (Fraction(0), Fraction(0)),
        (Fraction(2), None),
        (Fraction(1, 2), None),
        (Fraction(-1), None),
    ],
)
def test_rational_square_root_is_an_integer_predicate(value, expected):
    """Рациональность корня решается целыми, а не приближением и порогом."""

    assert ORACLE.rational_square_root(value) == expected


@pytest.mark.parametrize(
    ("speed_squared", "start", "end", "expected"),
    [
        # умолчание корпуса: q = |d|^2 -> единичная евклидова скорость
        (64, (0, 0), (8, 0), Fraction(1)),
        (256, (0, 0), (8, 0), Fraction(2)),
        (144, (0, 0), (8, 0), Fraction(3, 2)),
        # диагональ длины sqrt(2): q = |d|^2 = 2 -> вес всё равно 1
        (2, (0, 0), (1, 1), Fraction(1)),
        # q = 2*|d|^2 -> скорость sqrt(2), строкой не передаётся
        (128, (0, 0), (8, 0), None),
    ],
)
def test_surfer_edge_weight_translates_our_q(speed_squared, start, end, expected):
    """`w = sqrt(q)/|d|`: их вес — ЕВКЛИДОВА скорость, наше `q` — её квадрат
    в единицах нормали длины `|d|`. Перевод один на репозиторий, потому что
    второй немедленно разошёлся бы с первым."""

    assert ORACLE.surfer_edge_weight(speed_squared, start, end) == expected


# --------------------------------------------------------------------------
# 3. Пересечение моделей: отсев именованный, не тихий
# --------------------------------------------------------------------------


def test_unit_polygon_with_holes_is_admitted():
    import wavefront_cases as wc

    admission, _, rings = ORACLE.rings_of_polygon(wc.holes_grid(1, 2))
    assert admission is ORACLE.AdmissionOutcome.ADMITTED
    assert [len(ring.points) for ring in rings] == [4, 4, 4]
    assert {weight for ring in rings for weight in ring.weights} == {Fraction(1)}


def test_wall_edge_is_named_out_of_the_intersection():
    """Стена (`q = 0`) невыразима: у Surfer2 фронт идёт с ОБЕИХ сторон ребра."""

    from cftuv_envelope.wavefront.polygon import LoopV1, PolygonV1

    polygon = PolygonV1(LoopV1(((0, 0), (8, 0), (8, 8), (0, 8)), (64, 0, 64, 64)))
    admission, detail, rings = ORACLE.rings_of_polygon(polygon)
    assert admission is ORACLE.AdmissionOutcome.SURFER2_MODEL_HAS_NO_WALL_EDGE
    assert rings == ()
    assert "q = 0" in detail


def test_irrational_speed_is_named_out_of_the_intersection():
    """`sqrt(q)` иррационален -> вход вне области. Решение принято R0-спайком:
    весовой дифференциал на таких входах остаётся in-house (P0-3)."""

    from cftuv_envelope.wavefront.polygon import LoopV1, PolygonV1

    polygon = PolygonV1(
        LoopV1(((0, 0), (8, 0), (8, 8), (0, 8)), (128, 64, 64, 64))
    )
    admission, detail, _ = ORACLE.rings_of_polygon(polygon)
    assert (
        admission
        is ORACLE.AdmissionOutcome.SURFER2_MODEL_HAS_NO_IRRATIONAL_EDGE_SPEED
    )
    assert "иррациональна" in detail


def test_vertex_fan_is_named_out_of_the_intersection():
    """Веер — рёбра НУЛЕВОЙ длины; в GraphML это повторившийся узел, то есть
    вход, которого у них не бывает."""

    polygon = SimpleNamespace(vertex_fans=("не пусто",), loops=())
    admission, detail, _ = ORACLE.rings_of_polygon(polygon)
    assert admission is ORACLE.AdmissionOutcome.SURFER2_MODEL_HAS_NO_VERTEX_FAN
    assert "нулевой длины" in detail


def test_weighted_corpus_is_rational_and_not_trivially_unit():
    """Взвешенный корпус лежит в пересечении и НЕ вырожден в единичные веса.

    Без второй половины утверждения тест был бы зелёным и на корпусе, где
    веса молча стали единичными, — то есть доказывал бы согласие с самим собой.
    """

    weights = set()
    for _, polygon in CORPUS.weighted_corpus():
        admission, _, rings = ORACLE.rings_of_polygon(polygon)
        assert admission is ORACLE.AdmissionOutcome.ADMITTED
        weights |= {weight for ring in rings for weight in ring.weights}
    assert Fraction(3, 2) in weights
    assert Fraction(2) in weights


# --------------------------------------------------------------------------
# 4. Модель допуска
# --------------------------------------------------------------------------


def test_tolerance_is_derived_from_the_printed_digits():
    """Допуск ВЫВОДИТСЯ из числа значащих цифр печати, а не назначается.

    Поменяется печать Surfer2 — поменяется `printed_significant_digits`, и
    относительный допуск поедет за ней. Именно этим модель отличается от
    константы `1e-5`, стоявшей в спайке.
    """

    model = ORACLE.ToleranceModelV1()
    assert model.printed_significant_digits == 6
    assert model.half_ulp == pytest.approx(5e-6)
    assert model.relative == pytest.approx(1e-5)
    assert ORACLE.ToleranceModelV1(printed_significant_digits=8).relative == (
        pytest.approx(1e-7)
    )


def test_tolerance_scales_with_the_figure_and_the_floor_never_binds():
    """Абсолютного допуска не существует: он пропорционален масштабу.

    Пол объявлен сторожем вырождения, а не бюджетом, и утверждение проверяемо:
    на целочисленной решётке `scale >= 1`, значит пол не связывает никогда.
    """

    model = ORACLE.ToleranceModelV1()
    assert model.for_scale(8) == pytest.approx(8e-5)
    assert model.for_scale(4096) == pytest.approx(4.096e-2)
    assert not model.floor_binds(1)
    assert model.floor_binds(0)
    assert model.for_scale(0) == model.floor


# --------------------------------------------------------------------------
# 5. Разбор OBJ и фильтр внутренности
# --------------------------------------------------------------------------


def test_parse_obj_takes_vertices_and_ignores_faces():
    text = "v 1 2 3\n\nf 1 2 3\nv -0.5 0.25 4\nf 2 3\n"
    assert ORACLE.parse_obj_vertices(text) == ((1.0, 2.0, 3.0), (-0.5, 0.25, 4.0))


def test_parse_obj_refuses_a_truncated_vertex():
    """Обрезанная вершина — ОТКАЗ, а не молчаливый пропуск строки."""

    with pytest.raises(ValueError):
        ORACLE.parse_obj_vertices("v 1 2\n")


def test_point_in_polygon_with_holes_puts_the_hole_outside():
    """Точка в дыре лежит ВНЕ области: чётность пересечений по всем кольцам."""

    hole = ORACLE.RingV1(
        tuple(
            (Fraction(x), Fraction(y))
            for x, y in ((6, 6), (14, 6), (14, 14), (6, 14))
        ),
        (Fraction(1),) * 4,
    )
    outer = ORACLE.RingV1(
        tuple(
            (Fraction(x), Fraction(y))
            for x, y in ((0, 0), (20, 0), (20, 20), (0, 20))
        ),
        (Fraction(1),) * 4,
    )
    rings = (outer, hole)
    place = ORACLE.place_point
    assert place(3.0, 3.0, rings, 1e-6) is ORACLE.PointPlacement.INSIDE
    assert place(10.0, 10.0, rings, 1e-6) is ORACLE.PointPlacement.OUTSIDE
    assert place(25.0, 10.0, rings, 1e-6) is ORACLE.PointPlacement.OUTSIDE


def test_point_at_the_boundary_refuses_to_be_placed():
    """Ближе допуска к границе — `NEAR_BOUNDARY`, а не «наверное, внутри».

    Печать в шесть цифр не даёт права относить такую точку к какой-либо
    стороне, а тихий сброс спрятал бы расхождение в фильтре.
    """

    rings = (_square(20),)
    assert ORACLE.place_point(0.0, 10.0, rings, 1e-6) is (
        ORACLE.PointPlacement.NEAR_BOUNDARY
    )
    assert ORACLE.place_point(1e-7, 10.0, rings, 1e-6) is (
        ORACLE.PointPlacement.NEAR_BOUNDARY
    )
    assert ORACLE.place_point(0.5, 10.0, rings, 1e-6) is (
        ORACLE.PointPlacement.INSIDE
    )


def test_interior_nodes_drop_input_vertices_and_report_the_ambiguous():
    """`t = 0` — вершины ВХОДА, у нас событий с нулевым временем нет."""

    rings = (_square(20),)
    nodes = (
        {"x": 0.0, "y": 0.0, "t": 0.0},
        {"x": 10.0, "y": 10.0, "t": 10.0},
        {"x": 30.0, "y": 10.0, "t": 5.0},
        {"x": 20.0, "y": 10.0, "t": 1.0},
    )
    inside, ambiguous = ORACLE.interior_nodes(nodes, rings, 1e-6)
    assert inside == ((10.0, 10.0, 10.0),)
    assert [node["x"] for node in ambiguous] == [20.0]


# --------------------------------------------------------------------------
# 6. Паросочетание
# --------------------------------------------------------------------------


def test_match_reports_both_sides_of_a_disagreement():
    ours = ((1.0, 1.0, 1.0), (5.0, 5.0, 5.0))
    theirs = ((1.0, 1.0, 1.0 + 1e-7), (9.0, 9.0, 9.0))
    match = ORACLE.match_nodes(ours, theirs, 1e-6)
    assert [pair[:2] for pair in match.pairs] == [(0, 0)]
    assert match.kernel_only == (1,)
    assert match.surfer_only == (1,)
    assert match.max_residual == pytest.approx(1e-7)


def test_match_does_not_depend_on_the_order_of_the_input():
    """Паросочетание глобально-жадное, поэтому порядок списка ответ не решает."""

    ours = ((0.0, 0.0, 1.0), (0.0, 0.0, 1.0 + 3e-7))
    theirs = ((0.0, 0.0, 1.0 + 4e-7), (0.0, 0.0, 1.0 + 1e-7))
    forward = ORACLE.match_nodes(ours, theirs, 1e-6)
    backward = ORACLE.match_nodes(ours[::-1], theirs[::-1], 1e-6)
    assert len(forward.pairs) == len(backward.pairs) == 2
    assert forward.max_residual == pytest.approx(backward.max_residual)


# --------------------------------------------------------------------------
# 7. Наша сторона: перевод точных значений в float
# --------------------------------------------------------------------------


def test_kernel_rows_read_the_enclosure_and_not_a_float_field():
    """Перевод идёт через СТРОГУЮ оболочку: ошибка ниже 2^-100, вне бюджета."""

    exact = SimpleNamespace(
        enclosure=lambda bits: (Fraction(1, 3), Fraction(1, 3) + Fraction(1, 10**40))
    )
    node = SimpleNamespace(
        kind="EventKind.EDGE",
        point=SimpleNamespace(x=exact, y=exact),
        time=SimpleNamespace(enclosure=lambda bits: (Fraction(2), Fraction(2))),
        converging_vertices=3,
    )
    rows = ORACLE.kernel_node_rows(SimpleNamespace(nodes=(node,)))
    assert rows[0]["x"] == pytest.approx(1 / 3)
    assert rows[0]["t"] == 2.0
    assert rows[0]["converging"] == 3


# --------------------------------------------------------------------------
# 8. Обёртка процесса: заглушка вместо Surfer2
# --------------------------------------------------------------------------


@POSIX_ONLY
def test_failed_process_is_named_and_gives_no_nodes(tmp_path):
    """rc != 0 — ИМЕНОВАННЫЙ отказ и `nodes is None`, а не пустой список.

    Пустой список означал бы «оракул посчитал и узлов не нашёл», и провал
    процесса прошёл бы сверку как согласие на вырожденной фигуре.
    """

    binary = _stub(tmp_path, "import sys\nsys.stderr.write('boom\\n')\nsys.exit(3)\n")
    result = ORACLE.run_surfer(binary, ORACLE.BANNER_PROBE)
    assert result.outcome is ORACLE.OracleOutcome.SURFER2_ORACLE_PROCESS_FAILED
    assert result.rc == 3
    assert result.nodes is None
    assert "boom" in result.stderr_tail


@POSIX_ONLY
def test_empty_output_is_named_and_not_taken_for_an_answer(tmp_path):
    binary = _stub(tmp_path, "import sys\nopen(sys.argv[-1], 'w').write('')\n")
    result = ORACLE.run_surfer(binary, ORACLE.BANNER_PROBE)
    assert result.outcome is (
        ORACLE.OracleOutcome.SURFER2_ORACLE_OUTPUT_HAS_NO_VERTICES
    )
    assert result.nodes is None


@POSIX_ONLY
def test_runner_writes_input_and_reads_output_in_a_temporary_directory(tmp_path):
    """Прогон идёт во ВРЕМЕННОМ каталоге: CORE пишет `Core_Diagnostics` в CWD.

    Заглушка это и доказывает: она создаёт в своём CWD файл-маркер, а после
    прогона в дереве не остаётся ни его, ни входного GraphML.
    """

    binary = _stub(
        tmp_path,
        "import pathlib, sys\n"
        "pathlib.Path('Core_Diagnostics').mkdir()\n"
        "graphml = pathlib.Path(sys.argv[-2]).read_text()\n"
        "assert 'vertex-coordinate-x' in graphml\n"
        "pathlib.Path(sys.argv[-1]).write_text('v 1 2 3\\nv 4 5 0\\n')\n",
    )
    before = sorted(path.name for path in tmp_path.iterdir())
    result = ORACLE.run_surfer(binary, ORACLE.BANNER_PROBE)
    assert result.outcome is ORACLE.OracleOutcome.EXACT
    assert result.nodes == (
        {"x": 1.0, "y": 2.0, "t": 3.0},
        {"x": 4.0, "y": 5.0, "t": 0.0},
    )
    assert sorted(path.name for path in tmp_path.iterdir()) == before


def test_stderr_digest_ignores_colour_and_timestamps():
    """Дайджест берётся с НОРМИРОВАННОГО stderr, иначе расписка невоспроизводима.

    easyloggingpp печатает цвет и время каждой строки; сырой дайджест менялся
    бы от прогона к прогону ПО ПОСТРОЕНИЮ. Содержание нормировка не трогает.
    """

    first = "\x1b[36m133652.483 I  event#1 done.\n\x1b[0m"
    second = "\x1b[36m991111.001 I  event#1 done.\n\x1b[0m"
    assert ORACLE.normalize_stderr(first) == "I  event#1 done.\n"
    assert ORACLE.stderr_digest(first) == ORACLE.stderr_digest(second)
    assert ORACLE.stderr_digest(first) != ORACLE.stderr_digest("event#2 done.\n")


# --------------------------------------------------------------------------
# 9. Личность бинаря: подмена double-сборкой обязана быть громкой
# --------------------------------------------------------------------------


def test_missing_binary_is_named():
    identity = ORACLE.verify_binary(Path("/nonexistent/surfer"))
    assert identity.outcome is ORACLE.OracleOutcome.SURFER2_ORACLE_BINARY_IS_MISSING
    assert not identity.verified


def test_digest_mismatch_stops_the_run_before_it_starts(tmp_path):
    """Сверка личности fail-closed: чужой файл не доходит до прогона."""

    fake = tmp_path / "surfer"
    fake.write_bytes(b"not the oracle")
    identity = ORACLE.verify_binary(fake)
    assert identity.outcome is (
        ORACLE.OracleOutcome.SURFER2_ORACLE_BINARY_DIGEST_DOES_NOT_MATCH_RECEIPT
    )
    assert identity.sha256 != identity.expected_sha256


def _receipt_for(tmp_path: Path, binary: Path) -> Path:
    """Расписка с дайджестом ЗАГЛУШКИ: так сверка доходит до баннера.

    Без этого второе свидетельство было бы недостижимо в тесте: у любой
    заглушки sha256 чужой, и проверка останавливалась бы на первом.
    """

    receipt = json.loads(ORACLE.DEFAULT_RECEIPT.read_text(encoding="utf-8"))
    receipt["binary"]["sha256"] = ORACLE.file_digest(binary)
    path = tmp_path / "receipt.json"
    path.write_text(json.dumps(receipt), encoding="utf-8")
    return path


@POSIX_ONLY
def test_double_build_banner_is_refused_even_when_the_digest_matches(tmp_path):
    """Второе свидетельство ловит подмену, которую первое пропустило бы.

    Дайджест говорит «файл тот самый», баннер — «сборка та самая»: `cc/main.cpp`
    дописывает `-NT_USE-DOUBLE` при неточной сборке. Здесь дайджест СХОДИТСЯ с
    распиской, а баннер — double-сборки, и прогон обязан остановиться. Ровно
    ради этого случая баннер и проверяется: оракул на double-сборке дал бы
    «почти те же» числа и выглядел бы зелёным.
    """

    receipt = json.loads(ORACLE.DEFAULT_RECEIPT.read_text(encoding="utf-8"))
    banner = receipt["cmake"]["kernel_evidence"]["stats_version_banner"]
    binary = _stub(
        tmp_path,
        "import sys\n"
        f"sys.stderr.write({banner + '-NT_USE-DOUBLE'!r} + '\\n')\n"
        "open(sys.argv[-1], 'w').write('v 0 0 0\\n')\n",
    )
    identity = ORACLE.verify_binary(binary, _receipt_for(tmp_path, binary))
    assert identity.outcome is (
        ORACLE.OracleOutcome.SURFER2_ORACLE_BUILD_BANNER_DOES_NOT_MATCH_RECEIPT
    )
    assert identity.sha256 == identity.expected_sha256
    assert identity.banner.endswith("-NT_USE-DOUBLE")
    assert not identity.verified


@POSIX_ONLY
def test_matching_digest_and_banner_verify_the_binary(tmp_path):
    """Оба свидетельства сошлись -> прогон разрешён. Иначе тест выше доказывал
    бы лишь то, что сверка умеет отказывать всегда."""

    receipt = json.loads(ORACLE.DEFAULT_RECEIPT.read_text(encoding="utf-8"))
    banner = receipt["cmake"]["kernel_evidence"]["stats_version_banner"]
    binary = _stub(
        tmp_path,
        "import sys\n"
        f"sys.stderr.write('\\x1b[0m' + {banner!r} + '\\n')\n"
        "open(sys.argv[-1], 'w').write('v 0 0 0\\n')\n",
    )
    identity = ORACLE.verify_binary(binary, _receipt_for(tmp_path, binary))
    assert identity.outcome is ORACLE.OracleOutcome.EXACT
    assert identity.verified


def test_receipt_declares_the_exact_kernel_the_banner_proves():
    """Расписка и раннер читают ОДНО поле, а не два похожих."""

    receipt = json.loads(ORACLE.DEFAULT_RECEIPT.read_text(encoding="utf-8"))
    assert receipt["cmake"]["generated_config_h"]["NT_USE_DOUBLE"].startswith("undef")
    banner = receipt["cmake"]["kernel_evidence"]["stats_version_banner"]
    assert banner.startswith(ORACLE.BANNER_PREFIX)
    assert "NT_USE-DOUBLE" not in banner


# --------------------------------------------------------------------------
# 10. Пропуск корпусного прогона назван, а не молчалив
# --------------------------------------------------------------------------


def _corpus_run(*arguments) -> subprocess.CompletedProcess:
    return subprocess.run(
        [sys.executable, str(TOOLS / "surfer2_corpus_run.py"), *arguments],
        capture_output=True,
        text=True,
        timeout=120,
        check=False,
    )


def test_corpus_run_skips_with_a_named_outcome_when_the_binary_is_absent():
    """Нет бинаря — ИМЕНОВАННЫЙ пропуск и rc = 0. Расписка при этом НЕ пишется.

    Молчаливый пропуск сделал бы ворота бессмысленными: зелёный прогон без
    оракула выглядел бы как прогон с оракулом.
    """

    completed = _corpus_run("--binary=/nonexistent/surfer")
    assert completed.returncode == 0
    payload = json.loads(completed.stdout)
    assert payload["outcome"] == CORPUS.BINARY_IS_NOT_AVAILABLE
    assert payload["report_written"] is False


def test_corpus_run_can_be_told_that_the_binary_is_required():
    """`--require-binary` превращает пропуск в отказ — дверь fail-closed."""

    completed = _corpus_run("--binary=/nonexistent/surfer", "--require-binary")
    assert completed.returncode == 1
    assert json.loads(completed.stdout)["outcome"] == CORPUS.BINARY_IS_NOT_AVAILABLE


@POSIX_ONLY
def test_partial_selection_refuses_to_write_over_the_full_receipt(tmp_path):
    """`--case` без своего `--out` — ОТКАЗ, а не тихо урезанная расписка.

    Файл остался бы на месте, фигур в нём стало бы меньше, и ворота этого не
    увидели бы: расписка перестала бы означать «весь корпус сошёлся». Обе
    двери проверяются на заглушке — Surfer2 не запускается, потому что отказ
    наступает до первой фигуры.
    """

    receipt = json.loads(ORACLE.DEFAULT_RECEIPT.read_text(encoding="utf-8"))
    banner = receipt["cmake"]["kernel_evidence"]["stats_version_banner"]
    binary = _stub(
        tmp_path,
        "import sys\n"
        f"sys.stderr.write({banner!r} + '\\n')\n"
        "open(sys.argv[-1], 'w').write('v 0 0 0\\n')\n",
    )
    stub_receipt = _receipt_for(tmp_path, binary)
    common = (f"--binary={binary}", f"--receipt={stub_receipt}")

    partial = _corpus_run(*common, "--case=axis_square")
    assert partial.returncode == 1
    assert json.loads(partial.stdout)["outcome"] == (
        CORPUS.PARTIAL_RUN_WOULD_OVERWRITE_THE_FULL_RECEIPT
    )

    empty = _corpus_run(
        *common, "--case=no_such_figure", f"--out={tmp_path / 'partial.json'}"
    )
    assert empty.returncode == 1
    assert json.loads(empty.stdout)["outcome"] == CORPUS.CORPUS_SELECTION_IS_EMPTY
    assert not (tmp_path / "partial.json").exists()


def test_corpus_report_of_the_recorded_run_agrees_on_every_admitted_figure():
    """Расписка прогона на живом бинаре — свидетельство, а не украшение.

    Тест не запускает Surfer2: он читает то, что записал прогон, и требует от
    записи внутренней связности — исходы названы, невязка лежит в объявленном
    допуске, а фигур ровно столько, сколько в корпусе. Расхождение расписки с
    её собственной моделью допуска красит тест красным без всякого бинаря.
    """

    from wavefront_cases import named_corpus

    report_path = REPO_ROOT / "artifacts" / "surfer2_r0_spike" / "corpus_report.json"
    report = json.loads(report_path.read_text(encoding="utf-8"))
    expected = {name for name, _ in named_corpus()}
    expected |= {name for name, _ in CORPUS.weighted_corpus()}
    assert set(report["cases"]) == expected

    model = ORACLE.ToleranceModelV1(**{
        key: report["tolerance_model"][key]
        for key in ("printed_significant_digits", "safety_factor", "floor")
    })
    for name, case in report["cases"].items():
        assert case["admission"] == ORACLE.AdmissionOutcome.ADMITTED.value, name
        assert case["comparison"] == (
            ORACLE.ComparisonOutcome.NODES_AGREE_WITHIN_TOLERANCE.value
        ), name
        scale = max(case["scale_input"], case["scale_kernel"])
        assert case["tolerance_abs"] == pytest.approx(model.for_scale(scale)), name
        assert case["max_residual_abs"] <= case["tolerance_abs"], name
        assert not case["surfer_boundary_ambiguous"], name
    assert report["totals"]["cases_agreeing"] == report["totals"]["cases_total"]
    assert report["totals"]["max_residual_relative"] < model.relative
    assert report["binary_identity"]["outcome"] == ORACLE.OracleOutcome.EXACT.value
