"""Раннер оракула R0: exact-Surfer2 отвечает на тот же вход, что и наше ядро.

Спайк (`artifacts/surfer2_r0_spike/`) доказал, что внешний ответ вообще
достижим: 52/52 узла на 11 фигурах корпуса. Здесь тот же ход превращён в
воспроизводимый инструмент. Модуль — БИБЛИОТЕКА плюс тонкий CLI; прогон по
корпусу живёт отдельно (`tools/surfer2_corpus_run.py`), потому что он требует
ядра на `PYTHONPATH`, а этот файл не требует ничего, кроме stdlib.

CLEAN-ROOM. Код Surfer2 не копируется и не правится: в проекте живёт только
расписка сборки и путь к ВНЕШНЕМУ бинарю (GPL-3, `build_receipt.json`).
Всё, что здесь есть, — формат входа (GraphML по их README), формат выхода
(OBJ) и командная строка.

ЧТО ЭТОТ ОРАКУЛ ДОКАЗЫВАЕТ, А ЧТО НЕТ. Он топологически-позиционный, а не
численный: Surfer2 печатает координаты через `CGAL::to_double` без
`setprecision`, то есть ШЕСТЬ значащих цифр, независимо от того, что внутри
у него точное ядро CORE::Expr. Совпадение узлов в допуске ЧТЕНИЯ означает
«те же события в тех же местах», а не «те же числа». Поэтому модель допуска
объявлена нормативно (`ToleranceModelV1`), выводится из числа печатаемых
цифр и печатается в расписке вместе с abs- и rel-невязкой — а не задана
константой, которую можно тихо подкрутить под неудобный случай.

ПЕРЕСЕЧЕНИЕ МОДЕЛЕЙ. Вход у нас и у них РАЗНЫЙ, и это не деталь:

* наш вход — область, фронт идёт ВНУТРЬ; их вход — PSLG, фронт идёт с обеих
  сторон каждого ребра. Поэтому выход фильтруется геометрически
  (`place_point`), а не по номеру компоненты: номер зависит от порядка рёбер
  во входе (`BasicTriangulation::tag_components` сеет компоненту 0 слева от
  ПЕРВОГО ребра списка), и опираться на него значило бы опираться на
  недокументированную деталь. Номер компоненты остаётся вторым, независимым
  способом — как контроль методики, а не как источник ответа;
* вес ребра у Surfer2 — ЕВКЛИДОВА скорость (`WavefrontSupportingLine`:
  `normal = normal_unit * weight`). У нас скорость задана числом
  `q = |n|^2 * s^2`, то есть `s = sqrt(q)/|d|`. Рациональное `s` передаётся
  СТРОКОЙ и принимается лосслесс (`string_to_maybe_NT(x) = x` при точном
  ядре, CORE::Expr читает «p/q»). Иррациональное `s` не передаётся никак:
  это не наша уступка, а граница области — она названа
  `SURFER2_MODEL_HAS_NO_IRRATIONAL_EDGE_SPEED`, и весовой дифференциал на
  таких входах остаётся in-house (P0-3);
* стена (`q = 0`) и веер вогнутой вершины (рёбра НУЛЕВОЙ длины) во входе
  Surfer2 не выразимы вовсе — тоже именованные не-допуски, а не тихий
  пропуск.

ПОДМЕНА БИНАРЯ. Оракул на double-сборке молча дал бы «почти те же» числа и
выглядел бы зелёным. Поэтому перед прогоном сверяются ДВА независимых
свидетельства: sha256 файла и баннер `--stats-fd`. Баннер — не украшение:
`cc/main.cpp` дописывает к нему `-NT_USE-DOUBLE` при неточной сборке, то
есть равенство баннера расписке и есть доказательство точного режима.

CWD. Surfer2 (через CORE) пишет каталог `Core_Diagnostics` в ТЕКУЩИЙ
каталог. Раннер поэтому работает во временном каталоге и не сорит в дерево.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import re
import subprocess
import sys
import tempfile
from dataclasses import dataclass
from enum import Enum
from fractions import Fraction
from math import isqrt
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_RECEIPT = (
    REPO_ROOT / "artifacts" / "surfer2_r0_spike" / "build_receipt.json"
)

#: Таймаут одного прогона. Свой, а не «сколько получится»: зависший оракул
#: обязан стать именованным отказом, а не вечно занятым раннером.
DEFAULT_TIMEOUT_SECONDS = 300

#: Цвет и отметки времени easyloggingpp. Снимаются и при поиске баннера, и при
#: дайджесте stderr — по одной причине: это оформление, а не содержание.
_ANSI = re.compile(r"\x1b\[[0-9;]*m")
_LOG_TIMESTAMP = re.compile(r"^\s*\d+\.\d+\s+", re.MULTILINE)


class OracleOutcome(str, Enum):
    """Чем кончился разговор с внешним бинарём. Тихого выхода нет ни одного."""

    EXACT = "EXACT"
    SURFER2_ORACLE_BINARY_IS_MISSING = "SURFER2_ORACLE_BINARY_IS_MISSING"
    SURFER2_ORACLE_BINARY_DIGEST_DOES_NOT_MATCH_RECEIPT = (
        "SURFER2_ORACLE_BINARY_DIGEST_DOES_NOT_MATCH_RECEIPT"
    )
    SURFER2_ORACLE_BUILD_BANNER_DOES_NOT_MATCH_RECEIPT = (
        "SURFER2_ORACLE_BUILD_BANNER_DOES_NOT_MATCH_RECEIPT"
    )
    #: rc != 0 либо таймаут. Пустой список узлов НЕ выдаётся: отсутствие ответа
    #: и ответ «узлов нет» — разные вещи, и путать их значит терять отказ.
    SURFER2_ORACLE_PROCESS_FAILED = "SURFER2_ORACLE_PROCESS_FAILED"
    SURFER2_ORACLE_OUTPUT_IS_NOT_OBJ = "SURFER2_ORACLE_OUTPUT_IS_NOT_OBJ"
    SURFER2_ORACLE_OUTPUT_HAS_NO_VERTICES = (
        "SURFER2_ORACLE_OUTPUT_HAS_NO_VERTICES"
    )


class AdmissionOutcome(str, Enum):
    """Лежит ли вход в ОБЪЯВЛЕННОМ пересечении моделей."""

    ADMITTED = "ADMITTED"
    SURFER2_MODEL_HAS_NO_WALL_EDGE = "SURFER2_MODEL_HAS_NO_WALL_EDGE"
    SURFER2_MODEL_HAS_NO_VERTEX_FAN = "SURFER2_MODEL_HAS_NO_VERTEX_FAN"
    SURFER2_MODEL_HAS_NO_IRRATIONAL_EDGE_SPEED = (
        "SURFER2_MODEL_HAS_NO_IRRATIONAL_EDGE_SPEED"
    )


class ComparisonOutcome(str, Enum):
    """Чем кончилась сверка узлов ядра с узлами оракула."""

    NODES_AGREE_WITHIN_TOLERANCE = "NODES_AGREE_WITHIN_TOLERANCE"
    KERNEL_SKELETON_IS_NOT_EXACT = "KERNEL_SKELETON_IS_NOT_EXACT"
    ORACLE_DID_NOT_ANSWER = "ORACLE_DID_NOT_ANSWER"
    NODES_DISAGREE = "NODES_DISAGREE"
    #: Узел оракула лёг ближе допуска к границе входа: «строго внутри» на
    #: печатанных числах не решается. Отбрасывать его молча значило бы прятать
    #: расхождение в фильтре.
    INTERIOR_FILTER_IS_AMBIGUOUS = "INTERIOR_FILTER_IS_AMBIGUOUS"


class PointPlacement(str, Enum):
    """Где лежит напечатанная точка относительно области входа."""

    INSIDE = "INSIDE"
    OUTSIDE = "OUTSIDE"
    NEAR_BOUNDARY = "NEAR_BOUNDARY"


# --------------------------------------------------------------------------
# Модель допуска
# --------------------------------------------------------------------------


@dataclass(frozen=True, slots=True)
class ToleranceModelV1:
    """Допуск ЧТЕНИЯ чужой печати. Выводится, а не назначается.

    `SkeletonDCEL::write_obj` печатает `CGAL::to_double(p.x())` в поток без
    `setprecision`, то есть на дефолтной точности `ostream` — ШЕСТЬ значащих
    цифр. Значит абсолютного допуска не существует в принципе: невязка печати
    пропорциональна модулю печатаемого числа. Отсюда вся модель:

        half_ulp = 5 * 10^-digits          (половина последнего разряда)
        relative = safety_factor * half_ulp
        tol(scale) = max(floor, relative * scale)

    При `digits = 6` это `relative = 1e-5` — ровно то число, что стояло в
    спайке константой. Разница в том, что теперь оно ВЫВЕДЕНО: поменяется
    печать Surfer2 — поменяется `digits`, и допуск поедет за ней сам.

    `floor` — не бюджет, а сторож вырождения `scale -> 0`. На целочисленной
    решётке `scale >= 1`, поэтому `relative * scale >= 1e-5` и пол не связывает
    НИКОГДА; расписка это печатает (`floor_binds`), чтобы утверждение можно было
    проверить, а не принять на слово. Перевод нашей стороны в float идёт через
    строгую оболочку (`enclosure(160)`), его ошибка ниже 2^-100 и в бюджет не
    входит.
    """

    printed_significant_digits: int = 6
    #: Запас над половиной последнего разряда. Двойка, а не «на глаз»:
    #: сравниваются ДВА независимо напечатанных числа только в одну сторону
    #: (наша сторона точна), но `scale` берётся по фигуре целиком, а не по
    #: конкретной координате, поэтому у координат меньшего порядка запас съеден
    #: масштабом. Двойка покрывает этот перекос, не пряча расхождений: она в
    #: 10^5 раз меньше самого мелкого расстояния между узлами корпуса.
    safety_factor: int = 2
    floor: float = 1e-9

    @property
    def half_ulp(self) -> float:
        return 5.0 * 10.0 ** (-self.printed_significant_digits)

    @property
    def relative(self) -> float:
        return self.safety_factor * self.half_ulp

    def for_scale(self, scale: float) -> float:
        return max(self.floor, self.relative * abs(float(scale)))

    def floor_binds(self, scale: float) -> bool:
        return self.floor > self.relative * abs(float(scale))

    def payload(self) -> dict:
        return {
            "printed_significant_digits": self.printed_significant_digits,
            "safety_factor": self.safety_factor,
            "half_ulp": self.half_ulp,
            "relative": self.relative,
            "floor": self.floor,
            "law": "tol(scale) = max(floor, safety_factor * 5e-digits * scale)",
            "basis": (
                "SkeletonDCEL::write_obj печатает CGAL::to_double без "
                "setprecision -> 6 значащих цифр; невязка печати "
                "пропорциональна модулю числа, поэтому абсолютного допуска "
                "не существует"
            ),
        }


# --------------------------------------------------------------------------
# Вход: рациональные числа строками, многокольцевой GraphML
# --------------------------------------------------------------------------


GRAPHML_HEADER = (
    '<graphml xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" '
    'xmlns="http://graphml.graphdrawing.org/xmlns" '
    'xsi:schemaLocation="http://graphml.graphdrawing.org/xmlns '
    'http://graphml.graphdrawing.org/xmlns/1.0/graphml.xsd">\n'
    '  <key attr.name="vertex-coordinate-x" attr.type="string" for="node" id="x"/>\n'
    '  <key attr.name="vertex-coordinate-y" attr.type="string" for="node" id="y"/>\n'
    '  <key attr.name="edge-weight" attr.type="string" for="edge" id="w"/>\n'
    '  <graph edgedefault="undirected">\n'
)


def rational_text(value: int | Fraction) -> str:
    """Рациональное число СТРОКОЙ, без потери разряда.

    Атрибуты объявлены `attr.type="string"` именно затем: при точном ядре
    Surfer2 делает `string_to_maybe_NT(x) = x`, то есть строит CORE::Expr прямо
    из текста и принимает форму «p/q». Пройди число через `double`, лосслесс
    кончился бы на входе, и «точный оракул» стал бы точным только внутри.
    """

    fraction = Fraction(value)
    if fraction.denominator == 1:
        return str(fraction.numerator)
    return f"{fraction.numerator}/{fraction.denominator}"


@dataclass(frozen=True, slots=True)
class RingV1:
    """Одно кольцо входа: точки и ЕВКЛИДОВЫ скорости рёбер (веса Surfer2).

    `weights[i]` — вес ребра `points[i] -> points[i+1]`. Кольцо здесь плоское:
    внешнее оно или дыра, для PSLG-входа безразлично (граф неориентированный),
    а для фильтра внутренности достаточно чётности пересечений по ВСЕМ кольцам.
    """

    points: tuple[tuple[Fraction, Fraction], ...]
    weights: tuple[Fraction, ...]


def graphml_document(rings: tuple[RingV1, ...]) -> str:
    """GraphML-вход Surfer2 из колец. Порядок узлов и рёбер воспроизводим.

    Кольца идут подряд, первым — то, что подано первым (у нас внешнее).
    Порядок значим ровно в одном месте: `tag_components` сеет компоненту 0
    слева от ПЕРВОГО ребра, поэтому у CCW-внешнего кольца компонента 0 есть
    внутренность области. Раннер на этом не строит ответ, но печатает согласие
    двух способов выделения как контроль методики.
    """

    parts = [GRAPHML_HEADER]
    index = 0
    edges: list[tuple[int, int, Fraction]] = []
    for ring in rings:
        base = index
        for x, y in ring.points:
            parts.append(
                f'    <node id="{index}">\n'
                f'      <data key="x">{rational_text(x)}</data>\n'
                f'      <data key="y">{rational_text(y)}</data>\n'
                f"    </node>\n"
            )
            index += 1
        size = len(ring.points)
        for offset in range(size):
            edges.append(
                (base + offset, base + (offset + 1) % size, ring.weights[offset])
            )
    for source, target, weight in edges:
        parts.append(
            f'    <edge source="{source}" target="{target}">\n'
            f'      <data key="w">{rational_text(weight)}</data>\n'
            f"    </edge>\n"
        )
    parts.append("  </graph>\n</graphml>\n")
    return "".join(parts)


def rational_square_root(value: Fraction) -> Fraction | None:
    """`sqrt(value)` рационален -> он сам; иначе `None`.

    Целочисленный предикат, не порог: числитель и знаменатель несократимой
    дроби обязаны быть полными квадратами по отдельности.
    """

    fraction = Fraction(value)
    if fraction < 0:
        return None
    numerator_root = isqrt(fraction.numerator)
    denominator_root = isqrt(fraction.denominator)
    if numerator_root**2 != fraction.numerator:
        return None
    if denominator_root**2 != fraction.denominator:
        return None
    return Fraction(numerator_root, denominator_root)


def surfer_edge_weight(
    speed_squared: int | Fraction,
    start: tuple[int, int],
    end: tuple[int, int],
) -> Fraction | None:
    """Вес ребра Surfer2 из нашего `q`. `None` — вес иррационален.

    У нас `q` — квадрат скорости В ЕДИНИЦАХ НОРМАЛИ длины `|d|`, у них вес —
    евклидова скорость. Отсюда `w = sqrt(q)/|d| = sqrt(q/|d|^2)`, и вопрос
    «передаётся ли вход» сводится к рациональности одного корня.
    """

    dx = end[0] - start[0]
    dy = end[1] - start[1]
    return rational_square_root(Fraction(speed_squared) / (dx * dx + dy * dy))


def rings_of_polygon(polygon) -> tuple[AdmissionOutcome, str, tuple[RingV1, ...]]:
    """`PolygonV1` -> кольца входа Surfer2 либо ИМЕНОВАННЫЙ не-допуск.

    Отсев идёт до всякого счёта и целиком: вход либо лежит в объявленном
    пересечении моделей, либо назван причиной, по которой не лежит. Третьего
    («ну мы его как-нибудь приблизим») нет.
    """

    if getattr(polygon, "vertex_fans", ()):
        return (
            AdmissionOutcome.SURFER2_MODEL_HAS_NO_VERTEX_FAN,
            f"{len(polygon.vertex_fans)} вееров: рёбра нулевой длины во "
            "входе GraphML не выразимы (узел не повторяется)",
            (),
        )
    rings: list[RingV1] = []
    for loop in polygon.loops:
        points = loop.points
        speeds = loop.edge_speeds_squared
        weights: list[Fraction] = []
        for index in range(len(points)):
            start = points[index]
            end = points[(index + 1) % len(points)]
            if speeds[index] == 0:
                return (
                    AdmissionOutcome.SURFER2_MODEL_HAS_NO_WALL_EDGE,
                    f"ребро {start}->{end} есть стена (q = 0); у Surfer2 фронт "
                    "идёт с ОБЕИХ сторон каждого ребра, неподвижного ребра нет",
                    (),
                )
            weight = surfer_edge_weight(speeds[index], start, end)
            if weight is None:
                return (
                    AdmissionOutcome.SURFER2_MODEL_HAS_NO_IRRATIONAL_EDGE_SPEED,
                    f"ребро {start}->{end}: q = {speeds[index]}, "
                    "скорость sqrt(q)/|d| иррациональна и строкой не передаётся",
                    (),
                )
            weights.append(weight)
        rings.append(
            RingV1(
                tuple((Fraction(x), Fraction(y)) for x, y in points),
                tuple(weights),
            )
        )
    return AdmissionOutcome.ADMITTED, "", tuple(rings)


# --------------------------------------------------------------------------
# Сверка личности бинаря
# --------------------------------------------------------------------------


BANNER_PREFIX = "[SURF] VERSION"
#: Минимальный вход для баннерного прогона: квадрат единичного веса. Он свой,
#: а не «первая фигура корпуса», чтобы сверка личности не зависела от ядра.
BANNER_PROBE = graphml_document(
    (
        RingV1(
            tuple(
                (Fraction(x), Fraction(y))
                for x, y in ((0, 0), (4, 0), (4, 4), (0, 4))
            ),
            (Fraction(1),) * 4,
        ),
    )
)


def file_digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _collapsed(text: str) -> str:
    """Схлопнутые пробелы: баннер выровнен пробелами, и это не содержание."""

    return " ".join(text.split())


@dataclass(frozen=True, slots=True)
class BinaryIdentityV1:
    """Личность внешнего бинаря: два независимых свидетельства.

    Дайджест говорит «файл тот самый», баннер — «сборка та самая». Ни одно из
    них не заменяет другое: пересобранный из тех же исходников бинарь даст
    другой sha256 (и это законный случай), а подменённый double-сборкой даст
    баннер с суффиксом `-NT_USE-DOUBLE`.
    """

    outcome: OracleOutcome
    path: str
    sha256: str | None
    banner: str | None
    expected_sha256: str
    expected_banner: str
    detail: str = ""

    @property
    def verified(self) -> bool:
        return self.outcome is OracleOutcome.EXACT

    def payload(self) -> dict:
        return {
            "outcome": self.outcome.value,
            "path": self.path,
            "sha256": self.sha256,
            "sha256_expected": self.expected_sha256,
            "banner": self.banner,
            "banner_expected": self.expected_banner,
            "detail": self.detail,
        }


def _receipt_expectations(receipt_path: Path) -> tuple[str, str]:
    receipt = json.loads(receipt_path.read_text(encoding="utf-8"))
    return (
        receipt["binary"]["sha256"],
        receipt["cmake"]["kernel_evidence"]["stats_version_banner"],
    )


def probe_banner(binary: Path, timeout: int = DEFAULT_TIMEOUT_SECONDS) -> tuple[int, str]:
    """Строка `[SURF] VERSION` из `--stats-fd=2`. Прогон в tempdir, как все.

    Цвет снимается ДО поиска строки: easyloggingpp пишет свой лог в тот же
    дескриптор и оставляет перед баннером хвост сброса `ESC[0m`, поэтому
    поиск по началу строки на сыром stderr не находил ничего и объявлял
    подмену сборки там, где сборка была своя.
    """

    with tempfile.TemporaryDirectory(prefix="surfer2-banner-") as work:
        root = Path(work)
        (root / "probe.graphml").write_text(BANNER_PROBE, encoding="utf-8")
        completed = subprocess.run(
            [
                str(binary),
                "--stats-fd=2",
                "--component=-1",
                "probe.graphml",
                "probe.obj",
            ],
            cwd=root,
            capture_output=True,
            text=True,
            timeout=timeout,
            check=False,
        )
    for line in _ANSI.sub("", completed.stderr).splitlines():
        if line.startswith(BANNER_PREFIX):
            return completed.returncode, line
    return completed.returncode, ""


def verify_binary(
    binary: Path, receipt_path: Path = DEFAULT_RECEIPT
) -> BinaryIdentityV1:
    """Личность бинаря против расписки сборки. Fail-closed до всякого прогона."""

    binary = Path(binary)
    expected_sha, expected_banner = _receipt_expectations(Path(receipt_path))
    if not binary.is_file():
        return BinaryIdentityV1(
            OracleOutcome.SURFER2_ORACLE_BINARY_IS_MISSING,
            str(binary),
            None,
            None,
            expected_sha,
            expected_banner,
            "файла нет",
        )
    digest = file_digest(binary)
    if digest != expected_sha:
        return BinaryIdentityV1(
            OracleOutcome.SURFER2_ORACLE_BINARY_DIGEST_DOES_NOT_MATCH_RECEIPT,
            str(binary),
            digest,
            None,
            expected_sha,
            expected_banner,
            "sha256 расходится с build_receipt.json",
        )
    returncode, banner = probe_banner(binary)
    if returncode != 0 or _collapsed(banner) != _collapsed(expected_banner):
        return BinaryIdentityV1(
            OracleOutcome.SURFER2_ORACLE_BUILD_BANNER_DOES_NOT_MATCH_RECEIPT,
            str(binary),
            digest,
            banner,
            expected_sha,
            expected_banner,
            f"баннерный прогон rc={returncode}; суффикс -NT_USE-DOUBLE "
            "означал бы неточную сборку",
        )
    return BinaryIdentityV1(
        OracleOutcome.EXACT,
        str(binary),
        digest,
        banner,
        expected_sha,
        expected_banner,
    )


# --------------------------------------------------------------------------
# Выход: OBJ, дайджест stderr, JSON-конверт
# --------------------------------------------------------------------------


def normalize_stderr(text: str) -> str:
    """stderr без цвета и без отметок времени.

    Нормировка объявлена, а не скрыта: easyloggingpp печатает ANSI-цвет и
    время каждой строки, поэтому сырой дайджест менялся бы от прогона к
    прогону, и расписка перестала бы быть воспроизводимой ПО ПОСТРОЕНИЮ.
    Содержание сообщений нормировка не трогает.
    """

    return _LOG_TIMESTAMP.sub("", _ANSI.sub("", text))


def stderr_digest(text: str) -> str:
    return hashlib.sha256(normalize_stderr(text).encode("utf-8")).hexdigest()


def parse_obj_vertices(text: str) -> tuple[tuple[float, float, float], ...]:
    """Вершины OBJ. Строка `v x y z`, всё прочее (грани, пустые) — мимо.

    `z` у Surfer2 — ВРЕМЯ события: скелет пишется как поверхность в
    пространстве-времени, третья координата есть `t`.
    """

    vertices: list[tuple[float, float, float]] = []
    for line in text.splitlines():
        parts = line.split()
        if not parts or parts[0] != "v":
            continue
        if len(parts) < 4:
            raise ValueError(f"вершина OBJ без трёх координат: {line!r}")
        vertices.append((float(parts[1]), float(parts[2]), float(parts[3])))
    return tuple(vertices)


@dataclass(frozen=True, slots=True)
class OracleResultV1:
    """Ответ оракула на ОДИН вход. `nodes is None` — ответа не было.

    Разница между `None` и `()` здесь несущая: пустой список означал бы
    «оракул посчитал и узлов не нашёл», и провал процесса, выданный пустым
    списком, прошёл бы сверку как согласие на вырожденной фигуре.
    """

    outcome: OracleOutcome
    rc: int | None
    nodes: tuple[dict, ...] | None
    stderr_digest: str
    stderr_tail: str
    detail: str = ""

    def payload(self) -> dict:
        return {
            "outcome": self.outcome.value,
            "rc": self.rc,
            "nodes": None if self.nodes is None else list(self.nodes),
            "stderr_digest": self.stderr_digest,
            "stderr_tail": self.stderr_tail,
            "detail": self.detail,
        }


def _failed(
    outcome: OracleOutcome, rc: int | None, err: str, detail: str
) -> OracleResultV1:
    return OracleResultV1(
        outcome, rc, None, stderr_digest(err), normalize_stderr(err)[-400:], detail
    )


def run_surfer(
    binary: Path,
    graphml: str,
    *,
    component: str = "-1",
    timeout: int = DEFAULT_TIMEOUT_SECONDS,
) -> OracleResultV1:
    """GraphML -> Surfer2 -> узлы `{x, y, t}`. Работа во ВРЕМЕННОМ каталоге.

    Каталог временный не ради чистоты: CORE пишет `Core_Diagnostics` в CWD, и
    прогон из дерева репозитория оставлял бы там мусор от каждого запуска.
    """

    with tempfile.TemporaryDirectory(prefix="surfer2-oracle-") as work:
        root = Path(work)
        (root / "input.graphml").write_text(graphml, encoding="utf-8")
        try:
            completed = subprocess.run(
                [
                    str(binary),
                    f"--component={component}",
                    "input.graphml",
                    "output.obj",
                ],
                cwd=root,
                capture_output=True,
                text=True,
                timeout=timeout,
                check=False,
            )
        except subprocess.TimeoutExpired:
            return _failed(
                OracleOutcome.SURFER2_ORACLE_PROCESS_FAILED,
                None,
                "",
                f"таймаут {timeout} с",
            )
        except OSError as error:
            return _failed(
                OracleOutcome.SURFER2_ORACLE_BINARY_IS_MISSING,
                None,
                "",
                str(error),
            )
        if completed.returncode != 0:
            return _failed(
                OracleOutcome.SURFER2_ORACLE_PROCESS_FAILED,
                completed.returncode,
                completed.stderr,
                f"surfer --component={component} завершился с rc="
                f"{completed.returncode}",
            )
        output = root / "output.obj"
        text = output.read_text(encoding="utf-8") if output.is_file() else ""
        try:
            vertices = parse_obj_vertices(text)
        except ValueError as error:
            return _failed(
                OracleOutcome.SURFER2_ORACLE_OUTPUT_IS_NOT_OBJ,
                completed.returncode,
                completed.stderr,
                str(error),
            )
        if not vertices:
            return _failed(
                OracleOutcome.SURFER2_ORACLE_OUTPUT_HAS_NO_VERTICES,
                completed.returncode,
                completed.stderr,
                "OBJ без единой вершины",
            )
        return OracleResultV1(
            OracleOutcome.EXACT,
            completed.returncode,
            tuple({"x": x, "y": y, "t": t} for x, y, t in vertices),
            stderr_digest(completed.stderr),
            normalize_stderr(completed.stderr)[-400:],
        )


# --------------------------------------------------------------------------
# Фильтр внутренности: точная арифметика на печатанных числах
# --------------------------------------------------------------------------


def _segment_distance_squared(
    point: tuple[Fraction, Fraction],
    start: tuple[Fraction, Fraction],
    end: tuple[Fraction, Fraction],
) -> Fraction:
    """Квадрат расстояния до отрезка. Точный: ни корня, ни деления вслепую."""

    vx, vy = end[0] - start[0], end[1] - start[1]
    wx, wy = point[0] - start[0], point[1] - start[1]
    along = wx * vx + wy * vy
    if along <= 0:
        return wx * wx + wy * wy
    length_squared = vx * vx + vy * vy
    if along >= length_squared:
        ex, ey = point[0] - end[0], point[1] - end[1]
        return ex * ex + ey * ey
    return wx * wx + wy * wy - along * along / length_squared


def place_point(
    x: float, y: float, rings: tuple[RingV1, ...], tolerance: float
) -> PointPlacement:
    """Где лежит НАПЕЧАТАННАЯ точка: внутри области, вне её, либо у границы.

    Считается на точных дробях (`Fraction(float)` — двоичное значение как оно
    есть, без `limit_denominator` и без порога внутри самого предиката), а
    чётность пересечений берётся по ВСЕМ кольцам сразу: точка в дыре пересекает
    внешнее кольцо и кольцо дыры, чётность возвращается к нулю, и дыра
    оказывается снаружи области — ровно то, чего от неё и ждут.

    `NEAR_BOUNDARY` — не «почти внутри», а ОТКАЗ решать: печать в 6 цифр не даёт
    права относить точку к какой-либо стороне, если она ближе допуска к границе.
    Такой узел уходит в расписку именованной находкой, а не отбрасывается.
    """

    px, py = Fraction(x), Fraction(y)
    limit = Fraction(tolerance) ** 2
    inside = False
    for ring in rings:
        points = ring.points
        size = len(points)
        for index in range(size):
            start = points[index]
            end = points[(index + 1) % size]
            if _segment_distance_squared((px, py), start, end) <= limit:
                return PointPlacement.NEAR_BOUNDARY
            if (start[1] > py) != (end[1] > py):
                crossing = start[0] + (py - start[1]) * (end[0] - start[0]) / (
                    end[1] - start[1]
                )
                if px < crossing:
                    inside = not inside
    return PointPlacement.INSIDE if inside else PointPlacement.OUTSIDE


def interior_nodes(
    nodes: tuple[dict, ...], rings: tuple[RingV1, ...], tolerance: float
) -> tuple[tuple[tuple[float, float, float], ...], tuple[dict, ...]]:
    """Узлы-СОБЫТИЯ внутри области и отдельно — неразрешимые у границы.

    `t = 0` отбрасывается с обеих сторон: Surfer2 кладёт в DCEL и вершины
    ВХОДА, а у нас `sk.nodes` — только события. Сравниваются события.
    """

    inside: list[tuple[float, float, float]] = []
    ambiguous: list[dict] = []
    for node in nodes:
        if node["t"] <= 0:
            continue
        placement = place_point(node["x"], node["y"], rings, tolerance)
        if placement is PointPlacement.INSIDE:
            inside.append((node["x"], node["y"], node["t"]))
        elif placement is PointPlacement.NEAR_BOUNDARY:
            ambiguous.append(dict(node))
    return tuple(sorted(inside)), tuple(ambiguous)


# --------------------------------------------------------------------------
# Сверка: паросочетание и невязки
# --------------------------------------------------------------------------


@dataclass(frozen=True, slots=True)
class MatchV1:
    """Паросочетание узлов. `residual` — max по |dx|, |dy|, |dt| пары."""

    pairs: tuple[tuple[int, int, float], ...]
    kernel_only: tuple[int, ...]
    surfer_only: tuple[int, ...]
    max_residual: float


def _residual(a: tuple[float, ...], b: tuple[float, ...]) -> float:
    return max(abs(a[0] - b[0]), abs(a[1] - b[1]), abs(a[2] - b[2]))


def match_nodes(ours, theirs, tolerance: float) -> MatchV1:
    """Жадное паросочетание ГЛОБАЛЬНО по возрастанию невязки.

    Глобальное, а не «для каждого нашего узла ближайший чужой»: у второго
    порядок обхода решает исход на кратных узлах, то есть ответ зависел бы от
    того, каким пришёл список. Здесь пары берутся по возрастанию невязки, и
    результат от порядка входа не зависит.
    """

    candidates = sorted(
        (_residual(a, b), i, j)
        for i, a in enumerate(ours)
        for j, b in enumerate(theirs)
    )
    used_ours: set[int] = set()
    used_theirs: set[int] = set()
    pairs: list[tuple[int, int, float]] = []
    for distance, i, j in candidates:
        if distance > tolerance:
            break
        if i in used_ours or j in used_theirs:
            continue
        used_ours.add(i)
        used_theirs.add(j)
        pairs.append((i, j, distance))
    return MatchV1(
        tuple(sorted(pairs)),
        tuple(i for i in range(len(ours)) if i not in used_ours),
        tuple(j for j in range(len(theirs)) if j not in used_theirs),
        max((d for _, _, d in pairs), default=0.0),
    )


def enclosure_float(value, bits: int = 160) -> float:
    """Середина строгой оболочки. Ошибка перевода ниже 2^-100, вне бюджета."""

    low, high = value.enclosure(bits)
    return float((low + high) / 2)


def kernel_node_rows(skeleton, bits: int = 160) -> tuple[dict, ...]:
    """Узлы нашего скелета в float. Ядро здесь НЕ импортируется.

    Функция говорит с объектами через `enclosure`, поэтому живёт в модуле,
    которому не нужен `PYTHONPATH` ядра, и проверяется без него же.
    """

    rows = []
    for node in skeleton.nodes:
        low, high = node.time.enclosure(bits)
        rows.append(
            {
                "kind": str(node.kind),
                "x": enclosure_float(node.point.x, bits),
                "y": enclosure_float(node.point.y, bits),
                "t": float((low + high) / 2),
                "converging": node.converging_vertices,
            }
        )
    return tuple(rows)


# --------------------------------------------------------------------------
# CLI: один GraphML -> JSON-конверт
# --------------------------------------------------------------------------


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Прогнать GraphML через exact-Surfer2 и напечатать JSON-конверт "
            "{outcome, rc, nodes, stderr_digest}."
        )
    )
    parser.add_argument("graphml", type=Path)
    parser.add_argument("--binary", type=Path, required=True)
    parser.add_argument("--receipt", type=Path, default=DEFAULT_RECEIPT)
    parser.add_argument("--component", default="-1")
    arguments = parser.parse_args(argv)

    identity = verify_binary(arguments.binary, arguments.receipt)
    if not identity.verified:
        print(json.dumps({"binary_identity": identity.payload()}, indent=2,
                         ensure_ascii=False))
        return 1
    result = run_surfer(
        arguments.binary,
        arguments.graphml.read_text(encoding="utf-8"),
        component=arguments.component,
    )
    print(
        json.dumps(
            {"binary_identity": identity.payload(), "oracle": result.payload()},
            indent=2,
            ensure_ascii=False,
        )
    )
    return 0 if result.outcome is OracleOutcome.EXACT else 1


if __name__ == "__main__":
    sys.exit(main())
