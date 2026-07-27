"""Дифференциальный стенд: очередь событий против попарного кроя, один вход.

Срез НЕ переключает конвейер. Стенд ничего не подключает и ничего не двигает —
он считает покрытие ДВУМЯ путями на одном и том же входе и сравнивает точно.

    путь A, попарный крой   compile_reference_envelopes -> RawCoverage ->
                            resolve_coverage_interactions -> apply_policy_b
    путь B, очередь         build_skeleton -> build_faces -> coverage_at(alpha)

ЧТО СРАВНИВАЕТСЯ И ПОЧЕМУ ИМЕННО ЭТО. Площадь, точная. Причина не в удобстве:
именно площадь проверяет сам попарный крой в `interactions/policy_b.py:970`, и
именно её расхождение выдаёт `INTERACTION_POLICY_B_PARTITION_UNPROVEN`
(«Policy B clipping did not reproduce the exact RawCoverage set»). Задавая
очереди тот же вопрос тем же предикатом (`reference.planar_types.exact_sign`,
которым пользуется и сам крой), стенд сравнивает не два мнения, а два ответа на
один вопрос. Число регионов и петель идёт второй колонкой: площадь может
совпасть при разном разбиении, и это надо видеть.

Сравнение ТОЧНОЕ. Порогов нет: величины очереди — суммы корней с рациональными
коэффициентами, величины кроя — выражения SymPy, и знак разности берётся точным
предикатом, а не сравнением чисел.

Кеши `squarefree_split`/`prime_support` сбрасываются между точками: бенчмарк,
переиспользующий их, мерил бы кеш, а не работу.

Запуск:

    PYTHONPATH=kernel/src:kernel/tests:tools python3 tools/wavefront_differential.py
    PYTHONPATH=kernel/src:kernel/tests:tools python3 tools/wavefront_differential.py \
        --cases axis_square,right_triangle --alphas 1,2
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from fractions import Fraction
from pathlib import Path
import sys
import time

_ROOT = Path(__file__).resolve().parents[1]
for _extra in (_ROOT / "kernel" / "src", _ROOT / "kernel" / "tests"):
    if str(_extra) not in sys.path:
        sys.path.insert(0, str(_extra))

import sympy as sp

from cftuv_envelope import (
    compile_reference_envelopes,
    evaluate_reference_raw_coverage,
    resolve_coverage_interactions,
)
from cftuv_envelope.reference.planar_types import ExactScalar, exact_sign
from cftuv_envelope.wavefront import build_skeleton
from cftuv_envelope.wavefront.coverage import CoverageOutcome, coverage_at
from cftuv_envelope.wavefront.faces import FaceOutcome, build_faces
from cftuv_envelope.wavefront import sqrt_sum as sqrt_sum_module
from cftuv_envelope.wavefront.sqrt_sum import SqrtSumV1

from reference_factories import straight_snapshot
from wavefront_cases import named_corpus


class BenchOutcome(str):
    """Исходы стенда — строки, чтобы попадать в таблицу без перевода."""


# Стенд отказывается сам, а не подсовывает нули. Каждая строка — причина.
BENCH_HOLE_NOT_ONE_FACE = "HOLE_IS_NOT_ONE_SOURCE_FACE"
# `SUPPORT_LINE_SHARED_BY_SEVERAL_EDGES` отсюда УБРАН, а не переименован: стенд
# отсекал фигуру заранее, потому что сборщик граней отказывал на коллинеарных
# рёбрах. Ключ участника стал вхождением ребра, отказ исчез, и заранее отсекать
# нечего. Если сборщик всё же откажет, стенд назовёт исход через
# `BENCH_QUEUE_REFUSED` — то есть отказ по-прежнему громкий, просто не свой.
BENCH_QUEUE_REFUSED = "QUEUE_REFUSED"
BENCH_PAIRWISE_COMPILE_REFUSED = "PAIRWISE_COMPILE_REFUSED"
BENCH_PAIRWISE_RAW_REFUSED = "PAIRWISE_RAW_REFUSED"
BENCH_PAIRWISE_RAISED = "PAIRWISE_RAISED"
BENCH_EXACT = "EXACT"


def reset_caches() -> None:
    """Свежие значения: между точками кеши радикалов сбрасываются."""

    sqrt_sum_module.squarefree_split.cache_clear()
    sqrt_sum_module.prime_support.cache_clear()
    sqrt_sum_module.reset_sign_counts()


def to_sympy(value: SqrtSumV1) -> sp.Expr:
    """`SqrtSumV1` в выражение SymPy. Ровно те же коэффициенты, без округления."""

    total = sp.Integer(0)
    for radicand, coefficient in value.terms:
        rational = sp.Rational(coefficient.numerator, coefficient.denominator)
        total += rational * sp.sqrt(sp.Integer(radicand))
    return total


@dataclass(frozen=True, slots=True)
class Row:
    """Одна строка таблицы сверки. Все площади — точные выражения."""

    case: str
    alpha: Fraction
    outcome: str
    polygon_area: sp.Expr | None = None
    raw_area: sp.Expr | None = None
    resolved_area: sp.Expr | None = None
    queue_area: sp.Expr | None = None
    pairwise_outcome: str = ""
    raw_loops: int = 0
    queue_faces: int = 0
    components: int = 0
    candidates: int = 0
    queue_matches_raw: bool | None = None
    resolved_matches_raw: bool | None = None
    seconds_pairwise: float = 0.0
    seconds_queue: float = 0.0

    @property
    def queue_defect(self) -> sp.Expr | None:
        if self.raw_area is None or self.queue_area is None:
            return None
        return sp.simplify(self.raw_area - self.queue_area)

    @property
    def resolved_defect(self) -> sp.Expr | None:
        if self.raw_area is None or self.resolved_area is None:
            return None
        return sp.simplify(self.raw_area - self.resolved_area)


class AlphaIsNotDecimal(ValueError):
    """alpha без конечной десятичной записи. `DecalRequestV1` её не примет."""


def decimal_string(value: Fraction) -> str:
    """Точная десятичная запись дроби. Округления нет: его отказ назван.

    `LocalLengthV1` несёт `Decimal`, поэтому alpha стенда обязана быть
    представима конечной десятичной дробью. Знаменатель ступеней лестницы —
    степень двойки, так что условие выполнено по построению, но проверяется, а
    не предполагается.
    """

    denominator = value.denominator
    twos = fives = 0
    while denominator % 2 == 0:
        denominator //= 2
        twos += 1
    while denominator % 5 == 0:
        denominator //= 5
        fives += 1
    if denominator != 1:
        raise AlphaIsNotDecimal(str(value))
    digits = max(twos, fives)
    scaled = value.numerator * 10**digits // value.denominator
    if digits == 0:
        return str(scaled)
    sign = "-" if scaled < 0 else ""
    body = str(abs(scaled)).rjust(digits + 1, "0")
    return f"{sign}{body[:-digits]}.{body[-digits:]}"


def alpha_ladder(polygon, count: int = 3) -> tuple[Fraction, ...]:
    """Ступени alpha от малой до заведомо полной. Выведены из габарита."""

    xs = [x for loop in polygon.loops for x, _ in loop.points]
    ys = [y for loop in polygon.loops for _, y in loop.points]
    span = max(max(xs) - min(xs), max(ys) - min(ys))
    steps = (Fraction(1, 8), Fraction(1, 4), Fraction(1, 2))[:count]
    return tuple(Fraction(span) * step for step in steps)


def queue_coverage(polygon, alpha: Fraction):
    """Путь B: покрытие очередью событий. Возвращает (площадь, граней, исход)."""

    skeleton = build_skeleton(polygon)
    partition = build_faces(polygon, skeleton)
    if partition.outcome is not FaceOutcome.EXACT:
        return None, 0, partition.outcome.value
    coverage = coverage_at(partition, alpha)
    if coverage.outcome is not CoverageOutcome.EXACT:
        return None, 0, coverage.outcome.value
    live = sum(
        1 for face in coverage.faces if face.doubled_area.sign() > 0
    )
    return to_sympy(coverage.doubled_area) / 2, live, BENCH_EXACT


def pairwise_coverage(name: str, polygon, alpha: Fraction):
    """Путь A: попарный крой на тех же рёбрах с теми же законами прихода.

    Источником становится КАЖДОЕ ребро контура — ровно то, что видит очередь.
    Иначе два пути считали бы разные задачи и сравнение ничего не значило.
    """

    points = polygon.outer.points
    face = tuple((float(x), float(y)) for x, y in points)
    routes = tuple(
        {
            "name": f"e{index}",
            "points": (face[index], face[(index + 1) % len(face)]),
        }
        for index in range(len(face))
    )
    snapshot, request = straight_snapshot(
        faces=(face,),
        source_routes=routes,
        alpha=decimal_string(alpha),
        revision_name=f"q3-diff-{name}-{alpha}",
    )
    compiled = compile_reference_envelopes(snapshot, request)
    if compiled.compilation is None:
        return None, BENCH_PAIRWISE_COMPILE_REFUSED, compiled.outcome.value, None
    raw_result = evaluate_reference_raw_coverage(
        compiled.compilation, request.requested_alpha
    )
    if raw_result.raw_coverage is None:
        return None, BENCH_PAIRWISE_RAW_REFUSED, raw_result.outcome.value, None
    raw = raw_result.raw_coverage
    # Попарный путь на части входов не отказывает ИМЕНОВАННЫМ исходом, а
    # выбрасывает исключение (`ExactArrangementRotationSystemUnproven`).
    # Стенд его ловит и называет, а не падает: молчаливое падение стенда
    # скрыло бы ровно то, ради чего он написан.
    try:
        resolution = resolve_coverage_interactions(
            compiled.compilation, raw.boundary_resolved_envelopes, raw
        )
    except Exception as error:  # noqa: BLE001 — исход именуется, не глотается
        return raw, BENCH_PAIRWISE_RAISED, type(error).__name__, None
    return raw, BENCH_EXACT, "", resolution


def run_case(name: str, polygon, alpha: Fraction) -> Row:
    reset_caches()
    if polygon.holes:
        return Row(name, alpha, BENCH_HOLE_NOT_ONE_FACE)

    started = time.perf_counter()
    queue_area, queue_faces, queue_outcome = queue_coverage(polygon, alpha)
    queue_seconds = time.perf_counter() - started
    if queue_area is None:
        return Row(name, alpha, BENCH_QUEUE_REFUSED, pairwise_outcome=queue_outcome)

    reset_caches()
    started = time.perf_counter()
    raw, outcome, detail, resolution = pairwise_coverage(name, polygon, alpha)
    pairwise_seconds = time.perf_counter() - started
    if raw is None or resolution is None:
        return Row(
            name,
            alpha,
            outcome,
            raw_area=(
                None if raw is None
                else ExactScalar(raw.exact_area_expression).as_expr()
            ),
            queue_area=queue_area,
            queue_faces=queue_faces,
            raw_loops=0 if raw is None else len(raw.loops),
            pairwise_outcome=detail,
            seconds_queue=queue_seconds,
            seconds_pairwise=pairwise_seconds,
        )

    raw_area = ExactScalar(raw.exact_area_expression).as_expr()
    resolved = resolution.resolved_coverage
    resolved_area = (
        ExactScalar(resolved.exact_area_expression).as_expr()
        if resolved is not None
        else None
    )
    polygon_area = sp.Rational(
        sum(
            _doubled(loop.points) for loop in polygon.loops
        ),
        2,
    )
    return Row(
        case=name,
        alpha=alpha,
        outcome=BENCH_EXACT,
        polygon_area=polygon_area,
        raw_area=raw_area,
        resolved_area=resolved_area,
        queue_area=queue_area,
        pairwise_outcome=resolution.outcome.value,
        raw_loops=len(raw.loops),
        queue_faces=queue_faces,
        components=len(resolution.interaction_components),
        candidates=len(resolution.candidates),
        queue_matches_raw=exact_sign(raw_area - queue_area) == 0,
        resolved_matches_raw=(
            None
            if resolved_area is None
            else exact_sign(raw_area - resolved_area) == 0
        ),
        seconds_pairwise=pairwise_seconds,
        seconds_queue=queue_seconds,
    )


def _doubled(points) -> int:
    total = 0
    for index in range(len(points)):
        x0, y0 = points[index]
        x1, y1 = points[(index + 1) % len(points)]
        total += x0 * y1 - x1 * y0
    return total


def _mark(value: bool | None) -> str:
    if value is None:
        return "n/a"
    return "YES" if value else "NO"


def print_table(rows: tuple[Row, ...]) -> None:
    header = (
        f"{'case':26s} {'alpha':>9s} {'raw==queue':>10s} {'raw==resolved':>13s} "
        f"{'loops':>5s} {'faces':>5s} {'comp':>4s} {'cand':>4s} "
        f"{'pairwise outcome':34s} {'s(A)':>6s} {'s(B)':>6s}"
    )
    print(header)
    print("-" * len(header))
    for row in rows:
        if row.outcome != BENCH_EXACT:
            mark = (
                _mark(exact_sign(row.raw_area - row.queue_area) == 0)
                if row.raw_area is not None and row.queue_area is not None
                else "--"
            )
            print(
                f"{row.case:26s} {str(row.alpha):>9s} "
                f"{mark:>10s} {'--':>13s} {row.raw_loops:5d} {row.queue_faces:5d} "
                f"{'':>4s} {'':>4s} "
                f"{row.outcome + (':' + row.pairwise_outcome if row.pairwise_outcome else ''):34s}"
            )
            continue
        print(
            f"{row.case:26s} {str(row.alpha):>9s} "
            f"{_mark(row.queue_matches_raw):>10s} "
            f"{_mark(row.resolved_matches_raw):>13s} "
            f"{row.raw_loops:5d} {row.queue_faces:5d} "
            f"{row.components:4d} {row.candidates:4d} "
            f"{row.pairwise_outcome:34s} "
            f"{row.seconds_pairwise:6.2f} {row.seconds_queue:6.2f}"
        )
    print()
    print(
        "Расхождения площадей. Вердикт точный (`exact_sign`), числа рядом — "
        "для чтения, решения по ним не принимаются."
    )
    any_defect = False
    for row in rows:
        if row.raw_area is None or row.queue_area is None:
            continue
        queue_defect = row.queue_defect
        if queue_defect is not None and exact_sign(queue_defect) != 0:
            any_defect = True
            print(
                f"  {row.case} alpha={row.alpha}: raw - queue = "
                f"{_number(queue_defect)}  "
                f"(raw {_number(row.raw_area)}, queue {_number(row.queue_area)})"
            )
        resolved_defect = row.resolved_defect
        if resolved_defect is not None and exact_sign(resolved_defect) != 0:
            any_defect = True
            print(
                f"  {row.case} alpha={row.alpha}: raw - resolved = "
                f"{_number(resolved_defect)}"
            )
        if row.resolved_area is None and row.outcome == BENCH_EXACT:
            any_defect = True
            print(
                f"  {row.case} alpha={row.alpha}: попарный крой отказал — "
                f"{row.pairwise_outcome}; площади нет вовсе, "
                f"raw = {_number(row.raw_area)}, "
                f"queue = {_number(row.queue_area)}"
            )
    if not any_defect:
        print("  нет ни одного")


def _number(expression: sp.Expr) -> str:
    """Читаемое число рядом с точным вердиктом.

    Короткое выражение печатается как есть, длинное — только числом: на
    полевом контуре точная разность занимает несколько экранов и в таблице
    ничего не сообщает. Решение о равенстве при этом уже принято `exact_sign`,
    а не этой строкой.
    """

    text = str(expression)
    value = float(expression)
    if len(text) <= 40:
        return f"{text} = {value:.9f}"
    return f"{value:.9f} (точное выражение из {len(text)} знаков)"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cases", default="")
    parser.add_argument("--alphas", default="")
    parser.add_argument("--alpha-steps", type=int, default=3)
    arguments = parser.parse_args()

    wanted = tuple(
        item for item in arguments.cases.split(",") if item
    )
    corpus = tuple(
        (name, polygon)
        for name, polygon in named_corpus()
        if not wanted or name in wanted
    )
    explicit = tuple(
        Fraction(item) for item in arguments.alphas.split(",") if item
    )

    rows: list[Row] = []
    for name, polygon in corpus:
        alphas = explicit or alpha_ladder(polygon, arguments.alpha_steps)
        for alpha in alphas:
            rows.append(run_case(name, polygon, alpha))
    print_table(tuple(rows))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
