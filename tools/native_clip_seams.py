"""Oracle side of the clip differential seams: записывает вызовы функций ядра Python и сверяет их с нативными.

Единица нативной замены — ВСЯ `clip_geometry`, но её первая треть (числовой фасад, ячейки и привязка, предикаты
тесселяции) проверяется по частям: каждая функция из `cftuv_clip::seam::SEAMS` имеет швы в эталоне, и этот модуль
ставит на них записывающие обёртки (подмена имён в модулях и методов классов, с восстановлением), пока эталон
воспроизводит запись корпуса или тест ядра. Обёртка снимает аргументы (уже в проводе `cftuv_native.clip_seams`), результат
или исключение и, для функций, которые платят (знаки `SqrtSumV1`), состояние цены ДО и ПОСЛЕ: шесть статей бюджета,
`SIGN_COUNTS`, четыре таблицы памяти канонизации С ПОРЯДКОМ.

Проверка (`verify`) отдаёт те же аргументы нативному шву — вызов самодостаточен: заголовок загружает записанное состояние ДО
целиком — и сравнивает ТОЧНО: результат каноническим кодом `clip_memo._encode` (`int` и `Fraction` различны, `float` по
`hex`), исключение `(класс, текст)`, дельту `SIGN_COUNTS`, статьи бюджета (или `UNBUDGETED_WORK`) и журнал памяти,
применённый к копии таблиц ДО, против таблиц ПОСЛЕ. Нативный отказ `NativeUnsupported` — не расхождение, а учёт.

Выборка: холодные функции пишутся все, горячие (`_cheap_sign`, `line_value`, ...) — головой и шагом; вызов с ценой, исключением
или изменением памяти пишется всегда.
"""

from __future__ import annotations

import contextlib
import functools
import inspect
import sys
import time
from collections import Counter
from dataclasses import dataclass, field
from types import SimpleNamespace

import native_corpus as nc

import cftuv_envelope.materialize.clip as clip
import cftuv_envelope.materialize.clip_cells as clip_cells
import cftuv_envelope.materialize.clip_snap as clip_snap
import cftuv_envelope.materialize.lift_surface as lift_surface
import cftuv_envelope.materialize.tessellate as tessellate
import cftuv_envelope.wavefront.faces as faces

COUNT_KEYS = ("total", "closed_rational_zero", "closed_rational_nonzero", "closed_by_enclosure", "closed_by_conjugation")

#: Швы, которые платят цену (знаки): их состояние до и после снимается всегда.
COST_SEAMS = frozenset(
    {"ORIENTATION", "SHOELACE_SIGN", "WITHIN_EDGE_GAP", "CHORD_OF", "TRIANGULATE_EXACT", "CONVEX_QUAD_RING", "HAS_RIGHT_TURN", "SNAP_SOURCE_VERTICES", "ORDERED"}
)


@dataclass
class Sampling:
    """Сколько вызовов каждого шва писать: голова и шаг, потолок на шов."""

    head: int = 120
    stride: int = 40
    cap: int = 1500

    def wants(self, seen: int, recorded: int) -> bool:
        return recorded < self.cap and (seen < self.head or seen % self.stride == 0)


@dataclass
class SeamCall:
    """Один записанный вызов: провод аргументов, исход эталона, цена до и после, секунды эталона."""

    seam: str
    arguments: list
    result: object
    error: tuple | None
    pre: object | None
    post: object | None
    seconds: float
    extra: dict = field(default_factory=dict)


@dataclass
class Mismatch:
    seam: str
    field: str
    detail: str

    def __str__(self) -> str:
        return f"{self.seam}.{self.field}: {self.detail}"


def _py_version() -> tuple:
    return (sys.version_info.major, sys.version_info.minor)


class SeamRecorder:
    """Записывающие обёртки на швах эталона. `installed()` ставит и снимает их (подмены возвращаются в точности)."""

    def __init__(self, sampling: Sampling | None = None, *, only: frozenset | None = None) -> None:
        self.sampling = sampling or Sampling()
        self.only = only
        self.calls: list[SeamCall] = []
        self.seen: Counter = Counter()
        self.recorded: Counter = Counter()
        self.errors: Counter = Counter()
        self._originals: list = []

    # ---- запись ----------------------------------------------------------------------------------------------------

    def _wanted(self, seam: str) -> bool:
        return self.only is None or seam in self.only

    def _call(self, seam: str, original, args: tuple, kwargs: dict, encode, *, budget=None, precode: bool = False, observe=None):
        """Вызов эталона с записью. `encode()` строит провод аргументов; `precode` — кодировать ДО вызова (аргумент мутирует)."""

        seen = self.seen[seam]
        self.seen[seam] = seen + 1
        costly = seam in COST_SEAMS
        if not self._wanted(seam):
            return original(*args, **kwargs)
        sampled = self.sampling.wants(seen, self.recorded[seam])
        wire = encode() if (precode and sampled) else None
        pre = nc.capture_state(budget, None) if costly else None
        started = time.perf_counter()
        try:
            result, error = original(*args, **kwargs), None
        except Exception as exc:  # noqa: BLE001 - исключение эталона — часть исхода шва
            result, error = None, (type(exc).__qualname__, str(exc))
            raised = exc
        seconds = time.perf_counter() - started
        post = nc.capture_state(budget, None) if costly else None
        interesting = costly and (error is not None or _spent(pre, post))
        if (sampled or interesting) and self.recorded[seam] < self.sampling.cap + 200:
            try:
                if wire is None:
                    wire = encode()
            except Exception as exc:  # noqa: BLE001 - вход, который шов не несёт, учитывается
                self.errors[f"{seam}: {type(exc).__name__}"] += 1
            else:
                extra = observe(result, error) if observe is not None else {}
                self.calls.append(SeamCall(seam, wire, extra.pop("result", result), error, pre, post, seconds, extra))
                self.recorded[seam] += 1
        if error is not None:
            raise raised
        return result

    # ---- обёртки ---------------------------------------------------------------------------------------------------

    def _wrap_function(self, seam: str, function, parameters: tuple, encode, *, budget_name: str | None = "budget", precode: bool = False, observe=None):
        signature = inspect.signature(function)

        @functools.wraps(function)
        def wrapper(*args, **kwargs):
            bound = signature.bind(*args, **kwargs)
            bound.apply_defaults()
            values = bound.arguments
            budget = values.get(budget_name) if budget_name else None
            watcher = None if observe is None else (lambda result, error: observe(result, error, values))
            return self._call(seam, function, args, kwargs, lambda: encode(values), budget=budget, precode=precode, observe=watcher)

        return wrapper

    def _plan(self) -> list:
        """`(объект, имя, шов, параметры, кодировщик, опции)` — всё, что подменяется."""

        from cftuv_native import clip_seams as wire

        return _patch_table(self, wire)

    @contextlib.contextmanager
    def installed(self):
        applied = []
        try:
            for owner, name, seam, builder in self._plan():
                original = getattr(owner, name)
                setattr(owner, name, builder(original))
                applied.append((owner, name, original))
            yield self
        finally:
            for owner, name, original in reversed(applied):
                setattr(owner, name, original)


def _spent(pre, post) -> bool:
    """Цена вызова не нулевая: статьи, сопряжение или память."""

    if pre.budget != post.budget:
        return True
    if pre.unbudgeted != post.unbudgeted:
        return True
    if pre.sign_counts.get("closed_by_conjugation") != post.sign_counts.get("closed_by_conjugation"):
        return True
    return pre.known_primes != post.known_primes or pre.factorization != post.factorization or pre.squarefree != post.squarefree or pre.prime_support != post.prime_support


def _patch_table(recorder: SeamRecorder, wire) -> list:
    """Таблица подмен: модульные имена (в каждом модуле, который их импортировал) и методы классов."""

    plan: list = []

    def function(owner, name, seam, encode, **options):
        plan.append((owner, name, seam, lambda original, seam=seam, encode=encode, options=options: recorder._wrap_function(seam, original, (), encode, **options)))

    def method(owner, name, seam, wrapper_factory):
        plan.append((owner, name, seam, wrapper_factory))

    enc = wire
    point_pair = lambda value: enc.enc_point(value)  # noqa: E731

    for owner in (faces, tessellate):
        function(owner, "orientation", "ORIENTATION", lambda v: [point_pair(v["first"]), point_pair(v["second"]), point_pair(v["third"])])
    for owner in (faces, tessellate, clip):
        function(owner, "shoelace_sign", "SHOELACE_SIGN", lambda v: [enc.enc_points(v["points"])])
    for owner in (faces, clip):
        function(owner, "doubled_shoelace", "DOUBLED_SHOELACE", lambda v: [enc.enc_points(v["points"])], budget_name=None)
    for owner in (tessellate, clip):
        function(owner, "triangulate_exact", "TRIANGULATE_EXACT", lambda v: [enc.enc_points(v["points"])])
        function(owner, "convex_quad_ring", "CONVEX_QUAD_RING", lambda v: [enc.enc_points(v["points"])])
        function(owner, "has_right_turn", "HAS_RIGHT_TURN", lambda v: [enc.enc_points(v["points"]), list(v["ring"])])
    for owner in (clip_snap, clip):
        function(owner, "within_edge_gap", "WITHIN_EDGE_GAP", lambda v: [v["value"], v["edge_square"]])
    for owner in (clip_cells, clip):
        function(owner, "chord_of", "CHORD_OF", lambda v: _enc_chord(enc, v))
    function(clip_cells, "hinge_depth_square", "HINGE_DEPTH_SQUARE", lambda v: [v["jump_square"], list(v["column"])], budget_name=None)
    for owner in (clip_cells, clip, clip_snap):
        function(owner, "nanometres", "NANOMETRES", lambda v: [v["depth_square"]], budget_name=None)
    for owner in (clip_snap, clip):
        function(owner, "milli_cells", "MILLI_CELLS", lambda v: [v["square"]], budget_name=None)
    for owner in (clip_cells, clip):
        function(owner, "build_cells", "BUILD_CELLS", lambda v: _enc_build(enc, v), budget_name=None, precode=True, observe=_observe_build)
    for owner in (clip_snap, clip):
        function(owner, "snap_source_vertices", "SNAP_SOURCE_VERTICES", lambda v: [enc.enc_triangles(v["plane"].triangles), enc.enc_named_points(v["points"])])
    function(clip, "_rational_pair", "RATIONAL_PAIR", lambda v: [enc.enc_point(v["point"])], budget_name=None)
    _plan_methods(recorder, enc, method)
    return plan


def _observe_build(result, error, values) -> dict:
    """`memo` вызова ПОСЛЕ него (копия: вторая стадия продолжает её наполнять); пусто, если `memo` не передан."""

    memo = values["memo"]
    return {} if memo is None or error is not None else {"memo_after": dict(memo)}


def _enc_chord(enc, v) -> list:
    cell = v["cell"]
    jump = None if cell.hinge is None else cell.hinge.jump_square
    return [jump, cell.flat_square, [list(column) for column in v["values"]]]


def _enc_build(enc, v) -> list:
    memo = v["memo"]
    return [enc.enc_triangles(v["triangles"]), [enc.enc_key(key) for key in v["split"]], enc.enc_memo({} if memo is None else memo)]


def _plan_methods(recorder: SeamRecorder, enc, method) -> None:
    """Методы классов: `BoundSurfaceLiftV1` (подъём) и `ClipStageV1` (резка)."""

    lift_class, stage_class = lift_surface.BoundSurfaceLiftV1, clip.ClipStageV1

    def line_value(original):
        @functools.wraps(original)
        def wrapper(self, triangle, index, point):
            return recorder._call("LINE_VALUE", original, (self, triangle, index, point), {}, lambda: [enc.enc_chart(triangle.chart), index, point[0], point[1]])

        return wrapper

    def window(original):
        @functools.wraps(original)
        def wrapper(self, point):
            return recorder._call("WINDOW", original, (self, point), {}, lambda: [point[0], point[1]])

        return wrapper

    def values_in(original):
        @functools.wraps(original)
        def wrapper(self, triangle, point):
            return recorder._call("VALUES_IN", original, (self, triangle, point), {}, lambda: [enc.enc_triangle(triangle), point[0], point[1]])

        return wrapper

    def stretch_square(original):
        @functools.wraps(original)
        def wrapper(self, triangle):
            return recorder._call("STRETCH_SQUARE", original, (self, triangle), {}, lambda: [enc.enc_triangle(triangle)])

        return wrapper

    def lift_known(original):
        @functools.wraps(original)
        def wrapper(self, triangle, values):
            before = len(self._normal_by_position)

            def observe(result, error):
                written = None
                if error is None:
                    position = result[0]
                    written = self._normal_by_position.get((position.x, position.y, position.z))
                return {"normals_before": before, "normals_after": len(self._normal_by_position), "written": written}

            return recorder._call(
                "LIFT_KNOWN", original, (self, triangle, values), {}, lambda: [*_version(), enc.enc_triangle(triangle), list(values)], observe=observe
            )

        return wrapper

    def edge_constants(original):
        @functools.wraps(original)
        def wrapper(self, ti, index):
            return recorder._call("EDGE_CONSTANTS", original, (self, ti, index), {}, lambda: [enc.enc_chart(self.regions[ti].chart), index])

        return wrapper

    def cheap_sign(original):
        @functools.wraps(original)
        def wrapper(self, node, constants, watch):
            return recorder._call(
                "CHEAP_SIGN", original, (self, node, constants, watch), {}, lambda: [enc.enc_point(node.point), _enc_constants(constants), watch]
            )

        return wrapper

    def ordered(original):
        @functools.wraps(original)
        def wrapper(self, first, second, nodes):
            def encode():
                return [*_version(), enc.enc_point(first.point), enc.enc_point(second.point), enc.enc_points([node.point for node in nodes])]

            def observe(result, error):
                if error is not None:
                    return {}
                index = {id(node): position for position, node in enumerate(nodes)}
                return {"result": [index[id(node)] for node in result]}

            return recorder._call("ORDERED", original, (self, first, second, nodes), {}, encode, budget=self.budget, observe=observe)

        return wrapper

    for name, factory in (("line_value", line_value), ("window", window), ("values_in", values_in), ("stretch_square", stretch_square), ("lift_known", lift_known)):
        method(lift_class, name, name.upper(), factory)
    for name, factory in (("_edge_constants", edge_constants), ("_cheap_sign", cheap_sign), ("_ordered", ordered)):
        method(stage_class, name, name.upper(), factory)


def _version() -> list:
    return list(_py_version())


def _enc_constants(constants):
    if constants is None:
        return None
    return list(constants)


# --------------------------------------------------------------------------
# сравнение
# --------------------------------------------------------------------------


class _Tables:
    """Копии таблиц памяти ДО вызова с теми же именами, что у модуля ядра: сюда применяется журнал нативного вызова."""

    def __init__(self, state) -> None:
        self._KNOWN_PRIMES = list(state.known_primes)
        self._KNOWN_PRIME_SET = set(state.known_primes)
        self._FACTORIZATION_MEMO = dict(state.factorization)
        self._SQUAREFREE_MEMO = dict(state.squarefree)
        self._PRIME_SUPPORT_MEMO = dict(state.prime_support)

    def reset_factorization_memory(self) -> None:
        self._KNOWN_PRIMES.clear()
        self._KNOWN_PRIME_SET.clear()
        self._FACTORIZATION_MEMO.clear()
        self._SQUAREFREE_MEMO.clear()
        self._PRIME_SUPPORT_MEMO.clear()


def _decode_expected(seam: str, call: SeamCall):
    return call.result


def _normal_view(call: SeamCall):
    return call.extra


class Verifier:
    """Прогоняет записанные вызовы через нативные швы и собирает расхождения, учёт и время."""

    def __init__(self) -> None:
        from cftuv_native import clip_seams as wire
        from cftuv_native import cost

        self.wire = wire
        self.cost = cost
        self.runner = wire.SeamRunner()
        self.mismatches: list[Mismatch] = []
        self.checked: Counter = Counter()
        self.unsupported: Counter = Counter()
        self.native_seconds: Counter = Counter()
        self.compute_seconds: Counter = Counter()
        self.oracle_seconds: Counter = Counter()
        self.refused: Counter = Counter()

    def header(self, call: SeamCall):
        if call.pre is None:
            return None
        pre = call.pre
        return self.wire.full_header(pre.budget, pre.known_primes, pre.factorization, pre.squarefree, pre.prime_support)

    def check(self, call: SeamCall) -> list[Mismatch]:
        started = time.perf_counter()
        answer = self.runner.call(call.seam, call.arguments, self.header(call))
        found: list[Mismatch] = []
        if answer.unsupported:
            self.unsupported[call.seam] += 1
            return found
        decoded = None
        if answer.ok:
            decoded = self.wire.DECODERS[call.seam](answer.value)
        self.native_seconds[call.seam] += time.perf_counter() - started
        self.compute_seconds[call.seam] += answer.nanoseconds * 1e-9
        self.oracle_seconds[call.seam] += call.seconds
        self.checked[call.seam] += 1
        found.extend(self._compare_outcome(call, answer, decoded))
        if call.pre is not None:
            found.extend(self._compare_cost(call, answer))
        found.extend(self._compare_extra(call, answer, decoded))
        return found

    def _compare_outcome(self, call: SeamCall, answer, decoded) -> list[Mismatch]:
        if call.error is not None:
            if answer.ok:
                return [Mismatch(call.seam, "exception", f"oracle raised {call.error!r}, native answered {nc.canonical(decoded)[:200]}")]
            budget = nc.build_budget(call.pre.budget) if call.pre is not None else None
            native = self.wire.exception_of(answer, budget)
            self.refused[call.seam] += 1
            return [] if native == call.error else [Mismatch(call.seam, "exception", f"{call.error!r} != {native!r}")]
        if not answer.ok:
            return [Mismatch(call.seam, "exception", f"native refused (status {answer.status} {answer.detail!r}), oracle answered")]
        expected, actual = nc.canonical(call.result), nc.canonical(decoded)
        if expected != actual:
            return [Mismatch(call.seam, "result", nc._locate(call.result, decoded))]
        return []

    def _compare_cost(self, call: SeamCall, answer) -> list[Mismatch]:
        cost, pre, post = self.cost, call.pre, call.post
        found = []
        delta = [post.sign_counts[key] - pre.sign_counts[key] for key in cost.COUNT_KEYS]
        if list(answer.counts) != delta:
            found.append(Mismatch(call.seam, "sign_counts", f"{delta} != {list(answer.counts)}"))
        if pre.budget is None:
            spent = [after - before for after, before in zip(post.unbudgeted, pre.unbudgeted)]
        else:
            spent = list(post.budget["articles"])
        if list(answer.articles) != spent:
            found.append(Mismatch(call.seam, "budget", f"{spent} != {list(answer.articles)}"))
        tables = _Tables(pre)
        self.cost.CostMirror._apply_entries(tables, answer.result().log)
        for name, got, want in (
            ("known_primes", tables._KNOWN_PRIMES, post.known_primes),
            ("factorization", list(tables._FACTORIZATION_MEMO.items()), post.factorization),
            ("squarefree", list(tables._SQUAREFREE_MEMO.items()), post.squarefree),
            ("prime_support", list(tables._PRIME_SUPPORT_MEMO.items()), post.prime_support),
        ):
            if nc.canonical(got) != nc.canonical(want):
                found.append(Mismatch(call.seam, f"memory.{name}", nc._locate(want, got)))
        return found

    def _compare_extra(self, call: SeamCall, answer, decoded) -> list[Mismatch]:
        extra = call.extra
        if call.seam == "LIFT_KNOWN" and answer.ok and call.error is None:
            normal = decoded[1][1]
            if normal is not None and nc.canonical(extra["written"]) != nc.canonical(normal):
                return [Mismatch(call.seam, "normal_write", "the oracle did not write the returned normal at the lifted position")]
            if normal is None and extra["normals_after"] != extra["normals_before"]:
                return [Mismatch(call.seam, "normal_write", "the oracle wrote a normal where none was returned")]
        if call.seam == "BUILD_CELLS" and call.error is None and answer.ok and "memo_after" in extra:
            actual = self.wire.dec_memo(answer.value[1])
            if nc.canonical(extra["memo_after"]) != nc.canonical(actual):
                return [Mismatch(call.seam, "memo", nc._locate(extra["memo_after"], actual))]
        if call.seam == "LIFT_KNOWN" and call.error is not None and extra["normals_after"] != extra["normals_before"]:
            return [Mismatch(call.seam, "normal_write", "a failed lift left a normal behind")]
        return []

    def verify(self, calls) -> list[Mismatch]:
        found: list[Mismatch] = []
        for call in calls:
            found.extend(self.check(call))
        self.mismatches.extend(found)
        return found

    def report(self) -> dict:
        """Учёт по швам: сколько проверено, отказов эталона, микросекунды эталона на вызов, нативные целиком (кодирование, пересечение границы,
        разбор) и нативный счёт внутри расширения (`compute_us`: без границы; именно он сравним с эталоном, граница амортизируется целой операцией)."""

        rows = {}
        for seam, count in sorted(self.checked.items()):
            native, oracle, compute = self.native_seconds[seam], self.oracle_seconds[seam], self.compute_seconds[seam]
            rows[seam] = {
                "checked": count,
                "oracle_raised": self.refused[seam],
                "oracle_us": 1e6 * oracle / count,
                "native_total_us": 1e6 * native / count,
                "compute_us": 1e6 * compute / count,
                "compute_speedup": (oracle / compute) if compute else float("inf"),
            }
        return rows
