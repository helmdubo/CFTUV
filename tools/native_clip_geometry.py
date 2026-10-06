"""Whole-operation harness of the native `clip_geometry` (test-only): native against the LIVE Python oracle on recorded calls.

Один вызов записи корпуса (`native_corpus`: состояние процесса ДО, пикл входов) идёт двумя путями. Эталон — `nc.execute` на восстановленном
состоянии (ядро питона, ЭТОТ интерпретатор). Нативный — шов `CLIP_GEOMETRY` (`cftuv-clip/src/geometry_seam.rs`): те же входы кодируются в провод
(`cftuv_native.clip_seams.enc_geometry`), заголовок грузит записанное состояние ДО целиком, ответ расшифровывается в типы эталона
(`ClippedV1`, `LocalPoint3V1`, `SqrtSumV1`) и в `nc.Outcome`: результат, исключение `(класс, текст)`, состояние ПОСЛЕ (статьи бюджета, `SIGN_COUNTS`,
четыре таблицы памяти с порядком — журнал применён к копии таблиц ДО) и наблюдаемое — запись нормалей смещения в `plane._normal_by_position`
(упорядоченный список записей поверх исходного словаря плоскости). Сравнение — `nc.compare_outcomes`, как у корпуса: точное, без послаблений.

Нативный отказ `NativeUnsupported` — не расхождение, а учёт (`Run.unsupported`); тихого отката нет.

    python tools/native_clip_geometry.py compare [--stride N]      # полевой + синтетический + производные записи
    python tools/native_clip_geometry.py timing                    # compute нативного против эталона: p50/p95/max по сеткам
    python tools/native_clip_geometry.py dump DIR [--top N]        # запросы шва самых тяжёлых записей (для `cargo run --example clip_profile`)
    python tools/native_clip_geometry.py chain DIR MESH PATCH      # запросы шва соседних alpha одного патча по порядку (для `clip_profile --chain`)
"""

from __future__ import annotations

import argparse
import os
import random
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
for _path in (ROOT / "kernel" / "src", ROOT / "tools"):
    if str(_path) not in sys.path:
        sys.path.insert(0, str(_path))

import cftuv_envelope.exact_sqrt_sum as exact  # noqa: E402
import native_clip_generated as generated  # noqa: E402
import native_clip_seams as seams  # noqa: E402
import native_corpus as nc  # noqa: E402

def corpus_base() -> Path:
    """Полевой корпус ЭТОГО ядра (`nc.matching_corpus`); нет его — несуществующий путь с именем ядра: `.exists()` ложно, причина названа."""

    found = nc.matching_corpus()
    if found is not None:
        return found
    return Path(os.environ.get(nc.CORPUS_ENVIRONMENT) or nc.DEFAULT_CORPUS_BASE) / f"<ядро {nc.clip_memo.kernel_code_identity()}: корпуса нет>"


def field_paths(stride: int = 1) -> list:
    return sorted((corpus_base() / "records").glob("*/*clip_geometry*.rec"))[::stride]


def field_chains(minimum: int = 3) -> dict:
    """`{(mesh, patch): [path, ...]}`: the field calls of one patch in ascending alpha (the width slider's neighbouring steps), chains of `minimum` or more."""

    groups: dict = {}
    for path in field_paths():
        meta = nc.read_meta(path)
        groups.setdefault((meta["mesh"], meta.get("patch_id")), []).append((meta["alpha"], path))
    return {key: [path for _alpha, path in sorted(items, key=lambda item: item[0])] for key, items in groups.items() if len(items) >= minimum}


def derived_paths() -> list:
    return sorted((corpus_base() / "records" / "_derived").glob("*/*clip_geometry*.rec"))


def synthetic_paths() -> list:
    return sorted((corpus_base() / "synthetic_clip" / "records").glob("*/*clip_geometry*.rec"))


@dataclass
class Run:
    """Исход одного вызова на обоих путях: расхождения, секунды эталона, нативный compute (без кодека), нативный отказ."""

    differences: list
    oracle_seconds: float
    native_compute_seconds: float
    native_total_seconds: float
    outcome_label: str
    unsupported: str = ""
    expected: object = None
    actual: object = None
    extras: dict = field(default_factory=dict)

    @property
    def equal(self) -> bool:
        return not self.differences and not self.unsupported


def _after_state(before: "nc.StateV1", answer) -> "nc.StateV1":
    """Состояние процесса ПОСЛЕ нативного вызова: журнал памяти применён к копии таблиц ДО, статьи и знаки — из ответа."""

    from cftuv_native import cost

    tables = seams._Tables(before)
    cost.CostMirror._apply_entries(tables, answer.result().log)
    counts = dict(before.sign_counts)
    for key, delta in zip(cost.COUNT_KEYS, answer.counts):
        counts[key] = counts.get(key, 0) + delta
    if before.budget is None:
        budget = None
        unbudgeted = tuple(old + delta for old, delta in zip(before.unbudgeted, answer.articles))
    else:
        budget = {**before.budget, "articles": tuple(answer.articles)}
        unbudgeted = before.unbudgeted
    return nc.StateV1(
        list(tables._KNOWN_PRIMES),
        list(tables._FACTORIZATION_MEMO.items()),
        list(tables._SQUAREFREE_MEMO.items()),
        list(tables._PRIME_SUPPORT_MEMO.items()),
        budget,
        counts,
        unbudgeted,
        before.store,
        before.canonical_audit,
    )


def _observed(call: "nc.Call", writes: list) -> dict:
    """`nc.observe` для нативного вызова: словарь нормалей плоскости ДО вызова плюс записи вызова по порядку."""

    normals = getattr(call.args[0], "_normal_by_position", None)
    if normals is None:
        return {}
    merged = dict(normals)
    for position, normal in writes:
        merged[position] = normal
    return {"plane_normals": nc.canonical(list(merged.items()))}


class WholeRunner:
    """Нативный путь: один сеанс шва, заголовок грузит состояние ДО целиком (вызов самодостаточен)."""

    def __init__(self, version=None) -> None:
        from cftuv_native import clip_seams as wire

        self.wire = wire
        self.runner = wire.SeamRunner()
        #: `None`: the version of the running interpreter; a pair is the NEGATIVE control (the native emulation of another version).
        self.version = version

    def native(self, op_blob: bytes, before: "nc.StateV1"):
        """`(Outcome, ответ, секунды целиком)` нативного пути на записанных входах; `Outcome.result` — `ClippedV1`."""

        wire = self.wire
        budget = nc.build_budget(before.budget)
        call = nc.decode_call(nc.OP_CLIP, op_blob, budget, None)
        header = wire.full_header(before.budget, before.known_primes, before.factorization, before.squarefree, before.prime_support)
        started = time.perf_counter()
        arguments = wire.enc_geometry(call.args[0], call.kwargs, self.version)
        answer = self.runner.call("CLIP_GEOMETRY", arguments, header)
        if answer.unsupported:
            return None, answer, time.perf_counter() - started
        result, error = None, None
        if answer.ok:
            result = wire.dec_clipped(answer.value)
        else:
            error = wire.exception_of(answer, budget)
        total = time.perf_counter() - started
        after = _after_state(before, answer)
        observed = _observed(call, wire.dec_writes(answer.extras))
        return nc.Outcome(result, error, after, observed, total), answer, total

    def compare(self, record: "nc.Record", *, before: "nc.StateV1 | None" = None) -> Run:
        """Эталон и нативный путь на записи; `before` подменяет записанное состояние (свип потолка бюджета)."""

        before = record.before() if before is None else before
        call = nc.prepare_call(nc.OP_CLIP, record.call_blob, before)
        expected = nc.execute(call)
        outcome, answer, total = self.native(record.call_blob, before)
        label = "CLIPPED" if expected.exception is None else f"raised:{expected.exception[0]}"
        if outcome is None:
            detail = answer.detail[0] if answer.detail else ""
            return Run([], expected.seconds, 0.0, total, label, unsupported=str(self.wire.dec_str(detail)), expected=expected)
        differences = nc.compare_outcomes(nc.OP_CLIP, before, expected, outcome)
        compute = answer.extras[1] * 1e-9
        return Run(differences, expected.seconds, compute, total, label, expected=expected, actual=outcome)


# --------------------------------------------------------------------------
# Вызовы для свипов (общие у сверки шва и сверки вставки)
# --------------------------------------------------------------------------


def heavy_paths(count: int) -> list:
    """Самые долгие полевые вызовы (по секундам записи): по одному на сетку-патч, тяжёлые первыми."""

    timed = sorted(((nc.read_meta(path)["seconds"], path) for path in field_paths()), key=lambda item: -item[0])
    chosen, seen = [], set()
    for _seconds, path in timed:
        meta = nc.read_meta(path)
        label = (meta["mesh"], meta["patch_id"])
        if label not in seen:
            seen.add(label)
            chosen.append(path)
        if len(chosen) == count:
            break
    return chosen


def cap_levels(delta: int) -> list:
    """Потолки над уже потраченным: от нуля до всей траты и чуть выше, плотно у границ (точка исчерпания у каждого вызова разная)."""

    levels = {0, 1, 2, 3, 5, 8, 13, 21, 34, 55, 89, delta - 2, delta - 1, delta, delta + 1}
    return sorted(level for level in levels if level >= 0)


def fresh_calls(seed: int, count: int):
    """`(метка, подъём, kwargs, потолок)`: те же генераторы, что у корпуса, другое зерно и другая смесь шумов."""

    rng = random.Random(seed)
    names = sorted(generated.PLANES)
    for number in range(count):
        name = names[number % len(names)]
        lift = generated.PLANES[name]()
        kwargs = generated.random_call(
            rng, lift, number, noise=rng.choice((0.0, 0.0, 0.2, 0.5, 0.9)), int_coefficients=rng.random() < 0.2, outside=rng.choice((0.0, 0.0, 0.3, 1.0, 3.0))
        )
        if kwargs is not None:
            cap = rng.choice((None, None, None, 0, 1, 2, 4, 8, 16, 34, 70)) if rng.random() < 0.3 else None
            yield f"{name}-{number:03d}", lift, generated.with_plan(f"fresh-{seed}-{number}", kwargs, lift), cap


def compare_generated(runner, label: str, lift, kwargs: dict, cap):
    budget = exact.exact_work_budget(stage="MATERIALIZE", domain_id=f"fresh-{label}", superlevel="", cap=cap)
    plane = lift.bind(budget)
    with exact.isolated_factorization_memory():
        before = nc.capture_state(budget, None)
        blob = nc.encode_call(nc.Call(nc.OP_CLIP, (plane,), kwargs, budget, None))
    return runner.compare(nc.Record({"mesh": f"fresh/{label}"}, {"call": blob}), before=before)


class DropinRunner:
    """The production-shaped path: `cftuv_native.clip_geometry` (the drop-in) on the REAL process state, against the live oracle.

    Оба пути стартуют с одного восстановленного состояния ДО; нативный вызывается как `clip.clip_geometry` (плоскость, бюджет, именованные
    аргументы) и пишет в настоящие бюджет, `SIGN_COUNTS`, таблицы памяти и `plane._normal_by_position`. Сравнение — то же `nc.compare_outcomes`.
    Один и тот же `mirror` живёт между вызовами (кэш плоскостей, снимки таблиц), как в сеансе: это и проверяется.
    """

    def __init__(self, mirror=None) -> None:
        import cftuv_native

        self.mirror = cftuv_native.new_mirror() if mirror is None else mirror

    def outcome(self, record: "nc.Record", before: "nc.StateV1") -> "nc.Outcome":
        return nc.execute(nc.prepare_call(nc.OP_CLIP, record.call_blob, before), function=self.mirror.clip_geometry)

    def compare(self, record: "nc.Record", *, before: "nc.StateV1 | None" = None) -> Run:
        before = record.before() if before is None else before
        expected = nc.execute(nc.prepare_call(nc.OP_CLIP, record.call_blob, before))
        actual = self.outcome(record, before)
        label = "CLIPPED" if expected.exception is None else f"raised:{expected.exception[0]}"
        if actual.exception is not None and actual.exception[0] == "NativePortUnsupported":
            return Run([], expected.seconds, 0.0, actual.seconds, label, unsupported=actual.exception[1], expected=expected)
        differences = nc.compare_outcomes(nc.OP_CLIP, before, expected, actual)
        return Run(differences, expected.seconds, actual.seconds, actual.seconds, label, expected=expected, actual=actual)


def explain(runs: list, limit: int = 12) -> str:
    lines = []
    for name, run in runs:
        if run.unsupported:
            lines.append(f"{name}: NativeUnsupported {run.unsupported}")
        for item in run.differences:
            lines.append(f"{name}: {item}")
    return "\n".join(lines[:limit]) + (f"\n... and {len(lines) - limit} more" if len(lines) > limit else "")


def percentile(values: list, fraction: float) -> float:
    ordered = sorted(values)
    if not ordered:
        return 0.0
    return ordered[min(len(ordered) - 1, int(round(fraction * (len(ordered) - 1))))]


def summarize(values: list) -> dict:
    return {"n": len(values), "p50": percentile(values, 0.5), "p95": percentile(values, 0.95), "max": max(values, default=0.0)}


def _ms(seconds: float) -> str:
    return f"{seconds * 1e3:8.2f}"


def timing_table(rows: list, left: str = "oracle", right: str = "native") -> str:
    """`rows`: `(группа, секунды левого, секунды правого)` -> таблица p50/p95/max по группам и отношение процентилей."""

    groups: dict = {}
    for group, first, second in rows:
        groups.setdefault(group, ([], []))
        groups[group][0].append(first)
        groups[group][1].append(second)
    lines = [f"{'group':32s} {'n':>4s} | {left} ms p50 / p95 / max | {right} ms p50 / p95 / max | x(p50) x(p95) x(max)"]
    for group in sorted(groups):
        first, second = summarize(groups[group][0]), summarize(groups[group][1])
        ratios = [(first[key] / second[key]) if second[key] else float("inf") for key in ("p50", "p95", "max")]
        lines.append(
            f"{group:32s} {first['n']:4d} | {_ms(first['p50'])} {_ms(first['p95'])} {_ms(first['max'])} | "
            f"{_ms(second['p50'])} {_ms(second['p95'])} {_ms(second['max'])} | {ratios[0]:6.1f} {ratios[1]:6.1f} {ratios[2]:6.1f}"
        )
    return chr(10).join(lines)


def group_of(record: "nc.Record") -> str:
    meta = record.meta
    if meta["mesh"] == "rounded_wall_noise_top" and meta.get("patch_id") == 2:
        return "rounded_wall_noise_top patch 2"
    return str(meta["mesh"] or "_")


def oracle_warm_seconds(record: "nc.Record", before: "nc.StateV1") -> float:
    """Секунды эталона на ТЕПЛЫХ чистых кэшах (произведения радикандов): память канонизации и бюджет — как до вызова, кэши не сброшены."""

    import cftuv_envelope.exact_sqrt_sum as exact

    exact.reset_factorization_memory()
    exact._KNOWN_PRIMES.extend(before.known_primes)
    exact._KNOWN_PRIME_SET.update(before.known_primes)
    exact._FACTORIZATION_MEMO.update(before.factorization)
    exact._SQUAREFREE_MEMO.update(before.squarefree)
    exact._PRIME_SUPPORT_MEMO.update(before.prime_support)
    call = nc.decode_call(nc.OP_CLIP, record.call_blob, nc.build_budget(before.budget), None)
    started = time.perf_counter()
    try:
        nc.invoke(call)
    except Exception:  # noqa: BLE001 - отказ эталона тоже путь
        pass
    return time.perf_counter() - started


def run_timing(paths: list, runner: WholeRunner, repeat: int = 3) -> tuple:
    """`(warm, cold, число расхождений)`: строки `(группа, эталон, нативный compute)` для тёплых и холодных кэшей.

    Холодный эталон — прогон на состоянии, где сброшены кэши (`restore_state`), холодный нативный — новый сеанс (пустая память произведений).
    Тёплый эталон — повтор без сброса кэшей, тёплый нативный — тот же сеанс. Время — лучшее из `repeat`; нативное — внутри расширения, без кодека."""

    warm, cold, bad = [], [], 0
    for path in paths:
        record = nc.read_record(path)
        before = record.before()
        best = {"oracle_cold": float("inf"), "oracle_warm": float("inf"), "native_cold": float("inf"), "native_warm": float("inf")}
        equal = True
        for _ in range(repeat):
            run = runner.compare(record)
            equal = equal and run.equal
            best["oracle_cold"] = min(best["oracle_cold"], run.oracle_seconds)
            best["native_warm"] = min(best["native_warm"], run.native_compute_seconds or float("inf"))
            fresh = WholeRunner(runner.version).compare(record)
            equal = equal and fresh.equal
            best["native_cold"] = min(best["native_cold"], fresh.native_compute_seconds or float("inf"))
            best["oracle_warm"] = min(best["oracle_warm"], oracle_warm_seconds(record, before))
        bad += not equal
        label = group_of(record)
        for group in (label, "all"):
            warm.append((group, best["oracle_warm"], best["native_warm"]))
            cold.append((group, best["oracle_cold"], best["native_cold"]))
    return warm, cold, bad


def dump_requests(paths: list, out: Path, top: int) -> list:
    """Запросы шва (`[header, 124, arguments]`) самых медленных записей, по одной на сетку и патч: вход для `cargo run --example clip_profile`."""

    from cftuv_native import clip_seams as wire

    out.mkdir(parents=True, exist_ok=True)
    timed = []
    for path in paths:
        record = nc.read_record(path)
        timed.append((record.meta["seconds"], path, record))
    written, seen = [], set()
    for seconds, path, record in sorted(timed, key=lambda item: -item[0]):
        label = (record.meta["mesh"], record.meta.get("patch_id"))
        if label in seen or len(written) >= top:
            continue
        seen.add(label)
        before = record.before()
        budget = nc.build_budget(before.budget)
        call = nc.decode_call(nc.OP_CLIP, record.call_blob, budget, None)
        header = wire.full_header(before.budget, before.known_primes, before.factorization, before.squarefree, before.prime_support)
        request = wire.request_bytes("CLIP_GEOMETRY", wire.enc_geometry(call.args[0], call.kwargs), header)
        target = out / f"{record.meta['mesh']}-p{record.meta.get('patch_id')}-{path.stem}.req"
        target.write_bytes(request)
        written.append((target, seconds, len(request)))
    return written


def dump_chain(mesh: str, patch: int, out: Path) -> list:
    """Запросы шва полевых вызовов одного патча по возрастанию alpha: шаги ползунка ширины одним сеансом (`clip_profile --chain`)."""

    from cftuv_native import clip_seams as wire

    chain = field_chains(2).get((mesh, patch))
    if not chain:
        raise SystemExit(f"в полевом корпусе нет цепочки {mesh} патч {patch}")
    out.mkdir(parents=True, exist_ok=True)
    written = []
    for number, path in enumerate(chain):
        record = nc.read_record(path)
        before = record.before()
        call = nc.decode_call(nc.OP_CLIP, record.call_blob, nc.build_budget(before.budget), None)
        header = wire.full_header(before.budget, before.known_primes, before.factorization, before.squarefree, before.prime_support)
        target = out / f"{number:03d}-{mesh}-p{patch}.req"
        target.write_bytes(wire.request_bytes("CLIP_GEOMETRY", wire.enc_geometry(call.args[0], call.kwargs), header))
        written.append(target)
    return written


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("command", choices=("compare", "timing", "dump", "chain"))
    parser.add_argument("target", nargs="?", default=None)
    parser.add_argument("mesh", nargs="?", default=None)
    parser.add_argument("patch", nargs="?", type=int, default=None)
    parser.add_argument("--stride", type=int, default=1)
    parser.add_argument("--top", type=int, default=6)
    arguments = parser.parse_args(argv)
    runner = WholeRunner()
    if arguments.command == "compare":
        sources = {"field": field_paths(arguments.stride), "derived": derived_paths(), "synthetic": synthetic_paths()}
        bad = 0
        for name, paths in sources.items():
            runs = [(path.name, runner.compare(nc.read_record(path))) for path in paths]
            failed = [(label, run) for label, run in runs if not run.equal]
            print(f"{name}: {len(runs) - len(failed)}/{len(runs)} equal")
            if failed:
                print(explain(failed))
            bad += len(failed)
        return 1 if bad else 0
    if arguments.command == "timing":
        warm, cold, bad = run_timing(field_paths(arguments.stride), runner)
        print("warm caches (a long-lived process: oracle products cache kept, native session kept):")
        print(timing_table(warm))
        print("cold caches (oracle restore clears them, native: a new session per call):")
        print(timing_table(cold))
        print(f"python {sys.version.split()[0]}; records with a difference: {bad}")
        return 1 if bad else 0
    if arguments.command == "chain":
        for target in dump_chain(arguments.mesh, arguments.patch, Path(arguments.target)):
            print(target)
        return 0
    written = dump_requests(field_paths(), Path(arguments.target), arguments.top)
    for target, seconds, size in written:
        print(f"{target} {seconds * 1e3:.1f} ms oracle, {size} bytes")
    return 0


if __name__ == "__main__":
    sys.exit(main())
