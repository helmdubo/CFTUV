"""Замер нативных ЦЕЛЫХ операций ядра против эталона на Python на корпусе вызовов (`tools/native_corpus_export.py`): `coverage._coverage_at`, `clip.clip_geometry` и `skeleton.build_skeleton`.

    set PYTHONSAFEPATH=1
    python tools/native_bench_native.py [--op coverage|clip|both|skeleton] [--corpus <каталог корпуса>] [--repeat 3] [--meshes building,...] [--limit N] [--out <json>]
    "C:/Program Files/Blender Foundation/Blender 4.5/4.5/python/bin/python.exe" tools/native_bench_native.py ...   (питон 3.11 продукта;
        расширение: `pip install --target ~/.cftuv-native/py311-site <колесо>` и `PYTHONPATH=~/.cftuv-native/py311-site`)

СКЕЛЕТ (`--op skeleton`) меряет `tools/native_skeleton_whole.py bench` (корпус скелета, не этот): целый вызов `build_skeleton` через вставку, p50/p95/max по мешам и пять самых тяжёлых доменов.

КАЖДЫЙ нативный вызов сверяется с эталоном точно (`native_corpus.compare_outcomes`: результат с различием `int`/`Fraction`, исключение, цена, память с порядком,
счётчики знаков, неоплаченное, `store`, нормали плоскости); расхождение — отказ замера (код 1). Нет расширения `cftuv_native` или порт устарел относительно дерева ядра
(`native_status()`) — отказ (код 2), отката на питон нет.

ПОКРЫТИЕ. Для каждой записи цепочка шагов идёт от ОДНОГО состояния («до» записи): шаг 0 — сама запись (ХОЛОДНЫЙ вызов: разбиение переводится в нативную сессию, `store` — промах,
если он пуст), шаги 1.. — тот же `partition` с другими alpha на той же сессии, том же бюджете и том же `store` (ТЁПЛЫЕ шаги: попадание в `store`, память канонизации тёплая; именно их делает
ползунок ширины). Время шага — одна операция (`time.perf_counter` вокруг вызова), снимок состояния снимается ВНЕ замера. Время нативного вызова раскладывается (миллисекунды):

* `partition` — перевод разбиения в сессию (один раз на разбиение; только холодный вызов; привязка классов к сеансу — один раз на процесс — в замер не входит, как у резки);
* `args` — на вызов: синхронизация памяти и бюджета в шиме (`sync`) и разбор аргументов в расширении (alpha, поиск в `store`, заголовок стоимости);
* `compute` — сама операция внутри Rust (замер внутри расширения, GIL отпущен);
* `result` — на возврат: построение `CoverageV1` и запись `store` в Rust (`build`), применение журнала памяти к настоящим таблицам (Rust, `log`), статьи бюджета, счётчики и вид сеанса на таблицы в шиме (`post`);
* `other` — остальное: разбор аргументов PyO3, возврат кортежа, накладные `perf_counter`;
* `total` — стенка всего вызова, как её видит вызывающий.

РЕЗКА. Одна запись — один вызов, поэтому «холодный» и «тёплый» здесь про сеанс и плоскость, а не про alpha. ХОЛОДНЫЙ: эталон на восстановленном состоянии с чистыми кэшами; нативный — НОВЫЙ сеанс
(пустое зеркало памяти и кэш плоскостей: перевод треугольников, полная загрузка таблиц памяти) и новая плоскость. ТЁПЛЫЙ: тот же вызов повторён на ТОЙ ЖЕ плоскости (эталон — с тёплыми чистыми кэшами:
произведения радикандов, центры binary64; нативный — на живой сеансе с переведённой плоскостью), состояние процесса и нормали плоскости восстановлены до записанных, как перед вызовом в поле. ВСЁ,
что вызывающий оплачивает, входит в нативное время: синхронизация памяти (`sync`), перевод плоскости (`plane`), разбор аргументов (`args`), вычисление (`compute`), построение результата и запись
нормалей плоскости (`result`), применение журнала памяти, статей, счётчиков (`post`), остаток (`other`: разбор аргументов PyO3, `perf_counter`). Каждый нативный вызов, холодный и тёплый, сверяется с
холодным эталоном; тёплый эталон тоже (цена и ответ не зависят от чистых кэшей).
"""

from __future__ import annotations

import argparse
import gc
import json
import statistics
import sys
import time
from fractions import Fraction
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))  # `PYTHONSAFEPATH=1` каталог скрипта в путь не кладёт

import native_bench as nb  # noqa: E402  (путь к mpmath/sympy под питоном Blender; импортирует `native_corpus`)
import native_corpus as nc  # noqa: E402

#: Множители alpha тёплых шагов: ползунок ширины около записанной alpha (вперёд и назад, мелкие и крупные шаги).
WARM_FACTORS = (Fraction(15, 16), Fraction(17, 16), Fraction(7, 8), Fraction(9, 8), Fraction(3, 4), Fraction(5, 4), Fraction(1, 2))
PARTS = ("partition", "args", "compute", "result", "other", "total")


def load_extension(*operations: str):
    """Нативный шим либо отказ замера: тихого отката на питон нет, а порт, устаревший относительно дерева ядра (`native_status()`), не мерится."""

    try:
        import cftuv_native
    except ModuleNotFoundError as error:
        if error.name != "cftuv_native":
            raise
        print("NATIVE_BENCH_NATIVE_FAILED the cftuv_native extension is not importable (python tools/native_build.py; for 3.11: pip install --target ~/.cftuv-native/py311-site <wheel>)")
        raise SystemExit(2)
    status = cftuv_native.native_status()
    for operation in operations or ("coverage",):
        if status[operation] != "available":
            print(f"NATIVE_BENCH_NATIVE_FAILED the native {operation} port is {status[operation]}: it was compared with another version of the Python oracle (cftuv_native.pin); refusing to measure it")
            raise SystemExit(2)
    return cftuv_native


def _run(function, call):
    """Результат либо исключение операции (тип и текст) и секунды вокруг одного вызова."""

    gc.collect()
    started = time.perf_counter()
    try:
        result, error = function(call), None
    except Exception as exc:  # noqa: BLE001 - исключение операции — часть её исхода
        result, error = None, (type(exc).__qualname__, str(exc))
    return result, error, time.perf_counter() - started


def oracle_chain(blob, before, alphas) -> list:
    """Эталон по цепочке alpha на одном бюджете и одном `store`: `[(Outcome, секунды)]`."""

    call = nc.prepare_call(nc.OP_COVERAGE, blob, before)
    partition = call.args[0]
    steps = []
    for alpha in alphas:
        step = nc.Call(nc.OP_COVERAGE, (partition, alpha), {}, call.budget, call.store)
        result, error, seconds = _run(lambda item: nc.ORACLE[nc.OP_COVERAGE](item.args[0], item.args[1], item.budget, item.store), step)
        steps.append((nc.Outcome(result, error, nc.capture_state(call.budget, call.store), {}, seconds), seconds))
    return steps


def native_chain(mirror, blob, before, alphas) -> list:
    """Нативная сторона по той же цепочке: `[(Outcome, секунды, части времени в секундах)]`."""

    call = nc.prepare_call(nc.OP_COVERAGE, blob, before)
    partition = call.args[0]
    mirror._bind_coverage()  # once per session in a product (classes, layout probes), not a cost of the first call on a partition
    steps = []
    for alpha in alphas:
        step = nc.Call(nc.OP_COVERAGE, (partition, alpha), {}, call.budget, call.store)
        result, error, seconds = _run(lambda item: mirror.coverage_at(item.args[0], item.args[1], item.budget, item.store), step)
        timings = mirror.last_timings
        parts = {"partition": timings[4], "args": timings[0] + timings[5], "compute": timings[6], "result": timings[7] + sum(timings[8:]) + timings[2]}
        parts = {name: value * 1e-9 for name, value in parts.items()}
        parts["other"] = max(0.0, seconds - sum(parts.values()))
        parts["total"] = seconds
        if error is not None:  # a raised call leaves the timings of the call before it
            parts = {name: 0.0 for name in PARTS} | {"total": seconds}
        steps.append((nc.Outcome(result, error, nc.capture_state(call.budget, call.store), {}, seconds), seconds, parts))
    return steps


def chain_alphas(alpha) -> list:
    """Шаг 0 — alpha записи (как есть), дальше — тёплые шаги вокруг неё."""

    return [alpha] + [alpha * factor for factor in WARM_FACTORS]


def measure_record(extension, root: Path, row: dict, repeat: int) -> dict:
    """Одна запись: `repeat` проходов цепочки эталоном и нативной стороной, медиана по проходам для каждого шага; сверка КАЖДОГО шага."""

    record = nc.read_record(root / row["path"])
    before = record.before()
    alpha = nc.decode_call(nc.OP_COVERAGE, record.call_blob, None, None).args[1]
    alphas = chain_alphas(alpha)
    oracle_seconds = [[] for _ in alphas]
    native_seconds = [[] for _ in alphas]
    native_parts = [{name: [] for name in PARTS} for _ in alphas]
    differences: list = []
    raised = [False] * len(alphas)
    for index in range(repeat):
        expected = oracle_chain(record.call_blob, before, alphas)
        mirror = extension.new_mirror()
        actual = native_chain(mirror, record.call_blob, before, alphas)
        for step, ((want, seconds_o), (got, seconds_n, parts)) in enumerate(zip(expected, actual)):
            raised[step] = raised[step] or want.exception is not None
            oracle_seconds[step].append(seconds_o)
            native_seconds[step].append(seconds_n)
            for name in PARTS:
                native_parts[step][name].append(parts[name])
            found = nc.compare_outcomes(nc.OP_COVERAGE, before, want, got)
            if found:
                differences.append({"repeat": index + 1, "step": step, "fields": [str(item) for item in found][:4]})
    steps = [
        {
            "oracle": statistics.median(oracle_seconds[step]),
            "native": statistics.median(native_seconds[step]),
            "parts": {name: statistics.median(native_parts[step][name]) for name in PARTS},
            "raised": raised[step],
        }
        for step in range(len(alphas))
    ]
    return {"id": row["id"], "mesh": row["mesh"], "patch_id": row["patch_id"], "alpha": row["alpha"], "faces": row.get("faces"), "steps": steps, "differences": differences}


def aggregate(results: list) -> dict:
    """`{холодный|тёплый: {меш|ALL: {эталон, нативный, ускорение, части}}}` по записям: p50/p95/max."""

    table: dict = {"cold": {}, "warm": {}}
    for result in results:
        cold = [result["steps"][0]] if not result["steps"][0]["raised"] else []
        warm = [step for step in result["steps"][1:] if not step["raised"]]
        for mesh in (result["mesh"], "ALL"):
            table["cold"].setdefault(mesh, []).extend(cold)
            table["warm"].setdefault(mesh, []).extend(warm)
    return {kind: {mesh: _summary(items) for mesh, items in sorted(meshes.items())} for kind, meshes in table.items()}


def _summary(items: list) -> dict:
    parts = {name: nb.summarize([item["parts"][name] for item in items]) for name in PARTS}
    return {
        "n": len(items),
        "oracle": nb.summarize([item["oracle"] for item in items]),
        "native": nb.summarize([item["native"] for item in items]),
        "speedup_p50": nb.percentile([item["oracle"] / item["native"] for item in items], 0.5),
        "speedup_min": min(item["oracle"] / item["native"] for item in items),
        "parts": parts,
    }


def heaviest(results: list, count: int = 10) -> list:
    """Самые тяжёлые записи по времени эталона на холодном вызове."""

    ranked = sorted((item for item in results if not item["steps"][0]["raised"]), key=lambda item: -item["steps"][0]["oracle"])[:count]
    rows = []
    for item in ranked:
        warm = [step for step in item["steps"][1:] if not step["raised"]] or item["steps"][1:]
        rows.append({
            "id": item["id"], "mesh": item["mesh"], "patch_id": item["patch_id"], "faces": item["faces"],
            "cold_oracle": item["steps"][0]["oracle"], "cold_native": item["steps"][0]["native"], "cold_parts": item["steps"][0]["parts"],
            "warm_oracle": statistics.median(step["oracle"] for step in warm), "warm_native": statistics.median(step["native"] for step in warm),
            "warm_parts": {name: statistics.median(step["parts"][name] for step in warm) for name in PARTS},
        })
    return rows


def _ms(seconds: float) -> str:
    return f"{seconds * 1e3:8.3f}"


def text_report(report: dict) -> str:
    lines = [f"python {report['python']}  native {report['native_version']}  corpus {report['corpus']}  records {report['records']}  repeat {report['repeat']}  warm steps/record {len(WARM_FACTORS)}", ""]
    for kind in ("cold", "warm"):
        lines.append(f"{kind.upper()} ({'first call on a partition: conversion + store miss' if kind == 'cold' else 'later alphas on the same session: converted partition, store hit'}); times in ms")
        lines.append(f"{'mesh':<26}{'n':>5}  {'oracle p50':>10}{'p95':>9}{'max':>9}  {'native p50':>10}{'p95':>9}{'max':>9}  {'speedup p50':>11}{'min':>7}")
        for mesh, row in report["stats"][kind].items():
            o, n = row["oracle"], row["native"]
            lines.append(f"{mesh:<26}{row['n']:>5}  {_ms(o['p50']):>10}{_ms(o['p95']):>9}{_ms(o['max']):>9}  {_ms(n['p50']):>10}{_ms(n['p95']):>9}{_ms(n['max']):>9}  {row['speedup_p50']:>10.1f}x{row['speedup_min']:>6.1f}x")
        lines.append(f"  native split p50/p95/max (ms), ALL:  " + "   ".join(f"{name} {_ms(row['p50']).strip()}/{_ms(row['p95']).strip()}/{_ms(row['max']).strip()}" for name, row in report["stats"][kind]["ALL"]["parts"].items()))
        lines.append("")
    lines.append("heaviest records (by the oracle's cold time); ms")
    lines.append(f"{'record':<34}{'mesh':<24}{'faces':>6}  {'cold oracle':>11}{'native':>9}{'x':>7}   {'warm oracle':>11}{'native':>9}{'x':>7}   warm split: " + " ".join(PARTS))
    for row in report["heaviest"]:
        split = " ".join(_ms(row["warm_parts"][name]).strip() for name in PARTS)
        lines.append(
            f"{row['id']:<34}{row['mesh']:<24}{row['faces'] or 0:>6}  {_ms(row['cold_oracle']):>11}{_ms(row['cold_native']):>9}{row['cold_oracle'] / row['cold_native']:>6.1f}x   "
            f"{_ms(row['warm_oracle']):>11}{_ms(row['warm_native']):>9}{row['warm_oracle'] / row['warm_native']:>6.1f}x   {split}"
        )
    return "\n".join(lines)


def dump_partition(root: Path, row: dict, path: Path) -> None:
    """Разбиение и alpha записи в буфере границы (`cftuv_native.codec`) для `cargo run --release --example coverage_profile -p cftuv-core -- <файл>`.

    Содержимое: `[alpha, [[[ [x, y], ... ], [a, b, c, q]], ... грани]]`, `x`, `y` — суммы корней с типами коэффициентов (`int`/`Fraction`).
    """

    from cftuv_native import codec

    record = nc.read_record(root / row["path"])
    partition, alpha = nc.decode_call(nc.OP_COVERAGE, record.call_blob, None, None).args
    faces = [[[list(point) for point in face.points], [face.line.a, face.line.b, face.line.c, face.line.q]] for face in partition.faces]
    path.write_bytes(codec.encode_value([alpha, faces]))
    print(f"dumped {row['id']}: {len(faces)} faces, {path.stat().st_size} bytes -> {path}")


# --------------------------------------------------------------------------
# clip_geometry: вставка целиком, холодный и тёплый вызов
# --------------------------------------------------------------------------

CLIP_PARTS = ("sync", "plane", "args", "compute", "result", "log", "post", "other", "total")
#: Холодный и тёплый вызов эталона и нативного пути. Тёплый нативный: плоскость уже переведена, кэш результатов прошлых вызовов сброшен (ни один результат не переиспользуется).
CLIP_KINDS = ("oracle_cold", "oracle_warm", "native_cold", "native_warm")


def restore_warm(before: "nc.StateV1"):
    """Процесс в состоянии «до» БЕЗ сброса чистых кэшей (центры binary64, произведения радикандов): так выглядит повтор вызова в живом процессе."""

    exact = nc.exact
    for table in (exact._KNOWN_PRIMES, exact._KNOWN_PRIME_SET, exact._FACTORIZATION_MEMO, exact._SQUAREFREE_MEMO, exact._PRIME_SUPPORT_MEMO):
        table.clear()
    exact._KNOWN_PRIMES.extend(before.known_primes)
    exact._KNOWN_PRIME_SET.update(before.known_primes)
    exact._FACTORIZATION_MEMO.update(before.factorization)
    exact._SQUAREFREE_MEMO.update(before.squarefree)
    exact._PRIME_SUPPORT_MEMO.update(before.prime_support)
    exact.SIGN_COUNTS.update(before.sign_counts)
    return nc.build_budget(before.budget)


class PersistentCall:
    """Один вызов резки, собранный ОДИН раз: плоскость, аргументы, исходные нормали плоскости; `rewind` ставит процесс и нормали в состояние «до»."""

    def __init__(self, record: "nc.Record", before: "nc.StateV1") -> None:
        self.before = before
        self.call = nc.decode_call(nc.OP_CLIP, record.call_blob, restore_warm(before), None)
        self.normals = dict(self.call.args[0]._normal_by_position)

    def rewind(self) -> "nc.Call":
        budget = restore_warm(self.before)
        plane = self.call.args[0]
        plane._normal_by_position.clear()
        plane._normal_by_position.update(self.normals)
        return nc.Call(nc.OP_CLIP, self.call.args, self.call.kwargs, budget, None)


def clip_native_parts(mirror, seconds: float) -> dict:
    """Части нативного времени одного вызова в секундах: шим (`sync`, `post`) и расширение (`plane`, `args`, `compute`, `result`); `other` — остаток."""

    timings = mirror.last_clip_timings
    parts = {"sync": timings[0], "plane": timings[4], "args": timings[5], "compute": timings[6], "result": timings[7], "log": timings[8], "post": timings[2]}
    parts = {name: value * 1e-9 for name, value in parts.items()}
    parts["other"] = max(0.0, seconds - sum(parts.values()))
    parts["total"] = seconds
    return parts


def _timed(function, call) -> "nc.Outcome":
    """`Outcome` вызова (секунды в нём — вокруг самого вызова; сборка мусора перед замером, снимок состояния ПОСЛЕ него)."""

    gc.collect()
    return nc.execute(call, function=function)


def clip_group(record: "nc.Record") -> str:
    """Меш; патч 2 `rounded_wall_noise_top` и каждая из пяти записей `building` названы отдельно."""

    meta = record.meta
    if meta["mesh"] == "rounded_wall_noise_top" and meta.get("patch_id") == 2:
        return "rounded_wall_noise_top patch 2"
    if meta["mesh"] == "building":
        return f"building patch {meta.get('patch_id')}"
    return str(meta["mesh"])


def measure_clip_record(extension, path: Path, repeat: int) -> dict:
    """Одна запись резки: `repeat` вызовов каждого вида, медианы; каждый нативный и тёплый эталонный вызов сверен с холодным эталоном."""

    record = nc.read_record(path)
    before = record.before()
    expected = None
    samples = {kind: [] for kind in CLIP_KINDS}
    parts = {kind: {name: [] for name in CLIP_PARTS} for kind in ("native_cold", "native_warm")}
    differences: list = []

    def check(kind: str, outcome: "nc.Outcome", index: int) -> None:
        found = nc.compare_outcomes(nc.OP_CLIP, before, expected, outcome)
        if found:
            differences.append({"kind": kind, "repeat": index + 1, "fields": [str(item) for item in found][:4]})

    for index in range(repeat):
        cold = nc.execute(nc.prepare_call(nc.OP_CLIP, record.call_blob, before))
        expected = expected or cold
        samples["oracle_cold"].append(cold.seconds)
        mirror = extension.new_mirror()
        mirror._bind_clip()  # once per session in a product, not a cost of the first call on a plane
        native = _timed(mirror.clip_geometry, nc.prepare_call(nc.OP_CLIP, record.call_blob, before))
        samples["native_cold"].append(native.seconds)
        for name, value in clip_native_parts(mirror, native.seconds).items():
            parts["native_cold"][name].append(value)
        check("native_cold", native, index)
    persistent = PersistentCall(record, before)
    warm = extension.new_mirror()
    warm._bind_clip()
    for step in range(repeat + 1):
        oracle = _timed(None, persistent.rewind())
        warm.clear_clip_warm()  # the plane stays converted; no result of an earlier call is reused (that is the chain below)
        native = _timed(warm.clip_geometry, persistent.rewind())
        if step == 0:
            continue  # the first pass fills the caches of both sides: the warm numbers start at the second
        samples["oracle_warm"].append(oracle.seconds)
        samples["native_warm"].append(native.seconds)
        for name, value in clip_native_parts(warm, native.seconds).items():
            parts["native_warm"][name].append(value)
        check("oracle_warm", oracle, step - 1)
        check("native_warm", native, step - 1)
    meta = record.meta
    return {
        "id": path.name, "mesh": meta["mesh"], "patch_id": meta.get("patch_id"), "alpha": meta.get("alpha"), "group": clip_group(record),
        "recorded_seconds": meta.get("seconds"), "differences": differences, "outcome": "raised" if expected.exception else "clipped",
        **{kind: statistics.median(values) for kind, values in samples.items()},
        "parts": {kind: {name: statistics.median(values) for name, values in table.items()} for kind, table in parts.items()},
    }



def measure_clip_chain(extension, paths: list, repeat: int) -> dict:
    """Шаги ползунка: полевые вызовы одного патча по возрастанию alpha одним сеансом и ОДНОЙ плоскостью, кэш результатов между шагами сохранён.

    `{имя файла: секунды шага}` (медиана по `repeat` проходов; в первом шаге цепочки — и перевод плоскости) и расхождения: каждый шаг сверен с холодным
    эталоном ЭТОЙ записи. Состояние процесса и нормали плоскости на каждом шаге — записанные «до», как перед вызовом в поле."""

    records = [nc.read_record(path) for path in paths]
    befores = [record.before() for record in records]
    expected = [nc.execute(nc.prepare_call(nc.OP_CLIP, record.call_blob, before)) for record, before in zip(records, befores)]
    plane = nc.decode_call(nc.OP_CLIP, records[0].call_blob, restore_warm(befores[0]), None).args[0]
    seconds = {path.name: [] for path in paths}
    differences: list = []
    for index in range(repeat):
        mirror = extension.new_mirror()
        mirror._bind_clip()
        for path, record, before, want in zip(paths, records, befores, expected):
            budget = restore_warm(before)
            own = nc.decode_call(nc.OP_CLIP, record.call_blob, budget, None)
            plane._normal_by_position.clear()
            plane._normal_by_position.update(own.args[0]._normal_by_position)
            outcome = _timed(mirror.clip_geometry, nc.Call(nc.OP_CLIP, (plane,), own.kwargs, budget, None))
            seconds[path.name].append(outcome.seconds)
            found = nc.compare_outcomes(nc.OP_CLIP, before, want, outcome)
            if found:
                differences.append({"kind": "native_chain", "repeat": index + 1, "id": path.name, "fields": [str(item) for item in found][:4]})
    return {"seconds": {name: statistics.median(values) for name, values in seconds.items()}, "differences": differences}


def _ratios(items: list, numerator: str, denominator: str) -> dict:
    values = [item[numerator] / item[denominator] for item in items]
    return {"p50": nb.percentile(values, 0.5), "min": min(values)}


def clip_summary(items: list) -> dict:
    """Для набора записей: p50/p95/max каждого вида, отношения медиан по записям, отношение статистик, части нативного времени."""

    stats = {kind: nb.summarize([item[kind] for item in items]) for kind in CLIP_KINDS}
    chained = [item for item in items if item.get("native_chain") is not None]
    chain = None
    if chained:
        chain_stats = {"native_chain": nb.summarize([item["native_chain"] for item in chained]), "oracle_warm": nb.summarize([item["oracle_warm"] for item in chained])}
        chain = {
            "n": len(chained), **chain_stats, "speedup": _ratios(chained, "oracle_warm", "native_chain"),
            "ratio": {key: chain_stats["oracle_warm"][key] / chain_stats["native_chain"][key] for key in ("p50", "p95", "max")},
        }
    return {
        "n": len(items), **stats, "chain": chain,
        "speedup_cold": _ratios(items, "oracle_cold", "native_cold"), "speedup_warm": _ratios(items, "oracle_warm", "native_warm"),
        "ratio_cold": {key: stats["oracle_cold"][key] / stats["native_cold"][key] for key in ("p50", "p95", "max")},
        "ratio_warm": {key: stats["oracle_warm"][key] / stats["native_warm"][key] for key in ("p50", "p95", "max")},
        "parts": {kind: {name: nb.summarize([item["parts"][kind][name] for item in items]) for name in CLIP_PARTS} for kind in ("native_cold", "native_warm")},
    }


def clip_aggregate(results: list) -> dict:
    """`{меш или группа | ALL: сводка}`: меши, `rounded_wall_noise_top patch 2` и пять записей `building` отдельными строками."""

    groups: dict = {"ALL": []}
    for item in results:
        groups["ALL"].append(item)
        groups.setdefault(item["group"], []).append(item)
        if item["mesh"] != item["group"]:
            groups.setdefault(item["mesh"], []).append(item)
    return {name: clip_summary(items) for name, items in sorted(groups.items())}


def clip_text(report: dict) -> str:
    lines = [f"clip_geometry whole operation: python {report['python']}  native {report['native_version']}  records {report['records']}  repeat {report['repeat']}  calls checked {report['calls_checked']}", ""]
    for kind, left, right in (("COLD", "oracle_cold", "native_cold"), ("WARM", "oracle_warm", "native_warm")):
        lines.append(f"{kind} ({'new session and a new plane: conversion + full memory load' if kind == 'COLD' else 'same plane and live session, state restored'}); ms; x = oracle statistic / native statistic")
        lines.append(f"{'group':<32}{'n':>4}  {'oracle p50':>10}{'p95':>9}{'max':>9}  {'native p50':>10}{'p95':>9}{'max':>9}  {'x p50':>7}{'x p95':>7}{'x max':>7}  {'per-record x p50':>16}{'min':>7}")
        for name, row in report["stats"].items():
            o, n, r = row[left], row[right], row["ratio_cold" if kind == "COLD" else "ratio_warm"]
            per = row["speedup_cold" if kind == "COLD" else "speedup_warm"]
            lines.append(
                f"{name:<32}{row['n']:>4}  {_ms(o['p50']):>10}{_ms(o['p95']):>9}{_ms(o['max']):>9}  {_ms(n['p50']):>10}{_ms(n['p95']):>9}{_ms(n['max']):>9}"
                f"  {r['p50']:>6.1f}x{r['p95']:>6.1f}x{r['max']:>6.1f}x  {per['p50']:>15.1f}x{per['min']:>6.1f}x"
            )
        for label in ("ALL", "rounded_wall_noise_top patch 2"):
            row = report["stats"].get(label)
            if row:
                split = row["parts"]["native_cold" if kind == "COLD" else "native_warm"]
                lines.append(f"  native split p50/p95/max ms, {label}:  " + "   ".join(f"{name} {_ms(part['p50']).strip()}/{_ms(part['p95']).strip()}/{_ms(part['max']).strip()}" for name, part in split.items()))
        lines.append("")
    lines.append("CHAIN (neighbouring alphas of one patch, one session, one plane, the cross-call cache kept; the first step of a chain also converts the plane); ms; x = oracle warm / native chain")
    lines.append(f"{'group':<32}{'n':>4}  {'oracle p50':>10}{'p95':>9}{'max':>9}  {'native p50':>10}{'p95':>9}{'max':>9}  {'x p50':>7}{'x p95':>7}{'x max':>7}  {'per-record x p50':>16}{'min':>7}")
    for name, row in report["stats"].items():
        chain = row["chain"]
        if chain:
            o, n, r, per = chain["oracle_warm"], chain["native_chain"], chain["ratio"], chain["speedup"]
            lines.append(
                f"{name:<32}{chain['n']:>4}  {_ms(o['p50']):>10}{_ms(o['p95']):>9}{_ms(o['max']):>9}  {_ms(n['p50']):>10}{_ms(n['p95']):>9}{_ms(n['max']):>9}"
                f"  {r['p50']:>6.1f}x{r['p95']:>6.1f}x{r['max']:>6.1f}x  {per['p50']:>15.1f}x{per['min']:>6.1f}x"
            )
    lines.append("")
    return "\n".join(lines)


def run_clip(extension, args) -> tuple:
    """`(report, число записей с расхождением)` по записям резки полевого корпуса."""

    import native_clip_geometry as geometry

    paths = geometry.field_paths()
    meshes = set(filter(None, args.meshes.split(",")))
    results = []
    for number, path in enumerate(paths, 1):
        if meshes and nc.read_meta(path)["mesh"] not in meshes:
            continue
        if args.limit and len(results) >= args.limit:
            break
        results.append(measure_clip_record(extension, path, args.repeat))
        if results[-1]["differences"]:
            first = results[-1]["differences"][0]
            print(f"MISMATCH {path.name} {results[-1]['group']} {first['kind']} repeat {first['repeat']}: {first['fields']}", flush=True)
        if number % 25 == 0:
            print(f"  clip {number}/{len(paths)} records", flush=True)
    by_name = {item["id"]: item for item in results}
    chain_calls = 0
    for chain in geometry.field_chains(3).values():
        chain = [path for path in chain if path.name in by_name]
        if len(chain) < 3:
            continue
        measured = measure_clip_chain(extension, chain, args.repeat)
        chain_calls += len(chain) * args.repeat
        for name, value in measured["seconds"].items():
            by_name[name]["native_chain"] = value
        for found in measured["differences"]:
            by_name[found["id"]]["differences"].append(found)
            print(f"MISMATCH {found['id']} chain repeat {found['repeat']}: {found['fields']}", flush=True)
    mismatches = [item for item in results if item["differences"]]
    report = {
        "python": sys.version.split()[0], "native_version": extension.native_version(), "records": len(results), "repeat": args.repeat,
        "calls_checked": len(results) * args.repeat * 3 + chain_calls, "stats": clip_aggregate(results),
        "mismatches": [{"id": item["id"], "differences": item["differences"]} for item in mismatches], "per_record": results,
    }
    return report, len(mismatches)


def _arguments():
    parser = argparse.ArgumentParser()
    parser.add_argument("--op", choices=("coverage", "clip", "both", "skeleton"), default="both")
    parser.add_argument("--dump", default="", help="id записи: записать её разбиение и alpha в файл `--dump-to` и выйти")
    parser.add_argument("--dump-to", default="")
    parser.add_argument("--corpus", default="")
    parser.add_argument("--repeat", type=int, default=3)
    parser.add_argument("--meshes", default="")
    parser.add_argument("--limit", type=int, default=0)
    parser.add_argument("--out", default="")
    return parser.parse_args()


def run_coverage(extension, args, root: Path, index: dict) -> tuple:
    """`(report, число записей с расхождением)` по записям покрытия корпуса."""

    meshes = set(filter(None, args.meshes.split(",")))
    rows = [row for row in index["records"] if row["op"] == nc.OP_COVERAGE and not row.get("derived") and (not meshes or row["mesh"] in meshes) and not row["exception"]]
    rows = rows[: args.limit] if args.limit else rows
    results = []
    for number, row in enumerate(rows, 1):
        results.append(measure_record(extension, root, row, args.repeat))
        if results[-1]["differences"]:
            first = results[-1]["differences"][0]
            print(f"MISMATCH {row['id']} {row['mesh']} step {first['step']} repeat {first['repeat']}: {first['fields']}", flush=True)
        if number % 50 == 0:
            print(f"  {number}/{len(rows)} records", flush=True)
    mismatches = [item for item in results if item["differences"]]
    report = {
        "python": sys.version.split()[0], "native_version": extension.native_version(), "corpus": str(root), "records": len(results), "repeat": args.repeat,
        "calls_checked": sum(len(item["steps"]) for item in results) * args.repeat, "stats": aggregate(results), "heaviest": heaviest(results),
        "mismatches": [{"id": item["id"], "differences": item["differences"]} for item in mismatches], "per_record": results,
    }
    return report, len(mismatches)


def main() -> int:
    args = _arguments()
    if args.op == "skeleton":
        import native_skeleton_whole as whole

        return whole.main(["bench", "--corpus", args.corpus or "field", "--repeat", str(args.repeat), "--meshes", args.meshes, "--limit", str(args.limit), "--out", args.out])
    operations = ("coverage", "clip") if args.op == "both" else (args.op,)
    extension = load_extension(*operations)
    root = Path(args.corpus) if args.corpus else nb.default_corpus()
    index = nc.load_index(root)
    if args.dump:
        dump_partition(root, next(row for row in index["records"] if row["id"] == args.dump), Path(args.dump_to))
        return 0
    reports: dict = {}
    failed = 0
    if "coverage" in operations:
        reports["coverage"], bad = run_coverage(extension, args, root, index)
        print(text_report(reports["coverage"]))
        print(f"NATIVE_BENCH_NATIVE_{'FAILED' if bad else 'OK'} op=coverage records={reports['coverage']['records']} native_calls_checked={reports['coverage']['calls_checked']} mismatches={bad}")
        failed += bad
    if "clip" in operations:
        reports["clip"], bad = run_clip(extension, args)
        print(clip_text(reports["clip"]))
        print(f"NATIVE_BENCH_NATIVE_{'FAILED' if bad else 'OK'} op=clip records={reports['clip']['records']} native_calls_checked={reports['clip']['calls_checked']} mismatches={bad}")
        failed += bad
    if args.out:
        Path(args.out).write_text(json.dumps(reports, ensure_ascii=False, indent=1) + "\n", encoding="utf-8")
    return 1 if failed else 0


if __name__ == "__main__":
    code = main()
    if code:
        raise SystemExit(code)
