"""Атрибуция цены по МЕСТАМ ВЫЗОВА для среза 6 (числовое представление).

Три режима; все считают один домен тем же маршрутом, что и ворота (`gate.compute_row`).

    python attribute.py profile <patch> <density> <out.pstats> [<row.json>]
        cProfile одного домена целиком; `row.json` — ответ домена (сверка с baseline).
    python attribute.py aggregate <out.json> <p6.pstats> <p1.pstats> ...
        Таблицы по местам вызова из снятых pstats (без пересчёта):
          sympy  — каждая ТОЧКА ВХОДА в sympy (функция sympy, вызванная из НЕ-sympy
                   кода) с вызывающим местом ядра (`файл:строка(функция)`), числом
                   вызовов и накопленными секундами ребра (caller -> callee);
          fractions — `Fraction` (конструктор/арифметика/сравнение/repr/hash) по
                   вызывающим местам ядра, топ-15;
          sqrt_sum — публичные функции `exact_sqrt_sum.py` по вызывающим местам
                   вне этого модуля, топ-10.
        Встроенные и служебные вызывающие (`sorted`, `sum`, `repr`, `<string>`
        датаклассов) не прячут автора: вес ребра переносится на вызывающих их
        пропорционально накопленному времени (до трёх прыжков).
    python attribute.py instrument <patch> <density> <out.json>
        ЗНАЧЕНИЯ и память: обёртки (monkeypatch в процессе, файлы не правятся) на
        входах sympy-функций и точных предикатов считают, что приходит — рациональное,
        сумма корней, угол/пи — и кто зовёт; счётчики попаданий `_DensityExactMemo`.
"""

from __future__ import annotations

import cProfile
import collections
import json
import os
import pstats
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import gate  # noqa: E402
import pool_sweep  # noqa: E402

WARMUP_PATCH = 100  # малый домен (~1 с), см. `cmd_profile`

# ---------------------------------------------------------------- классификация


def norm(filename: str) -> str:
    return filename.replace("\\", "/")


def kind_of(filename: str) -> str:
    n = norm(filename)
    if n == "~":
        return "builtin"
    if "/sympy/" in n:
        return "sympy"
    if "/mpmath/" in n:
        return "mpmath"
    if n.endswith("/fractions.py"):
        return "fractions"
    if "/cftuv_envelope/" in n:
        return "kernel"
    if "/cftuv/" in n:
        return "host"
    if "/artifacts/" in n or "/perf_prepare_diag/" in n:
        return "harness"
    return "other"


def site(func) -> str:
    filename, lineno, name = func
    n = norm(filename)
    if n == "~":
        return f"<builtin>{name}"
    if "/cftuv_envelope/" in n:
        n = n.split("/cftuv_envelope/", 1)[1]
    elif "/cftuv/" in n:
        n = "host:" + n.split("/cftuv/", 1)[1]
    elif "/sympy/" in n:
        n = "sympy/" + n.split("/sympy/", 1)[1]
    else:
        n = "/".join(n.split("/")[-2:])
    return f"{n}:{lineno}({name})"


def sympy_family(func) -> str:
    filename, _, name = func
    rel = norm(filename).split("/sympy/", 1)[1]
    if name in ("sympify", "_sympify"):
        return "sympify"
    if name == "srepr":
        return "srepr"
    if name == "sqrt":
        return "sqrt"
    if name in ("factor", "factor_list", "_generic_factor", "cancel", "together"):
        return "factor/cancel"
    if name in ("expand", "radsimp", "simplify", "nsimplify", "fraction", "roots"):
        return name
    if name in ("floor", "__floor__"):
        return "floor"
    if name in ("equals", "ask", "_ask"):
        return "equals/ask"
    if name in ("cos", "sin", "atan", "atan2", "tan", "acos", "asin"):
        return "trig"
    if name == "<module>":
        return "IMPORT (not domain cost)"
    if name == "wrapper" and rel.startswith("core/cache.py"):
        return "cacheit wrapper (Expr op/ctor)"
    if name == "<genexpr>":
        return "internal genexpr (assumptions/args)"
    if name in ("Rational", "Integer") or (
        name == "__new__" and rel.startswith("core/numbers.py")
    ):
        return "Rational/Integer ctor"
    if name in ("Add", "Mul", "Pow") or (name == "__new__" and rel.startswith("core/")):
        return "Add/Mul/Pow ctor"
    if name in (
        "__add__", "__radd__", "__sub__", "__rsub__", "__mul__", "__rmul__",
        "__truediv__", "__rtruediv__", "__pow__", "__rpow__", "__neg__",
        "__abs__", "__pos__", "__floordiv__", "__rfloordiv__",
        "__sympifyit_wrapper", "_func", "binary_op_wrapper", "__mod__",
    ):
        return "Expr arithmetic"
    if name in ("__eq__", "__ne__", "__lt__", "__le__", "__gt__", "__ge__",
                "__hash__", "__int__", "__float__", "__bool__", "__index__"):
        return "Expr compare/convert/hash"
    if name.startswith("is_") or name in ("getit", "_eval_is_positive"):
        return "is_* / assumptions"
    if name in ("args", "has", "atoms", "free_symbols", "subs", "xreplace",
                "as_numer_denom", "as_coeff_Mul", "as_coeff_Add", "as_independent",
                "as_ordered_terms", "as_ordered_factors", "sort_key", "compare",
                "__iter__", "func"):
        return "tree access"
    return f"other:{name}"


# ---------------------------------------------------------------- режим profile


def cmd_profile(patch: int, density: int, pstats_out: str, row_out: str | None):
    pool_sweep.init_worker(quiet=True)
    # Прогрев: первый домен процесса платит за ИМПОРТ ядра и sympy (секунды),
    # это не цена домена. Малый домен прогревает импорты; память канонизации
    # `compute_row` сбрасывает сам, поэтому статьи бюджета прогрев не трогает.
    gate.compute_row(WARMUP_PATCH, density)
    shims = os.environ.get("NUMERIC_REPR_SHIMS", "")
    if shims:  # прототипы proto_shims.py: «что останется после замены»
        import proto_shims

        proto_shims.install(shims.split(","))
    profiler = cProfile.Profile()
    started = time.perf_counter()
    row = profiler.runcall(gate.compute_row, patch, density)
    wall = time.perf_counter() - started
    profiler.dump_stats(pstats_out)
    record = {
        "patch_id": patch,
        "density": density,
        "profiled_wall_seconds": round(wall, 3),
        "answer": row["answer"],
        "price": row["price"],
    }
    if row_out:
        Path(row_out).write_text(json.dumps(record, ensure_ascii=False), encoding="utf-8")
    print(f"patch{patch} d{density} profiled_wall={wall:.1f}s unprofiled_in_row={row['price']['seconds']}s")


# ---------------------------------------------------------------- режим aggregate


class Edges:
    """Рёбра графа вызовов из pstats + перенос веса через служебных вызывающих."""

    def __init__(self, stats: pstats.Stats):
        self.stats = stats.stats
        self.by_callee: dict[tuple, dict] = {}
        for callee, (cc, nc, tt, ct, callers) in self.stats.items():
            self.by_callee[callee] = callers
        self._resolved: dict[tuple, list[tuple[tuple, float]]] = {}

    def resolve(self, caller, depth=0):
        """[(эффективный вызывающий из ядра/хоста/гарнесса или sympy/..., вес)]."""

        if caller in self._resolved:
            return self._resolved[caller]
        kind = kind_of(caller[0])
        if kind not in ("builtin", "other") or depth >= 3:
            result = [(caller, 1.0)]
        else:
            parents = self.by_callee.get(caller, {})
            weights = {p: max(entry[3], 1e-12) for p, entry in parents.items()}
            total = sum(weights.values())
            if not total:
                result = [(caller, 1.0)]
            else:
                acc: dict[tuple, float] = collections.defaultdict(float)
                for parent, weight in weights.items():
                    for eff, w2 in self.resolve(parent, depth + 1):
                        acc[eff] += weight / total * w2
                result = list(acc.items())
        self._resolved[caller] = result
        return result

    def entries(self, callee_kinds, caller_blocklist_kinds):
        """Рёбра (эффективный caller -> callee) с долей ct/nc/cc, caller вне блок-видов."""

        for callee, (cc, nc, tt, ct, callers) in self.stats.items():
            if kind_of(callee[0]) not in callee_kinds:
                continue
            for caller, (ecc, enc, ett, ect) in callers.items():
                for eff, weight in self.resolve(caller):
                    if kind_of(eff[0]) in caller_blocklist_kinds:
                        continue
                    yield eff, callee, ecc * weight, enc * weight, ett * weight, ect * weight


def _top(mapping, key, n):
    return sorted(mapping.items(), key=key, reverse=True)[:n]


def aggregate_one(paths) -> dict:
    """Таблицы по одному pstats либо по СЛИЯНИЮ нескольких (`Stats.add`)."""

    paths = [paths] if isinstance(paths, str) else list(paths)
    stats = pstats.Stats(paths[0])
    if len(paths) > 1:
        stats.add(*paths[1:])
    edges = Edges(stats)
    total_tt = sum(entry[2] for entry in stats.stats.values())
    out: dict = {
        "pstats": [Path(item).name for item in paths],
        "profile_total_seconds": round(total_tt, 2),
    }

    # --- sympy: точки входа из не-sympy кода (mpmath внутри sympy — внутренность).
    sympy_entry = collections.defaultdict(lambda: {"calls": 0.0, "cum": 0.0})
    sympy_family_tot = collections.defaultdict(lambda: {"calls": 0.0, "cum": 0.0})
    sympy_by_caller = collections.defaultdict(
        lambda: {"calls": 0.0, "cum": 0.0, "families": collections.defaultdict(float)}
    )
    sympy_entry_callers = collections.defaultdict(lambda: collections.defaultdict(lambda: [0.0, 0.0]))
    sympy_total = 0.0
    for eff, callee, cc, nc, tt, ct in edges.entries({"sympy"}, {"sympy", "mpmath"}):
        fam = sympy_family(callee)
        if fam.startswith("IMPORT"):
            continue
        key = f"{fam} | {site(callee)}"
        sympy_entry[key]["calls"] += nc
        sympy_entry[key]["cum"] += ct
        sympy_family_tot[fam]["calls"] += nc
        sympy_family_tot[fam]["cum"] += ct
        who = site(eff)
        sympy_by_caller[who]["calls"] += nc
        sympy_by_caller[who]["cum"] += ct
        sympy_by_caller[who]["families"][fam] += ct
        sympy_entry_callers[fam][who][0] += nc
        sympy_entry_callers[fam][who][1] += ct
        sympy_total += ct
    out["sympy_inclusive_seconds_profiled"] = round(sympy_total, 2)
    out["sympy_share_of_profile"] = round(sympy_total / total_tt, 4) if total_tt else None
    out["sympy_by_family"] = [
        {
            "family": fam,
            "calls": round(v["calls"]),
            "cum_seconds": round(v["cum"], 3),
            "top_callers": [
                {"site": who, "calls": round(c), "cum_seconds": round(t, 3)}
                for who, (c, t) in sorted(
                    sympy_entry_callers[fam].items(), key=lambda kv: -kv[1][1]
                )[:4]
            ],
        }
        for fam, v in sorted(sympy_family_tot.items(), key=lambda kv: -kv[1]["cum"])[:25]
    ]
    out["sympy_by_entry_point_top25"] = [
        {"entry": key, "calls": round(v["calls"]), "cum_seconds": round(v["cum"], 3)}
        for key, v in _top(sympy_entry, lambda kv: kv[1]["cum"], 25)
    ]
    out["sympy_by_kernel_caller_top25"] = [
        {
            "site": who,
            "calls": round(v["calls"]),
            "cum_seconds": round(v["cum"], 3),
            "families": {
                f: round(t, 3)
                for f, t in sorted(v["families"].items(), key=lambda kv: -kv[1])[:5]
            },
        }
        for who, v in _top(sympy_by_caller, lambda kv: kv[1]["cum"], 25)
    ]

    # --- mpmath, вызванный НАПРЯМУЮ из ядра (интервальные оболочки предикатов).
    mp_by_caller = collections.defaultdict(lambda: {"calls": 0.0, "cum": 0.0})
    mp_total = 0.0
    for eff, callee, cc, nc, tt, ct in edges.entries({"mpmath"}, {"sympy", "mpmath"}):
        who = site(eff)
        mp_by_caller[who]["calls"] += nc
        mp_by_caller[who]["cum"] += ct
        mp_total += ct
    out["mpmath_direct_from_kernel"] = {
        "cum_seconds": round(mp_total, 3),
        "top_callers": [
            {"site": who, "calls": round(v["calls"]), "cum_seconds": round(v["cum"], 3)}
            for who, v in _top(mp_by_caller, lambda kv: kv[1]["cum"], 8)
        ],
    }

    # --- Fraction: точки входа из не-fractions кода.
    frac_by_caller = collections.defaultdict(
        lambda: {"calls": 0.0, "cum": 0.0, "ops": collections.defaultdict(lambda: [0.0, 0.0])}
    )
    frac_ops = collections.defaultdict(lambda: [0.0, 0.0])
    frac_total = 0.0
    repr_calls = 0.0
    repr_cum = 0.0
    repr_callers = collections.defaultdict(lambda: [0.0, 0.0])
    for eff, callee, cc, nc, tt, ct in edges.entries({"fractions"}, {"fractions"}):
        op = callee[2]
        who = site(eff)
        frac_by_caller[who]["calls"] += nc
        frac_by_caller[who]["cum"] += ct
        frac_by_caller[who]["ops"][op][0] += nc
        frac_by_caller[who]["ops"][op][1] += ct
        frac_ops[op][0] += nc
        frac_ops[op][1] += ct
        frac_total += ct
        if op == "__repr__":
            repr_calls += nc
            repr_cum += ct
            repr_callers[who][0] += nc
            repr_callers[who][1] += ct
    out["fractions_inclusive_seconds_profiled"] = round(frac_total, 2)
    out["fractions_by_operation"] = [
        {"op": op, "calls": round(c), "cum_seconds": round(t, 3)}
        for op, (c, t) in sorted(frac_ops.items(), key=lambda kv: -kv[1][1])[:14]
    ]
    out["fractions_top15_callers"] = [
        {
            "site": who,
            "calls": round(v["calls"]),
            "cum_seconds": round(v["cum"], 3),
            "ops": {
                op: [round(c), round(t, 3)]
                for op, (c, t) in sorted(v["ops"].items(), key=lambda kv: -kv[1][1])[:4]
            },
        }
        for who, v in _top(frac_by_caller, lambda kv: kv[1]["cum"], 15)
    ]
    out["fraction_repr"] = {
        "calls": round(repr_calls),
        "cum_seconds": round(repr_cum, 3),
        "top_callers": [
            {"site": who, "calls": round(c), "cum_seconds": round(t, 3)}
            for who, (c, t) in sorted(repr_callers.items(), key=lambda kv: -kv[1][1])[:8]
        ],
    }

    # --- exact_sqrt_sum: публичные функции, вызываемые ИЗВНЕ модуля.
    def is_sqrt_module(filename: str) -> bool:
        return norm(filename).endswith(("/exact_sqrt_sum.py", "/wavefront/sqrt_sum.py"))

    sq_by_caller = collections.defaultdict(
        lambda: {"calls": 0.0, "cum": 0.0, "funcs": collections.defaultdict(lambda: [0.0, 0.0])}
    )
    sq_funcs = collections.defaultdict(lambda: [0.0, 0.0])
    sq_total = 0.0
    for callee, (cc, nc, tt, ct, callers) in edges.stats.items():
        if not norm(callee[0]).endswith("/exact_sqrt_sum.py"):
            continue
        name = callee[2]
        public = (not name.startswith("_") and not name.startswith("<")) or name in (
            "__add__", "__sub__", "__mul__", "__truediv__", "__neg__",
        )
        if not public:
            continue
        for caller, (ecc, enc, ett, ect) in callers.items():
            for eff, weight in edges.resolve(caller):
                if is_sqrt_module(eff[0]) or kind_of(eff[0]) in ("fractions", "sympy", "mpmath"):
                    continue
                who = site(eff)
                sq_by_caller[who]["calls"] += enc * weight
                sq_by_caller[who]["cum"] += ect * weight
                sq_by_caller[who]["funcs"][name][0] += enc * weight
                sq_by_caller[who]["funcs"][name][1] += ect * weight
                sq_funcs[name][0] += enc * weight
                sq_funcs[name][1] += ect * weight
                sq_total += ect * weight
    out["exact_sqrt_sum_public_inclusive_seconds_profiled"] = round(sq_total, 2)
    out["exact_sqrt_sum_by_function"] = [
        {"func": name, "calls": round(c), "cum_seconds": round(t, 3)}
        for name, (c, t) in sorted(sq_funcs.items(), key=lambda kv: -kv[1][1])[:14]
    ]
    out["exact_sqrt_sum_top10_callers"] = [
        {
            "site": who,
            "calls": round(v["calls"]),
            "cum_seconds": round(v["cum"], 3),
            "funcs": {
                n: [round(c), round(t, 3)]
                for n, (c, t) in sorted(v["funcs"].items(), key=lambda kv: -kv[1][1])[:4]
            },
        }
        for who, v in _top(sq_by_caller, lambda kv: kv[1]["cum"], 10)
    ]

    # --- сверка с корзинами спайка (по собственному времени), чтобы таблицы были
    # сопоставимы с RECEIPT parallel_domains_spike.
    sys.path.insert(0, str(gate.SPIKE))
    import profile_domain  # noqa: E402

    totals, _ = profile_domain.classify(stats)
    grand = sum(totals.values()) or 1.0
    out["tottime_bucket_share_spike_method"] = {
        k: round(v / grand, 4) for k, v in sorted(totals.items(), key=lambda kv: -kv[1])
    }
    return out


def cmd_aggregate(out_path: str, pstats_paths: list[str]):
    result = {"domains": {}}
    for path in pstats_paths:
        name = Path(path).stem
        result["domains"][name] = aggregate_one(path)
        print(name, "done", flush=True)
    if len(pstats_paths) > 1:
        result["domains"]["COMBINED_" + "+".join(Path(p).stem for p in pstats_paths)] = (
            aggregate_one(pstats_paths)
        )
    Path(out_path).write_text(json.dumps(result, ensure_ascii=False, indent=1), encoding="utf-8")


# ---------------------------------------------------------------- режим instrument


def _value_class(value, sp) -> str:
    from fractions import Fraction

    if value is None:
        return "None"
    if isinstance(value, bool):
        return "bool"
    if isinstance(value, int):
        return "int"
    if isinstance(value, Fraction):
        return "Fraction"
    if isinstance(value, float):
        return "float"
    if isinstance(value, str):
        return "str(srepr-text)"
    if isinstance(value, sp.Basic):
        if value.is_Rational:
            return "sympy Rational/Integer"
        if value.has(sp.pi, sp.sin, sp.cos, sp.tan, sp.atan, sp.acos, sp.asin):
            return "sympy angle/trig"
        if value.free_symbols:
            return "sympy symbolic"
        exps_ok = True
        for power in value.atoms(sp.Pow):
            exponent = power.exp
            if not (exponent.is_Integer or (exponent.is_Rational and exponent.q == 2)):
                exps_ok = False
                break
        return "sympy sqrt-sum (quadratic surd)" if exps_ok else "sympy other algebraic"
    return type(value).__name__


class _CountDict(dict):
    """dict, считающий попадания/промахи/записи; владелец — `_DensityExactMemo`."""

    def __init__(self, name):
        super().__init__()
        self.name = name
        self.hits = 0
        self.misses = 0
        self.sets = 0

    def get(self, key, default=None):
        if dict.__contains__(self, key):
            self.hits += 1
            return dict.__getitem__(self, key)
        self.misses += 1
        return default

    def __contains__(self, key):
        present = dict.__contains__(self, key)
        if present:
            self.hits += 1
        else:
            self.misses += 1
        return present

    def __setitem__(self, key, value):
        self.sets += 1
        dict.__setitem__(self, key, value)


def cmd_instrument(patch: int, density: int, out_path: str):
    pool_sweep.init_worker(quiet=True)
    gate.compute_row(WARMUP_PATCH, density)  # прогрев импортов (до обёрток)
    import sympy as sp
    from cftuv_envelope.reference import metric as metric_mod
    from cftuv_envelope.reference import planar_types

    counts = collections.defaultdict(lambda: collections.Counter())
    callers_by_fn = collections.defaultdict(lambda: collections.Counter())
    kernel_marker = "cftuv_envelope"

    def kernel_caller(depth=2):
        frame = sys._getframe(depth)
        for _ in range(6):
            if frame is None:
                return "?"
            filename = frame.f_code.co_filename
            if kernel_marker in norm(filename):
                return f"{norm(filename).split('/cftuv_envelope/')[-1]}:{frame.f_lineno}({frame.f_code.co_name})"
            frame = frame.f_back
        return "?"

    def wrap(name, fn, arg_index=0):
        def wrapper(*args, **kwargs):
            caller = kernel_caller()
            if caller != "?":
                arg = args[arg_index] if len(args) > arg_index else None
                counts[name][_value_class(arg, sp)] += 1
                callers_by_fn[name][caller] += 1
            return fn(*args, **kwargs)

        wrapper.__wrapped__ = fn
        wrapper.__name__ = getattr(fn, "__name__", name)
        return wrapper

    # 1) публичные функции sympy, вызываемые ядром как `sp.<name>`.
    # ТОЛЬКО обычные функции: `floor`/`cos`/`sin`/`atan` — КЛАССЫ, ядро проверяет
    # `isinstance(x, sp.cos)`, и подмена класса функцией молча меняла бы ответ
    # (первый прогон этого режима так и сломал `density_interval_enclosure`).
    for name in ("sqrt", "factor", "cancel", "expand", "simplify", "radsimp",
                 "nsimplify", "srepr", "sympify", "roots", "ask", "fraction"):
        original = getattr(sp, name, None)
        if original is not None and not isinstance(original, type):
            setattr(sp, name, wrap(f"sp.{name}", original))
    # конструкторы Rational/Integer — классы; считаем через обёртку-подкласс нельзя,
    # поэтому считаем их косвенно профилем; здесь только значения.

    # 2) точные предикаты ядра (подмена во ВСЕХ модулях, куда имя уже импортировано).
    targets = {}
    for mod in (planar_types,):
        for fname in ("exact_normalize", "exact_sign", "_canonical_expr", "_expr",
                      "_parse_expr_uncached", "interval_enclosure",
                      "exact_quadratic_value", "exact_quadratic_expr"):
            if hasattr(mod, fname):
                targets[fname] = getattr(mod, fname)
    from cftuv_envelope.reference import angular as angular_mod

    for fname in ("_density_exact_sign", "_density_srepr", "_density_exact_vector"):
        if hasattr(angular_mod, fname):
            targets[fname] = getattr(angular_mod, fname)
    for fname in ("snap_exact_point", "snap_exact_quadratic_point", "_floor_exact",
                  "_residual_bound", "_floor_exact_quadratic"):
        if hasattr(metric_mod, fname):
            targets[fname] = getattr(metric_mod, fname)
    wrapped = {fname: wrap(fname, fn) for fname, fn in targets.items()}
    replaced = 0
    for module_name, module in list(sys.modules.items()):
        if not module_name.startswith("cftuv_envelope") or module is None:
            continue
        for fname, original in targets.items():
            if getattr(module, fname, None) is original:
                setattr(module, fname, wrapped[fname])
                replaced += 1

    # 3) память плотности: считающие словари вместо обычных.
    registry = []
    original_init = metric_mod._DensityExactMemo.__init__

    def counting_init(self):
        original_init(self)
        for slot in ("expressions", "dual_dots", "crosses", "intervals", "signs",
                     "sreprs", "subturns", "support_segments"):
            replacement = _CountDict(slot)
            dict.update(replacement, getattr(self, slot))
            setattr(self, slot, replacement)
        registry.append(self)

    metric_mod._DensityExactMemo.__init__ = counting_init

    planar_types.SYMBOLIC_FALLBACK_COUNTS.update(
        {key: 0 for key in planar_types.SYMBOLIC_FALLBACK_COUNTS}
    )
    started = time.perf_counter()
    row = gate.compute_row(patch, density)
    wall = time.perf_counter() - started

    memo_report = collections.defaultdict(lambda: collections.Counter())
    for memo in registry:
        for slot in ("expressions", "dual_dots", "crosses", "intervals", "signs",
                     "sreprs", "subturns", "support_segments"):
            table = getattr(memo, slot)
            memo_report[slot]["hits"] += table.hits
            memo_report[slot]["misses"] += table.misses
            memo_report[slot]["sets"] += table.sets
            memo_report[slot]["entries_at_end"] += len(table)
            for key, value in list(dict.items(table))[:0]:
                pass
    # примеры ключей/значений: классы
    key_value_classes = {}
    biggest = sorted(registry, key=lambda m: -len(m.expressions))[:1]
    for memo in biggest:
        for slot in ("expressions", "intervals", "signs", "sreprs"):
            table = getattr(memo, slot)
            sample = list(dict.items(table))[:2000]
            kc = collections.Counter(_value_class(k, sp) for k, _ in sample)
            vc = collections.Counter(type(v).__name__ if not isinstance(v, sp.Basic)
                                     else _value_class(v, sp) for _, v in sample)
            key_value_classes[slot] = {"keys": dict(kc), "values": dict(vc)}

    srepr_lengths = []
    for memo in biggest:
        for text in list(dict.keys(memo.expressions))[:5000]:
            srepr_lengths.append(len(text))
    record = {
        "patch_id": patch,
        "density": density,
        "instrumented_wall_seconds": round(wall, 2),
        "answer_equals_row": row["answer"],
        "wrapped_bindings_replaced": replaced,
        "memos_created": len(registry),
        "symbolic_fallback_counts": dict(planar_types.SYMBOLIC_FALLBACK_COUNTS),
        "first_arg_value_classes_by_function": {
            name: dict(counter.most_common()) for name, counter in sorted(counts.items())
        },
        "top_kernel_callers_by_function": {
            name: [{"site": s, "calls": c} for s, c in counter.most_common(6)]
            for name, counter in sorted(callers_by_fn.items())
        },
        "density_exact_memo": {k: dict(v) for k, v in memo_report.items()},
        "density_exact_memo_key_value_classes_first_memo": key_value_classes,
        "expression_srepr_len_first_memo": {
            "n": len(srepr_lengths),
            "mean": round(sum(srepr_lengths) / len(srepr_lengths), 1) if srepr_lengths else 0,
            "max": max(srepr_lengths) if srepr_lengths else 0,
        },
    }
    Path(out_path).write_text(json.dumps(record, ensure_ascii=False, indent=1), encoding="utf-8")
    print(f"patch{patch} d{density} instrumented wall={wall:.1f}s -> {out_path}")


def main():
    mode = sys.argv[1]
    if mode == "profile":
        patch, density, out = int(sys.argv[2]), int(sys.argv[3]), sys.argv[4]
        cmd_profile(patch, density, out, sys.argv[5] if len(sys.argv) > 5 else None)
    elif mode == "aggregate":
        cmd_aggregate(sys.argv[2], sys.argv[3:])
    elif mode == "instrument":
        cmd_instrument(int(sys.argv[2]), int(sys.argv[3]), sys.argv[4])
    else:
        raise SystemExit(__doc__)


if __name__ == "__main__":
    main()
