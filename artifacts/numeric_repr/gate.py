"""Ворота РАВЕНСТВА ОТВЕТА для среза 6 (числовое представление).

Срез 6 меняет ЧИСЛА внутри ядра (sympy -> Fraction/SqrtSumV1/целые), и его
единственное условие: ни один ответ ни одного домена `building` не меняется.
Эти ворота считают все 122 домена тем же маршрутом, что и кнопка
(`build_envelope_analysis_snapshot` -> `build_envelope_decal_request` ->
`run_queue_domain`, alpha 0.45, штатный кап) в пуле процессов и пишут на каждый
домен отпечатки ДВУХ классов, которые сравниваются по-разному:

ANSWER (обязан совпасть побитово; любое расхождение — код возврата 1):
    исход подготовки/покрытия, деталь отказа, масштаб решётки, alpha, имена
    законов, структурные счётчики `CONVEYOR_*` (число граней/узлов/вееров ...),
    отпечатки `regions/faces/segments` (repr-дайджест как в pool_sweep.py И
    числово-нормализованный), глубокий отпечаток ВНУТРЕННОСТЕЙ подготовки:
    мост, скелет (узлы с точными позициями, обязательства, отказы кандидатов),
    разбиение на грани, владельцы рёбер (строго типизированный и
    числово-нормализованный: `Fraction(3,1)` и `3` различаются только в первом).
PRICE (расхождение ПЕРЕЧИСЛЯЕТСЯ, но не ошибка — именно это срез и должен менять):
    шесть статей бюджета и `EXACT_WORK_SPENT`, внебюджетная работа, счётчики
    знаков `SIGN_COUNTS`, секунды (целиком, хост, подготовка, покрытие, стадии).

Запуск (из любого каталога; `env` лежит в artifacts/perf_prepare_diag):

    python artifacts/numeric_repr/gate.py run --workers 8 --densities 1,2 \
        --out artifacts/numeric_repr/baseline_<sha>.json
    python artifacts/numeric_repr/gate.py run --workers 8 --densities 1,2 \
        --out new.json --baseline artifacts/numeric_repr/baseline_<sha>.json
    python artifacts/numeric_repr/gate.py compare baseline.json new.json
    python artifacts/numeric_repr/gate.py selftest baseline.json   # отрицательные контроли

`--only 6,11` — подмножество доменов (сравнение тогда идёт по пересечению).

Срез, который меняет ОТВЕТ осознанно (закон веера), сравнивается по спецификации ожидаемого изменения, а не
глазами: `compare base.json new.json --spec right_angle_stable` (`--partial` при `--only`, `--list-specs`).
Спецификация — данные в `artifacts/materialize_sweep/specs/*.json`, её схема и правила — в
`artifacts/materialize_sweep/expected_change.py`; последняя строка вывода — один из трёх вердиктов:
`IDENTICAL`, `EXPECTED-CHANGE (spec X): N domains`, `UNEXPECTED (spec X): K problems; first: ...`.
Без `--spec` любое расхождение ответа — `UNEXPECTED`. ЦЕНА (секунды, бюджет работы) спецификацией не
судится: она перечисляется и ошибкой не бывает.

Fan Density передаётся аргументом `density=` в `build_envelope_decal_request`
(0..4, `None` — старый закон), ровно как это делает панель.
"""

from __future__ import annotations

import argparse
import copy
import dataclasses
import enum
import hashlib
import json
import os
import subprocess
import sys
import time
from fractions import Fraction
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
SPIKE = ROOT / "artifacts" / "parallel_domains_spike"
DIAG = ROOT / "artifacts" / "perf_prepare_diag"
for _entry in (str(SPIKE), str(DIAG)):
    if _entry not in sys.path:
        sys.path.insert(0, _entry)

import pool_sweep  # noqa: E402  (модуль лёгкий: на импорте только stdlib)

SWEEP_TOOL = ROOT / "artifacts" / "materialize_sweep"
if str(SWEEP_TOOL) not in sys.path:
    # В конец, не в начало: в каталоге свипа лежит свой `make_receipt.py`, и он не должен затенять соседний.
    sys.path.append(str(SWEEP_TOOL))

import expected_change  # noqa: E402  (спецификация ожидаемого изменения: общая со `sweep.py`)

ALPHA_VALUE = pool_sweep.ALPHA_VALUE
ALPHA_TEXT = pool_sweep.ALPHA_TEXT
SCHEMA = "numeric_repr_gate_v1"
#: Порядок по умолчанию для самого первого прогона: секунды d0 из старой
#: расписки полного обхода. Дальше порядок берётся из baseline.
DEFAULT_ORDER_SOURCE = ROOT / "artifacts" / "building_full_sweep" / "RECEIPT.json"

WORK_ARTICLES = (
    "EXACT_WORK_MODULAR_SQUARINGS",
    "EXACT_WORK_GCD_OPERATIONS",
    "EXACT_WORK_MILLER_RABIN_ROUNDS",
    "EXACT_WORK_POLLARD_ATTEMPTS",
    "EXACT_WORK_RADICAL_MATERIALIZATIONS",
    "EXACT_WORK_EXACT_POSITION_HYDRATIONS",
    "EXACT_WORK_SPENT",
)


# --------------------------------------------------------------------------
# Каноническая свёртка структур в дайджест
# --------------------------------------------------------------------------


class _Feed:
    """Потоковая свёртка значения в sha256 без промежуточного дерева."""

    def __init__(self, strict: bool):
        self.strict = strict
        self.stats = {"sympy": 0, "unstable_repr": 0, "nodes": 0}

    def digest(self, value) -> str:
        hasher = hashlib.sha256()
        self._feed(hasher, value)
        return hasher.hexdigest()[:12]

    def _feed(self, h, o) -> None:
        self.stats["nodes"] += 1
        t = type(o)
        w = h.update
        if o is None:
            w(b"N;")
        elif t is bool:
            w(b"B1;" if o else b"B0;")
        elif t is str:
            w(b"S%d:" % len(o))
            w(o.encode("utf-8"))
        elif t is int:
            w((b"I%d;" % o) if self.strict else (b"Q%d/1;" % o))
        elif t is Fraction:
            if self.strict:
                w(b"F%d/%d;" % (o.numerator, o.denominator))
            else:
                w(b"Q%d/%d;" % (o.numerator, o.denominator))
        elif t is float:
            w(b"D" + o.hex().encode("ascii") + b";")
        elif isinstance(o, enum.Enum):
            w(b"E" + t.__qualname__.encode() + b"." + str(o.name).encode() + b";")
        elif t.__qualname__ == "ExactWorkBudgetV1":
            w(b"BUDGET;")  # счёт работы — ЦЕНА, не ответ
        elif dataclasses.is_dataclass(o) and not isinstance(o, type):
            w(b"C" + t.__qualname__.encode() + b"{")
            for field in dataclasses.fields(o):
                w(field.name.encode() + b"=")
                self._feed(h, getattr(o, field.name))
            w(b"}")
        elif t is tuple or t is list:
            w(b"T%d[" % len(o))
            for item in o:
                self._feed(h, item)
            w(b"]")
        elif t is frozenset or t is set:
            parts = sorted(self._sub(item) for item in o)
            w(b"Z%d[" % len(parts))
            for part in parts:
                w(part.encode())
            w(b"]")
        elif t is dict:
            items = sorted((self._sub(k), self._sub(v)) for k, v in o.items())
            w(b"M%d[" % len(items))
            for k, v in items:
                w(k.encode() + v.encode())
            w(b"]")
        else:
            module = t.__module__ or ""
            text = repr(o)
            if module.startswith("sympy"):
                self.stats["sympy"] += 1
            if " at 0x" in text:
                self.stats["unstable_repr"] += 1
            w(b"X" + t.__qualname__.encode() + b":" + text.encode("utf-8") + b";")

    def _sub(self, value) -> str:
        hasher = hashlib.sha256()
        self._feed(hasher, value)
        return hasher.hexdigest()


def _repr_digest(payload) -> str:
    return hashlib.sha256(repr(payload).encode("utf-8")).hexdigest()[:12]


def _answer_of(prepared, domain) -> dict:
    """Все ОТВЕТНЫЕ отпечатки готового домена."""

    geometry = (domain.regions, domain.faces, domain.segments)
    meta = (
        domain.preparation_outcome,
        domain.coverage_outcome,
        domain.detail,
        domain.lattice_scale,
        domain.alpha,
        domain.lattice_alpha,
        domain.law_names,
    )
    strict = _Feed(True)
    value = _Feed(False)
    answer = {
        "outcome": f"{domain.preparation_outcome}/{domain.coverage_outcome}",
        "detail": domain.detail,
        "lattice_scale": domain.lattice_scale,
        "n_regions": len(domain.regions),
        "n_faces": len(domain.faces),
        "n_segments": len(domain.segments),
        "fp_meta": _repr_digest(meta),
        "fp_geometry": _repr_digest(geometry),
        "fp_geometry_value": value.digest(geometry),
        "fp_structural_counters": _repr_digest(
            tuple(
                item
                for item in domain.counters
                if not item[0].startswith("EXACT_WORK_")
            )
        ),
        "fp_host_counters": _repr_digest(domain.host_counters),
        "repr_has_address": " at 0x" in repr(geometry),
    }
    parts = {"bridge": [], "skeleton": [], "partition": [], "owners": []}
    for region in prepared.regions:
        parts["bridge"].append(region.bridge)
        parts["skeleton"].append(region.skeleton)
        parts["partition"].append(region.partition)
        parts["owners"].append(
            (
                region.region_id,
                region.owner_by_edge,
                region.wall_spans,
                region.wall_edge_count,
                region.ambiguous_owner_spans,
                region.degraded_miter_corners,
                region.bridge_outcome,
                region.skeleton_outcome,
                region.face_outcome,
            )
        )
    for name, items in parts.items():
        answer[f"fp_deep_{name}"] = strict.digest(tuple(items))
        answer[f"fp_deep_{name}_value"] = value.digest(tuple(items))
    answer["fp_deep_lattice"] = strict.digest(
        (prepared.lattice, prepared.law_names, prepared.outcome)
    )
    answer["deep_sympy_nodes"] = strict.stats["sympy"]
    answer["deep_unstable_repr_nodes"] = (
        strict.stats["unstable_repr"] + value.stats["unstable_repr"]
    )
    return answer


# --------------------------------------------------------------------------
# Один домен
# --------------------------------------------------------------------------


def compute_row(patch_id: int, density, alpha_value=ALPHA_VALUE, alpha_text=ALPHA_TEXT):
    """Домен `patch_id` при Fan Density `density`; возвращает `{answer, price}`."""

    ctx = pool_sweep._CTX
    canon = ctx["canon"]
    domain_id = ctx["typed_value"]("patch-domain", ctx["revision"], patch_id)
    canon.reset_factorization_memory()
    canon.reset_unbudgeted_work()
    canon.reset_sign_counts()
    _reset_backend_counts()
    started = time.perf_counter()
    host_seconds = 0.0
    answer: dict
    price: dict = {}
    prepared = domain = None
    try:
        snapshot = ctx["build_snapshot"](
            ctx["bundle"], included_patch_ids=frozenset({patch_id})
        )
        request = ctx["build_request"](
            snapshot,
            frozenset(ctx["by_domain"][domain_id]),
            alpha_value,
            decal_request_id_value=ctx["request_id"],
            density=density,
        )
        host_seconds = time.perf_counter() - started
        prepared, domain = ctx["run_queue_domain"](
            patch_id, domain_id, snapshot, request, alpha_text
        )
    except ctx["EnvelopeHostAdapterError"] as refusal:
        answer = {"outcome": "HOST_ADMISSION_REFUSED", "detail": str(refusal)}
    except Exception as error:  # noqa: BLE001 - исход домена, а не авария прогона
        answer = {
            "outcome": f"EXCEPTION/{type(error).__name__}",
            "detail": str(error)[:400],
        }
    else:
        answer = _answer_of(prepared, domain)
        # Цена вычисления - бюджет ПОКРЫТИЯ (копия состояния подготовки плюс покрытие); у дерева до PRICE-WITHOUT-HISTORY его нет,
        # и цену копил бюджет самой подготовки.
        charged = tuple(getattr(domain, "coverage_work", ()))
        if not charged:
            named = getattr(prepared, "work_budget", None)
            charged = () if named is None else named.counters()
        if not charged:
            charged = tuple(
                (name, value)
                for name, value in prepared.counters
                if name.startswith("EXACT_WORK_")
            )
        price.update(dict(charged))
        price["host_export_seconds"] = round(host_seconds, 3)
        price["prepare_seconds"] = round(domain.prepare_seconds, 3)
        price["coverage_seconds"] = round(domain.coverage_seconds, 3)
        price["stage_seconds"] = {
            name: round(value, 3) for name, value in domain.timings
        }
    price["seconds"] = round(time.perf_counter() - started, 3)
    price["leaked_unbudgeted"] = canon.UNBUDGETED_WORK.spent
    price["sign_counts"] = dict(canon.SIGN_COUNTS)
    price.update(_backend_price())
    return {"patch_id": patch_id, "density": density, "answer": answer, "price": price}


def _install_symbolic_backend() -> None:
    """`CFTUV_SYMBOLIC_BACKEND=SYMPY|NATIVE_EXACT|SHADOW` ставит режим символьного бэкенда в воркере.

    Переменную читает ТОЛЬКО харнесс ворот: продукт режим не выбирает (умолчание модуля —
    `symbolic_backend.DEFAULT_BACKEND` = `NATIVE_EXACT`), а `SHADOW` идёт с политикой записи
    расхождений, чтобы один прогон собрал их все (`artifacts/sympy_off_hot_path/`).
    """

    name = os.environ.get("CFTUV_SYMBOLIC_BACKEND", "")
    if not name:
        return
    from cftuv_envelope.reference import symbolic_backend

    symbolic_backend.set_backend_mode(symbolic_backend.SymbolicBackendV1(name))
    symbolic_backend.set_disagreement_policy(symbolic_backend.DisagreementPolicyV1.RECORD)


def _backend_price() -> dict:
    """Счётчики символьного бэкенда этого домена; пусто, пока режим `SYMPY`."""

    from cftuv_envelope.reference import planar_types, symbolic_backend

    if symbolic_backend.backend_mode() is symbolic_backend.SymbolicBackendV1.SYMPY:
        return {}
    return {
        "symbolic_backend": symbolic_backend.backend_mode().value,
        "backend_counts": dict(sorted(symbolic_backend.BACKEND_COUNTS.items())),
        "backend_disagreements": list(symbolic_backend.DISAGREEMENTS[:20]),
        "backend_text_differences": list(planar_types.TEXT_DIFFERENCES[:20]),
    }


def _reset_backend_counts() -> None:
    from cftuv_envelope.reference import planar_types, symbolic_backend

    symbolic_backend.reset_backend_counts()
    planar_types.TEXT_DIFFERENCES.clear()


def init_worker(quiet: bool = True):
    """Инициализатор воркера: как в pool_sweep + (необязательно) runtime-шимы прототипов.

    `NUMERIC_REPR_SHIMS=sign,compare,...` ставит шимы `proto_shims` в КАЖДОМ воркере. Так
    ворота проверяют прототип замены на всех доменах тем же кодом, которым проверят настоящую
    правку; без переменной это чистый `pool_sweep.init_worker`.
    """

    pool_sweep.init_worker(quiet=quiet)
    _install_symbolic_backend()
    shims = os.environ.get("NUMERIC_REPR_SHIMS", "")
    if shims:
        import proto_shims

        proto_shims.install(shims.split(","))


def _task(args):
    patch_id, density = args
    return compute_row(patch_id, density)


# --------------------------------------------------------------------------
# Прогон
# --------------------------------------------------------------------------


def _git(*args: str) -> str:
    try:
        out = subprocess.run(
            ["git", *args], cwd=ROOT, capture_output=True, text=True, timeout=30
        )
        return out.stdout.strip()
    except Exception:  # noqa: BLE001
        return ""


def _default_order() -> list[int]:
    rows = json.loads(DEFAULT_ORDER_SOURCE.read_text(encoding="utf-8"))["domains"]
    return [
        pid
        for pid, _ in sorted(
            ((int(key[5:]), row["seconds"]) for key, row in rows.items()),
            key=lambda item: (-item[1], item[0]),
        )
    ]


def _order_from(path: Path | None, density) -> list[int]:
    if path is None or not path.exists():
        return _default_order()
    record = json.loads(path.read_text(encoding="utf-8"))
    run = record["runs"].get(str(density))
    if run is None:
        return _default_order()
    return [int(item) for item in run["order"]]


def run_gate(args) -> dict:
    from concurrent.futures import ProcessPoolExecutor

    densities = [int(item) for item in args.densities.split(",")]
    order_path = Path(args.order) if args.order else (
        Path(args.baseline) if args.baseline else None
    )
    record = {
        "schema": SCHEMA,
        "sha": _git("rev-parse", "--short", "HEAD"),
        "sha_full": _git("rev-parse", "HEAD"),
        "tree_clean": _git("status", "--porcelain", "--", "cftuv", "kernel") == "",
        # Хэш незакоммиченных правок продукта: без него расхождение ответа между
        # двумя прогонами не отличить от «кто-то правил дерево между прогонами».
        "tree_diff_hash": hashlib.sha1(
            _git("diff", "--", "cftuv", "kernel").encode("utf-8")
        ).hexdigest()[:10],
        "alpha": ALPHA_TEXT,
        "workers": args.workers,
        "python": sys.version.split()[0],
        "cores": os.cpu_count(),
        "shims": os.environ.get("NUMERIC_REPR_SHIMS", "") or None,
        "runs": {},
    }
    try:
        import sympy

        record["sympy"] = sympy.__version__
    except ImportError:
        record["sympy"] = None
    for density in densities:
        order = _order_from(order_path, density)
        if args.only:
            wanted = [int(item) for item in args.only.split(",")]
            order = wanted
        print(f"[gate] density={density} domains={len(order)} workers={args.workers}",
              flush=True)
        if args.workers == 0:
            init_worker(quiet=True)
            started = time.perf_counter()
            rows = [compute_row(pid, density) for pid in order]
            wall = time.perf_counter() - started
        else:
            with ProcessPoolExecutor(
                max_workers=args.workers, initializer=init_worker
            ) as pool:
                started = time.perf_counter()
                rows = list(pool.map(_task, [(pid, density) for pid in order]))
                wall = time.perf_counter() - started
        rows.sort(key=lambda row: row["patch_id"])
        by_seconds = sorted(rows, key=lambda row: (-row["price"]["seconds"], row["patch_id"]))
        outcomes: dict[str, int] = {}
        for row in rows:
            outcomes[row["answer"]["outcome"]] = (
                outcomes.get(row["answer"]["outcome"], 0) + 1
            )
        record["runs"][str(density)] = {
            "density": density,
            "wall_seconds": round(wall, 3),
            "sum_domain_seconds": round(sum(r["price"]["seconds"] for r in rows), 3),
            "max_domain_seconds": by_seconds[0]["price"]["seconds"],
            "order": [row["patch_id"] for row in by_seconds],
            "outcomes": outcomes,
            "domains": {str(row["patch_id"]): {
                "answer": row["answer"], "price": row["price"]} for row in rows},
        }
        print(
            f"[gate] density={density} wall={wall:.1f}s "
            f"sum={record['runs'][str(density)]['sum_domain_seconds']:.1f}s "
            f"max={by_seconds[0]['price']['seconds']}s (patch{by_seconds[0]['patch_id']}) "
            f"outcomes={outcomes}",
            flush=True,
        )
    return record


# --------------------------------------------------------------------------
# Сравнение
# --------------------------------------------------------------------------

_REPR_ONLY_PAIRS = {
    "fp_geometry": "fp_geometry_value",
}


def _short(value, limit=120):
    text = json.dumps(value, ensure_ascii=False, default=str)
    return text if len(text) <= limit else text[: limit - 3] + "..."


def _flat_price(price: dict) -> dict:
    flat = {}
    for key, value in price.items():
        if isinstance(value, dict):
            for inner, item in value.items():
                flat[f"{key}.{inner}"] = item
        else:
            flat[key] = value
    return flat


#: Виды отпечатков ответа домена (`allow.digests` спецификации); остальные поля ответа — `allow.fields`.
ANSWER_DIGEST_KEYS = (
    "fp_meta",
    "fp_geometry",
    "fp_geometry_value",
    "fp_structural_counters",
    "fp_host_counters",
    "fp_deep_bridge",
    "fp_deep_bridge_value",
    "fp_deep_skeleton",
    "fp_deep_skeleton_value",
    "fp_deep_partition",
    "fp_deep_partition_value",
    "fp_deep_owners",
    "fp_deep_owners_value",
    "fp_deep_lattice",
)
ANSWER_FIELD_KEYS = (
    "outcome",
    "detail",
    "lattice_scale",
    "n_regions",
    "n_faces",
    "n_segments",
    "repr_has_address",
    "deep_sympy_nodes",
    "deep_unstable_repr_nodes",
)
VOCABULARY = expected_change.Vocabulary(
    digests=frozenset(ANSWER_DIGEST_KEYS),
    fields=frozenset(ANSWER_FIELD_KEYS),
    counters=frozenset(),
)


def _answer_view(row: dict):
    """Ответ строки ворот: ANSWER целиком (цена в виде не живёт)."""

    answer = row["answer"]
    return expected_change.RowView(fields=dict(answer), counters={}, ok=answer.get("outcome") == "EXACT/EXACT")


def answer_pair_views(base_row: dict, new_row: dict):
    return _answer_view(base_row), _answer_view(new_row)


def load_spec(reference: str | None):
    """Спецификация ожидаемого изменения по имени из `specs/` либо пути; `None` — без спецификации."""

    return None if reference is None else expected_change.load_spec(reference, "gate", VOCABULARY)


def compare_records(base: dict, new: dict, spec=None, partial: bool = False) -> dict:
    """Сравнить прогон с эталоном: ANSWER по спецификации (без неё — строго), PRICE — перечислить."""

    report = expected_change.evaluate([base, new], ["base", "new"], spec, answer_pair_views, VOCABULARY, partial)
    answer_diffs = []
    repr_only = []
    price_changes: dict[str, dict] = {}
    structural = []
    per_density = {}
    for density in sorted(set(base["runs"]) & set(new["runs"])):
        b_run, n_run = base["runs"][density], new["runs"][density]
        b_dom, n_dom = b_run["domains"], n_run["domains"]
        if set(b_dom) != set(n_dom):
            structural.append(
                {"density": density, "domain_set_only_in_base":
                 sorted(set(b_dom) - set(n_dom), key=int),
                 "domain_set_only_in_new": sorted(set(n_dom) - set(b_dom), key=int)}
            )
        common = sorted(set(b_dom) & set(n_dom), key=int)
        for patch in common:
            b_ans, n_ans = b_dom[patch]["answer"], n_dom[patch]["answer"]
            differing = [
                change.name
                for change in expected_change.diff_views(*answer_pair_views(b_dom[patch], n_dom[patch]))
            ]
            if differing:
                value_keys_same = all(
                    not key.endswith("_value")
                    and (key + "_value") in b_ans
                    and b_ans[key + "_value"] == n_ans.get(key + "_value")
                    for key in differing
                )
                entry = {
                    "density": density,
                    "patch_id": int(patch),
                    "keys": differing,
                    "detail": {
                        key: [b_ans.get(key, "<absent>"), n_ans.get(key, "<absent>")]
                        for key in differing[:6]
                    },
                }
                (repr_only if value_keys_same else answer_diffs).append(entry)
            # PRICE
            b_price = _flat_price(b_dom[patch]["price"])
            n_price = _flat_price(n_dom[patch]["price"])
            for key in sorted(set(b_price) | set(n_price)):
                if key == "pid":
                    continue
                lv, rv = b_price.get(key), n_price.get(key)
                if lv == rv:
                    continue
                slot = price_changes.setdefault(
                    f"d{density}:{key}",
                    {"domains_changed": 0, "sum_base": 0.0, "sum_new": 0.0,
                     "worst_domain": None, "worst_abs": -1.0},
                )
                slot["domains_changed"] += 1
                if isinstance(lv, (int, float)) and isinstance(rv, (int, float)):
                    slot["sum_base"] += lv
                    slot["sum_new"] += rv
                    if abs(rv - lv) > slot["worst_abs"]:
                        slot["worst_abs"] = abs(rv - lv)
                        slot["worst_domain"] = {"patch": int(patch), "base": lv, "new": rv}
        per_density[density] = {
            "domains_compared": len(common),
            "wall_seconds_base": b_run["wall_seconds"],
            "wall_seconds_new": n_run["wall_seconds"],
            "sum_domain_seconds_base": b_run["sum_domain_seconds"],
            "sum_domain_seconds_new": n_run["sum_domain_seconds"],
            "max_domain_seconds_base": b_run["max_domain_seconds"],
            "max_domain_seconds_new": n_run["max_domain_seconds"],
            "outcomes_base": b_run["outcomes"],
            "outcomes_new": n_run["outcomes"],
        }
    for slot in price_changes.values():
        for key in ("sum_base", "sum_new", "worst_abs"):
            slot[key] = round(slot[key], 3)
    return {
        "verdict": expected_change.verdict_line(report),
        "report": report,
        "spec": None if spec is None else spec.name,
        "answer_diffs": answer_diffs,
        "answer_repr_only_diffs": repr_only,
        "structural": structural,
        "price_changes": price_changes,
        "per_density": per_density,
        "base_sha": base.get("sha"),
        "new_sha": new.get("sha"),
    }


def verdict_is_clean(result: dict) -> bool:
    """`IDENTICAL` либо `EXPECTED-CHANGE`: ворота пройдены (код возврата 0)."""

    return result["report"].exit_code == 0


def print_comparison(result: dict) -> None:
    print("PER DENSITY")
    for density, info in result["per_density"].items():
        print(
            f"  d{density}: {info['domains_compared']} domains; wall "
            f"{info['wall_seconds_base']} -> {info['wall_seconds_new']} s; "
            f"sum {info['sum_domain_seconds_base']} -> {info['sum_domain_seconds_new']} s; "
            f"max {info['max_domain_seconds_base']} -> {info['max_domain_seconds_new']} s; "
            f"outcomes {info['outcomes_base']} -> {info['outcomes_new']}"
        )
    print("PRICE CHANGES (listed, not failures)")
    for key, slot in sorted(result["price_changes"].items()):
        if key.endswith((":pid",)):
            continue
        print(
            f"  {key}: {slot['domains_changed']} domains, sum "
            f"{slot['sum_base']} -> {slot['sum_new']}"
        )
    report = result["report"]
    if report.problems or result["spec"] is None:
        for entry in result["structural"]:
            print("STRUCTURAL", _short(entry, 300))
        for entry in result["answer_repr_only_diffs"][:20]:
            print("ANSWER REPR-ONLY DIFF", _short(entry, 300))
        for entry in result["answer_diffs"][:20]:
            print("ANSWER DIFF", _short(entry, 300))
        print(
            f"ANSWER diffs={len(result['answer_diffs'])} "
            f"repr-only={len(result['answer_repr_only_diffs'])} "
            f"structural={len(result['structural'])}"
        )
    expected_change.print_report(report)


# --------------------------------------------------------------------------
# Отрицательные контроли
# --------------------------------------------------------------------------


def selftest(base_path: Path, live: bool, live_patch: int | None) -> int:
    base = json.loads(base_path.read_text(encoding="utf-8"))
    report = {}
    failed = False

    def expect(name: str, record: dict, want: str, spec=None) -> None:
        nonlocal failed
        got = compare_records(base, record, spec)
        ok = got["verdict"].split(" ", 1)[0].split(":", 1)[0] == want
        report[name] = {"want": want, "got": got["verdict"], "ok": ok,
                        "answer_diffs": len(got["answer_diffs"]),
                        "price_keys": len(got["price_changes"])}
        failed |= not ok

    density = sorted(base["runs"])[0]
    domains = base["runs"][density]["domains"]
    ok_patch = next(
        pid for pid, item in domains.items()
        if item["answer"]["outcome"] == "EXACT/EXACT"
    )
    # 0. идентичная копия
    expect("identity", copy.deepcopy(base), "IDENTICAL")
    # 1. бит в отпечатке геометрии
    twin = copy.deepcopy(base)
    slot = twin["runs"][density]["domains"][ok_patch]["answer"]
    slot["fp_geometry"] = slot["fp_geometry"][:-1] + (
        "0" if slot["fp_geometry"][-1] != "0" else "1"
    )
    expect("fingerprint_bit_flip_detected", twin, "UNEXPECTED")
    # 2. бит во внутренностях
    twin = copy.deepcopy(base)
    slot = twin["runs"][density]["domains"][ok_patch]["answer"]
    slot["fp_deep_skeleton"] = "0" * 12
    expect("deep_skeleton_bit_flip_detected", twin, "UNEXPECTED")
    # 3. структурный счётчик ответа
    twin = copy.deepcopy(base)
    slot = twin["runs"][density]["domains"][ok_patch]["answer"]
    slot["n_faces"] += 1
    expect("answer_face_count_detected", twin, "UNEXPECTED")
    # 4. исход
    twin = copy.deepcopy(base)
    twin["runs"][density]["domains"][ok_patch]["answer"]["outcome"] = "REFUSED/NONE"
    expect("outcome_change_detected", twin, "UNEXPECTED")
    # 5. выпавший домен
    twin = copy.deepcopy(base)
    del twin["runs"][density]["domains"][ok_patch]
    expect("missing_domain_detected", twin, "UNEXPECTED")
    # 6. ЦЕНА не должна ронять ворота
    twin = copy.deepcopy(base)
    price = twin["runs"][density]["domains"][ok_patch]["price"]
    price["seconds"] = round(price["seconds"] / 3, 3)
    price["EXACT_WORK_SPENT"] = price.get("EXACT_WORK_SPENT", 0) + 12345
    price["EXACT_WORK_GCD_OPERATIONS"] = 1
    price["stage_seconds"] = {"SKELETON": 0.001}
    expect("price_only_change_is_not_failure", twin, "IDENTICAL")
    # 7. спецификация ожидаемого изменения: разрешённое поле проходит, соседнее — валит, неизменный объявленный домен — валит
    spec = expected_change.parse_spec(
        {
            "schema": expected_change.SPEC_SCHEMA, "name": "selftest", "tool": "gate", "law": "SELFTEST",
            "about": "selftest", "domains": {"by_density": {density: [int(ok_patch)]}}, "allow": {"fields": ["n_faces"]},
        },
        "gate",
        VOCABULARY,
    )
    twin = copy.deepcopy(base)
    twin["runs"][density]["domains"][ok_patch]["answer"]["n_faces"] += 1
    expect("spec_allows_the_declared_field", twin, "EXPECTED-CHANGE", spec)
    slot = twin["runs"][density]["domains"][ok_patch]["answer"]
    slot["fp_geometry"] = slot["fp_geometry"][:-1] + ("0" if slot["fp_geometry"][-1] != "0" else "1")
    expect("spec_rejects_another_field", twin, "UNEXPECTED", spec)
    expect("spec_declared_domain_unchanged_fails", copy.deepcopy(base), "UNEXPECTED", spec)
    if live:
        report["live"] = _live_negative_control(base, density, live_patch)
        failed |= not report["live"]["ok"]
    print(json.dumps(report, ensure_ascii=False, indent=1))
    print("SELFTEST", "FAILED" if failed else "PASSED")
    return 1 if failed else 0


def _live_negative_control(base, density, live_patch):
    """Настоящее возмущение ВХОДА: другая alpha на живом домене меняет ответ."""

    domains = base["runs"][density]["domains"]
    if live_patch is None:
        candidates = sorted(
            (
                (item["price"]["seconds"], pid)
                for pid, item in domains.items()
                if item["answer"]["outcome"] == "EXACT/EXACT"
            )
        )
        live_patch = int(candidates[len(candidates) // 4][1])
    pool_sweep.init_worker(quiet=True)
    same = compute_row(live_patch, int(density))
    shifted = compute_row(live_patch, int(density), alpha_value=0.5, alpha_text="0.5")
    base_answer = domains[str(live_patch)]["answer"]
    return {
        "patch": live_patch,
        "same_inputs_answer_equal": same["answer"] == base_answer,
        "alpha_0.5_answer_differs": shifted["answer"] != base_answer,
        "differing_keys": sorted(
            key for key in set(shifted["answer"]) | set(base_answer)
            if shifted["answer"].get(key) != base_answer.get(key)
        ),
        "ok": same["answer"] == base_answer and shifted["answer"] != base_answer,
    }


# --------------------------------------------------------------------------


def _spec_or_exit(parser, reference):
    try:
        return load_spec(reference)
    except expected_change.SpecError as error:
        parser.error(str(error))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    sub = parser.add_subparsers(dest="mode", required=True)

    run = sub.add_parser("run")
    run.add_argument("--workers", type=int, default=8)
    run.add_argument("--densities", default="1,2")
    run.add_argument("--out", required=True)
    run.add_argument("--baseline", default=None)
    run.add_argument("--order", default=None, help="json прошлого прогона: порядок задач")
    run.add_argument("--only", default="", help="patch id через запятую")
    run.add_argument("--spec", default=None, help="со --baseline: спецификация ожидаемого изменения (имя из specs/ либо путь)")

    cmp_ = sub.add_parser("compare")
    cmp_.add_argument("baseline", nargs="?")
    cmp_.add_argument("new", nargs="?")
    cmp_.add_argument("--json-out", default=None)
    cmp_.add_argument("--spec", default=None, help="спецификация ожидаемого изменения: имя из specs/ либо путь к .json")
    cmp_.add_argument("--partial", action="store_true", help="прогон части доменов (--only): объявленные домены вне записей — примечание")
    cmp_.add_argument("--list-specs", action="store_true", help="перечислить сохранённые спецификации")

    test = sub.add_parser("selftest")
    test.add_argument("baseline")
    test.add_argument("--live", action="store_true")
    test.add_argument("--live-patch", type=int, default=None)

    args = parser.parse_args()
    if args.mode == "run":
        record = run_gate(args)
        Path(args.out).write_text(
            json.dumps(record, ensure_ascii=False, separators=(",", ":")),
            encoding="utf-8",
        )
        print(f"[gate] wrote {args.out} ({Path(args.out).stat().st_size} bytes)")
        if args.baseline:
            base = json.loads(Path(args.baseline).read_text(encoding="utf-8"))
            result = compare_records(base, record, _spec_or_exit(parser, args.spec), bool(args.only))
            print_comparison(result)
            return 0 if verdict_is_clean(result) else 1
        return 0
    if args.mode == "compare":
        if args.list_specs:
            expected_change.print_stored_specs("gate")
            return 0
        if not args.baseline or not args.new:
            parser.error("compare needs <baseline.json> <new.json>")
        base = json.loads(Path(args.baseline).read_text(encoding="utf-8"))
        new = json.loads(Path(args.new).read_text(encoding="utf-8"))
        result = compare_records(base, new, _spec_or_exit(parser, args.spec), args.partial)
        print_comparison(result)
        if args.json_out:
            Path(args.json_out).write_text(
                json.dumps({key: value for key, value in result.items() if key != "report"}, ensure_ascii=False, indent=1),
                encoding="utf-8",
            )
        return 0 if verdict_is_clean(result) else 1
    return selftest(Path(args.baseline), args.live, args.live_patch)


if __name__ == "__main__":
    sys.exit(main())
