"""Собирает RECEIPT.json из файлов этого каталога (ничего не пересчитывает).

    python artifacts/numeric_repr/make_receipt.py

Числа берутся из `baseline_*.json`, `attribution.json`, `instrument_*.json`,
`rational_share_*.json`, `proto_results/**` и `compare_*.json`; тексты оценок риска — вручную ниже
(это суждение, а не измерение, и помечено как таковое).
"""

from __future__ import annotations

import glob
import json
from pathlib import Path

HERE = Path(__file__).resolve().parent


def load(name):
    return json.loads((HERE / name).read_text(encoding="utf-8"))


baseline_path = sorted(HERE.glob("baseline_*.json"))[0]
baseline = json.loads(baseline_path.read_text(encoding="utf-8"))
attribution = load("attribution.json")["domains"]
combined_key = next(k for k in attribution if k.startswith("COMBINED"))
combined = attribution[combined_key]
compare_wip = load("compare_wip_vs_baseline.json")
compare_shim = load("compare_shim6_vs_unshimmed.json")


def baseline_summary():
    out = {}
    for density, run in baseline["runs"].items():
        top = []
        for patch in run["order"][:6]:
            row = run["domains"][str(patch)]
            top.append(
                {
                    "patch": patch,
                    "seconds": row["price"]["seconds"],
                    "EXACT_WORK_SPENT": row["price"].get("EXACT_WORK_SPENT"),
                    "faces": row["answer"].get("n_faces"),
                    "SKELETON_seconds": row["price"].get("stage_seconds", {}).get("SKELETON"),
                }
            )
        out[f"d{density}"] = {
            "wall_seconds": run["wall_seconds"],
            "sum_domain_seconds": run["sum_domain_seconds"],
            "max_domain_seconds": run["max_domain_seconds"],
            "outcomes": run["outcomes"],
            "top6_by_seconds": top,
            "non_exact": {
                p: {"outcome": v["answer"]["outcome"], "detail": v["answer"]["detail"][:160]}
                for p, v in run["domains"].items()
                if v["answer"]["outcome"] != "EXACT/EXACT"
            },
        }
    return out


def chain_table():
    order = [
        "none", "sign", "sign+compare", "sign+compare+mul", "sign+compare+mul+addsub",
        "sign+compare+mul+addsub+radical",
        "sign+compare+mul+addsub+radical+floorcache",
    ]
    rank = ["sign", "compare", "mul", "addsub", "radical", "floorcache"]
    data: dict[str, list] = {}
    for f in glob.glob(str(HERE / "proto_results" / "chain" / "*.json")):
        r = json.loads(Path(f).read_text(encoding="utf-8"))
        tag = "+".join(sorted(r["shims"], key=rank.index)) or "none"
        data.setdefault(tag, []).append(r)
    rows = []
    base = None
    prev = None
    for tag in order:
        xs = [r["seconds"] for r in data.get(tag, [])]
        if not xs:
            continue
        mean = sum(xs) / len(xs)
        base = mean if tag == "none" else base
        rows.append(
            {
                "shims": tag,
                "passes": len(xs),
                "seconds_mean": round(mean, 2),
                "seconds_min": round(min(xs), 2),
                "vs_none_pct": round(100 * (mean / base - 1), 1),
                "step_seconds": None if prev is None else round(mean - prev, 2),
                "answer_budget_signcounts_equal": all(
                    r["answer_equal"] and r["budget_articles_equal"] and r["sign_counts_equal"]
                    for r in data[tag]
                ),
            }
        )
        prev = mean
    return rows


def matrix_table():
    out = {}
    for f in sorted(glob.glob(str(HERE / "proto_results" / "p*_d2_*.json"))):
        r = json.loads(Path(f).read_text(encoding="utf-8"))
        if len(r["shims"]) not in (0, 6):
            continue
        out.setdefault(f"p{r['patch']}_d2", {})["none" if not r["shims"] else "six_shims"] = {
            "seconds_solo": r["seconds"],
            "SKELETON": r["skeleton_seconds"],
            "answer_equal": r["answer_equal"],
            "budget_articles_equal": r["budget_articles_equal"],
            "sign_counts_equal": r["sign_counts_equal"],
        }
    return out


def attribution_compact():
    def trim(entry):
        return {k: v for k, v in entry.items()}

    out = {"combined_domains": combined_key, "per_domain_share": {}}
    for name, dom in attribution.items():
        if name.startswith("COMBINED"):
            continue
        out["per_domain_share"][name] = {
            "profile_total_seconds": dom["profile_total_seconds"],
            "sympy_inclusive_seconds": dom["sympy_inclusive_seconds_profiled"],
            "sympy_share": dom["sympy_share_of_profile"],
            "fractions_inclusive_seconds": dom["fractions_inclusive_seconds_profiled"],
            "exact_sqrt_sum_public_inclusive_seconds": dom[
                "exact_sqrt_sum_public_inclusive_seconds_profiled"
            ],
            "tottime_bucket_share": dom["tottime_bucket_share_spike_method"],
        }
    out["combined"] = {
        "profile_total_seconds": combined["profile_total_seconds"],
        "sympy_inclusive_seconds": combined["sympy_inclusive_seconds_profiled"],
        "sympy_by_family_top12": combined["sympy_by_family"][:12],
        "sympy_by_kernel_caller_top20": combined["sympy_by_kernel_caller_top25"][:20],
        "sympy_by_entry_point_top15": combined["sympy_by_entry_point_top25"][:15],
        "mpmath_direct_from_kernel": combined["mpmath_direct_from_kernel"],
        "fractions_inclusive_seconds": combined["fractions_inclusive_seconds_profiled"],
        "fractions_by_operation": combined["fractions_by_operation"][:10],
        "fractions_top15_callers": combined["fractions_top15_callers"],
        "fraction_repr": combined["fraction_repr"],
        "exact_sqrt_sum_public_inclusive_seconds": combined[
            "exact_sqrt_sum_public_inclusive_seconds_profiled"
        ],
        "exact_sqrt_sum_by_function": combined["exact_sqrt_sum_by_function"][:10],
        "exact_sqrt_sum_top10_callers": combined["exact_sqrt_sum_top10_callers"],
    }
    out["profile_to_real_seconds_factor"] = {
        "unprofiled_pool_seconds_sum_p6_p1_p7_d2": round(
            sum(baseline["runs"]["2"]["domains"][str(p)]["price"]["seconds"] for p in (6, 1, 7)), 2
        ),
        "profile_total_seconds_sum": combined["profile_total_seconds"],
        "note": "cProfile замедляет домен в ~2.1x; доли верны, секунды профиля переводятся "
        "умножением на это отношение (оценка, не измерение).",
    }
    return out


def instrument_compact():
    out = {}
    for p in (6, 1, 7):
        r = load(f"instrument_p{p}_d2.json")
        out[f"p{p}_d2"] = {
            "first_arg_value_classes": r["first_arg_value_classes_by_function"],
            "symbolic_fallback_counts": r["symbolic_fallback_counts"],
            "density_exact_memo": r["density_exact_memo"],
            "density_exact_memo_key_value_classes": r["density_exact_memo_key_value_classes_first_memo"],
            "expression_srepr_len": r["expression_srepr_len_first_memo"],
            "memos_created": r["memos_created"],
        }
    for p in (7, 6, 1):
        out[f"rational_share_p{p}_d2"] = load(f"rational_share_p{p}_d2.json")["by_function"]
    return out


RANKED = [
    {
        "rank": 1,
        "change": "SqrtSumV1.certified_sign (и тем самым sign) на целых с общим знаменателем",
        "call_site": "kernel/src/cftuv_envelope/exact_sqrt_sum.py: SqrtSumV1.certified_sign/enclosure "
        "<- sign <- event_time.compare_times (29.6 s проф.), EventTimeV1.normalized (8.3), "
        "exact_candidate_view.span_containment (4.9), faces.orientation (0.9)",
        "replacement": "L = lcm знаменателей коэффициентов; a_m = num*(L/den); границы "
        "low/high = sum a*floor_root (+1 для отрицательных), умноженные на положительное "
        "L*2^bits; решения `low>0` / `high<0` те же; `Fraction` не создаётся",
        "profile_share": "sign 44.5 s из 222.5 (20.0%) на 3 доменах; Fraction-операций в enclosure 10.5M",
        "measured_p7_d2_solo": "18.66 -> 15.38 c (-17.5%, 3 прохода)",
        "expected_saving_3_heaviest_d2_pool_seconds": "~ -18 c из 106 (по измеренным -17.5%)",
        "risk_answers": "нет по построению: те же isqrt и те же решения; SIGN_COUNTS и статьи бюджета "
        "равны (проверено на 244 строках ворот и прототипом на 808 тестах ядра)",
        "risk_digests": "нет: хранимые `terms`/`Fraction` не меняются, enclosure() для отчётов остаётся",
    },
    {
        "rank": 2,
        "change": "SqrtSumV1.__mul__ с накоплением целых числителей и одним нормированием дроби на член",
        "call_site": "exact_sqrt_sum.py:SqrtSumV1.__mul__ <- _divide_with_prime_universe (40.0 s проф. из 52.3 "
        "cum __mul__), divided_by (4.7), faces.orientation (3.3), event_time._event_point (2.8)",
        "replacement": "int-форма (общий знаменатель) обоих операндов, merge по радиканту на целых, "
        "Fraction(n, L1*L2) один раз на результирующий член",
        "profile_share": "__mul__ 52.3 s (23.5%); 10.7M Fraction-операций в его теле",
        "measured_p7_d2_solo": "после rank1+3: 14.28 -> 11.36 c (-2.92 c, -15.6%)",
        "expected_saving_3_heaviest_d2_pool_seconds": "~ -16 c",
        "risk_answers": "нет: коэффициент остаётся `Fraction` (не int) — repr/сортировка/дайджест; "
        "порядок членов сортировкой по радиканту как раньше",
        "risk_digests": "нет при сохранении типа коэффициента Fraction (repr `Fraction(3, 1)` уходит "
        "в ключи сортировки по repr и в дайджесты)",
    },
    {
        "rank": 3,
        "change": "event_time.compare_times и times_are_equal слитно на целых",
        "call_site": "wavefront/event_time.py:260 compare_times: 52.4 s проф. (23.5%) — scaled x2 + __sub__ + sign, "
        "393 725 вызовов",
        "replacement": "разность right.divisor*ld - left.divisor*rd сразу в целых; оболочка 64 бита; "
        "при неудаче фильтра — оригинальный путь (сопряжение и бюджет без изменений); SIGN_COUNTS ведутся так же",
        "profile_share": "compare_times 52.4 s",
        "measured_p7_d2_solo": "после rank1: 15.38 -> 14.28 c (-1.10 c, -5.9%)",
        "expected_saving_3_heaviest_d2_pool_seconds": "~ -6 c",
        "risk_answers": "нет: бюджет передаётся в оригинал на неудаче фильтра; ворота/тесты равны",
        "risk_digests": "нет",
    },
    {
        "rank": 4,
        "change": "SqrtSumV1.__add__/__sub__ без Fraction(0)-заглушек и промежуточного __neg__",
        "call_site": "exact_sqrt_sum.py:__add__ (10.6 s проф. на Fraction), __sub__ через `self + (-other)` (2.2M "
        "genexpr-нег)",
        "replacement": "merge в dict без `get(r, Fraction(0)) + c`; `old - c` напрямую; фильтр нулей по c._numerator",
        "measured_p7_d2_solo": "-0.57 c (-3.1%)",
        "expected_saving_3_heaviest_d2_pool_seconds": "~ -3 c",
        "risk_answers": "нет",
        "risk_digests": "нет (тип Fraction сохранён)",
    },
    {
        "rank": 5,
        "change": "кэш floor_root = isqrt(radicand << 128) по радиканту",
        "call_site": "exact_sqrt_sum.py enclosure / certified_sign (после rank1 isqrt заметен)",
        "replacement": "dict radicand -> floor_root; чистить в reset_factorization_memory()",
        "measured_p7_d2_solo": "-0.64 c (-3.4%), шум ±1 c (в одном из проходов -1.6 c)",
        "expected_saving_3_heaviest_d2_pool_seconds": "~ -3 c",
        "risk_answers": "нет (чистая функция)",
        "risk_digests": "нет; НО процессный кэш — под запретом стиля проекта, если не привязан к "
        "транзакции домена (тот же лад, что reset_factorization_memory)",
    },
    {
        "rank": 6,
        "change": "прогрев импортов ядра в воркере пула до первой задачи",
        "call_site": "воркер пула (cftuv/…pool): первый домен каждого воркера платит 1.0-1.3 c «host_export» "
        "за ленивый импорт wavefront+sympy (медиана остальных 0.026 c); тяжёлые домены идут первыми",
        "replacement": "import cftuv_envelope.wavefront.conveyor в инициализаторе воркера",
        "expected_saving_3_heaviest_d2_pool_seconds": "~ -0.9 c на КРИТИЧЕСКОМ пути стены (а не на секундах домена)",
        "risk_answers": "нет (порядок импорта)",
        "risk_digests": "нет",
    },
    {
        "rank": 7,
        "change": "repr как ключ сортировки: symbolic_component.overlay_signature",
        "call_site": "wavefront/symbolic_component.py:41 overlay_signature: 760 вызовов, 11.7 s проф. (5.3%), "
        "254 705 dataclass-repr; суммарно 6.5M Fraction.__repr__",
        "replacement": "кэш repr записи вершины на время вызова ИЛИ ключ без repr, если сигнатуры только "
        "сравниваются на равенство (аудит: сигнатуры входят в SymbolicSuperlevelClosureV1.signatures)",
        "expected_saving_3_heaviest_d2_pool_seconds": "до ~ -5 c (5.3% профиля), если ключ допустимо сменить",
        "risk_answers": "средний: порядок repr-сортировки попадает в tuple, возвращаемый наружу; нужен аудит использования",
        "risk_digests": "repr Fraction/dataclass ЗАМОРОЖЕН в FROZEN_DIGESTS-путях (superlevel repr-сортировки); "
        "менять тип/формат repr нельзя, менять ключ — только после аудита",
    },
    {
        "rank": 8,
        "change": "рациональные быстрые пути sympy: ExactPlanarMetric.dot_g, boundary._contact_candidates, "
        "adaptive_density_fan._dual_dot/_subturn (Fraction + одна sp.Rational)",
        "call_site": "reference/metric.py:340 dot_g (4.2 s проф.; чисто рациональны 25% вызовов на p1, 49% на p6, 61% на p7), "
        "reference/boundary.py:72 _contact_candidates (5.6 s; рациональны 0% на p1, 32% на p6, 48% на p7), "
        "adaptive_density_fan.py:174/401 (1.6 + 1.4 s); всего sympy входит в 23.1 s из 222.5 (10.4%), "
        "Expr-арифметика 17.6 s из них",
        "replacement": "при all-Rational входе считать на Fraction и собрать sp.Rational/Integer: srepr результата "
        "= `Rational(p, q)`/`Integer(n)` — строка идентичности та же (как _rational_srepr)",
        "expected_saving_3_heaviest_d2_pool_seconds": "~ -1..-3 c: рациональны лишь 25-49% вызовов dot_g на самых "
        "тяжёлых p1/p6 и 0-32% _contact_candidates, иррациональные остаются в sympy; после rank1-5 доля sympy "
        "растёт до ~14-19% оставшегося времени (стадии PLAN_COMPILE+EFFECTIVE_ALPHA ~3.5-4 c из 22), но достать "
        "её можно лишь частично",
        "risk_answers": "средний: множества sp.Expr (candidates) итерируются в порядке sympy-хэша, ничья при "
        "равных alpha зависит от него; точки/alpha идут в stable_id(...)",
        "risk_digests": "ВЫСОКИЙ для иррациональных (srepr = идентичность ExactScalar -> stable_id -> "
        "envelope_instance_id в ответе, point_key, RawCoverage.semantic_digest, замороженные хэши "
        "402ec97a…/fe440be5…); для рациональных — нулевой при сохранении строки",
    },
    {
        "rank": 9,
        "change": "замена sp.factor/exact_normalize и интервалов mpmath на SqrtSumV1",
        "call_site": "planar_types.exact_normalize (sp.factor, 7 442 вызова на домен, все иррациональные, 1.9 s проф.), "
        "interval_enclosure/_certified_interval_sign (mpmath, 2.5+0.6 s), _density_exact_sign (углы: sin/cos/atan/pi)",
        "replacement": "НЕ рекомендуется: результат sp.factor определяет строку srepr (идентичность), углы "
        "не выражаются в SqrtSumV1",
        "expected_saving_3_heaviest_d2_pool_seconds": "<= -2 c",
        "risk_answers": "высокий",
        "risk_digests": "высокий (менялись бы ExactScalar.expression и всё, что от них хэшируется)",
    },
]


receipt = {
    "task": "срез 6, шаг 1 — числовое представление: ворота равенства, атрибуция по местам вызова, "
    "выполнимость замен",
    "commit": {
        "sha": baseline["sha"],
        "sha_full": baseline["sha_full"],
        "tree_clean_at_baseline": baseline["tree_clean"],
        "note": "Baseline снят на ЧИСТОМ дереве HEAD. Позже в worktree появилась чужая незакоммиченная "
        "правка kernel/.../wavefront/conveyor.py (_preparation_outcome добавляет partition.detail к detail "
        "отказа) + DECISIONS.md/ROADMAP.md/artifacts/faces_chain_refusal/. Ворота САМИ её поймали: единственное "
        "расхождение ANSWER на 244 строках — patch17 d2, ключи detail и fp_meta "
        "(compare_wip_vs_baseline.json). После коммита этой правки baseline нужно переснять.",
    },
    "machine": {
        "cores_logical": baseline["cores"],
        "python": baseline["python"],
        "sympy": baseline["sympy"],
        "workers": baseline["workers"],
        "alpha": baseline["alpha"],
    },
    "gate": {
        "file": "artifacts/numeric_repr/gate.py",
        "commands": {
            "baseline": "python artifacts/numeric_repr/gate.py run --workers 8 --densities 1,2 --out "
            "artifacts/numeric_repr/baseline_<sha>.json",
            "check": "python artifacts/numeric_repr/gate.py run --workers 8 --densities 1,2 --out new.json "
            "--baseline artifacts/numeric_repr/baseline_<sha>.json   # код возврата 1 при расхождении ANSWER",
            "diff_two_files": "python artifacts/numeric_repr/gate.py compare A.json B.json [--json-out r.json]",
            "negative_controls": "python artifacts/numeric_repr/gate.py selftest baseline.json --live",
            "subset": "--only 6,7 (сравнение по пересечению); --densities 2",
            "prototype_shims": "NUMERIC_REPR_SHIMS=sign,compare,mul,addsub,radical,floorcache python gate.py run ...",
        },
        "density_passing": "density=<0..4|None> -> build_envelope_decal_request(..., density=...) -> "
        "envelope_angular_policy (cftuv/envelope_request_policy.py); alpha 0.45 как в sweep.py; "
        "Fan Density 1 и 2 — оба прогоняются",
        "ANSWER_class": [
            "исход подготовки/покрытия, detail (в т.ч. текст отказа), lattice_scale, n_regions/faces/segments",
            "fp_meta, fp_geometry (repr-дайджест regions/faces/segments как в pool_sweep), "
            "fp_geometry_value (числово-нормализованный: int и Fraction(n,1) равны)",
            "fp_structural_counters (CONVEYOR_*), fp_host_counters",
            "fp_deep_{bridge,skeleton,partition,owners}[_value], fp_deep_lattice: канонический обход ВНУТРЕННОСТЕЙ "
            "подготовки (узлы скелета с точными позициями, обязательства, отказы кандидатов, грани, владельцы); "
            "ExactWorkBudgetV1 вынут (это цена)",
            "исключение в домене (EXCEPTION/<тип>) — тоже исход",
        ],
        "PRICE_class": [
            "EXACT_WORK_* (шесть статей + SPENT), leaked_unbudgeted, SIGN_COUNTS, seconds, host_export/prepare/"
            "coverage seconds, stage_seconds — перечисляются в compare, но не ошибка",
        ],
        "diff_classes": "ANSWER diff (значение) / ANSWER REPR-ONLY (строгий дайджест разошёлся, числовой нет) / "
        "STRUCTURAL (набор доменов); любой из них -> MISMATCH",
        "reproducibility": "повторный прогон на том же дереве: ANSWER IDENTICAL на 244 строках; статьи бюджета "
        "равны побитово (PRICE-расхождений по EXACT_WORK_* нет); одиночный процесс даёт те же ответы, что пул",
        "negative_controls": {
            "synthetic_on_baseline": "selftest: бит в fp_geometry, бит в fp_deep_skeleton, n_faces+1, смена исхода, "
            "пропавший домен — все MISMATCH; изменение ТОЛЬКО цены (секунды, SPENT, GCD, стадии) — IDENTICAL "
            "(selftest_output.txt: SELFTEST PASSED)",
            "live_input_perturbation": "живой домен при alpha 0.5 вместо 0.45 -> расхождение fp_geometry/fp_meta; те же "
            "входы -> совпадение",
            "live_arithmetic_mutation": "шим bad_area (+1e-12 к площади грани) на patch100/103 d2 -> расхождение по "
            "13 ключам (исход уходит в отказ); шим int_types (значение то же, тип int) не меняет ответ, потому что "
            "поздние операции нормируют тип обратно — честный пример, что ловится ответ, а не внутренний тип; "
            "шим bad_mul (+1e-12 к коэффициенту) зацикливает марш (не завершается) — такие мутации гонять с timeout",
            "real_detection": "чужая правка conveyor.py ловится как ANSWER diff на patch17 d2 (compare_wip_vs_baseline.json)",
        },
    },
    "baseline": {
        "file": baseline_path.name,
        "size_bytes": baseline_path.stat().st_size,
        "summary": baseline_summary(),
        "wall_seconds_repeat_runs": {
            "d1": {"baseline": 27.909, "rerun_same_tree": 27.017, "wip_tree": 27.955},
            "d2": {"baseline": 39.372, "rerun_same_tree": 41.28, "wip_tree": 39.72},
            "note": "шум стены ±5%; секунды доменов — цена, не ответ",
        },
        "gate_runtime": "~70 с на оба density при 8 воркерах (+ старт пула)",
    },
    "attribution": {
        "domains": "patch 6, 1, 7 при Fan Density 2 (три самых тяжёлых по секундам baseline: 38.9 / 37.5 / 29.6 с)",
        "method": "cProfile + pstats callers; точка входа = функция sympy/fractions/exact_sqrt_sum, вызванная из "
        "кода ВНЕ своего пакета; встроенные/служебные вызывающие (sorted, repr, <string>) переносят вес на вызывающих "
        "пропорционально времени (до 3 прыжков; для sorted это даёт артефакты — помечено); импорт sympy/ядра "
        "вынесен прогревом малым доменом",
        "files": ["attribution.json", "instrument_p6_d2.json", "instrument_p1_d2.json", "instrument_p7_d2.json",
                  "rational_share_p7_d2.json", "rational_share_p6_d2.json", "rational_share_p1_d2.json"],
        "tables": attribution_compact(),
        "values_and_memo": instrument_compact(),
        "value_summary": {
            "rational": "Expr-арифметика в основном над sp.Rational: exact_sign 59-75% Rational (p1/p6/p7), dot_g 61%, "
            "_contact_candidates 48% чисто рациональны; gram/inverse_gram — всегда sp.Rational (из ExactRationalV1) или "
            "Integer-единичная",
            "quadratic_surds": "sqrt-суммы (1-2 радикала, srepr ~50 символов, макс 125): ВСЕ 7 442 вызова "
            "sp.factor (exact_normalize), _density_srepr, expressions-memo",
            "angles": "sin/cos/atan/pi только в Density A: _density_exact_sign (25-34% вызовов), ключи intervals/signs "
            "memo (58-72% ключей intervals); в Fraction/SqrtSumV1 не выражаются",
        },
        "ExactPlanarMetric": "frozen dataclass: gram/inverse_gram 2x2 sp.Rational (rational всегда), owner_orientation_sign int, "
        "chart_grid GridSpecV1, source_step Fraction, _density_exact_memo. Методы dot_g/length_g/unit_g/oriented_cross/"
        "owner_normal_g/offset_support_g/angle_g возвращают sp.Expr (127 аннотаций sp.Expr по ядру) — "
        "замена типа gram ломает сигнатуры; быстрый путь изнутри сохраняет sp.Expr-интерфейс. Прецедент: "
        "snap_exact_quadratic_point/_floor_exact_quadratic уже считают на SqrtSumV1 без SymPy.",
        "_DensityExactMemo": "8 кэшей (expressions, dual_dots, crosses, intervals, signs, sreprs, subturns, support_segments), "
        "на домен 10-11 экземпляров, по сотням записей (expressions 253-355, intervals 1.4-4.4k); попадания: "
        "expressions 4069/220, intervals 4804/4412 (contains), signs 748/3129, dual_dots 647/847, sreprs 60/137. Ключи — "
        "sp.Expr и srepr-строки; значения expressions — sp.Expr, sreprs — srepr-строки (идентичность ExactScalar). "
        "Память ничтожна; цена — хэш sp.Expr (Expr.__hash__ 472k вызовов, 0.64 с проф.). Пиклится пустым.",
    },
    "digest_dependence_on_sympy_strings": {
        "conclusion": "ДА, зависит — но только через ExactScalar.expression (= srepr): её нельзя менять без сдвига "
        "идентичностей и дайджестов. Путь очереди несёт эту зависимость в ответ: stable_id(..., "
        "ExactScalar.from_value(effective_alpha).expression) -> envelope_instance_id в гранях.",
        "sites": [
            "reference/planar_types.py: ExactScalar.expression = sp.srepr(sp.factor(sp.cancel(v))) (_canonical_expr) "
            "либо сборка `Integer(n)`/`Rational(p, q)` напрямую (_rational_srepr)",
            "reference/angular.py:248-254 _density_srepr: sp.srepr(expression) БЕЗ factor (структура дерева sympy = строка)",
            "stable_id(...) хэширует str(part): angular.py:1279, boundary.py:238, cap.py:46, strip.py:43, equality_locus.py:160-164,"
            " mutual_arrival.py:142/454-461, policy_b.py:166-183/351-376/620",
            "point_key(point) = (x.expression, y.expression): ключ и порядок в arrangement.py:956/1091/1222-1247, "
            "policy_b.py:153-155/356/376/524, boundary.py:231, domain_geometry.py:191/208",
            "reference/common.py:269 support_geometry_key = sorted(строки srepr)",
            "RawCoverageResultV1.semantic_digest = sha256(canonical_json(result)) включает exact_area_expression (srepr); "
            "заморожен в kernel/tests/test_building_002_point_contact_fixture.py:260 и "
            "test_sem_clb_02_chain_straight_regression.py:509 (402ec97a…); "
            "kernel/artifacts/kernel_audit_exact_proof/p0_3_post_p0_2b_absolute_digests.json",
            "тексты исключений: str(expression)/sp.srepr в IntervalEnclosureUnsupported, CertifiedPredicateUndecidable, "
            "ExactQuadraticFieldUnsupported, DensityIntervalEnclosureUnsupported — могут попасть в detail отказа",
            "kernel/tests/test_exact_numeric_fast_path.py (докстринг): «Строка srepr — идентичность ExactScalar и вход "
            "семантических дайджестов»",
        ],
        "not_dependent": [
            "FROZEN_DIGESTS скелета (wavefront/digest.py): JSON из SqrtSumV1/EventTimeV1 — `terms` Fraction, sympy нет",
            "но: repr(Fraction)/repr(dataclass) — КЛЮЧ СОРТИРОВКИ в ~40 местах wavefront (superlevel*, symbolic_*), "
            "и exact_identity.py прямо пишет «repr уходит в дайджест»; формат `Fraction(n, d)` и тип коэффициента "
            "(Fraction, не int) заморожены косвенно; замена типа Fraction на другой класс требует побитового repr",
            "queue-путь после conveyor._read_arrival_law (exact_rational -> Fraction) sympy не содержит: fp_deep_* по "
            "скелету/разбиению на всех доменах имеют deep_sympy_nodes=0",
        ],
    },
    "prototypes": {
        "files": ["proto_shims.py", "proto_run.py", "run_proto_matrix.sh", "run_proto_chain.sh", "pytest_shims.py"],
        "idea": "runtime-шимы (методы подменяются в памяти; файлы ядра не правятся): точная та же арифметика на целых; "
        "ответ проверяется теми же воротами, замороженные дайджесты — тестами ядра",
        "chain_patch7_d2_solo": chain_table(),
        "six_shims_solo_three_domains": matrix_table(),
        "full_gate_with_six_shims_vs_unshimmed_same_tree": {
            "verdict": compare_shim["verdict"],
            "answer_diffs": len(compare_shim["answer_diffs"]),
            "price_changes_only": sorted(compare_shim["price_changes"]),
            "d1_wall": "27.955 -> 17.838 c (-36%)",
            "d2_wall": "39.72 -> 24.321 c (-39%)",
            "d1_sum": "146.73 -> 90.84 c",
            "d2_sum": "200.21 -> 119.80 c",
            "d2_heaviest": "patch6 39.2 -> 22.8, patch1 37.7 -> 22.1, patch7 29.8 -> 15.3 c (-42/-41/-49%)",
            "EXACT_WORK_and_SIGN_COUNTS": "без изменений ни на одном из 244 доменов",
        },
        "kernel_tests_with_shims": "см. kernel_tests_frozen_digests",
    },
    "kernel_tests_frozen_digests": {
        "command": "PYTHONSAFEPATH=1 PYTHONPATH=kernel/src;artifacts/numeric_repr NUMERIC_REPR_SHIMS=<6 шимов> "
        "python -m pytest -p pytest_shims <файлы>",
        "files": [
            "test_wavefront_motorcycle_graph.py (FROZEN_DIGESTS)", "test_exact_work_budget.py",
            "test_wavefront_proof_obligations.py (_FROZEN_DIGESTS_ORACLE)", "test_wavefront_exact_time.py",
            "test_wavefront_event_queue.py", "test_wavefront_same_time_closure.py",
            "test_wavefront_partial_source.py", "test_exact_canonicalization_memory.py",
            "test_exact_identity_shadow.py", "test_wavefront_weighted_wall_differential.py (абсолютные дайджесты P0-3)",
        ],
        "result_unshimmed": "808 passed, 471 с",
        "result_with_six_shims": "808 passed, 245 с (шимы установлены: stderr плагина подтверждает)",
        "meaning": "прототипы замен на SqrtSumV1/event_time не сдвигают ни один замороженный дайджест скелета, "
        "ни абсолютные дайджесты P0-3; то же должно выдержать настоящее внедрение тех же решений",
    },
    "ranked_changes": RANKED,
    "compare_wip_vs_baseline_answer_diffs": compare_wip["answer_diffs"],
}

(HERE / "RECEIPT.json").write_text(
    json.dumps(receipt, ensure_ascii=False, indent=1), encoding="utf-8"
)
print("RECEIPT.json", (HERE / "RECEIPT.json").stat().st_size, "bytes")
