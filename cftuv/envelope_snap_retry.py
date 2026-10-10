"""Повторная попытка домена с масштабом привязки, сохраняющим плоскость патча: `SOURCE_SNAP_PLANE_PRESERVED_RETRY_V1`.

ЗАЧЕМ. Привязка источника к решётке выбирает наименьший масштаб окна, на котором задуманно прямые углы восстановлены. На почти осевой
плоскости (`cover.008`: патчи 118, 629, 630, 1002, 1005) этот масштаб рвёт плоскость патча: координаты, отличающиеся на 1 ulp float32,
садятся в разные узлы, и ниже по конвейеру патч идёт по near-planar ветке с приведённым базисом решётки плоскости (длины в десятки
километров, Грам 1e-19, окна вееров 1e-13...1e-18). Закон `PLANE_PRESERVING_V1` ядра берёт первый масштаб, на котором углы восстановлены
И патч лежит точно в своей плоскости, и восстанавливает все пять доменов. Как закон ПО УМОЛЧАНИЮ он двигает уже построенные домены
(`building`: углы декали на 37.8-75.4 мм, топология 1053 -> 1050 вершин) и отвергнут.

ЧТО ЗДЕСЬ. Закон заказывается ТОЛЬКО повтором. Домен сначала считается ровно как всегда; если и только если он кончился названным отказом
из ЗАМКНУТОГО множества `SNAP_LOTTERY_REFUSALS`, он пересчитывается с нуля (снапшот, подготовка, покрытие, материализация) под законом
`PLANE_PRESERVING_V1`. Исход повтора записан: построенный домен несёт диагностику `SOURCE_SNAP_PLANE_PRESERVED_RETRY_V1` с именем
ПЕРВОНАЧАЛЬНОГО отказа; повторный отказ называется своим исходом, а первоначальный остаётся в причине (`cause`). Построенный домен повтор
не касается вовсе (его результат не читается и не заменяется), поэтому он побитово тот же.

КЛЮЧИ. Повтор идёт тем же конвейером прогона на экспорте топологии с законом (`EnvelopeTopologyExportV1.with_grid_scale_retry`); закон входит в
ключи кэшей метрики, геометрии, подготовки, результата и содержимого (`envelope_topology_export.metric_law_key`), в привязку и ключ записи
сборки: подготовка повтора никогда не ложится под ключ обычной, и тёплое нажатие берёт повтор из кэша так же, как обычный домен.

ПОЧЕМУ ЗАМКНУТОЕ МНОЖЕСТВО. Повтор бесплатен только там, где причина отказа - лотерея привязки. Новое имя отказа в множество не попадает
молча: нужна строка `DECISIONS.md` и тест, потому что повтор, запущенный по чужому отказу, был бы лишней работой с чужим именем.
"""

from __future__ import annotations

import re
from dataclasses import replace

#: Имя исхода повтора. Записывается диагностикой построенного домена; повторный отказ называется исходом самого повтора.
SNAP_RETRY_OUTCOME = "SOURCE_SNAP_PLANE_PRESERVED_RETRY_V1"
#: Диагностика ядра-стороны закона: какие масштабы пропущены. Пишется только у подготовки, чей сертификат решётки их несёт.
SNAP_SCALE_DIAGNOSTIC = "SOURCE_SNAP_PLANE_PRESERVED_SCALE_V1"

#: ЗАМКНУТОЕ множество отказов лотереи привязки (каждое наблюдено на `cover.008` на привязке, порвавшей плоскость патча):
#: * `DENSITY_RATIONAL_AUTHORITY_EXHAUSTED` - окна вееров 1e-13...1e-18 на приведённом базисе плоскости (патчи 118, 629, 630; исход домена
#:   `COVERAGE_IS_NOT_EXACT`, причина `preparation:PLAN_IS_NOT_COMPILED: <имя>`);
#: * `PLANAR_OWNER_INTERIOR_DIRECTION_REQUIRED` - near-planar решётка не реализует поворот сектора владельца (патч 1005; исход домена
#:   `COVERAGE_IS_NOT_EXACT`, причина `preparation:DOMAIN_GEOMETRY_REFUSED: <имя>: ...`);
#: * `SURFACE_OFFSET_NORMAL_OPPOSES_TRIANGLE` - нормаль смещения вершины против нормали треугольника на порванной плоскости (патч 1002; исход
#:   домена - само имя).
#: Это и есть класс «отказ near-planar решётки»: отдельного имени у него нет, он назван теми тремя исходами, в которые приведённый базис
#: превращается дальше по конвейеру.
SNAP_LOTTERY_REFUSALS = frozenset(
    {
        "DENSITY_RATIONAL_AUTHORITY_EXHAUSTED",
        "PLANAR_OWNER_INTERIOR_DIRECTION_REQUIRED",
        "SURFACE_OFFSET_NORMAL_OPPOSES_TRIANGLE",
    }
)

#: Счётчики прогона: домены, отправленные на повтор; построенные повтором; отказавшие снова.
SNAP_RETRY_ATTEMPTED = "PRODUCTION_SNAP_RETRY_ATTEMPTED"
SNAP_RETRY_RECOVERED = "PRODUCTION_SNAP_RETRY_RECOVERED"
SNAP_RETRY_REFUSED_AGAIN = "PRODUCTION_SNAP_RETRY_REFUSED_AGAIN"

_MATERIALIZED = "MATERIALIZED"
_RAISED = "PRODUCTION_DOMAIN_RAISED"
_WORD = re.compile(r"[A-Z][A-Z0-9_]*")


def refusal_name(result) -> str:
    """Имя отказа домена: исход, а у отказа подготовки (`COVERAGE_IS_NOT_EXACT`, `preparation:<исход>: <имя>...`) - первое имя причины.

    Сравнивается ЦЕЛОЕ слово причины, а не подстрока: имя, содержащее имя из множества, лотереей не считается.
    """

    outcome = str(result.outcome)
    detail = str(result.detail or "")
    if outcome == "COVERAGE_IS_NOT_EXACT" and detail.startswith("preparation:"):
        found = _WORD.match(detail.partition(": ")[2].lstrip())
        if found is not None:
            return found.group(0)
    return outcome


def lottery_refusal(result) -> str | None:
    """Имя отказа из `SNAP_LOTTERY_REFUSALS`, если домен кончился им, иначе `None` (построенный домен и чужой отказ повтора не получают)."""

    if str(result.outcome) == _MATERIALIZED:
        return None
    name = refusal_name(result)
    return name if name in SNAP_LOTTERY_REFUSALS else None


def snap_scale_notes(prepared) -> tuple[str, ...]:
    """Диагностика закона масштаба: какие масштабы окна пропущены из-за порванной плоскости; пусто, если закон выбор не сдвинул.

    Читает сертификат решётки кадра подготовки. У домена, посчитанного обычным законом, проб `RELATIONS_RESTORED_PLANE_TORN` нет, и его
    диагностики остаются побитово прежними.
    """

    certificate = getattr(getattr(getattr(prepared, "context", None), "frame", None), "grid_certificate", None)
    skipped = getattr(certificate, "scales_skipped_for_plane", 0)
    if not skipped:
        return ()
    torn = ",".join(
        str(trial.scale) for trial in certificate.scale_trials if trial.outcome.value == "RELATIONS_RESTORED_PLANE_TORN"
    )
    step = certificate.window_step
    return (
        f"{SNAP_SCALE_DIAGNOSTIC}: source scale {certificate.source_scale} (step {step.numerator}/{step.denominator}) chosen over "
        f"{skipped} finer scale(s) that restore the intended right angles but tear the patch plane (skipped_scales=[{torn}], "
        f"reason=RELATIONS_RESTORED_PLANE_TORN, trials={len(certificate.scale_trials)})",
    )


def merged_result(original, retried, name: str):
    """Итог домена после повтора: построенный повтором (с записью исхода и первоначального отказа) либо отказ повтора с причиной.

    Повторный отказ ИМЕНУЕТСЯ исходом повтора, а первоначальный остаётся в `detail` как причина. Исключение внутри повтора - не названный
    отказ: домен остаётся при первоначальном отказе, а исключение записано в его причине.
    """

    first = f"{original.outcome}" + (f": {original.detail}" if original.detail else "")
    named = name if str(original.outcome) == name else f"{name} ({original.outcome})"
    if str(retried.outcome) == _MATERIALIZED:
        note = f"{SNAP_RETRY_OUTCOME}: the domain refused as {named} and was rebuilt under the plane-preserving scale law (PLANE_PRESERVING_V1)"
        return retried.with_changes(diagnostics=(note, *retried.diagnostics))
    if str(retried.outcome) == _RAISED:
        return original.with_changes(
            detail=f"{original.detail} [{SNAP_RETRY_OUTCOME}: the retry raised, the original refusal stands: {retried.detail}]"
        )
    again = f"{SNAP_RETRY_OUTCOME}: refused again as {refusal_name(retried)}"
    return retried.with_changes(
        detail=f"{retried.detail} [{again}; cause {name}: {first}]",
        diagnostics=(*retried.diagnostics, f"{again} (cause {name})"),
    )


def retry_snap_lottery(run, entries, results, pool, *, scan, dispatch, collect):
    """Результаты прогона, где домены с отказом лотереи привязки пересчитаны законом `PLANE_PRESERVING_V1`; остальные - те же объекты.

    `scan`, `dispatch`, `collect` - ступени продуктового прогона (`_scan`, `_dispatch`, `_domain_results`): повтор идёт ТЕМ ЖЕ конвейером на
    экспорте топологии с законом и своими ключами кэшей, поэтому воркеры, кэши сессии и хранилище по содержимому работают как у обычного
    прогона и не путают подготовку повтора с обычной. Счётчики записываются в профиль прогона.
    """

    targets = [(entry, result, lottery_refusal(result)) for entry, result in zip(entries, results)]
    targets = [item for item in targets if item[2] is not None]
    if not targets:
        return results
    export = run.topology_export.with_grid_scale_retry()
    retry = replace(
        run,
        topology_export=export,
        hooks=run.controller.worker_export_hooks(export, run.profile),
        patch_ids=tuple(entry.patch_id for entry, _result, _name in targets),
        relabeled=[],
        relabel_failures=[],
        registered=[],
        scan_records={},
    )
    retry_entries = scan(retry)
    work = [item for item in retry_entries if item.needs_work]
    done, refused = dispatch(
        retry,
        [item for item in work if item.prepared is not None or item.carried is not None],
        [item for item in work if item.prepared is None and item.carried is None],
        pool,
    )
    retried = {entry.patch_id: result for entry, result in zip(retry_entries, collect(retry_entries, done, refused))}
    merged = {entry.patch_id: merged_result(result, retried[entry.patch_id], name) for entry, result, name in targets}
    recovered = sum(1 for result in merged.values() if str(result.outcome) == _MATERIALIZED)
    run.profile.set_counter(SNAP_RETRY_ATTEMPTED, len(targets))
    run.profile.set_counter(SNAP_RETRY_RECOVERED, recovered)
    run.profile.set_counter(SNAP_RETRY_REFUSED_AGAIN, len(targets) - recovered)
    return [merged.get(entry.patch_id, result) for entry, result in zip(entries, results)]


__all__ = (
    "SNAP_LOTTERY_REFUSALS",
    "SNAP_RETRY_ATTEMPTED",
    "SNAP_RETRY_OUTCOME",
    "SNAP_RETRY_RECOVERED",
    "SNAP_RETRY_REFUSED_AGAIN",
    "SNAP_SCALE_DIAGNOSTIC",
    "lottery_refusal",
    "merged_result",
    "refusal_name",
    "retry_snap_lottery",
    "snap_scale_notes",
)
