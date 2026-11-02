"""Контекст непрерывного UV: хост собирает подготовки, уравнения и неизменяемый план принадлежат ядру."""

from __future__ import annotations

from dataclasses import replace

from .envelope_request_policy import ENVELOPE_UV_POLICY_DIRECT_STRIP, ENVELOPE_UV_POLICY_UNIFORM_TRIM_FAN


def context_key(key: tuple, uv_context) -> tuple:
    """Digest глобальной фазы расширяет только UV-ключ; без контекста прежний tuple тождественен."""

    return key if uv_context is None else (*key, str(uv_context.digest))


def _current_preparation(run, patch_id, domain_id, selected):
    """Текущая подготовка домена: запись этой ревизии либо обычный экспорт и подготовка через кэш сессии.

    Попадание хранилища содержимого может оставить подготовку прежней ревизии; её здесь не читают напрямую.
    Нормальный экспорт строит текущие идентичности, а `get_conveyor_preparation` гидратирует LazyPreparation.
    """

    from .envelope_kernel_backend import entered_backend
    from .envelope_production_scan import _inputs_of
    from .envelope_production_task import prepare_for_production_recorded

    controller = run.controller
    records = run.scan_records.get("records") or {}
    record = records.get(patch_id)
    if (record is not None and record.selected == selected
            and record.snapshot is controller.peek_domain_snapshot(run.topology_export, patch_id)):
        prepared = controller.peek_conveyor_preparation_by_key(record.prep_key)
        if prepared is not None:
            return prepared
    with entered_backend(run.backend, run.skeleton_backend, run.embedding_backend, density_backend=run.density_backend) as ledger:
        snapshot, request = _inputs_of(run, (patch_id, domain_id, selected), run.hooks.snapshot_provider)

        def build():
            return prepare_for_production_recorded(
                snapshot, request, backend=run.backend, skeleton_backend=run.skeleton_backend,
                embedding_backend=run.embedding_backend, density_backend=run.density_backend, borrowed_ledger=ledger,
            )[0]

        return controller.get_conveyor_preparation(
            run.revision, domain_id, selected, request, build, profile=run.profile,
            skeleton_id=run.skeleton_id, grid_scale_law=run.grid_scale_law,
        )


def _preparations(run, final_runs) -> tuple:
    """Все исходно выбранные домены, включая домены без материализованного результата; отказы остаются в обычном прогоне."""

    from .envelope_kernel_backend import PreparationRefused
    from .envelope_production_scan import _check_cancel
    from .envelope_request_export import EnvelopeHostAdapterError
    from .envelope_scan_memo import patch_domain_id

    prepared, excluded = [], []
    for patch_id in run.patch_ids:
        _check_cancel(run)
        domain_id = patch_domain_id(run.revision, patch_id)
        selected = frozenset(run.selected_by_domain[domain_id])
        try:
            item = _current_preparation(final_runs[patch_id], patch_id, domain_id, selected)
        except (EnvelopeHostAdapterError, PreparationRefused) as exc:
            # Не превращаем отказ подготовки в успешную геометрию: финальный прогон назовёт его для того же домена.
            excluded.append((domain_id, str(getattr(exc.outcome, "value", exc.outcome))))
            continue
        except Exception as exc:
            raise ValueError(f"UV_CONTEXT_PREPARATION_INCOMPLETE: {domain_id}: {exc}") from exc
        if item is None:
            raise ValueError(f"UV_CONTEXT_PREPARATION_INCOMPLETE: {domain_id}: no current preparation")
        if item.outcome.value == "EXACT":
            prepared.append(item)
        else:
            excluded.append((domain_id, str(item.outcome.value)))
    run.profile.set_counter("PRODUCTION_UNIFORM_CONTEXT_PREPARATIONS", len(prepared))
    run.profile.set_counter("PRODUCTION_UNIFORM_CONTEXT_REFUSED_PREPARATIONS", len(excluded))
    return tuple(prepared), tuple(excluded)


def _compile_context(prepared, excluded):
    from cftuv_envelope.materialize.uv_phase import compile_uniform_uv_context

    return compile_uniform_uv_context(prepared, excluded_domain_ids=excluded)


def _direct_prepass(run, pool, prepass):
    """Обычный полный DIRECT-путь, включая именованный повтор масштаба; запоминаем, чья подготовка дала итог домена."""

    from .envelope_production_export import _dispatch, _domain_results, _scan, retry_snap_lottery

    final_runs = {patch: run for patch in run.patch_ids}

    def retry_scan(retry):
        final_runs.update((patch, retry) for patch in retry.patch_ids)
        return _scan(retry)

    entries, done, refused = prepass(run, pool)
    retry_snap_lottery(run, entries, _domain_results(entries, done, refused), pool,
                       scan=retry_scan, dispatch=_dispatch, collect=_domain_results)
    return final_runs


def with_uniform_context(run, pool, *, prepass):
    """Один полный DIRECT-проход на первом Build, затем один вызов чистого компилятора ядра. Тики используют план Build."""

    if run.uv_policy_id != ENVELOPE_UV_POLICY_UNIFORM_TRIM_FAN:
        if run.uv_context is not None:
            raise ValueError("UV_UNIFORM_PHASE_CONTEXT_POLICY_MISMATCH: a UV phase context belongs to Uniform trim")
        return run
    if run.uv_context is not None:
        return run
    direct = replace(
        run, uv_policy_id=ENVELOPE_UV_POLICY_DIRECT_STRIP, lean=False, served={}, skipped=frozenset(),
        scan_records={}, relabeled=[], relabel_failures=[], registered=[], byte_check_ledger=[],
    )
    final_runs = _direct_prepass(direct, pool, prepass)
    run.byte_check_ledger.extend(direct.byte_check_ledger)
    run.profile.set_counter("PRODUCTION_UNIFORM_CONTEXT_PREPASS", 1)
    context = _compile_context(*_preparations(direct, final_runs))
    for name, value in getattr(context, "work_counters", ()):
        run.profile.set_counter(f"PRODUCTION_UNIFORM_CONTEXT_{name}", value)
    return replace(run, uv_context=context)
