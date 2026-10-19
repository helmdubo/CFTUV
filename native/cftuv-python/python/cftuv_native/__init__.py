"""Шим нативного ускорителя: ЕДИНСТВЕННЫЙ импортёр расширения `cftuv_native._core` (правило — `tests/test_architecture.py`).

Шим переводит объекты ядра в буферы целой операции и воспроизводит её побочные эффекты (бюджет точной работы, память
канонизации), как их воспроизводит попадание `clip_memo`. Тихого отката на Python здесь нет: нет расширения — `ImportError`.

Потоки: каждый публичный метод `CostMirror` идёт под ОДНОЙ процессной блокировкой `cost.NATIVE_LOCK` (синхронизация памяти, нативная операция, журнал, статьи и
счётчики — целиком): продукт зовёт `coverage_at` из потока предпросмотра alpha и из главного, а таблицы памяти и сессия — состояние процесса.

Три части:

* числа (`run_number_ops`): сценарий операций в ОДНОМ вызове, только для сверки с эталоном;
* целые операции: `coverage_at` (`wavefront.coverage._coverage_at`, с разбиениями, которые нативная сессия переводит один раз), `clip_geometry`
  (`materialize.clip.clip_geometry`: подъём плоскости переводится один раз, результат строится из Rust, нормали смещения пишутся в плоскость) и `build_skeleton`
  (`wavefront.skeleton.build_skeleton`: полигон читается из объектов Python, `SkeletonV1` строится из Rust, статьи бюджета и строка `superlevel` пишутся в бюджет; контракт — `skeleton_op.py`);
  все сверены с ОДНОЙ версией эталона и отказываются по имени, если дерево ядра ушло от неё (`pin`, `native_status`, `NativePortStale`);
* стоимость (`default_mirror`, `new_mirror`, `sign`, `divided_by`, ...): долгоживущая нативная сессия владеет зеркалом памяти
  канонизации, а `cost.CostMirror` держит зеркало равным настоящим таблицам Python до вызова и применяет журнал изменений
  к ним после (бюджет, `SIGN_COUNTS`, `UNBUDGETED_WORK`, исключения). Подробности — в `cost.py`.

Четвёртая целая операция — `snap_embedding_certificate` (`_embedding._compute_source_snap_embedding_certificate`: чистый лист без состояния, бюджета и памяти; контракт — `embedding_op.py`): отказ порта — `NativePortUnsupported` ДО вычисления,
исключение эталона (`ValueError`) поднимается с его текстом.

Пятая целая операция — `effective_alpha_contacts` (`reference.boundary._native_contacts`: предфильтр «контактов нет» и точный лист контактов ОДНОЙ пары опорный отрезок — отрезок границы; кадр источника
переводится один раз, пара идёт по четырём текстам; чистый лист без состояния; контракт — `effective_alpha_op.py`): пару, которую порт не несёт, он отказывает по имени ДО вычисления (`NativePortUnsupported`),
и её считает эталон со своими счётами и своим резервным путём через sympy.

Шестая целая операция — `snapshot_check` (проверка снапшота анализа, которую воркер EXPORT делает дважды на домен: `validate_analysis_snapshot` и `validate_reference_geometry_payload`; контракт — `snapshot_check_op.py`):
ЧИСТО ИЛИ ОТКАЗ. Расширение либо доказывает, что валидаторы Python не нашли бы ничего, и возвращает свидетельство (с приращениями счётчиков, которые эти валидаторы оставили бы), либо отказывает по имени ДО любого
эффекта (`NativePortUnsupported`), и вызывающий гонит валидаторы Python как раньше: отказ с его текстом выдаёт эталон, а не порт.

Седьмая целая операция — `frame` (допуск плоской рамки домена на выгрузке: первая ступень `planar_metric.build_rational_affine_planar_metric` на источнике, привязанном к решётке, — точная плоскость, окно шага, перебор масштабов, канонический репер;
контракт — `frame_op.py`): ПРИНЯТЬ ИЛИ ОТКАЗАТЬ. Расширение либо строит `RationalAffinePlanarMetricV2`, который построил бы Python, либо отказывает по имени ДО любого эффекта (`NativePortUnsupported`): околоплоская проекция, развёртка
кривого патча и полосы, отказы с их текстом и исключения — это эталон, а не порт.

`radical_v2_eval` — не стадия продукта, а ВОЗМОЖНОСТЬ расширения (`native_status()["radical_v2"]`, закрепление `reference/radical_v2.py`): программа операций над `RadicalSumV2` (`native/cftuv-radical`, побитовое
зеркало эталона S2 — цены, отказы, тексты, провода) для сверки с эталоном и замеров; хост её не зовёт, потребитель появится с переносом ядра плотности на Rust. Контракт программы — `tools/native_radical_v2_corpus.py`.

Отказ целой операции бывает двух видов (`NATIVE_REFUSALS`, `cost.ORACLE_STATUSES`). Исход ЭТАЛОНА (`MaterializationRefusal`, `ExactCanonicalizationWorkBudgetExhausted`,
`OverflowError`, `ValueError`, `ZeroDivisionError`, `KeyError`, ...) оставляет частичные эффекты ровно так, как их оставляет исключение Python. Отказ ПОРТА
(`NativePortStale`, `NativeUnsupportedPython`, `NativePortUnsupported` — в том числе поздний, посреди вычисления —, `NativeDivisionDiverged`) оставляет ВСЁ видимое из
Python состояние, как оно было до вызова: статьи бюджета, `SIGN_COUNTS`, `UNBUDGETED_WORK`, четыре таблицы памяти с порядком, `plane._normal_by_position`, `store`,
`traces`. Вызывающий вправе запустить эталон на тех же бюджете, плоскости и таблицах и получить исход и состояние чистого прогона эталона.

Личность сборки: `native_build_id()` — содержательный отпечаток (sha256) нативного кода и шима, устойчивый к перелинковке (`buildid.py`).

Доступ к слотам (`Fraction`, `SqrtSumV1`, `FaceCoverageV1`, `LocalPoint3V1`): по смещению, найденному пробой настоящего экземпляра, либо по протоколу атрибутов. Режим читается ОДИН раз при импорте
расширения из `CFTUV_NATIVE_SLOTS`: `auto` (умолчание: смещение там, где проба подтвердила раскладку, иначе протокол атрибутов), `raw` (только смещение; раскладка, которую проба не подтвердила, —
названный отказ при привязке классов, а не тихий откат) и `attr` (только протокол атрибутов, проб нет); неизвестное значение — названный отказ импорта. `slot_mode()` называет режим процесса,
`slot_counters()` — какой путь слотов реально прошёл вызов (тест принудительного режима доказывает, что режим сработал, а не предполагает это).
"""

from __future__ import annotations

from . import _core, angular_op, buildid, chain_station_op, codec, cost, density_op, effective_alpha_op, embedding_op, frame_op, materialize_hot_op, materialize_w1_op, pin, snapshot_check_op

__all__ = (
    "CostMirror",
    "NATIVE_REFUSALS",
    "NativeDivisionDiverged",
    "NativePortStale",
    "NativePortUnsupported",
    "NativeUnsupportedPython",
    "apply_silhouette",
    "build_skeleton",
    "chain_station_plans",
    "clip_geometry",
    "clip_seam_run",
    "clip_seam_table",
    "coverage_at",
    "default_mirror",
    "density_compute",
    "density_compute_candidate",
    "density_compute_with_memo",
    "density_compute_with_memo_candidate",
    "DensityNativeInternalErrorV1",
    "divide_with_prime_universe",
    "divided_by",
    "effective_alpha_contacts",
    "exact_mesh",
    "angular_compute",
    "angular_patch",
    "warm_angular",
    "warm_chain_station",
    "frame_metric",
    "frame_raw_layouts",
    "canonical_triangles",
    "last_w1_timings",
    "lift_vertices",
    "source_lift",
    "warm_materialize_hot",
    "last_snapshot_check_timings",
    "last_frame_timings",
    "int_round_trip",
    "last_clip_timings",
    "last_coverage_timings",
    "last_skeleton_timings",
    "native_build_id",
    "native_build_parts",
    "native_status",
    "native_version",
    "new_clip_seam_session",
    "new_skeleton_seam_session",
    "new_mirror",
    "number_op_table",
    "oracle_statuses",
    "prime_support",
    "prime_universe_remembered",
    "radical",
    "radical_sum",
    "radical_v2_eval",
    "reset_slot_counters",
    "run_number_ops",
    "sign",
    "snap_embedding_certificate",
    "skeleton_oracle_statuses",
    "skeleton_seam_run",
    "skeleton_seam_table",
    "slot_counters",
    "slot_mode",
    "snapshot_check",
    "snapshot_check_raw_layouts",
    "warm_snapshot_check",
    "warm_frame",
    "squarefree_split",
    "tree_digest",
)

CostMirror = cost.CostMirror
NativePortStale = pin.NativePortStale
NativePortUnsupported = pin.NativePortUnsupported
NativeUnsupportedPython = pin.NativeUnsupportedPython
NativeDivisionDiverged = cost.NativeDivisionDiverged
DensityNativeInternalErrorV1 = density_op.DensityNativeInternalErrorV1

#: The exceptions `coverage_at` and `clip_geometry` raise for a refusal of the PORT (not an outcome of the oracle). Each of them leaves every Python-visible state
#: exactly as it was before the call (budget articles, `SIGN_COUNTS`, `UNBUDGETED_WORK`, the four memory tables with their order, `plane._normal_by_position`,
#: the `store`, the `traces`), so the caller may run the Python oracle on the same budget, plane and tables. Not in the tuple: the exceptions of the oracle
#: (`MaterializationRefusal`, `ExactCanonicalizationWorkBudgetExhausted`, `OverflowError`, `ValueError`, `ZeroDivisionError`, `KeyError`, ...), which leave the partial
#: effects the oracle's own exception leaves.
NATIVE_REFUSALS = (NativePortStale, NativeUnsupportedPython, NativePortUnsupported, NativeDivisionDiverged)

_DEFAULT: list = []
_BUILD: list = []


def native_version() -> str:
    return _core.version()


def native_build_parts() -> dict:
    """`{"id", "rust", "shim", "version"}`: the content identity of this build and the two halves it is made of (`buildid.py`).

    `rust` is the sha256 of the Rust sources the extension was built from (embedded at build time), `shim` the sha256 of the shim's own `.py` files as they are
    now; both normalise CRLF to LF. Computed once per process.
    """

    with cost.NATIVE_LOCK:
        if not _BUILD:
            rust, shim = _core.source_digest(), buildid.shim_digest()
            _BUILD.append({"id": buildid.compose(rust, shim), "rust": rust, "shim": shim, "version": _core.version()})
        return dict(_BUILD[0])


#: The environment variable the extension reads the process-wide slot mode from, once, when it is imported (see the module note).
SLOT_MODE_ENVIRONMENT = "CFTUV_NATIVE_SLOTS"

#: The slot modes: `auto` (raw where the probe confirms the layout), `raw` (forced, a failed probe is a named refusal), `attr` (forced attribute protocol).
SLOT_MODES = ("auto", "raw", "attr")

#: The names of the four slot counters, in the order the extension reports them.
SLOT_COUNTER_NAMES = ("raw_reads", "attr_reads", "raw_builds", "attr_builds")


def slot_mode() -> str:
    """`raw`, `attr` or `auto`: the process-wide slot mode, read from `CFTUV_NATIVE_SLOTS` once when the extension was imported (a mirror made with `new_mirror(slots=...)` has its own)."""

    return _core.process_slot_mode()


def slot_counters() -> dict:
    """`{"raw_reads", "attr_reads", "raw_builds", "attr_builds"}`: which path the slot accesses of the whole operations took since the process started or `reset_slot_counters()`.

    A read is one slot of an input object (`Fraction._numerator`, `SqrtSumV1.terms`, ...), a build one result instance (`Fraction`, `SqrtSumV1`, `FaceCoverageV1`, `LocalPoint3V1`); `raw` is the
    offset path, `attr` the attribute protocol. The counters are process-wide (every mirror adds to them).
    """

    return dict(zip(SLOT_COUNTER_NAMES, _core.slot_counters()))


def reset_slot_counters() -> None:
    """Zeroes the four slot counters (`slot_counters`)."""

    _core.reset_slot_counters()


def native_build_id() -> str:
    """A content identity of this build: 64 lowercase hex characters, the sha256 of the Rust sources it was built from and of the shim's `.py` files.

    Stable across relinks and rebuilds of the same content (the MSVC link stamps a timestamp and a PDB GUID into `_core.pyd`, so the binary's bytes are NOT the identity),
    different whenever the native code, its manifests or the shim change. See `buildid.py` and `tools/native_build_id.py`.
    """

    return native_build_parts()["id"]


def oracle_statuses() -> tuple:
    """Test-only: the status codes the extension treats as outcomes of the oracle (`refusal.rs`); `cost.ORACLE_STATUSES` is the same table."""

    return tuple(_core.oracle_statuses())


def tree_digest(native_root) -> str:
    """Test-only: the digest of the Rust sources of the workspace at `native_root`, by the algorithm `build.rs` embeds (`digest.rs`, `tools/native_build_id.py`)."""

    return _core.tree_digest(str(native_root))


def native_status() -> dict:
    """`{operation: "available" | "stale(files)" | "unsupported_python"}` for the whole operations (`coverage`, `clip`, `skeleton`, `snap_embedding`, `effective_alpha`, `materialize_w1`, `snapshot_check`) and the capability `radical_v2` (the mirror of the `RadicalSumV2` oracle, not a product stage).

    `unsupported_python` only below the floor (`pin.MINIMUM_PYTHON`, 3.11); the ports are tested on 3.11 and 3.13 (`pin.TESTED_PYTHON`) and nothing in their
    answers depends on the interpreter version (the kernel names the CPython 3.11 sort and float fold explicitly, `_cpython311.py`).
    """

    found = pin.native_status()
    found["angular"] = angular_op.status()
    found["chain_station"] = chain_station_op.status()
    found["materialize_hot"] = materialize_hot_op.status()
    if "density" not in found:
        found["density"] = "pending_validation" if hasattr(_core, "density_compute") else "unavailable"
    return found


def chain_station_plans(snapshot, patch_domain_id):
    """`CHAIN_STATION_PLAN_V1`: the frozenset of `ChainStationPlanV1` the Python law answers, or `NativePortUnsupported` naming why the port declined (see `chain_station_op.py`)."""
    return chain_station_op.chain_station_plans(_core, snapshot, patch_domain_id)


def warm_chain_station() -> bool:
    """Binds the native chain station law to the live kernel ahead of the first domain (`chain_station_op.warm`)."""
    return chain_station_op.warm(_core)


def density_compute(batch):
    """Целый raw batch; до согласованного pin доступен только candidate entry."""
    if not hasattr(_core, "density_compute"):
        raise NativePortUnsupported("density_compute: extension capability is absent")
    return density_op.density_compute(_core.density_compute, batch)


def density_compute_candidate(batch):
    """Явный opt-in для differential; никаких product/default изменений."""
    if not hasattr(_core, "density_compute"):
        raise NativePortUnsupported("density_compute: extension capability is absent")
    return density_op.density_compute(_core.density_compute, batch, candidate=True)


def density_compute_with_memo(batch, snapshot):
    """Density с host-owned immutable successful-attempt snapshot и delta."""
    if not hasattr(_core, "density_compute_with_memo"):
        raise NativePortUnsupported("density_compute_with_memo: extension capability is absent")
    return density_op.density_compute_with_memo(_core.density_compute_with_memo, batch, snapshot)


def density_compute_with_memo_candidate(batch, snapshot):
    """Явная candidate сверка; не включает pin/default."""
    if not hasattr(_core, "density_compute_with_memo"):
        raise NativePortUnsupported("density_compute_with_memo: extension capability is absent")
    return density_op.density_compute_with_memo(_core.density_compute_with_memo, batch, snapshot, candidate=True)


def int_round_trip(value: int) -> int:
    """Test-only: an `int` through the boundary conversions (`pyobj.rs`) and back."""

    return _core.int_round_trip(value)


def number_op_table() -> tuple[tuple[int, str], ...]:
    """`(opcode, name)` of every number operation the extension knows."""

    return tuple(_core.number_op_table())


def run_number_ops(ops, *, strict: bool = True, memo: bool = True) -> list:
    """Test-only differential entry: a whole script of number operations in ONE crossing.

    `ops` is a sequence of `(operation name, arguments)`; the result is one decoded value per operation, a
    `codec.NativeError` where the operation failed with a named error (`OverflowError`, `ZeroDivisionError`,
    `ValueError`). `strict` makes the native decoder check that every rational is in lowest terms; `memo=False`
    turns the radicand-product memory off. A buffer the extension refuses is a `ValueError`.
    """

    return codec.decode_response(_core.run_number_ops(codec.encode_request(ops, strict=strict, memo=memo)))


def new_mirror(slots: str | None = None) -> cost.CostMirror:
    """A new native session with its own mirror (empty): for tests and for callers that isolate state.

    `slots` (`SLOT_MODES`): the slot mode of THIS session alone (tests); `None` takes the process-wide mode (`slot_mode()`). `raw` is refused by name, at the first call that binds the
    kernel classes, when the probe does not confirm a layout.
    """

    return cost.CostMirror(_core.Session(slots))


def default_mirror() -> cost.CostMirror:
    """The process-wide mirror: one native session per process, matching the process-wide Python tables."""

    with cost.NATIVE_LOCK:
        if not _DEFAULT:
            _DEFAULT.append(new_mirror())
        return _DEFAULT[0]


def coverage_at(partition, alpha, work_budget=None, store=None, traces=None):
    """`wavefront.coverage._coverage_at(partition, alpha, work_budget, store)`, native and whole (see `CostMirror.coverage_at`).

    A refusal of the port (`NATIVE_REFUSALS`) leaves every Python-visible state exactly as before the call; an exception of the oracle leaves its partial effects.
    """

    return default_mirror().coverage_at(partition, alpha, work_budget, store, traces)


def clip_geometry(plane, budget, *, points, cycles, polygons, law, seam, fans, flows, by_faces, inert=frozenset()):
    """`materialize.clip.clip_geometry(plane, budget, ...)`, native and whole (see `CostMirror.clip_geometry`).

    A refusal of the port (`NATIVE_REFUSALS`) leaves every Python-visible state exactly as before the call; an exception of the oracle leaves its partial effects.
    """

    return default_mirror().clip_geometry(plane, budget, points=points, cycles=cycles, polygons=polygons, law=law, seam=seam, fans=fans, flows=flows, by_faces=by_faces, inert=inert)


def build_skeleton(polygon, *, split_search=None, work_budget=None, dense_hydration=False):
    """`wavefront.skeleton.build_skeleton(polygon, *, split_search, work_budget, dense_hydration)`, native and whole (see `CostMirror.build_skeleton` and `skeleton_op.py`).

    `split_search=None` is the oracle's default (the motorcycle search). A refusal of the port (`NATIVE_REFUSALS`) leaves every Python-visible state exactly as before the call, so the caller
    may run the oracle on the same budget and memory tables; an exception of the oracle leaves its partial effects, and is raised with the oracle's text.
    """

    return default_mirror().build_skeleton(polygon, work_budget=work_budget, split_search=split_search, dense_hydration=dense_hydration)


def snap_embedding_certificate(before, after, faces, intended_corners, unclassifiable_corners, snapping_law):
    """`_embedding._compute_source_snap_embedding_certificate(before, after, faces, intended_corners, unclassifiable_corners, snapping_law)`, native and whole (see `embedding_op.py`).

    Returns the oracle's `SourceSnapEmbeddingCertificateV1`; raises the oracle's `ValueError` (inconsistent physical edge) with its text. A refusal of the port (`NATIVE_REFUSALS`: a stale pin, an input
    the port does not carry) changes nothing anywhere, so the caller runs the oracle on the same arguments.
    """

    return embedding_op.snap_embedding_certificate(_core.snap_embedding, before, after, faces, intended_corners, unclassifiable_corners, snapping_law)


def snapshot_check(snapshot, scope=snapshot_check_op.SCOPE_ANALYSIS_AND_REFERENCE):
    """`SnapshotCheckV1` когда валидаторы Python области `scope` ничего бы не нашли в `snapshot`, иначе `NativePortUnsupported` с причиной (см. `snapshot_check_op.py`).

    Отказ порта (`NATIVE_REFUSALS`: устаревший пин, снапшот, которого порт не несёт) ничего не меняет; вызывающий гонит валидаторы Python на том же снапшоте.
    """

    return snapshot_check_op.snapshot_check(_core, snapshot, scope)


def warm_snapshot_check() -> bool:
    """Binds the `snapshot_check` reader to the live kernel ahead of the first domain (see `snapshot_check_op.warm`); `True` when bound."""

    return snapshot_check_op.warm(_core)


def snapshot_check_raw_layouts() -> dict:
    """`{class name: True when the raw slot reader confirmed that class's layout}` of the `snapshot_check` reader (all True under `auto`/`raw`, all False under `attr`; see `CFTUV_NATIVE_SLOTS`)."""

    return snapshot_check_op.raw_layouts(_core)


def last_snapshot_check_timings() -> tuple:
    """Nanoseconds of the last `snapshot_check`: `(read the graph, check the model)`."""

    return snapshot_check_op.last_timings()


def frame_metric(*, source_vertices, source_faces, owner_patch_id, source_revision, patch_domain_id, source_lineage, grid_policy, grid_scale_law=None):
    """`RationalAffinePlanarMetricV2` владельца, как его построил бы `planar_metric.build_rational_affine_planar_metric` для патча, точно плоского после привязки источника, иначе `NativePortUnsupported` с причиной (см. `frame_op.py`).

    Отказ порта (`NATIVE_REFUSALS`: устаревший пин, патч, которого порт не несёт) ничего не меняет; вызывающий гонит построитель Python на тех же аргументах.
    """

    return frame_op.frame_metric(
        _core,
        source_vertices=source_vertices,
        source_faces=source_faces,
        owner_patch_id=owner_patch_id,
        source_revision=source_revision,
        patch_domain_id=patch_domain_id,
        source_lineage=source_lineage,
        grid_policy=grid_policy,
        grid_scale_law=grid_scale_law,
    )


def warm_frame() -> bool:
    """Binds the `frame` reader/builder to the live kernel ahead of the first domain (see `frame_op.warm`); `True` when bound."""

    return frame_op.warm(_core)


def frame_raw_layouts() -> dict:
    """`{class name: True when the raw slot access confirmed that class's layout}` of the `frame` reader/builder (all True under `auto`/`raw`, all False under `attr`; see `CFTUV_NATIVE_SLOTS`)."""

    return frame_op.raw_layouts(_core)


def last_frame_timings() -> tuple:
    """Nanoseconds of the last `frame_metric`: `(read the graph, compute, build the metric)`."""

    return frame_op.last_timings()


def effective_alpha_contacts(source, boundary, frame):
    """`(prefilter rejected, prefilter passed, contacts)` пары, как их даёт `reference.boundary._native_contacts` (см. `effective_alpha_op.py`).

    `contacts` — кортеж `(alpha, station, point)` эталона. Отказ порта (`NATIVE_REFUSALS`: устаревший пин, пара, которую порт не несёт) ничего не меняет, вызывающий считает эталон.
    """

    return effective_alpha_op.effective_alpha_contacts(_core, source, boundary, frame)


def exact_mesh(frame_faces, region_ids, layout, table, notes, chord_names, lattice_alpha, budget, chart_cw, law, exact_plane, lap=None):
    """`materialize.mesh.exact_mesh(...)`, native and whole (see `CostMirror.exact_mesh` and `materialize_w1_op.py`).

    A refusal of the port (`NATIVE_REFUSALS`) leaves every Python-visible state exactly as before the call, so the caller may run the oracle on the same budget, notes and memory tables; an
    exception of the oracle leaves its partial effects, and is raised with the oracle's text.
    """

    return default_mirror().exact_mesh(frame_faces, region_ids, layout, table, notes, chord_names, lattice_alpha, budget, chart_cw, law, exact_plane, lap)


def apply_silhouette(source, budget):
    """`materialize.silhouette.apply_silhouette(source, budget)`, native and whole (see `materialize_hot_op.py`): a refusal of the port leaves every Python-visible state as before the call."""

    return materialize_hot_op.apply_silhouette(default_mirror(), source, budget)


def lift_vertices(points, plane):
    """`materialize.assemble.lift_vertices(points, plane)` on a `PlaneLiftV1`, native (see `materialize_hot_op.py`)."""

    return materialize_hot_op.lift_vertices(default_mirror(), points, plane)


def source_lift(plane, positions, triangles, host_positions, step, polygons, points, budget, reverse):
    """`materialize.domain._sourced(...)` (the law `SOURCE_VERTEX_LIFTED_AT_HOST_POSITION_V1` and the check of the emitted faces), native and whole (see `materialize_hot_op.py`)."""

    return materialize_hot_op.source_lift(default_mirror(), plane, positions, triangles, host_positions, step, polygons, points, budget, reverse)


def warm_materialize_hot() -> bool:
    """Binds the native SILHOUETTE operations to the live kernel ahead of the first domain (`materialize_hot_op.warm`)."""

    return materialize_hot_op.warm(default_mirror())


def canonical_triangles(frame_faces, cycles, budget, reverse):
    """`materialize.mesh.canonical_triangles(frame_faces, cycles, budget, reverse)`, native and whole (see `materialize_w1_op.py`)."""

    return default_mirror().canonical_triangles(frame_faces, cycles, budget, reverse)


def last_w1_timings() -> tuple:
    """Nanoseconds of the last W1 call of the default mirror: `(sync in, native call, post, total, tables, arguments, compute, result, memory log)`."""

    return default_mirror().last_w1_timings


def radical_v2_eval(program, *, passes: int = 0, inner: int = 1) -> tuple:
    """`(outcomes, run_ns, timings)` of a PROGRAM over `RadicalSumV2` (`native/cftuv-radical`, the mirror of `reference/radical_v2.py`): the test and bench entry of the capability `radical_v2`.

    `program` is a sequence of instruction tuples and `outcomes` one outcome tuple per instruction, both defined in `tools/native_radical_v2_corpus.py` (and in `cftuv_radical::program`). `run_ns` is the
    time the extension took to RUN the program (decoding and encoding excluded); with `passes > 0` `timings` holds the nanoseconds of each instruction (the best of `passes` passes, each instruction
    executed `inner` times in a row), else `None`. A defect of the program is a `ValueError`; a stale pin (`reference/radical_v2.py` moved past the mirror) is the named `NativePortStale`.
    """

    pin.require("radical_v2")
    return _core.radical_v2_eval(program, passes, inner)


def last_skeleton_timings() -> tuple:
    """Nanoseconds of the last `build_skeleton` of the default mirror: `(sync in, native call, post, total, arguments, compute, result, memory log)`."""

    return default_mirror().last_skeleton_timings


def skeleton_oracle_statuses() -> tuple:
    """Test-only: the status codes of `build_skeleton` the extension treats as outcomes of the oracle (`skeleton.rs`); `skeleton_op.SKELETON_STATUSES` is the same table."""

    return tuple(_core.skeleton_oracle_statuses())


def last_clip_timings() -> tuple:
    """Nanoseconds of the last `clip_geometry` of the default mirror: `(sync in, native call, post, total, plane, arguments, compute, result)`."""

    return default_mirror().last_clip_timings


def last_coverage_timings() -> tuple:
    """Nanoseconds of the last `coverage_at` of the default mirror: `(sync in, native call, post, total, prepare, arguments, compute, result, memory log)`."""

    return default_mirror().last_timings


def sign(value, *, filter_bits: int = 64, budget=None) -> int:
    """`SqrtSumV1.sign(filter_bits=..., budget=...)`, native; budget, counters and memory updated like Python's."""

    return default_mirror().sign(value, filter_bits=filter_bits, budget=budget)


def divided_by(numerator, denominator, budget=None):
    """`numerator.divided_by(denominator, budget)`, native."""

    return default_mirror().divided_by(numerator, denominator, budget)


def divide_with_prime_universe(numerator, denominator, prime_universe, budget=None):
    """`_divide_with_prime_universe(numerator, denominator, prime_universe, budget)`, native."""

    return default_mirror().divide_with_prime_universe(numerator, denominator, prime_universe, budget)


def radical(coefficient, radicand, budget=None):
    """`SqrtSumV1.radical(coefficient, radicand, budget)`, native."""

    return default_mirror().radical(coefficient, radicand, budget)


def radical_sum(parts, budget=None):
    """`radical_sum(parts, budget)`, native."""

    return default_mirror().radical_sum(parts, budget)


def squarefree_split(n: int, budget=None) -> tuple:
    """`squarefree_split(n, budget)`, native."""

    return default_mirror().squarefree_split(n, budget)


def prime_support(radicand: int, budget=None) -> tuple:
    """`prime_support(radicand, budget)`, native."""

    return default_mirror().prime_support(radicand, budget)


def prime_universe_remembered(q_values, budget=None, store=None) -> tuple:
    """`prime_universe_remembered(q_values, budget, store)` with the default `build`, native."""

    return default_mirror().prime_universe_remembered(q_values, budget, store)


def new_clip_seam_session():
    """Test-only: a fresh native session for the clip differential seams (`cftuv_native.clip_seams`)."""

    return _core.Session()


def clip_seam_run(session, request: bytes) -> bytes:
    """Test-only: ONE clip seam (`cftuv-clip/src/seam.rs`) on `session`; the answer buffer is decoded by `clip_seams`."""

    return _core.clip_seam_run(session, request)


def clip_seam_table() -> tuple[tuple[int, str], ...]:
    """Test-only: `(opcode, name)` of every clip seam the extension knows."""

    return tuple(_core.clip_seam_table())


def new_skeleton_seam_session():
    """Test-only: a fresh native session for the skeleton differential seams (`cftuv_native.skeleton_seams`)."""

    return _core.Session()


def skeleton_seam_run(session, request: bytes) -> bytes:
    """Test-only: ONE skeleton seam (`cftuv-skeleton/src/seam.rs`) on `session`; the answer buffer is decoded by `skeleton_seams`."""

    return _core.skeleton_seam_run(session, request)


def skeleton_seam_table() -> tuple[tuple[int, str], ...]:
    """Test-only: `(opcode, name)` of every skeleton seam the extension knows."""

    return tuple(_core.skeleton_seam_table())


def angular_compute(*, binding_inputs, dependencies, raw, symbolic_mode, before_assemble):
    """Whole ANGULAR operation; unavailable until approved profile pins exist."""
    return angular_op.angular_compute(_core,binding_inputs=binding_inputs,dependencies=dependencies,
                                      raw=raw,symbolic_mode=symbolic_mode,before_assemble=before_assemble)


def angular_patch(patch):
    """Test-only: the raw single-patch ANGULAR evaluator on an encoded wire dict (`angular_wire.encode` shape): `(0, plans)` or `(2, tag, detail)`.

    No pins, no binding, no assembly, no effects: the whole operation is `angular_compute`, which declines while no profile is approved."""
    return _core.angular_patch(patch)


def warm_angular(*, binding_inputs, dependencies):
    """Warm checked bindings using caller-supplied actual host dependencies."""
    return angular_op.warm(_core,binding_inputs=binding_inputs,dependencies=dependencies)
