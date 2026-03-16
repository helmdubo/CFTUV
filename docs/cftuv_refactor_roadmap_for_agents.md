# CFTUV Refactor Roadmap For Agents
## Critical review, priorities, and safe execution order

---

## Purpose

Этот документ нужен для AI агентов и будущих рефакторингов.

Он не заменяет `cftuv_architecture_v2.0.md`.
Наоборот: архитектурный документ остаётся главным источником истины,
а этот файл описывает:

- какие structural problems уже видны в коде;
- какие из них реально опасны;
- в каком порядке их исправлять;
- что нельзя ломать по пути.

Главная цель: не делать большой "красивый" рефакторинг вслепую.
Нужно идти малыми фазами, сохраняя chain-first strongest-frontier.

---

## Non-Negotiable Invariants

Любой рефакторинг обязан сохранять:

1. Scaffold остаётся `chain-first strongest-frontier`, не patch-first.
2. `HOLE` loop не должен участвовать в scaffold placement pool.
3. Quilt solve connectivity и topology connectivity — не одно и то же.
4. `analysis.py` строит topology IR, `solve.py` не пересчитывает topology.
5. `PatchGraph` остаётся центральным topology IR.
6. Debug visualization обязана продолжать работать на каждой фазе.

---

## Current Structural Findings

### F1. Solve policy размазана по коду

Сейчас solve rules живут не в одном слое, а во множестве локальных условий:

- `OUTER`-only filtering;
- same-type propagation;
- tree-edge-only cross-patch sewing;
- conformal-only fallback;
- seed restrictions;
- anchor restrictions.

Это уже дважды привело к bug drift:

- `HOLE` patches попадали в planning, но не были достижимы frontier;
- ring/cylinder quilts замыкались в UV cycle, хотя должны были оставаться tree.

Риск:
любое новое правило solve снова придётся дублировать в нескольких местах.

### F2. В `analysis.py` есть риск рассинхрона между raw chains и final chains

Сейчас pipeline выглядит так:

1. построить `raw_chains`;
2. создать `BoundaryChain`;
3. merge border chains;
4. строить corners.

Но corners частично опираются на `raw_chains`, а не только на финальную chain topology.

Риск:
после merge corner metadata может уже не соответствовать финальному loop layout.
Это не обязательно ломает каждый меш, но создаёт скрытый источник нестабильности.

### F3. Frontier state хранится как набор параллельных registry

Сейчас frontier builder использует отдельно:

- `point_registry`;
- `vert_to_placements`;
- `placed_chain_refs`;
- `placed_chains_map`;
- `chain_dependency_patches`;
- `build_order`.

Риск:
это state synchronization problem.
Чем больше special cases, тем выше шанс забыть обновить одну из структур.

### F4. Solve IR содержит поля, которые выглядят как контракт, но фактически полумёртвые

В `ScaffoldPatchPlacement` есть поля, которые либо не заполняются полностью,
либо выглядят как future fields:

- `root_chain_index`;
- `max_chain_gap`;
- `gap_reports`;
- `pinned`;
- `origin_offset`.

Риск:
агент или разработчик начинает верить этому IR больше, чем код реально гарантирует.

### F5. Слишком много stringly-typed и sentinel-based semantics

Примеры:

- `neighbor_patch_id = -1 / -2`;
- `source_kind: str`;
- `status: str`;
- tuple fields без строгой формы.

Риск:
контракты слабые, IDE и статический анализ почти не помогают,
ошибки всплывают поздно.

### F6. Есть две разные связности, но API это не подчёркивает

Существуют:

- topology connectivity (`PatchGraph.connected_components()`);
- solve connectivity (`solve_components`, attachment-derived).

Обе нужны.
Проблема не в наличии двух моделей, а в том, что их слишком легко перепутать.

---

## Refactor Strategy

Не делать монолитный rewrite.

Правильный порядок:

1. Зафиксировать regression cases.
2. Стабилизировать topology consistency в `analysis.py`.
3. Вынести solve policy в один явный слой.
4. Упаковать frontier runtime state.
5. Потом чистить типы и IR.

Этот порядок важен.
Если сначала "чистить классы", а solve rules всё ещё размазаны по коду,
вы просто получите красиво оформленный источник новых багов.

---

## Phase 0. Regression Harness

### Goal

Создать минимальный набор проверок, чтобы безопасно делать следующие фазы.

### Tasks

1. Собрать эталонные production-like meshes:
   - simple wall;
   - wall with hole-attached patches;
   - floor caps;
   - closed ring house;
   - mixed wall/floor;
   - bevel-heavy wall.

2. Для каждого кейса фиксировать:
   - patch count;
   - quilt count;
   - unsupported patch ids;
   - invalid closure count;
   - final conformal fallback count;
   - заметные frontier logs.

3. Сделать один markdown checklist "expected behavior per mesh".

### Exit Criteria

- После любой refactor phase можно быстро понять, что сломалось.
- Проверка выполняется руками, без сложной тестовой инфраструктуры.

### Important

Не пытаться сначала построить полноценный unit-test framework для Blender.
Сейчас gold logs и manual verification полезнее.

---

## Phase 1. Topology Consistency In Analysis

### Goal

Убрать скрытый drift между финальными chains и corner topology.

### Tasks

1. Проследить весь pipeline `_build_boundary_loops()`.
2. Определить, где `raw_chain` metadata ещё используется после chain merge.
3. Перестроить corner generation так, чтобы оно зависело от финального chain representation.
4. Проверить:
   - `start_corner_index`;
   - `end_corner_index`;
   - wrap-around merge;
   - corner turn angles;
   - corner vertex identity.

### Recommended Rule

После того как loop перешёл в финальный `BoundaryLoop.chains`,
вся endpoint/corner topology должна вычисляться только из этого финального представления.

### Exit Criteria

- chain count, corner count, endpoint links и debug overlay согласованы;
- bevel-heavy meshes не дают странных corner jumps;
- no new regressions в Phase 0 harness.

---

## Phase 2. Central Solve Policy Layer

### Goal

Убрать архитектурно опасное дублирование solve rules.

### Problem

Сейчас правила solve разбросаны по planning, frontier scoring, anchor search,
seed selection, report formatting и conformal fallback.

### Required Result

Появляется один solve-time layer, например:

- `SolveView`;
- или `SolvePolicy`;
- или `SolverTopologyView`.

Название не критично.
Критично, чтобы он был единым местом для правил solve-допуска.

### This Layer Must Answer

1. Какие loops solve-visible?
2. Какие chains solve-visible?
3. Разрешён ли cross-patch propagation для пары patches?
4. Является ли seam tree-edge quilt или cut-edge?
5. Какие patch types совместимы для solve?

### Tasks

1. Собрать текущие правила из `solve.py`.
2. Перенести фильтрацию туда.
3. Переподключить:
   - attachment candidate generation;
   - seed selection;
   - anchor search;
   - chain scoring;
   - frontier pool assembly.

### Exit Criteria

- solve rules живут в одном слое;
- добавление нового solve rule не требует правок в 5+ местах;
- `HOLE` case и ring case продолжают работать.

---

## Phase 3. Frontier Runtime State

### Goal

Упростить и стабилизировать runtime state frontier builder.

### Problem

Сейчас frontier loop управляет несколькими взаимозависимыми registry вручную.

### Required Result

Сделать один explicit runtime container, например `FrontierState`.

### Suggested Responsibilities

- register placed chain;
- resolve anchor candidates;
- maintain point/vertex lookup;
- count placed chains per patch;
- expose dependency patches;
- expose placed refs and build order.

### Tasks

1. Создать dataclass внутри `solve.py`.
2. Перенести туда текущие registry.
3. Убрать ручную синхронизацию в главной frontier loop.
4. Сократить повторяющиеся вычисления внутри hot path.

### Exit Criteria

- `build_quilt_scaffold_chain_frontier()` заметно проще читать;
- special cases меняются в одном месте;
- runtime behavior совпадает с предыдущим.

---

## Phase 4. IR Cleanup

### Goal

Сделать runtime contracts честными.

### Problem

IR сейчас содержит смесь:

- реально используемых полей;
- transitional fields;
- sleeping fields;
- loosely typed fields.

### Tasks

1. Проверить каждое поле `ScaffoldPatchPlacement`.
2. Разделить их на группы:
   - active runtime fields;
   - planned-but-not-active fields;
   - dead fields.
3. Для active fields:
   - обеспечить полное заполнение.
4. Для planned/dead:
   - удалить из runtime IR;
   - или явно пометить как Phase 5 only.
5. По возможности заменить строковые статусы и source kinds на enum-like модели.

### Exit Criteria

- IR содержит только то, что runtime реально гарантирует;
- reports не читают фиктивные или неинициализированные данные.

---

## Phase 5. Typed Raw Analysis Payloads

### Goal

Уменьшить количество `dict.get(...)` в topology builder.

### Why This Is Late

Это не первая по важности фаза.
Она улучшает безопасность изменений, но не закрывает самые опасные текущие риски.

### Tasks

1. Внутри `analysis.py` ввести private dataclasses для raw payload:
   - raw loop;
   - raw chain.
2. Не переносить это в `model.py`, если нет отдельного решения.
3. Минимизировать string-key contracts внутри analysis pipeline.

### Exit Criteria

- код `analysis.py` проще читать;
- меньше неявных полей и `dict`-протоколов;
- corner/split/merge logic проще сопровождать.

---

## What Not To Do

### 1. Не делать большой split `solve.py` только ради длины файла

Если policy и state останутся неявными, дробление файла ничего не даст.

### 2. Не делать крупную миграцию ownership между `model.py` и `solve.py` в рамках локального багфикса

Сейчас persistent topology/solve IR уже живёт в `model.py`,
а planning/runtime helpers остаются в `solve.py`.
Не надо смешивать локальный bugfix с новой массовой перекладкой структур между модулями.

### 3. Не строить новый graph layer без необходимости

Если `SolveView` покрывает solve policy, отдельный runtime graph может быть лишним.

### 4. Не трогать scoring weights одновременно с structural refactor

Сначала стабилизировать architecture.
Потом уже настраивать scoring.

### 5. Не ломать debug path

Если debug visualization перестала объяснять scaffold state,
рефакторинг считается неудачным.

---

## Recommended Execution Style For Agents

Для любой фазы:

1. Сначала прочитать `docs/cftuv_architecture_v2.0.md`.
2. Затем прочитать этот roadmap.
3. Выбрать только одну фазу.
4. Не смешивать structural cleanup и scoring tuning в одном change set.
5. Перед правками определить:
   - какие meshes проверяются;
   - какие логи считаются ожидаемыми;
   - какие invariants нельзя нарушать.

После правок агент обязан сообщить:

- какую фазу он выполнял;
- какие функции трогал;
- какие regression cases проверил;
- какие риски остались.

---

## Immediate Recommendation

Если начинать следующую архитектурную работу прямо сейчас, порядок такой:

1. Phase 0
2. Phase 1
3. Phase 2

Только после этого:

4. Phase 3
5. Phase 4
6. Phase 5

Причина:

- Phase 1 убирает скрытую topology inconsistency;
- Phase 2 убирает главный источник solve drift;
- Phase 3 становится намного проще после Phase 2;
- Phase 4 и 5 уже не исправляют корневые баги, а улучшают maintainability.

---

## Short Summary

Главная архитектурная проблема CFTUV сейчас не в том, что "мало абстракций".

Главная проблема в другом:

- solve policy не материализована в один слой;
- frontier runtime state раздроблен;
- topology representation местами может расходиться после merge;
- runtime IR частично честный, частично transitional.

Поэтому правильный путь не "сделать больше классов",
а последовательно:

1. зафиксировать regression harness;
2. стабилизировать topology consistency;
3. централизовать solve policy;
4. потом чистить state и типы.

---
