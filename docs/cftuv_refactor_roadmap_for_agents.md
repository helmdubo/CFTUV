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
3. Quilt solve connectivity и topology connectivity - не одно и то же.
4. `analysis.py` строит topology IR, `solve.py` не пересчитывает topology.
5. `PatchGraph` остаётся центральным topology IR.
6. Debug visualization обязана продолжать работать на каждой фазе.
7. Временный UV-dependent шаг в `analysis.py` нельзя размазывать по другим частям analysis.

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

### F2. В `analysis.py` нет формализованного контракта для raw payload

Промежуточный pipeline до построения финальных `BoundaryChain` всё ещё живёт на
`raw_loop` / `raw_chain` dict payload.

Риск:

- поля появляются и исчезают по шагам неявно;
- `dict.get(...)` скрывает ошибки контракта;
- безопасно менять split/merge pipeline трудно.

Это не просто stylistic issue.
Без typed intermediate payload любой следующий refactor topology builder остаётся partly blind.

### F3. В `analysis.py` есть риск рассинхрона между raw chains и final chains

Сейчас pipeline логически выглядит так:

1. построить raw chains;
2. построить `BoundaryChain`;
3. merge border chains;
4. построить corners.

Но corner generation ещё частично опирается на `raw_chains`, а не только на финальную chain topology.

Риск:
после merge corner metadata может уже не соответствовать финальному loop layout.
Это не обязательно ломает каждый меш, но создаёт скрытый источник нестабильности.

### F4. `analysis.py` не полностью чистый: `OUTER/HOLE` classification идёт через UV side effects

`_classify_loops_outer_hole()` использует временный UV unwrap через Blender operator.
Это единственный явно допущенный side effect внутри analysis pipeline.

Риск:

- debug path временно мутирует UV layer и selection state;
- semantics "analysis только читает topology" нарушается;
- шаг плохо изолирован и осложняет дальнейший refactor analysis.

Это не значит, что функцию надо срочно выкинуть любой ценой.
Но её side effects должны быть локализованы, документированы и не должны расползаться дальше.

### F5. Scaffold result не сериализуется и не кэшируется в удобном для регрессии виде

Сейчас после правок сложно ответить на вопрос "что именно изменилось в scaffold".
Сравнение идёт в основном через консоль и ручной осмотр.

Риск:

- regression harness остаётся ручным;
- сложно сравнить scaffold до/после рефактора;
- debug iteration cycle медленнее и менее надёжен, чем должен быть.

Важно:
речь не обязательно о тяжёлом persistent cache сразу.
Для начала нужен хотя бы стабильный serializable scaffold snapshot/report.

### F6. Frontier state хранится как набор параллельных registry

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

### F7. Solve IR содержит поля и типы, которые выглядят как контракт, но частично transitional

Проблемы этого класса:

- `ScaffoldPatchPlacement` содержит поля, которые либо не заполняются полностью, либо выглядят как future fields;
- есть stringly-typed runtime semantics (`source_kind`, `status`);
- остаются sentinel-based соглашения (`neighbor_patch_id = -1 / -2`).

Риск:
агент или разработчик начинает верить этому IR больше, чем код реально гарантирует.

### F8. Есть две разные связности, но API это не подчёркивает достаточно жёстко

Существуют:

- topology connectivity (`PatchGraph.connected_components()`);
- solve connectivity (`solve_components`, attachment-derived).

Обе нужны.
Проблема не в наличии двух моделей, а в том, что их слишком легко перепутать.

---

## Refactor Strategy

Не делать монолитный rewrite.

Правильный порядок:

1. Зафиксировать regression cases и snapshot baseline.
2. Формализовать raw payload в `analysis.py`.
3. Стабилизировать topology consistency в `analysis.py`.
4. Изолировать analysis side effects там, где это ещё нужно.
5. Вынести solve policy в один явный слой.
6. Упаковать frontier runtime state.
7. Потом чистить типы и IR.

Этот порядок важен.
Если сначала "чистить классы", а solve rules всё ещё размазаны по коду,
вы просто получите красиво оформленный источник новых багов.

---

## Phase 0. Regression Harness And Snapshot Baseline

### Goal

Создать минимальный набор проверок и сравнений, чтобы безопасно делать следующие фазы.

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

4. Определить стабильный scaffold snapshot/report format, который можно сравнивать до/после:
   - quilt ids;
   - patch membership;
   - unsupported patches;
   - build order;
   - closure status;
   - pinned/unpinned summary.

### Exit Criteria

- После любой refactor phase можно быстро понять, что сломалось.
- Есть хотя бы один serializable snapshot baseline, а не только ручной просмотр консоли.
- Проверка выполняется руками, без сложной тестовой инфраструктуры.

### Important

Не пытаться сначала построить полноценный unit-test framework для Blender.
Сейчас gold logs, scaffold snapshots и manual verification полезнее.

---

## Phase 1A. Typed Raw Analysis Payload

### Goal

Формализовать промежуточный payload в `analysis.py`, прежде чем чинить topology drift.

### Tasks

1. Внутри `analysis.py` ввести private dataclasses для raw payload:
   - raw loop;
   - raw chain.

2. Не переносить это в `model.py`, если нет отдельного решения.

3. Минимизировать string-key contracts внутри analysis pipeline.

4. Явно зафиксировать, какие поля обязательны после каждого шага:
   - split by neighbor;
   - bevel merge;
   - geometric split;
   - border corner split.

### Exit Criteria

- Код `analysis.py` проще читать.
- Меньше неявных полей и `dict`-протоколов.
- Следующая фаза может чинить corner/split/merge logic не вслепую.

---

## Phase 1B. Topology Consistency In Analysis

### Goal

Убрать скрытый drift между финальными chains и corner topology.

### Tasks

1. Проследить весь pipeline `_build_boundary_loops()`.
2. Определить, где raw metadata ещё используется после chain merge.
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

## Phase 1C. Analysis Purity And UV-Dependent Classification Boundary

### Goal

Сделать UV-dependent шаг в analysis явным, ограниченным и безопасным для debug/runtime paths.

### Tasks

1. Явно задокументировать, что `_classify_loops_outer_hole()` - единственный допустимый side-effect inside analysis.
2. Проверить и упростить rollback contract:
   - temporary UV layer;
   - active UV layer;
   - selection state.

3. Изолировать этот шаг так, чтобы остальной analysis pipeline не зависел от hidden mutable state.

4. Решить, нужен ли на этом этапе:
   - только better isolation;
   - или полноценная замена UV-unwrap classification на другой метод.

### Exit Criteria

- Side effects analysis локализованы и явно описаны.
- Debug path не получает неожиданных persistent изменений состояния меша при обычном выполнении.
- Дальнейшие refactor phases не распространяют UV side effects на новые части analysis.

---

## Phase 2. Central Solve Policy Layer

### Goal

Убрать архитектурно опасное дублирование solve rules.

### Problem

Сейчас правила solve разбросаны по planning, frontier scoring, anchor search,
seed selection, report formatting и conformal fallback.

### Architectural Decision Required Before Implementation

До начала этой фазы нужно явно решить, как делится solve policy:

1. `Stateless` часть:
   правила, которые зависят только от `PatchGraph`, `SolvePlan`, quilt membership и config.

2. `Stateful` часть:
   правила, которые зависят от текущего frontier progress:
   placed chains, available anchors, closure guards, build order.

Текущий код смешивает эти категории.

Рекомендуемое направление:

- сделать явный stateless solve-time view для topology/visibility/filtering;
- не притворяться, что anchor/seed/closure rules тоже stateless;
- stateful runtime rules оставить в явном frontier/runtime policy layer.

Иными словами:
не пытаться запихнуть весь solve behavior в один "как бы stateless" объект.

### Required Result

Появляется solve-time policy/view слой, например:

- `SolveView`;
- или `SolvePolicy`;
- или связка `SolveView` + `FrontierRuntimePolicy`.

Название не критично.
Критично, чтобы правила solve-допуска больше не были набором независимых `if` по коду.

### This Layer Must Answer

#### Stateless questions

1. Какие loops solve-visible?
2. Какие chains solve-visible?
3. Какие patch types совместимы для solve?
4. Разрешён ли cross-patch relation для пары patches?
5. Является ли seam tree-edge quilt или cut-edge?

#### Stateful questions

1. Есть ли у chain допустимые anchors в текущем frontier state?
2. Может ли chain быть seed в текущем quilt state?
3. Разрешено ли placement/closure с текущим набором placed chains?

### Tasks

1. Собрать текущие правила из `solve.py` и разделить их на stateless и stateful.
2. Перенести stateless фильтрацию в один явный слой.
3. Явно описать, какие runtime checks остаются stateful.
4. Переподключить:
   - attachment candidate generation;
   - seed selection;
   - anchor search;
   - chain scoring;
   - frontier pool assembly.

### Exit Criteria

- solve rules больше не размазаны по случайным локальным `if`;
- добавление нового solve rule не требует правок в 5+ местах;
- distinction between stateless visibility and stateful runtime gating явно выражен в коде;
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
- runtime behavior совпадает с предыдущим;
- Phase 2 policy split при этом не размывается обратно.

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
   - или явно пометить как future-only.

5. По возможности заменить строковые статусы и source kinds на enum-like модели.

6. Явно задокументировать место sentinel semantics и при возможности сузить их зону.

### Exit Criteria

- IR содержит только то, что runtime реально гарантирует;
- reports не читают фиктивные или неинициализированные данные;
- новый агент не ошибается в ожиданиях от solve IR.

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

### 4. Не пытаться насильно объявить stateful frontier rules "stateless policy"

Это просто снова спрячeт архитектурный долг под новую абстракцию.

### 5. Не трогать scoring weights одновременно с structural refactor

Сначала стабилизировать architecture.
Потом уже настраивать scoring.

### 6. Не ломать debug path

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
2. Phase 1A
3. Phase 1B
4. Phase 1C
5. Phase 2

Только после этого:

6. Phase 3
7. Phase 4

Причина:

- Phase 0 даёт baseline и сравнимый scaffold snapshot;
- Phase 1A убирает blind editing в analysis pipeline;
- Phase 1B закрывает topology inconsistency;
- Phase 1C локализует side effects analysis;
- Phase 2 убирает главный источник solve drift;
- Phase 3 становится намного проще после Phase 2;
- Phase 4 уже не исправляет корневые баги, а улучшает maintainability.

---

## Short Summary

Главная архитектурная проблема CFTUV сейчас не в том, что "мало абстракций".

Главная проблема в другом:

- solve policy не материализована в один слой;
- analysis pipeline до сих пор partly untyped;
- topology representation местами может расходиться после merge;
- analysis содержит локальный UV side-effect step;
- scaffold трудно сравнивать между ревизиями;
- frontier runtime state раздроблен;
- runtime IR частично честный, частично transitional.

Поэтому правильный путь не "сделать больше классов",
а последовательно:

1. зафиксировать regression harness и snapshot baseline;
2. формализовать raw analysis payload;
3. стабилизировать topology consistency;
4. локализовать analysis side effects;
5. централизовать solve policy;
6. потом чистить state и типы.

---
