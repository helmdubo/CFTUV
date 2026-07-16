# CFTUV Decal Runtime + Intrinsic Surface Program
## Самодостаточный план исполнения для Codex

**Репозиторий:** `helmdubo/CFTUV`  
**Исходная ветка:** `claude/blender-decal-corner-preview-yq4lir`  
**Цель программы:** довести Patch Voronoi decal backend до производственного realtime-контракта, затем расширить его на developable/криволинейные owner surfaces без переписывания 2D Voronoi/corner-ядра; принимать решение о C++ и GPU только по измеренному профилю.

Этот документ предназначен для прямой передачи исполнителю в Codex. Он не требует истории обсуждения.

Канонический экземпляр документа лежит в самой рабочей ветке:
`docs/CFTUV_decal_runtime_intrinsic_codex_workplan.md`. При расхождении с
любой внешней копией приоритет у версии в ветке. Ветка
`codex/decal-patch-voronoi` кодом идентична исходной точке этой ветки
(`100bba8`), но плана не содержит и канонической не является.

---

# 0. Инструкция исполнителю перед началом

1. Переключиться на актуальный HEAD ветки `claude/blender-decal-corner-preview-yq4lir`.
2. Зафиксировать baseline:
   - `git rev-parse HEAD`;
   - `git status --short`;
   - версию Blender, Python и `pyvoronoi`;
   - результат полного тестового набора;
   - текущие benchmark-результаты на `walls.003` и `walls.001`.
3. Прочитать полностью:
   - `AGENTS.md`;
   - `docs/cftuv_decal_runtime.md`;
   - `docs/cftuv_decal_nonplanar_roadmap.md`;
   - `docs/cftuv_decals.md`;
   - `docs/architectural_debt.md` — с оговоркой: inline-маркеров
     `ARCHITECTURAL_DEBT` в `.py`-файлах сейчас НЕТ (проверено), а леджер
     ссылается на несуществующую `analysis_junctions.py:_derive_junction_disk_cycle`.
     Не искать маркеры в коде. Привести леджер в соответствие с фактическим
     кодом — отдельный небольшой docs-коммит в рамках Tranche A.
4. Не cherry-pick'ать сторонние экспериментальные ветки целиком. Использовать их только как дизайн-вход; каждое изменение должно соответствовать текущему HEAD.
5. Выполнять **по одной задаче** из этого плана. После каждой задачи:
   - тесты;
   - differential;
   - Blender fixture;
   - обновление docs при изменении контракта;
   - отдельный commit.
6. Не объединять runtime-correctness, intrinsic charts и native backend в один PR/commit.

Базовые команды:

```bash
python -m compileall -q cftuv tests
python -m pytest -q
git diff --check
```

Приёмка Patch Voronoi не считается валидной, если `tests/test_decal_voronoi.py` был пропущен из-за отсутствующего `pyvoronoi`.

---

# 1. Неизменяемые архитектурные инварианты

1. `pyvoronoi.Construct()` выполняется только на compile-этапе. Во время modal drag его вызовов быть не должно.
2. `CornerSpec` хранит только статические intrinsic-факты. Runtime policy не возвращается в compile plan.
3. `evaluate_patch_voronoi_plan()` остаётся единственным математическим evaluator'ом для preview и confirm; различаться может только adapter/materialization, но не geometry semantics.
4. Voronoi, corner classification, crop и arrangement работают в 2D chart space и не читают 3D surface topology напрямую.
5. Связь с mesh принадлежит `DecalSurfaceDomain` и chart/provenance-слою.
6. Никакого 3D nearest-surface поиска для intrinsic routing: геометрически близкие, но топологически далёкие поверхности не должны склеиваться.
7. Patch и Chain остаются первичными сущностями CFTUV. Corner/Junction не становятся solve entities основного UV pipeline.
8. Третьесторонние зависимости допускаются только внутри decal backend, документируются и имеют явный fallback.
9. Preview не имеет права уничтожать или мутировать production decal object до confirm.
10. Невалидный промежуточный кадр оставляет последний валидный preview и последние валидные настройки.
11. Любая потеря face/site/selected edge должна быть либо допустимым формально описанным случаем, либо явной ошибкой/diagnostic. Молчаливые потери запрещены.
12. Нельзя начинать C++ или GPU solve до прохождения соответствующего performance gate.

---

# 2. Текущее состояние, которое нужно сохранить

На исходной ветке уже существуют и не должны быть переписаны без необходимости:

- width/policy-independent `PatchVoronoiPlan`;
- runtime `CornerRuntimeSettings` и `classify_corner_runtime()`;
- тест изменения `Acute Split Angle` без повторной компиляции plan;
- общий compile lifetime для `invoke()`, `execute()` и headless;
- component-level hybrid routing Patch Voronoi / Legacy Network;
- strict runtime: успешно скомпилированный Patch Voronoi component не меняет backend во время drag;
- persistent preview mesh и topology-signature fast path;
- affine lift cache;
- keyed fragment merge;
- sweep broad phase для polygon simplicity;
- `DecalSurfaceDomain(kind="INTRINSIC")` с barycentric `lift/normal_at` как интерфейсный задел;
- piecewise-planar owner-surface fallback текущего поколения.

Текущие измеренные ориентиры из runtime docs нужно сохранить как baseline, а не как гарантированный результат после каждого изменения:

- `walls.003`, 27 edges: evaluator около 20–25 ms;
- `walls.001`, 133 edges: evaluator около 95–110 ms.

---

# 3. Модель поставки

Программа делится на независимые tranches:

- **A — Runtime correctness и первая live-corner версия.** Обязательна до новых UI controls.
- **B — Geometry infrastructure для intrinsic charts.** Diagram transform, индексы, shared types, provenance contract.
- **C — Developable strip charts.** Фаски, цилиндрические скругления, открытые трубы.
- **D — Periodic domains.** Замкнутые трубы/кольца.
- **E — Approximate non-developable research.** Только после metrics и explicit admission.
- **F — Performance/native/GPU.** Только по benchmark gate.

Нельзя начинать C до завершения A и B. Нельзя начинать D до принятого C. Нельзя начинать F-native до Python profiling после C или incremental A-performance.

---

# TRANCHE A — Runtime Corner Correctness + Live Preview

## A0. Baseline harness и reproducibility

### Цель
Сделать все последующие изменения измеримыми и differential-проверяемыми.

### Изменения

1. Добавить/нормализовать скрипт benchmark, например:
   - `artifacts/verify_decal_runtime.py`;
   - результат в JSON: compile time, evaluate time по фазам, materialization time, face/vert/edge counts, dropped count, policy counts, backend routing, счётчики тихих fallback'ов compile/runtime (см. A2 п. 7).
2. Сериализовать `_NetworkFace` детерминированно:
   - `surface_id`;
   - `component_kind/component_side`;
   - ordered `vert_keys`;
   - positions и UV с фиксированным округлением;
   - topology signature.
3. Зафиксировать baseline на:
   - `walls.003`;
   - `walls.001`;
   - widths `2.0 / 3.0 / 3.7076 / 4.5`;
   - preview и confirm.
4. Добавить счётчик вызовов `Pyvoronoi.Construct()` в integration harness.

### Acceptance

- JSON воспроизводим между двумя последовательными запусками.
- Preview/confirm differences документированы по backend'ам.
- Во время серии width drag число новых `Construct()` равно нулю.
- Никакого изменения production geometry в этой задаче.

### Основные файлы

- `artifacts/`
- `tests/test_decal_voronoi.py`
- `tests/test_decals.py`
- `docs/cftuv_decal_runtime.md`

---

## A1. Честная семантика Apex Limit; устранить мёртвую BEVEL-ветку

### Принятое решение

Текущий `miter_limit` трактуется как **геометрический Apex Limit**, а не как самостоятельная corner policy.

- Property id `decal_corner_miter_limit` сохранить для обратной совместимости.
- UI label и docs изменить на `Apex Limit`.
- `_CornerPolicy.BEVEL` можно временно сохранить как `RESERVED` для будущего band-дизайна, но `classify_corner_runtime()` не должен возвращать его до P2-семантики.
- Нельзя молча называть усечённый contour «BEVEL policy», пока не определены его UV и band semantics.

### Геометрический контракт

Для любой policy, содержащей удалённый apex:

- `apex_distance <= alpha * apex_limit`;
- `alpha = width / 2`;
- clamp детерминирован и scale-invariant.

Применить к:

1. `MITER` — если intersection дальше лимита, использовать усечённый contour без далёкого apex.
2. `KITE` — заменить hardcoded `8.0` на runtime setting.
3. `ACUTE_SPLIT` — outer apex clamp обязателен. Не удалять весь OUTER component. Построить `clamped_apex` вдоль bisector/apex ray; он должен оставаться строго с внешней стороны cap chord. Если пользовательский limit меньше минимально допустимого, использовать геометрический minimum + epsilon и добавить diagnostic `APEX_LIMIT_SATURATED`.

### Изменения API

- Переименовать внутреннее поле в `apex_limit`, сохранив compatibility adapter от `corner_miter_limit`.
- Один helper `DecalSettings -> CornerRuntimeSettings`; удалить дублирующиеся конструкторы.
- Добавить optional runtime diagnostics:
  - `clamped_miter_count`;
  - `clamped_kite_count`;
  - `clamped_acute_count`.

### Тесты

Добавить full-evaluator tests, а не только classifier tests:

- `test_apex_limit_changes_convex_miter_contour`
- `test_apex_limit_changes_kite_contour`
- `test_apex_limit_clamps_acute_outer_without_gap`
- один compiled plan, limits `1 / 8 / 100`;
- заранее определённые разные contours/areas;
- нет zero-area, overlap, self-intersection;
- preview == confirm geometry signature для Patch Voronoi.

### Stop condition

Если невозможно сформулировать watertight acute outer clamp без изменения coverage, не импровизировать. Зафиксировать failing fixture и вынести на отдельное design decision.

---

## A2. Post-quantization validation и локализация compile failures

### Проблема
Сырое ненулевое edge может схлопнуться после `_quantize_diagram_point()`, получить `segment_length == 0`, вызвать division by zero либо degenerate Boost site.

### Изменения

1. После квантизации каждого site проверить:
   - endpoints различны;
   - `quantized_length > diagram_epsilon`;
   - inward normal валиден.
2. Reject reason:
   - `QUANTIZED_DEGENERATE_SITE`;
   - включить `patch_id`, `edge_index`, raw length, quantized length, quantum.
3. Guard в `_corner_offset_lines()` — division by zero никогда не допускается.
4. Весь вызов `_compile_surface()` обернуть в локальную обработку:
   - известные geometry/backend failures -> `PatchVoronoiCompileFailure`;
   - не скрывать programmer errors без контекста: diagnostic должен содержать exception type/message.
5. После `Construct()` использовать доступные проверки PyVoronoi:
   - degenerate segments;
   - intersecting segments;
   - points on segment.
6. Compile postconditions:
   - каждый accepted physical edge породил не менее одного valid site;
   - каждый site имеет valid SEGMENT ownership atom;
   - отсутствие endpoint POINT cell допустимо;
   - потеря SEGMENT cell = compile failure component'а.
7. Диагностические счётчики тихих деградаций качества. Три fallback'а
   сегодня «впечатывают» триангуляцию в видимые границы граней либо
   молча упрощают контур, не оставляя следа:
   - disorder-fallback в `_cell_polygon` (`decal_voronoi.py:~1349`);
   - tessellation-fallback на `node.mesh_tris` (`:~912`);
   - `_convex_fragment_decomposition` при неудачном merge (`:~859`).
   Сами fallback'и сохраняются (они безопасны), но каждый получает
   именованный счётчик; ненулевые значения видны в verbose report и в
   benchmark JSON из A0. Постусловия п. 6 при этом отличают
   недопустимую потерю (SEGMENT cell) от допустимой деградации.

### Tests

- raw edge `5e-5 BU` при текущем quantum -> rejected/fallback, без Python exception;
- quantized duplicate endpoints;
- injected `Construct()` failure;
- missing segment atom;
- valid degenerate endpoint point-cell не отклоняет plan.

### Acceptance

Один плохой component не отменяет clean components. Routing report содержит точную причину.

---

## A3. Точный accounting selected edges

### Инвариант

```text
selected = accepted_patch_voronoi ∪ accepted_legacy ∪ rejected
```

Множества попарно не пересекаются. Ни одно selected edge не исчезает молча.

### Изменения

1. `_collect_manual_edge_decals()` должен возвращать accounting по каждому physical edge.
2. Edge без BoundaryChain use:
   - `NO_BOUNDARY_CHAIN_USE`;
   - не считать Patch Voronoi или Legacy geometry.
3. `len(uses) >= 3`:
   - `NON_MANIFOLD_EDGE_USE`;
   - не брать первые две стороны.
4. Edge с одной стороной допускается только как явно односторонний boundary site.
5. Routing summary считает только реально скомпилированные edges.
6. Operator report перечисляет rejected count и, в verbose mode, причины/indices.

### Tests

- edge без chain use;
- non-manifold three-use edge;
- one-sided border;
- mixed clean + rejected selection;
- сумма счётчиков равна исходному selected count.

---

## A4. Детерминированное ownership соседних corner crops

### Проблема
Два endpoint corner одного короткого segment могут независимо материализовать одну область.

### Решение

Для каждого incident site ввести endpoint ownership half-plane:

- corner у `site.point_a` владеет частью `t <= 0.5`;
- corner у `site.point_b` владеет частью `t >= 0.5`;
- divider — прямая, перпендикулярная segment в midpoint;
- правило не зависит от порядка corner/site indices.

Применять half-plane всегда; на длинном segment он не меняет crop, пока crop не достигает midpoint.

### Требования

1. Corner components клипятся своим endpoint ownership до merge.
2. Segment crop заполняет остаток — coverage не теряется.
3. Нельзя решать порядковым subtraction corner A из corner B.
4. Работает для `MITER`, `KITE`, `ACUTE_SPLIT`, CAP и будущих policies.

### Tests

- короткий segment между двумя corners, `width > edge_length`;
- разные policies на концах;
- reversed site order;
- проверка:
  - intersection area corner A/B = 0;
  - expected union area == actual union area;
  - нет duplicate face keys;
  - один edge-connected component там, где ожидается.

---

## A5. Fail-fast mathematical/BMesh transaction

### Изменения

1. `_materialize_network_faces()` возвращает structured result либо бросает `DecalMaterializationError`.
2. `dropped > 0` запрещён:
   - preview -> `RETAINED_LAST_VALID`;
   - confirm -> ERROR, production object не заменяется.
3. Все backend partitions сначала evaluate полностью, затем materialize в новый temporary BMesh.
4. Любая ошибка materialization освобождает весь temporary BMesh; partial result не публикуется.
5. Error context:
   - partition backend;
   - edge indices;
   - face index;
   - repeated keys/vertex count;
   - original component kind/side.

### Tests

- injected invalid face после valid partition;
- dropped face не оставляет partial geometry;
- confirm не заменяет старый production object;
- preview сохраняет last-valid pointers.

---

## A6. Structured generation result и valid-settings transaction

### API

Ввести внутренний результат:

```python
class PreviewStatus(Enum):
    UPDATED = "UPDATED"
    RETAINED_LAST_VALID = "RETAINED_LAST_VALID"
    EMPTY = "EMPTY"
    ERROR = "ERROR"

@dataclass(frozen=True)
class DecalGenerationResult:
    status: PreviewStatus
    object_name: str | None
    reason: str = ""
    topology_changed: bool = False
    backend_summary: str = ""
    policy_counts: tuple[tuple[str, int], ...] = ()
```

Существующий public helper, возвращающий `list[str]`, можно оставить compatibility wrapper'ом для не-modal callers.

### Modal contract

Только `UPDATED` меняет:

- `_modal_current_settings`;
- scene property;
- `_modal_current_value`;
- last-valid generation state.

`RETAINED_LAST_VALID`, `EMPTY`, `ERROR`:

- не двигают настройки;
- не меняют production;
- показывают причину в header;
- modal продолжает работать, если ошибка recoverable.

Confirm использует только последние valid settings. Если valid frame не было — явный error.

### Tests

- серия invalid frames после valid;
- confirm использует last valid;
- header содержит reason;
- mocked evaluator обязан проверять проброс `corner_settings`.

---

## A7. Временный preview object и настоящий cancel

### Цель
Не мутировать пользовательский `Decal_*` до confirm.

### Lifecycle

1. Invoke:
   - production object остаётся нетронутым;
   - создаётся один persistent temporary preview object с отдельным internal name;
   - preview object исключён из render/export по возможности и помечен custom property.
2. Mouse move:
   - обновляется только preview object/mesh;
   - topology fast path сохраняется.
3. Confirm:
   - exact evaluate + exact temporary BMesh;
   - если production object существует, сохранить object identity и object-level custom properties;
   - создать/обновить final mesh транзакционно;
   - material slots/required metadata перенести явно;
   - удалить preview object;
   - старый mesh удалить только после успешного swap и если `users == 0`.
4. ESC/RMB:
   - удалить preview object;
   - production object/mesh/materials/custom props не изменились;
   - восстановить scene property/header.
5. Реализовать `cancel(self, context)` для принудительного завершения modal/window shutdown.

### Tests

- existing production object identity, mesh pointer, material slots и custom props после ESC;
- no pre-existing object;
- forced `cancel()`;
- confirm failure;
- в `Decals_Generated` не остаётся мусора.

---

## A8. Input rebase и operator-owned drag targets

### Сначала исправить Shift

При смене `event.shift`:

```text
base_value = current_value
start_mouse = current_mouse
precise_mode = event.shift
```

Нельзя пересчитывать весь накопленный delta с новой sensitivity.

### Drag target abstraction

В `decal_modal.py` ввести descriptor:

```python
@dataclass(frozen=True)
class DecalDragTarget:
    key: str
    label: str
    settings_field: str
    scene_property: str
    kind: str  # DISTANCE | ANGLE | RATIO
    minimum: float
    maximum: float | None
    sensitivity: float
```

Targets:

- `W` — width/height текущего mode;
- `A` — Acute Split Angle;
- `M` — Apex Limit после A1.

Смена target также выполняет rebase. Угол в header показывать в градусах.

Не использовать Property `update=` callback как основной механизм: он не владеет compiled plan/preview state.

### Tests

- Shift press/release не вызывает скачка;
- W -> A -> M без скачка;
- clamp angle `[1°, 179°]`;
- ratio `>= 1`;
- distance > 0.

---

## A9. Честный world/local units contract

### Временный production-safe контракт

Принимать только metric-preserving source transform:

- rotation + translation;
- positive uniform scale.

Отклонять до generation:

- non-uniform scale;
- shear;
- negative determinant/reflection.

Проверять не только `to_scale()`, а isotropy `MᵀM ≈ s²I` и `det(M) > 0`.

Для positive uniform scale `s`:

```text
width_local  = width_world  / s
height_local = height_world / s
offset_local = offset_world / s
uv_length_scale_local = uv_length_scale_world * s
```

Последний пункт обязателен для сохранения world-space texel density.

### Tests

- scale `2.0`: world width и offset равны settings;
- V-length соответствует world arc length;
- non-uniform, shear, mirror -> explicit operator error до preview;
- rotation/translation не меняют topology/signature в local metric.

### Future
Полный world-space backend — отдельный проект. Не смешивать с intrinsic chart work.

---

## A10. Первый live-corner slice: Acute Split Angle

### Preconditions
A1–A9 полностью приняты.

### UX

- Чистый Patch Voronoi selection: `A` включает drag `Acute Split Angle`.
- Если routing содержит Legacy components:
  - рекомендуемый первый вариант: corner targets disabled;
  - header: `Live corner controls require Legacy:0`.
- Молчаливое применение только к части сети запрещено.

### Header diagnostics

Показывать:

```text
Acute Split: 60.0° | MITER:12 KITE:3 SPLIT:2 | 22.4 ms
```

Policy counts получать из evaluator diagnostics, не вторичным полным обходом.

### Tests

- один plan, threshold пересекает известный corner angle;
- `MITER <-> ACUTE_SPLIT`;
- topology `1 component <-> 2 components`;
- старые faces не остаются;
- preview object/mesh identity стабильны;
- `Construct()` count за drag = 0;
- invalid threshold frame -> last valid.

### Definition of done
Поведение «как в видео» для одного честно определённого параметра на Patch Voronoi-only selection.

---

## A11. Four-band parity, CAP/JUNCTION и UV — только после design oracle

### Mandatory design document
До кода создать `docs/decal_corner_bands.md` и получить подтверждение пользователя.

Документ обязан ответить:

1. Какой угол сравнивается: interior, extrusion или signed turn?
2. Какие exact policies/components соответствуют каждому band?
3. Меняется ли UV orientation/topology?
4. Поведение ровно на threshold.
5. Четыре independent bands или hysteresis pairs?

Добавить таблицу углов `170 / 130 / 100 / 75 / 45 / 20°` с ожидаемым contour и UV.

### После approval

1. Расширить `CornerRuntimeSettings` ordered thresholds.
2. Добавить normalization и, если подтверждено, hysteresis state modal lifetime.
3. Реализовать отдельные crop builders.
4. Tangent-aligned CAP вместо axis-aligned square.
5. Valence-N JUNCTION из ordered angular sectors.
6. Определить незанятые reflex sectors.
7. UV anchors для всех semantic corner components.
8. Исправить:
   - V continuity между chain fragments;
   - signed lateral U для two-sided same-chart sites;
   - deterministic UV owner KITE/MITER.
9. Решить legacy parity либо оставить permanent gate.

### Stop condition
Без утверждённой таблицы semantics не добавлять четыре FloatProperty «по названиям со слайда».

---

## A12. S1 Static Interior: переякорение FAN / ACUTE_SPLIT / HAIRPIN

### Результат production review

Первая реализация A12 не принята визуально. Статичный клин вычитался из
incident strips целиком уже на первом кадре, хотя corner crop ещё не заполнил
его. Differential на силуэт-fixture показал потерю площади
`106.000686 -> 95.365561` и сокращение strip-компонентов `8 -> 2`; в Blender
это проявилось разрывами крыльев, схлопыванием и перекрученными faces.

Исправление: stable default возвращён к проверенной A10 collision/evaluation
семантике (полное покрытие, опорные линии только появляются и удлиняются).
A11/A12 bands оставлены явным experimental opt-in без преждевременного
вычитания клина. Следующая попытка dynamic bands требует отдельной
event-freeze модели и покадровой production-приёмки, а не дальнейшего
переякорения crop-полигонов.

### Проблема (диагностирована, подтверждена обоими ревью)

При width drag внутренние швы сетки переезжают, хотя в референс-модели
после локального разрешения области они обязаны замереть. Эмпирика на
силуэт-fixture (углы 131°/62°/111°): на переходах width 1->6 исчезают
1–4 опорные прямые внутренних швов за кадр; движущиеся линии — швы
`FAN:A|B <-> SEGMENT`, лежащие на offset-рельсах инцидентных сегментов
(offset прямой уменьшается ровно на шаг alpha за кадр).

Источник в коде:

- `_corner_offset_lines()` (`cftuv/decal_voronoi.py:~1372`) ставит
  опорные точки компонентов в `P + inward_normal * alpha`;
- `_acute_crop_components()` (`:~1691`) строит на них chord/anchors;
- alpha-зависимые crop-границы затем повторно прорезают strip-грани
  каждый кадр (`_evaluate_surface_crops`, subtraction `:~4128`).

Oracle-инвариант: `docs/decal_corner_bands.md` §5.1 «S1 Static
Interior» (внесён поправкой). Кратко: внутренние швы лежат на
ширино-независимых опорных прямых и только растут вдоль них; двигается
исключительно фронтир (`alpha`-рельсы, apex-клампы).

### Изменения

1. Переякорить ТОЛЬКО `FAN`, `ACUTE_SPLIT`, `HAIRPIN`:
   - боковые границы corner-компонентов — статичные перпендикулярные
     лучи через `P` (границы endpoint-cell диаграммы); рёбра растут
     вдоль них с alpha, прямые не переезжают;
   - crop-полигон угла не выходит за клин между этими лучами и не
     прорезает соседнюю полосу вдоль её рельса;
   - split-хорда INNER/OUTER — статичный compiled-факт, якорённый к
     самой диаграмме (первая Voronoi-вершина клина / пересечение с
     границей конкурента, fallback — граница домена), НЕ
     compiled-дистанция и НЕ `alpha`-точки; хранится в compiled plan
     (например, новое поле `CornerSpec.split_chord`);
   - пока фронтир не достиг хорды, OUTER spike не материализуется и
     появляется, когда рельсы пересекают хорду.
2. `MITER` и `KITE` НЕ трогать — их опорные линии уже статичны
   (перпендикуляры через `P`, биссектрисы); подтверждено замером.
3. Замороженная область обязана замирать после СВОЕГО локального
   разрешения (фронтир покинул окрестность), а не после насыщения
   всего объекта (S1 §5.1 п.4).

### Тесты и приёмка

1. В `tests/test_decal_corner_stability.py` два strict-xfail:
   - `test_s1_interior_supporting_lines_never_move`;
   - `test_s1_resolved_faces_keep_geometry_and_uv`.
   После фикса они начнут проходить, strict-xfail упадёт как XPASS —
   снять оба маркера в том же коммите. Guard
   `test_interior_frozen_after_saturation` обязан остаться зелёным.
2. UV-инварианты по S1 §5.1 п.5: на замороженных гранях стабильны
   `v_lengths` и `u_frac * alpha`; U-пере-нормировка на текущую
   полуширину легитимна (тримовая семантика). При threshold-drag с
   фиксированной width UV замороженных граней идентичны побитово —
   добавить отдельный тест.
3. Differential: изменения вывода разрешены только у FAN/SPLIT/HAIRPIN
   углов и прилегающих strip-граней; MITER/KITE fixtures — байт-в-байт.
4. Покадровая ручная сверка с референсом `slide9` (Adjustable Values):
   при width drag после collision разрешённые области не шевелятся.

### Stop conditions

- Если после переякорения churn опорных линий остаётся (xfail не
  снимается) — НЕ импровизировать event-graph/инкрементальную
  перестройку: зафиксировать замер и вынести отдельным design decision.
- Если статичную split-хорду невозможно определить из диаграммы для
  какого-то класса углов — зафиксировать failing fixture и спросить.

### Не входит в A12

- Дефолты порогов `120/90/60/30` подтверждены пользователем; вопрос
  паритета с референсными `120/90/90/60` — отдельное решение, не здесь.
- Event graph / инкрементальный evaluator — только по F1 и только при
  остаточном churn.

---

# TRANCHE B — Intrinsic Geometry Infrastructure

## B0. Adaptive DiagramTransform

Статус: **IMPLEMENTED**. Compile boundary центрирует chart input,
детерминированно выбирает совместимые `quantum/scale`, проверяет sites и
guard frame до передачи в Boost и локализует невозможный диапазон как
`DIAGRAM_DYNAMIC_RANGE_UNSUPPORTED`. Stable A10 fixtures (`planar`, `door`,
`acute`) сохранены byte-identical; topology проверена на масштабах
`1e-4 / 1 / 1e4` и при global translation.

### Цель
Убрать fixed `_DIAGRAM_SCALE`/absolute quantum до длинных tube charts.

### Data model

```python
@dataclass(frozen=True)
class DiagramTransform:
    center: tuple[float, float]
    scale: float
    inv_scale: float
    quantum: float
    max_abs_input: float
    int_safety_margin: float
```

### Контракт

- chart coordinates сначала центрируются;
- scale выбирается детерминированно из extent и требуемой tolerance;
- все guard segments входят в int safety range;
- post-transform sites не degenerate;
- translation/scale-equivalent geometry даёт эквивалентную topology.

Выбор scale:

- desired quantum учитывает local weld tolerance и relative extent tolerance;
- upper bound учитывает integer range PyVoronoi/Boost с safety margin;
- если одновременно сохранить precision и range невозможно — compile failure `DIAGRAM_DYNAMIC_RANGE_UNSUPPORTED`, не overflow.

### Tests

- одинаковая topology при global translation;
- масштабы `1e-4 / 1 / 1e4`;
- длинный chart;
- micro-edge либо сохраняется, либо явно rejected;
- никакого int overflow.

---

## B1. Развязать shared decal types от legacy backend

Статус: **IMPLEMENTED**. `DecalGeometryFace`, offset-plane lift и минимальные
2D primitives вынесены в acyclic `cftuv/decal_geometry.py`; legacy private
имена сохранены aliases. Differential относительно `6996539` byte-identical
для трёх Patch Voronoi и трёх legacy network fixtures.

Создать небольшой shared module, например `cftuv/decal_geometry.py`:

- public face/result dataclasses;
- offset-plane lift helper;
- общие строго необходимые 2D primitives.

`decal_voronoi.py` не должен импортировать private `_NetworkFace` и `_lift_position` из `decal_network.py`.

Оставить compatibility aliases в legacy module, чтобы не делать большой одновременный refactor.

Acceptance: imports acyclic; оба backend'а проходят differential без geometry changes.

---

## B2. Spatial index для domain triangles и site/corner relations

### Domain triangle grid

Добавить в `DecalSurfaceDomain` immutable индекс AABB -> triangle ids.

Использовать для:

1. intrinsic point location;
2. cell-triangle clipping compile stage;
3. chart edge/triangle-boundary station insertion;
4. self-overlap detection chart builder.

Fallback full scan допустим только как diagnostic/reference path.

### Compiled relation indices

Добавить:

- `atoms_by_site`;
- `owner_atoms_by_corner`;
- `corners_by_site`;
- `sites_by_vertex`;
- `ports_by_vertex/site`.

Это одновременно готовит incremental evaluator.

### Differential
Индексы включаются/выключаются test flag'ом; output identical reference scan.

---

## B3. Chart/provenance contract

### Проблема
Plain `(u, v)` недостаточно для lift на multi-triangle surface и chart transitions.

### Minimal IR

```python
@dataclass(frozen=True)
class DomainLocation:
    chart_id: int
    triangle_id: int
    uv: tuple[float, float]
    barycentric: tuple[float, float, float]
    source_feature: str  # TRIANGLE | EDGE | VERTEX
    source_feature_id: object
    transition_key: object | None
```

2D algorithms продолжают работать с tuples. Provenance хранится рядом с station/materialization layer.

### Lift semantics

- triangle interior: barycentric source position + selected normal mode;
- source edge/fold: least-squares/intersection lift по normals adjacent owner faces;
- source vertex: multi-plane lift по incident owner normals;
- chart cut copies одного source feature получают общий transition key.

`DecalSurfaceDomain.project()` для arbitrary intrinsic 3D point не требуется. Site coordinates создаёт chart builder из известной face/edge provenance.

### Hard vs smooth first slice
Первый intrinsic production slice поддерживает `PIECEWISE_PLANAR_HARD`:

- face normals;
- fold edges используют multi-plane lift;
- smooth interpolated normals — отдельное расширение после developable acceptance.

---

## B4. Explicit width-budget contract для strip charts

Статус: **IMPLEMENTED (contract slice)**. `PatchVoronoiPlan` теперь явно
хранит `alpha_budget`, per-surface `support_triangle_ids`, `budget_source` и
запрошенный compile budget. Текущий PLANAR full-component backend остаётся
честно unbounded и не получает искусственного runtime clamp. Finite strip
plans отклоняют excess width до crop/arrangement с
`DOMAIN_BUDGET_EXCEEDED`; modal сохраняет last valid preview и выполняет
controlled recompile только на confirm, никогда на raw `MOUSEMOVE`.

Strip chart не может молча предполагать бесконечную будущую width.

### Plan fields

- `alpha_budget`;
- `support_triangle_ids`;
- `active_triangle_ids(alpha)`;
- `budget_source`.

### Recommended first policy

- operator передаёт compile budget больше initial width;
- для небольшого patch допускается full connected component;
- для крупного patch используется strip budget;
- если `alpha > alpha_budget` во время drag:
  - `DOMAIN_BUDGET_EXCEEDED`;
  - last valid preview сохраняется;
  - header сообщает limit;
  - confirm может выполнить controlled recompile на requested width, но не во время raw `MOUSEMOVE`.

Добавить advanced setting/derived policy только после UX review. Нельзя скрытно урезать decal domain.

### Strip grouping

Supports двух selected chains на одной surface, пересекающиеся при `alpha_budget`, объединяются в один competition domain. Иначе фронты потеряют глобальное Voronoi collision behavior.

---

# TRANCHE C — Developable Intrinsic Strip Charts

## C0. Chart builder module и IR

Статус: **IMPLEMENTED**. Добавлен чистый `cftuv/decal_charts.py` без
`bpy`/`bmesh`: immutable `ChartTriangle`, `ChartAdjacency`,
`IntrinsicStripChart`, `ChartCut`, `ChartBuildMetrics`, `ChartBuildFailure` и
serialized `ChartSiteSeed`. `PatchNode` теперь хранит `mesh_vert_indices`,
поэтому triangle/BoundaryChain provenance связывается через исходные vertex,
edge и face ids без live BMesh refs. C0 только валидирует и переносит IR;
adjacency flood и support selection остаются C1, hinge unroll — C2.

Создать отдельный module, например `cftuv/decal_charts.py`.

Основные структуры:

- `ChartTriangle`;
- `ChartAdjacency`;
- `IntrinsicStripChart`;
- `ChartCut`;
- `ChartBuildMetrics`;
- `ChartBuildFailure`.

Не помещать chart construction в `operators.py` или в corner builders.

PatchGraph не хранит live BMesh refs. Использовать serialized:

- `mesh_verts`;
- `mesh_tris`;
- `mesh_tri_face_indices`;
- `mesh_tri_face_normals`;
- BoundaryChain edge/face provenance.

---

## C1. Построение triangle adjacency и strip support

Статус: **IMPLEMENTED**. `decal_charts.py` строит детерминированную adjacency
по общим source edges, находит owner triangles selected site seeds и выполняет
ограниченный одним `PatchNode` conservative flood: 3D AABB lower-bound к
seed-segments плюс one-ring safety. Пересекающиеся supports независимых
selected network components объединяются до chart solve. `ChartBoundaryEdge`
фиксирует patch boundary/support cut вместе с triangle/local-edge/face/source
edge provenance. Hinge unroll и production routing в этот срез не входят.

1. Построить adjacency по общим undirected source edges.
2. Seeds — owner triangles selected seam sites.
3. Flood только внутри одного CFTUV patch/topological component.
4. Conservative support inclusion:
   - нельзя исключить triangle, который может пересекать geodesic band `alpha_budget`;
   - для первого варианта использовать adjacency flood + conservative 3D lower-bound/AABB distance и one-ring safety;
   - over-inclusion допустим, under-inclusion запрещён.
5. Supports intersecting network components объединить до chart solve.
6. Зафиксировать support boundary edges и source provenance.

Tests:

- straight bevel strip;
- open quarter cylinder;
- two selected chains, supports merge/non-merge;
- thin nearby wall в другом patch не попадает в support.

---

## C2. Deterministic hinge unroll

### Algorithm

1. Выбрать root triangle детерминированно:
   - сначала triangle с selected site;
   - tie-break source face/triangle id.
2. Положить root в 2D по длинам трёх рёбер.
3. BFS/DFS по deterministic adjacency order.
4. Adjacent triangle раскладывать вокруг shared chart edge, сохраняя три edge lengths.
5. Выбирать сторону shared edge, противоположную interior parent triangle.
6. Сохранять source triangle id, parent edge, chart points.

### Validation

- edge-length relative error <= tolerance;
- orientation consistent;
- already-placed loop closure residual измеряется, не усредняется молча;
- zero-area source triangle -> explicit failure.

### Metrics

- max edge error;
- max loop closure residual;
- discrete angle defect;
- triangle overlap count;
- chart area/source area ratio;
- support triangle count.

---

## C3. Developable admission и cut policy

Первый production slice принимает только charts, удовлетворяющие strict criteria:

- topology disk/open strip;
- angle defect/holonomy ниже tolerance;
- нет non-adjacent 2D self-overlap;
- no ambiguous multiple placements;
- selected chain не лежит на proposed cut.

При нарушении:

- не выдавать приблизительный silent result;
- `ChartBuildFailure`;
- component-level legacy fallback.

### Cut scope первого slice

- Разрешить простой deterministic cut для одной cycle только если cut не пересекает selected sites и после cut chart становится disk.
- Сложные multi-cut/atlas случаи не импровизировать; оставить D/E.

---

## C4. Создание INTRINSIC DecalSurfaceDomain

Chart builder выдаёт:

- `boundary_triangles` в chart space;
- `intrinsic_triangles` с chart/source positions/face normals;
- `chart_id`;
- triangle spatial index;
- source-edge/source-vertex feature maps;
- `alpha_budget`;
- transition metadata.

Sites создаются напрямую из source edge provenance и chart triangle coordinates.

Patch Voronoi compile/evaluate не должен отличать PLANAR и INTRINSIC, кроме domain adapters.

---

## C5. Provenance-safe arrangement и surface-conforming edges

### Station provenance

Каждая materialized station должна разрешаться в `DomainLocation`.

Особые случаи:

- interior triangle;
- source fold edge;
- source vertex;
- chart boundary/cut transition.

### Triangle-boundary insertion

Любое arrangement edge, пересекающее intrinsic triangle boundary, получает station в точке пересечения. Это предотвращает chord, прорезающий piecewise-planar owner surface.

### Curvature-adaptive subdivision

Для будущего smooth mode добавить generic criterion, но в C first slice достаточно точного split по source triangle boundaries.

Если smooth normal mode позже включён:

- subdivide, когда sagitta/normal variation выше tolerance;
- budget vertices per edge;
- deterministic.

### Fold lift

Station на source fold edge должна использовать adjacent face normals и общий source-edge key. Две стороны не получают разные несваренные offset points.

---

## C6. Integration и fallback

1. `compile_patch_voronoi_attempt()`:
   - PLANAR fast path сохраняется;
   - non-planar patch сначала пробует intrinsic developable strip chart;
   - текущий mini-planar split можно оставить fallback'ом только если explicitly routed/documented.
2. Failure локализуется на topology component.
3. Backend summary показывает:
   - `PLANAR`;
   - `INTRINSIC_DEVELOPABLE`;
   - `LEGACY`;
   - failure reasons.
4. Runtime/preview settings общие.

---

## C7. Acceptance fixtures developable

Обязательные Blender fixtures:

1. Two-plane folded strip.
2. Multi-face bevel strip.
3. Open quarter cylinder.
4. Open full cylinder with chart seam outside selected network.
5. Densely tessellated cylindrical fillet.
6. Two chains on one curved surface, collision при росте width.

Проверки:

- один edge-connected decal component, где ожидается;
- no zero-area/non-manifold/overfull edges;
- no connector artifacts между каждой парой facets;
- source triangle edge-length preservation;
- requested geodesic half-width error в пределах tolerance;
- preview/confirm parity Patch Voronoi;
- no `Construct()` during drag;
- active triangle count proportional strip support, не full patch, для large fixture;
- deterministic output при reversed triangle enumeration.

Definition of done C: фаски, cylindrical fillets и open tubes обслуживаются одним intrinsic chart, а не набором planar mini-surfaces с connector'ами.

---

# TRANCHE D — Periodic Domains

## D0. Periodic IR

Расширить domain:

- `periodic_axis`;
- `period`;
- `wrap_origin`;
- deterministic cut source features;
- transition equivalence map.

## D1. Cut selection

Для annulus/tube выбрать cut:

- не пересекает selected chain/sites;
- максимально удалён от network в пределах support;
- deterministic tie-break по source edge id;
- записан в diagnostics.

## D2. Periodic Voronoi copies

- sites в пределах `alpha_budget` от U-boundary получают copies `U ± period`;
- domain boundary triangles при необходимости также копируются;
- после clipping coordinates canonicalize modulo period;
- keys через transition equivalence, не только quantized U/V.

## D3. UV transport

- closed chain имеет один transport direction;
- V continuity через periodic seam;
- U side parity стабильна;
- no duplicate faces на wrap.

## D4. Tests

- closed cylinder;
- closed polygonal tube;
- selected seam близко к chart cut;
- collision через periodic boundary;
- reversed winding/normal sign;
- width до alpha_budget.

---

# TRANCHE E — Non-developable / Organic Research

Не начинать как production implementation без отдельного spike.

## E0. Admission metrics spike

На sphere/cliff fixtures измерить:

- discrete angle defect;
- alternate-path closure residual;
- chart self-overlap;
- sampled chart-distance vs mesh-path distance;
- width distortion;
- normal variation.

## E1. Recommended first approximation

Сначала проверить, достаточно ли локального hinge chart с deterministic cuts на узкой support strip. Не внедрять отдельный exponential-map solver, пока metrics не показывают необходимость.

## E2. Distortion budget

Approximate domain принимается только если:

- max width error <= утверждённого допуска;
- no fold-over/self-overlap;
- no topology ambiguity;
- diagnostics явно маркируют `INTRINSIC_APPROXIMATE`.

Иначе component fallback/reject. Silent approximation запрещена.

## E3. Multi-chart atlas

Только если реальные production assets требуют широкой декали на поверхности, не проходящей E2:

- overlapping charts;
- transition maps;
- shared station keys;
- cross-chart arrangement stitching.

---

# TRANCHE F — Performance, Native C++, GPU

## F0. Phase-level profiler

Evaluator timings минимум:

- runtime classification;
- crop construction;
- atom clipping;
- subtraction;
- fragment merge;
- arrangement station insertion;
- owner resolution/lift;
- NetworkFace emission;
- BMesh build;
- mesh update.

Записывать median/p95, не один best run.

---

## F1. Python algorithmic/incremental pass

До native backend:

1. Использовать B2 relation indices — убрать `corners × atoms`, `ports × sites` scans.
2. Threshold drag:
   - определить changed policy corners;
   - reuse cached pending fragments неизменившихся components;
   - rebuild whole dirty surface arrangement, если локальная conformity не доказана.
3. Width drag по-прежнему invalidates all width-dependent crops.
4. Coalesce MOUSEMOVE: максимум один evaluate на redraw/timer tick.
5. Active intrinsic triangle set по strip width/budget.

### Gate

Цель первой ступени:

- `walls.003` p95 <= 33 ms;
- `walls.001` p95 <= 50 ms, целевой <= 33 ms;
- curved 200+ edge fixture p95 записан.

Если target достигнут — C++ не обязателен.

---

## F2. Native backend decision spike

Начинать только если profile показывает, что Python polygon pipeline остаётся доминирующим.

### Не предполагать заранее 10–50x
Speedup считается только по prototype benchmark.

### Варианты

1. **Direct pybind11 port current semantics** — выше шанс bit/differential parity.
2. **Clipper2/int64** — потенциально удобные booleans, но перед использованием проверить:
   - license/redistribution;
   - hole/concave semantics;
   - provenance и deterministic ordering;
   - отличие от текущего coverage.

### API boundary

Не вызывать C++ на каждый polygon. Один native batch минимум на surface, предпочтительно на полный evaluation batch:

```text
sites + atoms + corner policies + crops + domain triangles
    -> faces + keys + UV + diagnostics
```

Передавать contiguous POD buffers/memoryviews, не тысячи Python objects.

### Reference contract

- Python evaluator остаётся reference/fallback.
- Feature flag native on/off.
- Randomized differential corpus + production fixtures.
- Любое отличие классифицируется как intended semantics change или bug.

### Distribution

- optional bundled wheels для поддерживаемых Blender/Python/platform matrix;
- import isolation внутри decal backend;
- clear unavailable fallback;
- CI/import test каждой wheel.

---

## F3. GPU overlay — только display adapter

### Gate

Начинать, если после Python/native evaluator materialization/mesh update составляет заметную долю frame time (например >25%) либо мешает target FPS.

### Контракт

- evaluator остаётся CPU/native exact;
- GPU draw handler рисует exact output triangles/UV preview;
- BMesh/mesh создаётся один раз на confirm;
- никакого shader/SDF approximation как authoritative preview;
- cancel/lifecycle/view change обработаны.

GPU compute для polygon topology не входит в scope без отдельного доказанного research case.

---

# 4. Curvature-specific safety contract

## Offset safety

Для intrinsic strip собирать conservative curvature proxy:

```text
kappa_edge ≈ angle(normal_a, normal_b) / edge_length
```

Хранить:

- `max_kappa`;
- estimated minimum radius;
- requested/effective offset ratio.

Если `abs(offset) * max_kappa` превышает утверждённый limit:

- preview сохраняет last valid и показывает warning либо использует явно видимый clamp;
- confirm не должен молча выдавать сильно уменьшенный offset;
- diagnostics содержат requested/effective offset.

Не выбирать numeric threshold без fixture validation.

## Chord error

- Piecewise planar: split edges по source triangle boundaries — exact within facets.
- Smooth mode: adaptive stations по normal variation/sagitta with vertex budget.

---

# 5. Общая acceptance matrix

Каждый принятый этап обязан сохранять:

| Инвариант | Проверка |
|---|---|
| No Construct in drag | instrumented counter |
| Determinism | repeated serialized output |
| Preview/confirm semantics | Patch Voronoi signature equality |
| Coverage | underfill = 0 и overfill/overlap = 0 в tolerance |
| Topology | no zero-area, duplicate, non-manifold, overfull edges |
| Accounting | selected = accepted + rejected |
| Transaction | failure не заменяет production object |
| Cancel | production identity/data preserved |
| Units | world width/offset/UV density correct for accepted transforms |
| Routing | every component backend/failure explicit |
| Performance | median + p95 before/after |
| Fallback | unsupported chart/component isolated |

---

# 6. Рекомендуемая последовательность commits

1. `test: add decal runtime benchmark and deterministic serializer`
2. `fix: make apex limit semantics explicit across corner builders`
3. `fix: reject quantized degenerate sites and localize compile failures`
4. `fix: account every selected seam edge and reject non-manifold uses`
5. `fix: partition overlapping endpoint corner ownership`
6. `fix: fail fast on decal face materialization loss`
7. `refactor: return structured decal preview results`
8. `fix: isolate temporary modal preview and implement cancel lifecycle`
9. `fix: rebase decal drag on shift and target changes`
10. `fix: enforce metric source transform and world unit conversion`
11. `feat: live acute split angle target with policy diagnostics`
12. `refactor: add adaptive diagram transform`
13. `refactor: extract shared decal geometry types`
14. `perf: add domain triangle and corner relation indices`
15. `feat: define intrinsic chart provenance and width budget`
16. `feat: build strict developable strip hinge charts`
17. `feat: materialize provenance-safe intrinsic decal domains`
18. `feat: add periodic tube domains`
19. `perf: incremental surface evaluation`
20. Native/GPU commits только после gates.

---

# 7. Стоп-условия для исполнителя

Остановиться и запросить решение, если:

1. Four-band semantics нельзя вывести однозначно из утверждённого oracle.
2. Acute apex clamp не сохраняет coverage/watertight topology.
3. Strip support не может гарантировать отсутствие under-inclusion при выбранном width budget.
4. Hinge chart имеет holonomy/self-overlap выше допуска и требует сложного atlas cut.
5. Non-uniform/mirrored source transform требуется поддержать немедленно — это отдельный world-space design.
6. Native prototype меняет polygon ordering/coverage без объяснимого intended change.
7. Для исправления decal backend требуется менять core frontier/UV solve semantics.
8. Тесты проходят только за счёт ослабления tolerance или удаления regression assertion.

---

# 8. Первый готовый task prompt для Codex

Ниже — рекомендуемый первый пакет. Не начинать весь master plan сразу.

```text
Работай в helmdubo/CFTUV от актуального HEAD ветки
claude/blender-decal-corner-preview-yq4lir. Канонический workplan лежит в
этой же ветке: docs/CFTUV_decal_runtime_intrinsic_codex_workplan.md.

Перед изменениями прочитай AGENTS.md, docs/cftuv_decal_runtime.md,
docs/cftuv_decal_nonplanar_roadmap.md и этот workplan. Зафиксируй HEAD,
полный pytest и benchmark walls.003/walls.001.

Выполни только задачи A0, A1 и A2:
1) deterministic runtime benchmark/serializer;
2) честная семантика Apex Limit для MITER/KITE/ACUTE_SPLIT,
   без активной BEVEL policy;
3) post-quantization site validation, division guards,
   PatchVoronoiCompileFailure вокруг _compile_surface и segment-atom postconditions.

Guardrails:
- не менять PyVoronoi compile/evaluate split;
- Construct не вызывается во время drag;
- не добавлять four-band UI;
- не начинать intrinsic/C++/GPU;
- не менять core solve/frontier;
- comments в коде на русском;
- каждый behavior change покрыть full-evaluator regression;
- standard fixtures должны совпасть differential, кроме углов,
  у которых Apex Limit намеренно меняет contour.

Verification:
python -m compileall -q cftuv tests
python -m pytest -q
git diff --check
Blender differential walls.003/walls.001 widths 2.0/3.0/3.7076/4.5

После выполнения дай:
- список changed files;
- точное описание semantics;
- commands/results;
- benchmark before/after;
- remaining risks;
- один commit на A0, один на A1, один на A2.
```

---

# 9. Финальная цель программы

Пользователь выбирает seam network на архитектурном asset, запускает modal preview и получает:

- live Width/Acute/Apex/band controls без повторного Voronoi solve;
- строгий last-valid transaction и безопасный cancel;
- корректные corners без overlap/тихих потерь;
- одинаковую metric width в world units;
- один intrinsic decal network на фасках, cylindrical fillets и tubes;
- explicit fallback на unsupported curvature;
- предсказуемый performance, подтверждённый p95 benchmarks;
- optional native acceleration и GPU overlay только там, где они дают измеримый выигрыш.
