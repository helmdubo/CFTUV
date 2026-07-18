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

# 0a. Текущая очередь исполнения (обновлено на финале Tranche E)

Порядок строг; смешивание срезов запрещено differential-дисциплиной:
срезы 1 и 3 имеют ПРОТИВОПОЛОЖНЫЕ differential-критерии (сохранить
байт-в-байт против осознанно изменить с пересдачей эталонов) — в одном
коммите/срезе differential перестаёт доказывать что-либо.

1. **Срез W (§0b)** — актуальный рабочий срез: W1 §8b-фиксы по
   T7-P3 (design-stop разрешён, см. `docs/decal_atlas_design.md`
   §3a T7-P3 и §8b п.2) -> W2 полный E-acceptance -> W3 C8.1+C8.4
   (re-baseline) -> W4 E0-замер грубых фикстур (данные для G3,
   порогов не менять).
2. Параллельно (пользователь): ручная проверка rounded_wall drag в
   Blender на актуальном code commit.
3. **C8.3 -> C8.2** (секция C8): продолжение полевых дефектов.
   НАМЕРЕННО меняет exact-путь; differential пересдаётся по каждому
   пункту. Только после E-acceptance (C8.1+C8.4 уходят в Срез W).
4. **G1** (TRANCHE G): imprint source-рёбер — строго после C8.2
   (общий merge/key-слой). Следом G3 (пороги по данным W4) и G2
   (border-крыло; закрывает полевую «юбку» вместе с G3).
5. **Merge GPU-ветки** `claude/decal-gpu-preview-f3` (F0-lite + F3
   готовы, 353 passed): rebase на стабилизированную основную ветку +
   ручной чек-лист acceptance F3 в Blender.
6. **F1** (инкрементальность на M1) — гейт FP2 открывается полным
   E-acceptance; цель — вернуть APPROXIMATE-кадр из секунд в
   интерактивный диапазон.

---

# 0b. Срез W — закрытие §8b (T7-P3) + полевые дефекты кольца

Контекст. Исполнитель реализовал буквальный 1-quantum gate из §8b
п.2 и корректно остановился: gate вскрыл, что unbounded fallback был
load-bearing (`owner image is missing` на 669q при width 0.25; drop
внешних faces рвёт связность `[646, 5, 2]` при width 1.0). Design
decision принят и записан — **T7-P3** в
`docs/decal_atlas_design.md` §3a: T2-декларация интервалов И ЕСТЬ
транспортированный owner-space SEGMENT-предикат; расстояние никогда
не является источником метки. Транспорт новых SEGMENT-кривых через
T7-P2-механику НЕ требуется. Ответ на бинарный вопрос исполнителя:
вариант A по духу, но предикат уже транспортирован станционной
декларацией — нового curve-экспорта не вводить; вариант B (adoption
+ проверка) отклонён как два механизма с возможным расхождением.

Полевой вход (полусфера, лог 2026-07-17): (1) юбка вместо облегания
— связка G2+G3, в этом срезе только W4-замер (данные до кода);
(2) расшитие/наслоение у дырки почти-замкнутого кольца — C8.4
(+C8.1), в этом срезе целиком, с диагностическими якорями X6/L6.

Differential-дисциплина внутри среза: W1-W2 — EP1 байт-в-байт
(default-нейтральные atlas-коммиты); W3 — намеренная пересдача
эталонов exact-пути; W4 — только данные/отчёт, нулевой продакшен-diff.
Смешивать W1/W3-коммиты запрещено.

## W1. Закрытие §8b по T7-P3 (atlas, EP1 байт-в-байт)

1. Переложить WIP hard-gate на семантику T7-P3 (§3a, СЕМЬ пунктов —
   предписание, не варианты): declared-token присвоение на любой
   дистанции + `atlas_declared_owner_count`; tie-break >= 2 токенов
   только owner-space интервалом — ТОЧНЫЙ half-open containment
   `[s_i, s_{i+1})`, последний интервал замкнут, proximity-стоп у
   станции УДАЛЁН (T7-P3.3, follow-up stop chart 15); канонизация
   вырожденных интервалов <= 1 кванта на T2-этапе до публикации
   контракта, инвариант «нет интервалов <= 1q» + счётчик
   `atlas_degenerate_interval_merge_count` (T7-P3.7, снимает
   фантомную KITE/SEGMENT смену класса из near-boundary
   нормализации); sliver без декларации — строго <= 1 квант +
   существующий счётчик; drop дальше кванта без декларации —
   counted; сверка `expected_semantic_class` ->
   `ATLAS_TRANSITION_DESYNC` при расхождении.
2. Закоммитить подготовленный O1-UV фикс: сравнение СЫРЫХ per-side
   UV с каноническим до override, допуск `max(1.5 кванта, 2% alpha)`
   — допуск утверждён, обоснование в §9 через E2-бюджет.
3. Закоммитить подготовленные тесты: fault-injection (искусственная
   порча изометрии/держателя -> оракул обязан покраснеть — снимает
   тавтологичность O1 доказуемо), grid double-cover sweep,
   reversed-enumeration на >= 3 ширинах.
4. Регрессия на репро design-stop: saddle, width 0.25/0.5/1.0/2.0 —
   components == 1, double-cover == 0, `atlas_declared_owner_count`
   > 0, beyond-quantum sliver adoption == 0 (ассерт), drop-счётчики
   стабильны по ширинам. Отдельно оба follow-up face-кейса
   (chart 15, transition (15, 37), width 0.25): same-class
   SEGMENT-пара 12028/12030 с representative в 0.434q от станции —
   классифицируется half-open без стопа; хвостовой 1q-интервал
   12032 поглощён (счётчик > 0), face получает единственный
   declared-токен KITE-владельца. Третий кейс (chart 16, две
   touched transitions (15,36)+(15,37), width 0.25): проекции вне
   обоих touched-span'ов — clamp по T7-P3.3a на каждой transition,
   выбор по максимальной мере touched-span'а (T7-P3.3b) -> токен
   12030; ни одного multi-hop вызова на этом кейсе (ассерт — 3b
   не ходит по графу). Четвёртый кейс — O1 raw-UV drift (R5;
   transition (39,61) 17<->19, subedge 4..709, width 0.25,
   alpha 0.25, |dV| был 0.0499812): после фикса транспорта оба raw
   совпадают с canonical в tolerance 0.005 на ОБОИХ концах
   subedge; election-ассерт (локальный owner frame строго выше
   imported-копии, никаких chart_id-tie между разноранговыми);
   provenance-ассерт (imported frame трассируется к owner chart);
   fault-injection порчи изометрии по-прежнему красный. Пятый кейс
   — T2-C coverage (та же transition (39,61): контракт публиковал
   `(0, 2838, owner=None)` при материализованных subedges, стороны
   несли imported-frames РАЗНЫХ sites 12036/12034, dV=0.050159,
   у 12034 clamped-параметр): после фикса — election контракта
   включает imported-image кандидатов, объявленный владелец над
   всем материализованным спаном; инвариант T2-C (owner=None под
   парным subedge = `ATLAS_TRANSITION_DESYNC`) зелёный на всех
   фикстурах; R1-аудит images chart19 (endpoint-corner/12036);
   `atlas_clamped_segment_face_count == 0` глобально; обе стороны
   subedge трассируются к ОДНОМУ owner-frame'у (chart18), O1 в
   tolerance на обоих концах. Шестой кейс — V-континуация
   (T7-P3.8; chart4, transition (5,26) 4<->6, face stations
   1111..1250, declared SEGMENT 12010, параметр за endpoint на
   0.0669797 при q=1e-4, corner5 tau ~ 0.0006 rad, policy KITE):
   после фикса V течёт сквозь corner5 без clamp'а;
   `atlas_clamped_segment_face_count == 0` глобально БЕЗ ослабления
   guard'а; `atlas_v_continuation_count > 0` на репро; континуация
   только через углы с поглощённой (суб-квантовой) структурой —
   ассерт; O1 в tolerance на subedge; кейсы 1-5 остаются зелёными.
   Седьмой кейс — структурный гейт континуации (chart14/site12026,
   vertex14 degree=2, tau=0.00124, alpha=0.125: alpha*tau=1.55q,
   overrun=2.01q, внутренний face без transition): континуация
   разрешается по компайл-флагу `corner_structure_absorbed`, НЕ по
   коэффициенту — кейс проходит, потому что crop угла фактически
   поглощён; негативные пары: absorbed=False + overrun -> fail;
   отсутствие image соседа -> fail; absorbed=False без кривых угла
   в arrangement -> fail (потерянные кривые). Восьмой кейс —
   enforcement separator-требований (T7-P3.9; chart5, transition
   (5,27) 5<->6, class-change станция 2831 SEGMENT12010 ->
   KITE12012, kite absorbed=False): requirement-стадия из
   канонизированного контракта экспортирует kite crop-edge кривую
   с якорем в станции 2831 -> face разрезан, O2 зелёный; негатив
   (искусственно удалить экспорт) обязан падать
   `ATLAS_CLASS_DESYNC` НА requirement-стадии ДО arrangement, а не
   на O2 — проверка порядка обнаружения. Ассерт на всех фикстурах:
   в опубликованных T2-контрактах нет интервалов длиной
   <= 1 кванта.
5. §9: записать T7-P3 evidence + пункт 4 прежнего списка §8b
   (bisector-ветка/single-hop guard, обоснование допуска).

Acceptance W1: полный suite зелёный; EP1 байт-в-байт (worktree-diff);
счётчики в benchmark JSON; ни одного нового solver/Construct().

## W2. Полный E-acceptance

После W1 — отчёт (baseline SHA, suite, differential-протокол,
счётчики saddle/sphere/rounded_wall фикстур, T8-оракул на всех
ширинах). Короткое подтверждение ревьювером по diff — затем
E-acceptance объявляется закрытым в §0a. Ничего не имплементировать
в W2 — это gate.

## W3. C8.1 + C8.4 (stable-путь, намеренная пересдача эталонов)

Строго после W2. Состав — секции C8.1 и C8.4 плюс якоря полевого
лога (в C8.4): пре-шаг канонизации X7/X8 span-чекера
(диагностический слой, нулевой blast-radius, отдельный коммит);
headless-репро «кольцо N, выделено N-1..N-2»; X6
endpoint_pair_mismatch и L6 как repro-маркеры — после фикса X6
обязан исчезнуть, L6-топология (незамыкающая seam-дуга в одном
патче) обслуживается, а не подавляется. Каждый пункт — отдельный
коммит с пересдачей differential и обновлением связанных S1-тестов.

Acceptance W3: репро-фикстура — ноль overfull, ноль щелей, сегменты
у дырки коллинеарны направляющим; CAP не axis-aligned на stable-пути;
X6 отсутствует на репро; полный suite зелёный.

## W4. E0-замер грубых фикстур (только данные, для G3)

Купол 16x8 (полевой Sphere-кейс), купол 32x16, cyl 12/16/24: таблица
per-fixture — угловой дефект max/mean, measured width error узкой
декали (E0-harness), какой из reasons режет сейчас
(DISTORTION_BUDGET_EXCEEDED / NORMAL_VARIATION_EXCEEDED /
PERIODIC_HOLONOMY_UNSUPPORTED). НИКАКИХ правок порогов в этом срезе
— таблица уходит ревьюверу и открывает G3 в следующем (правило
E0.b: данные до кода).

## Stop conditions среза

- Любая необходимость нового curve-транспорта, нового key-space или
  ослабления component/double-cover оракулов в W1 — стоп, отчёт.
- W3: если диагноз C8.4 уводит в voronoi-механику wrap-биссектрисы
  (не концевые фреймы legacy/boundary-строителя) — стоп с диагнозом
  до фикса.
- Расхождение W4-данных с ожиданием (measured width error купола
  16x8 ВНЕ бюджета) — не чинить, зафиксировать в таблице: это
  меняет решение G3 и решается ревьювером.

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

Статус: **IMPLEMENTED**. C1 support раскладывается immutable BFS по
детерминированному root/site-owner и deterministic adjacency order. Каждый
`ChartTriangle` хранит `parent_triangle_id`, `parent_source_edge` и три chart
points; первая placement остаётся канонической, а повторный приход через cycle
только измеряет loop-closure residual без усреднения координат. Реализованы
edge-length/orientation validation, explicit zero-area failure и метрики angle
defect, non-adjacent overlap, area ratio, closure и edge error. Developable
admission/cut policy остаётся C3, production routing — C6.

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

Статус: **IMPLEMENTED**. Утверждённый oracle из
`docs/decal_chart_admission.md` реализован отдельным pure module:
G1–G8, 2% defect/closure budget, numeric/area/overlap guards и canonical
failure reasons. Disk проходит без cut; annulus получает максимум один
детерминированный physical source-edge cut между boundary components, никогда
не по synthetic triangulation diagonal или selected site. Остальная topology
локально отклоняется без atlas-импровизации.

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

Статус: **IMPLEMENTED**. Admitted chart преобразуется в
`DecalSurfaceDomain(kind="INTRINSIC")` с barycentric source positions,
piecewise face normals, immutable triangle AABB grid, physical/synthetic
source-edge и source-vertex feature maps, `alpha_budget`, chart/cut transition
metadata. Selected sites разрешаются по edge+owner-face provenance и проходят
через тот же `_compile_surface()`/PyVoronoi core, что PLANAR domain.

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

Статус: **IMPLEMENTED**. Arrangement groups intrinsic faces по chart identity
и после обычного cell-to-cell conforming pass вставляет deterministic stations
во все crossings и collinear source vertices triangle-boundary graph. Каждая
station затем разрешается существующим `DomainLocation`; physical fold edges
получают общий transition key и least-squares offset lift по normals обеих
owner faces, поэтому materialized edge следует facets, а не режет их 3D chord.

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

Статус: **IMPLEMENTED**. `compile_patch_voronoi_attempt()` сохраняет
PLANAR fast path, а non-planar PatchNode компилирует через C1–C5
INTRINSIC developable chart pipeline. Chart/admission failure локализуется до
всех selected sites затронутого patch component и явно маршрутизируется в
LEGACY без частичного покрытия. Backend summary различает `PLANAR`,
`INTRINSIC_DEVELOPABLE`, mixed plan и `LEGACY`, а также печатает canonical
failure reasons. Runtime width/preview используют общий compiled plan; для
intrinsic surfaces старые межплоскостные junction-connectors не создаются.

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

Статус: **IMPLEMENTED**. Девять oracle-классов покрыты автоматическими C1–C7
fixtures: F1/F2/F3/F4/F5/F6 проходят developable admission, N1/N2/N3 дают
canonical rejection. C7 дополнительно проверяет multi-facet bevel без
junction-connectors, geodesic half-width четверть-цилиндра, 96-сегментный
strip support (`9/192` triangles) и merge двух фронтов на кривой. Живой
Blender sweep на `rounded_wall` (68 seam edges) подтвердил единый connected
result, no zero-area/overfull/connectors, preview/confirm parity и ноль
повторных `Construct()` во время drag. Замеры записаны в
`docs/cftuv_decal_runtime.md`.

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

## C8. Полевые дефекты кривых поверхностей (ремедиация по production-скриншотам)

Диагностировано независимым расследованием с headless-репро (стена
четверть-цилиндра + планарный top + планарный торец, chain по верхней
кромке + arris). Три задачи, по одной на коммит. ВНИМАНИЕ: C8.1 и C8.2
намеренно меняют exact-путь (это баг-фиксы) — differential пересдаётся
осознанно с записью в docs; координация с T7-P работами в
decal_voronoi.py обязательна (разные зоны файла: corner crops и key
derivation против atlas transitions).

### C8.1 FLAT CAP на stable-пути (вершинный «веер во все стороны»)

Корень: классификация валентности-1 корректна (CAP), но при
`dynamic_corner_bands=False` (production default)
`_corner_crop_components` (`decal_voronoi.py:~2690-2711`) не доходит
до tangent-aligned `_cap_crop_components` (`:~2176`, dynamic-only) и
берёт `_corner_crop_polygon`, который для валентности != 2 возвращает
**chart-axis-aligned квадрат `(x±alpha, y±alpha)`** (`:~2893-2902`).
Endpoint point-cell не ограничен вглубь поверхности — blob заполняет
квадрат на ОБЕИХ смежных поверхностях (замер: reach 1.27*alpha назад
за вершину, расползание по стене до z=0.828). Это старый axis-aligned
CAP дефект первого аудита, доживший на stable-пути.

Фикс: policy CAP маршрутизируется в `_cap_crop_components` НЕЗАВИСИМО
от dynamic_corner_bands (bands-оракул §7: FLAT — дефолт). Легаси-
квадрат остаётся только для валентности > 2 до junction-фанов §7.
Acceptance: на репро-фикстуре behind-reach конца цепочки ~0 (замерено:
с dynamic=True уже так); грани CAP — 4-вершинные START/END полосы
только на двух смежных через ребро поверхностях; differential:
изменения только у valence-1 вершин.

### C8.2 Source-feature ключи для PLANAR (несшитые полосы на переходе)

Корень (три факта): (1) `DecalSurfaceDomain.locate` для PLANAR даёт
константный provenance, а `_domain_location_key` (`:~6956-6963`)
рано возвращает геометрический patch-local ключ `('pv', patch_id,
uv)` ДО source-feature ветки (`:~6976-6994`), которой пользуются
INTRINSIC-поверхности — смежные полосы, терминирующиеся на ОДНОМ
невыбранном source-ребре, получают разные канонические ключи: ноль
сварок, видимый зазор ~1.4*offset (каждая сторона лифтует по своей
нормали). (2) Mid-spine сварки существуют только через одностороннее
зеркалирование `_synchronize_cross_surface_spine_stations`
(`:~7308-7402`) — knockout-эксперимент разваливает фикстуру на
компоненты. (3) Единственный compiled owner у arris-ребра
(`decals.py:~1065-1070` принимает single-use boundary run) даёт
одностороннюю полосу, связанную с остальными только через
вершинный ключ: edge-components = 2 на всех ширинах — точный репро
скриншота.

Фикс: (a) планарные owner-поверхности получают source-feature
provenance (boundary source edge/vertex ids в compile; `locate()`
резолвит границу в EDGE/VERTEX), PLANAR early-return в
`_domain_location_key` снимается при наличии source feature — обе
стороны любого общего source-ребра (выбранного и нет) выводят
идентичные ключи, сварка становится key-exact вместо sync-зависимой;
(b) single-use внутреннего seam-ребра — routed compile failure
(legacy fallback), не молчаливая односторонняя полоса; (c) spine-sync
обобщается на слияние уже разбитых runs по позиции вдоль общего
source-сегмента. Acceptance: на репро edge-components == 1 на всех
ширинах; зазор лифта на общем ребре == 0 (единая станция); knockout
sync не разваливает компоненты (ключи самодостаточны).

### C8.3 SMOOTH pass-through sub-band (сплиты convex-микроуглов)

Корень: sub-band `tau <= 10 deg` из E4-поправки bands-оракула не
реализован (откачен до M1 и не возвращён). На тесселированной гладкой
кромке каждый convex-микроугол получает corner-компонент, тогда как
concave-сторона обслуживается биссектрисой Вороного — асимметрия со
скриншота 3.

Фикс: политика SMOOTH по оракулу (`decal_corner_bands.md`, sub-band
в MITER): angle-only порог (константа модуля), corner-компонент не
создаётся, area point-cell'а классифицируется к смежным
SEGMENT-владельцам по биссектрисе, V течёт насквозь. На stable-пути
реализуется как pass-through в crop-слое (wedge-заполнение с kind
SEGMENT и непрерывной V, сварка с сегментными гранями по общим
ключам). Acceptance: на кромке 10-15 сегментов convex-сторона даёт
ту же рёберную структуру, что concave (одно биссектрисное ребро на
вершину, ноль дополнительных сплит-граней); углы tau > порога — без
изменений; S1-тесты зелёные (критерий angle-only).

### C8.4 Почти-замкнутое кольцо: расшитие/наслоение у однорёберной дырки

Полевой репорт (полусфера, нижняя кромка, выделено всё кольцо кроме
одного ребра): рядом с дыркой сегменты ленты слегка повёрнуты
относительно направляющих — перекрытие с одной стороны, щель с другой.
Гипотеза: концевые фреймы двух концов почти замкнутой цепочки
считаются по односторонним данным и расходятся; закрытое кольцо
обслуживается wrap-биссектрисой, открытая цепочка — концы далеко,
а почти-замкнутый случай не покрыт; возможен вклад C8.1
(перекрывающиеся axis-aligned CAP обоих концов внутри дырки).
Требуется: headless-репро (кольцо N рёбер, выделено N-1), диагноз
точного строителя (legacy trim / boundary / voronoi), фикс +
регрессия «кольцо с дыркой в 1..2 ребра: ноль overfull, ноль щелей,
сегменты у дырки коллинеарны направляющим». Выполнять вместе с C8.1
(общая зона концевых фреймов).

Диагностические якоря из полевого лога (полусфера, 2026-07-17) —
использовать как repro-маркеры headless-фикстуры:
- `[CFTUV][TopologyInvariant] X6 endpoint_pair_mismatch` — стороны
  общей границы декомпозируют её на chains с разными концевыми
  парами; вырожденная пара вида `(v,v)` — closed-chain anchor,
  выбранный сторонами по-разному. Ровно рассинхрон концевых фреймов
  из гипотезы C8.4: после фикса на фикстуре «кольцо N, выделено
  N-1..N-2» X6 обязан исчезнуть.
- `[CFTUV][Boundary] L6 raw_loop_repeats_boundary_edge` — обход
  границы патча проходит seam-ребро дважды: сигнатура
  незамыкающей seam-дуги внутри ОДНОГО патча (кольцо с дыркой не
  разделяет поверхность). Это штатная топология репро, а не порча
  меша; чекер прав, но фикс C8.4 обязан обслуживать этот случай,
  а не подавлять предупреждение.
- Пре-шаг нулевого blast-radius (диагностический слой): X7/X8
  span-чекер (`_validate_boundary_loop_topology`,
  `analysis_boundary_loops.py`) не канонизирует closed single-chain
  случай — при `start_loop_index == end_loop_index`
  `_loop_vertex_span`/`_loop_edge_span` возвращают полный список
  без поворота и без замыкающего повтора, тогда как реальная
  закрытая цепочка хранится с anchor-поворотом и повтором первой
  вершины (actual 33 против expected 32 в логе). Ложные X7/X8 топят
  реальный сигнал X6/L6 — канонизировать сравнение (поворот +
  closing repeat) до сравнения. Продукционный код не трогать.
- `[CFTUV][LoopClassDiag] uv_vs_planar_mismatch` (перестановка
  HOLE/OUTER) на сферических патчах — ожидаемое расхождение:
  планарная проекция купола складывается, containment-порядок
  ненадёжен. Не задача C8.4; действий не требует (патч всё равно
  отвергается admission и уходит в legacy).

### Порядок и связи

C8.1 -> C8.3 -> C8.2 (по возрастанию blast-radius; C8.2 трогает
key-пространство — после него пересдаётся differential сварок).
C8.4 — вместе с C8.1. G1 (imprint) строго после C8.2: обе задачи в
merge/key-слое.

---

---

# TRANCHE D — Periodic Domains

Замкнутые трубы/кольца: чарт строится с одним deterministic cut
(механика C уже умеет, fixture F4), а периодичность — метаданные
поверх этого cut. Никакого нового типа чарта.

## D-предрешения (утверждены; не пере-обсуждать в автономном режиме)

**DP1 — Транзверсальное пересечение cut разрешено; коллинеарное —
запрещено.** Уточнение G7 admission-оракула для periodic-случая:
кольцевая цепочка вокруг трубы НЕИЗБЕЖНО пересекает каждую
образующую, включая cut. Запрещено только совпадение cut с рёбрами
selected chain (коллинеарный overlap — прежний
`CUT_CROSSES_SELECTED_CHAIN`). Транзверсальное пересечение полосы
декали с cut — штатный режим, который и обслуживают periodic copies.
Без этого уточнения каждая кольцевая декаль была бы отвергнута —
т.е. весь транш был бы пуст. Обновить формулировку G7 в
`docs/decal_chart_admission.md` тем же коммитом, что и D1.

**DP2 — Только трансляционная периодичность в v1.** Два cut-ребра в
chart space обязаны быть параллельными трансляциями друг друга
(взаимный поворот <= 0.02 rad — тот же бюджет, что G3). Кольцо на
коническом фрустуме разворачивается в сектор кольца (периодичность
вращательная) — v1 отказывает с новым reason
`PERIODIC_HOLONOMY_UNSUPPORTED` -> component fallback. Конусные
кольца — кандидат в E, не импровизировать.

**DP3 — Период квантуется решёткой диаграммы.** `period` снапится к
кванту `DiagramTransform` (B0) при compile, чтобы modulo-свёртка
координат была бит-точной. Сварка вершин через periodic seam — ТОЛЬКО
через transition equivalence keys (`ChartCut.transition_key`, B3/C0);
сырой float `U % period` в качестве ключа запрещён. Станция на
U = 0 и её образ на U = period — один и тот же ключ по построению.

**DP4 — `alpha_budget` клампится в `period / 2`.** Drag шире —
существующая B4-семантика `DOMAIN_BUDGET_EXCEEDED` (last valid
preview + header), никакого нового UX. Ровно на `period/2` кольцевая
декаль закрывает трубу целиком: фронты самосталкиваются вдоль
антиподальной линии, покрытие обязано замкнуться без щели и без
double-cover (это фикстура D4.6).

**DP5 — Копии существуют только в диаграмме.** Sites/corners в зоне
`alpha_budget` от любого из двух cut-краёв дублируются на
`U ± period` как входы PyVoronoi; материализация дедуплицируется
каноническими ключами (DP3). Ни одна грань не эмитится дважды; счётчик
copies — в diagnostics/benchmark JSON.

## D0. Periodic IR

Статус: **IMPLEMENTED**. `IntrinsicStripChart` и `DecalSurfaceDomain`
получили валидируемые immutable periodic fields: axis, квантованный period,
quantum, wrap origin, generating `ChartCut` и transition equivalences.
Non-periodic IR запрещает скрытые periodic metadata. Benchmark JSON добавляет
`periodic_domains` только при их наличии, сохраняя non-periodic report schema.

### Изменения
- Расширить domain/chart: `periodic_axis` (уже поле), `period`
  (квантованный, DP3), `wrap_origin`, ссылка на порождающий
  `ChartCut`, transition equivalence map (canonical key <-> images).
- Валидация в `__post_init__`: period > 0 при periodic_axis != "",
  period кратен кванту, wrap_origin внутри [0, period).
### Tests
- IR-валидации; сериализация periodic-полей в benchmark JSON.
### Acceptance
- Никакого изменения поведения non-periodic путей (differential
  walls + C7 fixtures — байт-в-байт).

## D1. Cut selection для замкнутого support

Статус: **IMPLEMENTED**. Annulus cut candidates скорятся по максимальной
3D-дистанции до ближайшего selected site с canonical source-edge tie-break;
диагностика хранит distance/count. Транзверсальное пересечение разрешено,
коллинеарный selected source edge по-прежнему отклоняется. После hinge
unroll две cut images проверяются на translation holonomy с бюджетом 0.02;
layout канонически ориентируется по U и period вычисляется на общей
`DiagramTransform`-решётке. Frustum даёт
`PERIODIC_HOLONOMY_UNSUPPORTED`. G7 oracle уточнён в том же коммите.

### Изменения
- Кандидаты cut: пути по source-рёбрам вдоль periodic-направления.
- Фильтр: без коллинеарного совпадения с selected chains (DP1).
- Скоринг: максимальная дистанция до ближайшего selected site в
  пределах support; tie-break по минимальному canonical source edge id.
- Проверка DP2 (параллельность cut-краёв после unroll) — иначе
  `PERIODIC_HOLONOMY_UNSUPPORTED`.
- Выбор записан в diagnostics (`cut.reason`, дистанция, кандидаты).
### Tests
- Детерминизм при reversed enumeration и при повороте меша;
- кольцевая цепочка (транзверсальная к cut) ПРИНИМАЕТСЯ (DP1);
- конусный фрустум -> reject с точным reason.

## D2. Periodic Voronoi copies

Статус: **IMPLEMENTED**. `_compile_surface()` добавляет diagram-only images
selected sites у обоих cut-краёв на `U ± period`, включая точную границу
`alpha_budget`. Image cells клипятся базовым intrinsic domain и возвращаются
к canonical owner site до atom IR, поэтому materialized sites/faces не
дублируются. Period проверяется на той же `DiagramTransform` quantum-grid;
`periodic_copy_count` доступен в diagnostics/benchmark. Non-periodic input
сохраняет прежний порядок PyVoronoi segments.

### Изменения
- Зона копирования: `alpha_budget` от каждого cut-края; копируемые
  сущности: sites, их endpoint corners, при необходимости boundary
  triangles домена.
- После clipping координаты канонизируются modulo period (бит-точно,
  DP3); ключи станций/вершин — через equivalence map, не через U/V.
- Диагностика: `periodic_copy_count`, `periodic_weld_count`.
### Tests
- Сварка через seam бит-точна: серия ширин, ни одного дубля ключа,
  ни одной пары несваренных совпадающих вершин;
- цепочка, лежащая ровно на cut-краю зоны копирования (граничный
  случай квантования).
### Stop condition
- Если бит-точная сварка недостижима без изменения квантования B0 —
  остановиться и задать вопрос, не ослаблять допуски сварки.

## D3. UV transport через periodic seam

Статус: **IMPLEMENTED**. Замкнутые selected-компоненты periodic chart получают
единственное детерминированное направление V от минимальной source vertex и
incident edge; суммарный диапазон равен физической длине кольца. Diagram image
shift хранится целым числом на atom и применяется к runtime crop/site view,
не размножая materialized IR. Seam identity канонизируется исключительно по
`transition_equivalences`; `periodic_weld_count` считает фактически сведённые
станции с двумя chart images. Float-modulo не используется.

### Изменения
- Замкнутая цепочка: один transport direction, V монотонна вдоль
  кольца, V-шов — в детерминированной точке (closure point цепочки),
  суммарная V-длина = длина кольца (для будущего snap на целые
  повторы паттерна — ответственность texturing-слоя, producer отдаёт
  факт).
- U side parity стабильна через seam; winding валидируется против
  owner normal (существующее правило).
- S1-инварианты действуют: periodic images статичны, внутренние швы
  не переезжают при width drag.
### Tests
- V-непрерывность и отсутствие V-скачка на seam (кроме closure
  point); дубликаты faces на wrap = 0; S1 supporting-lines тест на
  кольцевой фикстуре.

## D4. Приёмочные фикстуры

Статус: **IMPLEMENTED**. Acceptance suite
`tests/test_decal_tranche_d_acceptance.py` содержит все восемь fixture cases:
round cylinder, hard polygonal duct, near-cut image, collision через seam,
reversed winding, точное `period/2`, frustum reject и отсутствие legal cut.
Общие gates: connected manifold, zero-area=0, duplicate faces=0,
preview=confirm, `Construct()` неизменен после compile. D4.6 дополнительно
сравнивает площадь arrangement с площадью periodic domain, одновременно
доказывая gap=0 и double-cover=0.

| # | Фикстура | Проверяет |
|---|---|---|
| D4.1 | Closed cylinder, кольцевая цепочка | базовый путь: cut транзверсален (DP1), copies, сварка |
| D4.2 | Closed polygonal tube (hex/oct duct, hard edges) | периодичность + fold'ы одновременно |
| D4.3 | Selected chain на расстоянии < alpha от cut | copies обязаны включиться; сварка через seam |
| D4.4 | Две цепочки, коллизия ЧЕРЕЗ periodic boundary | конкуренция фронтов сквозь seam |
| D4.5 | Reversed winding / normal sign | детерминизм и parity |
| D4.6 | Width drag до `alpha_budget = period/2` | антиподальное замыкание: gap = 0, double-cover = 0, S1 после самостолкновения |
| D4.N1 | Конусный фрустум, кольцевая цепочка | **REJECT** `PERIODIC_HOLONOMY_UNSUPPORTED` + fallback |
| D4.N2 | Cut, коллинеарный selected chain (все кандидаты) | **REJECT** `CUT_CROSSES_SELECTED_CHAIN` |

Общие проверки — как в C7 (component connectivity, zero-area = 0,
preview == confirm, `Construct() == 0` в drag, детерминизм).

### Definition of done Tranche D
1. Все восемь фикстур по таблице; счётчики copies/weld в отчёте.
2. Кольцевой трим на замкнутой трубе закрывается без щели на любом
   width вплоть до `period/2`.
3. Non-periodic пути не изменились (differential C7 + walls).
4. Обновлены `docs/decal_chart_admission.md` (G7 + DP1-DP2 reasons) и
   `docs/cftuv_decal_runtime.md`.

## D5. Ремедиация по результатам независимого ревью (ОБЯЗАТЕЛЬНА до E)

Ревью HEAD 191c22e: DP1/DP3/DP5 соблюдены (weld бит-точен на 10
ширинах, копии только в диаграмме, антиподальное замыкание точное),
но найдены дефекты. Выполнить как первый пакет любой следующей сессии.

### D5.1 (HIGH) DP4 недостижим на production-пути
`plan.alpha_budget` берёт requested-бюджет, если тот конечен
(`decal_voronoi.py:~4879-4892`), а periodic-clamp живёт только на
`chart/domain.alpha_budget` (`decal_chart_admission.py:~495`).
Production (`decals.py:~76-83`) всегда передаёт конечный бюджет =>
preflight `DOMAIN_BUDGET_EXCEEDED` (`decal_voronoi.py:~830`) для
periodic-чартов не срабатывает никогда: drag шире `period/2` молча
успешен. `docs/cftuv_decal_runtime.md` утверждает обратное.
Фикс: `plan.alpha_budget = min(requested, domain-минимум по всем
surfaces)`. Тест: D4.6 дополнить ассертом
`width > period` -> `DOMAIN_BUDGET_EXCEEDED` через ПУБЛИЧНЫЙ
`compile_patch_voronoi_plan` (не через ручную сборку плана — текущие
тесты именно поэтому дефект не видят).

### D5.2 (HIGH, scope) Cut-кандидаты — только одиночные рёбра
`decal_chart_admission.py:~188-212`: кандидат cut — одно ребро между
двумя boundary-компонентами. Труба высотой более одного quad-ряда
отклоняется `MULTI_CUT_REQUIRED` при любом бюджете — весь periodic-путь
работает только на вырожденной односегментной полосе. Реализовать
D1 по спеке: cut = детерминированный ПУТЬ по source-рёбрам вдоль
periodic-направления (тот же скоринг/tie-break). Фикстура: труба
8x3 кольца вершин, кольцевая цепочка по среднему кольцу -> ADMIT,
замыкание без щели. Без D5.2 DoD-2 транша считается невыполненным.

### D5.3 (MEDIUM) Отсутствует periodic S1-тест — и он бы падал
Требуемый D3 «S1 supporting-lines тест на кольцевой фикстуре» не
написан. Фактический sweep по D4.1: счётчик граней прыгает
14->8->10->8, шесть внутренних швов на статичных опорных прямых
исчезают между ширинами (topology-pop; покрытие при этом точное —
area ratio 1.0). Причина — width-чувствительный fragment merge через
границы intrinsic-треугольников (вероятно унаследовано из C). Добавить
S1-тест (паттерн `test_decal_corner_stability.py`) на D4.1-фикстуре и
устранить pop. Если корень в C-merge — чинить там, differential C7
пересдать осознанно.

### D5.4 (LOW/MEDIUM) Латентная асимметрия seam-corner subtraction
`decal_voronoi.py:~6191-6194`: subtraction переводит corner crops
только на `atom.periodic_shift * period`, а emission-путь (~6129-6133)
дополнительно учитывает `corner.site_u_offsets`. Для corner'а с
crops, чьи sites пересекают cut с ненулевыми offsets — double cover.
Сегодня недостижимо (seam-углы MITER-flat/CAP с нулевыми offsets),
станет достижимым с D5.2. Выровнять формулы + тест с не-geodesic
цепочкой через cut.

### D5.5 (LOW) `budget_source` мислейбл + мёртвые equivalence-ключи
`PERIODIC_HALF_PERIOD` ставится безусловно, даже когда связал
strip-бюджет (`decal_chart_admission.py:~495-496`) — исправить.
`transition_equivalences` image-ключи (`~483-491`) не используются
ни одним resolve-путём — удалить или задействовать, не оставлять
декоративную инфраструктуру.

### Acceptance D5
Все пять пунктов; полный suite зелёный; differential C7/walls
байт-в-байт; D4.6 расширен budget-ассертом; periodic S1 тест зелёный;
`docs/cftuv_decal_runtime.md` приведён в соответствие с фактическим
поведением бюджета.

---

# TRANCHE E — Non-developable / Approximate Charts

Пологая двойная кривизна: купола, апсиды, мягкие скалы. Базовый механизм
остаётся hinge unroll (C2), а E добавляет ИЗМЕРЕНИЕ фактического искажения,
второй admission-tier, multi-chart atlas для interior overlap и одну
пользовательскую ручку. Конформный backend допускается только как отдельный
research fallback после отрицательного результата atlas-пути.

## E-предрешения (утверждены; не пере-обсуждать в автономном режиме)

**EP1 — Поведение по умолчанию не меняется ни на бит.** Gate G1-G8 с
порогами C остаётся производственным дефолтом. E добавляет второй tier
`APPROXIMATE` поверх: `EXACT` (defect <= 1e-4 rad) | `APPROXIMATE`
(в пределах budget, с явной маркировкой) | REJECT. При дефолтных
настройках admission-исходы всех C7/D4/walls фикстур байт-идентичны
текущим — это differential-guardrail всего транша.

**EP2 — Hinge/atlas first; conformal только отдельным исследованием.**
Production-путь E строится из существующего hinge unroll, измерения,
admission и E3-atlas. Собственные LSCM/ABF/exponential-map реализации и
`bpy.ops.uv.unwrap` не добавляются в этот путь. Если atlas не способен
пройти E.3 по метрикам, разрешён отдельный conformal spike с теми же
policy-free метриками, headless/parity-анализом и собственным design review.
Его результат не становится production backend автоматически.

**EP3 — Admission по ИЗМЕРЕННОМУ искажению, не только по прокси.**
G3/G4 (angle defect / closure residual) остаются быстрым отсевом, но
решение о tier'е принимает прямое сэмплирование ошибки ширины (E0
harness): `max_width_error_sampled <= budget`. Прокси может ошибаться
в обе стороны на неравномерной тесселяции — меряем то, что видит
артист.

**EP4 — Материализованная полоса непрерывна, chart topology может быть
атласной.** Curvature-relief cuts в margin-зоне остаются допустимы. Внутри
`alpha_budget` разрешены только E3 chart-переходы: обе стороны перехода
материализуются как одна поверхность и свариваются по каноническим
transition keys механизма D. Raw cut, несваренная граница или UV-фазовый
скачок внутри полосы запрещены. Поэтому «нет трещин» остаётся product
инвариантом, но больше не означает «ровно один hinge chart».

**EP5 — Ручка одна, и она compile-параметр.**
`decal_chart_distortion_budget`: float, default `0.02`, min `0.005`,
soft_max `0.05`, max `0.10`. Участвует в admission (compile stage) —
поэтому НЕ modal drag target и НЕ поле `CornerRuntimeSettings`;
изменение в панели требует повторного запуска оператора (как выбор
backend'а). Default `0.02` == текущий G3-порог => EP1 выполняется
автоматически.

**EP6 — Знак кривизны безразличен.** Седло (отрицательная гауссова
кривизна) обрабатывается тем же |defect|-критерием, что купол.
Отдельных веток для эллиптических/гиперболических точек не заводить.

**EP7 — E3 разрешён и предшествует окончательной приёмке E2.** E0 дал
полевое доказательство: gentle saddle имеет малую width error, но interior
self-overlap single hinge chart. Пользователь разрешил реализацию atlas.
Порядок продолжения: независимое review настоящего контракта и
`docs/decal_atlas_design.md` -> E3 implementation -> E1 margin relief -> E2
admission/UI. E.3 saddle остаётся целевой положительной фикстурой. Если E3
не проходит её без нарушения EP1/EP3/EP4, разрешённое снижение scope должно
быть явным: saddle становится REJECT/deferred, а не скрытой аппроксимацией.

## E0. Measurement harness + spike (два коммита)

### E0.a Harness (без изменения production-поведения)

- Расширить `ChartBuildMetrics`: `max_width_error_sampled`,
  `max_station_normal_variation` (rad, по owner-набору станции),
  `foldover_count` (если отличается от `triangle_overlap_count` —
  задокументировать различие).
- Измерение ширины — по определению §4 admission-оракула
  (`docs/decal_chart_admission.md`): K >= 20 станций x 2 rails;
  эталон mesh-расстояния — точный путь по развёрнутой triangle-strip
  (не Dijkstra по рёбрам: он завышает).
- Всё — в benchmark JSON (A0) и в `verify_decal_charts`-скрипт.
- Acceptance: differential всех существующих фикстур байт-в-байт;
  на C7-фикстурах `max_width_error_sampled <= 1%` (проверка самого
  harness'а: на developable он обязан показывать ~0).

### E0.b Spike-отчёт (данные до кода admission)

- Прогнать harness на новых кривых фикстурах (см. E2-таблицу) БЕЗ
  изменения admission; результаты — таблицей в
  `docs/decal_organic_spike.md`: fixture x {defect, residual,
  width_error_sampled, normal_variation, overlap}.
- Сверка с provisional-порогами E2. Расхождение больше чем вдвое по
  любому порогу — stop condition: отчёт пользователю, пороги не
  подгонять самостоятельно.

## E1. Curvature-relief cuts в margin-зоне

- Deterministic relief cuts из границы support'а внутрь, терминация
  не ближе `alpha_budget` к цепочке (EP4); цель — устранение
  2D self-overlap широких кривых support'ов, НЕ метрики внутри полосы.
- Выбор детерминирован (max дистанция до цепочки, tie-break по
  canonical edge id — паттерн D1); каждый cut с reason и
  transition-ключом.
- Acceptance: фикстура «широкий support на куполе», которая без
  relief cuts даёт `CHART_SELF_OVERLAP`, с ними — admissible;
  материализованная полоса без швов; differential C/D нетронут.
- E1 не обязан устранять interior overlap E.3: это ответственность E3.
  Margin relief и interior atlas используют один canonical transition-key
  contract, но остаются разными compile decisions и разными counters.

## E2. Approximate tier + budget knob

### Изменения

- `IntrinsicStripChart.admission_tier: "EXACT" | "APPROXIMATE"`
  (метрики остаются policy-free контейнером — tier живёт в chart).
- Gate: EXACT если `discrete_angle_defect <= 1e-4` и
  `max_width_error_sampled <= 0.005`; иначе APPROXIMATE если ВСЕ:
  - `max_width_error_sampled <= budget` (EP5);
  - `discrete_angle_defect <= 2 * budget` (быстрый отсев, G3');
  - `triangle_overlap_count == 0` и `foldover_count == 0` (binary);
  - `max_station_normal_variation <= 0.35 rad (~20°)` — стабильность
    lift/offset (provisional, калибруется E0.b);
  - G1/G5/G6/G7/G8 без изменений;
  иначе REJECT.
- Новые reasons: `DISTORTION_BUDGET_EXCEEDED`, `FOLDOVER_DETECTED`,
  `NORMAL_VARIATION_EXCEEDED`. `NON_DEVELOPABLE_SUPPORT` остаётся для
  отказа при выключенном approximate-пути (budget на дефолте).
- Маркировка: tier в diagnostics, routing report
  (`Approx:<n>c`), benchmark JSON; `approximate_admit_count`.
- UI: одна FloatProperty (EP5) в панели Decals, tooltip честно
  называет это допуском искажения ширины.
- Offset-safety §4 workplan обязателен на APPROXIMATE-чартах
  (кривизна там гарантированно ненулевая).

### Фикстуры E2

| # | Фикстура | Ожидание (budget=0.02 если не сказано) |
|---|---|---|
| E.1 | Кольцо-полоса на sphere cap, R >= 10*alpha | APPROXIMATE; width_err <= 2%; S1-стабильность швов при width drag |
| E.2 | Мягкий cliff-band (displaced plane, bounded curvature) | APPROXIMATE; ноль трещин в полосе |
| E.3 | Седло (гиперболическая точка, gentle) | APPROXIMATE через E3 atlas (EP6/EP7); ноль materialization cracks |
| E.4 | Купол промежуточной кривизны | REJECT при 0.02; APPROXIMATE при budget=0.05 (семантика ручки) |
| E.N1 | Sphere cap R ~ 2*alpha | REJECT `DISTORTION_BUDGET_EXCEEDED` даже при max budget |
| E.N2 | Высокочастотный crumple | REJECT `FOLDOVER_DETECTED` |
| E.N3 | Дефолтные настройки на E.4 | REJECT — исход идентичен поведению до E (EP1) |

### Definition of done E (автономная часть)

1. Фикстуры по таблице, отрицательные включительно; счётчики в отчёте.
2. Differential: C7 + D4 + walls при дефолтах — байт-в-байт (EP1).
3. `Construct() == 0` в drag; S1-тесты проходят на E.1.
4. Полоса на APPROXIMATE atlas не содержит геометрических трещин или
   UV-фазовых скачков на внутренних chart transitions (EP4).
5. `docs/decal_chart_admission.md` дополнен tier-таблицей и новыми
   reasons; `docs/cftuv_decal_runtime.md` — замером E.1.
6. `docs/decal_organic_spike.md` содержит данные E0.b и фактические
   значения всех порогов.

## E3. Multi-chart atlas — design review и implementation

Статус: **IMPLEMENTED** (`ba75d0c`). Предусловие P0 закрыто `578269a`;
независимое review R1–R4 зафиксировано `55249aa`.

Канонический design contract — `docs/decal_atlas_design.md`. До кода он
проходит независимое review. Реализация обязана:

- детерминированно разбить support на локально injective hinge charts;
- хранить interior transitions как compile IR, не пересобирать их при drag;
- использовать canonical source/station keys и D-style equivalence для
  бит-точной геометрической сварки;
- переносить непрерывную V-фазу и согласованный U через переход;
- материализовать каждый source-region ровно один раз, без double cover;
- публиковать `atlas_chart_count`, `interior_transition_count`,
  `interior_weld_count` и per-chart admission metrics;
- пройти E.3 saddle, не меняя C7/D4/walls differential и S1.

Если эти условия недостижимы, зафиксировать failing fixture и отдельно
выбрать один из уже разрешённых исходов: временно исключить saddle из
положительного E2 scope либо открыть conformal research spike по EP2.

## Stop conditions Tranche E

1. E0.b расходится с provisional-порогами более чем вдвое.
2. E3 не может построить injective atlas E.3 с бит-точной сваркой —
   остановиться перед изменением квантования/материализации и вынести
   evidence на решение: scope reduction либо conformal spike (EP2/EP7).
3. Измерение width error недетерминировано между прогонами.
4. Differential при дефолтах меняется (EP1) — остановиться, не
   «чинить» differential правкой эталонов.
5. Фикстура E.1 требует ослабления S1 — остановиться (это сигнал
   архитектурной проблемы, не допуска).

---

# E4 — Single-Cover Arrangement (M1): архитектурное закрытие stop-condition E#5

Статус: решение принято архитектурным ревью. Симптом: на organic
fixtures публичный compile/evaluate даёт fragmented materialization
(sphere_cap: 9 компонентов при ОДНОЙ цепочке, 3 overfull-ребра;
ear-грань дублирует часть SEGMENT-ngon; MITER + 2 SEGMENT владеют
одним ребром). Корень ДО atlas: вся эмиссия построена на
inclusion-exclusion — N независимых clipping-конвейеров
(atom ∩ crop, atom − crops, ownership dividers, triangulation
conforming) обязаны совпасть бит-в-бит, чтобы плоскость была покрыта
ровно один раз. На плоскости совпадения держались точной
коинцидентностью линий; кривизна её разрушает. Это ТА ЖЕ семья, что
перекрывающиеся corner crops (A-аудит), ear-области T-junction union
(area-preserving костыль) и topology-pop D5.3.

## Решение: канонический слой — pre-materialization arrangement

Принцип: **перестать вычитать — начать классифицировать.**
Cells, crops и dividers перестают быть режущими инструментами и
становятся ПРЕДИКАТАМИ; эмиссия итерирует грани одного общего
snap-rounded arrangement на surface. Compile atom ownership даёт
факты (кривые и предикаты — S1-статичны), arrangement даёт структуру;
правка семантики crops поштучно отклонена — путь whack-a-mole, что и
показал список откатов (post-lift T-weld, подавление point-cell,
удаление ear по subset-ключам).

## Инвариант M1 (single-cover conforming arrangement)

Для каждой surface на каждом width: эмитированные грани образуют
конформное разбиение покрытой области —

1. каждая точка покрытой области внутри ровно одной грани;
2. каждое внутреннее каноническое ребро принадлежит ровно 2 граням;
   frontier/domain-границы — ровно 1; overfull невозможен;
3. нет T-junctions: вершина на ребре всегда делит его в обеих гранях;
4. каждая arrangement-грань эмитируется ровно один раз с ровно одним
   owner (site, component_kind);
5. никакого spatial post-weld: сварка тривиальна по lattice-ключам,
   потому что все грани — из одного графа.

## Алгоритм (детерминированный, integer)

1. Собрать граничные кривые как polylines в квантованной решётке B0:
   (i) рёбра Voronoi-ячеек (compile), (ii) статичные corner supporting
   rays / биссектрисы / split-хорды (S1, compile), (iii) рёбра
   domain-треугольников (compile), (iv) frontier: rails на alpha,
   apex-клампы (runtime), (v) atlas transitions (R1/R2 контракта E3).
2. Snap-rounding: каждый сегмент разрезать во всех пересечениях и во
   всех вершинах, лежащих на нём (обобщение T-junction root fix);
   совпадающие sub-рёбра схлопнуть в одно. Результат — планарный граф
   без пересечений и без T-junctions.
3. Трассировка граней half-edge обходом (существующий merge — 80%
   этой машинерии).
4. Классификация каждой грани ОДИН раз по репрезентативной внутренней
   точке, детерминированный приоритет: JUNCTION-сектор > corner
   component (точка в клине между лучами + policy-область) > SEGMENT
   (ячейка-владелец); tie-break (corner_index, site_index). После
   snap-rounding границы предикатов совпадают с рёбрами графа —
   неоднозначность невозможна.
5. Эмиссия: kind/UV от owner'а (существующий affine cache).

Double-cover и overfull становятся НЕВОЗМОЖНЫ по построению, а не
подавлены пост-обработкой. Perf: тот же асимптотический объём кривых,
что у текущего клиппинга; инкрементальность (пересборка только
frontier-затронутых граней) — F-транш.

## SMOOTH pass-through (MITER на smooth organic chain)

Да: sub-band внутри MITER, angle-only (S1-безопасно, НЕ зависит от
width): `tau <= 10 deg` (константа модуля, без ручки) — corner
component не создаётся вовсе; area point-cell'а классифицируется к
смежным SEGMENT-owner'ам, разделённым биссектрисой (она уже кривая
arrangement); V течёт насквозь. Это убирает сотни микро-углов на
тесселированных smooth-цепочках (главный источник sliver-нагрузки),
но корректность обеспечивает M1, а не pass-through: микро-MITER выше
порога тоже обязан быть корректным. Поправка внесена в
`docs/decal_corner_bands.md`.

## Acceptance-оракул M1 (отличает законные компоненты от cracks)

Реализация-независимый, законен до и после E4:

1. `overfull == 0` на всех width;
2. grid double-cover: детерминированная растеризация решёткой
   `4 * quantum` — каждая внутренняя ячейка покрыта <= 1 гранью;
3. **компонентный инвариант**: (a) при минимальном width число
   edge-connected компонентов == числу связных компонентов выбранной
   сети (sphere ring: 1 — мгновенно ловит текущие «9»); (b) число
   компонентов МОНОТОННО НЕ ВОЗРАСТАЕТ по width — фронты сливаются;
   рост числа компонентов при росте ширины = crack (формальное
   отличие законного «ещё не столкнулись» от дефекта); (c) на
   насыщении == ожидаемому финальному числу;
4. позиционный граф связности == canonical-key граф (несваренные
   совпадающие вершины);
5. существующие S1-sweeps.

Закоммитить оракул НЕМЕДЛЕННО как strict-xfail на E.1/E.3 (паттерн
D5.3): он — механизм приёмки E4.

## Вердикт по E1/E2/E3 commit

Принять СЕЙЧАС с отключённым production-routing: policy/harness/IR
ортогональны материализации. Условия: (a) APPROXIMATE-tier чарты НЕ
материализуются — маршрут в существующий fallback с явным reason
`APPROXIMATE_MATERIALIZATION_PENDING` (инвариант 11 — никакого тихого
успеха); (b) M1-оракул закоммичен strict-xfail; (c) EP1 differential
зелёный; (d) E3-atlas код — за тем же гейтом (его acceptance всё
равно недостижим до E4: saddle 69 компонентов). После E4: снять
xfail, включить routing, пройти acceptance E2/E3 целиком.

## Stop conditions E4

1. Snap-rounding требует изменения кванта B0 — остановиться.
2. Классификация даёт неоднозначность на конкретной грани —
   зафиксировать минимальную фикстуру, не вводить приоритет-хаки.
3. Perf-регрессия evaluator больше 1.5x на walls.001 — остановиться,
   профилировать до оптимизации.
4. Manifold/S1-оракулы НЕ ослабляются ни при каких условиях.

---

# TRANCHE F — Performance, Native C++, GPU

## F-предрешения (последовательность и зависимости от E)

**FP1 — Порядок исполнения: F0 -> F3 -> (E4) -> F1 -> F2.**
GPU preview (F3) продвинут ВПЕРЁД и больше не гейтится долей
materialization в кадре: его основная ценность — не миллисекунды, а
**стабильность латентности**: во время drag исчезают все записи в
mesh datablock и depsgraph-уведомления — источник viewport-статтера,
не видимый в чистом времени evaluator'а. F3 — display adapter, он не
трогает геометрическую семантику вообще (самый безопасный срез
программы).

**FP2 — Зависимость от Tranche E раздельная.**
- F0 и F3 НЕ требуют E: работают на planar/C/D production-путях и
  могут идти параллельно E4 (интерфейс faces стабилен: E4 меняет,
  КАК грани вычисляются, не их формат).
- F1 и F2 ЖЁСТКО ждут E4/M1: инкрементальность и порт на C++
  строятся вокруг горячего цикла, а M1 меняет сам горячий цикл
  (clip/subtract/merge -> arrangement + classification).
  Оптимизировать или портировать pre-M1 конвейер = вкладываться в
  код, который E4 выбрасывает.
- E1/E2/E3 (organic admission/atlas) для F не требуются вовсе.

**FP3 — Blender-версии.** F3 реализуется на базовом `gpu` API
(`gpu.types.GPUShader`, `gpu_extras.batch`, `SpaceView3D.
draw_handler_add`, `gpu.state`) — доступен во всём поддерживаемом
диапазоне 4.1+; жёсткой зависимости от 4.5 НЕТ. Blender 4.5 LTS
даёт выгоду прозрачно (Vulkan-бэкенд ускоряет те же вызовы);
использовать 4.5-only API можно только за feature-детекцией с
fallback, не как требование.

**FP4 — Parity by construction.** GPU-путь рисует ТОТ ЖЕ выход
evaluator'а, что пишется в mesh; confirm — прежняя точная BMesh
materialization. Никакой shader/SDF-аппроксимации как authoritative
preview (прежний запрет остаётся).

## Параллельный GPU-трек: протокол координации (F0-lite + F3)

GPU-срез выполняется ОТДЕЛЬНЫМ исполнителем параллельно E4. Правила:

1. **Ветка.** GPU-агент работает в отдельной ветке
   `claude/decal-gpu-preview-f3`, созданной от зафиксированного
   baseline-коммита основной ветки (указывается в стартовом задании).
   В основную ветку E4-исполнителя GPU-агент НЕ пушит. Слияние — по
   готовности F3, rebase GPU-ветки на актуальную основную; конфликтов
   быть не должно при соблюдении п.2.
2. **Владение файлами.** GPU-агент МОЖЕТ менять: новый модуль
   `cftuv/decal_gpu_preview.py`, display-путь preview в
   `cftuv/operators.py` / `cftuv/decal_modal.py`, benchmark-скрипты
   `artifacts/`, новые тесты `tests/test_decal_gpu_preview.py`,
   docs. GPU-агент НЕ КАСАЕТСЯ: `decal_voronoi.py`,
   `decal_charts.py`, `decal_chart_admission.py`, `decal_network.py`,
   геометрических путей `decals.py` — это территория E4. Потребность
   изменить их = stop condition, не правка.
3. **Замороженный интерфейс.** Контракт faces
   (`DecalGeometryFace`: positions/u_fracs/v_lengths/
   component_kind/component_side/vert_keys/surface_*) не меняется ни
   одним из агентов в одностороннем порядке.
4. **F0 делится.** GPU-агент делает **F0-lite**: интегральные времена
   реального modal-кадра (evaluator total / materialization /
   depsgraph-остаток) измерением СНАРУЖИ evaluator'а + baseline
   до/после F3. Внутренний per-phase профайлер (правки внутри
   `decal_voronoi.py`) — ПОСЛЕ E4, основным исполнителем: фазы
   меняются вместе с M1.
5. **Headless-тесты GPU-кода.** Модуль `gpu` недоступен в
   background — batch-логика (триангуляция, сборка буферов, решение о
   пересборке по topology signature, state-machine lifecycle
   handler'а) тестируется как чистая логика через injected gpu-фасад
   (паттерн существующих bpy-стабов conftest). Реальная отрисовка —
   ручная проверка в Blender.
6. **Preview-статусы.** Overlay обязан уважать A6-контракт: на
   не-UPDATED кадре рисуется последний валидный batch; header-причина
   — как в mesh-пути.

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

Записывать median/p95, не один best run. Дополнительно к фазам —
**три интегральных времени реального modal-кадра** (через MCP-сессию
Blender): evaluator, materialization, depsgraph/redraw остаток. Это
даёт базу для честной оценки эффекта F3 до и после. Baseline-таблица:
walls.003, walls.001, rounded_wall, D-труба 8x3, saddle (fallback).

---

## F1. Python algorithmic/incremental pass — ПОСЛЕ E4 (FP2)

Инкрементальность формулируется в терминах M1-arrangement (это её
естественная форма): интерьерные грани статичны по построению,
между width-кадрами меняются только грани, пересечённые фронтиром.

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

## F2. Native backend decision spike — ПОСЛЕ F1-gate (FP2)

Начинать только если profile показывает, что Python pipeline остаётся
доминирующим ПОСЛЕ E4 и F1.

### Не предполагать заранее 10–50x
Speedup считается только по prototype benchmark.

### Объект порта — M1-конвейер, не legacy clip/subtract

После E4 горячий цикл — snap-rounding сегментов + half-edge
трассировка + point-in-region классификация. Полигональные booleans
из него ИСЧЕЗАЮТ, поэтому прежний кандидат Clipper2 пере-оценивается:
скорее всего он больше не нужен — маленькое собственное ядро
(целочисленное пересечение отрезков + face tracing + классификация)
проще, полностью детерминировано и легче держит bit-parity с
Python-референсом.

### Варианты

1. **Direct pybind11 port M1 semantics** — приоритетный: выше шанс
   bit/differential parity, нет внешних лицензий.
2. **Clipper2/int64** — только если prototype покажет, что своё ядро
   недостаточно; проверить license/redistribution, provenance,
   deterministic ordering, соответствие coverage.

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

## F3. GPU preview overlay — display adapter (можно начинать сейчас, FP1/FP2)

### Контракт

- evaluator остаётся CPU exact; GPU рисует ТОТ ЖЕ выход (FP4);
- во время drag НОЛЬ записей в mesh datablock и ноль depsgraph
  updates — весь preview живёт в draw handler;
- confirm — одна точная BMesh materialization существующим путём;
  cancel/ESC — снятие handler'а, сцена не тронута (контракт A7);
- никакого shader/SDF approximation как authoritative preview;
- headless/background (`bpy.app.background`) — GPU-путь запрещён,
  автоматический fallback на mesh-preview; execute/headless parity
  не меняется.

### Реализация (срезы, по одному коммиту)

**F3.a Batch-строитель.** Триангуляция faces существующим
детерминированным ear clip (concave n-gons); вершины уже лифтованы и
offset'нуты — буферы position/uv/kind-color собираются как flat
POD-массивы; `batch_for_shader` один на surface (или на
component-kind для окраски). Пересборка batch'а ТОЛЬКО при смене
topology signature (существующий механизм A-транша переиспользуется
как критерий); при чистом движении вершин — перезаливка vertex buffer
без пересборки.

**F3.b Draw handler + lifecycle.** `SpaceView3D.draw_handler_add(...,
'WINDOW', 'POST_VIEW')` на invoke; удаление на confirm/cancel/ESC и в
`cancel()` (принудительное закрытие) — ноль утечек handler'ов
(тест: 100 циклов invoke/cancel, счётчик хендлеров стабилен).
`gpu.state`: depth_test LESS_EQUAL, depth_mask off, blend alpha;
z-fighting закрыт существующим физическим offset декали.
`region.tag_redraw()` вместо mesh-write в MOUSEMOVE.

**F3.c Режимы отображения.** v1: SOLID — полупрозрачная заливка по
component_kind (та же палитра, что debug) + POLYLINE-контур фронтира.
v2 (отдельный коммит): TEXTURED — IMAGE-шейдер с trim-текстурой из
активного материала, UV прямо из evaluator'а (это даёт превью
паттерна кладки при drag — то, чего mesh-preview не показывал без
материала). sRGB/color management учесть при биндинге
(`gpu.texture.from_image`).

**F3.d Toggle и совместимость.** `Preview Display: GPU | MESH`
(default GPU, MESH — отладочный fallback и режим для окружений с
проблемным GPU). Persistent-mesh путь НЕ удаляется: он остаётся
confirm-механизмом и fallback'ом.

### Acceptance

1. Во время drag на walls.001: mesh-write counter == 0, depsgraph
   updates == 0; кадр = evaluator + batch upload (замер в F0-таблицу
   до/после).
2. Confirm-выход байт-идентичен pre-F3 (differential): F3 не трогает
   геометрию по построению.
3. 100 циклов invoke/ESC/принудительный cancel — ноль утечек
   handler'ов, ноль артефактов во viewport.
4. Topology-переключение при threshold-drag (band flip) корректно
   пересобирает batch за кадр.
5. Multi-viewport (два 3D-окна) — оба рисуют; закрытие area
  mid-drag не роняет modal.
6. Работает на Blender 4.1 и 4.5 (feature-детекция, без 4.5-only
   API); на 4.5 с Vulkan — прогон вручную, без спец-кода.

### Честная оценка эффекта

Сейчас materialization ~5-15% кадра: чистый выигрыш мс мал. Реальная
ценность — исчезновение depsgraph-статтера и object churn во время
drag (латентная стабильность) и готовая витрина для F1/F2: когда
evaluator упадёт до единиц мс, GPU-кадр останется гладким без
дополнительной работы.

GPU compute для polygon topology не входит в scope без отдельного
доказанного research case; единственный будущий кандидат — SDF hint
overlay, и он явно НЕ планируется.

---

# TRANCHE G — Бэклог фич (после E-acceptance)

## G1. Source-edge imprint на кривых поверхностях (утверждено пользователем)

Продуктовая цель: широкая декаль на кривой поверхности (полусфера от
кромки) получает ребро на каждом пересечённом source-ребре оригинала
(«вершины-близнецы» соединяются) — готовая облегающая сетка. Для
плоских поверхностей поведение не меняется.

Корень: фрагменты уже режутся по domain-треугольникам, но keyed
fragment merge сваривает их через границы, стирая fold-рёбра. На
M1-пути (APPROXIMATE) domain-рёбра — кривые arrangement'а, фича там
фактически есть.

Предписанное правило (не toggle): сваривать фрагменты через общее
source-ребро ТОЛЬКО если два source-ФЕЙСА (provenance
`mesh_tri_face_indices`, не треугольники) копланарны в пределах
epsilon. Диагонали триангуляции квадов свариваются всегда; рёбра
появляются только на реальных изломах оригинала. Копланарный случай
= текущее поведение => плоские differential-фикстуры байт-идентичны
автоматически.

UV не меняются: сплит геометрический, V/U непрерывны через
впечатанные рёбра, новых UV-швов нет.

Acceptance:
1. Фикстура «полусфера, кромка, широкая декаль»: каждое source-ребро
   с dihedral > epsilon, пересечённое полосой, имеет соответствующее
   decal-ребро между twin-вершинами; количество и позиции совпадают
   с пересечёнными рёбрами оригинала.
2. Плоские фикстуры (walls, C7 F1-F2 планарные части) —
   differential байт-в-байт.
3. UV-непрерывность через впечатанные рёбра (u*alpha, v — в пределах
   кванта).
4. S1: impint-рёбра лежат на compile-статичных кривых (границы
   треугольников) — только растут при drag, не переезжают.
5. Решение об опциональном toggle плотности (для game-export
   оптимизации) — отдельно, НЕ в этой задаче.

## G2. Border-рёбра: одностороннее крыло ПО поверхности (design decision)

Полевой репорт: выделенная нижняя кромка полусферы (border, одна
owner-грань) дала линейчатую «юбку» в пустоту вместо крыла по куполу —
цепочки border-рёбер маршрутизируются в boundary/трим-строитель
(планарная логика прототипа), минуя chart-путь целиком.
Требуемое решение (пользовательское): семантика декали на border-
кромке — одностороннее крыло по owner-поверхности через тот же
intrinsic/planar chart-механизм (двусторонний seam-случай с
вырожденной второй стороной); «юбки» остаются осознанной семантикой
ТОЛЬКО режимов Top/Bottom-тримов. Объём: маршрутизация border-runs в
Patch Voronoi как one-sided sites + CAP/торцы по bands-оракулу §7.
После C8.2 (единые ключи) и E-acceptance.

Подтверждено полевым логом (2026-07-17): юбку на Sphere построил
именно legacy-путь — routing-строка
`LEGACY:1c/3Xe | Fallback[...]` при том, что здоровые объекты
(rounded_wall, walls.003) идут `PLANAR+INTRINSIC_DEVELOPABLE`.

## G3. Калибровка E-admission для low-poly (прокси против измерения)

Полевой факт: купол 16 сегментов (~22° шаг) имеет угловой дефект
~0.1 rad на вершину — выше даже max budget 0.10 => REJECT, хотя это
НОРМАЛЬНАЯ игровая плотность, а измеренная ошибка ширины узкой декали
на нём может быть в пределах бюджета. Текущий гейт нарушает дух EP3
(«admission по ИЗМЕРЕННОМУ искажению, прокси — быстрый отсев»):
прокси G3' (defect <= 2*budget) режет ДО измерения. Изменение:
дефект-прокси перестаёт быть отсекающим при наличии measured
width error — отсев только по measurement (+ foldover/overlap binary
gates без изменений); прокси остаётся ранним exit'ом для явных
отказов (defect > жёсткой константы, например 0.5 rad). Acceptance:
купол 16x8, узкая декаль по параллели — ADMIT APPROXIMATE с
width error в бюджете, облегание + G1-imprint работают; E.N1/E.N2
по-прежнему REJECT. Требует E0-замера на грубых фикстурах до правки
порогов (данные до кода — правило E0.b).

Подтверждено полевым логом (2026-07-17): Sphere отвергается с
`Fallback[DISTORTION_BUDGET_EXCEEDED]` (одиночная цепочка), в
двух-патчевом прогоне — `NORMAL_VARIATION_EXCEEDED` +
`PERIODIC_HOLONOMY_UNSUPPORTED`. Ровно предсказанный набор reasons:
режут прокси-гейты ДО измерения ширины.

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
