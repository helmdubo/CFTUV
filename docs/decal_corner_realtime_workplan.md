# План работ: Runtime Corner Correctness + Live Corner Preview

Статус: согласованный план после двух независимых аудитов (внутреннего и
внешнего). Цель — перестраивание corner-геометрии на лету во время modal
drag (как в референсном инструменте), поверх существующего
compile/evaluate контракта Patch Voronoi backend.

Документ самодостаточен: исполнителю не требуется история обсуждения.
Перед началом прочитать `AGENTS.md`, `docs/cftuv_decal_runtime.md`,
`docs/cftuv_decal_nonplanar_roadmap.md`.

---

## Архитектурный контекст (что уже готово и не переделывается)

- `compile_patch_voronoi_plan()` ширино- и policy-независим; PyVoronoi
  решается один раз на modal invoke.
- `CornerSpec` (`cftuv/decal_voronoi.py:76`) хранит только статические
  факты угла; policy выбирается на каждом кадре в
  `classify_corner_runtime()` (`decal_voronoi.py:1847`) из
  `CornerRuntimeSettings`.
- Persistent preview: один object + один mesh datablock на весь modal,
  topology-signature fast path (`decals.py:2224-2301`).
- Замеры: evaluator ~22 ms на 27 edges, ~104 ms на 133 edges
  (`docs/cftuv_decal_runtime.md`). Узкое место — Python
  polygon-конвейер evaluator'а, НЕ materialization.

## Глобальные ограничения (guardrails, действуют на всех этапах)

1. `pyvoronoi.Construct()` никогда не вызывается во время drag.
   Compile зависит только от selected edges, PatchGraph, owner charts,
   offset. Corner policy остаётся строго runtime-слоем.
2. Preview и confirm используют один evaluator с одинаковыми
   параметрами (execution parity, `docs/cftuv_decal_runtime.md`).
3. Каждое изменение evaluator'а проверяется differential-прогоном на
   fixtures `walls.003` / `walls.001` при ширинах
   `2.0 / 3.0 / 3.7076 / 4.5`. Вывод обязан совпасть байт-в-байт,
   КРОМЕ мест, где изменение вывода — заявленная цель этапа; тогда
   фиксируется новый эталон и запись в docs.
4. Тесты обязаны проходить без Blender (стабы `tests/conftest.py`;
   pyvoronoi-тесты через `importorskip`).
5. Комментарии в коде — русские (конвенция проекта).
6. Четыре band-порога НЕ добавляются до этапа P2. Перенос на C++ НЕ
   начинается до профилирования после P3.
7. Каждый этап завершается: зелёный pytest, differential-прогон,
   обновление `docs/cftuv_decal_runtime.md` / `docs/cftuv_decals.md`
   при изменении контракта, коммит с описанием.

---

## P0 — Корректность (до любого нового UI)

Смысл этапа: новые слайдеры не должны делать существующие некорректные
состояния легко достижимыми. Пункты P0.1–P0.5 — geometry/compile,
P0.6–P0.8 — modal, P0.9 — units-контракт.

### P0.1 Честная семантика Miter Limit; судьба BEVEL

Факты: ветка BEVEL в `classify_corner_runtime`
(`decal_voronoi.py:1869-1871`) недостижима для обычных 2-site углов
(после `is_convex → MITER` и `interior > pi → KITE` случаев не
остаётся). `settings.miter_limit` читается только в
`_corner_crop_polygon` (`:1291`); KITE хардкодит `alpha * 8.0`
(`:1103`); ACUTE_SPLIT не ограничивает внешний apex вовсе. Эмпирически
`miter_limit=1.0` и `100.0` дают идентичный вывод на всех текущих
fixtures — но это зафиксировано экспериментом, не регрессией.

Принятое решение — вариант «геометрический clamp»:

- `miter_limit` — apex clamp, не policy. Недостижимая policy
  `BEVEL` удаляется из `classify_corner_runtime` и из `_CornerPolicy`
  (тесты BEVEL не используют — проверено; упоминания в
  `docs/cftuv_decal_nonplanar_roadmap.md:27-28` обновить).
- UI label переименовать в `Apex Limit` (property id
  `decal_corner_miter_limit` сохранить для совместимости настроек).
- Пробросить settings в `_kite_crop_polygon` (`:1089-1103`) вместо
  хардкода `8.0`; добавить аналогичный clamp внешнего apex в
  `_acute_crop_components` (`:1154`).
- `component_kind` продолжает отражать policy; факт clamp'а — не
  отдельный kind. Обновить описание в
  `docs/cftuv_decal_runtime.md:24-26` и docstring property
  (`operators.py:210-218`).
- Полноценный bevel/fillet band, если он нужен как отдельная
  обработка, — предмет P2 (после фиксации band-семантики), не P0.

Acceptance:
- Новый full-evaluator регрессионный тест: один compiled plan,
  `miter_limit = 1 / 8 / 100` → заранее определённые РАЗНЫЕ контуры
  на двух fixtures: острый выпуклый угол (длинный miter) и reflex
  угол (KITE). Сейчас настройку не проверяет ни один тест.
- Differential на стандартных fixtures: изменения только у углов с
  apex дальше лимита.

### P0.2 Микро-рёбра и локализация compile-ошибок

Факты: quantum = `max(1e-5, DECAL_WELD_DISTANCE*0.1) = 1e-4 BU`
(`decal_voronoi.py:1430-1435`); входной фильтр — только сырые
`length > 1e-9` (`:1662`). Ребро между 1e-9 и ~1e-4 схлопывается в
нулевой quantized site → `ZeroDivisionError` в `_corner_offset_lines`
(`:1073-1079`) и/или degenerate segment в `AddSegment` (`:2029`).
`compile_patch_voronoi_attempt` не оборачивает `_compile_surface` —
исключение отменяет весь оператор вместо component-fallback.

Изменения:
- Инвариант после квантизации: `quantized_length > 0`. Ребро,
  потерявшее site, попадает в rejected с причиной
  `QUANTIZED_DEGENERATE_SITE` (обычный component-fallback routing).
- Guard деления в `_corner_offset_lines`.
- Обернуть `_compile_surface` в try/except →
  `PatchVoronoiCompileFailure` с контекстом (patch, edge count).
- Постусловия compile: каждый accepted edge создал >= 1 валидный
  site; каждый site создал segment atom; потеря SEGMENT cell =
  compile failure этого component. Отсутствие endpoint point-cell —
  допустимо (легитимный вырожденный случай Boost).

Acceptance: fixture с ребром 5e-5 BU → component уходит в legacy
routing без исключения, отчёт показывает причину; тест на потерю
segment atom → `PatchVoronoiCompileFailure`, не Python exception.

### P0.3 Точный учёт selected edges

Факты: ребро без BoundaryChain use не создаёт run
(`decals.py:779-848`), но `_manual_seam_edge_components`
(`:851-898`) всё равно делает его singleton component, и оно попадает
в accepted count отчёта, не породив геометрии. Non-manifold ребро:
берутся `uses[0]/uses[1]`, третья сторона молча отбрасывается
(`:822-825`).

Изменения:
- Инвариант: `selected = accepted ∪ rejected`, дизъюнктно; каждый
  accepted edge имеет доказанный owner-side use.
- Ребро без use → rejected с причиной `NO_BOUNDARY_CHAIN_USE`.
- `len(uses) >= 3` → rejected с причиной `NON_MANIFOLD_EDGE_USE`
  (не first-two-wins).
- Routing report (`Patch Voronoi:Nc/Ne | Legacy:...`) считает только
  фактически скомпилированные рёбра; rejected-причины видны в report
  или console.

Acceptance: тесты на wire/dangling seam edge и non-manifold fixture —
ребро видно в rejected, счётчики отчёта сходятся с фактической
геометрией.

### P0.4 Ownership перекрывающихся соседних corner crops

Факты: оба corner'а короткого сегмента включают его atoms в свои
`owner_atoms` (`decal_voronoi.py:2992-3003`); corner crops вычитаются
из SEGMENT faces (`:3030-3050`), но не друг из друга. При
`width > длина сегмента` область `strip ∩ crop_A ∩ crop_B`
материализуется дважды (дубли/z-fighting). Для live thresholds это
критично: переключение `MITER → ACUTE_SPLIT` резко расширяет corner
region прямо во время drag.

Изменения:
- Порядко-независимое правило владения: divider вдоль общего
  сегмента по равному расстоянию до двух endpoints (полуплоскость
  через середину сегмента). Каждый corner crop дополнительно
  клипается своей половиной ТОЛЬКО при фактическом пересечении
  crops (короткий сегмент), чтобы не менять вывод там, где overlap
  невозможен.
- Запрещено решать задачу порядковым boolean subtraction
  (результат зависел бы от индекса угла).

Acceptance: fixture «короткое ребро между двумя углами», width >
длины ребра → нет пересекающихся/дублирующихся faces (проверка по
quantized face keys + суммарной площади); differential на стандартных
fixtures — байт-в-байт (там overlap не достигается).

### P0.5 Fail-fast materialization

Факты: `_materialize_network_faces` считает невалидные faces в
`dropped`, печатает в console и продолжает (`decals.py:1948-1986`).

Изменения: `dropped > 0` ⇒ preview-кадр невалиден (см. P0.6,
`RETAINED_LAST_VALID`); на confirm — явная ошибка без замены
последнего валидного объекта. BMesh-транзакция догоняет уже
существующую математическую транзакцию partitions.

### P0.6 Структурированный результат preview; контракт настроек

Факты: guard `if not created` мёртв — при пустом кадре
`_finalize_decal_object` возвращает старый объект
(`decals.py:2337-2348`), а `_modal_current_settings = settings`
присваивается ДО проверки (`operators.py:1438-1449`). Невалидная
ширина становится финальной; confirm может завершиться FINISHED со
старым объектом. `_modal_last_preview_error` пишется, но нигде не
показывается. Это нарушает контракт
`docs/cftuv_decal_runtime.md:46-47`.

Изменения:
- Ввести структурированный статус кадра:
  `PreviewStatus = UPDATED | RETAINED_LAST_VALID | EMPTY | ERROR`.
- Только `UPDATED` имеет право менять `_modal_current_settings`,
  scene property и `_modal_current_value`.
- Header при не-UPDATED кадре показывает причину, например:
  `Invalid preview — keeping Width 3.7076 (QUANTIZED_DEGENERATE_SITE: edge 154)`.
- Confirm проверяет результат `_generate`: пустой → report ERROR,
  последний валидный объект и настройки остаются.

Acceptance: тест — серия кадров с пустым результатом не двигает
настройки; confirm после невалидных кадров использует последние
валидные значения; header-текст содержит причину (проверка через
стабы).

### P0.7 Безопасный cancel и защита пользовательского объекта

Факты: у оператора нет `cancel()` (принудительное завершение modal
оставляет header-текст и mid-drag значения); ESC мутирует сцену и
возвращает CANCELLED (ломает undo); существующий
`Decal_Seams_<obj>` уничтожается уже на invoke — ESC не
восстанавливает identity/материалы/ручные правки
(`decals.py:2401-2410`, `operators.py:1452-1468`).

Изменения:
- Preview строится во временном объекте (отдельное имя/коллекция);
  production-объект заменяется ТОЛЬКО на confirm одной точной
  транзакцией.
- ESC: удалить временный объект, восстановить scene property,
  production-объект не тронут.
- Добавить метод `cancel(self, context)`: очистка header, удаление
  временного объекта, восстановление property.

Acceptance: тест на ESC — существующий объект (его mesh pointer и
custom props в стабах) не изменился; обновить
`docs/cftuv_decal_runtime.md` (persistent preview → temporary object,
swap на confirm).

### P0.8 Shift rebase

Факты: значение всегда считается от исходного `base_value` и полного
накопленного delta; Shift умножает чувствительность на 0.1 для всей
истории (`operators.py:1413-1417`, `decal_modal.py:11-15`) — значение
скачет при нажатии/отпускании. Для порогов скачок перепрыгивает
несколько topology-band'ов сразу.

Изменения: rebase при смене modifier state (и при смене drag target,
см. P1.1): `base_value = current_value; start_mouse = current_mouse`.

### P0.9 Контракт единиц (world vs local)

Факты: настройки документированы как мировые единицы, но геометрия
считается в local space, а декаль получает `matrix_world` источника
(`decals.py:2382, 2391, 2421`). При scale 2.0 физическая ширина
удваивается; при non-uniform scale зависит от направления.

Изменения (минимальный производственный контракт):
- Детект non-identity scale у source object; uniform scale —
  компенсировать (width/height/offset делить на scale, чтобы
  мировые единицы соблюдались); non-uniform scale — явный warning в
  report и header (поведение задокументировать).

Acceptance: тест с uniform scale 2.0 → фактическая мировая ширина
равна настройке; non-uniform → warning.

---

## P1 — Live Acute Split Angle (первый срез «как на видео»)

Пред-условие: P0 полностью. Срез ограничен ОДНИМ рабочим порогом —
`Acute Split Angle` уже имеет честную runtime-семантику и тест
переклассификации без рекомпиляции (`tests/test_decal_voronoi.py:627`).

### P1.1 Operator-owned drag targets

- Таблица целей вместо одного жёсткого поля
  (`operators.py:1384-1397`):
  `W` — Width (текущее поле по mode), `A` — Acute Split Angle,
  `M` — Apex Limit (после P0.1).
- Переключение клавишей → rebase (механика P0.8).
- Per-target sensitivity / clamp / формат header в
  `decal_modal.py`: текущий пол `DECAL_SIZE_MIN=0.001` не годится
  для углов — угловая цель клампится в
  `[pi/180, 179*pi/180]` (как FloatProperty), Apex Limit — `>= 1.0`;
  углы в header — в градусах.
- НЕ использовать `update=` колбэки scene properties: колбэк не
  владеет compiled plan и preview state; вся логика — в modal.

### P1.2 Gate для legacy-партиций

Факты: legacy backend не принимает corner settings
(`decal_network.py:2167`), использует константу
`DECAL_CORNER_MITER_LIMIT = 4.0` (`constants.py:53`); гибридное
выделение реагировало бы на пороги только частью сети.

Изменения: если routing содержит `Legacy > 0` components — corner
targets отключены, либо header явно показывает:
`Corner controls affect Patch Voronoi components only`. Молчаливое
частичное применение запрещено.

### P1.3 Policy breakdown в header

Во время corner drag header показывает распределение политик,
например `MITER:12 KITE:3 SPLIT:2` — мгновенная обратная связь о
пересечении порога.

### P1.4 Тесты среза

- Modal-цикл с РЕАЛЬНЫМ evaluator (не мок) на маленькой fixture:
  drag порога через значение угла → policy переключается
  `MITER ↔ ACUTE_SPLIT`, topology signature меняется, object/mesh
  identity сохраняются, переход `1 ↔ 2 components` не оставляет
  старых faces.
- Счётчик `Construct()` за время drag равен нулю (обёртка в тесте).
- Кадр с пустым результатом на пороге → `RETAINED_LAST_VALID`.

### P1.5 Тест-гигиена (сопутствующее)

- Моки `evaluate_patch_voronoi_plan` в `tests/test_decals.py:153-161`
  должны ассертить проброс `corner_settings` (сейчас его удаление
  не ловится ни одним тестом).
- Gap-тест (`tests/test_decal_voronoi.py:741`) дополнить проверкой
  отсутствия overreach за пределы alpha (ловит axis-aligned CAP
  дефект до его исправления в P2.3 — как xfail/known).

Definition of done P1 — поведение как в референсном видео для чистых
Patch-Voronoi выделений: движение мыши → порог пересекает угол →
`MITER ↔ ACUTE_SPLIT` → topology перестраивается в том же persistent
mesh, без создания/удаления объектов, с явной обратной связью и без
фиктивных настроек.

---

## P2 — Четыре band'а и паритет с референсом

Пред-условие P2.0 (обязательное): зафиксировать семантику band'ов
отдельным документом `docs/decal_corner_bands.md` ДО кода. Слайды
референса дают имена/значения (`obtuse_thres1 ~120°`,
`obtuse_thres2 ~90°`, `acute_thres1 ~90°`, `acute_thres2 ~60°`), но
не отвечают на:
- какой угол сравнивается (interior / extrusion / signed turn);
- какой тип компонента соответствует каждому диапазону;
- меняется ли только topology или также UV orientation;
- поведение ровно на границе диапазона (`>=` vs `>`);
- это 4 независимых band'а или 2 hysteresis-пары.

Документ обязан содержать таблицу контрольных углов (например 170°,
130°, 100°, 75°, 45°) с ожидаемой политикой и эскизом контура для
каждого. Только после этого:

- P2.1 `CornerRuntimeSettings`: ordered thresholds, нормализация
  (сортировка/кламп при нарушении порядка), опциональный hysteresis
  для anti-flicker при drag (вход в band при `< t`, выход при
  `> t + delta`).
- P2.2 Новые runtime crop builders для band-обработок (обтузные
  варианты, второй acute band; настоящий BEVEL band, если семантика
  подтвердит его необходимость).
- P2.3 Tangent-aligned CAP и угловой valence-N JUNCTION вместо
  axis-aligned квадратов (`decal_voronoi.py:1270-1276`): форма торца
  не должна зависеть от ориентации меша в мире; policy незанятых
  reflex-секторов; UV anchors для junction-секторов.
- P2.4 UV-модель corner cells: V-непрерывность между chain
  fragments одного branch, корректный `uv_sign` для two-sided
  same-chart sites, детерминированный UV owner для KITE/MITER
  компонентов.
- P2.5 Legacy: либо проброс порогов в `evaluate_seam_network_plan`,
  либо формализация gate из P1.2 как постоянного решения. Preview
  LOD legacy (`HARD` vs `LOW`, `decal_network.py:2216`) — либо
  выровнять, либо явно задокументировать как осознанный.
- P2.6 UI: четыре FloatProperty + панель + modal targets `1/2/3/4`;
  один helper `DecalSettings → CornerRuntimeSettings` вместо двух
  дублирующихся конструкторов (`decals.py:2575, 2675`).

---

## P3 — Incremental evaluation (профилирование до C++)

Сегодня изменение порога стоит столько же, сколько изменение ширины:
evaluator заново строит crops, merge, arrangement и BMesh для всего
выделения. Инкрементальность — отдельный слой:

- P3.1 `corner → owner_atom_indices` в compiled plan (убирает
  O(corners × atoms) скан `decal_voronoi.py:2992-3003`); аналогично
  индекс для junction ports (`:2801-2815`).
- P3.2 Кэш runtime components неизменившихся corners: dirty =
  corners, чей policy band изменился (corners заранее отсортированы
  по `extrusion_angle`; при смене порога t0→t1 dirty — только углы в
  интервале), плюс их incident sites.
- P3.3 Surface-local arrangement rebuild (пересборка только dirty
  surface); coalescing MOUSEMOVE событий.
- P3.4 Кэш ширино-независимых merge внутренних (некропаемых)
  фрагментов cell'ов.
- P3.5 Профилирование на `walls.001` и более крупной fixture.
  Цель: <= 33 ms на 133 edges. По остаточным hot spots — отдельное
  решение о нативном бэкенде (Clipper2 либо pybind11-порт текущих
  алгоритмов; переносить весь evaluate-цикл одним вызовом, Python
  остаётся документированным fallback). C++ не начинать до этого
  профилирования.

---

## Вне очереди (отдельные треки, не блокируют P0–P3)

- Адаптивная `DiagramTransform(center, scale, quantum)`:
  квантизация и guard относительно extent patch'а вместо абсолютных
  `_DIAGRAM_SCALE=100000` / quantum 1e-4 BU (`decal_voronoi.py:42,
  1430-1435, 2012-2018`). Закрывает и микро-геометрию, и int32-риск
  Boost на крупных patches. Обязательна до непланарных чартов
  (длинные развёртки tubes).
- Развязка импорта приватных `_NetworkFace`, `_lift_position` и др.
  из `decal_network.py` в `decal_voronoi.py:29-34`.
- Docs-гигиена: `docs/architectural_debt.md` ссылается на
  несуществующую функцию `_derive_junction_disk_cycle` и inline
  `ARCHITECTURAL_DEBT` маркеры, которых в `.py` нет (проверено
  grep'ом); таблица настроек в `docs/cftuv_decals.md:250` не
  включает corner-свойства и `decal_seam_network`.
- Диагностические счётчики тихих fallback'ов compile
  (`_cell_polygon` disorder, tessellation fallback,
  `_convex_fragment_decomposition`) — вывод в verbose console.

---

## Протокол верификации (каждый этап)

1. `pytest tests/ -q` без Blender — весь suite зелёный (pyvoronoi
   установлен; иначе voronoi-тесты скипаются — это допустимо только
   локально, не для приёмки).
2. Differential: `walls.003` / `walls.001`, ширины
   `2.0 / 3.0 / 3.7076 / 4.5` — вывод байт-в-байт, кроме заявленных
   изменений (новый эталон + запись в docs).
3. Blender ручной чек: modal drag (header, переключение targets,
   Shift), ESC, confirm, принудительное закрытие окна (cancel);
   object/mesh identity стабильны; `Decals_Generated` без мусора.
4. Замер frame time на `walls.003` и `walls.001` до/после — в
   `docs/cftuv_decal_runtime.md`.
