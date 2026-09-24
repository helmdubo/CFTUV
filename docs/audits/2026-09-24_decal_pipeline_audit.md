# Аудит создания декалей в CFTUV — 2026-09-24

Аудит только на чтение: код не менялся. Объект — всё, что участвует в создании
декалей: production-кнопка «Decal Seams» (legacy), Envelope-ядро (`kernel/`),
хост-адаптер `cftuv/envelope_*.py`, тесты, CI, документация и процесс.
Базовая линия — `main` = `3021146`. Рабочая ветка
`claude/kernel-audit-exact-proof-at8d23` проверена только частично (см. §2).

Обозначения статуса: **[C+]** — подтверждено и независимо перепроверено
координатором (повтор repro или чтение кода); **[C]** — подтверждено треком
(тест, repro или однозначное чтение кода); **[P]** — вероятно, не воспроизведено.

## 1. Главное

1. **`main` не отражает реальное состояние проекта (G-1, G-2 — Critical).** Работа
   над движком декалей продолжалась в ветке `claude/kernel-audit-exact-proof-at8d23`.
   Она опережает `main` на 568 коммитов, не влита, PR не открыт, последняя
   активность — 2026-08-08. На `main` остался отменённый control plane, а документы
   показывают неверный текущий гейт.
2. **Production-кнопка «Decal Seams» нарушает собственный контракт «превью = результат».**
   - L-1: GPU-превью показывает частичный scope, confirm его отклоняет.
   - L-6: превью использует другой UV.
   - L-3: weld выполняется только при confirm.

   Кроме того:
   - L-2: `except Exception` тихо переключает backend при внутренних ошибках.
   - L-3: на узких лентах и мелком масштабе геометрия молча теряется.
   - L-4, L-5: путь для кривых поверхностей (APPROXIMATE/atlas) работает без
     утверждённого гейта. Он роняет весь выбор `RuntimeError`, а на ширине по
     умолчанию падает на сфере.
3. **Envelope-ядро на `main` корректно по архитектуре, но пока только debug.**
   Ownership, UV и `GeometryBatch` не реализованы.
   - K-1: канонический JSON округляет `Decimal` до точности глобального контекста.
     Отсюда коллизии digest и сертификат угла 240°, который не содержит истинного
     значения.
   - F-1…F-3: хост-адаптер отклоняет целые домены на реальной геометрии: швы-«щели»,
     окна в стенах, любой рефлекс-угол, не кратный π рационально.
4. **CI не защищает production-декали.**
   - G-8: на `main` не запускается ни один из ≈390 legacy-тестов декалей;
     `pytest` из корня падает на этапе сбора.
   - G-7: на рабочей ветке все 29 прогонов CI красные, а «приёмки» и
     «стабильная сборка» записаны поверх красного CI.
   - G-6: PR вливаются за 8–26 секунд без ревью.

## 2. Что проверено и чего не хватило

| Трек | Объект | Итог |
|---|---|---|
| A | `decal_voronoi.py` (legacy Patch Voronoi) | завершён |
| B | `decals.py`, rails, geometry | завершён |
| C | intrinsic charts, atlas, CornerModel | завершён |
| D | оператор, modal, session, GPU-превью | завершён |
| E1 | контракты, кодек, валидация и метрики ядра | завершён |
| E2 | reference-вычислитель и interactions ядра | **не завершён** (остановлен) |
| F | хост-адаптер и debug-мост Envelope | завершён (на `main`) |
| G | процесс, документация, CI, артефакты | завершён |
| H/I | рабочая ветка: QUEUE/wavefront-ядро, хост, перепроверка находок | **не завершены** (остановлены) |

Legacy-модули декалей (`decals.py`, `decal_voronoi.py`, `decal_rails.py`,
`decal_rail_geometry.py`, session/modal/GPU) на рабочей ветке **не менялись**
(`git diff --stat` пуст), поэтому раздел 4 действует для обеих линий. Ядро и
хост-адаптер на рабочей ветке переписаны: там +38 500 строк исходников ядра,
новые `wavefront/*`, `robust/*` и `envelope_queue_export.py`. Разделы 5–6 описывают
только `main`.

Ограничения: Blender недоступен (`bpy`/`bmesh`/`mathutils` — заглушки из
`tests/conftest.py`). Поэтому modal, GPU и undo проверены только на уровне Python-логики.

### Прогоны тестов

| Линия | Набор | Результат |
|---|---|---|
| `main` | `kernel/tests` | **224 passed**, 5 мин 14 с |
| `main` | `tests/` (без `tests/blender`) | **527 passed, 2 failed, 3 skipped, 2 xfailed**, 7 мин 06 с; оба падения — тесты `atlas_frozen` (`BEVEL_JOIN_SIDE_AMBIGUOUS`, `CORNER_MATERIAL_VERTEX_OUTSIDE_SEMANTIC_CONTOUR`) |
| `main` | `pytest -q -m "not atlas_frozen"` из корня (документированная команда) | **падает на сборе**: 3 ошибки (`tests/blender` требует Blender, `kernel/tests` без `PYTHONPATH`, `research/s_wf0` требует numpy) |
| `main` | `tools/validate_envelope_ec0.py`, `validate_envelope_legacy_evidence.py`; `kernel/tools` в режиме `--check` | OK (EC0-валидатор печатает устаревший статус гейта) |
| рабочая ветка | `tests/` | **1306 passed, 2 failed, 38 skipped**; оба падения — `FileNotFoundError: 'powershell'` |
| рабочая ветка | `kernel/tests` (3142 теста) | остановлен на ≈25 %, падений до остановки не было |

Сводная серьёзность: **2 Critical**, **13 High**, около 40 Medium, остальное — Low.

## 4. Production-конвейер «Decal Seams» (legacy, одинаков на `main` и на рабочей ветке)

Модули: `decals.py` (2 823), `decal_voronoi.py` (16 889), `decal_rails.py` (2 486),
`decal_rail_geometry.py` (5 583), `decal_charts*.py`/`decal_atlas.py`/`decal_corner_model.py`
и др. (≈5 300), `decal_session.py`, `decal_modal.py`, `decal_gpu_preview.py`,
`decal_transform.py`, оператор `HOTSPOTUV_OT_GenerateDecals` (`operators.py:1483-2173`).
Всего ≈31 400 строк `cftuv/decal_*.py`; на рабочей ветке эти файлы не менялись,
поэтому всё ниже относится к обеим линиям.

### 4.1 Как это реально работает

1. Кнопка «Decal Seams» → `invoke`: архивные режимы `TOP/BOTTOM/CORNERS` отклоняются
   сразу (`decals.py:58-87`), проверяется метрика объекта (non-uniform scale, shear,
   mirror → именованный отказ), захватываются выделенные seam-рёбра, строится
   `AnalysisBundle` по **всем** граням объекта (`operators.py:929-933`).
2. `compile_manual_seam_decal_plan` (`decals.py:1745`): сбор uses по рёбрам
   (2 → пара, 1 → одна сторона, 0/3+ → Failed), R0 rail compile, R1 planar rail
   geometry. Если **весь** scope планарный и без конфликтов — `RAIL_PLANAR`, иначе
   **весь** scope уходит в `compile_patch_voronoi_attempt(allow_partial=True)`
   (Boost segment-Voronoi через `pyvoronoi`, intrinsic charts/atlas для непланарных
   патчей).
3. Modal: drag меняет ширину, на каждый `MOUSEMOVE` — только evaluate (без
   `Construct`), превью по умолчанию — GPU-оверлей (`GPU_TEXTURED`).
4. Confirm: строгая проверка scope/accounting/Failed → временный BMesh →
   валидация → `remove_doubles(0.001)` → подмена mesh у `Decal_Seams_<src>`.

Исходный меш не модифицируется (временные изменения при analysis восстанавливаются
в `finally`), запись транзакционная, IR хранит только индексы.

### 4.2 Находки

| ID | Sev | Суть | Где | Сценарий отказа | Статус |
|---|---|---|---|---|---|
| **L-1** (B-1, D-1, X-1) | **High** | GPU-превью (режим по умолчанию) вычисляет план, игнорируя `plan.rejected_edges` и проверки материализатора; confirm атомарно отклоняет тот же план | `decals.py:2228-2256` против `2302-2367`; `operators.py:1532-1559, 1931-1951` | Выделение с одним неподдержанным ребром (например `PYVORONOI_UNAVAILABLE`, `NON_MANIFOLD_EDGE_USE`): превью показывает частичную ленту, сессия `READY`, на LMB — `Strict SEAMS runtime failed … Failed:1e` и отмена. Противоречит `docs/cftuv_decals.md` («Один Failed атомарно отклоняет весь scope до evaluation») | [C+] |
| **L-2** (B-2, A-4) | **High** | `except Exception` превращает внутренние баги в «штатные» исходы: R0 → `RAIL_COMPILE_INTERNAL_ERROR` → весь scope тихо уходит в Patch Voronoi, RM9 terminal routing теряется; per-patch compile → `SURFACE_COMPILE_EXCEPTION` без traceback | `decal_rails.py:2446-2455`; `decals.py:1797-1798, 922-926`; `decal_voronoi.py:7356-7370` | `KeyError` внутри `compile_decal_rail_plan` → пользователь видит успешный результат, но торцы построены другим backend'ом; причина только в `print`. Нарушает запрет geometry fallback | [C+] |
| **L-3** (A-1, X-2, B-3) | **High** | Допуски arrangement/identity/snap абсолютные (`DECAL_WELD_DISTANCE = 0.001` в локальных единицах, ≈25 мест) при адаптивном квантовании диаграммы → тихая потеря геометрии | `decal_voronoi.py:16687, 7533-7539, 13201-13203, 13525, 13601`; `constants.py:42`; `decals.py:1485-1494` | Ширина 0.0005 (UI-минимум 0.001 world при uniform scale ≥ 2): площадь 0.75 от ожидаемой; патч ×1e-3 → 0.50; ×1e-4 → 0 граней, без ошибки. Weld только в BMesh-пути: превью цело, confirm схлопывает | [C+] (repro повторён) |
| **L-4** (C-1) | **High** | APPROXIMATE-чарты и multi-chart atlas работают в production без утверждённого гейта; `_compile_corner_seeds` вызывается вне per-patch try | `decal_voronoi.py:7408-7418, 2926`; `decal_chart_admission.py:762-770` | Gentle saddle / cliff: `RuntimeError: BEVEL_JOIN_SIDE_AMBIGUOUS` пробивает `allow_partial=True` и отменяет весь SEAMS-invoke. Документы требуют «атлас default-off» и fail-closed `APPROXIMATE_MATERIALIZATION_PENDING` — этого кода нет | [C+] |
| **L-5** (C-2) | **High** | Evaluate APPROXIMATE-чарта падает на ширине по умолчанию | `decal_corner_model.py:554-573`; `decal_voronoi.py:13053` | Sphere cap R=12 (позитивная фикстура E.1): ширины 0.05/0.1/0.3 проходят, 0.15 (default `width_seam`) и 0.5 → `CORNER_MATERIAL_VERTEX_OUTSIDE_SEMANTIC_CONTOUR`; при drag превью мигает. Вероятная причина: допуск трассировки 1e-5 меньше кванта диаграммы 1e-4 | [C] падение; причина [P] |
| L-6 (B-4, D-2) | Medium | Текстурное превью показывает другой UV: `((u+1)/2, v)` против `(0.95+0.05·u, v·uv_length_scale)` при confirm; тест закрепляет превьюшное отображение | `decal_gpu_preview.py:226-231`; `decals.py:1251-1283` | Художник подбирает ширину по картинке, где виден другой участок атласа и другая плотность (при 512/2048 — в 10 раз шире по U и в 4 раза плотнее по V) | [C+] |
| L-7 (B-9, D-5) | Medium | Confirm переиспользует **любой** объект с именем `Decal_Seams_<src>`: подменяет mesh, старый удаляет; маркер владения есть только у preview | `decals.py:1346-1347, 1514-1520, 1605-1617` | Второй confirm по другим швам тихо стирает первую декаль (возврат только через Undo); чужой объект с таким именем теряет mesh; отчёт пишет «Created» | [C+] |
| L-8 (B-5) | Medium | Выбор торца (RR9a «точная ничья → DAM») сравнивает float через `==` | `decal_rails.py:662-732` (также `1433, 1630-1636`; `decal_rail_geometry.py:3372`) | Один и тот же симметричный узел: поворот 0° → SNAP_TIE_DAM, 30° → ROUTE e105, 45° → ROUTE e103; одинаковые модули меша получают разные торцы | [C] |
| L-9 (B-6, A-9) | Medium | Routing нелокален: один «трудный» компонент (branch, острый угол, BEVEL, неплоская грань) переводит **все** компоненты scope в Patch Voronoi; углы реализованы в двух backend'ах по-разному | `decals.py:1809-1821, 1917-1918`; `decal_rail_geometry.py:445, 3379-3392` | Добавление одного ребра в выделение меняет форму углов несвязанных швов | [C] |
| L-10 (B-7) | Medium | Сессия выбирает `CONTROLLED_RECOMPILE` по подстроке `"DOMAIN_BUDGET_EXCEEDED"` в тексте исключения; confirm перекомпилирует и публикует без сверки с последним превью | `decal_session.py:149-150, 177-186, 244-296` | Drag за бюджет → превью пропало → LMB публикует план, которого пользователь не видел | [C] guard отсутствует |
| L-11 (A-3) | Medium | Отброшенные Voronoi-ячейки/грани/atlas-грани только увеличивают счётчики; production не передаёт diagnostics и не проверяет покрытие домена | `decal_voronoi.py:6294-6317, 13201, 16834-16838, 10098-10104` | Point-ячейка с непростым полигоном отбрасывается → дыра в углу без ошибки | [C] код; триггер [P] |
| L-12 (A-2) | Medium | Предусловия Boost проверяются не полностью: пересечение сегментов после квантования не проверяется, отсутствие метода проверки в wheel — молча пропускается | `decal_voronoi.py:6222-6245` | 111 из 1 500 синтетических «щелей» тоньше кванта компилируются без отказа при пересекающихся сайтах; evaluate → 0 граней | [C] синтетика |
| L-13 (A-5, B-8, C-5, C-9) | Medium | Стоимость кадра и compile пропорциональна всему мешу/патчу, а не footprint декали: линейный `locate`, клип по всем треугольникам, SAT-overlap O(n²) дважды, `validate_analysis_bundle` O(P·F) | `decal_voronoi.py:772-800, 13785-13894`; `decal_rail_geometry.py:354-363, 970-994`; `decal_charts.py:1256-1276`; `analysis_surface.py:275-279` | 192 выбранных ребра дуги → 2.6 с на кадр; 40k граней → R1 compile 12.6 с + кадр 164 мс; цилиндр 4 000 треугольников → admission 80 с синхронно в invoke | [C] (синтетика, CPython без Blender) |
| L-14 (C-6) | Medium | Ложный `PERIODIC_HOLONOMY_UNSUPPORTED` на идеальном цилиндре: образы cut-вершины сортируются лексикографически, решает float-шум | `decal_chart_admission.py:421-422, 448-450, 546-551` | 14 из 120 вариантов порядка вершин отвергаются; результат зависит от поворота меша | [C] |
| L-15 (C-7, C-8) | Medium | Непроведённое измерение ширины засчитывается как 0 ошибки; локальные чарты атласа не проходят width/normal/foldover gates вопреки `decal_chart_admission.md:114` | `decal_chart_measurement.py:338-389`; `decal_chart_admission.py:769-770, 792-795`; `decal_atlas.py:361-378` | Crumple при α=1: измерено 3 пробы из 40, admission считает ширину EXACT | [C] механизм; ложный admit [P] |
| L-16 (C-4, F-8) | Medium | RNA `FloatProperty` хранится во float32 (0.10 → 0.100000001490); граничные значения ползунков дают `ValueError` вместо `ChartBuildFailure` и роняют весь compile | `operators.py:258-264`; `decal_chart_admission.py:741-743` | f32(0.10) и f32(0.005) на sphere cap → исключение | [C] (без Blender) |
| L-17 (D-3) | Medium | SEAMS работает только в Edit Mode + Edge Select, но кнопка доступна из Object Mode; отказ приходит после полного analysis с временным unwrap | `operators.py:912-933, 850-889` | Object Mode → долгий анализ → «0 edge(s): SEAMS_REQUIRE_SELECTED_EDGE_PLAN» | [C] |
| L-18 (D-4) | Medium | Split Angle молча режется до `kite_angle` (90°), UI и drag «A» позволяют до 179°; в шапке показывается неэффективное значение | `decal_voronoi.py:314-334`; `operators.py:295-305` | Drag A = 120° → в шапке 120°, в геометрии 90° | [C+] |
| L-19 (D-6) | Medium | Полевая приёмка невоспроизводима: 8 из 17 `artifacts/verify_decal_*.py` требуют внешних сцен (`E:\testscene.blend`, `walls.00x`), 16 JSON содержат `C:\Users\helmd\…`; 2 скрипта устарели и падают | `artifacts/verify_decal_modal_targets.py:45,62,69`; `verify_decal_gpu_preview.py:32-36` и др. | Цифры F0-lite из docs не воспроизвести | [C] (скрипты запускались) |
| L-20 (A-8, B-12, C-3) | Medium | Тесты не ловят L-1…L-5: нет end-to-end happy-path без monkeypatch (107 monkeypatch в `test_decals.py`), нет parity GPU↔BMesh, 11 именованных причин без тестов (включая `PYVORONOI_UNAVAILABLE`), stub `mathutils` без `geometry` → production-триангуляция не исполняется; `atlas_frozen`-xfail скрывает compile-`RuntimeError` | `tests/test_decals.py`; `tests/test_decal_voronoi.py`; `tests/test_decal_atlas.py:240-247, 382, 409-415` | Зелёный набор при сломанном контракте превью/confirm | [C] |
| L-21 (A-6) | Medium | `decal_voronoi.py` — монолит 16 889 строк (54 % всего decal-кода): 7 функций > 400 строк (`_m1_surface_arrangement` — 1 816), ход выполнения зависит от текста исключений, 89 из 125 тестов лезут в 69 private-имён | весь файл; `:8699`, `:13164`, `:15031` | Любое переименование сообщения меняет поведение; декомпозиция ломает десятки тестов | [C] |
| L-22 (D-7, D-8) | Low | Нет try/except в `modal()`; выгрузка аддона во время modal не вызывает `cancel()`; после ошибки отрисовки `_draw` не проверяет `failed` и продолжает рисовать застывший кадр | `operators.py:1993-2137`; `decal_gpu_preview.py:328-336` | Draw-handler и заголовок остаются после исключения; устаревший оверлей поверх mesh-превью | [C+] D-8; D-7 [P] |
| L-23 (D-9…D-13, B-10, B-11, A-7, A-11, C-10…C-13) | Low | Мелочи: пустая коллекция `Decals_Generated` после ESC; выделение сужается до seam-рёбер; имена ID > 63 байт; `mode` по умолчанию `"BOTTOM"` (архив) ломает F3/скрипты; `bl_info` 3.0 при целевой 4.1+; неиспользуемые Corner Width/Trim Height; ≈225 строк мёртвого кода в Voronoi, 4 admission-политики при одной живой; `pyvoronoi` не закреплён по версии; `AnalysisBundle.__getattr__` ломает `copy/deepcopy/pickle` | см. отчёты треков B, C, D | — | [C] |

### 4.3 Что сделано хорошо

- Строгий типизированный план: каждое выбранное ребро учтено ровно один раз как
  `RAIL_PLANAR` / `PATCH_VORONOI` / `Failed`, с проверкой stale-plan и scope mismatch.
- Транзакционная публикация: сначала новый mesh, потом подмена; при ошибке
  осиротевшие данные удаляются; исходный меш не трогается.
- Diagram строится один раз на compile, drag не вызывает `Construct` (есть тест).
- Бит-детерминизм Voronoi-пути подтверждён при разных `PYTHONHASHSEED`.
- `pyvoronoi` изолирован: ленивый импорт, именованный отказ `PYVORONOI_UNAVAILABLE`.
- Метрика объекта проверяется до анализа (non-uniform scale / shear / mirror).
- Быстрые тесты: 138 + 187 + 91 + 72 тестов этих треков проходят за секунды.

## 5. Envelope-ядро на `main` (EC1 → EC2.5 → C-R2C, пакет `cftuv-envelope-core 0.7.0`)

`kernel/` — 17 268 строк исходников, 224 теста (все зелёные, 5 мин 14 с).
Контракты: `AnalysisSnapshotV1` → `DecalRequestV1` → `CompiledPatchEvaluationPlanV1`
(ключ `PlanKeyV1(DecalRequestId, PatchDomainId)`) → точный reference
`RawCoverageResultV1` → EC2.5 `ResolvedCoverageResultV1` (Policy B).
Ownership, UV/station, тесселяция и `GeometryBatch` **не реализованы** — ядро на
`main` не производит геометрию декали, только debug-визуализацию через
Grease Pencil.

### 5.1 Контракты, кодеки, валидация, метрики (трек E1)

| ID | Sev | Суть | Где | Сценарий отказа | Статус |
|---|---|---|---|---|---|
| **K-1** (E1-1) | **High** | Канонический JSON кодирует `Decimal` через `value.normalize()`, который округляет до точности **глобального** контекста `decimal` процесса (28 знаков). Документ утверждает обратное. Хост строит `CertifiedDecimalIntervalV1` через `Decimal(int).scaleb(-28)` с тем же округлением | `kernel/src/cftuv_envelope/codec.py:68-77`; `docs/envelope_ec1_contract_reference.md:123`; `cftuv/envelope_request_export.py:1434-1438` | Два разных значения → одинаковые байты и digest. Интервал для φ/π = 4/3 (угол 240°) после dumps→loads схлопывается в `[1.333…3, 1.333…3]` и **не содержит 4/3**; у хоста верхняя граница < 4/3 уже в памяти. Любой add-on, изменивший `getcontext().prec` в общем интерпретаторе Blender, меняет digest (`prec=4` → `1.23456789` кодируется как `"1.235"`). Сейчас латентно: принятые углы кратны π/4 | [C+] (repro повторён) |
| K-2 (E1-2) | Medium | Валидатор проверяет выбор k (hidden edges) Decimal-арифметикой в контексте, а не `Fraction`, как reference-компилятор | `validation.py:1326-1334, 767-771` против `reference/compile.py:126-150` | При Δ/π ≈ 0.666…667 `upper*3` округляется до `2.000…0` → план с k=1 принимается, хотя должен быть `ANGULAR_PROFILE_SELECTION_UNCERTAIN` | [C+] |
| K-3 (E1-3, E1-4) | Medium | `-0.0` и `0.0` дают разные канонические байты; конструкторы принимают `int`/`bool` вместо float; два разных закона float→exact для одного поля (`as_integer_ratio` в `planar_metric.py:65-67` против `Rational(str(x))` в `reference/validation.py:37-38`, `reference/planar_types.py:33-34`) | `codec.py:111-114`; `numeric.py:30-79` | `GeometryBatchSemanticDigest` зависит от порядка граней; 0.1 = 1/10 в одном домене и 3602879701896397/2⁵⁵ в другом → cross-Patch координация не сойдётся точно | [C] |
| K-4 (E1-5, E1-6, E1-8) | Medium | Валидатор не проверяет ряд AM7-инвариантов: один FrontComponent на обычный ChainUse, кардинальность SEAM_SELF/PHYSICAL_SEAM, `analysis_proven` (нигде не читается), ключ плана у `resolved_coverage`/`geometry_batch_provenance`, общий `shared_semantic_anchor_id` для cross-Patch Junction; у `ChainUseV1` нет поля `side`; флаги `blocks_originating_seed`/`blocks_other_fronts` никто не читает | `contracts/analysis.py:285-290, 321-348`; `validation.py:935-938, 1020-1027, 1248, 1289-1298, 1353-1361` | Синтезированный второй компонент use, SEAM_SELF с одним use, `SOURCE_LAUNCH_BOUNDARY` с `blocks_originating_seed=True`, разные anchors в P05 → 0 issues. Инварианты держит только хост | [C] (repro) |
| K-5 (E1-7) | Medium | Подтверждён долг канона: `FrontComponentV1` хранит alpha/capacity (компилятор всегда пишет `effective=requested`, `NONE`), реальные значения — в `ComponentEffectiveAlphaV1` | `contracts/plan.py:98-101`; `reference/compile.py:495-498` | Digest плана меняется на каждом drag → `COMPILE_CONTRACT_ALPHA_INDEPENDENT=False`, кэш выключен: каждый drag = compile + полная валидация + точный пересчёт | [C] |
| K-6 (E1-9) | Medium | Валидация квадратична (перебор треугольников на каждую грань, линейный поиск вершин) и выполняется при каждом compile каждого домена | `validation.py:528-531, 604-612, 757` | 1 600 граней → 1.1–1.4 с; 6 400 граней → 31 с на домен на каждый drag | [C] (замер) |
| K-7 (E1-10) | Medium | `evaluate_filtered_runtime_raw_coverage` отбрасывает результаты интервального фильтра и всегда делает полный exact; `tracemalloc` включается/выключается без try/finally | `runtime_metric.py:63-91, 125-132`; `planar_metric.py:413, 445-451` | Телеметрия «fast path» ни на что не влияет; в Blender вызов ломает чужую трассировку `tracemalloc`. Сам фильтр корректен (fuzz 20 000 троек, 0 ошибок) | [C] |
| K-8 (E1-11) | Medium | Развёртывание: `sympy==1.14.0` ставится в общий `addons/modules`; инструкция требует ядро 0.5.0 (в коде 0.7.0), хост проверяет версию sympy, но не ядра; CI ядра — только Python 3.10, Blender использует 3.11 | `kernel/pyproject.toml:11`; `docs/envelope_blender_debug_setup.md:17, 69, 158`; `cftuv/envelope_request_export.py:206-226` | Установка по инструкции → `AttributeError` без named outcome | [C] |
| K-9 (E1-12…E1-15) | Low | Кодек принимает дубликаты ключей и неканонические Decimal-строки; `BARRIER_SPLIT_REQUIRED` объявлен в 4 enum; два тавтологичных теста (метаморфный «ретриангуляция не меняет digest» не перекомпилирует план); нет golden-digest и проверки с разными `PYTHONHASHSEED`; `chart_orientation` — first-wins по первой грани | `codec.py:176-182, 252-257`; `kernel/tests/test_metamorphic_contracts.py:107-118`; `planar_metric.py:245-262` | Обновление sympy/Python/кодека может молча поменять все digest'ы | [C] |

Что хорошо: номинальные типы ID, строгий тегированный кодек (лишние/недостающие поля
отвергаются), схемы генерируются из типов и совпадают (`--check` OK), стена импортов
и CI «извлечение ядра в пустой репозиторий», точная аффинная метрика,
`AnalysisSnapshotV1` содержит только факты хоста (56 типов записей, все —
host facts), NaN/Inf отвергаются.

### 5.2 Reference-вычислитель и interactions (трек E2)

Трек не завершён: его остановили, чтобы не расходовать бюджет. Проверено только
следующее. Канон (`01_GLOBAL_CANON.md` §3) сам признаёт два блокера:

- ложный non-manifold отказ при точечных касаниях: вершины сливаются по координате,
  допускается одно исходящее ребро, containment проверяется пробами по вершинам;
- многократная сборка arrangement: каждый envelope клипается отдельно, затем
  всё повторно объединяется.

Независимо это не перепроверялось. На рабочей ветке `reference/*` существенно
переписан (+ `robust/*`, `wavefront/*`), так что по `main` вывод имеет ограниченную ценность.

## 6. Хост-адаптер Envelope и debug-мост (`cftuv/envelope_*.py`, 5 244 строки на `main`)

Поток: `source_revision_from_bmesh` (sha256 от `float.hex` координат/топологии) →
`EnvelopeDebugSessionController` (кэш `AnalysisBundle`) → topology export
(ID `host-v0:<kind>:sha256(...)`, по индексам) → `AnalysisSnapshotV1` на каждый
выбранный домен → compile → Raw → interactions → `build_envelope_debug_scene` →
GP-рендер через `GreasePencilDebugWriter`. 33 теста зелёные (11.8 с), но
`tests/blender/*` в CI не запускается.

Контракт владения соблюдён: snapshot не содержит seeds/fronts/envelopes/alpha;
ремонта геометрии (snapping/welding/допусков) в рабочем пути нет; почти-плоские
патчи дают именованный `RUNTIME_NEAR_PLANAR_PROJECTION_POLICY_REQUIRED`, а не
молчаливую проекцию; ни один `envelope_*` не импортирует legacy; ядро
импортируется лениво (`KERNEL_UNAVAILABLE`, аддон регистрируется без него).

| ID | Sev (`main`) | Суть | Где | Сценарий отказа | Статус |
|---|---|---|---|---|---|
| **F-1** | **High** | SEAM_SELF: разрез-«щель» analysis отдаёт **одной** цепочкой, адаптер требует ровно 2 use → `ENVELOPE_DEBUG_SELF_SEAM_USE_PAIR_UNAVAILABLE`; для кольца с радиальным швом `owner_support` берёт обе грани у ребра → «owner-face directions disagree» | `cftuv/envelope_request_export.py:581-586, 663-665, 1562-1616`; `analysis_boundary_loops.py:113-129` | Весь патч отклоняется даже при выделении дальнего бордюра; фикстура `_self_seam_bundle` синтетическая (одна грань, без углов) | [C+] (repro повторён) |
| **F-2** | **High** | Петля из одной цепочки получает GEOMETRIC-углы с `prev=next=0` (`analysis_corners.py:166-194`); адаптер привязывает их к концам кусков ChainUse и отклоняет весь домен | `envelope_request_export.py:1619-1641` | Стена с окном-вставкой: выделение ребра стены → `EXACT_ANGULAR_CERTIFICATE_UNAVAILABLE` («anchor is not a physical ChainUse endpoint»); при неразбитом окне отклоняются оба домена | [C+] (repro повторён; фикстура построена по формату analysis, живой Blender не запускался) |
| **F-3** | **High** | Каждый рефлекс-угол домена (включая невыделенные) требует φ/π ∈ ℚ (`sympy ... is_Rational`) | `envelope_request_export.py:1425-1431, 1704-1713` | Любой рефлекс-угол общего вида (π + atan 2) и принятая фикстура 270°, повёрнутая на 30° в локальной XY, отклоняют домен: на `main` ядро работает только на осевой/45° геометрии. Контракт ядра допускает интервал ненулевой ширины | [C+] (repro повторён) |
| F-4 | Medium | Адаптер объявляет `TYPED_JUNCTION_ROUTE_TOPOLOGY_V1`, но всегда отдаёт `junction_relations=frozenset()` | `envelope_request_export.py:2354, 2370` | T/X-узлы швов молча становятся Cap'ами на каждой стороне, межпатчевой координации нет | [C] |
| F-5 | Medium | Перехватывается только `EnvelopeHostAdapterError`; любое другое исключение уничтожает результаты всех доменов, Outcome = имя класса | `envelope_request_export.py:2918, 2953-3044`; `envelope_debug_session.py:321-345` | Нарушен закон «ровно один receipt на домен»; вместо `…PARTIAL_CHAIN_SELECTION_UNSUPPORTED` пользователь видит `EnvelopeHostAdapterError` | [C] |
| F-6 | Medium | Debug-оператор синхронный: без modal, прогресса, отмены и бюджета | `operators.py:2231-2415` | `building.002` (24 вершины): 4.7 с на цепочку, 225 с на десять, **870 с** на все швы — Blender висит ≈14.5 мин (почти всё — `RAW_UNION`) | [C] (артефакты R1) |
| F-7 | Medium | Инструменты: gate-скрипт жёстко привязан к `building.002`, по умолчанию перезаписывает закоммиченный артефакт, требует внешнюю `E:\testscene.blend` и идёт не тем путём, что оператор (filtered runtime, alpha 0.3 фиксирована) | `tools/run_envelope_mr1_building_gate.py:74, 164-172, 261-265` | Полевые цифры нельзя воспроизвести из репозитория | [C] |
| F-8 | Low–Medium | alpha: `FloatProperty` (float32) → `Decimal(str(float))` — ни десятичное значение пользователя, ни точное двоичное; ≈345 строк мёртвого `_exact_frame` с `Rational(str(float))`; квадратичная нормализация шва (3 600 патчей → 4.9 с); контроллеры в словаре уровня класса без `load_post` | `operators.py:366-371`; `envelope_request_export.py:1072-1415, 469-482, 2478`; `envelope_debug_session.py:495-525` | 0.2 → 0.20000000298023224 попадает в digest | [C]/[P] |

**Статус на рабочей ветке не проверен** (трек остановлен). Известно только из её
`ROADMAP.md`: блокер F-3 («угловой сертификат требовал рациональной доли π»)
объявлен снятым «по тестам» (новый `cftuv/envelope_angle_certificate.py`), полевой
прогон `building.002` ускорен с 870 с до 24 с (union) и ≈1 с (QUEUE). F-1, F-2 и F-4
по рабочей ветке не проверялись.

## 7. Процесс, документация, CI, артефакты

### 7.1 Истинное состояние (проверено по git и GitHub API)

- `main` = `3021146` (2026-07-24). Последний коммит — «AI execution control pack v1.1»,
  который считает BASE-00 «READY», C-R2C-01…04 «BLOCKED» и ссылается на
  несуществующий `docs/architecture_status.json`. При этом C-R2C (boundary rotation)
  уже влит в `43e69d3` за 20 минут до пакета.
- Рабочая линия — ветка `claude/kernel-audit-exact-proof-at8d23`: merge-base `c2622d0`,
  **+568 / −5** коммитов относительно `main`, последний коммит `e16b93b` от 2026-08-08.
  PR в `main` не открыт. На ней пакет карточек отменён и удалён (`87dc126`,
  2026-07-27), управление переехало в `ROADMAP.md`, `DECISIONS.md` (7 306 строк),
  `ACCEPTANCE.md`, `tests/test_architecture.py`.
- В удалённом репозитории 145 веток; принятые гейты BASE-00/DOC-00/FIX-00 живут
  только в `codex/base-00-canonical-integration` (`architecture_status.json`:
  `D_R2_00_PRODUCT_OWNER_GATE`, `"main_merged": false`).
- С 2026-08-08 (47 дней) активности нет ни в одной ветке.

### 7.2 Находки

| ID | Sev | Суть | Доказательство | Последствие | Статус |
|---|---|---|---|---|---|
| **G-1** | **Critical** | Рабочее состояние движка декалей и все принятые после 2026-07-24 гейты находятся в незащищённых, невлитых ветках; `main` устарел на 568 коммитов | `git rev-list --left-right --count main...claude/kernel-audit-exact-proof-at8d23` → `5 568`; `architecture_status.json` → `"main_merged": false` | Установка/клон с default-ветки получает устаревший код и отменённый control plane; удаление или force-push ветки уничтожит принятое evidence; любая сессия, стартующая с `main` (включая эту), работает не с актуальным кодом | [C+] |
| **G-2** | **Critical** | Control plane на `main` устарел в момент коммита и позже отменён, но на `main` об этом нет ни слова; `build_agent_packet.py --card C-R2C-01` падает (`missing required file: docs/architecture_status.json`); на этот файл ссылаются 40 документов | `docs/agent_execution/envelope_v1/task_manifest.json:8,29,101`; `01_GLOBAL_CANON.md:45-58`; `87dc126` на рабочей ветке | Агент, следующий `OWNER_OPERATING_GUIDE`, повторит BASE-00 или C-R2C поверх уже сделанной работы | [C+] |
| G-3 | High | На `main` нет ни одного документа с верным текущим гейтом; EC0-валидатор печатает захардкоженное `EC1 gate: OPEN_FOR_SEPARATE_SESSION_B_ONLY` | `docs/envelope_engine_start_here.md:109-113` («следующая — Session B/EC1»); `envelope_ec0_correction_log.md:4-6`; `envelope_runtime_r0_acceptance.md:3` (`M_R1_OPEN`); `envelope_blender_debug_setup.md:17` (ядро 0.5.0 при фактических 0.7.0); `tools/validate_envelope_ec0.py:1552` | Решения принимаются по неверному статусу | [C+] |
| G-4 | High | Три разных порядка авторитета (AGENTS.md → AM7-документы; `start_here` §1 → «AM11 wins»; OWNER_GUIDE §3); superseded-документы не помечены (`decal_engine_refactor_brief.md:4` «КАНОН ОНБОРДИНГА», workplan 293 KB, `decal_envelope_roadmap_claude.md`); раскладка модулей в README/AGENTS не знает 15 `decal_*`, 8 `envelope_*` и `kernel/`; README/`bl_info` — Blender 3.0, AGENTS — 4.1+ | `AGENTS.md:46-49`; `README.md:11`; `cftuv/__init__.py:5` | Агент без контекста возьмёт legacy- или AM7-семантику (MITER/BEVEL), отменённую AM11 | [C] |
| G-5 | High | Доска legacy-цели `docs/goals/surface-conforming-decals` формально активна (T043 active, T020/T030/T040/T999 queued, режим «не останавливаться»), не обновлялась с 2026-07-13; её oracle требует TOP/BOTTOM, которые архивированы; 79 путей `C:/…`; ссылается на несуществующий `tests/test_analysis.py` | `state.yaml:63, 1132-1172`; `goal.md:27, 55, 69-71`; `docs/cftuv_decals.md:3-6` | Вызов `/goal` возобновит фичи в замороженном legacy-коде; цель недостижима | [C+] |
| G-6 | High | Ревью и приёмка формальны: PR №8 (52 коммита, 247 файлов, +98 530 строк) влит через 8 с после открытия; №7 — через 9 с; №5 — через 26 с; handoff'ы V0…C-R2C имеют статус READY_FOR_REVIEW, но следующие сессии называют их «accepted»; приёмка Session A выведена агентом из косвенной фразы | GitHub API (PR №5, №7, №8); `docs/session_c_r2{a,b,c}…:7`; `envelope_ec0_correction_log.md:305-309` | Приёмку фактически выводит агент, а не фиксирует пользователь (противоречит `AGENTS.md:139-143`) | [C+] |
| G-7 | High | CI рабочей ветки красный на **всех 29** прогонах 2026-08-02…08; коммиты «приёмка … записана» и «стабильная сборка объявлена» (`7e3c5fa`) сделаны на красном CI | GitHub Actions: runs 31271720234, 30914423477 и др.; локально на рабочей ветке host-suite: 2 failed (`FileNotFoundError: 'powershell'`) / 1306 passed | Зелёный статус декларируется, а не измеряется; герметичность ядра в CI нарушена | [C+] |
| G-8 | High | CI `main` не запускает ни один legacy-тест декалей (≈390 тестов в 23 файлах `test_decal*.py`), `test_operators.py`, `tests/blender/*`; чистый `pytest` из корня падает на сборе (3 ошибки: `tests/blender` без Blender, `kernel/tests` без `PYTHONPATH`, `research/s_wf0` без numpy), без `-m "not atlas_frozen"` — 2 красных теста | `.github/workflows/*.yml`; `pytest.ini`; прогон координатора | Регрессии production-декалей CI не видит. На рабочей ветке это исправлено (`host-suite.yml`, `pytest.ini` с `addopts`/`norecursedirs`) — но она не влита | [C+] |
| G-9 | Medium | Фильтры `paths:` CI не включают `cftuv/model.py`, `surface_ir.py`, `constants.py`, `analysis*.py`, `tests/conftest.py`, хотя job берёт их в sparse-checkout; скан legacy-импортов покрывает 6 из 8 `envelope_*` и 4 точных имени (`from . import decals` проходит); `check_forbidden_imports.py` не видит `kernel/tools` | `.github/workflows/envelope-kernel.yml:4-27, 103-115, 140-161` | Правка analysis молча ломает envelope-хост (см. F-1, F-2) | [C+] |
| G-10 | Medium | Полевые гейты V0…C-R2C и рабочей ветки держатся на внешней `E:\testscene.blend`; локальные пути в 41 JSON/MD; портативная фикстура FIX-00 есть только в своей ветке | handoff C-R2C:91, M-R1:99; `artifacts/envelope_r2c/building_002_point_touch_report.json:112`; `ACCEPTANCE.md` рабочей ветки | Критерий «field fixtures reproducible without a developer-local path» (`00_MASTER_PLAN.md:181`) невыполним | [C] |
| G-11 | Medium | Обязательный минимум handoff (13 полей, `envelope_engine_start_here.md:154-174`) не выполнен в трёх последних handoff'ах (C-R2C, M-R1, V0-R1B: нет Changed paths / Assumptions / Risks / Special opinion / allowlist / digest) | `docs/session_c_r2c_…_handoff.md`; `artifacts/envelope_runtime_r1/session_m_r1_handoff.json` | Следующая сессия не знает границ и рисков | [C] |
| G-12 | Medium | Разделение ролей держится на самодекларации: коммиты `43e69d3`, `e38d140`, `fe091b4`, `d224345` одновременно меняют `kernel/src` и `cftuv/`/`tests/blender`; независимый verifier не записан | `git show --stat` указанных коммитов | Контракт «роли в разных сессиях» не проверяем | [C] факт; независимость [P] |
| G-13 | Medium | В `artifacts/` 85 MB (69 PNG ≈ 76 MB, 2 `.blend`); есть презентационные коллажи (`compose_decal_r19_report.py` клеит PIL-коллаж со шрифтом `C:\Windows\Fonts\arial.ttf` → `decal_r19_before_after_annotated.png`); все добавлены до введения artifact policy (07-19…07-21), политика проверяется только в `envelope_ec0` | `artifacts/*`; `git log --diff-filter=A` | Раздутая история; политика на `main` не исполняется механически | [C] |
| G-14 | Medium | Три конвейера декалей поддерживаются одновременно (Rail, Patch Voronoi, Envelope); cut-over к Envelope описан только в superseded-документе; на рабочей ветке Фаза 3 (паритет) пуста | `operators.py:2690` (кнопка → legacy); `docs/decal_envelope_roadmap_compromise.md:246-257`; `ROADMAP.md` «Фаза 3» | «Каждая новая возможность стоит втрое» (AGENTS рабочей ветки) без срока окончания | [C] |
| G-15 | Low | Гигиена: `Hotspot_UV_v2_5_19.py` в корне байт-в-байт совпадает с копией в `.tmp_review/`; именные worktree в `.gitignore`; `version_info` отстаёт на 201 коммит; AGENTS.md описывает маркеры `ARCHITECTURAL_DEBT`, а в `.py` их **ноль** (ни одна из известных decal/envelope-задолженностей не внесена в `docs/architectural_debt.md`) | `grep -rn ARCHITECTURAL_DEBT --include=*.py .` → пусто | Ложные инварианты, шум | [C+] |

### 7.3 Сильные стороны процесса

- Сильные семантические контракты: именованные исходы, точная арифметика, hash-манифесты фикстур.
- Быстрый детерминированный EC0-валидатор (23 кейса), герметичная сборка ядра и
  job «извлечение ядра в пустой репозиторий».
- Handoff'ы Session A–D образцовые; полевое evidence содержит отпечатки меша.
- Рабочая ветка сама исправляет часть проблем `main`: исполняемые архитектурные
  тесты, храповики на рост документов/бинарников, host-suite в CI, чистый
  `pytest.ini`, удаление мёртвых Hotspot-копий и `band_operator.py`.

## 8. Рекомендации по приоритету

**P0 — сначала**

1. Выбрать каноническую линию и довести её до `main`. Рабочая ветка плюс принятые
   BASE/DOC/FIX-00 — после зелёного CI. Включить защиту `main` с обязательными
   проверками. До чистки 145 веток поставить теги на принятые SHA (G-1, G-2).
2. Вернуть зелёный CI на рабочей ветке:
   - перенести данные тестов ядра в `kernel/fixtures`;
   - пометить powershell-тесты как Windows-only;
   - запретить коммиты «приёмки» на красном SHA (G-7).
3. Починить контракт production SEAMS (L-1…L-5):
   - одна strict-проверка (scope, accounting, `rejected_edges`, валидация граней)
     для GPU- и BMesh-адаптеров;
   - общий UV-маппинг и одинаковый weld в обоих путях;
   - никаких reroute на внутреннем исключении: только именованный отказ с причиной
     в summary;
   - `_compile_corner_seeds` и admission оборачивать per-patch;
   - APPROXIMATE/atlas закрыть fail-closed до приёмки;
   - допуски выводить из `DiagramTransform.quantum` и alpha, либо давать
     именованный отказ, когда ширина меньше k·tol.

**P1**

4. Ядро:
   - `Decimal` кодировать без контекста, валидаторы перевести на `Fraction`;
   - нормализовать `-0.0`, оставить один закон float→exact;
   - хост строит интервалы без `scaleb`-округления (K-1…K-3);
   - добавить проверки AM7 в валидатор (K-4);
   - разделить статический план и alpha-состояние, линеаризовать валидацию (K-5, K-6).
5. Хост-адаптер (сначала проверить, что уже исправлено на рабочей ветке):
   - SEAM_SELF-«щель» и петли из одной цепочки;
   - сертификаты для нерациональных углов;
   - изоляция ошибок по доменам;
   - проверка версии ядра;
   - `JunctionRelation` — экспортировать либо не объявлять (F-1…F-5).
6. CI и тесты:
   - host-suite с legacy-тестами на каждой ветке;
   - расширить `paths` и скан legacy-импортов;
   - end-to-end тест без monkeypatch и тест паритета GPU↔BMesh;
   - тест на каждую именованную причину (G-8, G-9, L-20).
7. Документы:
   - один машиночитаемый файл статуса;
   - пометить superseded (control pack, `start_here` §5, AM7-roadmap, legacy
     workplan и brief);
   - архивировать доску `surface-conforming-decals`;
   - исправить README и AGENTS: Blender 4.1+, карта модулей, `kernel/` (G-2…G-5).
8. Приёмка:
   - дословная фраза пользователя, SHA и id прогона CI;
   - портативная фикстура в репозитории вместо `E:\testscene.blend` (G-6, G-10).

**P2**

9. Производительность:
   - spatial index в `locate` и в клипе угловых полос;
   - broad-phase для SAT-overlap;
   - индексы faces по patch в `PatchSurfaceIR`;
   - analysis только по патчам рядом с выделением (L-13).
10. Гигиена:
    - удалить мёртвый код (≈225 строк в Voronoi, 4 admission-политики, `_exact_frame`);
    - поставить маркеры `ARCHITECTURAL_DEBT` и завести записи в реестре;
    - вынести PNG/blend из истории (LFS);
    - закрепить версию `pyvoronoi`;
    - `mode` по умолчанию — `SEAMS`.
11. Назначить срок и критерии Фазы 4 (удаление legacy), чтобы прекратить тройное
    сопровождение (G-14). Пока legacy живёт, чинить в нём только L-1…L-5.

## 9. Что не сделано в этом аудите

- Трек E2 (reference/arrangement/interactions на `main`) не завершён.
- Рабочая ветка проверена только на уровне истории, CI и host-тестов. Её
  QUEUE/wavefront-ядро (≈38 500 строк) и статус находок F-1…F-4 и K-1…K-8 на ней
  не аудировались.
- В реальном Blender ничего не запускалось.

Если нужно продолжение, разумный следующий шаг — один узкий трек: «рабочая ветка:
какие находки из этого отчёта там ещё живы».
