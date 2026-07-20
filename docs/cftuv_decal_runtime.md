# Decal runtime contract

## Deterministic benchmark harness

`artifacts/verify_decal_runtime.py` запускается внутри Blender и сохраняет
JSON с PatchGraph/compile/evaluate/materialization timings, mesh/drop counts,
runtime policy counts, backend routing и именованными fallback counters.
`_NetworkFace` сериализуется детерминированно: face order и cyclic loop start
канонизируются, positions и UV-факты округляются, topology signature и SHA-256
geometry digest сохраняются для differential.

Harness выполняет repeated preview и exact confirm для ширин
`2.0 / 3.0 / 3.7076 / 4.5`. Он явно проверяет, что geometry snapshots
совпадают между повторами и preview/confirm, а число `Construct()` после
compile не меняется во время width drag.

Baseline Blender 4.3.2 на включённой fixture
`VERIFY_SRC_MANUAL_SEAMS` (25 selected edges): compile `162.82 ms`, median
preview evaluator `25.27 / 23.74 / 23.61 / 22.99 ms`; materialization
`1.05 / 0.72 / 0.71 / 0.71 ms`. Во всех четырёх ширинах dropped faces = 0,
repeated output и preview/confirm совпали, `Construct during drag = 0`.
Полный JSON лежит в `artifacts/decal_runtime_baseline.json`.

Production fixtures `walls.003` и `walls.001` не входят в репозиторий. Harness
по умолчанию запрашивает именно их и записывает отсутствующие имена в
`missing_objects`; production baseline нужно снять тем же скриптом в исходном
studio `.blend`, не подменяя его verification fixture.

## Static compile

`compile_patch_voronoi_plan()` зависит только от выбранных seam edges,
PatchGraph, owner surface charts и offset. Он один раз сохраняет:

- segment/endpoint Voronoi cells;
- boundary-clipped atoms;
- `CornerSpec` с геометрическими фактами угла;
- surface lift provenance.

Ширина и corner policy не входят в compiled plan.

## Dynamic evaluation

`evaluate_patch_voronoi_plan()` получает текущую ширину и
`CornerRuntimeSettings`. На каждом кадре он дешёво выбирает policy из
`interior_angle`, `extrusion_angle`, `is_convex` и `miter_ratio`, после чего
строит runtime crops и пересекает их с уже скомпилированными cells.

Сейчас доступны три параметра, семантика которых уже определена backend:

- `Acute Split Angle` — ниже порога corner становится двухкомпонентным
  `ACUTE_SPLIT`;
- `Apex Limit` — максимальное расстояние удалённого apex относительно
  half-width для `MITER`, `KITE` и outer-части `ACUTE_SPLIT`.
- `Corner Join` — `MITER` сохраняет обычный выпуклый apex, `BEVEL`
  меняет только заполнение GAP-стороны углового piece на прямой
  треугольник `(V,P1,P2)`. Arrangement, crops, limits и collision
  ownership идут тем же путём, что при `MITER`.

Blender property id `decal_corner_miter_limit` сохранён для совместимости, но
UI и runtime называют параметр `Apex Limit`; внутренний контракт —
`CornerRuntimeSettings.apex_limit`. Arrangement-классификатор никогда не
возвращает `_CornerPolicy.BEVEL`: emission-классификатор выбирает его только
для GAP между двумя уже эмитированными strip-квадами. Базовый MITER/KITE
corner-piece после общего arrangement заменяется треугольником из его же
post-clip `V/P1/P2`; segment faces и их station-UV остаются бит-идентичны
режиму MITER. Дальний MITER/KITE apex заменяет
усечённый contour. ACUTE_SPLIT всегда сохраняет INNER и OUTER components:
outer apex двигается вдоль исходного apex ray и остаётся строго с внешней
стороны cap chord. Если limit меньше геометрического minimum, используется
minimum + epsilon и увеличивается `apex_limit_saturated_count`.

Optional `PatchVoronoiDiagnostics` содержит `clamped_miter_count`,
`clamped_kite_count`, `clamped_acute_count` и
`apex_limit_saturated_count`, а также counted причины сохранённых CAP в
`cap_keep_counts` (`CAP_KEEP_<REASON>`); изменение Apex Limit не вызывает новый
`Pyvoronoi.Construct()`.

CAP сохраняет геометрическое закрытие, но при ровно одном semantic
SEGMENT-соседе той же Patch-surface принимает его station-UV и становится
частью strip. Совпадение normals не требуется: fold не является UV seam.
Несколько общих рёбер образуют одну adjacency; общий UV канонизируется по
shared vertex keys без числового допуска. Неоднозначность сохраняет CAP и
обязана появиться в `cap_keep_counts`.

Terminal boundary contact читает физический contour, а не границы
производных `BoundaryChain`: если chain закончилась в вершине с ровно одним
непройденным seam/border-продолжением, тот же station route продолжается за
угол. Физический конец сохраняет CAP, развилка без единственного mark — DAM.
Два берега торца имеют независимые route/reach и общим числом не связываются.

Каждый corner crop до merge ограничивается endpoint ownership incident site:
corner у `point_a` владеет `t <= 0.5`, corner у `point_b` — `t >= 0.5`.
Midpoint divider перпендикулярен segment и определяется его физическими
endpoints, а не индексами corner/site. Поэтому crops двух концов короткого
segment не перекрываются даже при `width > segment_length`; segment-cell
вычитает только уже ограниченные crops и сохраняет весь оставшийся coverage.

Дополнительные acute/obtuse bands из референсного инструмента нельзя добавлять
до определения их точной семантики. Они должны расширять
`CornerRuntimeSettings`, а не возвращать policy в compile stage.

## Execution parity

Modal `invoke`, обычный `execute` и headless/Python invocation обязаны
компилировать один и тот же `ManualSeamDecalPlan`. Preview и confirm используют
тот же evaluator с одинаковыми параметрами; отличается только точность
материализации.

## Structured generation transaction

Внутренний `generate_decal_result()` возвращает immutable
`DecalGenerationResult` со статусом `UPDATED`, `RETAINED_LAST_VALID`, `EMPTY`
или `ERROR`, а также `object_name`, `topology_changed`, backend summary и
runtime policy counts. Старый `generate_decal_objects() -> list[str]` сохранён
как raising compatibility adapter для non-modal callers.

Только `UPDATED` разрешает modal изменить scene property,
`_modal_current_value`, `_modal_current_settings` и last-valid generation
state. Остальные статусы оставляют эти значения и production geometry без
изменений и выводят status/reason в header. Confirm всегда перестраивает
последние валидные settings; отсутствие хотя бы одного valid state даёт явный
error вместо подтверждения произвольного текущего cursor value.

## Operator-owned drag targets

Modal хранит immutable descriptors `DecalDragTarget` для трёх целей одного
horizontal gesture:

- `W` — `Trim Height`, `Corner Width` или `Seam Width` текущего mode;
- `A` — `Acute Split Angle`, clamp `[1°, 179°]`;
- `M` — `Apex Limit`, clamp `>= 1`.

Каждый descriptor владеет settings field, Blender scene property, типом
`DISTANCE / ANGLE / RATIO`, диапазоном и sensitivity. Property `update=` не
участвует в runtime: compiled plan, preview state и last-valid transaction
остаются у modal operator.

Смена target и любое изменение состояния Shift выполняют input rebase:
текущее принятое значение становится новым base, а текущий X мыши — новым
start. Поэтому накопленный coarse delta никогда не пересчитывается с precise
sensitivity и значение не скачет при нажатии/отпускании Shift либо переходе
`W -> A -> M`. Угол хранится в радианах, но в header показывается в градусах.
Невалидный кадр по-прежнему не меняет ни target value, ни общий settings
snapshot. Esc/RMB восстанавливает исходные значения всех W/A/M properties,
которые могли измениться за одну modal-сессию.

### A10: live Acute Split для clean Patch Voronoi scope

Corner targets `A` и `M` допускаются только когда весь captured manual SEAMS
scope принят одним Patch Voronoi routing: selected edges в точности совпадают
с accepted Patch Voronoi edges и `Failed:0`. Legacy SEAMS runtime удалён;
для rail routing header сообщает требование Patch Voronoi-only scope.
`Failed` scope не запускает modal вообще: частичный `W`-preview запрещён.

Для активного `A` header имеет форму
`Acute Split: 60.0° | MITER:12 KITE:3 SPLIT:2 | 22.4 ms`. Policy counts
пишет сам Patch Voronoi evaluator во время текущего evaluation; отдельного
обхода materialized faces нет. Время также относится только к evaluator.
Drag меняет runtime threshold внутри уже compiled plan и не вызывает
`Construct()`. Переход порога может менять MITER на ACUTE_SPLIT и топологию
preview, сохраняя identity preview object и mesh datablock. Ошибка evaluation
не принимает новый threshold, удаляет preview и запрещает stale confirm.

### A11: approved corner bands, CAP/JUNCTION и UV

Oracle `docs/decal_corner_bands.md` утверждён. Runtime использует четыре
независимых ordered threshold `120/90/60/30°` и пять policy:
`MITER -> KITE -> FAN -> ACUTE_SPLIT -> HAIRPIN`. Сравнивается только compiled
`CornerSpec.extrusion_angle`; convex/concave задаёт parity, но не выбирает
band. На границе побеждает более мягкий band. Normalization клампит
`T1 >= T2 >= T3 >= T4`, поэтому совпавшие thresholds намеренно закрывают
промежуточный band. Hysteresis в v1 нет.

Каждый semantic corner crop получает owner-independent UV anchors в
каноническом frame: биссектриса `+V`, поперечная ось `U` с turn parity,
V-origin берётся из накопленной длины connected selected network. FAN
материализуется двумя triangle-компонентами; ACUTE_SPLIT сохраняет отдельные
INNER/OUTER UV на общем mesh edge; HAIRPIN оставляет blunt inner component и
не создаёт spike. Domain/Voronoi clipping вправе добавить stations или
триангулировать видимую часть, не меняя semantic kind/side.

CAP больше не строит axis-aligned square: terminal component ориентирован по
tangent выбранного site и имеет перпендикулярный FLAT торец. Valence-N
JUNCTION разворачивается в детерминированно упорядоченные angular sectors;
каждый non-reflex sector использует тот же band-classifier. Это действует как
внутри planar surface, так и для cross-surface connector fan. Reflex и
straight pass-through sectors не получают произвольного fill.

V-фаза теперь назначается connected network traversal, а не сбрасывается на
границе каждой `BoundaryChain`. Для two-sided same-chart site U вычисляется
как signed lateral distance, поэтому обе rails получают `-1/+1`. MITER/KITE
также используют semantic anchors и больше не зависят от случайного owner
PyVoronoi atom. Legacy parity отсутствует: действует A10 gate `Failed:0`.

## Source transform and world/local units

Публичный `DecalSettings` и modal properties хранят размеры в world units.
Перед compile/evaluate source transform проходит строгий metric preflight.
Допускаются только rotation + translation и positive uniform scale `s`.
Проверка использует полный 3x3 basis: `MᵀM` должен совпадать с `s²I` в
tolerance, а determinant обязан быть положительным.

Backend работает в local coordinates и получает:

```text
width_corner_local = width_corner_world / s
width_seam_local    = width_seam_world / s
height_trim_local   = height_trim_world / s
offset_local        = offset_world / s
uv_length_scale_local = uv_length_scale_world * s
```

Последнее умножение сохраняет world-space texel density: local arc length
после `matrix_world` становится в `s` раз больше, а UV остаётся рассчитанным
по мировой длине. Corner angle и Apex Limit безразмерны и не меняются.

Non-uniform scale, shear, reflection/negative determinant и degenerate matrix
отклоняются до PatchGraph analysis и создания preview. Structured API
возвращает `ERROR` с кодом `NON_UNIFORM_OR_SHEARED_SOURCE_TRANSFORM`,
`MIRRORED_SOURCE_TRANSFORM` или `DEGENERATE_SOURCE_TRANSFORM`; raising
compatibility API бросает `DecalSourceTransformError`. Полная поддержка таких
transforms требует отдельного world-space backend и не маскируется через
`Object.scale` либо `Matrix.to_scale()`.

## Developer build identity

В верхней части панели Hotspot UV показываются `Branch` и первые восемь
символов `Code commit`. В developer checkout resolver читает Git metadata
напрямую и поддерживает обычный checkout и linked worktree. Для скопированной
или упакованной версии без `.git` используется встроенная identity из
`version_info.py`.

Точный SHA нельзя включить внутрь самого коммита, потому что изменение файла
меняет SHA. Поэтому каждая поставляемая итерация заканчивается двумя
коммитами: implementation commit и metadata-only commit, встраивающим SHA
предыдущего implementation commit. Панель показывает именно `Code commit`,
а в handoff отдельно указывается metadata HEAD репозитория.

## Persistent preview materialization

Modal preview сохраняет один отдельный internal object
`.CFTUV_Preview_<Mode>_<Source>` и один mesh datablock. Object помечен
`cftuv_decal_preview`, содержит имя source и исключён из render. Production
`Decal_*` не читается и не мутируется ни на invoke, ни на `MOUSEMOVE`.

Новый BMesh кадра записывается в существующий preview mesh через
`BMesh.to_mesh()`; object/mesh не удаляются на каждом `MOUSEMOVE`. Esc/RMB и
Blender callback `cancel()` только удаляют preview object и orphan mesh и
возвращают scene property/header — production geometry не регенерируется.

Confirm сначала строит exact mesh в отдельном datablock. Только после
успешной materialization существующий production object сохраняет identity и
object-level custom properties, получает новый mesh и явно перенесённые
material slots. Старый production mesh удаляется после swap только при
`users == 0`. При exact failure production object/mesh остаются нетронутыми,
а temporary preview очищается.

Если SEAMS runtime crop падает или временно не создаёт faces, modal продолжает
работать, но временный preview удаляется и confirm блокируется до следующего
валидного кадра. Невалидные промежуточные параметры не становятся final
settings. Остальные producers сохраняют прежнюю last-valid preview семантику.

Blender 4.3 fixture `walls.003`, 27 selected seam edges: ширины
`2.5 / 3.0 / 3.7076 / 3.9` сохранили одинаковые object/mesh pointers при
изменении topology `84 → 86 → 80` vertices. Median шести preview-кадров:
`44.87 ms` с принудительным remove/new против `35.08 ms` с persistent mesh
(`1.28x`), без роста object/mesh datablocks и без zero-area faces.

## Topology fast path

После исключения object churn добавлен консервативный topology fast path:

- modal хранит `DecalPreviewState`, но не глобальные Blender/BMesh ссылки;
- signature канонизирует вершины по первому появлению в ordered face loops;
- при совпадении signature и object/mesh identity обновляются только vertex
  coordinates и loop UV через `foreach_set`;
- topology event автоматически использует полный `BMesh.to_mesh()` в том же
  persistent mesh datablock.

Differential на `walls.003` подтвердил `coordinate error = 0`, `UV error = 0`,
стабильные object/mesh pointers и корректные rebuild-события `84 → 86 → 80`
vertices. На этой средней fixture median почти не изменился
(`42.13 → 41.70 ms`): evaluator и построение временного BMesh уже дороже самой
записи mesh, а 5 из 10 кадров реально меняли connectivity.

## Affine lift cache

Профилирование показало, что обход временного BMesh не является следующим
узким местом. Экспериментальный direct `_NetworkFace -> Mesh` путь дал
`35.62 ms` против `34.77 ms` BMesh-пути на `walls.003` и поэтому не был
оставлен в production code.

Основная повторная работа находилась в orientation guard: для каждой station
трижды заново искался ближайший owner site и выполнялся полный lift при
`scale = 0 / 0.5 / 1`, после чего тот же поиск повторялся при emission.
Теперь один runtime evaluation:

- один раз разрешает owner site каждой arrangement station;
- сохраняет две affine позиции (`scale=0` и requested scale);
- аналитически получает коэффициенты signed area
  `A(t) = qa*t^2 + qb*t + qc` из этих endpoints;
- переиспользует endpoints при final emission через линейную интерполяцию.

Blender 4.3 differential на `walls.003` (27 edges) и `walls.001` (133 edges),
ширины `2.0 / 3.0 / 3.7076 / 4.5`: serialized `_NetworkFace` output совпал с
baseline, zero-area faces отсутствуют, object/mesh pointers стабильны.
Median evaluator:

- `walls.003`: `33.86 -> 22.76 ms` (`1.49x`);
- `walls.001`: `182.57 -> 103.95 ms` (`1.76x`).

Полный persistent modal preview на ширине `3.7076`: `23.90 ms` и
`109.84 ms` соответственно.

## Keyed fragment merge

Surface crop возвращает несколько triangle fragments одной semantic cell.
Раньше каждый fragment несколько раз заново проходил quantization, dedupe и
signed-area validation: при union-find, boundary reconstruction и затем ещё
раз в `_append_pending_fragments()`.

Теперь runtime merge:

- один раз нормализует fragment и вычисляет его quantized point keys;
- переиспользует эти keys для union-find и boundary half-edges;
- вычисляет signed area один раз на validation stage;
- не повторяет dedupe уже валидного merged contour.

Алгоритм объединения и topology policy не изменены. Differential против
`f8e0d1b` совпал на `walls.003` и `walls.001` при ширинах
`2.0 / 3.0 / 3.7076 / 4.5`. Дополнительное ускорение evaluator поверх affine
cache: `24.96 -> 21.82 ms` (`1.14x`) и `109.83 -> 97.46 ms` (`1.13x`).
Zero-area faces отсутствуют, persistent object/mesh identity сохранена.

## Startup polygon sweep

Startup profiling разделяет три независимые фазы: PatchGraph preparation,
width-independent PyVoronoi compile и первый exact BMesh. На `walls.001`
основная задержка находилась не в PyVoronoi C++ solve, а в Python
`_polygon_is_simple()`: sampled parabolic cells проходили полный O(n^2)
перебор всех пар рёбер перед triangulation.

Проверка теперь использует sweep broad phase по edge AABB. В active set
попадают только рёбра с пересекающимся X-диапазоном, Y-overlap отбрасывает
ещё одну часть пар, а прежний exact orientation/on-segment narrow phase
остаётся без изменений.

Differential против `09a945d` совпал на `walls.003` и `walls.001` при
ширинах `2.0 / 3.0 / 3.7076 / 4.5`. Compile:

- `walls.003`: `2069 -> 183 ms` (`11.3x`);
- `walls.001`: `5774 -> 389 ms` (`14.8x`).

Полный холодный старт `walls.001` после изменения: около `0.7 s`, включая
PatchGraph preparation, compile и exact BMesh; zero-area faces отсутствуют.

Сам Voronoi backend уже нативный (`pyvoronoi`). Если production fixture на
тысячах sites всё ещё требует ускорения, следующий native кандидат —
опциональный C++ backend для polygon clipping/fragment union с Python
fallback. GPU больше подходит для overlay preview: branch-heavy polygon
topology и переменная длина contours делают compute backend существенно
сложнее, чем C++ narrow phase.

Следующий существенный performance-слой должен профилировать surface-local
crop/clip отдельно и использовать независимость owner surfaces.
Простое кэширование по width не подходит: глобальная ширина меняет все
активные strips. BMesh остаётся точной транзакционной materialization на
confirm и проверенным preview adapter.

GPU overlay остаётся возможной следующей ступенью, но не требуется для
проверки runtime corner policy.

## Invalid Boost parabolas

`pyvoronoi` может вернуть curved edge, endpoint которого не удовлетворяет
параболе соответствующих point/segment sites. Это известный отказ внешнего
discretizer: увеличение `parabola_equation_tolerance` скрывает противоречие,
но создаёт недостоверную дугу и возможный разрыв у endpoint.

Обычные curved edges по-прежнему дискретизируются с точным tolerance
`0.0001`. Только `FocusOnDirectixException` и
`UnsolvableParabolaEquation` локально заменяют оба направления twin edge
общей хордой Boost endpoints. Поэтому одна некорректная парабола не отменяет
compile всей seam-сети, а граница соседних cells остаётся общей и замкнутой.

## Strict component routing

Manual Decal Seams не имеет legacy backend. Selected physical edges сначала
делятся на topology
components по общим source vertices. Partial Patch Voronoi probe локализует
rejected edges, после чего содержащие их components помечаются как `Failed`.

Все clean components повторно объединяются в один Patch Voronoi plan. Поэтому
раздельные chains на общей owner surface сохраняют глобальную Voronoi
competition и могут столкнуться при увеличении width. Если есть хотя бы один
`Failed` component, вся текущая SEAMS-транзакция атомарно отклоняется до
evaluation/BMesh: текущий preview удаляется, а routing report содержит physical
edges и каноническую причину. Частично «успешная» геометрия не маскирует отказ.

Operator report и console явно показывают routing:
`RAIL_PLANAR:<components>c/<edges>e | PLANAR:<components>c/<edges>e |
Unsupported[<reason>:xN] | Failed:<edges>e[<reason>:xN]`.
Runtime materialization транзакционна: сначала вычисляются faces всех
partitions и только затем записываются в BMesh. Пустой либо аварийный partition
не оставляет частично построенную hybrid-сетку.

До первой BMesh-записи валидируется вся partition: длины loop arrays, минимум
три вершины и отсутствие повторных vertex keys. `_materialize_network_faces()`
возвращает `DecalMaterializationResult`; любое прежнее условие `dropped > 0`
теперь бросает `DecalMaterializationError` с backend, physical edge indices,
face index, исходным vertex count и component kind/side. Temporary BMesh
полностью освобождается, finalize не вызывается. Для SEAMS preview удаляется,
а confirm текущего ошибочного кадра отменяется: last-valid mesh и last-valid
settings не маскируют текущий runtime failure.

### Exact selected-edge accounting

Manual SEAMS compile сохраняет строгую дизъюнктную разбивку:

```text
selected = accepted RAIL_PLANAR + accepted Patch Voronoi + failed
```

Edge без единого `BoundaryChain` use получает `NO_BOUNDARY_CHAIN_USE` и не
попадает ни в один geometry backend. Edge с тремя и более uses получает
`NON_MANIFOLD_EDGE_USE`: первые две стороны больше не выбираются молча.
Ровно один use остаётся допустимым односторонним boundary site, ровно два —
парным seam site. Backend summary показывает
`Failed:<count>e[<reason>:xN]`; console всегда печатает причины и physical
edge indices.

### Owner-face provenance

`PatchNode.mesh_tris` теперь сохраняет выровненные `mesh_tri_face_indices` и
`mesh_tri_face_normals`. Это важно для quad/ngon: Blender polygon имеет одну
owner-normal, тогда как его неявный triangle fan на слегка непланарной грани
даёт несколько несовпадающих triangle planes. Decal backend назначает site
исходной грани через `BoundaryChain.side_face_indices`, а не угадывает её по
triangle normal.

Планарные соседние faces с одной plane по-прежнему объединяются в общий
Voronoi chart. Warped polygon остаётся отдельным tangent owner-surface и
соединяется с соседними surfaces junction-слоем. На полном `walls.010` это
убирает ложный component rejection: все 23 components / 458 seam edges
компилируются Patch Voronoi, `Failed:0e`.

### Strict SEAMS runtime

После успешного compile backend фиксируется на весь modal lifetime. Ошибка
RAIL_PLANAR/Patch Voronoi evaluation или пустой runtime crop не перестраивает
component через legacy Seam Network либо старый miter pipeline. Evaluation всех
partitions выполняется до первой BMesh-записи; поэтому failure не оставляет
частично записанную транзакцию.

Во время SEAMS drag ошибка удаляет последний preview и возвращает structured
`ERROR`: текущая поломка не скрывается старой геометрией. Compile-rejected
components остаются явными `Failed`; наличие хотя бы одного из них атомарно
останавливает весь selected scope до evaluation. Настройка
`decal_seam_network=False` игнорируется и не может включить legacy.

## Tranche C developable chart acceptance

Дата замера: 2026-07-17. C7 synthetic F5 — цилиндрический fillet из 96
сегментов / 192 source triangles, один end-site, `alpha_budget=0.05`.
Chart build + admission на CPython unit harness (30 повторов):

- support: `9 / 192` triangles (`4.69%`, не full patch);
- mean: `3.780 ms`;
- p95: `3.917 ms`;
- max: `4.812 ms`.

Живой Blender acceptance на production mesh `rounded_wall` (64 vertices,
154 edges, 92 faces), все 68 seam edges: routing
`PLANAR+INTRINSIC_DEVELOPABLE:1c/68e`, шесть surfaces, из них две intrinsic,
без legacy/failure. Один compiled plan проверен на ширинах от `0.02` до
`25.9034`: preview evaluator занимал `100–138 ms`; число PyVoronoi
`Construct()` осталось `6` после всего drag sweep. На контрольных кадрах:

- один edge-connected decal component после объединения фронтов;
- zero-area faces = 0;
- overfull edges = 0;
- junction connector faces = 0;
- preview/confirm serialization совпадает.

Для полного выделения support двух небольших intrinsic patches закономерно
равен full patch (`60 / 60` triangles): все seam sites покрывают всю их
поверхность. Полосная пропорциональность отдельно доказана крупным F5 выше.

## Tranche D translational periodic charts

Annulus support после единственного generating cut-path допускается только при
доказанной трансляционной holonomy. Chart канонически ориентируется так, что
periodic axis — `U`; period и все diagram images лежат на общей
`DiagramTransform` quantum-grid. Конусный фрустум требует вращательной
периодичности и явно отклоняется с `PERIODIC_HOLONOMY_UNSUPPORTED`.

G7 запрещает только коллинеарный overlap generating cut с selected source
edge. Транзверсальное пересечение кольцевой chain и cut является штатным:
Voronoi sites у cut получают diagram-only images на `U ± period`. Atom хранит
целочисленный image shift, а runtime применяет его к site/crop view; canonical
sites и materialized faces не размножаются. При `alpha = period/2` фронты
антиподально смыкаются без gap и double-cover; больший width отклоняется
существующим `alpha_budget` preflight. Итоговый plan берёт минимум requested
strip budget и бюджетов всех intrinsic domains; `budget_source` указывает
реально связавший предел (`STRIP_BUDGET` либо `PERIODIC_HALF_PERIOD`).

Generating cut является одним детерминированным путём source-рёбер между
boundary-компонентами, поэтому поддерживает трубу из нескольких quad-рядов.
Сварка не использует float modulo. Каждая сторона cut получает собственный
periodic image-key; обе стороны сводятся только через
`ChartCut.transition_key` и фактически используемые
`transition_equivalences`.
Замкнутая selected-компонента получает одну детерминированную V-ориентацию;
диапазон V равен физической длине кольца, а closure остаётся в канонической
source vertex. Diagnostics/benchmark публикуют `periodic_copy_count` после
compile и `periodic_weld_count` текущего evaluation. Ни один periodic кадр не
вызывает новый `Pyvoronoi.Construct()`.

Acceptance oracle — `tests/test_decal_tranche_d_acceptance.py`: восемь cases
D4.1–D4.N2 покрывают круглый цилиндр, polygonal duct с hard fold'ами,
near-cut images, collision через seam, reversed winding, точную половину
period и оба канонических отказа. Non-periodic C7 fixtures остаются общим
differential guard.

D5-remediation дополнительно проверяет public budget preflight, cut-path на
трубе `8 x 3` колец, S1 supporting-lines при drag, одинаковый transport
seam-corner crop для emission/subtraction и фактическое использование
periodic image-ключей.

E0 measurement harness добавляет в compile diagnostics и benchmark JSON
фактическую sampled-ошибку geodesic half-width, максимальную вариацию
нормалей вдоль station-owner strip и число foldover. Скрипт
`artifacts/verify_decal_charts.py` выгружает те же поля для Blender fixtures;
admission policy на этом этапе не меняется.

Живой Blender 4.3.2 acceptance выполнен на отдельном open cylinder из 32
quad facets с кольцевой boundary chain. Результат: один periodic domain,
`period=12.5462`, `alpha_budget=6.2731`, `33` diagram copies, `2` seam weld,
`MITER:32` без особого corner в точке cut. На width `0.5` получен один
connected component из 32 faces, `vertical cut boundary edges = 0`,
`overfull edges = 0`; median/p95 evaluator `48.29/48.97 ms`, repeated preview
и confirm совпали, `Construct during drag = 0`.

Non-periodic differential на `rounded_wall` сохранил 156 faces и прежний
geometry digest
`336c91d5304583b3c3a629a598244b013466338dcc442b79db85a4afd4fb353d`;
periodic counters остались нулевыми. Полный machine-readable отчёт:
`artifacts/decal_tranche_d_blender_acceptance.json`.

## Terminal-route saturation (R1.9 / RR8d)

Terminal contact materialization consumes the compile-static station ledger.
When requested alpha exhausts a contour route, exhaustion is a valid
saturated state: the contact remains on the last station-prefix that produces
a simple terminal cut. This fallback is deliberately unavailable before route
exhaustion, so malformed ordinary cuts still raise
`TERMINAL_BRIDGE_CUT_INVALID`.

The materializer defensively stops before a repeated physical route edge even
though rail compile already emits a DAM at that point. Opposing routes that
meet on the same contour use equality of their accumulated station distances;
the source `start_edge_id` supplies the structural ordering if several exact
candidates exist. Runtime diagnostics expose saturation, station clamp,
revisit guard, and contact meeting as counted events.

On the user `sagging_wall` field case, width 35.98 and width 102.752491
(10x mesh bbox diagonal) both produce identical preview/confirm network-face
serialization. The previously reported preview-only success was the modal
last-valid frame retained before a controlled budget recompile, not a
different evaluator validation path.

## Rail competition ledger (R2 / RC1-RC3 / RR10)

A physical multi-edge rail that starts on one selected pChain and terminates
on a different selected pChain of the same patch is stored once. Reverse
traces are canonicalized before pole/merge processing, so a shared thread is
not represented as an owner route plus a synthetic `MERGE` route.

The canonical route has two `RailRouteReading` records. Both reference the
same route/stations, but measure accumulated distance from opposite chain
origins. Their exact equality is compiled into a `RailFreezeLocus` on either
a source vertex or a source-edge parameter. The lower canonical `chain_ref`
owns an exact tie. Readings and freeze loci are topology/compile facts and do
not change during width drag; the overlay draws freeze loci in orange.

R2 only publishes this immutable competition IR. Direct discovery of a
single-edge thread and non-planar materialization remain R3 work; no path
search or second curved geometry representation is introduced here.

## GPU preview overlay (F3)

Display adapter modal preview: во время drag декаль рисуется GPU draw
handler'ом (`cftuv/decal_gpu_preview.py`) без единой записи в mesh
datablock и без depsgraph updates; confirm — прежняя точная BMesh
materialization (parity by construction, FP4). Toggle
`Preview Display: GPU | GPU Textured | Mesh` в панели Decals; mesh —
отладочный fallback, автоматически используется в background и при
любом отказе gpu-среды (постоянный fail-safe в рамках сессии).

Семантика A6 сохранена: не-UPDATED кадр оставляет на экране последний
валидный batch. Пересборка batch'ей — только при смене topology
signature (kind/side/счётчик вершин на грань); чистое движение вершин
при width drag перезаливает буферы. SOLID — полупрозрачная раскраска
по component_kind + контур покрытия; GPU Textured — IMAGE-шейдер с
первой image-текстурой активного материала источника.

F0-lite split (headless, `artifacts/verify_decal_gpu_preview.py`,
median по 7 кадрам): batch-build составляет 2.3-6.4% времени
evaluator'а (silhouette 0.14-0.21 ms при evaluator 5-8 ms;
fan_wall_24 4.0-5.0 ms при 75-82 ms). Кадр GPU-пути = evaluator +
batch-build; mesh-write/depsgraph отсутствуют по построению.
Колонка depsgraph заполняется только живой Blender-сессией — ручной
чек-лист acceptance F3 (пп. 1-6 workplan) остаётся обязательным шагом
перед merge в основную ветку.
