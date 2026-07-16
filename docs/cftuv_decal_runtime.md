# Decal runtime contract

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

Сейчас доступны два параметра, семантика которых уже определена backend:

- `Acute Split Angle` — ниже порога corner становится двухкомпонентным
  `ACUTE_SPLIT`;
- `Miter Limit` — максимальная длина miter относительно half-width.

Дополнительные acute/obtuse bands из референсного инструмента нельзя добавлять
до определения их точной семантики. Они должны расширять
`CornerRuntimeSettings`, а не возвращать policy в compile stage.

## Execution parity

Modal `invoke`, обычный `execute` и headless/Python invocation обязаны
компилировать один и тот же `ManualSeamDecalPlan`. Preview и confirm используют
тот же evaluator с одинаковыми параметрами; отличается только точность
материализации.

## Persistent preview materialization

Modal preview сохраняет один Blender object и один mesh datablock. Новый BMesh
кадра записывается в существующий mesh через `BMesh.to_mesh()`; object/mesh не
удаляются на каждом `MOUSEMOVE`. Confirm и cancel выполняют одну точную замену.

Если runtime crop падает или временно не создаёт faces, modal продолжает
работать и оставляет последний валидный preview. Невалидные промежуточные
параметры не становятся final settings.

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

Следующий существенный performance-слой должен профилировать surface-local
crop/clip отдельно и использовать независимость owner surfaces.
Простое кэширование по width не подходит: глобальная ширина меняет все
активные strips. BMesh остаётся точной транзакционной materialization на
confirm и проверенным preview adapter.

GPU overlay остаётся возможной следующей ступенью, но не требуется для
проверки runtime corner policy.
