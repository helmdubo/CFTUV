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

## Следующий performance-срез

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

Следующий существенный performance-слой должен:

1. формировать preview payload непосредственно из `_NetworkFace`, не создавая
   промежуточный BMesh на стабильном кадре;
2. при topology event перестраивать только dirty patch;
3. измерять evaluator, arrangement и materialization отдельно;
4. выполнять точную транзакционную BMesh materialization только на confirm.

GPU overlay остаётся возможной следующей ступенью, но не требуется для
проверки runtime corner policy.
