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

## Следующий performance-срез

Runtime policy не решает Blender object churn. Следующий слой должен:

1. сохранять один preview object/mesh на весь modal lifetime;
2. сравнивать topology signature между кадрами;
3. при неизменной topology обновлять только coordinates и loop UV;
4. при topology event перестраивать только dirty patch;
5. при ошибке кадра оставлять последний валидный preview;
6. выполнять точную транзакционную materialization только на confirm.

GPU overlay остаётся возможной следующей ступенью, но не требуется для
проверки runtime corner policy.
