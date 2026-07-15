# Decal Surface Domains: путь к непланарным patches

## Инвариант

Voronoi, `CornerSpec`, runtime crop-компоненты и `DecalArrangement` работают
только в intrinsic 2D-домене. Они не должны знать, является owner surface
плоскостью, развёрнутой face-strip, цилиндром или локальным atlas органической
поверхности.

Связь с mesh принадлежит `DecalSurfaceDomain`:

- `project`/chart coordinates для source sites;
- boundary triangles для ограничения диаграммы;
- `lift(point, offset)` для обратного переноса результата;
- `normal_at(point)` для offset и ориентации faces;
- periodic-axis metadata для труб и замкнутых лент.

Текущий backend компилирует только `PLANAR` domain. Это admission policy, а не
ограничение corner/Voronoi алгоритма.

## CFTUV-сущности

- **Patch** задаёт область конкуренции сайтов. Геометрически близкие, но
  топологически несвязанные поверхности не конкурируют.
- **BoundaryChain / selected seam chain** создаёт segment sites.
- **CornerSpec** классифицирует intrinsic turn внутри одного domain.
- **Junction** связывает domains в общей source-вершине, но не становится
  первичной единицей Voronoi solve.

Так сохраняется chain-first архитектура основного CFTUV: decal backend
потребляет chains/patch adjacency, не переопределяя solve frontier.

## Этапы

### 1. Piecewise-planar face-strip

Для фасок, скруглений из нескольких граней и hard-surface труб:

1. Flood-fill только внутри одного CFTUV patch.
2. Развернуть соседние triangles по face adjacency, сохраняя длины общих edges.
3. Делать chart cut при чрезмерной holonomy/distortion или вокруг periodic cycle.
4. Строить одну segment-Voronoi partition в chart.
5. Поднимать vertices barycentric через `_IntrinsicDomainTriangle`.

Это ближайший практический этап: он покрывает bevels и полигональные rounded
corners без геодезического solver.

### 2. Periodic domains

Для цилиндров и труб domain хранит `periodic_axis="U"`:

- диаграмма получает копии sites около `U ± period`;
- после clipping coordinates сворачиваются обратно;
- seam chart выбирается детерминированно вне активной decal chain;
- closed chains сохраняют единый transport direction и UV parity.

### 3. Smooth / organic atlas

Для cliffs и сильно искривлённых surfaces одной развёртки может не быть.
Domain становится atlas из перекрывающихся intrinsic charts:

- sites распространяются по triangle adjacency;
- конкуренция ограничена geodesic/intrinsic расстоянием внутри patch;
- широкая декаль может переходить между charts через transition maps;
- новые charts создаются при превышении distortion budget;
- финальный arrangement сшивает общие chart-boundary stations до BMesh.

Это не должен быть 3D nearest-surface поиск: он склеит тонкие стены и близкие,
но топологически далёкие части mesh.

## Что переносится без изменений

- segment/point metadata `pyvoronoi`;
- `CornerSpec` и политики `KITE` / `ACUTE_SPLIT`;
- динамический crop по ширине во время drag;
- surface-level station insertion;
- preview/final materialization из одного arrangement.

## Что ещё потребуется

- intrinsic chart builder из PatchGraph face adjacency;
- provenance chart-triangle для каждой station после merge/clipping;
- transition keys между charts;
- periodic Voronoi copies;
- distortion/holonomy budget и deterministic chart cuts;
- регрессии на bevel strip, quarter-cylinder, closed tube и cliff fold.

