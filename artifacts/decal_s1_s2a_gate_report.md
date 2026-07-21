# TRANCHE S — S1+S2a gate report

Дата: 2026-07-20
Baseline: `0eeca5cfe28ea76c810b55628b3359cf95bfac73`
Ветка: `claude/blender-decal-corner-preview-yq4lir`

## Вердикт

S1+S2a готов к ревью. Analysis публикует атомарный `AnalysisBundle`
(`SourceRevision` + `PatchGraph` + `PatchSurfaceIR` + capabilities), а decal
runtime больше не получает полную tessellation через `PatchNode`. Rail читает
исходные polygon cycles, chart — Blender loop triangles с реальными
triangle-normal, tessellation diagonal имеет `physical_edge_id=None`.

Оба preview-adapter используют `PreviewFailurePolicy.CLEAR`; GPU topology и
boundary считаются по `vert_keys`; dihedral хранит per-segment пару локальных
нормалей двух chain uses. World/local settings, `MetricContext`, backend enums,
typed plans и глубоко immutable `GeometryBatch` введены в runtime boundary.

## Перенос из PatchNode

Удалены ровно шесть surface-tessellation полей:

- `mesh_verts`
- `mesh_vert_indices`
- `mesh_tris`
- `mesh_tri_face_indices`
- `mesh_tri_face_normals`
- `mesh_tri_edge_indices`

Solve-facing `vert_cos`, `basis_u`, `basis_v`, patch normal/area и прочие
PatchGraph facts не переносились и не меняли ownership.

## Намеренные изменения поведения

S0-style semantic equality здесь неприменима: исправляется authoritative
tessellation/topology. Сравнение ниже получено из tracked field-артефакта на
baseline и нового `artifacts/decal_r13_rr9a_field_acceptance.json`.

| Объект | Было | Стало | Почему это ожидаемо |
|---|---|---|---|
| `rounded_wall.001` | backend тот же; faces `39/39` при width `0.4/0.8` | backend тот же; `39/40` | Широкое покрытие читает Blender tessellation; одна грань больше не теряется fan-представлением. |
| `rounded_wall_noise_top` | `DISTORTION_BUDGET_EXCEEDED`, 0 faces | `FOLDOVER_DETECTED`, 0 faces | Реальные triangle normals и Blender winding называют более раннюю фактическую причину; unsupported остаётся видимым. |
| `sagging_wall` | `130/136` faces | `130/137` faces | На width `0.8` Blender tessellation сохраняет одну дополнительную допустимую грань; routing и accounting те же. |
| `wall_noise_top` | rail: `RAIL_GEOMETRY_SOURCE_FACE_NON_PLANAR`, 0 faces | rail: `RAIL_GEOMETRY_BOUNDARY_PIECE_AMBIGUOUS`, 0 faces | Rail теперь читает исходный `SourceFace.vertex_cycle/edge_cycle`, а не реконструирует polygon из fan triangles; общий unsupported outcome не маскируется. |
| `half_sphere` | `DISTORTION_BUDGET_EXCEEDED`, 0 faces | без изменения | R3-ограничение осталось именованным и видимым. |
| `building` | `RAIL_PLANAR`, `16/16` faces | `RAIL_PLANAR`, `14/14` faces | Exact SourceFace cycles и только физические edge ids убрали две fan-derived materialization pieces; routing/accounting остаются точными. |

## Blender tessellation gate

Blender 4.3.2 / Python 3.11.9. Артефакт:
`artifacts/decal_s1_surface_ir_blender.json` (`verdict=PASS`).

- concave U-shaped n-gon: 6 Blender triangles, площадь `7.0`, все центроиды
  внутри polygon;
- non-planar quad: две различные реальные triangle normals;
- reversed U-shaped n-gon: площадь `7.0`, normals имеют обратный знак;
- во всех случаях tessellation diagonals сериализованы как `None`.

## Полевая приёмка §0d

Сцена: `E:\testscene.blend`, SHA-256
`0ac29c4ebb6df66f78735cab8c0babbd2f184a02b69bb84c30d04ad7a6995cb1`.
Blender 4.3.2, `blend_saved=false`, missing objects = 0. Все 6 объектов имеют
exact selected-edge accounting и `compile_static=true` на width `0.4` и `0.8`.

Автоскриншоты канонического ракурса:

- `artifacts/decal_s1_field_rounded_wall_001.png`
- `artifacts/decal_s1_field_rounded_wall_noise_top.png`
- `artifacts/decal_s1_field_sagging_wall.png`
- `artifacts/decal_s1_field_wall_noise_top.png`
- `artifacts/decal_s1_field_half_sphere.png`
- `artifacts/decal_s1_field_building.png`

Визуальная сверка: поддержанные strips связны и лежат на source surface;
известные unsupported objects показывают выбранные рёбра и именованный отказ,
без stale preview. Пользовательских аннотированных стрелок для этого
структурного среза не было; все наблюдаемые field-diff перечислены
по-стрелочно в таблице выше.

## Тестовые гейты

- Tier 0/1: затронутые analysis/surface, rail, chart, GPU, geometry,
  transform, decals и operator suites пройдены модульными батчами; последний
  fixture-only boundary miss (`test_decal_corner_stability`) исправлен и его
  локальный прогон дал `6 passed`.
- Tier 2: `python -m pytest -q -m "not atlas_frozen"` ->
  `455 passed, 3 skipped, 4 deselected` за 65.05 s.
- Frozen GL atlas не запускался по §0d.3a; изменённые atlas fixtures входят в
  обычную import/collection проверку, сами 4 теста остаются marker-excluded.
- `python -m compileall -q ...` — PASS.
- `git diff --check` — PASS (только уведомления Git о будущей LF/CRLF
  нормализации, whitespace errors отсутствуют).

## Малые пивоты (§0f.6)

1. Внутри Voronoi оставлен frozen read-only `_PatchSurfaceNodeView` с прежними
   field-shaped именами. Это не второй источник: view строится только из
   `PatchSurfaceIR` и не принимает решений. Полное переименование 14k-строчного
   монолита отложено до ownership-декомпозиции S3, чтобы не смешивать
   механический churn с контрактным срезом.
2. Cross-IR учитывает законную форму open chain: normals имеют ровно
   `edge_count` элементов, а `side_face_indices` может иметь endpoint-запись
   (`edge_count + 1`). Реально отсутствующие segment normals всё ещё дают
   `DECAL_ANALYSIS_SCHEMA_UNSUPPORTED`.
3. Field schema поднята до `cftuv.decal_r13_rr9a.field.v2`: из артефакта удалён
   process-specific `id(plan)`, оставлен детерминированный `compile_static`.

## Риски

- `MetricContext` и несовместимые World/Local settings уже запрещают повторно
  конвертировать `LocalDecalSettings`, но единое session-владение request/context
  появится только в S2b. До него compatibility entry points могут независимо
  построить локальный request для compile и evaluate; числовой результат не
  конвертируется дважды, но lifetime ещё не выражен одним controller object.
- `GeometryBatch` теперь immutable и един для preview/materializer, однако
  тотальная I5 provenance исторического PatchVoronoi output ещё не завершена:
  rail faces несут `RailFaceProvenance`, часть intrinsic faces пока имеет
  `provenance=None`. Подделывать provenance в фундаментальном срезе не стал;
  это нужно закрыть при ревизии заготовок/R2 до объявления I5 выполненным.
- `atlas_frozen` намеренно не исполнялся. Этот срез меняет вход surface IR для
  atlas fixtures, поэтому при возобновлении GL потребуется отдельный полный
  atlas gate; текущий fail-closed
  `APPROXIMATE_MATERIALIZATION_PENDING` не ослаблялся.
