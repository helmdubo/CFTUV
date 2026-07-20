# TRANCHE S — S-CM.a gate report

Дата: 2026-07-20  
Accepted baseline: `08ae83d`  
Ветка: `claude/blender-decal-corner-preview-yq4lir`

## Вердикт

S-CM.a готов к ревью. Фазовый контракт реализован как
`CornerSeed -> CornerModel -> ResolvedCornerView`: compile-static seed хранит
фактический `apex_ref`, упорядоченную пару incident sites, side, sector, owner
и join; width-dependent model строится после локального owner-domain/own-strip
clip и до глобальной конкуренции; resolved view сохраняет identity тех же
P1/P2 и не выводит форму повторно.

MITER и тестовый BEVEL являются двумя режимами одной модели. Единственная
семантическая ветка по join находится в `CornerModel._material_boundary`.
Runtime BEVEL по-прежнему fail-closed с
`DECAL_CORNER_JOIN_ARCHIVED_UNTIL_CORNER_MODEL`; публичный preview/confirm его
не включает до R2+S-CM.b.

Amendment `e7e8c9e` закрыт в этом же срезе как S-DF2-lite: аналитический
`DistanceWitness2D` + polygon-field служат только acceptance oracle и
семплируют итоговые materialized faces. Oracle не участвует в production
routing, не строит topology и не выбирает join.

## Атомарные коммиты

1. `f28f1ae` — typed phased CornerModel contracts и изолированные оракулы.
2. `bbecd37` — compile-static seeds и P1/P2 из локально клипнутых own-strips.
3. `c29cc36` — общий модельный путь MITER/тестового BEVEL.
4. `757b950` — отдельный I5 micro-slice: total PatchVoronoi provenance.
5. `7b720f1` — локальный clip в периодических chart images.
6. `b26dff6` — legacy KITE band-policy направлена в MITER-режим CornerModel;
   потребитель больше не сохраняет вторую форму угла.
7. `2f1a769` — насыщенный owner-domain выбирает P на endpoint-cap, сохраняя
   width-independent supporting lines при extreme-width drag.
8. `a737c32` — S-DF2-lite: labeled distance witnesses, CornerModel polygon
   field и sampling итоговой materialization по coverage/width/owner.

## Семантическая приёмка CornerModel

- BEVEL-контур содержит ровно `V/P1/P2`; остаточного MITER-apex,
  overlay-face и независимо выведенной второй границы нет.
- Collision, ownership, UV, projection и emission получают один и тот же
  authoritative boundary object.
- Каждая materialized вершина обязана трассироваться к ребру derived contour;
  иначе возникает именованный
  `CORNER_MATERIAL_VERTEX_OUTSIDE_SEMANTIC_CONTOUR`.
- Планарный изолированный BEVEL oracle эмитирует одну face из трёх вершин.
- MITER использует те же V/P1/P2 и, когда допустимо, один derived
  `MITER_APEX`; при clamp форма остаётся внутри той же модели.
- `ResolvedCornerView` ссылается на исходные P1/P2 по identity и хранит тот же
  derived boundary.
- RF28-негативы доказывают, что fan-дискретизация не может быть принята за
  BEVEL-хорду, а arrangement не содержит захардкоженной join-ветки.
- Статический поиск по runtime нашёл только две ожидаемые проверки:
  публичный BEVEL-гейт и единственную модельную ветку.

## S-DF2-lite analytical oracle

`DistanceWitness2D` возвращает не только неотрицательное расстояние, но и
`closest_point`, gradient, site id, явный `BandSide`, source parameter,
`source_s` и feature kind. Направление `source_s` берётся из station ref, а не
угадывается по знаку chart-оси.

`CornerPolygonField2D` получает уже готовый shared boundary от CornerModel.
В нём отсутствует чтение `CornerJoinMode`, `MITER` или `BEVEL`: зависимость
строго `join policy -> CornerModel -> oracle field`. Материализация переводится
в chart как финальные faces с настоящим `semantic_owner_id`; векторные samples
берутся из вершин, рёбер и face interiors intended/materialized polygons.

Позитивные оракулы:

- изолированный BEVEL `V/P1/P2` совпадает с intended field;
- финальные production MITER faces после полного materialization совпадают по
  coverage, обеим P-ширинам и owner.

Негативы независимо ловят materialized область за BEVEL-хордой/остаточный
апекс, неверную requested half-width и owner desync. Растрового поля, marching
squares и min/max pseudo-SDF композиции нет: oracle не является
полигонизатором и не создаёт production topology.

## I5 provenance gate

I5 закрыт полностью до R2. Для каждой PatchVoronoi face и каждой вершины
сериализуются source face/edge, route, station и domain location. Provenance
сохраняется через cross-surface station insertion, junction connectors, cap
alignment, terminal merge и финальную materialization; `GeometryBatch`
замораживает её вместе с геометрией.

Объём вне углов оказался ненулевым, поэтому вынесен в отдельный атомарный
micro-slice `757b950`. После него остаточный gap равен нулю. Планарный oracle
дополнительно проверяет, что face provenance указывает на фактический
содержащий Blender-triangle/source face, а preview и confirm сериализуют
одинаковую immutable batch.

## Differential MITER

Baseline поведения: `bbecd37` (seed/clip уже присутствуют, CornerModel ещё не
владеет runtime-формой). After: `a737c32`. Blender 4.3.2 / Python 3.11.9,
`E:\testscene.blend`, SHA-256
`4d9014dc436ac80c284fdf7f473c693c8c81550ceb026db6db712c42737bae10`;
сцена не сохранялась.

`artifacts/decal_s_cm_a_comparison_field.json`:

- одинаковый blend hash и plan semantics на всех 6 полевых объектах;
- topology/policy равны на обоих width для всех объектов;
- `all_preview_confirm_equal=true`;
- raw plan digest намеренно меняется там, где добавлены I5 provenance и
  `CornerStationRef.source_s_per_chart_unit` для labeled S-DF witness;
- geometry identity digest меняется на `rounded_wall.001` и `sagging_wall`,
  потому что внутренняя owner identity переходит `KITE -> MITER`; наблюдаемая
  геометрия обязательных ракурсов проверена отдельно пиксельно.

## Полевая приёмка §0d.0

До -> после, одинаковые camera/selection/width и один и тот же `.blend`:

| Ракурс | До | После | Проверка |
|---|---|---|---|
| `walls.006`, 15 выбранных seams, width `3.2` | legacy `KITE:2`, `SEGMENT:20` | CornerModel `MITER:2`, `SEGMENT:20` | 1800x1100, `changed_components=0` |
| `sagging_wall`, edges `6,7,8`, width `3.2` | legacy `KITE:1`, `SEGMENT:19` | CornerModel `MITER:1`, `SEGMENT:19` | 1800x1100, `changed_components=0` |

Артефакты:

- `artifacts/decal_s_cm_a_before_walls006_miter_w32.png`
- `artifacts/decal_s_cm_a_field_walls006_miter_w32.png`
- `artifacts/decal_s_cm_a_before_sagging_wall_miter_w32.png`
- `artifacts/decal_s_cm_a_field_sagging_wall_miter_w32.png`
- `artifacts/decal_s_cm_a_field_pixels.json`

`all_pixel_equal=true`: остаточного overlay/apex или новой видимой границы в
MITER-режиме нет. Пользовательских стрелок для этого структурного среза не
поступало; воспроизведены два прямо названных обязательных ракурса.

## Тестовые гейты

- CornerModel/Voronoi/RF28/Tranche-D/extreme-width stability:
  `159 passed` (включая S-DF2-lite positive/negative oracles).
- Tier 2: `python -m pytest -m "not atlas_frozen" -q` ->
  `471 passed, 3 skipped, 4 deselected` за 73.52 s.
- `python -m compileall -q ...` — PASS.
- `git diff --check` — PASS.
- Blender differential и два render job — PASS; `.blend` не сохранялся.
- Frozen GL atlas не запускался: marker-excluded 4 теста не относятся к
  локальному S-CM.a, а curved/approximate materialization остаётся fail-closed.

## Малые пивоты (§0f.6)

1. I5 закрыт отдельным micro-slice, потому что provenance gap включал не только
   corner faces. Смешивать тотальную родословную с модельной формой было хуже
   для ревью; незакрытого объёма не осталось.
2. Tranche-D выявил периодические chart images: локальный own-strip clip теперь
   рассматривает соседние period copies, не меняя canonical source keys.
3. Полевой рендер выявил, что прежний KITE band-policy всё ещё обходил модель.
   Он перенаправлен в MITER-режим CornerModel без изменения видимых пикселей.
4. Extreme-width suite выявил неверный выбор ближайшего rail/domain-угла как P.
   P теперь берётся с клипнутого endpoint-cap; все S1 supporting-line/freeze
   инварианты снова зелёные.
5. Docs-only amendment `e7e8c9e` пришёл перед публикацией и добавил
   S-DF2-lite в текущий gate. Он реализован отдельным oracle-коммитом; S-WF0
   намеренно не начат.

## Риски

- `ResolvedCornerView` и повторное использование якорей доказаны контрактом и
  локальными тестами, но полный post-competition freeze/lifecycle подключается
  только в R2+S-CM.b. В S-CM.a глобальную конкуренцию не переархитектуризировал.
- BEVEL проверен только приватным test contour и намеренно остаётся за runtime-
  гейтом. UI/runtime возврат до R2+S-CM.b запрещён.
- Периодические planar charts учтены, но curved R3 materialization не заявлена:
  half-sphere/approximate paths остаются именованно fail-closed.
- `atlas_frozen` не запускался; при продолжении atlas work нужен отдельный GL-
  гейт. Текущий срез не ослабляет его admission/failure contracts.
- Текущий `E:\testscene.blend` имеет новый hash относительно более раннего
  S1+S2a отчёта. Все S-CM.a before/after receipts и кадры используют именно
  один текущий hash, поэтому differential внутри среза валиден.
- S-DF2-lite — семплирующий аналитический oracle, а не формальное доказательство
  равенства произвольных polygon unions. Он детерминированно покрывает vertices,
  edge quartiles/midpoints и face interiors и имеет адресные негативы Б5;
  production topology остаётся у vector arrangement.
- S-WF0 не начинался. В коде S-CM.a не добавлено нового предположения
  «rail всегда идёт по рёбрам»: вопрос edge-path против характеристики
  intrinsic field остаётся открытым до отдельного пользовательского решения.
