# TRANCHE S — R2 + S-CM.b gate report

Дата: 2026-07-20
Accepted baseline: `3085410`
Ветка: `claude/blender-decal-corner-preview-yq4lir`

## Вердикт

R2 + S-CM.b готов к ревью. Решение пользователя по S-WF0 зафиксировано как
**A**: distance field остаётся acceptance oracle и не участвует в production
routing. Планарная конкуренция разрешается только станционной метрикой; при
отсутствии единственного ответа действует RC4-stop
`RAIL_COMPETITION_METRIC_UNRESOLVED`.

R2 компилирует один физический route на нить и два направленных чтения
станций. Freeze-locus хранится в каноническом vertex/edge station key и не
двигается при width drag. S-CM.b подключает `ResolvedCornerView` после
конкуренции, сохраняя identity исходных P1/P2 и derived boundary. BEVEL
возвращён в публичный UI/runtime как compile-событие; тихой коэрции нет.

## Атомарные коммиты

1. `de5ebec` — compile-static station competition, dual readings и freeze.
2. `a84009e` — R2 consumer, `CapacityPolicy`, S-CM.b и публичный BEVEL.
3. `dda8636` — patch identity в ключе `ResolvedCornerView`.
4. `b50613d` — единый Blender acceptance harness R2 + S-CM.b.
5. `7f9eb5c` — RV2 failure-priority перед RC4 и I8 periodic assertion.
6. `544f868` — live-Blender RF7/RF17 receipt в общем полевом гейте.

## R2: RC1–RC4, RR10 и R1.9

- Восемь продольных нитей RF7 дают `8` routes, `16` readings и `8`
  `THREAD_DUAL_READING` freeze-loci, а не 16 встречных routes с ложными
  merge-событиями.
- Каждый route читается от обеих chains со станциями `(0,1,2)` и `(2,1,0)`;
  structural tie-break выбирает минимальный chain ref.
- Freeze ledger бит-идентичен при `alpha_budget 3 -> 30` и обратном порядке
  selected edges. RF17 использует тот же объект locus; отдельной геометрии
  нити для второго чтения нет.
- Mid-edge вариант хранит source edge + parameter; vertex-вариант — source
  vertex. Runtime не пересчитывает локус из текущей ширины.
- `TERMINAL_BRIDGE_CUTS_OVERLAP` больше не разрешается 2D-импровизацией:
  встречные terminal guides обязаны ссылаться на один compile-static
  `RailFreezeLocus`; иначе RC4-stop.
- Ревизия R1.9 принята на новом фундаменте. `CapacityPolicy` различает
  `SATURATE_PROVEN`, `CONTROLLED_RECOMPILE` и `REJECT_UNPROVEN`. Periodic и
  approximate capacity не клампятся без доказательства и дают
  `DOMAIN_CAPACITY_UNPROVEN`.
- Footprint validation выполняется до RC4: истинно вырожденная source-face
  сохраняет первичный именованный отказ `SOURCE_TRIANGLE_DEGENERATE`.

## S-CM.b и RF28

- `ResolvedCornerView` создаётся после arrangement/competition и читает те же
  `CornerModel`, `CornerDerivedGeometry`, P1 и P2 по identity.
- Ключ resolved source включает `(patch_id, semantic_owner_id, chart_id)`.
  Полевой `sagging_wall` обнаружил, что одного owner/chart недостаточно:
  одинаковые chart ids разных PatchSurface смешивали вершины углов. Исправление
  закрыто отдельной регрессией.
- BEVEL-контур итоговой материализации содержит ровно `V/P1/P2`; остаточного
  апекса, FAN-дискретизации и отдельной overlay-границы нет.
- Own-strip differential использует provenance: corner-local fallback может
  исторически иметь `component_kind=SEGMENT`, но
  `semantic_owner_id=('corner', ...)` и не является собственной лентой.
  После отделения corner matter собственные strip-segments MITER/BEVEL
  бит-идентичны на обеих полевых сценах и обеих ширинах.
- На `walls.006`, width `3.2`, чужой фронтир содержит `10` materialized
  competition stations и доходит до BEVEL-хорды. Distance-witness не нашёл
  утечки за хорду или owner mismatch.

## Полевая приёмка §0d.0

Blender `4.3.2`, Python `3.11.9`, `E:\testscene.blend`, SHA-256
`2276811c5e42cc04f6aa21ba103185bb0c8438f25bf44213ff976c74a5376168`.
Source commit receipt: `544f8685ad3aa9524a883d82bb74ca457943ad06`.
Сцена не сохранялась.

| Контур | Проверка | Результат |
|---|---|---|
| RF7/RF17 live fixture | routes/readings/freeze, alpha `3 -> 30`, reverse enumeration | `8/16/8`, compile-static `true` |
| `walls.006`, 15 seams | MITER/BEVEL, widths `0.8/3.2` | own segments equal; 2 BEVEL faces; 10 chord stations на `3.2` |
| `sagging_wall`, edges `6/7/8` | MITER/BEVEL, widths `0.8/3.2` | own segments equal; 2 BEVEL faces; oracle safe |
| Канонические 6 объектов | MITER widths `0.4/0.8` | accounting exact; preview==confirm; round-trip stable |

Итого: `20/20` preview/confirm пар и `10/10` round-trip чтений
бит-идентичны; `34/34` materialized CornerModel records сохраняют anchor/
derived identity и проходят safe distance-witness проверку. На wide RF28
зафиксировано `20` competition vertices суммарно по MITER/BEVEL.

Артефакты:

- `artifacts/decal_r2_s_cm_b_field_acceptance.json` — полный машинный receipt;
- `artifacts/decal_r2_s_cm_b_field_walls006_bevel_w32.png` — стена с проёмом;
- `artifacts/decal_r2_s_cm_b_field_sagging_wall_bevel_w32.png` — sagging wall;
- `artifacts/decal_r2_rf7_overlay.png` — сохранённый overlay prep `94bb679`;
  канонический locus повторно проверен текущим live receipt.

Wireframe-ракурсы показывают зоны срезов: orange — spine, cyan — owned
strips, magenta — материал CornerModel/BEVEL chord. Пользовательских стрелок
для нового ракурса в этом срезе не поступало; воспроизведены два прямо
названных обязательных вида.

## Тестовые гейты

- Targeted R2/RF7/RV2/I8: `30 passed`.
- Tier 2: `python -m pytest -q -m "not atlas_frozen"` ->
  `485 passed, 3 skipped, 4 deselected` за `66.16 s`.
- `python -m compileall -q ...` — PASS.
- `git diff --check` — PASS.
- Blender acceptance — `green=true`; `.blend` не сохранялся.

## Малые пивоты (§0f.6)

1. Заготовка `94bb679` не переносилась механически. Один route/two readings и
   vertex freeze сохранены, схема расширена route-pair competition и
   edge-parameter locus; ложные merge-события удалены.
2. Полевой прогон обнаружил collision identity разных PatchSurface с одним
   chart id. `patch_id` добавлен в resolved key отдельным fix-коммитом.
3. Первое сравнение `sagging_wall` считало corner-local fallback с kind
   `SEGMENT` частью own strip. Differential переведён на provenance, production
   geometry не менялась; быстрый и полный Blender-гейты после этого зелёные.
4. Полный suite обнаружил приоритет RC4 над RV2. Source-geometry validation
   перенесена перед competition compile без изменения успешных routes.

## Риски

- В пользовательской сцене RF7-конфигурации встречных нитей нет в выбранных
  production selections; `8/16/8` доказаны live-Blender synthetic fixture и
  unit regressions. Полевые объекты доказывают materialization, но не должны
  выдаваться за событие, которого в них нет.
- `rounded_wall_noise_top`, `wall_noise_top` и `half_sphere` сохраняют прежние
  именованные admission failures (`FOLDOVER_DETECTED`,
  `DEGENERATE_SOURCE_TRIANGLE`/`DISTORTION_BUDGET_EXCEEDED`, curved owner).
  Они не замаскированы пустой успешной геометрией; R3 остаётся владельцем
  curved scope.
- Distance witness — семплирующий oracle. При конкуренции итоговая материя
  является безопасным подмножеством intended CornerModel: coverage mismatch
  допустим только как срез чужим фронтиром; leak и owner mismatch запрещены.
- `atlas_frozen` не запускался: четыре marker-excluded GL-теста не относятся к
  R2/S-CM.b и требуют отдельного окружения при работе над atlas.

## Особое мнение

`component_kind=SEGMENT` недостаточен для семантического differential:
поглощённая point-cell может сохранить этот kind, оставаясь угловой материей
по provenance. Дальнейшие no-op сравнения должны классифицировать собственную
ленту по semantic owner/source provenance, иначе тест будет штрафовать
легальную смену разбиения углового куска.

R3 начинать рано. Следующий утверждённый срез — S2b (session ownership и
MetricContext). На старте R3 остаётся отдельный пользовательский гейт выбора
интринсик-бэкенда ширины: FMM — кандидат, Heat — отрицательный контроль.
