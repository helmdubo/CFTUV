# CFTUV Decal Engine

> **TRANCHE S status.** Единственный production capability — compiled strict
> `SEAMS`. `TOP`, `BOTTOM` и `CORNERS` архивированы и возвращают стабильные
> mode-specific причины до analysis/BMesh. Их будущий acceptance contract
> находится в [decal_archived_modes_product_contract.md](decal_archived_modes_product_contract.md).

## Runtime contour

```text
selected seam edges
  → exact physical-edge scope
  → rail/chart compile plan
  → width-dependent evaluator
  → immutable face batch
  → preview / confirm adapters
```

Preview и confirm читают один compiled plan и один evaluator output. Ни один
compile/runtime отказ не имеет права переключить scope на старый producer,
частично материализовать успешные компоненты или оставить предыдущий preview
подтверждаемым.

Историческая seam network, one-shot network и direct miter/junction pipeline
не являются runtime-вариантами. Старое Blender-свойство сети сохраняется
только ради чтения прежних `.blend`, скрыто из панели и не влияет на routing.

## Выбор и accounting

`SEAMS` доступен только в Edit Mode / Edge Select Mode:

1. Каждое выбранное seam edge является атомарной единицей scope.
2. Его одна или две owner-side записи находятся в полном PatchGraph по
   исходному mesh edge index. Разное разбиение chains на двух patches не
   расширяет выделение до всей петли и не создаёт дубль.
3. Выбранные edges сохраняются выделенными после операции.
4. Compiled plan обязан один раз и без пересечений учесть каждый edge как
   `RAIL_PLANAR`, `PATCH_VORONOI` либо `Failed`.
5. Один `Failed` атомарно отклоняет весь выбранный scope до evaluation и
   BMesh-записи. Object/Face-mode и вызов без selected-edge plan дают
   именованный отказ `SEAMS_REQUIRE_SELECTED_EDGE_PLAN`.
6. Устаревший plan для другого selection даёт `SEAMS_PLAN_SCOPE_MISMATCH`;
   повреждённый accounting — `SEAMS_PLAN_ACCOUNTING_MISMATCH`.

User-facing summary публикует backend routing и детерминированные счётчики
канонических причин. Edge IDs и причины видны в exception/header и не
зависят от verbose console.

## Geometry и UV

`Seam Width` — полная world-space ширина. На некомпланарном seam она
распределяется по локальным owner-sides; на копланарном rail образует
центрированную ленту. `Offset` применяется к каждой локальной owner-surface.

UV занимает `DECAL_UV_RECT_SEAM = (0.9, 0.0, 1.0, 1.0)`. Продольная
координата идёт по V с плотностью `UVSettings.final_scale`, поперечная — по U.
Материализация валидирует весь batch до первой BMesh-записи: malformed,
bow-tie или degenerate face отклоняет транзакцию целиком.

Размеры и offset задаются в world units. Перед evaluator они переводятся в
local metric только для положительного uniform scale. Non-uniform scale,
shear и mirror transform дают явный отказ до preview; исходному объекту нужно
применить scale или убрать reflection.

## CornerModel join contract

`MITER` и `BEVEL` доступны как compile-time режимы одной `CornerModel`.
Сохранённый `.blend`, script и operator-вызов проходят один и тот же compile;
UI не блокирует `BEVEL`, а тихая коэрция в `MITER` запрещена. Смена join у
уже скомпилированного plan даёт `DECAL_CORNER_JOIN_RECOMPILE_REQUIRED`.

Свои strip-сегменты не меняются между режимами; меняется только материя угла.
Для `BEVEL` семантический контур — ровно `V/P1/P2`, и чужая конкуренция вправе
дойти до хорды. `ResolvedCornerView` после arrangement читает те же anchors и
не выводит форму повторно.

## Modal contract

- горизонтальный drag вправо увеличивает `Seam Width`, влево уменьшает;
- `Shift` даёт одну десятую чувствительности с input rebase;
- минимум размера — `0.001` world units;
- X-anchor берётся из экранной проекции центра bbox источника, с viewport
  fallback; движение по Y не участвует;
- `W` редактирует ширину, `A` — acute split angle, `M` — apex limit;
- `A/M` доступны только чистому Patch Voronoi scope;
- LMB/Enter подтверждает последний валидный текущий кадр; RMB/Esc отменяет и
  восстанавливает settings snapshot;
- ошибка текущего кадра удаляет preview и блокирует confirm до нового
  успешного evaluation.

## Обновление установленного аддона

Decal engine и PatchGraph analysis обновляются единым пакетом `cftuv`.
Частичная замена файлов не поддерживается. После синхронизации каталога Blender
нужно перезапустить или перезагрузить все analysis/decal modules вместе, затем
сверить SHA-256 установленного пакета с repository checkout.

Patch Voronoi требует `pyvoronoi >= 1.2.8` в Python environment Blender.
Отсутствие wheel даёт явный compile fail и никогда не включает legacy
fallback.

## Verification

Минимальный affected-module gate:

```powershell
python -m pytest -q tests/test_decals.py tests/test_decal_voronoi.py `
  tests/test_decal_rail_geometry.py tests/test_operators.py
```

Field gate выполняется без сохранения `.blend` на canonical six-object scene и
на `artifacts/cftuv_decal_verification.blend`. Для S0b обязательный
differential сравнивает до/после:

- semantic digest compiled plan;
- детерминированную serialization face batch;
- policy/fallback counters;
- preview и confirm output.

Финальный арбитр остаётся пользовательский visual pass в живой Blender-сцене.
