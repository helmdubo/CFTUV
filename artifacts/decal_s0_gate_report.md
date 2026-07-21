# TRANCHE S — S0 gate report

## Outcome

S-guard, S0a и S0b выполнены на implementation branch
`codex/decal-engine-tranche-s0`. Production runtime оставляет только compiled
strict `SEAMS`; `TOP`, `BOTTOM`, `CORNERS` и scripted/persisted `BEVEL`
fail closed с именованными причинами.

Исходный commit `445b44d` отсутствовал в доступном clone/origin, поэтому S0a
был воспроизведён от canonical amendment base `2831a22`, без чтения archive.
Это задекларированный малый pivot: наблюдаемый результат и требуемая
фазировка сохранены, но commit ancestry не является ancestry контейнера
предыдущего исполнителя.

## Archive evidence

- remote branch `archive/legacy-decal-generations` →
  `2831a223224c2e84bc39c9c405c8aefe459fd7c1`;
- immutable annotated tag
  `archive/legacy-decal-generations-pre-s0-20260720`;
- tag object `f881bebca7e0a9f57bbf3573d218e6e627004281`;
- peeled commit `2831a223224c2e84bc39c9c405c8aefe459fd7c1`;
- tag message: `Immutable pre-S0 snapshot: legacy decal generations`.

## Commit ledger

- `64593ed` — runtime guard для archived `BEVEL`;
- `746da7e` — S0a capability cut и удаление historical decal network;
- `11bf6f4` — product contract архивированных modes до удаления builders;
- `c9d7449` — S0b physical deletion direct builders;
- `9b0c53a` — сохранённые after-receipts и differential comparison.

## S0b differential

Before commit: `11bf6f44d387bf8f4fa4b6f01b01556e4868b97d`.
After commit: `c9d74499839a99a76732cc8b7c16a615502dcd62`.

| Suite | Objects | Plan digest | Face-batch serialization | Counters | Preview == confirm |
|---|---:|---|---|---|---|
| canonical field | 6 | before == after | before == after | before == after | yes |
| saved verification | 1 | before == after | before == after | before == after | yes |

`GeometryBatch` вводится только в следующем S2a; для pre-S2a S0 сравнивается
его текущий канонический эквивалент — полная детерминированная serialization
`DecalGeometryFace` batch. Comparison reports:

- [decal_s0b_comparison_field.json](decal_s0b_comparison_field.json);
- [decal_s0b_comparison_verification.json](decal_s0b_comparison_verification.json).

Оба отчёта имеют `semantic_equal=true` и
`all_preview_confirm_equal=true`. `.blend` не сохранялись.

## Test gates

- affected modules before commit: `272 passed, 3 skipped`;
- post-cut `tests/test_decals.py`: `79 passed`, включая regression на
  физическое отсутствие 7 archived builder symbols;
- tier 2, единственный полный прогон:
  `444 passed, 3 skipped, 4 deselected` с
  `-m "not atlas_frozen"`;
- Blender 4.3.2 / Python 3.11.9 / `pyvoronoi 1.2.8`;
- canonical six-object render: `CFTUV_R13_FIELD_RENDERS=6`, exit code 0.

## Visual field gate

S0 — структурный no-op, а не hotfix по аннотированному пользовательскому
скрину, поэтому стрелочного дефекта §0d.0 нет. Проверка для каждого ракурса:
`strict SEAMS до → тот же plan/face batch после`; это доказано differential
выше. After-renders просмотрены вручную:

- [rounded_wall.001](decal_r13_field_rounded_wall_001.png);
- [rounded_wall_noise_top](decal_r13_field_rounded_wall_noise_top.png);
- [sagging_wall](decal_r13_field_sagging_wall.png);
- [wall_noise_top](decal_r13_field_wall_noise_top.png);
- [half_sphere](decal_r13_field_half_sphere.png);
- [building](decal_r13_field_building.png).

## Риски

1. Три canonical objects остаются именованно неподдержанными на этом base:
   `rounded_wall_noise_top`, `wall_noise_top`, `half_sphere`. Их exact failure
   reasons/counters до и после совпадают; это не S0-регрессия, но и не
   production geometry coverage. `rounded_wall.001`, `sagging_wall` и
   `building` материализуют geometry.
2. Field renderer для rejected plan показывает пустой batch и compile summary;
   production mesh transaction дополнительно отклоняет весь scope до BMesh.
   Differential сохраняет оба compile outcome, но screenshot сам по себе не
   демонстрирует operator exception.
3. Negative tombstone fields старых SEAMS plans сохранены, потому что они
   fail closed и входят в pre-S0 compiled-plan schema. Удаление их в S0b
   изменило бы plan digest; runtime backend или builder за ними не стоит.
4. `BEVEL` implementation code внутри будущего corner evaluator ещё существует,
   но недостижимо через production compile/evaluate entrypoints благодаря
   S-guard. Его пересмотр относится к `CornerModel`, не к S0.
5. `pyvoronoi 1.2.8` установлен в пользовательский Blender 4.3 addon-modules
   для полноценной полевой проверки; это изменение среды, не репозитория.
