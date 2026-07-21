# S-WF0: методика Surface Wavefront spike

Статус: research-only. Этот каталог не импортируется из `cftuv/` и не меняет
production routing. Решение A/B/C принимает пользователь после чтения данных.

## Вопрос

Нужно проверить, является ли привязка rail к рёбрам меша необходимой частью
геометрической семантики или артефактом тесселяции на гладкой кривизне.
Сравниваются четыре независимых семейства:

1. `CURRENT_RAIL_CHART` — настоящий `compile_decal_rail_plan` для событий и
   freeze-loci плюс текущий hinge-unroll chart для width/source-s. Это не
   графовый Dijkstra и не переписанная модель текущего движка.
2. `HEAT` и `FMM` — geometry-central через pinned `potpourri3d`. Поле
   unsigned; owner/source-s восстанавливаются отдельными labeled solves.
   Поле не выбирает join и не материализует topology.
3. `MMP_EXACT` — `pygeodesic`, точная полиэдральная геодезика от дискретного
   множества source-вершин. Источник на фикстурах согласован с рядами query-
   вершин; результат не называется точным расстоянием до произвольной точки
   непрерывного сегмента.
4. `STRAIGHT_SKELETON_2D` — `py_straight_skeleton` только для простых
   планарных polygon-fronts. Неприменимость к T/X-PSLG записывается как
   `unsupported`, а не подменяется distance-field результатом.

Зависимости живут только в отдельном research venv. Аддон их не получает.

## Фикстуры

| fixture | проверяемая развилка |
|---|---|
| `planar_l` | concave L polygon, точные planar width и skeleton events |
| `planar_tx` | labeled T/X source network и merge/cut topology |
| `concave_owner` | запрет утечки через вырез owner-domain |
| `cylinder_one_seam` | periodic cut locus и double-cover |
| `half_sphere` | гладкая положительная кривизна |
| `close_parallel_sheets` | intrinsic owner isolation против ambient-nearest |
| `retriangulated_surface/a,b` | одна и та же developable quad-surface с двумя диагонализациями |
| `fold_near_smooth` | геометрический fold рядом с рёбрами гладкой тесселяции |

У пары `retriangulated_surface` совпадают вершины и сами planar quads, поэтому
смена диагонали не меняет embedded piecewise-planar surface. Сравнение не
смешивает ретриангуляцию с заменой геометрии.

## Замороженные метрики

Все ошибки нормируются на `alpha_reference` фикстуры, если не указано иное.

- `width_error_{mean,p95,max}`: `|d_method - d_MMP| / alpha_reference` на
  валидных query-вершинах.
- `owner_error_rate`: доля query без exact tie, где labeled owner отличается
  от MMP. Tie определяется из разницы первого и второго exact owner-distance.
- `source_s_error_{mean,p95,max}`: абсолютная ошибка накопленной станции,
  нормированная на полную длину source-network; ambiguous exact ties исключены.
- `event_sequence_edit_distance` и `event_alpha_error_max`: сравнение
  детерминированной PL-wavefront sequence (`MERGE`, `BOUNDARY`, `FREEZE`) с
  MMP; для current rail дополнительно сохраняется нативная RR event sequence.
- `cut_freeze_locus_f1`: F1 по canonical mesh-edge keys относительно MMP.
- `double_cover_count`: chart triangle overlaps плюс неоднозначные внутренние
  покрытия вне канонического cut locus. Сам cut locus double-cover не считается.
- `boundary_leak_count`: материализуемые sample-вершины вне owner-domain или
  конечное поле на другом disconnected owner-sheet.
- `compile_ms`, `extract_ms`: медиана отдельных стадий после одного warm-up;
  число повторов фиксировано harness-ом и сохраняется в receipt.
- `retriangulation_*`: пары значений в общих vertex/sample keys между `a` и
  `b`; главные числа — max width delta, owner disagreement, source-s delta,
  event sequence equality и locus F1.

Topology extraction использует только векторные mesh keys и labeled scalar
values: lower-star merges, boundary hits и local-max freeze ridges. Растровое
поле, marching pixels и screen-space tolerance запрещены.

## Правило рекомендации

- **A — поле только оракул:** поле не даёт устойчивого выигрыша по intrinsic
  width/retriangulation либо ломает owner/provenance/topology.
- **B — поле генерирует R2-события, rails материализуют:** поле заметно лучше
  по width/retriangulation, но не доказывает полный materialization contract
  (locus, owner, source-s, double-cover, boundary isolation).
- **C — SurfaceWavefront как R3 backend:** поле одновременно улучшает
  width/retriangulation и сохраняет полный labeled/topological contract на
  всех применимых фикстурах. Planar exact backend при этом остаётся отдельным.

«Заметно» означает эффект больше удвоенного observed repeat/retriangulation
noise floor и больше `1e-4 * alpha_reference`; это критерий различимости
измерения, не production-порог поведения.

## Воспроизведение

```powershell
python -m venv --system-site-packages "$env:TEMP\cftuv_s_wf0_env_313"
& "$env:TEMP\cftuv_s_wf0_env_313\Scripts\python.exe" -m pip install -r research/s_wf0/requirements.txt
& "$env:TEMP\cftuv_s_wf0_env_313\Scripts\python.exe" research/s_wf0/run_spike.py
```

Harness пишет JSON/CSV только по явно переданному `--output-dir`; дефолт —
`artifacts/s_wf0`.
