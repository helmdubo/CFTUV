# R2.1 hotfix gate — RF29b + RF30 + RF31

Base: `e921fef`

Candidate implementation: `1b2d411`

Field snapshot: `cftuv_r21_user_field.blend`, SHA-256
`7db56e359deab0cca39426cabee66e16620f6de31aba68cdca3c19eed30c480f`

Blender: `4.3.2`; source `.blend` was never saved.

## Поправка к предыдущему отчёту

Предыдущее особое мнение по стрелкам на `walls.001` **ОТОЗВАНО**.
Это был неверный тестовый меш: планарный `walls.001` не воспроизводит
вариативность поверхности, на которой пользователь показал дефект.
Каноническая полевая фикстура R2.1 — `sagging_wall`. На ней диагноз RC5b
подтвердился, а живое выделение второго скрина было зафиксировано как
точная пара `e7 + e26`.

## Результат

- **RF30 / RC5b исправлен на `sagging_wall`.** План компилирует 33
  immutable mutual-arrival atoms. В runtime counted-событие
  `MUTUAL_ARRIVAL_RELEASE` наблюдается 16 раз по dense sweep: статический
  локус не режет материю до прибытия второго владельца. На общей
  pChain-станции `e11` скомпилирован точный `TERMINAL_SITE_PAIR` в
  `t=0.5`, arrivals `0.5394465099 / 0.5394465099`.
- **RF31 / встреча двух pChain исправлена.** На точном пользовательском
  выделении `sagging_wall e7+e26` два terminal routes приходят с разных
  концов одного selected site, но проходят по разным физическим boundary
  routes. Прежний route-pair detector поэтому не создавал границу встречи.
  Теперь site несёт станционный freeze-locus `e7, t=0.5`, arrivals
  `0.7271339594 / 0.7271339594`.
- **Переход strip -> бирюзовый CAP устранён.** На base при width `1.6`
  появляются 2 CAP, а с `2.4` — 4 CAP и только 2 SEGMENT. После фикса на
  всех 22 ширинах `0.2..36`, и в MITER, и в BEVEL, materialized output
  содержит `CAP=0`. Когда BODY короткого site полностью поглощён,
  встречные START/END CAP одного source edge получают общий station-UV и
  остаются SEGMENT (`CAP_PAIR_ALIGNED`), сохраняя саму freeze-границу.
- **RF29b исправлен.** `sagging_wall e6,e7,e8`, widths
  `7.8,12,18,24,30,36`: terminal bridge насыщается на последней
  конструктивной станции и публикует counted
  `TERMINAL_ROUTE_STATION_CLAMPED`; `TERMINAL_BRIDGE_CUT_INVALID` не
  выбрасывается.
- Все 72 width-records кандидата имеют `status=OK` и полные
  GeometryBatch-digests `preview == confirm`; итоговый gate `green: true`.

## Структурная причина «веера» и CAP

В этом случае реальной поверхности-коллайдера между декалями нет.
Fan-подобный рисунок был следствием двух связанных ошибок ownership:

1. RC1 умел встретить потоки на общем physical route, но не два pChain
   terminal, входящих с противоположных концов одного selected spine site
   через разные routes. Поэтому станционная граница на самом site
   отсутствовала.
2. После исчезновения BODY оставались START/END CAP. Они продолжали
   материализоваться как торцевые поверхности и триангулировались как
   отдельные pieces — отсюда бирюзовый цвет и визуальный fan.

Исправление не добавляет воображаемую поверхность и не выбирает границу по
геометрической близости. Оно компилирует структурную встречу по
`source_edge + противоположные endpoints`, а затем переносит оба торца на
одну станцию. Компонентные sides `CAP_PAIR_START_ALIGNED` /
`CAP_PAIR_END_ALIGNED` намеренно сохраняют station edge при последующем
merge — инвариант S1 не нарушен.

## RC5b и I6

Для point-cell compile IR хранит reciprocal owner regions. В runtime
release вычисляется как `arrived corner crop - blocking material`; часть
freeze-локуса активна только там, где обе material waves уже существуют.
Это pointwise mutual-arrival gate, а не растровое поле и не новый выбор
join.

На base прямой evaluator падает одинаково при `preview=True` и
`preview=False`; «preview прошёл, Done упал» было last-valid изображением
UI. Структурный разрыв находился между CapacityPolicy (проверяла только
исчерпание route) и поздней проверкой constructibility terminal bridge.
Единый pre-materialization resolver теперь валидирует requested guide,
на `SATURATE_PROVEN` откатывает station-prefix до последнего
конструктивного, а истинно неконструируемый prefix сохраняет именованный
fail. Preview и confirm читают один validated guide.

## Evidence

- `decal_r21_hotfix_before.json`: base `e921fef`, правильная фикстура
  `sagging_wall`; RF31 даёт до 4 CAP, RF30 имеет четыре error-width,
  RF29b падает на 6/6 ширинах.
- `decal_r21_hotfix_after.json`: candidate `1b2d411`; все пять oracle
  flags true, `green: true`.
- `decal_r21_pchain_drag_before.png` / `after.png`: одинаковые camera,
  `sagging_wall e7+e26`, MITER, width `2.4`; `4 CAP + 2 SEGMENT` против
  `6 SEGMENT`.
- `decal_r21_drag_e7_e26_before.json` / `after.json`: полный labeled
  dense-width dump materialized faces, vert keys, provenance и policy.
- `decal_r21_rf30_inspect.json`: one-sided material witnesses и 33
  compile-static mutual-arrival records на широком `sagging_wall` case.
- `verify_decal_r21_hotfix.py`: воспроизводимый read-only acceptance gate.
- `inspect_decal_r21_drag.py` и `inspect_decal_r21_rf30.py`: подробные
  labeled witnesses для dense drag и mutual-arrival.
- `render_decal_r21_pchain_drag.py`: воспроизводимый read-only wireframe.
- Targeted rail/voronoi tests: `168 passed`.
- Full non-atlas suite: `500 passed, 3 skipped, 4 deselected`.

## Риски

1. `TERMINAL_SITE_PAIR` намеренно узок: ровно два terminal uses,
   противоположные endpoints одного source edge и разные physical routes.
   T/X или неоднозначная группа остаётся fail-closed и должна решаться
   обычной RC1-конкуренцией, а не midpoint-эвристикой.
2. CAP всё ещё легален на настоящем физическом конце route. В SEGMENT
   переводится только доказанная START/END пара одной surface и одного
   source edge; совпадение по координатам недостаточно.
3. Mutual-arrival увеличивает compile IR на intrinsic point-cells. Runtime
   остаётся без растрового поля, но стоимость больших selection следует
   продолжать отслеживать на старте R3.
4. Station clamping теперь может произойти до полного исчерпания route —
   это ожидаемая bridge-capacity semantics, наблюдаемая counted policy.

## Особое мнение

Возражений против пользовательского полевого факта нет: после перехода на
правильный `sagging_wall` он воспроизводится детерминированно. Узкое
уточнение терминологии: «fan» здесь описывает вид materialized CAP pieces,
но не доказывает существование физической поверхности-конкурента. Поэтому
фикс основан на station ownership/provenance, а не на добавлении
геометрического collision surface.
