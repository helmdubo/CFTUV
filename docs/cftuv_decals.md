# CFTUV Decal Producer (Phase 3)

> **TRANCHE S status.** Единственный production mode — compiled strict
> `SEAMS`. `TOP`, `BOTTOM` и `CORNERS` архивированы до пересдачи через
> mode-specific plan → `GeometryBatch` → общие adapters и дают именованный
> fail до analysis/BMesh. Их описание ниже временно сохраняет наблюдаемую
> продуктовую семантику для `docs/decal_archived_modes_product_contract.md`;
> это не описание доступного runtime.

## Обновление установленного аддона

Decal producer и PatchGraph analysis обновляются только как единый пакет
`cftuv`. Частичная замена отдельных файлов не поддерживается: например, новый
`decals.py` при старом `analysis_boundary_loops.py` получает пустые
`BoundaryChain.side_face_normals` и вынужденно возвращается к patch-average
normal. На wrapped WALL patch это снова смешивает TOP/BOTTOM и совмещает оба
крыла CORNERS. После обновления всего каталога аддона Blender нужно
перезапустить либо перезагрузить все analysis/decal modules вместе.

## Ручной режим по Edge Select Mode

Отдельного boolean-переключателя нет. В Edit Mode режим выбора определяет scope:

1. Face Select Mode — автоматическая генерация по выбранным faces.
2. Edge Select Mode — ручная генерация по выбранным seam-marked edges.
3. `PatchGraph` строится по всему mesh, поэтому face selection не требуется и не
   обрезает соседние patches.
4. Каждое выбранное seam-ребро является атомарным manual scope. Его две
   owner-side записи находятся внутри канонических `BoundaryChain` и
   спариваются по исходному mesh edge index. Поэтому разная сегментация chains
   на двух patches не расширяет выбор до целой петли и не создаёт дубли.
5. Выбранные seam edges сохраняются выделенными после генерации для повторной
   настройки или создания другого decal mode на том же scope.
6. Последовательные выбранные edges с endpoint-valence `2` собираются в один
   oriented corner run. На общей станции каждое крыло пересекает offset-линии
   соседних сегментов и получает constant-width MITER. Слишком длинное или
   неплоское skew-пересечение автоматически превращается в BEVEL с локальной
   соединительной гранью. Point-contact и valence `3+` не выбирают
   произвольную «главную» ветку: chain-runs остаются независимыми. В режиме
   `Decal Seams` сеть по умолчанию строится единым α-clipped partition
   (см. «SEAMS network backend»); ветви разделяются биссектрисами на каждой
   owner surface, а не сшиваются постфактум.

Для `Decal Corners` и `Decal Seams` ручной режим не фильтрует `PatchType`: WALL,
FLOOR, SLOPE, потолки и другие ориентации равноправны. PATCH-neighbor chain и
две стороны одного `SEAM_SELF` создают двухкрылый corner-strip по двум локальным
поверхностям. Непарный `MESH_BORDER/SEAM_SELF` chain создаёт одно corner-крыло
внутрь owner patch. `Decal Corners` использует `Corner Width` и corner UV rect;
`Decal Seams` — `Seam Width` и seam UV rect. На неплоском стыке seam состоит
из двух локальных полукрыльев, а на копланарном — из одной центрированной
ленты. Стороны общего физического ребра дедуплицируются по source edge index.
При stitching сторона A/B выравнивается по непрерывности локальных нормалей, а
не по patch id, поэтому смена chain/neighbor вдоль run не переставляет крылья.

### SEAMS legacy runtime удалён

`decal_seam_network`, compiled `LEGACY_NETWORK`, one-shot seam network и
прямой miter/junction pipeline больше не являются runtime-вариантами SEAMS.
Старое Blender-свойство остаётся только для чтения прежних `.blend`, скрыто
из панели и не влияет на поведение. Исторический `decal_network.py` удалён из
рабочей ветки и сохранён только архивным tag. Manual SEAMS принимает только явно
скомпилированные `RAIL_PLANAR` и `PATCH_VORONOI` partitions.

Неподдержанный topology component получает component-wide `Failed` с
канонической compile-причиной. Один `Failed` атомарно отклоняет весь выбранный
SEAMS scope до evaluation/BMesh. Если текущий drag-кадр падает, старый preview
удаляется: ошибка видна геометрически и в header/console, а не маскируется
last-valid mesh. Object/Face-mode automatic SEAMS без selected-edge plan также
явно отклоняется. TOP/BOTTOM/CORNERS сейчас недоступны и возвращают
mode-specific `DECAL_*_ARCHIVED_UNTIL_ENGINE_PLAN`.

User-facing routing включает детерминированные счётчики каждой canonical
failure reason (`Failed:Ne[REASON:xN]`), а exception дополнительно показывает
physical edge IDs; причины не спрятаны за verbose console.

Перед evaluation runtime дополнительно сверяет compiled accounting и точное
равенство selected-edge scope. Устаревший plan для другого выделения не может
тихо материализовать только известное ему подмножество.

После такого `ERROR` LMB/Enter не может подтвердить предыдущий валидный размер:
confirm отменяется. Чтобы продолжить, нужно вернуть drag в поддержанный кадр;
он вычисляется заново и только тогда снова становится подтверждаемым.

## Интерактивный размер автоматических декалей

Кнопки `Decal Top`, `Decal Bottom` и `Decal Corners` видимы disabled как карта
архивированных capabilities. `Decal Seams` требует ручного seam-edge scope в
Edge Select Mode и compiled strict plan:

- Seams использует горизонтальный жест: движение мыши
  вправо увеличивает размер, движение влево уменьшает его.
- Seams меняет `Seam Width` обоих крыльев.
- Уменьшение ограничено положительным минимумом `0.001`, поэтому лента не
  схлопывается в нулевую геометрию.
- После нажатия кнопки X курсора выравнивается по экранной проекции центра
  bounding box исходного объекта. Y остаётся в точке клика: вертикального warp
  нет, и движение вверх/вниз не участвует в расчёте размера. Если центр объекта
  вне экрана или не проецируется, используется центральный X окна 3D View.
- `W` выбирает размер текущего mode, `A` — `Acute Split Angle`, `M` —
  `Apex Limit`. Переключение цели использует текущее принятое значение и
  текущий X мыши как новую базу, поэтому параметр не скачет.
- Для manual `Decal Seams` targets `A/M` включаются только при clean Patch
  Voronoi scope (`Failed:0`). Для rail scope `A/M` отключены. Любой failed
  component останавливает весь modal до материализации и показывает причину.
- При активном `A` header показывает текущий порог и evaluator diagnostics:
  `Acute Split: <deg> | MITER:<n> KITE:<n> FAN:<n> SPLIT:<n> HAIRPIN:<n> | <ms>`.
  Перетаскивание переоценивает уже compiled Patch Voronoi plan без нового
  `Construct()`; invalid SEAMS frame удаляет preview и показывает ошибку.
- Panel задаёт четыре ordered band thresholds: `Miter/Kite/Split/Hairpin
  Angle` с defaults `120/90/60/30°`. Они выбирают пять semantic corner
  policy по `extrusion_angle`; точное равенство относится к более мягкому
  band. `A` остаётся live target для Split Angle, остальные thresholds входят
  в immutable settings snapshot текущего запуска.
- `Shift` включает точную регулировку с input rebase как при нажатии, так и
  при отпускании: уже накопленный coarse delta не пересчитывается с новой
  sensitivity. Угол в header показывается в градусах.
- LMB/Enter подтверждает общий последний валидный settings snapshot. RMB/Esc
  отменяет preview и восстанавливает все W/A/M properties, изменённые в этой
  modal-сессии.

В Edge Select Mode `Decal Seams` также остаётся интерактивным: captured manual
scope не меняется во время drag, повторно строятся только выбранные seam edges.
Архивированные manual modes не запускают preview или immediate generation.

Генерация mesh-декалей (тримы, углы, швы) из PatchGraph. Логика перенесена из
прототипа `hotspotingUV_mesh_decals_Full.py` (v1.2.0, «Global System») и
переписана нативно на PatchGraph-цепочки CFTUV.

## Место в архитектуре

```text
operators.py
  → _prepare_patch_graph()          # edit-mode BMesh → PatchGraph
  → DecalSettings.from_blender_settings()
  → decals.generate_decal_objects() # PatchGraph → decal mesh объекты
```

- `decals.py` — чистый потребитель PatchGraph (как `debug.py`): исходный
  BMesh не читается, вся геометрия берётся из `vert_cos` цепочек (локальное
  пространство source object).
- `decal_modal.py` — единая экранная математика и immutable target descriptors:
  проекция bbox, viewport fallback, cursor warp, W/A/M sensitivity и clamps.
- Модуль строит ленты в собственном `bmesh.new()` и материализует их
  объектами в коллекции `Decals_Generated` с `matrix_world` источника.
- Settings/modal задаются в world units. Перед backend они переводятся в
  local metric через проверенный positive uniform scale source object:
  размеры и offset делятся на scale, а `uv_length_scale` умножается на scale.
  Non-uniform scale, shear и mirror transform отклоняются до preview с явной
  ошибкой; применить scale либо убрать reflection нужно на исходном объекте.
- Настройки — frozen dataclass `DecalSettings` в `model.py` (инвариант «без
  глобалов»); пороги и UV-ректы — в `constants.py` (`DECAL_*`).
- Повторная генерация заменяет одноимённый объект (`Decal_<Mode>_<Source>`),
  как delete-and-recreate в debug слое.

## Режимы (оператор `hotspotuv.generate_decals`, mode enum)

### TOP / BOTTOM — тримы кромок стен

1. Для каждого WALL patch берутся цепочки всех boundary loops (OUTER и HOLE —
   проёмы окон/дверей тоже получают тримы), у которых сосед — `MESH_BORDER`
   или patch не-WALL типа (FLOOR/SLOPE). WALL-WALL швы исключены: их
   обслуживают corner/seam декали (в прототипе на горизонтальных WALL-WALL
   стыках возникали двойные ленты — здесь это устранено).
2. Каждое ребро цепочки классифицируется по analysis-owned normal его
   `side_face`: `outward = edge_dir × side_face_normal`; локальный `up` —
   проекция `WORLD_UP` на эту поверхность. Выше `DECAL_DIR_THRESHOLD` —
   верхняя кромка, ниже минус порога — нижняя. `patch.normal` используется
   только как запасная compatibility-normal, если локальный sample отсутствует. Это важно
   для wrapped WALL patch: один patch может содержать весь периметр здания,
   и его средняя normal не описывает ни одну конкретную стену. Рёбра короче
   `DECAL_NOISE_THRESHOLD` пропускаются.
3. Каждый сегмент сохраняется как часть ориентированного ribbon-run вместе с
   `ChainRef`, owner-normal и owner-up. Runs соединяются через совпадающие
   endpoints; вершины с неоднозначным point-contact (`degree != 2`) разрывают
   путь вместо произвольного жадного продолжения.
4. Вдоль run строится лента: база = точка + `normal * offset`, вытяжка вниз
   (TOP) / вверх (BOTTOM) на `height_trim` вдоль адаптивного `up`. На стыке
   двух patches вершина получает биссектрису соседних owner-frames.
5. Если для непрерывного обхода часть run пришлось развернуть, каждый quad
   проверяет winding относительно сохранённой owner-normal. Winding меняется
   независимо от UV: `base` всегда остаётся на нижней стороне UV-ректа, `tip`
   — на верхней. Поэтому замкнутая лента не начинает отображаться с обратной
   стороны и не меняет верх/низ текстуры.

`BoundaryChain.side_face_normals` сериализуется analysis-слоем на финальных
chains и выровнен с `edge_indices`. Это геометрический факт owner-side без
живых `BMFace` ссылок; decal producer не читает исходный BMesh.

### CORNERS — углы WALL-WALL

Цепочки с `neighbor_kind == PATCH`, оба patch WALL и
`dot(n_a, n_b) ≤ DECAL_COPLANAR_DOT`, а также все парные chain uses
`neighbor_kind == SEAM_SELF` одного wrapped WALL patch. Для `SEAM_SELF` нет
углового порога: явный seam остаётся CORNERS при любой плотности tube mesh.
Спайн идёт вдоль полилинии цепочки (упорядочена в
PatchGraph — работает на изогнутых и Г-образных углах, в отличие от
прототипа, сортировавшего вершины вдоль одной оси), смещается вдоль средней
нормали на `offset / dot` (постоянный зазор до обеих плоскостей). Два крыла
шириной `width_corner / 2` ложатся на каждую стену; направления крыльев
считаются отдельно для входящего и выходящего сегментов. Внутренняя станция
строится как пересечение их offset-линий, поэтому на повороте сохраняется
постоянная ширина. `DECAL_CORNER_MITER_LIMIT` ограничивает острые шипы;
неустойчивые, слишком длинные и skew-пересечения получают BEVEL fallback.

### SEAMS — плоские швы WALL-WALL

Копланарные пары (`dot > DECAL_COPLANAR_DOT`): плоская лента `width_seam`,
центрированная на полилинии шва, со смещением `offset` по нормали. В
прототипе этот генератор существовал, но не был выведен в UI (мёртвый код) —
здесь режим включён.

Каждая пара поверхностей обрабатывается один раз. PATCH-пара канонизируется по
двум patch ids и source edge indices; `SEAM_SELF` — по owner patch id и source
edge indices двух chain uses. Поэтому один seam на замкнутой tube-поверхности
даёт одну двухстороннюю corner-декаль без искусственного разбиения patch.
Закрытые цепочки (кольцевые швы) замыкаются
дублированием первой точки, касательные/фреймы в точке замыкания считаются
с wrap-around (биссектриса последнего и первого сегментов), поэтому дубли
вершин совпадают и свариваются `remove_doubles` (`DECAL_WELD_DISTANCE`) —
кольцо сплошное, без щели на стыке.

## UV лент

Прямоугольники атласа `(u_min, v_min, u_max, v_max)`:

| Режим   | Константа              | Значение             |
|---------|------------------------|----------------------|
| TOP     | `DECAL_UV_RECT_TOP`    | (0.0, 0.8, 1.0, 1.0) |
| BOTTOM  | `DECAL_UV_RECT_BOTTOM` | (0.0, 0.0, 1.0, 0.2) |
| CORNERS | `DECAL_UV_RECT_CORNER` | (0.9, 0.0, 1.0, 1.0) |
| SEAMS   | `DECAL_UV_RECT_SEAM`   | (0.9, 0.0, 1.0, 1.0) |

Продольная координата — длина дуги × `uv_length_scale`. В отличие от
прототипа (голый `uv_scale`), множитель равен `UVSettings.final_scale`
(`texel_density / texture_size * uv_scale`) — плотность декалей согласована
с основным UV-пайплайном. У тримов вдоль ленты идёт U, у углов и швов — V
(как в прототипе); у угла спайн лежит на середине ректа.

## Настройки (панель, секция Decals)

| Свойство             | По умолчанию | Смысл                          |
|----------------------|--------------|--------------------------------|
| `decal_width_corner` | 0.20         | полная ширина угловой ленты    |
| `decal_width_seam`   | 0.15         | полная ширина шовной ленты     |
| `decal_height_trim`  | 0.25         | высота тримов                  |
| `decal_offset`       | 0.02         | отступ от поверхности          |

## Осознанные отличия от прототипа

1. WALL-WALL кромки не попадают в тримы (нет двойных лент на ledge-стыках).
2. Спайн угла — полилиния цепочки, а не сортировка вершин вдоль оси
   `n_a × n_b`; направления крыльев — по-вершинные.
3. Нормали пары — нормали patches (`node.normal`), а не среднее нормалей
   фейсов вдоль шва; для плоских стен идентично. Шовная лента, как и в
   прототипе, смещается по нормали стороны-владельца.
4. Дизъюнктные участки общей границы пары дают отдельные ленты по цепочкам
   (прототип сливал их в один спайн — артефакт).
5. UV-плотность вдоль ленты — `final_scale` вместо голого `uv_scale`.
6. Seams выведен в UI.
7. Две стороны внутреннего шва (`SEAM_SELF`) сопоставляются по одному физическому
   mesh edge и всегда образуют одну CORNERS-ленту. Это не зависит от угла между
   соседними face normals и плотности tube mesh; встречных дублей нет.
8. Направление лент хранится вместе с chain/owner-frame. Глобальный обход может
   развернуть run ради непрерывного U, но winding каждого quad валидируется
   относительно owner-normal без перестановки семантических base/tip UV.
9. Замкнутые кольца лент замыкаются без щели (wrap-around биссектриса в
   точке замыкания); прототип артефактно разрывал кольцо.

## Известные ограничения (унаследованы от прототипа)

- `DECAL_NOISE_THRESHOLD` применяется к отдельным рёбрам: на очень мелкой
  тесселяции кромки (рёбра < 0.05) трим-лента получает разрывы. При
  необходимости порог можно уменьшить в `constants.py`.
- Non-manifold геометрия должна чиститься заранее (`Clean Non-Manifold
  Edges`); дублирующиеся рёбра в trim-сборке отбрасываются first-wins.

## Верификация

- `tests/test_decals.py` — чистая геометрия (классификация кромок, сборка
  путей, биссектрисы, corner/seam разбор, дедупликация пар) на стабах
  `tests/conftest.py`, без Blender.
- Ручная проверка в Blender: Edit Mode → выделить seam edges → `Decal Seams`;
  объект появляется в `Decals_Generated`. Архивированные кнопки disabled.

### Пользовательский срез 1 — planar angle + closed trim orientation

1. Создать замкнутый четырёхстенный объём; угловые seams должны разделять стены
   на WALL patches. Выделить стены и выполнить `Decal Top`, затем `Decal Bottom`.
2. Проверить замкнутое кольцо со всех сторон: поверхность декали смотрит наружу,
   верх и низ рисунка не меняются местами ни на одной стене, UV идёт непрерывно
   до единственного seam замыкания.
3. Создать две плоские WALL поверхности, сходящиеся по seam под углом 60°/120°,
   и выполнить `Decal Corners`.
4. Проверить, что каждое крыло лежит в плоскости своей стены и угол раскрытия
   соответствует геометрии, а не фиксированным 90°.
5. Создать T-junction/point-contact из трёх подходящих TOP-кромок. Проверить,
   что генератор не выбирает случайную ветку: в неоднозначной вершине получаются
   отдельные ribbon-runs без самопересекающегося продолжения.

### Пользовательский срез SEAMS T-junction

1. В плоской поверхности отметить seams в форме `T`: две коллинеарные ветви и
   одна поперечная. Выделить все три chains в Edge Select и запустить
   `Decal Seams`.
2. Проверить, что ширина всех branches одинакова, в центре нет отверстия,
   перекрывающихся faces и диагонального сужения. Между соседними chains должна
   идти усреднённая линия `core → outer miter`, а не линия вдоль branch spine.
   Поперечных прямоугольных рёбер в месте бывшего branch endpoint быть не должно.
3. Повторить на пространственном стыке трёх owner surfaces. Все три локальных
   крыла должны прийти в один offset core без loose vertices.
4. Изменить `Seam Width` горизонтальным modal drag: junction должен
   перестраиваться вместе с branches и сохранять их точный selected scope.
