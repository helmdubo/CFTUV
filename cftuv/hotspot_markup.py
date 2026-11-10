"""Разметка Hotspot (HOTSPOT_MARKUP_V1): формат, чтение, запись и проверка. Без Blender.

Разметка лежит в .blend, в текстовом датаблоке (версионный JSON); материал ссылается на неё своим свойством (`hotspot_blender`).
Здесь только данные: что такое метка, как она пишется в JSON и почему не принята. Каждая причина отказа - имя (`MARK_*`, `MARKUP_*`),
проверка возвращает ВСЕ находки, а не первую.

ФОРМАТ (`format`, `version`), пиксели текстуры, начало - левый нижний угол, как UV Blender::

    {"format": "CFTUV_HOTSPOT_MARKUP", "version": 1,
     "texture": {"width": 1024, "height": 1024, "image": "trim.png"},
     "marks": [{"id": 0, "name": "drips", "kind": "trim", "rect": {"x": 0, "y": 0, "w": 1024, "h": 128}, "up": "+y"},
               {"id": 1, "name": "edge", "kind": "corner", "rect": {"x": 0, "y": 128, "w": 1024, "h": 96}, "up": "+y", "band": 12}]}

* `rect` - прямоугольник по осям в целых пикселях; `up` - направление «вверх» метки (`+y`, `-y`, `+x`, `-x`): толщина метки - её размер вдоль `up`,
  длина - поперёк. Метка повторяется по длине через wrap текстуры, поэтому её длина обязана занимать текстуру целиком (правило Valve).
* `trim`: нижний край (противоположный `up`) - шов, верхний - внешний край юбки. Толщина метки = ширина декали.
* `corner`: шов - средняя линия по толщине, юбки двух сторон шва - крылья бабочки, каждое на свою половину; `band` - ширина центральной полосы
  детали в пикселях (только для показа), не шире толщины.
* `next_id` (необязательно, HOTSPOT_MARK_ID_MONOTONIC_V1) - счётчик номеров: номер удалённой метки не достаётся новой. Разметка без него читается
  как прежде; номер новой метки - наибольший из счётчика, номеров меток плюс один и номеров, на которые ссылаются записи декалей файла.

Экспорт в файлы (в том числе Valve `.rect`) - позже: всё, что ему нужно, уже здесь (прямоугольник в пикселях, размер текстуры, направление).
"""

from __future__ import annotations

import colorsys

import json
from dataclasses import dataclass, field, replace

FORMAT_NAME = "CFTUV_HOTSPOT_MARKUP"
FORMAT_VERSION = 1

KIND_TRIM = "trim"
KIND_CORNER = "corner"
KINDS = (KIND_TRIM, KIND_CORNER)
#: `up` -> единичный вектор «вверх» метки в пикселях.
UP_VECTORS = {"+y": (0, 1), "-y": (0, -1), "+x": (1, 0), "-x": (-1, 0)}

# Имена отказов разметки целиком.
MARKUP_NOT_JSON = "MARKUP_NOT_JSON"
MARKUP_NOT_AN_OBJECT = "MARKUP_NOT_AN_OBJECT"
MARKUP_FORMAT_UNKNOWN = "MARKUP_FORMAT_UNKNOWN"
MARKUP_VERSION_UNSUPPORTED = "MARKUP_VERSION_UNSUPPORTED"
MARKUP_FIELD_UNKNOWN = "MARKUP_FIELD_UNKNOWN"
MARKUP_TEXTURE_INVALID = "MARKUP_TEXTURE_INVALID"
MARKUP_MARKS_INVALID = "MARKUP_MARKS_INVALID"
# Имена отказов метки.
MARK_NOT_AN_OBJECT = "MARK_NOT_AN_OBJECT"
MARK_FIELD_MISSING = "MARK_FIELD_MISSING"
MARK_FIELD_UNKNOWN = "MARK_FIELD_UNKNOWN"
MARK_ID_INVALID = "MARK_ID_INVALID"
MARK_ID_DUPLICATE = "MARK_ID_DUPLICATE"
MARK_NAME_INVALID = "MARK_NAME_INVALID"
MARK_KIND_UNKNOWN = "MARK_KIND_UNKNOWN"
MARK_UP_INVALID = "MARK_UP_INVALID"
MARK_RECT_INVALID = "MARK_RECT_INVALID"
MARK_RECT_EMPTY = "MARK_RECT_EMPTY"
MARK_RECT_OUT_OF_TEXTURE = "MARK_RECT_OUT_OF_TEXTURE"
MARK_RECTS_OVERLAP = "MARK_RECTS_OVERLAP"
MARK_REPEAT_WITHOUT_FULL_SPAN = "MARK_REPEAT_WITHOUT_FULL_SPAN"
MARK_BAND_INVALID = "MARK_BAND_INVALID"
MARK_BAND_ON_TRIM = "MARK_BAND_ON_TRIM"
MARK_CORNER_BAND_WIDER_THAN_RECT = "MARK_CORNER_BAND_WIDER_THAN_RECT"
#: HOTSPOT_MARK_ID_MONOTONIC_V1: счётчик номеров разметки (`next_id`) - не целое число >= 0.
MARKUP_NEXT_ID_INVALID = "MARKUP_NEXT_ID_INVALID"
#: HOTSPOT_MARK_FALLBACK_V1: номер псевдометки «вся текстура» (её нет в разметке, она только в записи меша); код грани `2 * id` влезает в INT32.
WHOLE_TEXTURE_MARK_ID = (1 << 30) - 1

PROBLEM_NAMES = (
    MARKUP_NOT_JSON, MARKUP_NOT_AN_OBJECT, MARKUP_FORMAT_UNKNOWN, MARKUP_VERSION_UNSUPPORTED, MARKUP_FIELD_UNKNOWN, MARKUP_TEXTURE_INVALID,
    MARKUP_MARKS_INVALID, MARK_NOT_AN_OBJECT, MARK_FIELD_MISSING, MARK_FIELD_UNKNOWN, MARK_ID_INVALID, MARK_ID_DUPLICATE, MARK_NAME_INVALID,
    MARK_KIND_UNKNOWN, MARK_UP_INVALID, MARK_RECT_INVALID, MARK_RECT_EMPTY, MARK_RECT_OUT_OF_TEXTURE, MARK_RECTS_OVERLAP,
    MARK_REPEAT_WITHOUT_FULL_SPAN, MARK_BAND_INVALID, MARK_BAND_ON_TRIM, MARK_CORNER_BAND_WIDER_THAN_RECT, MARKUP_NEXT_ID_INVALID,
)

_MARKUP_FIELDS = frozenset(("format", "version", "texture", "marks", "next_id"))
_TEXTURE_FIELDS = frozenset(("width", "height", "image"))
_MARK_REQUIRED = ("id", "name", "kind", "rect", "up")
_MARK_FIELDS = frozenset(_MARK_REQUIRED + ("band",))
_RECT_FIELDS = ("x", "y", "w", "h")


@dataclass(frozen=True, slots=True)
class MarkupProblemV1:
    """Одна находка проверки: имя, номер метки (`None` - разметка целиком) и подробность для человека."""

    name: str
    mark_id: int | None
    detail: str

    def text(self) -> str:
        where = "" if self.mark_id is None else f" (mark {self.mark_id})"
        return f"{self.name}{where}: {self.detail}"


@dataclass(frozen=True, slots=True)
class HotspotMarkV1:
    """Метка: прямоугольник `(x, y, w, h)` в пикселях, `up`, вид и ширина центральной полосы (`corner`; у `trim` - 0)."""

    id: int
    name: str
    kind: str
    x: int
    y: int
    w: int
    h: int
    up: str = "+y"
    band: int = 0

    @property
    def thickness(self) -> int:
        """Размер вдоль `up`: высота трима (`trim`) или обеих половин бабочки (`corner`)."""

        return self.h if self.up in ("+y", "-y") else self.w

    @property
    def length(self) -> int:
        """Размер поперёк `up`: вдоль него метка повторяется."""

        return self.w if self.up in ("+y", "-y") else self.h


@dataclass(frozen=True, slots=True)
class HotspotMarkupV1:
    """Разметка одной текстуры: размер в пикселях, имя изображения (для показа), метки по возрастанию `id` и счётчик номеров `next_id`
    (HOTSPOT_MARK_ID_MONOTONIC_V1: номер удалённой метки не достаётся новой; 0 - счётчика нет, разметка до него)."""

    width: int
    height: int
    image: str = ""
    marks: tuple = field(default=())
    next_id: int = 0

    def mark(self, mark_id: int):
        return next((item for item in self.marks if item.id == int(mark_id)), None)

    def with_marks(self, marks) -> "HotspotMarkupV1":
        return replace(self, marks=tuple(sorted(marks, key=lambda item: item.id)))


def _is_int(value) -> bool:
    return isinstance(value, int) and not isinstance(value, bool)


def _problem(problems: list, name: str, mark_id, detail: str) -> None:
    problems.append(MarkupProblemV1(name, mark_id, detail))


def _overlaps(a: HotspotMarkV1, b: HotspotMarkV1) -> bool:
    return a.x < b.x + b.w and b.x < a.x + a.w and a.y < b.y + b.h and b.y < a.y + a.h


def validate_markup(markup: HotspotMarkupV1) -> tuple:
    """Все находки разметки (`MarkupProblemV1`), пусто - разметка принята. Порядок детерминирован: разметка, затем метки по порядку, затем пары."""

    problems: list = []
    if not (_is_int(markup.width) and _is_int(markup.height) and markup.width > 0 and markup.height > 0):
        _problem(problems, MARKUP_TEXTURE_INVALID, None, f"texture size {markup.width!r} x {markup.height!r} is not two positive integers")
        return tuple(problems)
    if not _is_int(markup.next_id) or markup.next_id < 0:
        _problem(problems, MARKUP_NEXT_ID_INVALID, None, f"next_id {markup.next_id!r} is not a non-negative integer")
    seen: dict = {}
    for mark in markup.marks:
        _check_mark(problems, markup, mark)
        if _is_int(mark.id):
            if mark.id in seen:
                _problem(problems, MARK_ID_DUPLICATE, mark.id, f"id {mark.id} is used by more than one mark")
            seen.setdefault(mark.id, mark)
    sized = [mark for mark in markup.marks if all(_is_int(value) for value in (mark.x, mark.y, mark.w, mark.h)) and mark.w > 0 and mark.h > 0]
    for at, first in enumerate(sized):
        for second in sized[at + 1:]:
            if _overlaps(first, second):
                _problem(problems, MARK_RECTS_OVERLAP, second.id, f"mark {second.id} overlaps mark {first.id}")
    return tuple(problems)


def _check_mark(problems: list, markup: HotspotMarkupV1, mark: HotspotMarkV1) -> None:
    mark_id = mark.id if _is_int(mark.id) else None
    if not _is_int(mark.id) or mark.id < 0:
        _problem(problems, MARK_ID_INVALID, mark_id, f"id {mark.id!r} is not a non-negative integer")
    elif mark.id >= WHOLE_TEXTURE_MARK_ID:
        _problem(problems, MARK_ID_INVALID, mark_id, f"id {mark.id} is reserved (ids from {WHOLE_TEXTURE_MARK_ID} on belong to the whole-texture fallback)")
    if not isinstance(mark.name, str):
        _problem(problems, MARK_NAME_INVALID, mark_id, f"name {mark.name!r} is not a string")
    if mark.kind not in KINDS:
        _problem(problems, MARK_KIND_UNKNOWN, mark_id, f"kind {mark.kind!r} is not one of {', '.join(KINDS)}")
    if mark.up not in UP_VECTORS:
        _problem(problems, MARK_UP_INVALID, mark_id, f"up {mark.up!r} is not one of {', '.join(UP_VECTORS)}")
    if not all(_is_int(value) for value in (mark.x, mark.y, mark.w, mark.h)):
        _problem(problems, MARK_RECT_INVALID, mark_id, "rect x, y, w, h must be integers (pixels)")
        return
    if mark.w <= 0 or mark.h <= 0:
        _problem(problems, MARK_RECT_EMPTY, mark_id, f"rect {mark.w} x {mark.h} has no area")
        return
    if mark.x < 0 or mark.y < 0 or mark.x + mark.w > markup.width or mark.y + mark.h > markup.height:
        _problem(problems, MARK_RECT_OUT_OF_TEXTURE, mark_id, f"rect ({mark.x}, {mark.y}, {mark.w}, {mark.h}) leaves the {markup.width} x {markup.height} texture")
    elif mark.up in UP_VECTORS:
        along_y = mark.up in ("+y", "-y")
        start, span, full = (mark.x, mark.w, markup.width) if along_y else (mark.y, mark.h, markup.height)
        if start != 0 or span != full:
            axis = "x" if along_y else "y"
            _problem(problems, MARK_REPEAT_WITHOUT_FULL_SPAN, mark_id, f"the mark repeats along {axis} and must span the texture: {start}..{start + span} of 0..{full}")
    if not _is_int(mark.band) or mark.band < 0:
        _problem(problems, MARK_BAND_INVALID, mark_id, f"band {mark.band!r} is not a non-negative integer")
    elif mark.kind == KIND_TRIM and mark.band:
        _problem(problems, MARK_BAND_ON_TRIM, mark_id, "a trim mark has no centre band")
    elif mark.kind == KIND_CORNER and mark.up in UP_VECTORS and mark.band > mark.thickness:
        _problem(problems, MARK_CORNER_BAND_WIDER_THAN_RECT, mark_id, f"band {mark.band} px is wider than the mark thickness {mark.thickness} px")


def markup_to_dict(markup: HotspotMarkupV1) -> dict:
    marks = []
    for mark in sorted(markup.marks, key=lambda item: item.id):
        item = {"id": mark.id, "name": mark.name, "kind": mark.kind, "rect": {"x": mark.x, "y": mark.y, "w": mark.w, "h": mark.h}, "up": mark.up}
        if mark.kind == KIND_CORNER:
            item["band"] = mark.band
        marks.append(item)
    data = {"format": FORMAT_NAME, "version": FORMAT_VERSION, "texture": {"width": markup.width, "height": markup.height, "image": markup.image}, "marks": marks}
    if _is_int(markup.next_id) and markup.next_id > 0:
        data["next_id"] = markup.next_id
    return data


def save_markup(markup: HotspotMarkupV1) -> tuple:
    """`(текст JSON, находки)`: текст только у принятой разметки, иначе `None` и все находки (пишущий НЕ пишет: прежний текст остаётся)."""

    problems = validate_markup(markup)
    if problems:
        return None, problems
    return json.dumps(markup_to_dict(markup), indent=1, ensure_ascii=False) + "\n", ()


def _mark_from(problems: list, at: int, item):
    if not isinstance(item, dict):
        _problem(problems, MARK_NOT_AN_OBJECT, None, f"marks[{at}] is not an object")
        return None
    mark_id = item.get("id") if _is_int(item.get("id")) else None
    missing = [name for name in _MARK_REQUIRED if name not in item]
    if missing:
        _problem(problems, MARK_FIELD_MISSING, mark_id, f"marks[{at}] has no {', '.join(missing)}")
        return None
    unknown = sorted(set(item) - _MARK_FIELDS)
    if unknown:
        _problem(problems, MARK_FIELD_UNKNOWN, mark_id, f"marks[{at}] has unknown fields {', '.join(unknown)}")
    rect = item["rect"]
    if not isinstance(rect, dict) or set(rect) != set(_RECT_FIELDS):
        _problem(problems, MARK_RECT_INVALID, mark_id, f"marks[{at}].rect must be an object with exactly {', '.join(_RECT_FIELDS)}")
        return None
    return HotspotMarkV1(item["id"], item["name"], item["kind"], rect["x"], rect["y"], rect["w"], rect["h"], item["up"], item.get("band", 0))


def load_markup(text: str) -> tuple:
    """`(разметка | None, находки)` из текста JSON. Разметка возвращается и с находками проверки (её можно показать и править), `None` - только когда
    текст не читается как разметка вовсе (не JSON, чужой формат, версия, поля верхнего уровня)."""

    problems: list = []
    try:
        data = json.loads(text)
    except (TypeError, ValueError) as exc:
        return None, (MarkupProblemV1(MARKUP_NOT_JSON, None, str(exc)),)
    if not isinstance(data, dict):
        return None, (MarkupProblemV1(MARKUP_NOT_AN_OBJECT, None, "the markup must be a JSON object"),)
    if data.get("format") != FORMAT_NAME:
        return None, (MarkupProblemV1(MARKUP_FORMAT_UNKNOWN, None, f"format {data.get('format')!r} is not {FORMAT_NAME}"),)
    if data.get("version") != FORMAT_VERSION:
        return None, (MarkupProblemV1(MARKUP_VERSION_UNSUPPORTED, None, f"version {data.get('version')!r}; this build reads version {FORMAT_VERSION}"),)
    unknown = sorted(set(data) - _MARKUP_FIELDS)
    if unknown:
        return None, (MarkupProblemV1(MARKUP_FIELD_UNKNOWN, None, f"unknown fields {', '.join(unknown)}"),)
    texture = data.get("texture")
    if not isinstance(texture, dict) or not {"width", "height"} <= set(texture) or set(texture) - _TEXTURE_FIELDS or not isinstance(texture.get("image", ""), str):
        return None, (MarkupProblemV1(MARKUP_TEXTURE_INVALID, None, "texture must be an object with width, height and an optional image name"),)
    marks_data = data.get("marks", [])
    if not isinstance(marks_data, list):
        return None, (MarkupProblemV1(MARKUP_MARKS_INVALID, None, "marks must be a list"),)
    marks = [mark for mark in (_mark_from(problems, at, item) for at, item in enumerate(marks_data)) if mark is not None]
    markup = HotspotMarkupV1(texture["width"], texture["height"], texture.get("image", ""), tuple(marks), data.get("next_id", 0))
    problems.extend(validate_markup(markup))
    if _is_int(markup.width) and _is_int(markup.height):
        markup = markup.with_marks(marks) if all(_is_int(mark.id) for mark in marks) else markup
    return markup, tuple(problems)


# --------------------------------------------------------------------------
# Правки редактора (HOTSPOT_EDITOR_UX_V1): поворот, разворот «вверх», полоса corner
# --------------------------------------------------------------------------

#: Поворот метки отказан: повёрнутая полоса легла бы на другую метку или вышла бы из текстуры; метка остаётся как была.
MARK_ROTATE_BLOCKED = "MARK_ROTATE_BLOCKED"
#: «Вверх» после поворота на 90° ПО ЧАСОВОЙ стрелке в пикселях текстуры (начало - левый нижний угол, x вправо, y вверх, как UV Blender):
#: вектор (x, y) -> (y, -x), то есть +y -> +x -> -y -> -x -> +y.
UP_CLOCKWISE = {"+y": "+x", "+x": "-y", "-y": "-x", "-x": "+y"}
#: Разворот стрелки: «вверх» в обратную сторону, прямоугольник тот же.
UP_OPPOSITE = {"+y": "-y", "-y": "+y", "+x": "-x", "-x": "+x"}


def _grid_start(centre: float, size: int, grid: int, full: int) -> int:
    """Начало отрезка длины `size` с серединой у `centre`: по сетке `grid`, внутри `0..full` (текстура важнее сетки)."""

    start = round((centre - size / 2.0) / max(1, int(grid))) * max(1, int(grid))
    return int(max(0, min(full - size, start)))


def rotated_mark(markup: HotspotMarkupV1, index: int, grid: int = 1) -> tuple:
    """`(метка | None, находка | None)`: метка `markup.marks[index]` повёрнута на 90° по часовой стрелке (`UP_CLOCKWISE`).

    Толщина (размер вдоль «вверх») та же, длина - во всю текстуру по новой оси (метка повторяется по длине, HOTSPOT_MARKUP_V1); середина
    по толщине - середина прежнего прямоугольника по той же оси, по сетке и внутри текстуры. Толщина больше стороны текстуры или пересечение
    с другой меткой - отказ `MARK_ROTATE_BLOCKED` с номерами меток, метка не меняется. Корень - чистый: правит только редактор."""

    mark = markup.marks[index]
    up = UP_CLOCKWISE.get(mark.up, "+x")
    thickness = mark.thickness
    if up in ("+y", "-y"):
        full = markup.height
        rotated = replace(mark, up=up, x=0, y=_grid_start(mark.y + mark.h / 2.0, thickness, grid, full), w=markup.width, h=thickness)
    else:
        full = markup.width
        rotated = replace(mark, up=up, x=_grid_start(mark.x + mark.w / 2.0, thickness, grid, full), y=0, w=thickness, h=markup.height)
    if thickness > full or thickness <= 0:
        return None, MarkupProblemV1(MARK_ROTATE_BLOCKED, mark.id, f"turned to up {up}, the mark is {thickness} px thick across a {full} px texture side")
    blocking = [other.id for at, other in enumerate(markup.marks) if at != index and _overlaps(rotated, other)]
    if blocking:
        return None, MarkupProblemV1(MARK_ROTATE_BLOCKED, mark.id, f"turned to up {up} ({rotated.x}, {rotated.y}, {rotated.w}, {rotated.h}) it overlaps mark(s) {', '.join(map(str, blocking))}")
    return rotated, None


def flipped_mark(mark: HotspotMarkV1) -> HotspotMarkV1:
    """Стрелка «вверх» в обратную сторону (`UP_OPPOSITE`), прямоугольник не двигается."""

    return replace(mark, up=UP_OPPOSITE.get(mark.up, "-y"))


def band_from_edge(mark: HotspotMarkV1, along: float, grid: int = 1) -> int:
    """Ширина полосы corner, когда край полосы тянут в точку `along` (пиксели по оси «вверх»): полоса симметрична средней линии метки,
    кратна сетке, не меньше 0 и строго меньше толщины метки."""

    centre = mark.y + mark.h / 2.0 if mark.up in ("+y", "-y") else mark.x + mark.w / 2.0
    grid = max(1, int(grid))
    widest = max(0, (mark.thickness - 1) // grid * grid)
    return int(max(0, min(widest, round(2.0 * abs(float(along) - centre) / grid) * grid)))


#: Сетка инструмента: шаг в пикселях от 1 до 1024.
GRID_MIN, GRID_MAX = 1, 1024


def grid_key_step(event_type: str, ctrl: bool) -> int:
    """Клавиша сетки редактора: -1 - мельче (шаг пополам: `[`, Ctrl + `=` / Ctrl + плюс цифровой клавиатуры), +1 - крупнее (шаг вдвое: `]`,
    Ctrl + `-` / Ctrl + минус цифровой клавиатуры), 0 - не клавиша сетки (`=` и `-` без Ctrl остаются Image Editor)."""

    if event_type in ("LEFT_BRACKET", "RIGHT_BRACKET"):
        return -1 if event_type == "LEFT_BRACKET" else 1
    if ctrl and event_type in ("EQUAL", "NUMPAD_PLUS"):
        return -1
    if ctrl and event_type in ("MINUS", "NUMPAD_MINUS"):
        return 1
    return 0


def grid_step(grid: int, step: int) -> int:
    """Шаг сетки после клавиши `grid_key_step`: пополам или вдвое, в пределах `GRID_MIN..GRID_MAX`."""

    grid = int(grid)
    return int(max(GRID_MIN, min(GRID_MAX, grid // 2 if step < 0 else grid * 2 if step > 0 else grid)))


def problem_names(text: str) -> tuple:
    """Имена находок (с номером метки) из строки `MarkupProblemV1.text()`, соединённой `; `: короткая строка заголовка мини-окна."""

    names = []
    for item in str(text).split("; "):
        name = item.partition(": ")[0].strip()
        if name and name not in names:
            names.append(name)
    return tuple(names)


def empty_markup(width: int, height: int, image: str = "") -> HotspotMarkupV1:
    return HotspotMarkupV1(int(width), int(height), str(image), ())


#: Имя первой метки новой разметки (HOTSPOT_FIRST_MARK_WHOLE_SQUARE_V1) - как у метки, нарисованной инструментом.
FIRST_MARK_NAME = "mark 0"


def first_markup(width: int, height: int, image: str = "") -> HotspotMarkupV1:
    """Новая разметка материала (HOTSPOT_FIRST_MARK_WHOLE_SQUARE_V1): сразу метка 0 - trim во весь квадрат текстуры (0..1, «вверх» +y), годная
    к работе без рисования; счётчик номеров - 1."""

    mark = HotspotMarkV1(0, FIRST_MARK_NAME, KIND_TRIM, 0, 0, int(width), int(height), "+y", 0)
    return HotspotMarkupV1(int(width), int(height), str(image), (mark,), 1)


def covers_whole_square(markup: HotspotMarkupV1) -> bool:
    """Единственная метка разметки - весь квадрат текстуры (первая метка новой разметки или такая же): в UV это 0..1 при любом размере, поэтому
    подложка другого размера переводит такую разметку в свой размер, как разметку без меток (HOTSPOT_BACKDROP_V1)."""

    return len(markup.marks) == 1 and (markup.marks[0].x, markup.marks[0].y, markup.marks[0].w, markup.marks[0].h) == (0, 0, int(markup.width), int(markup.height))


def next_mark_id(markup: HotspotMarkupV1, floor: int = 0) -> int:
    """Номер новой метки (HOTSPOT_MARK_ID_MONOTONIC_V1): не меньше счётчика разметки, чем любой номер её меток плюс один и чем `floor` (номера,
    на которые ссылаются записи декалей файла). Номер удалённой метки поэтому не достаётся новой: декали не перескакивают на неё молча."""

    ids = [mark.id for mark in markup.marks if _is_int(mark.id) and mark.id < WHOLE_TEXTURE_MARK_ID]
    counter = markup.next_id if _is_int(markup.next_id) and markup.next_id > 0 else 0
    return max(counter, max(ids) + 1 if ids else 0, int(floor))


def whole_texture_mark(width: int, height: int) -> HotspotMarkV1:
    """Псевдометка «вся текстура» (HOTSPOT_MARK_FALLBACK_V1): trim во весь квадрат UV, «вверх» +y. В разметку не пишется."""

    return HotspotMarkV1(WHOLE_TEXTURE_MARK_ID, "whole texture", KIND_TRIM, 0, 0, int(width), int(height), "+y", 0)


#: Заливка метки: свой оттенок у каждого id (золотое сечение по кругу оттенков - соседние id далеко друг от друга), прозрачность ~0.2;
#: выделенная метка плотнее; полоса corner - контрастный (тёмный насыщенный) тон того же оттенка.
GOLDEN_HUE_STEP = 0.618033988749895
FILL_ALPHA, FILL_ALPHA_ACTIVE = 0.2, 0.3
BAND_ALPHA, BAND_ALPHA_ACTIVE = 0.4, 0.5


def mark_colours(mark_id: int, active: bool) -> tuple:
    """`(заливка, полоса, контур)` RGBA метки: оттенок от id, стабильный между сеансами."""

    hue = (int(mark_id) * GOLDEN_HUE_STEP) % 1.0
    fill = (*colorsys.hsv_to_rgb(hue, 0.6, 1.0), FILL_ALPHA_ACTIVE if active else FILL_ALPHA)
    band = (*colorsys.hsv_to_rgb(hue, 0.95, 0.45), BAND_ALPHA_ACTIVE if active else BAND_ALPHA)
    outline = (1.0, 1.0, 1.0, 1.0) if active else (*colorsys.hsv_to_rgb(hue, 0.5, 1.0), 0.95)
    return fill, band, outline


__all__ = (
    "FORMAT_NAME",
    "FORMAT_VERSION",
    "HotspotMarkV1",
    "HotspotMarkupV1",
    "KINDS",
    "KIND_CORNER",
    "KIND_TRIM",
    "MARK_ROTATE_BLOCKED",
    "MARKUP_NEXT_ID_INVALID",
    "WHOLE_TEXTURE_MARK_ID",
    "MarkupProblemV1",
    "PROBLEM_NAMES",
    "UP_CLOCKWISE",
    "UP_OPPOSITE",
    "UP_VECTORS",
    "FIRST_MARK_NAME",
    "band_from_edge",
    "covers_whole_square",
    "empty_markup",
    "first_markup",
    "flipped_mark",
    "GRID_MAX",
    "GRID_MIN",
    "grid_key_step",
    "grid_step",
    "problem_names",
    "rotated_mark",
    "load_markup",
    "mark_colours",
    "markup_to_dict",
    "next_mark_id",
    "save_markup",
    "validate_markup",
    "whole_texture_mark",
)
