"""Словарь, который отвечает «лежит ли этот объект среди значений» за O(1) (HELD_BY_IDENTITY_INDEX_V1).

Зачем. Кэш подготовок сессии (`_conveyor_preparation_cache`) вытесняется не им, а хранилищем по содержимому: на каждую вытесненную запись хранилище зовёт `_forget_preparation_blob`, и тот спрашивал,
держит ли подготовку кэш сессии, обходом ВСЕХ значений (`any(item is prepared for item in tuple(cache.values()))`). На холодной `cover.008` (1051 домен, три четверти записей хранилища вытесняются)
это 833 тысячи шагов генератора и квадрат по числу доменов. Здесь значения посчитаны по тождеству при каждой записи, и вопрос стоит одного поиска в словаре.

Смысл вопроса тот же: «среди значений есть объект, тождественный данному» (`is`, не `==`). Значение под двумя ключами считается дважды, пока не уйдут оба.
"""

from __future__ import annotations


class IdentityHeldDict(dict):
    """`dict`, у которого `holds(value)` - есть ли значение, ТОЖДЕСТВЕННОЕ `value`, среди текущих значений. Любая запись, замена и удаление держат счёт."""

    __slots__ = ("_held",)

    def __init__(self, *args, **kwargs) -> None:
        super().__init__()
        self._held: dict[int, int] = {}
        self.update(*args, **kwargs)

    def _add(self, value) -> None:
        key = id(value)
        self._held[key] = self._held.get(key, 0) + 1

    def _drop(self, value) -> None:
        key = id(value)
        count = self._held[key] - 1
        if count:
            self._held[key] = count
        else:
            del self._held[key]

    def holds(self, value) -> bool:
        return id(value) in self._held

    def __setitem__(self, key, value) -> None:
        if key in self:
            self._drop(dict.__getitem__(self, key))
        dict.__setitem__(self, key, value)
        self._add(value)

    def __delitem__(self, key) -> None:
        value = dict.__getitem__(self, key)
        dict.__delitem__(self, key)
        self._drop(value)

    def pop(self, key, *default):
        if key in self:
            value = dict.pop(self, key)
            self._drop(value)
            return value
        if default:
            return default[0]
        raise KeyError(key)

    def popitem(self):
        key, value = dict.popitem(self)
        self._drop(value)
        return key, value

    def clear(self) -> None:
        dict.clear(self)
        self._held.clear()

    def setdefault(self, key, default=None):
        if key in self:
            return dict.__getitem__(self, key)
        self[key] = default
        return default

    def update(self, *args, **kwargs) -> None:
        for key, value in dict(*args, **kwargs).items():
            self[key] = value

    def __ior__(self, other):
        self.update(other)
        return self


__all__ = ("IdentityHeldDict",)
