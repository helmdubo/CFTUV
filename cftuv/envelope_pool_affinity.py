"""Воркеры пула сидят на первых потоках физических ядер (POOL_WORKERS_ON_PRIMARY_THREADS_V1).

ЗАЧЕМ. На процессоре с SMT (Ryzen 9 5900X: 12 ядер, 24 логических процессора) планировщик Windows гоняет двенадцать занятых воркеров по всем двадцати четырём процессорам и время от времени
сажает двух на братские потоки одного ядра или двигает задачу с ядра на ядро (холодная `cover.008`: 87 % задач меняют процессор на ходу). Два потока одного ядра считают каждый заметно медленнее, чем поток на ядре в одиночку. Маска воркеров = ПЕРВЫЕ логические процессоры всех физических ядер (для SMT-пар 0, 2, 4, ... 22) оставляет каждому
воркеру своё ядро целиком; братские потоки остаются родителю и всему остальному в системе. Замер (зонд времён задач в воркерах и в родителе, воркеры 12, один сеанс Blender на запуск, условия чередуются,
пять-восемь повторов, сумма занятости воркеров падает на 7-8 %): холодная `cover.008` стена пула с маской 6.81 с, без неё (остальное то же) 7.36 с (-0.55 с, -7.4 %), без всего 7.63 с; холодная `building`
1.537 против 1.644 с (-0.107 с, -6.5 %; маска ниже в восьми повторах из восьми); шаг ширины `cover.008` 1.099 против 1.196 с, `building` 0.188 против 0.196 с. Закрепление каждого воркера за ОДНИМ процессором
(по одному на ядро) хуже общей маски: `building` +14 % (воркер не может уйти с занятого другим процессом процессора), поэтому маска общая для всех воркеров, а выбор процессора внутри неё остаётся планировщику.

ГДЕ ПРИМЕНЯЕТСЯ. Только Windows и только там, где маска что-то даёт и ничего не отнимает; иначе воркеры идут без привязки, а ПРИЧИНА - один числовой счётчик прогона (`ENVELOPE_DOMAIN_POOL_WORKER_AFFINITY`,
коды ниже), ни строки консоли, ни отказа:

* топология читается `GetLogicalProcessorInformationEx` (`physical_core_masks`); не читается - `AFFINITY_TOPOLOGY_UNKNOWN`;
* ровно по два логических процессора на каждое ядро, один класс эффективности, одна группа процессоров; иначе (SMT выключен, гибридный процессор, больше 64 процессоров) - `AFFINITY_NOT_SMT_PAIRS`: на таких машинах
  выигрыш не доказан, и привязки нет;
* воркеров не больше, чем физических ядер (16 воркеров на 12 ядрах - настройка владельца ради SMT): иначе - `AFFINITY_MORE_WORKERS_THAN_CORES`;
* процесс-родитель сам ограничен маской (Process Lasso, `start /affinity`): воркеры получают пересечение, и если в нём меньше первых потоков, чем воркеров, - `AFFINITY_PARENT_MASK_TOO_NARROW`;
* `CFTUV_POOL_AFFINITY=OFF` - владелец выключает (`AFFINITY_DISABLED`); сравнение двух условий одного дерева в `tools/blender_tree_ab.py` ведётся этой переменной;
* `SetProcessAffinityMask` отказал хоть для одного воркера - `AFFINITY_APPLY_FAILED`, и уже поставленные маски остаются (ответ от маски не зависит).

Родитель (Blender, его интерфейсный поток, потоки-читатели) маски не получает. Ответ от привязки не зависит: она только решает, КАКОЙ процессор исполнит тот же код.
"""

from __future__ import annotations

import os
import struct
import sys

#: Код в счётчике прогона: 0 - пул не заказывал привязку (нет пула, прогон не через пул), 1 - маска поставлена, остальное - почему нет.
AFFINITY_NOT_EVALUATED = 0
AFFINITY_APPLIED = 1
AFFINITY_NOT_WINDOWS = 2
AFFINITY_TOPOLOGY_UNKNOWN = 3
AFFINITY_NOT_SMT_PAIRS = 4
AFFINITY_MORE_WORKERS_THAN_CORES = 5
AFFINITY_PARENT_MASK_TOO_NARROW = 6
AFFINITY_APPLY_FAILED = 7
AFFINITY_DISABLED = 8

ENVIRONMENT_VARIABLE = "CFTUV_POOL_AFFINITY"

_RELATION_PROCESSOR_CORE = 0
_PROCESS_SET_INFORMATION = 0x0200
_PROCESS_QUERY_LIMITED_INFORMATION = 0x1000
_ERROR_INSUFFICIENT_BUFFER = 122


def _windows() -> bool:
    return sys.platform == "win32"


def physical_core_masks():
    """`[(маска логических процессоров ядра, класс эффективности, группа), ...]` по `GetLogicalProcessorInformationEx`, либо `None`, если Windows не отвечает."""

    if sys.platform != "win32" or struct.calcsize("P") != 8:
        return None
    try:
        import ctypes
        from ctypes import wintypes

        kernel32 = ctypes.WinDLL("kernel32", use_last_error=True)
        function = kernel32.GetLogicalProcessorInformationEx
        function.argtypes = [wintypes.DWORD, ctypes.c_void_p, ctypes.POINTER(wintypes.DWORD)]
        function.restype = wintypes.BOOL
        length = wintypes.DWORD(0)
        function(_RELATION_PROCESSOR_CORE, None, ctypes.byref(length))
        if ctypes.get_last_error() != _ERROR_INSUFFICIENT_BUFFER or not length.value:
            return None
        buffer = ctypes.create_string_buffer(length.value)
        if not function(_RELATION_PROCESSOR_CORE, buffer, ctypes.byref(length)):
            return None
        data = buffer.raw[: length.value]
        cores, offset = [], 0
        while offset + 8 <= len(data):
            relationship, size = struct.unpack_from("<II", data, offset)
            if size < 8 or offset + size > len(data):
                return None
            if relationship == _RELATION_PROCESSOR_CORE:
                efficiency = data[offset + 9]
                (groups,) = struct.unpack_from("<H", data, offset + 30)
                masks = [struct.unpack_from("<QH", data, offset + 32 + 16 * index) for index in range(groups)]
                cores.append((sum(mask for mask, _group in masks), efficiency, tuple(group for _mask, group in masks)))
            offset += size
        return cores or None
    except Exception:  # noqa: BLE001 - топология не читается: воркеры идут без привязки, причина названа кодом
        return None


def parent_affinity_mask():
    """Маска процессоров самого процесса (воркеры наследуют её при старте) либо `None`."""

    if sys.platform != "win32":
        return None
    try:
        import ctypes
        from ctypes import wintypes

        kernel32 = ctypes.WinDLL("kernel32", use_last_error=True)
        kernel32.GetCurrentProcess.restype = ctypes.c_void_p
        kernel32.GetProcessAffinityMask.argtypes = [ctypes.c_void_p, ctypes.POINTER(ctypes.c_size_t), ctypes.POINTER(ctypes.c_size_t)]
        kernel32.GetProcessAffinityMask.restype = wintypes.BOOL
        process, system = ctypes.c_size_t(0), ctypes.c_size_t(0)
        if not kernel32.GetProcessAffinityMask(kernel32.GetCurrentProcess(), ctypes.byref(process), ctypes.byref(system)):
            return None
        return int(process.value)
    except Exception:  # noqa: BLE001 - маска родителя неизвестна: привязки нет, причина названа кодом
        return None


def plan_worker_mask(cores, parent_mask, workers: int) -> tuple[int, int]:
    """`(маска воркеров, код)`: маска 0 - привязки нет, и код называет почему (`AFFINITY_*`); код `AFFINITY_APPLIED` - маска годна.

    `cores` - `physical_core_masks()` либо `None`; `parent_mask` - `parent_affinity_mask()` либо `None`; `workers` - сколько воркеров заказал пул. Чистая функция: топологию подставляет тест.
    """

    if not _windows():
        return 0, AFFINITY_NOT_WINDOWS
    if os.environ.get(ENVIRONMENT_VARIABLE, "").strip().upper() == "OFF":
        return 0, AFFINITY_DISABLED
    if not cores or parent_mask is None:
        return 0, AFFINITY_TOPOLOGY_UNKNOWN
    if len({efficiency for _mask, efficiency, _groups in cores}) != 1 or any(len(groups) != 1 or groups[0] != 0 or bin(mask).count("1") != 2 for mask, _efficiency, groups in cores):
        return 0, AFFINITY_NOT_SMT_PAIRS
    if workers > len(cores):
        return 0, AFFINITY_MORE_WORKERS_THAN_CORES
    primary = 0
    for mask, _efficiency, _groups in cores:
        primary |= mask & -mask  # младший бит маски ядра: первый логический процессор
    allowed = primary & parent_mask
    if bin(allowed).count("1") < workers:
        return 0, AFFINITY_PARENT_MASK_TOO_NARROW
    return allowed, AFFINITY_APPLIED


def set_process_affinity(pid: int, mask: int) -> bool:
    """`SetProcessAffinityMask` для процесса `pid`; `False` - Windows отказала."""

    try:
        import ctypes
        from ctypes import wintypes

        kernel32 = ctypes.WinDLL("kernel32", use_last_error=True)
        kernel32.OpenProcess.argtypes = [wintypes.DWORD, wintypes.BOOL, wintypes.DWORD]
        kernel32.OpenProcess.restype = ctypes.c_void_p
        kernel32.SetProcessAffinityMask.argtypes = [ctypes.c_void_p, ctypes.c_size_t]
        kernel32.SetProcessAffinityMask.restype = wintypes.BOOL
        kernel32.CloseHandle.argtypes = [ctypes.c_void_p]
        handle = kernel32.OpenProcess(_PROCESS_SET_INFORMATION | _PROCESS_QUERY_LIMITED_INFORMATION, False, int(pid))
        if not handle:
            return False
        try:
            return bool(kernel32.SetProcessAffinityMask(handle, mask))
        finally:
            kernel32.CloseHandle(handle)
    except Exception:  # noqa: BLE001 - отказ назван кодом, воркер остаётся без маски
        return False


def process_affinity_mask(pid: int):
    """Маска процессоров чужого процесса `pid` либо `None` (для теста и диагностики)."""

    try:
        import ctypes
        from ctypes import wintypes

        kernel32 = ctypes.WinDLL("kernel32", use_last_error=True)
        kernel32.OpenProcess.argtypes = [wintypes.DWORD, wintypes.BOOL, wintypes.DWORD]
        kernel32.OpenProcess.restype = ctypes.c_void_p
        kernel32.GetProcessAffinityMask.argtypes = [ctypes.c_void_p, ctypes.POINTER(ctypes.c_size_t), ctypes.POINTER(ctypes.c_size_t)]
        kernel32.GetProcessAffinityMask.restype = wintypes.BOOL
        kernel32.CloseHandle.argtypes = [ctypes.c_void_p]
        handle = kernel32.OpenProcess(_PROCESS_QUERY_LIMITED_INFORMATION, False, int(pid))
        if not handle:
            return None
        try:
            process, system = ctypes.c_size_t(0), ctypes.c_size_t(0)
            if not kernel32.GetProcessAffinityMask(handle, ctypes.byref(process), ctypes.byref(system)):
                return None
            return int(process.value)
        finally:
            kernel32.CloseHandle(handle)
    except Exception:  # noqa: BLE001
        return None


def apply_worker_affinity(pids, workers: int) -> int:
    """Маска первых потоков ядер - каждому процессу из `pids`; код `AFFINITY_*` (1 - поставлена всем). `workers` - сколько воркеров заказал пул (а не сколько стартовало сейчас)."""

    mask, code = plan_worker_mask(physical_core_masks(), parent_affinity_mask(), workers)
    if code != AFFINITY_APPLIED:
        return code
    failed = [pid for pid in pids if not set_process_affinity(pid, mask)]
    return AFFINITY_APPLY_FAILED if failed else AFFINITY_APPLIED


__all__ = (
    "AFFINITY_APPLIED",
    "AFFINITY_APPLY_FAILED",
    "AFFINITY_DISABLED",
    "AFFINITY_MORE_WORKERS_THAN_CORES",
    "AFFINITY_NOT_EVALUATED",
    "AFFINITY_NOT_SMT_PAIRS",
    "AFFINITY_NOT_WINDOWS",
    "AFFINITY_PARENT_MASK_TOO_NARROW",
    "AFFINITY_TOPOLOGY_UNKNOWN",
    "ENVIRONMENT_VARIABLE",
    "apply_worker_affinity",
    "parent_affinity_mask",
    "physical_core_masks",
    "plan_worker_mask",
    "process_affinity_mask",
    "set_process_affinity",
)
