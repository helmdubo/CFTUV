"""Имена записей `PhysicalChainV1.data_record_lineage`, которые ядро читает по смыслу.

Одно место формата на обе стороны: хост пишет записи `chain-source` этим префиксом, ядро
(`_corner_treatment`) сверяет их с ним. Два литерала разошлись бы молча; одна функция —
не разойдутся, а сверка хоста и ядра становится вызовом, а не договорённостью на словах.

`chain-source:<PatchId>:<токен>` — ЦЕПЬ источника патча до разреза по изломам. Шовная цепь двух
патчей несёт записи обоих; «одна цепь» для угла патча — запись ЕГО патча (двоеточие после
идентификатора: патч 1 — не патч 12).
"""

from __future__ import annotations

CHAIN_SOURCE_LINEAGE_PREFIX = "chain-source:"


def owner_chain_source_prefix(owner_patch_id) -> str:
    """Префикс записей `chain-source` ровно ЭТОГО патча (`PatchId` либо его значение)."""

    value = getattr(owner_patch_id, "value", owner_patch_id)
    return f"{CHAIN_SOURCE_LINEAGE_PREFIX}{value}:"
