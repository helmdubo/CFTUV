"""Схемы контрактов поверхности S0 — тем же потоком, своим файлом.

Отдельный генератор, а не строка в `generate_contract_schemas.py`, по одной
причине: тот импортирует типы С ФАСАДА `cftuv_envelope`, а публичный список
фасада заморожен дайджестом (`test_public_contracts.py`). Проводка новых
контрактов в фасад и в union — отдельная карточка после интеграционной
вершины; до неё схемы обязаны существовать, не двигая ни одного замороженного
числа. Рендер побайтно тот же (`sort_keys`, `indent=2`, перевод строки в
конце), поэтому файлы неотличимы от остальных схем каталога.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from cftuv_envelope.contracts.surface_adjacency import (
    SURFACE_ADJACENCY_IR_SCHEMA_V1,
    SURFACE_COMPLETE_IR_SCHEMA_V1,
    SurfaceAdjacencyIRV1,
    SurfaceCompleteIRV1,
)
from cftuv_envelope.contracts.surface_arrival import (
    SURFACE_ARRIVAL_COMPLEX_V1_SCHEMA,
    SurfaceArrivalComplexV1,
)
from cftuv_envelope.contracts.surface_metric_v2 import (
    SURFACE_METRIC_DESCRIPTOR_V2_SCHEMA,
    SurfaceMetricDescriptorV2,
)
from cftuv_envelope.schema import json_schema_for


SCHEMAS = (
    (
        SurfaceAdjacencyIRV1,
        SURFACE_ADJACENCY_IR_SCHEMA_V1,
        "surface_adjacency_ir_v1.schema.json",
    ),
    (
        SurfaceCompleteIRV1,
        SURFACE_COMPLETE_IR_SCHEMA_V1,
        "surface_complete_ir_v1.schema.json",
    ),
    (
        SurfaceMetricDescriptorV2,
        SURFACE_METRIC_DESCRIPTOR_V2_SCHEMA,
        "surface_metric_descriptor_v2.schema.json",
    ),
    (
        SurfaceArrivalComplexV1,
        SURFACE_ARRIVAL_COMPLEX_V1_SCHEMA,
        "surface_arrival_complex_v1.schema.json",
    ),
)


def rendered_schemas() -> dict[str, str]:
    return {
        filename: json.dumps(
            json_schema_for(record_type, schema_id),
            ensure_ascii=False,
            sort_keys=True,
            indent=2,
        )
        + "\n"
        for record_type, schema_id, filename in SCHEMAS
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()
    expected = rendered_schemas()
    if args.check:
        errors = [
            filename
            for filename, content in expected.items()
            if not (args.output / filename).is_file()
            or (args.output / filename).read_text(encoding="utf-8") != content
        ]
        if errors:
            print("generated surface schemas differ: " + ", ".join(errors))
            return 1
        print("generated-surface-contract-schemas: OK")
        return 0
    args.output.mkdir(parents=True, exist_ok=True)
    for filename, content in expected.items():
        (args.output / filename).write_text(content, encoding="utf-8", newline="\n")
    print(f"generated {len(expected)} surface schemas")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
