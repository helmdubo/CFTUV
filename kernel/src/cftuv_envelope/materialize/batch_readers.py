"""Читатели батча материализованного домена в воркере (BATCH_READERS_NATIVE_V1): проверка (`validate_geometry_batch`), аудит сетки (`audit_batch`) и подпись структуры
(`batch_structure`) одним проходом нативного ядра.

Эталон - эти три функции Python, ответ - тот же: замечания проверки, `BatchAuditV1`, `StructureSignatureV1`. Порт (`cftuv_native.batch_checks`) ДОКАЗЫВАЕТ, что проверка
замечаний не найдёт, и считает аудит и подпись; замечаний он не пишет. Не доказал (батч сломан, у эталона нашлось бы что назвать) - все три считает эталон, и отказ
`BATCH_DID_NOT_VALIDATE` несёт те же имена, что и прежде. Вход, который порт не несёт, - откат на эталон по имени (`NATIVE_PORT_UNSUPPORTED` в журнале бэкенда).
Вне блока бэкенда и при стадии `PYTHON` порт не зовётся вовсе.
"""

from __future__ import annotations

from .. import backend
from ..canonical import geometry_batch_semantic_digest
from ..contracts.geometry_batch import GEOMETRY_BATCH_SCHEMA_V1
from ..contracts.metric import CUT_RIGHT_COPY_MARK
from ..ids import SemanticDigestValue
from ..validation import validate_geometry_batch
from .audit import BatchAuditV1, audit_batch
from .structure import StructureSignatureV1, batch_structure


def _oracle(batch, source_normal, vertex_normals, validate: bool, audit: bool, structure: bool, check_semantic_digest: bool) -> tuple:
    issues = validate_geometry_batch(batch, check_semantic_digest=check_semantic_digest) if validate else ()
    audited = None
    if audit:
        audited = audit_batch(batch, source_normal, vertex_normals) if vertex_normals else audit_batch(batch, source_normal)
    return issues, audited, batch_structure(batch) if structure else None


def batch_readers(batch, source_normal, vertex_normals=None, *, validate: bool, audit: bool, structure: bool, check_semantic_digest: bool = True) -> tuple:
    """`(замечания проверки, аудит | None, подпись структуры | None)` батча: то, что ответили бы `validate_geometry_batch`, `audit_batch` и `batch_structure`.

    `vertex_normals` - `{ключ вершины: нормаль смещения}` домена-развёртки (у плоского домена пусто либо `None`), `check_semantic_digest` - как у `validate_geometry_batch`.
    """

    def oracle() -> tuple:
        return _oracle(batch, source_normal, vertex_normals, validate, audit, structure, check_semantic_digest)

    if not (validate or audit or structure):
        return (), None, None

    def adopt(found) -> tuple:
        clean, audited, signature = found
        if validate and not clean:
            return oracle()  # не доказано: замечания называет эталон, и аудит с подписью - его же
        if validate and check_semantic_digest and geometry_batch_semantic_digest(batch).sha256_hex != batch.semantic_digest.value:
            return oracle()
        return (
            (),
            None if audited is None else BatchAuditV1(*audited),
            None if signature is None else StructureSignatureV1(signature[0], tuple(signature[1])),
        )

    arguments = (batch, GEOMETRY_BATCH_SCHEMA_V1, SemanticDigestValue, source_normal, vertex_normals or None, CUT_RIGHT_COPY_MARK, validate, audit, structure)
    return backend.batch_checks_compute(oracle, adopt, arguments)


__all__ = ("batch_readers",)
