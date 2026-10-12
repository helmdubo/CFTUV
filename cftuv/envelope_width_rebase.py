"""Результат домена, сдвинутый аффинным сертификатом на другую ширину (WIDTH_REFRESH_INCREMENTAL_V1).

ЗАЧЕМ. Внутри заверенного интервала домена (`AlphaIntervalV1`, статус `CERTIFIED`) при неизменной структуре позиция каждой вершины батча и произведения `u * alpha`, `v * alpha` каждой петли UV
аффинны по ширине, а ядро выпускает их наклоны вместе с результатом (`ParametricCertificateV1`). Поэтому результат домена на соседней ширине - это тот же результат, чьи позиции и UV сдвинуты
наклоном: ни покрытия, ни материализации, ни воркера. Так живая ширина обслуживает домены, которых пересчёт не требует, и считает заново только остальные.

ФОРМУЛЫ (ровно те, что у кадра модели `envelope_width_preview_model.positions_and_uvs`; `dt = alpha - alpha0`, `alpha0` - ширина сертификата):
    позиция батча = P0 + s * dt
    uv            = uv0 + (S - uv0) * dt / alpha        (S - наклон произведения `uv * alpha`)
На `dt = 0` ответ - сам результат БИТОВО. Нормали смещения вершин, ссылки, грани, швы, владельцы и цепи не двигаются: у плоского домена нормаль одна, у домена-развёртки она заморожена на ширине
сертификата (`PREVIEW_DEVELOPABLE_FROZEN_NORMAL_V1`: такой домен обслуживается только в своей досягаемости, её держит модель), `src:`-вершины хоста стоят (наклон нулевой).

ЧТО НЕ ЗАВЕРЕНО. Структура домена на новой ширине (тесселяция, силуэт, закон митры): сдвинутый результат несёт ПРЕЖНЮЮ структуру. Её сверяет полный lean-такт и подтверждение
(`envelope_width_incremental`, `envelope_width_preview_model.deviation`). Домен, который сдвинуть нечем, не сдвигается и не подменяется: его причина названа (`REBASE_*`), и он считается заново.
"""

from __future__ import annotations

from dataclasses import replace
from fractions import Fraction

import numpy as np

from .envelope_production_result import rebased_result
from .envelope_production_view import DomainViewV1, build_domain_view
from .envelope_production_weld import DomainVerticesV1

REBASE_NOT_MATERIALIZED = "INCREMENTAL_REBASE_NOT_MATERIALIZED"
REBASE_NO_CERTIFICATE = "INCREMENTAL_REBASE_NO_CERTIFICATE"
REBASE_NO_VIEW = "INCREMENTAL_REBASE_NO_VIEW"
REBASE_SHAPE_MISMATCH = "INCREMENTAL_REBASE_SHAPE_MISMATCH"
REBASE_ALPHA_NOT_POSITIVE_FINITE = "INCREMENTAL_REBASE_ALPHA_NOT_POSITIVE_FINITE"


def certificate_usable(result) -> bool:
    """У результата есть сертификат с числами (без названных причин отказа): его можно сдвигать."""

    certificate = getattr(result, "parametric", None)
    return certificate is not None and not certificate.markers and certificate.vertex_count > 0


def view_of(result):
    """Вид домена результата: присланный воркером, либо посчитанный из батча тем же кодом (результат, который считал родитель) и запомненный на результате; `None` - вида нет (lean-результат без вида быть не может)."""

    view = getattr(result, "view", None)
    if view is not None:
        return view
    if result.__dict__.get("lean"):
        return None
    view = build_domain_view(result)
    object.__setattr__(result, "view", view)  # производное поле результата (чистая функция его батча и нормалей, как `_thaw` у отложенных): следующий сдвиг того же результата не разбирает батч заново
    return view


def rebased_view(view: DomainViewV1, certificate, alpha: float) -> DomainViewV1:
    """Вид домена на ширине `alpha`: позиции батча и UV сдвинуты наклонами сертификата; всё прочее - те же объекты, что у `view`."""

    dt = alpha - float(certificate.alpha)
    vertices = view.vertices
    if dt == 0.0:
        return view
    slopes = np.frombuffer(certificate.position_slopes, dtype=np.float64).reshape(-1, 3)
    points = np.asarray(vertices.positions, dtype=np.float64) + slopes * dt
    loops = np.frombuffer(certificate.loop_slopes, dtype=np.float64).reshape(-1, 2)
    uvs = np.asarray(view.uvs, dtype=np.float64)
    uvs = uvs + (loops - uvs) * (dt / alpha)
    return DomainViewV1(
        None,
        DomainVerticesV1(vertices.patch_id, tuple(map(tuple, points.tolist())), vertices.normals, vertices.refs, vertices.walls),
        view.faces,
        tuple(map(tuple, uvs.tolist())),
        view.owners,
        view.seams,
        view.boundary_chains,
        view.source_revision,
    )


def rebase_result(result, alpha_text: str):
    """`(результат на ширине alpha_text, "")` либо `(None, имя причины)`: lean-копия результата прежнего прогона, сдвинутая его сертификатом (`rebased_result`).

    Сертификат копии - тот же (наклоны не меняются), но на новой ширине, поэтому модель следующего образца читает его как сертификат этого прогона (`_certificate_slopes` сверяет ширину).
    """

    alpha = float(alpha_text)
    if not (alpha > 0.0 and alpha < float("inf")):
        return None, REBASE_ALPHA_NOT_POSITIVE_FINITE
    if not result.is_materialized:
        return None, REBASE_NOT_MATERIALIZED
    if not certificate_usable(result):
        return None, REBASE_NO_CERTIFICATE
    view = view_of(result)
    if view is None or view.failure is not None or view.vertices is None:
        return None, REBASE_NO_VIEW
    certificate = result.parametric
    if certificate.vertex_count != len(view.vertices.positions) or certificate.loop_count != len(view.uvs):
        return None, REBASE_SHAPE_MISMATCH
    return rebased_result(result, view=rebased_view(view, certificate, alpha), certificate=replace(certificate, alpha=Fraction(alpha_text))), ""


__all__ = (
    "REBASE_ALPHA_NOT_POSITIVE_FINITE",
    "REBASE_NOT_MATERIALIZED",
    "REBASE_NO_CERTIFICATE",
    "REBASE_NO_VIEW",
    "REBASE_SHAPE_MISMATCH",
    "certificate_usable",
    "rebase_result",
    "rebased_view",
    "view_of",
)
