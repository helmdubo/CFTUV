"""Мутации снапшота для `snapshot_check`: доказательство СОЛИДНОСТИ чистого ответа порта.

Порт обязан отвечать «чисто» ТОЛЬКО на снапшот, который валидаторы Python пропускают, - тихое принятие нарушает правило 4 AGENTS.md. Здесь настоящий снапшот ломают сотнями способов, и на каждом сравнивают:
валидаторы Python (`validate_analysis_snapshot` без допуска, затем `validate_reference_geometry_payload` каждого домена: проверка объекта на выгрузке; и `validate_analysis_snapshot` под допуском
запроса) и нативный ответ. НАРУШЕНИЕ: порт принял, а Python отказал, исключился либо оставил другие счётчики. Нарушений обязано быть ноль; отказ порта на снапшот, который Python принимает, - не нарушение,
а цена (она в сводке).

Мутации двух родов:

* ПОЛЕВЫЕ - по одному полю каждой записи графа (идентификатор, число, перечисление, флаг, десятичное, элемент контейнера, ссылка на запись того же класса), значение подменяется из набора
  возмущений своего типа. Граф собирается заново БЕЗ `__post_init__` (`object.__new__` и запись слотов, как строит родная сторона): снапшот, которого конструктор не пропустил бы, - тоже вход.
* ЦЕЛЕВЫЕ - по одной на класс замечания (`ValidationCode`: схема, возможности, дубликат идентификатора, ссылка в пустоту, перекрёстное противоречие, угловой сертификат, топология поверхности, метрика,
  топология маршрута, терминал, ребро нулевой длины) и на исход эталонного валидатора (`ReferenceOutcome`: сертификат угла не содержит поворот опор, метрика домена неверна).

Покрытие: сводка называет, сколько мутантов дали КАЖДЫЙ код замечания и исход диагностики, и порт обязан не принять ни одного.
"""

from __future__ import annotations

import dataclasses
import enum
import random
import sys
import time
from collections import Counter
from decimal import Decimal
from fractions import Fraction
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT / "kernel" / "src") not in sys.path:
    sys.path.insert(0, str(ROOT / "kernel" / "src"))

import cftuv_envelope as kernel  # noqa: E402
from cftuv_envelope import validation  # noqa: E402
from cftuv_envelope.reference import planar_types, symbolic_backend  # noqa: E402
from cftuv_envelope.reference import validation as reference_validation  # noqa: E402

SCOPE_ANALYSIS = 0
SCOPE_BOTH = 1


# --------------------------------------------------------------------------
# граф
# --------------------------------------------------------------------------


def is_record(value) -> bool:
    return dataclasses.is_dataclass(value) and not isinstance(value, type)


def nodes_of(root) -> list:
    """Записи графа в детерминированном порядке обхода (по возрастанию `repr` у множеств): `[(узел, глубина)]`."""

    found: list = []
    seen: set = set()

    def visit(value) -> None:
        if is_record(value):
            if id(value) in seen:
                return
            seen.add(id(value))
            found.append(value)
            for field in dataclasses.fields(value):
                visit(getattr(value, field.name))
        elif isinstance(value, tuple):
            for item in value:
                visit(item)
        elif isinstance(value, frozenset):
            for item in sorted(value, key=repr):
                visit(item)

    visit(root)
    return found


def rebuild(root, target=None, field: str | None = None, value=None, drop=None, extra=None):
    """Копия графа БЕЗ `__post_init__`; у записи `target` поле `field` заменено на `value`.

    `drop` - `(запись, поле, элемент)`: элемент (по тождеству) удалён из контейнера; `extra` - `(запись, поле, элемент)`: копия элемента добавлена в КОРТЕЖ (множество от повтора не меняется).
    """

    memo: dict = {}

    def clone(item):
        if is_record(item):
            if id(item) in memo:
                return memo[id(item)]
            fresh = object.__new__(type(item))
            memo[id(item)] = fresh
            for definition in dataclasses.fields(item):
                name = definition.name
                current = getattr(item, name)
                if item is target and name == field:
                    new = value
                elif drop is not None and item is drop[0] and name == drop[1]:
                    kept = (clone(part) for part in current if part is not drop[2])
                    new = tuple(kept) if isinstance(current, tuple) else frozenset(kept)
                elif extra is not None and item is extra[0] and name == extra[1] and isinstance(current, tuple):
                    new = tuple(clone(part) for part in current) + (clone(extra[2]),)
                else:
                    new = clone(current)
                object.__setattr__(fresh, name, new)
            return fresh
        if isinstance(item, tuple):
            return tuple(clone(part) for part in item)
        if isinstance(item, frozenset):
            return frozenset(clone(part) for part in item)
        return item

    return clone(root)


# --------------------------------------------------------------------------
# возмущения листьев
# --------------------------------------------------------------------------


def perturbations(value, pool: dict, rng: random.Random):
    """Значения, на которые можно подменить лист `value` (кроме него самого)."""

    if value is None:
        return []
    if isinstance(value, bool):
        return [not value]
    if isinstance(value, enum.Enum):
        return [member for member in type(value) if member is not value]
    if isinstance(value, int):
        return [value + 1, value - 1, 0, -value, value * 2]
    if isinstance(value, float):
        return [value * 1.000001, value + 1.0, 0.0, -value, float("nan")]
    if isinstance(value, Decimal):
        return [value + Decimal(1), value - Decimal("0.5"), Decimal(0), -value, value * 2]
    if isinstance(value, Fraction):
        return [value + 1, value * 2, Fraction(0), -value]
    if isinstance(value, str):
        others = [other for other in pool.get(str, []) if other != value]
        picks = rng.sample(others, min(2, len(others))) if others else []
        return [value + "x", "", *picks]
    return []


def collect_pool(root) -> dict:
    """Строки и записи графа по типам: источник подмен «ссылкой на то, что есть»."""

    pool: dict = {str: set(), "records": {}}
    for node in nodes_of(root):
        pool["records"].setdefault(type(node), []).append(node)
        for definition in dataclasses.fields(node):
            current = getattr(node, definition.name)
            if isinstance(current, str):
                pool[str].add(current)
    pool[str] = sorted(pool[str])
    return pool


# --------------------------------------------------------------------------
# вердикты
# --------------------------------------------------------------------------


def python_verdict(snapshot, budget) -> dict:
    """Что валидаторы Python говорят о снапшоте: объект на выгрузке (без допуска, затем полезная нагрузка каждого домена) и замечания под допуском запроса."""

    symbolic_backend.reset_backend_counts()
    before = dict(planar_types.SYMBOLIC_FALLBACK_COUNTS)
    verdict = {"export_issues": (), "diagnostics": (), "export_exception": None, "request_issues": (), "request_exception": None, "counters": {}}
    try:
        verdict["export_issues"] = tuple(issue.code.value for issue in validation.validate_analysis_snapshot(snapshot))
        if not verdict["export_issues"]:
            diagnostics = []
            for domain in sorted((item.patch_domain_id for item in snapshot.patch_domains), key=lambda item: item.value):
                _, found = reference_validation.validate_reference_geometry_payload(snapshot, domain)
                diagnostics.extend(item.outcome.value for item in found)
            verdict["diagnostics"] = tuple(diagnostics)
    except Exception as error:  # noqa: BLE001 - исключение эталона - тоже исход
        verdict["export_exception"] = f"{type(error).__name__}: {error}"[:200]
    verdict["counters"] = {
        "backend": dict(symbolic_backend.BACKEND_COUNTS),
        "symbolic": {key: value - before.get(key, 0) for key, value in planar_types.SYMBOLIC_FALLBACK_COUNTS.items() if value != before.get(key, 0)},
    }
    try:
        verdict["request_issues"] = tuple(issue.code.value for issue in validation.validate_analysis_snapshot(snapshot, developable_stretch_budget=budget))
    except Exception as error:  # noqa: BLE001
        verdict["request_exception"] = f"{type(error).__name__}: {error}"[:200]
    return verdict


def export_clean(verdict: dict) -> bool:
    return not (verdict["export_issues"] or verdict["diagnostics"] or verdict["export_exception"])


def request_clean(verdict: dict) -> bool:
    return not (verdict["request_issues"] or verdict["request_exception"])


def native_answer(native, snapshot, scope):
    """`(SnapshotCheckV1, None)` либо `(None, причина)`."""

    try:
        return native.snapshot_check(snapshot, scope), None
    except native.NativePortUnsupported as error:
        return None, str(error)


def violations_of(native, snapshot, budget) -> tuple:
    """`(нарушения, python_verdict, ответ порта по областям)` для одного снапшота."""

    verdict = python_verdict(snapshot, budget)
    found = []
    export, export_why = native_answer(native, snapshot, SCOPE_BOTH)
    request, request_why = native_answer(native, snapshot, SCOPE_ANALYSIS)
    if export is not None:
        if not export_clean(verdict):
            found.append(("export", "native attested a snapshot the Python validators refuse", verdict["export_issues"] or verdict["diagnostics"] or verdict["export_exception"]))
        else:
            backend = verdict["counters"]["backend"]
            expected = (backend.get("angle_certificate_sign.certified", 0), backend.get("angle_certificate_sign.exact_fallback.straddles_zero", 0))
            other = {key: value for key, value in backend.items() if not key.startswith("angle_certificate_sign.")}
            if (export.certified, export.straddled) != expected or other or verdict["counters"]["symbolic"]:
                found.append(("export-counters", f"native ({export.certified}, {export.straddled}) python {backend} {verdict['counters']['symbolic']}", None))
    if request is not None and not request_clean(verdict):
        found.append(("request", "native attested issues-under-budget that Python reports", verdict["request_issues"] or verdict["request_exception"]))
    return found, verdict, ((export, export_why), (request, request_why))


# --------------------------------------------------------------------------
# мутанты
# --------------------------------------------------------------------------


def field_mutants(root, rng: random.Random, limit: int):
    """До `limit` случайных полевых мутантов: `(описание, корень)`."""

    pool = collect_pool(root)
    nodes = nodes_of(root)
    choices = []
    for node in nodes:
        for definition in dataclasses.fields(node):
            choices.append((node, definition.name))
    rng.shuffle(choices)
    made = 0
    for node, name in choices:
        if made >= limit:
            return
        current = getattr(node, name)
        kind = type(node).__name__
        options = []
        if isinstance(current, (tuple, frozenset)):
            members = list(current) if isinstance(current, tuple) else sorted(current, key=repr)
            if members:
                options.append(("drop", dict(drop=(node, name, rng.choice(members)))))
                options.append(("duplicate", dict(extra=(node, name, rng.choice(members)))))
        elif is_record(current):
            siblings = [other for other in pool["records"].get(type(current), []) if other is not current]
            if siblings:
                options.append(("swap-record", dict(target=node, field=name, value=rng.choice(siblings))))
        else:
            for new in perturbations(current, pool, rng):
                options.append((f"set {new!r}"[:40], dict(target=node, field=name, value=new)))
        if not options:
            continue
        label, arguments = rng.choice(options)
        yield f"{kind}.{name}: {label}", rebuild(root, **arguments)
        made += 1


def rational(sample, numerator: int, denominator: int):
    """Запись, равная по классу `sample` (`ExactRationalV1`), но собранная БЕЗ `__post_init__`: несокращённая дробь - тоже вход."""

    fresh = object.__new__(type(sample))
    object.__setattr__(fresh, "numerator", numerator)
    object.__setattr__(fresh, "denominator", denominator)
    return fresh


def targeted_mutants(root, rng: random.Random):
    """Одна и более мутаций на класс замечания и исход диагностики."""

    nodes = nodes_of(root)
    by_class: dict = {}
    for node in nodes:
        by_class.setdefault(type(node).__name__, []).append(node)

    def first(name):
        found = by_class.get(name, [])
        return found[0] if found else None

    def pick(name):
        found = by_class.get(name, [])
        return rng.choice(found) if found else None

    top = root
    surface = top.surface_ir
    yield "SCHEMA_VERSION: snapshot", rebuild(root, top, "schema_version", "cftuv.envelope.analysis_snapshot.v0")
    yield "SCHEMA_VERSION: surface", rebuild(root, surface, "schema_version", "x")
    capabilities = sorted(top.analysis_capabilities, key=lambda item: item.value)
    yield "CAPABILITY: one required dropped", rebuild(root, top, "analysis_capabilities", frozenset(capabilities[1:]))
    fixture = kernel.AnalysisCapability.EC0_COORDINATE_FREE_FIXTURE_V5
    yield "CAPABILITY: fixture label on a full surface", rebuild(root, top, "analysis_capabilities", top.analysis_capabilities | {fixture})
    chain = pick("PhysicalChainV1")
    if chain is not None:
        others = [item for item in by_class["PhysicalChainV1"] if item is not chain]
        if others:
            yield "DUPLICATE_ID: a chain id twice", rebuild(root, others[0], "physical_chain_id", chain.physical_chain_id)
        yield "SURFACE_TOPOLOGY: chain edge count", rebuild(root, chain, "ordered_physical_edge_ids", chain.ordered_physical_edge_ids[:-1])
        yield "MISSING_REFERENCE: chain vertex", rebuild(root, chain, "ordered_source_vertex_ids", (type(chain.ordered_source_vertex_ids[0])("host-vertex:none"),) + chain.ordered_source_vertex_ids[1:])
    use = pick("ChainUseV1")
    if use is not None:
        yield "MISSING_REFERENCE: chain use chain", rebuild(root, use, "physical_chain_id", type(use.physical_chain_id)("none"))
        yield "CROSS_CONTRACT_MISMATCH: chain use owner", rebuild(root, use, "owner_patch_id", type(use.owner_patch_id)("none"))
        yield "MISSING_REFERENCE: chain use loop", rebuild(root, use, "boundary_loop_id", None)
    edge = pick("SourceEdgeV1")
    if edge is not None:
        yield "SURFACE_TOPOLOGY: edge endpoint", rebuild(root, edge, "vertex_b_id", edge.vertex_a_id)
        vertices = {item.vertex_id: item for item in top.source_vertices}
        b, a = vertices.get(edge.vertex_b_id), vertices.get(edge.vertex_a_id)
        if a is not None and b is not None:
            yield "SOURCE_EDGE_ZERO_LENGTH: edge ends on one position", rebuild(root, b, "position", a.position)
    triangle = pick("SurfaceTriangleV1")
    if triangle is not None:
        yield "SURFACE_TOPOLOGY: triangle names another face", rebuild(root, triangle, "source_face_id", type(triangle.source_face_id)("none"))
    certificate = pick("ReflexAngleCertificateV1")
    if certificate is not None:
        yield "ANGULAR_CERTIFICATE: exact 2 pi", rebuild(root, certificate, "exact_two_pi", True)
        measure = certificate.measure_payload
        if type(measure).__name__ == "CertifiedReflexAngleMeasureV1":
            delta = measure.reflex_excess_over_pi
            yield "ANGULAR_CERTIFICATE: delta shifted", rebuild(root, delta, "lower", delta.lower + Decimal("0.001"))
            yield "ANGULAR_CERTIFICATE: phi outside (1, 2)", rebuild(root, measure.phi_over_pi, "lower", Decimal("2.5"))
            yield "REFERENCE_ANGLE: orientation flipped", rebuild(root, measure, "orientation", next(m for m in type(measure.orientation) if m is not measure.orientation))
            yield "REFERENCE_ANGLE: the interval moved off the turn (both ends, both phi and delta)", _move_interval(root, measure, Decimal("0.01"))
    corner = pick("CornerRelationV1")
    if corner is not None:
        yield "ANGULAR_CERTIFICATE: corner is 2 pi", rebuild(root, corner, "exact_two_pi", True)
        yield "MISSING_REFERENCE: corner certificate", rebuild(root, corner, "reflex_angle_certificate_id", type(corner.reflex_angle_certificate_id)("none"))
    sector = pick("OrientedOwnerSectorV1")
    if sector is not None:
        yield "ANGULAR_CERTIFICATE: sector supports", rebuild(root, sector, "ordered_incident_chain_use_ids", sector.ordered_incident_chain_use_ids[:1])
        direction = sector.incoming_support_ref.direction_payload
        if type(direction).__name__ == "CertifiedAffineSupportDirectionV2":
            vector = direction.direction_in_domain
            zero = rational(vector.x, 0, 1)
            yield "REFERENCE_ANGLE: incoming direction scaled", rebuild(root, vector, "x", rational(vector.x, vector.x.numerator * 2, vector.x.denominator))
            yield "ANGULAR_CERTIFICATE: zero direction", _zero_vector(root, vector, zero)
    terminal = pick("TerminalRelationV1")
    if terminal is not None:
        yield "TERMINAL_RELATION: owner patch", rebuild(root, terminal, "owner_patch_id", type(terminal.owner_patch_id)("none"))
        other_role = next((m for m in type(terminal.endpoint_role) if m is not terminal.endpoint_role and m.name in ("START", "END")), None)
        if other_role is not None:
            yield "TERMINAL_RELATION: role vs chain end", rebuild(root, terminal, "endpoint_role", other_role)
    junction = pick("JunctionRelationV1")
    if junction is not None:
        yield "ROUTE_TOPOLOGY: kind vs variant", rebuild(root, junction, "relation_kind", next(m for m in type(junction.relation_kind) if m is not junction.relation_kind))
        yield "ROUTE_TOPOLOGY: incident uses", rebuild(root, junction, "incident_chain_use_ids", junction.incident_chain_use_ids[:-1])
    metric = first("RationalAffinePlanarMetricV2")
    if metric is not None:
        gram = metric.exact_gram_matrix
        yield "SURFACE_METRIC: gram m00", rebuild(root, gram, "m00", rational(gram.m00, gram.m00.numerator + gram.m00.denominator, gram.m00.denominator))
        yield "SURFACE_METRIC: inverse gram", rebuild(root, metric.exact_inverse_gram_matrix, "m00", rational(gram.m00, 7, 3))
        yield "SURFACE_METRIC: basis a", rebuild(root, metric.exact_basis_a, "x", rational(gram.m00, metric.exact_basis_a.x.numerator + 1, metric.exact_basis_a.x.denominator))
        certificate = metric.planarity_certificate
        normal = certificate.exact_plane_normal
        yield "SURFACE_METRIC: plane normal negated", rebuild(root, normal, "x", rational(normal.x, -normal.x.numerator, normal.x.denominator))
        yield "SURFACE_METRIC: metric revision", rebuild(root, metric, "source_revision", type(metric.source_revision)("other"))
        coordinates = sorted(metric.exact_source_vertex_coordinates, key=repr)
        if coordinates:
            point = coordinates[0].domain_coordinate
            yield "SURFACE_METRIC: a coordinate off the source", rebuild(root, point, "x", rational(point.x, point.x.numerator + point.x.denominator, point.x.denominator))
            yield "SURFACE_METRIC: a coordinate dropped", rebuild(root, drop=(metric, "exact_source_vertex_coordinates", coordinates[0]))
        yield "SURFACE_METRIC: certificate not exact", rebuild(root, certificate, "exact", False)
        yield "SURFACE_METRIC: grid scale zero", rebuild(root, metric.grid_certificate, "source_scale", 0)
        yield "SURFACE_METRIC: window step zero", rebuild(root, metric.grid_certificate, "window_step", None)
    domain = pick("PatchDomainV1")
    if domain is not None:
        yield "SURFACE_METRIC: domain not planar", rebuild(root, domain, "surface_regime", next(m for m in type(domain.surface_regime) if m.name != "PLANAR"))
        yield "MISSING_REFERENCE: domain owner patch", rebuild(root, domain, "owner_patch_id", type(domain.owner_patch_id)("none"))
    yield "SURFACE_TOPOLOGY: payload mode", rebuild(root, surface, "payload_mode", next(m for m in type(surface.payload_mode) if m is not surface.payload_mode))


def _move_interval(root, measure, shift: Decimal):
    phi, delta = measure.phi_over_pi, measure.reflex_excess_over_pi
    moved_phi = rebuild(phi, phi, "lower", phi.lower + shift)
    moved_phi = rebuild(moved_phi, moved_phi, "upper", phi.upper + shift)
    moved_delta = rebuild(delta, delta, "lower", delta.lower + shift)
    moved_delta = rebuild(moved_delta, moved_delta, "upper", delta.upper + shift)
    step = rebuild(root, measure, "phi_over_pi", moved_phi)
    node = next(item for item in nodes_of(step) if type(item).__name__ == "CertifiedReflexAngleMeasureV1" and item.orientation is measure.orientation and item.normalization_law is measure.normalization_law)
    return rebuild(step, node, "reflex_excess_over_pi", moved_delta)


def _zero_vector(root, vector, zero):
    step = rebuild(root, vector, "x", zero)
    node = next(item for item in nodes_of(step) if type(item).__name__ == "ExactVector2V1" and item.x == zero)
    return rebuild(step, node, "y", zero)


# --------------------------------------------------------------------------
# сводка
# --------------------------------------------------------------------------


def run(native, snapshots, budget, per_snapshot: int, seed: int, quiet: bool = False) -> dict:
    """Все мутанты снапшотов; `{mutants, refused, attested, declined, violations, codes, ...}`."""

    rng = random.Random(seed)
    summary = {"originals": 0, "originals_attested": 0, "mutants": 0, "python_refuses": 0, "python_clean": 0, "native_attests": 0, "native_declines_clean": 0, "native_declines_refused": 0,
               "violations": [], "codes": Counter(), "diagnostics": Counter(), "exceptions": Counter(), "targeted": Counter(), "decline_reasons": Counter(), "seconds": 0.0}
    started = time.perf_counter()
    for snapshot in snapshots:
        found, verdict, (export, request) = violations_of(native, snapshot, budget)
        summary["originals"] += 1
        summary["originals_attested"] += export[0] is not None
        if found or not export_clean(verdict):
            summary["violations"].append(("original", found or verdict))
        for kind, generator in (("field", field_mutants(snapshot, rng, per_snapshot)), ("targeted", targeted_mutants(snapshot, rng))):
            for label, mutant in generator:
                found, verdict, (export, request) = violations_of(native, mutant, budget)
                summary["mutants"] += 1
                clean = export_clean(verdict)
                summary["python_refuses" if not clean else "python_clean"] += 1
                if kind == "targeted":
                    summary["targeted"][label.split(":")[0]] += 1
                for code in verdict["export_issues"] + verdict["request_issues"]:
                    summary["codes"][code] += 1
                for outcome in verdict["diagnostics"]:
                    summary["diagnostics"][outcome] += 1
                if verdict["export_exception"] or verdict["request_exception"]:
                    summary["exceptions"][(verdict["export_exception"] or verdict["request_exception"]).split(":")[0]] += 1
                if export[0] is not None:
                    summary["native_attests"] += 1
                elif clean:
                    summary["native_declines_clean"] += 1
                    summary["decline_reasons"][(export[1] or "")[:90]] += 1
                else:
                    summary["native_declines_refused"] += 1
                if found:
                    summary["violations"].append((kind, label, found))
    summary["seconds"] = time.perf_counter() - started
    return summary
