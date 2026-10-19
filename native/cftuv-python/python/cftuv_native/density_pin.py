"""Density closure: own oracle files, narrow shared helpers, live wire shapes. No extension import."""
from __future__ import annotations

import ast
import dataclasses
from decimal import Decimal
from enum import Enum
from fractions import Fraction
import hashlib
import importlib
import inspect
import json
from pathlib import Path
import sys

# Только математический oracle; extraction/attachment/compile glue здесь не заморожен.
FILES = (
    "reference/adaptive_density_fan.py", "reference/adaptive_density_atlas.py", "reference/adaptive_density_band.py",
    "reference/density_residual.py", "reference/density_projective.py", "reference/density_binding_types.py", "reference/density_fan_types.py",
    "reference/density_raw_frontend_v1.py", "reference/density_raw_noise_v1.py", "reference/density_owned_laws_v1.py",
    "reference/density_construct_v1.py", "reference/density_scalar_v2.py", "reference/density_angle_v1.py",
    "reference/density_angle_value_v1.py", "reference/density_trig_v1.py", "reference/density_shell_v2.py",
    "reference/density_oracle_v1.py", "reference/density_controller_v1.py", "reference/density_result_v1.py",
    "reference/radical_v2.py", "exact_sqrt_sum.py", "_radicand_products.py", "float_filter.py",
    "reference/density_attempt_memo_v1.py",
)
FUNCTION_NAMES = (
    ("_density_policy.py", "density_interval_enclosure"), ("_density_policy.py", "huber_density_value_contract"),
    ("reference/planar_types.py", "_expr"), ("reference/planar_types.py", "exact_sign"),
    ("reference/planar_types.py", "exact_quadratic_value"), ("reference/common.py", "stable_id"),
)
CONSTANT_NAMES = (
    ("_authoring_intent.py", "CANONICAL_RESTORATION_ARTIST_ERROR"),
    ("_canonical_angle.py", "PI_RATIONAL_UPPER_BOUND"), ("_canonical_angle.py", "CANONICAL_REFLEX_EXCESS_RELATIONS"),
    ("_canonical_angle.py", "CANONICAL_ROTATION_DENOMINATORS"), ("_canonical_angle.py", "CANONICAL_SUBTURN_FAN_PREDICATES"),
    ("_density_policy.py", "CANONICAL_ROTATION_TABLE"), ("_density_policy.py", "CANONICAL_FAN_RAYS_PREDICATES"),
    ("_density_policy.py", "EVALUATION_SUBTURN_LIFT_PREDICATES"),
    ("reference/evaluation_binding_noise.py", "NOISE_PREDICATES"), ("reference/density_fan_binding.py", "_BAND_REFUSALS"),
    ("reference/adaptive_density_fan.py", "_DENSITY_EXACT_WORK_CAP"), ("reference/adaptive_density_fan.py", "_BOX_REFINEMENT_CAP"),
    ("reference/adaptive_density_fan.py", "_RESOURCE_HEIGHT_CAP"), ("reference/adaptive_density_fan.py", "ADAPTIVE_FAN_PROVEN_PREDICATES"),
    ("reference/adaptive_density_atlas.py", "DENSITY_FAN_PREPARATION_WORK_CAP"),
    ("reference/adaptive_density_band.py", "ADAPTIVE_FAN_NARROW_BAND_HALF_TANGENT"),
    ("reference/adaptive_density_band.py", "WINDOW_LAW_NARROW_BAND"), ("reference/adaptive_density_band.py", "WINDOW_LAW_VORONOI"),
    ("reference/density_residual.py", "_PRECISION"), ("reference/density_residual.py", "_MARGIN_BITS"),
)
VERSIONS = (("sympy", "1.14.0"), ("mpmath", "1.3.0"))


def _module(file):
    return importlib.import_module("cftuv_envelope." + file[:-3].replace("/", "."))


def _resolve(file, qualified):
    result = _module(file)
    for name in qualified.split("."):
        result = getattr(result, name)
    return result


def _freeze(value):
    if value is dataclasses.MISSING:
        return ["missing"]
    if isinstance(value, Enum):
        return ["enum", type(value).__module__, type(value).__qualname__, value.name, _freeze(value.value)]
    if value is None or type(value) in (str, bool, int):
        return value
    if type(value) is Fraction:
        return ["rational", value.numerator, value.denominator]
    if type(value) is Decimal:
        return ["decimal", *_freeze(tuple(value.as_tuple()))]
    if isinstance(value, type):
        return ["type", value.__module__, value.__qualname__]
    if dataclasses.is_dataclass(value):
        return ["record", type(value).__module__, type(value).__qualname__,
                *([field.name, _freeze(getattr(value, field.name))] for field in dataclasses.fields(value))]
    if type(value) in (tuple, list):
        return [type(value).__name__, *(_freeze(item) for item in value)]
    if type(value) in (set, frozenset):
        return [type(value).__name__, *sorted((_freeze(item) for item in value), key=_packed)]
    if type(value) is dict:
        return ["dict", *sorted(([_freeze(key), _freeze(item)] for key, item in value.items()), key=_packed)]
    if callable(value):
        return ["callable", value.__module__, value.__qualname__]
    raise TypeError("unsupported Density pin value: " + type(value).__name__)


def _packed(value):
    return json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=True)


def _value_digest(value):
    return hashlib.sha256(_packed(_freeze(value)).encode("ascii")).hexdigest()


_SOURCE_CACHE = {}
_VALUE_CACHE = {}
_CLASS_CACHE = {}


def _source_digest(function):
    # Живой объект удерживается: id не переиспользуется, подмена __code__ видна.
    cached = _SOURCE_CACHE.get(function)
    code = getattr(function, "__code__", None)
    if cached is not None:
        return cached[1] if cached[0] is code else "changed live code"
    digest = hashlib.sha256(inspect.getsource(function).replace("\r\n", "\n").encode("utf8")).hexdigest()
    _SOURCE_CACHE[function] = (code, digest)
    return digest


def _cached_value_digest(key, value):
    frozen = _freeze(value)
    cached = _VALUE_CACHE.get(key)
    if cached is not None and cached[0] == frozen:
        return cached[1]
    digest = hashlib.sha256(_packed(frozen).encode("ascii")).hexdigest()
    _VALUE_CACHE[key] = (frozen, digest)
    return digest


def _file_function_digest(root, file, qualified):
    text = (Path(root)/file).read_text(encoding="utf8").replace("\r\n", "\n")
    scope = ast.parse(text).body
    for part in qualified.split("."):
        node = next(item for item in scope if isinstance(item, (ast.ClassDef, ast.FunctionDef, ast.AsyncFunctionDef)) and item.name == part)
        scope = node.body
    start = min([node.lineno, *(item.lineno for item in node.decorator_list)])
    source = "".join(text.splitlines(keepends=True)[start-1:node.end_lineno])
    return hashlib.sha256(source.encode("utf8")).hexdigest()


def _assignment_digest(root, file, name, class_name=None):
    text = (Path(root)/file).read_text(encoding="utf8").replace("\r\n", "\n")
    scope = ast.parse(text).body
    if class_name is not None:
        scope = next(node.body for node in scope if isinstance(node, ast.ClassDef) and node.name == class_name)
    for node in scope:
        targets = node.targets if isinstance(node, ast.Assign) else (node.target,) if isinstance(node, ast.AnnAssign) else ()
        if any(isinstance(target, ast.Name) and target.id == name for target in targets):
            source = "".join(text.splitlines(keepends=True)[node.lineno-1:node.end_lineno])
            return hashlib.sha256(source.encode("utf8")).hexdigest()
    raise AttributeError(name)


def _class_shape(cls):
    shape = ["class", cls.__module__, cls.__qualname__, tuple((base.__module__, base.__qualname__) for base in cls.__bases__)]
    if dataclasses.is_dataclass(cls):
        fields = tuple((field.name, str(field.type), field.init, field.repr, field.compare, field.kw_only,
                        field.default, field.default_factory) for field in dataclasses.fields(cls))
        params = cls.__dataclass_params__
        shape.extend((fields, tuple(getattr(params, name) for name in ("init", "repr", "eq", "order", "unsafe_hash", "frozen"))))
    if issubclass(cls, Enum):
        shape.append(tuple((name, value.value) for name, value in cls.__members__.items()))
    shape.append(tuple(getattr(cls, "__slots__", ())))
    for name in ("__post_init__", "validate_candidate"):
        method = getattr(cls, name, None)
        shape.append(None if method is None else _source_digest(method))
    return tuple(shape)


def _class_digest(cls):
    # Дешёвый снимок схемы каждый вызов; текст методов и JSON только при изменении.
    token = [cls.__module__, cls.__qualname__, cls.__bases__, tuple(getattr(cls, "__slots__", ()))]
    if dataclasses.is_dataclass(cls):
        token.append(tuple((f.name, str(f.type), f.init, f.repr, f.compare, f.kw_only,
                            _freeze(f.default), _freeze(f.default_factory)) for f in dataclasses.fields(cls)))
        params = cls.__dataclass_params__
        token.append(tuple(getattr(params, n) for n in ("init", "repr", "eq", "order", "unsafe_hash", "frozen")))
    if issubclass(cls, Enum):
        token.append(tuple((name, _freeze(value.value)) for name, value in cls.__members__.items()))
    for name in ("__post_init__", "validate_candidate"):
        method = getattr(cls, name, None)
        token.append((method, getattr(method, "__code__", None)))
    cached = _CLASS_CACHE.get(cls)
    if cached is not None and cached[0] == token:
        return cached[1]
    digest = _value_digest(_class_shape(cls))
    _CLASS_CACHE[cls] = (token, digest)
    return digest


# DENSITY_SHAPES_BOUND_ONCE_V1 (владелец, 2026-10-19): схема класса (базы, слоты, поля и параметры датакласса, члены enum) снимается при ПРИВЯЗКЕ —
# первая проверка процесса (прогрев в старте воркера), после `unbind()` (`pin.refresh`) или когда атрибут модуля называет другой объект. Изменение схемы
# привязанного класса на месте — вне контракта. На каждом вызове остаётся: атрибут модуля — тот же привязанный класс, проверяемые методы — те же объекты
# с тем же `__code__` (подмена кода видна сразу). Привязка с несовпавшим дайджестом остаётся отказом до перепривязки.
_METHOD_NAMES = ("__post_init__", "validate_candidate")
_BOUND = {}


def unbind():
    """Forgets the bound classes: the next check takes every schema again."""

    _BOUND.clear()


def _live_module(file):
    name = "cftuv_envelope." + file[:-3].replace("/", ".")
    module = sys.modules.get(name)
    return module if module is not None else importlib.import_module(name)


def _methods(cls):
    return tuple((name, method, getattr(method, "__code__", None)) for name in _METHOD_NAMES for method in (getattr(cls, name, None),))


def _bound_class_digest(file, qualified):
    cls = _live_module(file)
    for name in qualified.split("."):
        cls = getattr(cls, name)
    bound = _BOUND.get((file, qualified))
    if bound is not None and bound[0] is cls and all(getattr(cls, name, None) is method and getattr(method, "__code__", None) is code for name, method, code in bound[1]):
        return bound[2]
    digest = _class_digest(cls)
    _BOUND[(file, qualified)] = (cls, _methods(cls), digest)
    return digest


# Закрытые части общего enum: добавление чужого исхода не меняет Density oracle.
ENUM_SUBSET_NAMES = (
    ("reference/contracts.py", "ReferenceOutcome", (
        "REFERENCE_CERTIFIED_PREDICATE_UNDECIDABLE", "DENSITY_RATIONAL_AUTHORITY_EXHAUSTED",
        "DENSITY_FAN_BOX_REFINEMENT_EXHAUSTED", "DENSITY_WINDOW_CHART_UNREPRESENTABLE",
        "DENSITY_SEALED_FAN_INVALID", "EVALUATION_BINDING_NOISE_LAW_NOT_APPLIED", "CANONICAL_FAN_RAYS_LAW_NOT_APPLIED",
        "DENSITY_DECIMAL_SHELL_V2_APPLIED", "DENSITY_DECIMAL_GRID_POINT_V2",
        "DENSITY_DECIMAL_NON_DYADIC_GRID_STRAW_V2", "DENSITY_SOURCE_CONSTRUCTION_V1_APPLIED")),
)


def _enum_subset(file, qualified, names):
    enum = _resolve(file, qualified)
    return tuple((name, getattr(enum, name).value) for name in names)


def _shared_source_stale(root):
    failures = []
    for file, qualified, wanted in FUNCTIONS:
        try:
            if _file_function_digest(root, file, qualified) != wanted:
                failures.append(file + ":" + qualified)
        except (OSError, SyntaxError, StopIteration, AttributeError, TypeError):
            failures.append(file + ":" + qualified + " (missing/invalid)")
    for file, name, wanted in SOURCE_CONSTANTS:
        try:
            if _assignment_digest(root, file, name) != wanted:
                failures.append(file + ":" + name + " (source constant)")
        except (OSError, SyntaxError, StopIteration, AttributeError, TypeError):
            failures.append(file + ":" + name + " (source constant missing/invalid)")
    for file, class_name, name, wanted in SOURCE_ENUMS:
        try:
            if _assignment_digest(root, file, name, class_name) != wanted:
                failures.append(file + ":" + class_name + "." + name + " (source enum)")
        except (OSError, SyntaxError, StopIteration, AttributeError, TypeError):
            failures.append(file + ":" + class_name + "." + name + " (source enum missing/invalid)")
    return tuple(failures)


def runtime_stale():
    failures = []
    for package, wanted in VERSIONS:
        try:
            if importlib.import_module(package).__version__ != wanted:
                failures.append(package + ":version")
        except (ImportError, AttributeError):
            failures.append(package + ":version (missing)")
    for file, qualified, wanted in FUNCTIONS:
        try:
            if _source_digest(_resolve(file, qualified)) != wanted:
                failures.append(file + ":" + qualified + " (live)")
        except (ImportError, AttributeError, OSError, TypeError):
            failures.append(file + ":" + qualified + " (live missing/invalid)")
    for file, qualified, wanted in CONSTANTS:
        try:
            if _cached_value_digest((file, qualified), _resolve(file, qualified)) != wanted:
                failures.append(file + ":" + qualified + " (constant)")
        except (ImportError, AttributeError, OSError, TypeError, ValueError):
            failures.append(file + ":" + qualified + " (constant missing/invalid)")
    for file, qualified, wanted in CLASSES:
        try:
            if _bound_class_digest(file, qualified) != wanted:
                failures.append(file + ":" + qualified + " (shape)")
        except (ImportError, AttributeError, OSError, TypeError, ValueError):
            failures.append(file + ":" + qualified + " (shape missing/invalid)")
    for file, qualified, names, wanted in ENUM_SUBSETS:
        try:
            if _cached_value_digest((file, qualified, names), _enum_subset(file, qualified, names)) != wanted:
                failures.append(file + ":" + qualified + " (enum subset)")
        except (ImportError, AttributeError, TypeError):
            failures.append(file + ":" + qualified + " (enum subset missing/invalid)")
    return tuple(failures)


def stale_sources(root):
    return _shared_source_stale(root)


def expected_closure_digest():
    """Идентичность ожидаемого контракта; не читает runtime oracle."""
    manifest = (FUNCTIONS, CONSTANTS, CLASSES, ENUM_SUBSETS, VERSIONS, SOURCE_CONSTANTS, SOURCE_ENUMS)
    return hashlib.sha256(_packed(manifest).encode("ascii")).hexdigest()


SOURCE_CONSTANTS = (('_authoring_intent.py',
  'CANONICAL_RESTORATION_ARTIST_ERROR',
  '9295ee7fd4ce43dc2ddf90d4cd3de2103dae07f063a69b87274f3102bc5f1cb2'),
 ('_canonical_angle.py', 'PI_RATIONAL_UPPER_BOUND', '0f23985491fde44d951fe5a97e5e47381998ae83ab55abe66833fa08fa2ba1b1'),
 ('_canonical_angle.py',
  'CANONICAL_REFLEX_EXCESS_RELATIONS',
  'beab1ed10806792ef2054dcf4bcd97ceab5295027e3fbd31b32f5bef645ecaa7'),
 ('_canonical_angle.py',
  'CANONICAL_ROTATION_DENOMINATORS',
  'c5e58d860266ffa7d79e6956da9fd55dc46a84634785199a7bb333d32152d0b7'),
 ('_canonical_angle.py',
  'CANONICAL_SUBTURN_FAN_PREDICATES',
  '0de3868f3580b86686686d74b3ac16f57640f475c27fc3b9c595afd93ba6879a'),
 ('_density_policy.py', 'CANONICAL_ROTATION_TABLE', 'cbf1e2aa765b04d5e96cd7a9872cfbb37fc5a6b89c46bb182df00bced013eef6'),
 ('_density_policy.py',
  'CANONICAL_FAN_RAYS_PREDICATES',
  '4758fe5607fc7f52dfa58e7963cd62a8868c30138cbf6b0e6f5db25e14b2e195'),
 ('_density_policy.py',
  'EVALUATION_SUBTURN_LIFT_PREDICATES',
  '1fd639415c0fb34c55bf28fcb359673b4e0fe8163a66806da50b39da9777599f'),
 ('reference/evaluation_binding_noise.py',
  'NOISE_PREDICATES',
  '9351d0b77b317e611e756c71d33380545d6fffc065ae5e6bfad90087d57d6bb5'),
 ('reference/density_fan_binding.py',
  '_BAND_REFUSALS',
  '50e070eb858f55617b00b5f33183cb0e8843145431c676c3d4c998e828a787b5'),
 ('reference/adaptive_density_fan.py',
  '_DENSITY_EXACT_WORK_CAP',
  '5b1eb7cb5cc53d78a6b2202bd4fd62fe194c7553e16044a2d43086ce8b4f4982'),
 ('reference/adaptive_density_fan.py',
  '_BOX_REFINEMENT_CAP',
  '5469b40a4bffe91236b6506c9c38a00cc05dc08cb16729667d0f2dd31908de32'),
 ('reference/adaptive_density_fan.py',
  '_RESOURCE_HEIGHT_CAP',
  '7a9a631b4369e9b3c3335e2caa3d4246a85553ffd37d6bf18f2f9e8a85948614'),
 ('reference/adaptive_density_fan.py',
  'ADAPTIVE_FAN_PROVEN_PREDICATES',
  'f46a949ff465f075dddc2e988d3c6cf4eeb09499c55a2191fbc503375dcf7d0b'),
 ('reference/adaptive_density_atlas.py',
  'DENSITY_FAN_PREPARATION_WORK_CAP',
  '814bca737f4ba8ac33f2a1a3e0bceb925a3ea05d4d47f773796e47458278f20d'),
 ('reference/adaptive_density_band.py',
  'ADAPTIVE_FAN_NARROW_BAND_HALF_TANGENT',
  '5f5d64d961b5ad061a51c9dbf38d63eff0c89dc839f69db6052b7a41ce6e7817'),
 ('reference/adaptive_density_band.py',
  'WINDOW_LAW_NARROW_BAND',
  'e9cbbcab8f912bfde55247d77128295596d2e0dccd90d56333d069a7a75c6e85'),
 ('reference/adaptive_density_band.py',
  'WINDOW_LAW_VORONOI',
  'd15698c2eda230d0d76e0df78bc6c3621a168ea37c4504a984a8b7f296e66761'),
 ('reference/density_residual.py', '_PRECISION', '876b079cb17e34440fe4fef581075a924743dfa5e1d5b8a8c6438b0254e3fc01'),
 ('reference/density_residual.py', '_MARGIN_BITS', '75751a7683e0b1960b80b5f2d2daeca76ae43b5616a34af56a8bf31808b2d8df'))
SOURCE_ENUMS = (('reference/contracts.py',
  'ReferenceOutcome',
  'REFERENCE_CERTIFIED_PREDICATE_UNDECIDABLE',
  '965c1f904274389bbe93ea3daa276116e7c78c1e7fb6f896886207193ce82a02'),
 ('reference/contracts.py',
  'ReferenceOutcome',
  'DENSITY_RATIONAL_AUTHORITY_EXHAUSTED',
  'aeabb6218ceda3415bfe6adcd8416000ccbaf4e8ab5426d857b7b1e0df303e6b'),
 ('reference/contracts.py',
  'ReferenceOutcome',
  'DENSITY_FAN_BOX_REFINEMENT_EXHAUSTED',
  '139b2edbdbec4af1e479858148e0651f27de10b4aeee317c90568dbf0d5ea779'),
 ('reference/contracts.py',
  'ReferenceOutcome',
  'DENSITY_WINDOW_CHART_UNREPRESENTABLE',
  '0679d28d8cddd513dbfa317c95d5a782efeb0491a4d338eba3b9eb078d816438'),
 ('reference/contracts.py',
  'ReferenceOutcome',
  'DENSITY_SEALED_FAN_INVALID',
  '4eef850f5c3422a2eeb878ad5aec9b226e270384b4ba1fef2270c84f4bd8b40a'),
 ('reference/contracts.py',
  'ReferenceOutcome',
  'EVALUATION_BINDING_NOISE_LAW_NOT_APPLIED',
  'dd8485067c9226abbe9a0d8bbf2c3e7de715aef2e5c974ed9a405bdf542460e3'),
 ('reference/contracts.py',
  'ReferenceOutcome',
  'CANONICAL_FAN_RAYS_LAW_NOT_APPLIED',
  '14c9d7cf1a7e31664133da297ab737cc0cf029f1ca78f4a51fb1d9b3d9a069c5'),
 ('reference/contracts.py',
  'ReferenceOutcome',
  'DENSITY_DECIMAL_SHELL_V2_APPLIED',
  '4a7d901d08bb0d3c77078e5bba57cb2438b94b29ae295709e800a6995c93fa1d'),
 ('reference/contracts.py',
  'ReferenceOutcome',
  'DENSITY_DECIMAL_GRID_POINT_V2',
  '67d12e60c9a2bf622c762a38273e720c121fcf4007590a7f269d7fd8358ceccc'),
 ('reference/contracts.py',
  'ReferenceOutcome',
  'DENSITY_DECIMAL_NON_DYADIC_GRID_STRAW_V2',
  '588c9421b7b7c1053b1ad5d57825e3f824e114a10bb423fa0909ac584e9cb480'),
 ('reference/contracts.py',
  'ReferenceOutcome',
  'DENSITY_SOURCE_CONSTRUCTION_V1_APPLIED',
  '11a110e8b65b837e4b2b72afade8cd465416e63bc0fca4b165261d999e773333'))


# Regenerated only after the accepted oracle closure has stabilised.
FUNCTIONS = (('_density_policy.py',
  'density_interval_enclosure',
  '3514c9cfeeaf19ee6cfdc865124ec173a2fa7008cd50c23c52893c1538a363c1'),
 ('_density_policy.py',
  'huber_density_value_contract',
  '558c2ac1678fe6a2532694733f515dcea4d9ea1014d26ad3ec655f7104015776'),
 ('reference/planar_types.py', '_expr', '85200e0cd4ad1bc24f5a52feca4a088f84835cf29f16910ebb5e8d35fd64223e'),
 ('reference/planar_types.py', 'exact_sign', '5b313b77356479efb25ec16a87b2a7c65c55a94808fbd76cccc48f0a26c1c3a6'),
 ('reference/planar_types.py',
  'exact_quadratic_value',
  'f4bf16be097ffa4ac152c980ce76c2c21aa7fdc9f15aac790d39380bad7c00b3'),
 ('reference/common.py', 'stable_id', '8875810a74aa27d1904b45ea906dc297c102bfe41af4463438ffdbbd3ec52221'))
CONSTANTS = (('_authoring_intent.py',
  'CANONICAL_RESTORATION_ARTIST_ERROR',
  'edbecfef57323851a4f375a7620486f45e7588e526e16f4ee8e955b1ee3a40b2'),
 ('_canonical_angle.py', 'PI_RATIONAL_UPPER_BOUND', '0574eb70cfd5b9276a97214f0e5571ec7360330494ba12639a454187b9c4e02f'),
 ('_canonical_angle.py',
  'CANONICAL_REFLEX_EXCESS_RELATIONS',
  '25acdb35a8bf090f31410bd7e3f0df36acf95b6d655a019d7a3a411f81104a5f'),
 ('_canonical_angle.py',
  'CANONICAL_ROTATION_DENOMINATORS',
  '0cb35aed9d763ffa36e34e11026ab68eeaef8c913738321aa7a32b8900d89c96'),
 ('_canonical_angle.py',
  'CANONICAL_SUBTURN_FAN_PREDICATES',
  '0c827eb0c09146d278ec1a58e615b3d8da8c9343a44ca3699789c72f35d84699'),
 ('_density_policy.py', 'CANONICAL_ROTATION_TABLE', '06ef1fd897cddbafce5a400ac3b115d29e085782ca575b8b472852ebce2f7665'),
 ('_density_policy.py',
  'CANONICAL_FAN_RAYS_PREDICATES',
  '6720835ff4ddec7114cbe84ad35bbf845308685600f83469a5a49488fc3ed6e0'),
 ('_density_policy.py',
  'EVALUATION_SUBTURN_LIFT_PREDICATES',
  '644d03204e13efef9dd07a5c0173e2077b3431c31884c49b9eed304322f2f819'),
 ('reference/evaluation_binding_noise.py',
  'NOISE_PREDICATES',
  '342730a08708a1378715afa9e039eb3c72962e30d930af9a2178de19e040e3d9'),
 ('reference/density_fan_binding.py',
  '_BAND_REFUSALS',
  '48e6325da951b10400772bd29365b2abc103b2140b1253530689a3c12949d70f'),
 ('reference/adaptive_density_fan.py',
  '_DENSITY_EXACT_WORK_CAP',
  'f1de634787fe5cc0fe96b0b7f617755847d22b30d06aee392ffb5787d7c9f1c9'),
 ('reference/adaptive_density_fan.py',
  '_BOX_REFINEMENT_CAP',
  '7b1a278f5abe8e9da907fc9c29dfd432d60dc76e17b0fabab659d2a508bc65c4'),
 ('reference/adaptive_density_fan.py',
  '_RESOURCE_HEIGHT_CAP',
  '6cce36d9f8a9e151b100234af75cca89d55bcb94c153f51847debdf1f39cae45'),
 ('reference/adaptive_density_fan.py',
  'ADAPTIVE_FAN_PROVEN_PREDICATES',
  'e96cb1304793e7dca3ae1302c715094693e812c34e361fd8dffaf7cede05a9ca'),
 ('reference/adaptive_density_atlas.py',
  'DENSITY_FAN_PREPARATION_WORK_CAP',
  '0f255ec16482c343fd099ba9668863778be40dc166a6856b1d470e0bc97cc6fd'),
 ('reference/adaptive_density_band.py',
  'ADAPTIVE_FAN_NARROW_BAND_HALF_TANGENT',
  '29756d55af0994067f0fa2a8001c12057a2cb6670d8d5e0015703ecc5b4a7834'),
 ('reference/adaptive_density_band.py',
  'WINDOW_LAW_NARROW_BAND',
  '7667a0cf56068fb12deecd4a73ff65fff0b379816702bdb53ea5708aa3070a88'),
 ('reference/adaptive_density_band.py',
  'WINDOW_LAW_VORONOI',
  'ad3602b6377575c6f7b72229ff03c356200e9a83ec374effc4ff28d93cb0378f'),
 ('reference/density_residual.py', '_PRECISION', 'a512db2741cd20693e4b16f19891e72b9ff12cead72761fc5e92d2aaf34740c1'),
 ('reference/density_residual.py', '_MARGIN_BITS', 'ad57366865126e55649ecb23ae1d48887544976efea46a48eb5d85a6eeb4d306'))
CLASSES = (('contracts/analysis.py', 'TurnOrientation', '5870525e36943a3896e7a6bf88cb1e6172a0c29a6bf709c03d6d15b8c69e5039'),
 ('contracts/envelopes.py',
  'AdaptiveFareyHeightRangeWitnessV2',
  'bd78ebe1822985597809d071a6b9f7cf71ca700501513dc82d5ca9014163fd15'),
 ('contracts/envelopes.py',
  'AdaptiveMinimalRationalFanAuthorityV2',
  'aab9087bd24643733069e414531b0d1d228a0224a30ff5f767e14200822be4b3'),
 ('contracts/envelopes.py',
  'AdaptiveProjectivePoleOwnershipV1',
  '5acb4c8e5cbaf55c68506ebeccc0791cf7a234ffb285d7776d7580888131b623'),
 ('contracts/envelopes.py',
  'AdaptiveRationalFanOrdinalWindowAtlasV1',
  '922ffbebabec9344cf0f25b91581859ecc5329ad2f0c4fcfcf98e01f50f31e3e'),
 ('contracts/envelopes.py',
  'AdaptiveRationalFanOrdinalWindowV2',
  'faea7cd43b158c69e7c1997e92d521ad62af362bcfc53a28dff0432c3abe547d'),
 ('contracts/envelopes.py',
  'AdaptiveRationalFanProjectiveChartPieceV1',
  '85275a91c0f64302aebedd150c61c3d0ac36b99fec4bc2a50ac817bd3232d462'),
 ('contracts/envelopes.py',
  'CanonicalFanRaysLawV1',
  '2c31c32614ca28f0c8e70453f57bc98656f17aa7fbd138582c52fc7de8176a9d'),
 ('contracts/envelopes.py',
  'CanonicalRationalRotationFanAuthorityV1',
  '88b949323dc6945dca692463ef57f746ebbd1889a8e6ae7985356cd9005214ee'),
 ('contracts/envelopes.py',
  'CanonicalReflexAngleRelationV1',
  '992d4675af4cbb4ebceb34867ab92c198e4cad50a4db40ef27de2f094b282a6e'),
 ('contracts/envelopes.py',
  'CanonicalSubturnFanAuthorityV1',
  '718f583b46ab4c4a3293c3263f903481f7bcf7c4dd903ff5a869e9911756b091'),
 ('contracts/envelopes.py',
  'DirectionBindingCertificateV1',
  '96d6e127514d3517eac2045e6bedd627604422536aa803f0a2b2e06d85da5e9a'),
 ('contracts/envelopes.py',
  'DirectionBindingReasonV1',
  'dda80381abdbdba6f5e7390cfd59249553b0843fe52194ee739fb1244df0b508'),
 ('contracts/envelopes.py',
  'EvaluationGeometryDirectionBindingCertificateV1',
  '1f193c99aa8615d189cf30135c0e59c1ffc6dda4e750e20bcb9ca120a1bd9a28'),
 ('contracts/envelopes.py',
  'EvaluationGeometrySubturnCountLiftLawV1',
  '712504545e983432bf9fc7d2c5f2413d5e0be95921b7b68c51ac4e6e0844c16e'),
 ('contracts/envelopes.py',
  'EvaluationGeometrySubturnCountLiftV1',
  'a7b0f8aaf08a94d421f9fc2d24067ad20f56527484fd658c27a9bef920a9025e'),
 ('contracts/envelopes.py', 'ExactTurnSignV1', 'c25b2516c47cf18b9e0a629f7e70240886695c7d15e10302e6c7ade87d1c1063'),
 ('contracts/envelopes.py',
  'SubturnGuaranteeLawV1',
  'e6891b049af4df1fc52a5bea499d80e640819f968f886201e0859571b2ca04a3'),
 ('contracts/metric.py', 'ExactRationalV1', '10e13485ce35c10fcf96a4783ee6defdea33775bc18311c34bdacdd0d26a61f1'),
 ('ids.py', 'EnvelopeSpecId', 'a0fa12d7607645d75c58556e7785e27e51249e39ebd8cc8d013321eb25d85f14'),
 ('ids.py', 'SelectionCertificateId', '9c747c1a17d9a453e9d6dd5551940263deb53301013f5e607977afa041dcffa0'),
 ('numeric.py', 'CertifiedDecimalIntervalV1', '44fa0927fa62707412c42ed7d5ec7e79e586321695b5291e2356201164c3f032'),
 ('numeric.py', 'ExactRatioV1', '16aa3b3dad26659d707081333b5c178718363f8151598697d37c539811c819a5'),
 ('numeric.py', 'IntervalEndpointKind', 'bf6c276e22d783b6464361b7dde1e43ee8f6167afdd6c624d06987fe96bbd4a2'),
 ('reference/contracts.py',
  'CanonicalFanRaysRefusalV1',
  'abd0e384ee7b3501a3b8e2fdba0314062317c13567e0cd7818e30ac33e526202'),
 ('reference/contracts.py',
  'EvaluationBindingNoiseEffectV1',
  '10bbf5d1ec8b91a5728615d7fc319e533f54f104f8723e7531d56f08d136dd6d'),
 ('reference/contracts.py',
  'EvaluationBindingNoiseLawV1',
  '720ba3f37bc953038280208c57535a717471171e9a20001463a151be76761421'),
 ('reference/contracts.py',
  'EvaluationBindingNoiseOnCanonicalAngleV1',
  '9a5e441a3277be783fc25be588afa49805a8eaa44fb08b04ce12f29a1160d094'),
 ('reference/contracts.py',
  'EvaluationBindingNoiseRefusalV1',
  'b324da65b716fae187ab4b4e06697961deee835d398bfb8300659c520b17b81e'),
 ('reference/contracts.py',
  'ReferenceCompileResultV1',
  '26c79e7b1b0e1a3cfa60835d6c63c18bd5e4bc2dc8a8bc7aaf7f60820aff8d06'),
 ('reference/contracts.py',
  'ReferenceDiagnosticSeverity',
  'd34a29e35f1414793475ab08de332c9066783b83025252e7738dd1d3ec65ebf6'),
 ('reference/contracts.py',
  'ReferenceEvaluationDiagnosticV1',
  'c06fd4c1cebdea15965a80b77650412ea7e64349b203542c575279b4d771f432'),
 ('reference/density_angle_v1.py', 'AngleAtomV1', '230b848539221761222feff490be57340a1b4da5de2961e062212b4e81bdf0eb'),
 ('reference/density_angle_v1.py', 'AngleStatsV1', '3c818d82a8c614bb41c2c209496a461cbe5a69262ce469d5e06e8000ee40bde0'),
 ('reference/density_angle_v1.py',
  'AngleV1ErrorCode',
  'bc2816f35d2fa0ebe4b9a75745b0e2ed88f035974ba1a27b4b3f81bf5a59f8ba'),
 ('reference/density_angle_v1.py',
  'FractionIntervalV1',
  'd92c4c3f076e760552b208333b8a1b2a558b8411efd248b28c020e89c84ecd52'),
 ('reference/density_angle_v1.py',
  'RayEnclosureV1',
  '4b894f9c25de591a3dca82192483b246e0ab62302fa1a547e0472baa3eccb6f3'),
 ('reference/density_angle_v1.py',
  'RootEnclosureV1',
  'd36cf492ecf54785b8f41c6d7ffb0e2b373405047a2dae46960bfba045fa271c'),
 ('reference/density_angle_value_v1.py',
  'AngleValueErrorCode',
  '44c32e4bd00775146f875b70eccfa2a6de6f5d3978c5308cdf99344644ec9941'),
 ('reference/density_angle_value_v1.py',
  'PrincipalComponentV1',
  '58b0347ac584b0bb04cd227b8bcb9fc9a3e219f4aa87b4a53cfce0f86accc2c1'),
 ('reference/density_construct_v1.py',
  'DensityConstructProfileCode',
  'b00b72823cc9eda8087995b88c7b1529828f0cd9d91b7c0ed1485613047f0561'),
 ('reference/density_construct_v1.py',
  'ExpandLimitsV1',
  '8389062f67b212c8f14879be7c63c83d2d2b1270cc445e8095e3914deac296ca'),
 ('reference/density_construct_v1.py',
  'OwnedExprV1',
  '95248a782398e7eb8324c9e22b7228d78fe4f977d9a285395ecd0c7558d64256'),
 ('reference/density_construct_v1.py',
  'OwnedFormV1',
  'ee8b19b5f3df682ccae5a59e48c88450d27e5381f7ececcc04ef46beb38424ff'),
 ('reference/density_construct_v1.py',
  'OwnedVec2V1',
  'a662c023180c2582982130e9def0b2aaf911690f8ebfe5c516a28fe9fc1da79b'),
 ('reference/density_construct_v1.py', '_Node', '41e6225a6e2ef783c008fad4864dadad1a97367232ab98b5007578d2a2f668f4'),
 ('reference/density_construct_v1.py',
  '_NonzeroWitnessV1',
  '88ba8089fcd761544fa5edd45cf31493766e99d94fb36608e6616c0e9663ef02'),
 ('reference/density_oracle_v1.py',
  'AdaptiveFanAuthorityFactsV1',
  '4b7f45bf4e2476542e2d29fe8f97c85863b87502c560fe62efde470741a07e6b'),
 ('reference/density_oracle_v1.py',
  'AtlasWindowFactsV1',
  '2d9d6a75dce315505f23efa1f4b39219cf82accebeec1aa4505a24cc3d6c5ce0'),
 ('reference/density_oracle_v1.py',
  'BindingCertificateFactsV1',
  'de39ae2415090f403ccace06216a91cf6f123be7ba62ca06abe132b3a50bb879'),
 ('reference/density_oracle_v1.py',
  'CanonUsageFactsV1',
  'cc620b40f5c60de6ffc6774a526eae7862f66427568dd300ab121f44d8680778'),
 ('reference/density_oracle_v1.py',
  'CanonicalFanAuthorityFactsV1',
  '4ca9eda1d161475515c3bdce3d664e2a3e6c9659408d37ebf73deba675f74427'),
 ('reference/density_oracle_v1.py',
  'CanonicalLawRefusalFactsV1',
  '7ea5dd6b95e1192b1259f24431fe53b392e4e0a0e29d971673961b258d9adeae'),
 ('reference/density_oracle_v1.py',
  'CanonicalSubturnAuthorityFactsV1',
  'd9f5ba2d2d802b745191959535f17856866ae2ea15e42105b2bf28f043ffeff8'),
 ('reference/density_oracle_v1.py',
  'ChartPieceFactsV1',
  '915e5657e9452fec1dd4543940dcf29b266c36138db66a78543bf78cc0ebb5af'),
 ('reference/density_oracle_v1.py',
  'CountLiftFactsV1',
  'ca2e36be19c8c806df8d6442699f625b335e45375a062596f66ab58f39979588'),
 ('reference/density_oracle_v1.py',
  'DecimalIntervalFactsV1',
  '0ad67e22bc9094c0c58757efdbc7f844dbefd856932630e63406ec6eb4c3c02e'),
 ('reference/density_oracle_v1.py',
  'DecimalValueFactsV1',
  'f652dc3b829730dc69ff4124c2d5b94d35842c3b01edf49872bb61d1a1fe953c'),
 ('reference/density_oracle_v1.py',
  'DensityFailureFactsV1',
  'c60188fb12f27bc50eb05c6e7212517666d991f106c41ea91a6bfcdf6cb862c9'),
 ('reference/density_oracle_v1.py',
  'DensityOracleCanonV1',
  '119708bc2616773cffbc5ee1bd0086dfb6f02fa9fd141c01601eeed652e74931'),
 ('reference/density_oracle_v1.py',
  'DensityOracleLimitsV1',
  '25943ea3161f076b5059195b1dd956257ff486faefeec8bd6a9175f0b2b31fc5'),
 ('reference/density_oracle_v1.py',
  'DensityOracleRequestV1',
  '62f052bc1b74281ca4b39cbe878ea6687d3110d8c66bb45bb0098e307718413c'),
 ('reference/density_oracle_v1.py',
  'DensityPriceArticleV1',
  '4b734fb6f39bf320ff684fb788b52555094cf1f42ca7b59944358715df9230a8'),
 ('reference/density_oracle_v1.py',
  'DensityShellEventFactsV1',
  '5ef46d4d12db30145e3a0f31345ef45e71f5c31dc12ed200973c4b17ad211562'),
 ('reference/density_oracle_v1.py',
  'FareyHeightWitnessFactsV1',
  'd980b335242173a547e48b7d7f54e928418c2ff23d6c1dd5829c36faba6432a1'),
 ('reference/density_oracle_v1.py',
  'LocalWindowFactsV1',
  '8b276c512ec8f68bd17e611cb8c61d88b2c8509f4b49df329e0c01fd8dcd61ec'),
 ('reference/density_oracle_v1.py',
  'NoiseRecordFactsV1',
  '5a4fe7360e08f4829004b0557bde73d7b10fcf87af3a3f93504f096d318d78de'),
 ('reference/density_oracle_v1.py',
  'RawDensityBatchInputV1',
  'a6f2066528a551b9e1dd95b9c9b32d7d244fbe6d8d160b6883f30ec1449e39b9'),
 ('reference/density_oracle_v1.py',
  'RawDensityCanonicalFactsV1',
  'e2e10aed00957be9ad164d27d091894c517e5079cbfaa2c9983c70cf9d87157a'),
 ('reference/density_oracle_v1.py',
  'RawDensityGeometryV1',
  '43a091274bcc88c857919ca6fc1047ca8b22ad82a4d5bec25abdf09916d3cdbc'),
 ('reference/density_oracle_v1.py',
  'RawDensityIncidentV1',
  '7b324d608b633f333faa053b2dc0f3c021054d1f13e5a119bb762dfd96e9043f'),
 ('reference/density_oracle_v1.py',
  'RawDensityMetricV1',
  'f2c556a9db0cb7f0723249ee04c7fe136949e50ad927a9f1f6f27563bfe3611d'),
 ('reference/density_oracle_v1.py',
  'RawDensityPointV1',
  '101a0320f00194d779711c6ad4775b4a763f8e7ed1ab2a49db545726e97339f6'),
 ('reference/density_oracle_v1.py',
  'RawDensitySelectionV1',
  'f639477e9066fca86eba8e496ed8ff9eff462df7c7a1cbeef538db09a6ab07de'),
 ('reference/density_oracle_v1.py',
  'RawDensitySpecInput',
  '1dfeea5094032e920ec0e9ce6c2164ce1ff63f7bd9505b5628f1c380171ab44b'),
 ('reference/density_oracle_v1.py',
  'RawDensitySpecResultV1',
  '011d2518ef96b662e64d7ab2e1736e2848cc7dbcd42998eef7ecf17d7cf03fe1'),
 ('reference/density_scalar_v2.py',
  'DensityScalarProfileCode',
  '40eff96e0ee12b3c24dba9f0552a0f94147e21cb14f70900207ad60a05bf2574'),
 ('reference/density_scalar_v2.py',
  'ScalarLiteralAstV1',
  '20e411c24a2d7a0c8da5c4350845c91df151cbcba675ac041e993aefc2571218'),
 ('reference/density_scalar_v2.py', 'ScalarV2', '166adc9faedf31719acb757bf1dc61c3d5d9ffff0069a7677989f8b028b64916'),
 ('reference/density_attempt_memo_v1.py',
  'DensityAttemptKeyV1',
  'b008488cbd0fccdc111abadaf357198faeabc9fb08e3d7a72a8ecf8944b9a909'),
 ('reference/density_attempt_memo_v1.py',
  'DensityAttemptReceiptV1',
  '5fbea31716f09a7204695371a2bbcd629dd98640cbf3664e99a6ef37fdde91bd'),
 ('reference/density_attempt_memo_v1.py',
  'DensityAttemptMemoSnapshotV1',
  'caf78524a99c98b700ce83817ff9fa82d28e0c974b74a83552a61a0a426e2103'),
 ('reference/density_attempt_memo_v1.py',
  'DensityMemoBatchResultV1',
  '768d86d0b1a0d829fae0b34775cd75e3cdbdd941ee7da4a981b0f848b38dcc9a'))
ENUM_SUBSETS = (('reference/contracts.py',
  'ReferenceOutcome',
  ('REFERENCE_CERTIFIED_PREDICATE_UNDECIDABLE',
   'DENSITY_RATIONAL_AUTHORITY_EXHAUSTED',
   'DENSITY_FAN_BOX_REFINEMENT_EXHAUSTED',
   'DENSITY_WINDOW_CHART_UNREPRESENTABLE',
   'DENSITY_SEALED_FAN_INVALID',
   'EVALUATION_BINDING_NOISE_LAW_NOT_APPLIED',
   'CANONICAL_FAN_RAYS_LAW_NOT_APPLIED',
   'DENSITY_DECIMAL_SHELL_V2_APPLIED',
   'DENSITY_DECIMAL_GRID_POINT_V2',
   'DENSITY_DECIMAL_NON_DYADIC_GRID_STRAW_V2',
   'DENSITY_SOURCE_CONSTRUCTION_V1_APPLIED'),
  'f76c3c18c6cd17a4d3348dfcc34ab8eeaefabd76b69e4857b832fb3bf4271738'),)
