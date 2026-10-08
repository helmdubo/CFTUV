"""Аудит использования sympy и mpmath в ядре: каждое место названо, классифицировано и заморожено.

Шаг 1 плана SYMPY-OFF-HOT-PATH. Это исполняемая форма аудита: таблица `SYMPY_AUDIT` ниже — полный
перечень модулей `kernel/src/cftuv_envelope`, которые импортируют `sympy` или `mpmath`, с классом
каждого и ПОЛНЫМ набором используемых возможностей. Тест перечитывает исходники (`ast`) и падает, когда

* появился модуль, которого нет в таблице (новый потребитель sympy обязан быть классифицирован);
* модуль стал пользоваться возможностью, которой в его строке нет (новая возможность может поменять
  класс: `sp.roots` в модуле класса (а) — уже не класс (а));
* строка таблицы ссылается на модуль, которого больше нет (таблица не копит мёртвые строки).

Классы:

* `REPLACED_HOT` — класс (а), горячее место, заменено за переключателем
  `reference/symbolic_backend.py` (`SYMPY` | `NATIVE_EXACT` | `SHADOW`);
* `A_LATER` — класс (а): рациональные числа и суммы корней из рациональных, но место холодное либо
  это клей; остаётся на sympy до шага 3, цена измерена и мала;
* `A_DONE` — уже без sympy-арифметики (прецедент: фактор-свободный канон классов);
* `B_KEEP` — общая алгебра, sympy нужен: тригонометрия рациональных кратных пи, вложенные
  радикалы (`sqrt(10 - 2*sqrt(5))` при q=5), корни многочленов, интервальная тригонометрия;
* `BRIDGE` — сам мост sympy <-> родная арифметика.

Доли времени (cProfile `building` patch 6/7/1, плотность 2, основа 6288950) лежат в
`artifacts/sympy_off_hot_path/AUDIT.json`; здесь только классы, потому что число в тесте — слух.
"""

from __future__ import annotations

import ast
from pathlib import Path

KERNEL = Path(__file__).resolve().parents[1] / "src" / "cftuv_envelope"
HOST = Path(__file__).resolve().parents[2] / "cftuv"

CLASSES = {"REPLACED_HOT", "A_LATER", "A_DONE", "B_KEEP", "BRIDGE"}

SYMPY_AUDIT = {
    "_density_policy.py": (
        "B_KEEP",
        'интервальные оболочки sin/cos/atan/pi (iv.*): тригонометрия угла, не сумма корней',
        (
            "mp.iv",
            "mp.iv.atan2",
            "mp.iv.cos",
            "mp.iv.mpf",
            "mp.iv.pi",
            "mp.iv.sin",
            "mp.iv.sqrt",
            "sp.Expr",
            "sp.atan",
            "sp.atan2",
            "sp.cos",
            "sp.pi",
            "sp.sin",
        ),
    ),
    "interactions/equality_locus.py": (
        "A_LATER",
        'только Expr в аннотациях: арифметика идёт через ExactScalar',
        (
            "sp.Expr",
        ),
    ),
    "interactions/policy_b.py": (
        "A_LATER",
        'Expr/Integer: клей Policy B, значения читает ExactScalar',
        (
            "sp.Expr",
            "sp.Integer",
        ),
    ),
    "reference/adaptive_density_band.py": (
        "B_KEEP",
        'полоса веера плотности: значения q=5 вложенные радикалы',
        (
            "sp.Rational",
        ),
    ),
    "reference/adaptive_density_fan.py": (
        "B_KEEP",
        'sqrt(5) и sqrt(10-2*sqrt(5)) при q=5 (вложенный радикал); q=2,3,4,6 — класс (а), отложено',
        (
            "sp.Expr",
            "sp.Integer",
            "sp.Rational",
            "sp.cancel",
            "sp.sqrt",
            "sp.sympify",
        ),
    ),
    "reference/alpha_bounds.py": (
        "REPLACED_HOT",
        'оболочка alpha: родная для RadicalSumV1, mpmath остаётся для выражений sympy',
        (
            "mp.iv",
            "mp.iv.prec",
            "mp.libmp.to_rational",
            "sp.Expr",
        ),
    ),
    "reference/angle_measure.py": (
        "B_KEEP",
        'atan2/pi: сертифицированные углы, тригонометрия',
        (
            "mp.iv",
            "mp.iv.atan2",
            "mp.iv.pi",
            "mp.iv.prec",
            "mp.libmp",
            "mp.libmp.to_rational",
            "sp.Expr",
            "sp.atan2",
            "sp.factor",
            "sp.pi",
        ),
    ),
    "reference/angular.py": (
        "B_KEEP",
        'roots, cos/sin/atan2/pi, expand: плотность Хубера и подшаги веера',
        (
            "mp.iv",
            "mp.iv.prec",
            "sp.Expr",
            "sp.Rational",
            "sp.Symbol",
            "sp.atan2",
            "sp.cancel",
            "sp.cos",
            "sp.expand",
            "sp.pi",
            "sp.roots",
            "sp.sin",
            "sp.sqrt",
            "sp.srepr",
            "sp.sympify",
        ),
    ),
    "reference/arrangement.py": (
        "A_LATER",
        'Expr — хранение координат; предикаты уже на SqrtSumV1 (exact_quadratic_value)',
        (
            "sp.Expr",
            "sp.Integer",
            "sp.sympify",
        ),
    ),
    "reference/boundary.py": (
        "REPLACED_HOT",
        '_contact_candidates: родной двойник boundary_native',
        (
            "sp.Expr",
            "sp.Integer",
            "sp.Rational",
        ),
    ),
    "reference/cap.py": (
        "A_LATER",
        'Expr только в аннотациях типов: арифметика идёт через ExactScalar',
        (
            "sp.Expr",
        ),
    ),
    "reference/common.py": (
        "A_LATER",
        'Rational/Integer: клей построения контекста',
        (
            "sp.Expr",
            "sp.Integer",
            "sp.Rational",
        ),
    ),
    "reference/compile.py": (
        "B_KEEP",
        'expand/cancel в точном свидетельстве поворота плотности',
        (
            "sp.cancel",
            "sp.expand",
        ),
    ),
    "reference/direction_binding.py": (
        "B_KEEP",
        'sqrt(5), десятичные оболочки направлений плотности (iv)',
        (
            "mp.iv",
            "mp.iv.prec",
            "mp.libmp",
            "mp.libmp.to_rational",
            "sp.Expr",
            "sp.Rational",
            "sp.sqrt",
        ),
    ),
    "reference/direction_window_exact.py": (
        "A_LATER",
        'Expr только в аннотациях типов: арифметика идёт через ExactScalar',
        (
            "sp.Expr",
        ),
    ),
    "reference/junction.py": (
        "A_LATER",
        'Expr только в аннотациях типов: арифметика идёт через ExactScalar',
        (
            "sp.Expr",
        ),
    ),
    "reference/metric.py": (
        "REPLACED_HOT",
        'dot_g/length_g: родные двойники; floor/сквозная привязка к решётке (snap_exact_point) — отложено',
        (
            "sp.Expr",
            "sp.Integer",
            "sp.Rational",
            "sp.floor",
            "sp.sqrt",
            "sp.srepr",
        ),
    ),
    "reference/native_exact.py": (
        "BRIDGE",
        'мост sympy <-> RadicalSumV1: from_sympy, to_sympy, текст srepr',
        (
            "sp.Add",
            "sp.Basic",
            "sp.Expr",
            "sp.Integer",
            "sp.Rational",
            "sp.cancel",
            "sp.factor",
            "sp.sqrt",
            "sp.srepr",
            "sp.sympify",
        ),
    ),
    "reference/planar_types.py": (
        "REPLACED_HOT",
        'exact_sign, exact_normalize, ExactScalar (srepr); sympy остаётся обменным типом',
        (
            "mp.iv",
            "mp.iv.mpf",
            "mp.iv.prec",
            "mp.iv.sqrt",
            "sp.Add",
            "sp.Expr",
            "sp.Integer",
            "sp.Q",
            "sp.Q.negative",
            "sp.Q.positive",
            "sp.Rational",
            "sp.ask",
            "sp.cancel",
            "sp.factor",
            "sp.sqrt",
            "sp.srepr",
            "sp.sympify",
        ),
    ),
    "reference/radical_rationality.py": (
        "A_DONE",
        'уже без факторизации тем же алгоритмом классов (дубль native_exact; объединить на шаге 3)',
        (
            "sp.Expr",
            "sp.srepr",
        ),
    ),
    "reference/raw_coverage.py": (
        "A_LATER",
        'Rational: разбор запрошенной alpha',
        (
            "sp.Rational",
        ),
    ),
    "reference/strip.py": (
        "A_LATER",
        'Expr только в аннотациях типов: арифметика идёт через ExactScalar',
        (
            "sp.Expr",
        ),
    ),
    "reference/validation.py": (
        "B_KEEP",
        'cos(pi*r): оболочки угла проверки полезной нагрузки',
        (
            "sp.Expr",
            "sp.Rational",
            "sp.cos",
            "sp.pi",
            "sp.sqrt",
        ),
    ),
    "source_grid.py": (
        "B_KEEP",
        'sqrt синуса угла и angular_fraction_of_pi: шаг решётки по углу',
        (
            "sp.Rational",
            "sp.sqrt",
        ),
    ),
    "surface_cone_angle.py": (
        "B_KEEP",
        'iv.cos/pi: угол конуса, интервальная тригонометрия',
        (
            "mp.iv",
            "mp.iv.cos",
            "mp.iv.mpf",
            "mp.iv.pi",
            "mp.iv.prec",
            "mp.libmp",
            "mp.libmp.to_rational",
        ),
    ),
    "wavefront/conveyor.py": (
        "REPLACED_HOT",
        'доказательство рациональности _rational_after_scaling: родной предикат, radsimp/simplify только оракул режима SYMPY и уступка вне поля',
        (
            "sp.Rational",
            "sp.radsimp",
            "sp.simplify",
            "sp.srepr",
            "sp.sympify",
        ),
    ),
}


#: Хост (`cftuv/`) трогает sympy в трёх файлах; проверка версий пула (`envelope_domain_pool.py`,
#: `HOST_PACKAGES`) не арифметика и сюда не входит.
HOST_SYMPY_AUDIT = {
    "envelope_debug_renderer.py": (
        "A_LATER",
        "float(sympify(srepr)) при рисовании отладочных точек: холодный путь рендера, читается родным мостом",
    ),
    "envelope_export_input.py": (
        "HOST_WARMUP",
        "прогрев ленивой подгрузки sympy в родителе (sympy.Symbol('warm') + 1): ~0.35 с на процесс, "
        "пока в ядре остаётся класс (б)",
    ),
    "envelope_request_export.py": (
        "A_LATER",
        "Rational(str(float)) при выгрузке координат и factor в exact_rational: холодный экспорт хоста",
    ),
}


def _sympy_features(path: Path) -> set[str]:
    """Возможности sympy/mpmath, на которые ссылается модуль: `sp.Expr`, `mp.iv.prec`, ..."""

    source = path.read_text(encoding="utf-8-sig")
    tree = ast.parse(source)
    aliases: dict[str, str] = {}
    features: set[str] = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            for alias in node.names:
                root = alias.name.split(".")[0]
                if root in {"sympy", "mpmath"}:
                    aliases[alias.asname or root] = alias.name
        elif isinstance(node, ast.ImportFrom) and node.module:
            root = node.module.split(".")[0]
            if root in {"sympy", "mpmath"}:
                for alias in node.names:
                    aliases[alias.asname or alias.name] = f"{node.module}.{alias.name}"
                    features.add(f"{node.module}.{alias.name}")
    if not aliases:
        return set()
    for node in ast.walk(tree):
        if not isinstance(node, ast.Attribute):
            continue
        chain = []
        current: ast.AST = node
        while isinstance(current, ast.Attribute):
            chain.append(current.attr)
            current = current.value
        if isinstance(current, ast.Name) and current.id in aliases:
            features.add(f"{aliases[current.id]}.{'.'.join(reversed(chain))}")
    return {item.replace("sympy.", "sp.").replace("mpmath.", "mp.") for item in features}


def _importers() -> dict[str, set[str]]:
    found = {}
    for path in sorted(KERNEL.rglob("*.py")):
        features = _sympy_features(path)
        if features:
            found[path.relative_to(KERNEL).as_posix()] = features
    return found


def test_every_module_that_uses_sympy_or_mpmath_is_audited():
    actual = set(_importers())
    declared = set(SYMPY_AUDIT)
    assert not actual - declared, (
        f"новые потребители sympy/mpmath без класса: {sorted(actual - declared)}. "
        "Классифицируйте их в SYMPY_AUDIT (REPLACED_HOT / A_LATER / B_KEEP) с причиной."
    )
    assert not declared - actual, f"строки аудита без модуля: {sorted(declared - actual)}"


def test_no_module_uses_a_sympy_feature_its_audit_row_does_not_name():
    for module, features in _importers().items():
        _cls, _why, allowed = SYMPY_AUDIT[module]
        unnamed = features - set(allowed)
        assert not unnamed, (
            f"{module} использует {sorted(unnamed)}, которых нет в строке аудита: "
            "новая возможность sympy может изменить класс модуля — пересмотрите строку."
        )


def test_every_audit_row_names_a_known_class_and_a_reason():
    for module, (cls, why, _features) in SYMPY_AUDIT.items():
        assert cls in CLASSES, (module, cls)
        assert len(why) > 20, (module, why)


def test_class_b_rows_are_the_only_ones_with_general_algebra():
    """Общая алгебра (корни многочленов, тригонометрия, упрощение) допустима ТОЛЬКО в классе (б)."""

    general = {
        "sp.roots", "sp.sin", "sp.cos", "sp.atan", "sp.atan2", "sp.pi", "sp.simplify", "sp.nsimplify",
        "sp.radsimp", "sp.expand", "sp.Symbol", "sp.ask", "sp.Q.positive", "sp.Q.negative", "sp.Q",
    }
    # `planar_types` держит ask/Q в точном символьном пути знака (уступка sympy для выхода из поля), а
    # `conveyor` — radsimp/simplify в оракуле доказательства рациональности (режим SYMPY, уступка вне поля): оба
    # названы в своих строках.
    named_exceptions = {"reference/planar_types.py", "wavefront/conveyor.py"}
    for module, (cls, _why, features) in SYMPY_AUDIT.items():
        if cls == "B_KEEP" or module in named_exceptions:
            continue
        assert not general & set(features), (module, cls, sorted(general & set(features)))


def test_replaced_rows_exist_in_the_backend_switch():
    from cftuv_envelope.reference import symbolic_backend

    assert {item.value for item in symbolic_backend.SymbolicBackendV1} == {
        "SYMPY",
        "NATIVE_EXACT",
        "SHADOW",
    }
    replaced = {module for module, (cls, *_rest) in SYMPY_AUDIT.items() if cls == "REPLACED_HOT"}
    assert replaced == {
        "reference/alpha_bounds.py",
        "reference/boundary.py",
        "reference/metric.py",
        "reference/planar_types.py",
        "wavefront/conveyor.py",
    }


def _host_sympy_importers() -> set[str]:
    users = set()
    for path in sorted(HOST.rglob("*.py")):
        for node in ast.walk(ast.parse(path.read_text(encoding="utf-8-sig"))):
            if isinstance(node, ast.Import) and any(
                item.name.split(".")[0] in {"sympy", "mpmath"} for item in node.names
            ):
                users.add(path.name)
            elif isinstance(node, ast.ImportFrom) and (node.module or "").split(".")[0] in {"sympy", "mpmath"}:
                users.add(path.name)
            elif (
                isinstance(node, ast.Call)
                and getattr(node.func, "attr", getattr(node.func, "id", "")) == "import_module"
                and node.args
                and isinstance(node.args[0], ast.Constant)
                and node.args[0].value in {"sympy", "mpmath"}
            ):
                users.add(path.name)
    return users


def test_the_scanner_sees_a_new_general_algebra_feature(tmp_path):
    """Отрицательный контроль сканера: без него «ничего не найдено» не отличить от «сканер слеп»."""

    probe = tmp_path / "probe.py"
    probe.write_text(
        "import sympy as sp\nfrom mpmath import iv\n\ndef f(x):\n    return sp.roots(x), iv.prec\n",
        encoding="utf-8",
    )
    assert _sympy_features(probe) == {"sp.roots", "mp.iv", "mp.iv.prec"}


def test_host_sympy_users_are_the_audited_files():
    assert _host_sympy_importers() == set(HOST_SYMPY_AUDIT), sorted(
        _host_sympy_importers() ^ set(HOST_SYMPY_AUDIT)
    )
