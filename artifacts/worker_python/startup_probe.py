"""Старт воркеров пула в фоновом Blender: встроенный Python 3.11 против внешнего 3.13.

Меряет то, что решает политику путей внешнего интерпретатора, а не ответ:

- `bundled` — встроенный интерпретатор, полный `sys.path` родителя (как было);
- `external_A` — ВЫБРАННАЯ политика: `-I -S`, воркер получает только каталоги
  ядра, `sympy` и `mpmath` родителя, и они стоят после его собственной stdlib;
- `external_B` — альтернатива «пусть внешний пользуется своим site-packages»:
  обычный запуск (`site` включён, переменные окружения читаются), родитель
  отдаёт воркеру один каталог ядра, `sympy` и `mpmath` воркер берёт свои.

Для каждого варианта: время `ensure_started()` на 8 воркеров (до «готов»,
включая `hello` и импорт ядра), файлы `sympy`/`mpmath`, из которых воркер их
загрузил (сверка версий не отличает «те же файлы» от «такой же версии»), и цена
отпечатка ядра в родителе.

    blender -b --python-exit-code 1 --python startup_probe.py -- \\
        --root <дерево> --out <json> --external C:\\Python313\\python.exe
"""

from __future__ import annotations

import argparse
import json
import statistics
import sys
import time
from pathlib import Path

import bpy  # noqa: F401 - скрипт исполняется только внутри Blender


def _arguments() -> argparse.Namespace:
    tail = sys.argv[sys.argv.index("--") + 1 :] if "--" in sys.argv else []
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", required=True)
    parser.add_argument("--out", required=True)
    parser.add_argument("--external", required=True)
    parser.add_argument("--workers", type=int, default=8)
    parser.add_argument("--repeats", type=int, default=5)
    return parser.parse_args(tail)


def _load_tree(root: Path):
    installed = sys.modules.get("cftuv")
    if installed is not None:
        try:
            installed.unregister()
        except Exception as exc:  # noqa: BLE001 - диагностика окружения
            print("installed unregister failed:", type(exc).__name__, exc)
    for name in tuple(sys.modules):
        if name in {"cftuv", "cftuv_envelope"} or name.startswith(
            ("cftuv.", "cftuv_envelope.")
        ):
            del sys.modules[name]
    for path in (root / "kernel" / "src", root):
        text = str(path)
        if text in sys.path:
            sys.path.remove(text)
        sys.path.insert(0, text)
    import cftuv

    assert Path(cftuv.__file__).resolve().parent == (root / "cftuv").resolve()
    return cftuv


def _variant_b(pool_module):
    """Вариант B: без `-I -S`, и родитель отдаёт воркеру только каталог ядра."""

    original = pool_module.subprocess.Popen

    def popen(command, *args, **kwargs):
        command = [item for item in command if item not in ("-I", "-S")]
        spec = json.loads(kwargs["env"][pool_module.SPEC_ENVIRONMENT_VARIABLE])
        kernel = pool_module.package_directory("cftuv_envelope")
        spec["sys_path"] = [str(Path(kernel).parent)]
        kwargs["env"] = {
            **kwargs["env"],
            pool_module.SPEC_ENVIRONMENT_VARIABLE: json.dumps(spec),
        }
        return original(command, *args, **kwargs)

    return original, popen


def _measure(pool_module, name, external, workers, repeats, patch=None):
    seconds = []
    loaded = {}
    for _ in range(repeats):
        pool = pool_module.DomainPool(workers, external_python=external)
        original = None
        if patch is not None:
            original, replacement = patch(pool_module)
            pool_module.subprocess.Popen = replacement
        try:
            started = time.perf_counter()
            pool.ensure_started()
            seconds.append(round(time.perf_counter() - started, 3))
            interpreter = pool.interpreter
            loaded = {
                "interpreter": f"{interpreter.version_text} external={interpreter.external}",
                "rejected": interpreter.outcome,
                "workers": pool.worker_count,
                "worker_sympy_dirs": sorted(
                    {worker.identity.get("sympy_path", "") for worker in pool._workers}
                ),
            }
        finally:
            if original is not None:
                pool_module.subprocess.Popen = original
            pool.close()
    return {
        "variant": name,
        "ensure_started_seconds": seconds,
        "median": round(statistics.median(seconds), 3),
        **loaded,
    }


def main() -> None:
    args = _arguments()
    root = Path(args.root).resolve()
    _load_tree(root)
    from cftuv import envelope_domain_pool as pool_module

    # Холодный старт каждого интерпретатора (кэш байт-кода и диска) не считается.
    for external in ("", args.external):
        warm = pool_module.DomainPool(2, external_python=external)
        warm.ensure_started()
        warm.close()
    rows = [
        _measure(pool_module, "bundled", "", args.workers, args.repeats),
        _measure(pool_module, "external_A", args.external, args.workers, args.repeats),
        _measure(
            pool_module,
            "external_B",
            args.external,
            args.workers,
            args.repeats,
            patch=_variant_b,
        ),
    ]
    host_environment = None
    costs = []
    for _ in range(args.repeats):
        started = time.perf_counter()
        host_environment = pool_module.describe_environment()
        costs.append(time.perf_counter() - started)
    fingerprint_costs = []
    kernel = pool_module.package_directory("cftuv_envelope")
    for _ in range(args.repeats):
        started = time.perf_counter()
        pool_module.kernel_fingerprint(kernel)
        fingerprint_costs.append(time.perf_counter() - started)
    kernel_files = sum(1 for item in Path(kernel).rglob("*.py"))
    result = {
        "root": str(root),
        "external": args.external,
        "parent_python": ".".join(map(str, sys.version_info[:3])),
        "rows": rows,
        "host_sympy_dir": pool_module.package_directory("sympy"),
        "host_kernel_dir": kernel,
        "kernel_py_files": kernel_files,
        "kernel_fingerprint_seconds_median": round(statistics.median(fingerprint_costs), 4),
        "describe_environment_seconds_median": round(statistics.median(costs), 4),
        "host_environment": host_environment,
    }
    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    Path(args.out).write_text(
        json.dumps(result, ensure_ascii=False, indent=1, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    for row in rows:
        print("START", row["variant"], row["median"], row["ensure_started_seconds"])
    print("STARTUP_DONE")


main()
