"""Догон нативных портов за ядром на Python: пин, сборка, корпус, сверка, замер — отдельными шагами и одной командой.

Нативная операция побитово равна ОДНОЙ версии ядра (`cftuv_native.pin`). Когда ядро ушло (`native_status()` называет файлы эталона, которые
изменились), догон — это слияние ветки ядра, перенос дельты в Rust (то, что делает человек и чего здесь нет) и ВОСПРОИЗВОДИМАЯ часть, которую
делает этот скрипт. Порядок после слияния и переноса дельты:

    git merge <ветка ядра или тег>                               # руками; конфликты — руками
    python tools/native_catchup.py status                        # какие файлы эталона ушли от пина (до переноса дельты)
    python tools/native_catchup.py pin                           # пересобрать PINS в pin.py из дерева ядра, назвать изменённые файлы
    python tools/native_catchup.py build                         # cargo test --workspace, колесо, установка в venv (3.13) и py311-site (3.11)
    python tools/native_catchup.py corpus                        # выгрузка из Blender, производные записи (питон Blender 3.11), синтетический корпус, корпус скелета (поле, тесты, генератор, производные, швы)
    python tools/native_catchup.py test                          # все tests/test_native_*.py под 3.13 и под 3.11, нули расхождений
    python tools/native_catchup.py bench --out <каталог>         # замер целых операций под 3.11 и 3.13
    python tools/native_catchup.py all --out <каталог>           # pin, build, corpus, test, bench подряд

Корпус кладётся в `<CFTUV_NATIVE_CORPUS или E:/cftuv_native_corpus>/<8 знаков HEAD>/` (старые каталоги не трогаются; тесты и замер берут ТОТ, чей индекс записан под отпечаток
кода ядра процесса — `native_corpus.matching_corpus`). Blender запускается ТОЛЬКО без интерфейса (`-b`), сцена не сохраняется.
Пути: `CFTUV_BLENDER` (blender.exe), `CFTUV_BLENDER_PYTHON` (питон Blender, 3.11), `CFTUV_NATIVE_VENV` (dev-venv, 3.13), `CFTUV_NATIVE_SCENE` (сцена).
"""

from __future__ import annotations

import argparse
import importlib.util
import os
import re
import shutil
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
KERNEL_PACKAGE = ROOT / "kernel" / "src" / "cftuv_envelope"
PIN_FILE = ROOT / "native" / "cftuv-python" / "python" / "cftuv_native" / "pin.py"
BLENDER = Path(os.environ.get("CFTUV_BLENDER", "C:/Program Files/Blender Foundation/Blender 4.5/blender.exe"))
BLENDER_PYTHON = Path(os.environ.get("CFTUV_BLENDER_PYTHON", "C:/Program Files/Blender Foundation/Blender 4.5/4.5/python/bin/python.exe"))
SCENE = os.environ.get("CFTUV_NATIVE_SCENE", "E:/testScene.blend")
PY311_SITE = Path.home() / ".cftuv-native" / "py311-site"
PINS_BLOCK = re.compile(r"PINS: dict = \{\n.*?\n\}\n", re.S)


def load_pin(path: Path = PIN_FILE):
    """Модуль `pin.py` по пути файла (он не импортирует расширение и не знает пакета)."""

    spec = importlib.util.spec_from_file_location("cftuv_native_pin_file", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def pinned_files(pin) -> list:
    """Все файлы эталона, которые зеркалят порты (у операций списки пересекаются: у файла один дайджест)."""

    return sorted({name for names in pin.OPERATION_FILES.values() for name in names})


def changed_files(pin, root: Path = KERNEL_PACKAGE) -> dict:
    """`{операция: [файлы эталона в дереве `root`, чей дайджест не равен пину]}` (отсутствующий файл назван `<файл> (missing)`)."""

    return {operation: list(pin.stale_files(operation, root)) for operation in pin.OPERATION_FILES}


def rewrite_pins(pin_file: Path = PIN_FILE, root: Path = KERNEL_PACKAGE) -> list:
    """Переписывает блок `PINS` файла `pin_file` дайджестами дерева `root`; возвращает файлы, чей дайджест изменился. Нет файла в дереве — отказ, пин не тронут."""

    pin = load_pin(pin_file)
    names = pinned_files(pin)
    found = {name: pin.digest(root, name) for name in names}
    gone = [name for name, value in found.items() if value is None]
    if gone:
        raise SystemExit(f"NATIVE_CATCHUP_FAILED the oracle tree {root} lacks pinned files {gone}: the port mirrors files that moved or were deleted (edit OPERATION_FILES by hand)")
    block = "PINS: dict = {\n" + "".join(f'    "{name}": "{found[name]}",\n' for name in names) + "}\n"
    text = pin_file.read_text(encoding="utf-8")
    if len(PINS_BLOCK.findall(text)) != 1:
        raise SystemExit(f"NATIVE_CATCHUP_FAILED {pin_file} has no single `PINS: dict = {{...}}` block to rewrite")
    pin_file.write_text(PINS_BLOCK.sub(lambda _match: block, text, count=1), encoding="utf-8")
    return [name for name in names if pin.PINS.get(name) != found[name]]


def run(command: list, *, environment: dict | None = None, cwd: Path = ROOT, output: Path | None = None) -> None:
    """Команда с печатью; ненулевой код — отказ шага (`check`), вывод по желанию в файл."""

    print("+", " ".join(str(part) for part in command), flush=True)
    merged = {**os.environ, "PYTHONSAFEPATH": "1", **(environment or {})}
    if output is None:
        subprocess.run([str(part) for part in command], cwd=cwd, env=merged, check=True)
        return
    with open(output, "w", encoding="utf-8") as handle:
        done = subprocess.run([str(part) for part in command], cwd=cwd, env=merged, stdout=handle, stderr=subprocess.STDOUT, check=False)
    print(f"  -> {output} (exit {done.returncode})", flush=True)
    if done.returncode:
        raise SystemExit(done.returncode)


def venv_python() -> Path:
    base = Path(os.environ.get("CFTUV_NATIVE_VENV", Path.home() / ".cftuv-native" / "venv"))
    return base / ("Scripts/python.exe" if os.name == "nt" else "bin/python")


def interpreters() -> list:
    """`(метка, интерпретатор, окружение)`: dev-venv (3.13) и питон Blender (3.11) с колесом из `py311-site`."""

    return [("py313", venv_python(), {}), ("py311", BLENDER_PYTHON, {"PYTHONPATH": str(PY311_SITE)})]


def step_status() -> None:
    pin = load_pin()
    for operation, files in changed_files(pin).items():
        print(f"{operation}: {'matches the pin' if not files else 'moved: ' + ', '.join(files)}")


def step_pin() -> None:
    changed = rewrite_pins()
    print(f"PINS rewritten from {KERNEL_PACKAGE}: {len(changed)} file(s) changed" + (": " + ", ".join(changed) if changed else ""))


def step_build() -> None:
    run([venv_python(), ROOT / "tools" / "native_build.py", "--test"])
    wheel = sorted((ROOT / "native" / "target" / "wheels").glob("cftuv_native-*.whl"), key=lambda path: path.stat().st_mtime)[-1]
    run([venv_python(), "-m", "pip", "install", "-q", "--upgrade", "--force-reinstall", "--no-deps", "--target", PY311_SITE, wheel])


def step_corpus() -> None:
    head = subprocess.run(["git", "rev-parse", "HEAD"], cwd=ROOT, capture_output=True, text=True, check=True).stdout.strip()
    out = Path(os.environ.get("CFTUV_NATIVE_CORPUS", "E:/cftuv_native_corpus")) / head[:8]
    run([BLENDER, "-b", SCENE, "--python-exit-code", "1", "--python", ROOT / "tools" / "native_corpus_export.py", "--", "--out", out, "--overwrite"])
    run([BLENDER_PYTHON, ROOT / "tools" / "native_corpus_derive.py", "--corpus", out])
    # the synthetic corpus goes beside the field corpus of THIS kernel (`native_corpus.matching_corpus`); the records of an earlier build are dropped first
    shutil.rmtree(out / "synthetic_clip", ignore_errors=True)
    run([venv_python(), ROOT / "tools" / "native_clip_synthetic.py", "build"])
    step_skeleton_corpus(out)


def step_skeleton_corpus(out: Path) -> None:
    """Корпус скелета рядом с корпусом покрытия и резки ЭТОГО ядра: поле (Blender, питон 3.11) и его производные, тесты ядра, сгенерированные полигоны, производные синтетики, швы обоих."""

    field, synthetic = out / "skeleton", out / "synthetic_skeleton"
    tools = ROOT / "tools"
    run([BLENDER, "-b", SCENE, "--python-exit-code", "1", "--python", tools / "native_skeleton_export.py", "--", "--out", field, "--overwrite"])
    run([BLENDER_PYTHON, tools / "native_skeleton_derive.py", "--corpus", field])
    shutil.rmtree(synthetic, ignore_errors=True)
    run([venv_python(), tools / "native_skeleton_synthetic.py", "build", "--out", synthetic])
    run([venv_python(), tools / "native_skeleton_generated.py", "generate", "--corpus", synthetic])
    run([venv_python(), tools / "native_skeleton_derive.py", "--corpus", synthetic])
    for corpus in (field, synthetic):
        run([venv_python(), tools / "native_skeleton_seams.py", "record", "--corpus", corpus, "--full", "all", "--full-ids", "all"])


def step_test() -> None:
    files = sorted(str(path) for path in (ROOT / "tests").glob("test_native_*.py"))
    for label, interpreter, environment in interpreters():
        run([interpreter, "-m", "pytest", "-q", "-p", "no:cacheprovider", "-rs", *files], environment=environment)
        print(f"NATIVE_CATCHUP_TEST_OK {label}", flush=True)


def step_bench(out: Path) -> None:
    out.mkdir(parents=True, exist_ok=True)
    for label, interpreter, environment in interpreters():
        run([interpreter, ROOT / "tools" / "native_bench_native.py", "--op", "both", "--repeat", "3", "--out", out / f"bench_{label}.json"], environment=environment, output=out / f"bench_{label}.txt")


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("step", choices=("status", "pin", "build", "corpus", "test", "bench", "all"))
    parser.add_argument("--out", type=Path, default=None, help="каталог для замера (шаги bench и all)")
    arguments = parser.parse_args(argv)
    steps = {"status": step_status, "pin": step_pin, "build": step_build, "corpus": step_corpus, "test": step_test}
    if arguments.step in ("bench", "all") and arguments.out is None:
        parser.error("--out is required for bench and all")
    for name in ("pin", "build", "corpus", "test") if arguments.step == "all" else (arguments.step,):
        if name in steps:
            steps[name]()
    if arguments.step in ("bench", "all"):
        step_bench(arguments.out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
