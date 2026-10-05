"""Сборка нативного ускорителя одной командой: колесо maturin -> установка в dev-venv.

    python tools/native_build.py            # release-колесо и `pip install --force-reinstall --no-deps`
    python tools/native_build.py --test     # плюс `cargo test` ядра (профиль test уже оптимизирован)

Запускается ЛЮБЫМ Python: собирает и ставит интерпретатором venv (`CFTUV_NATIVE_VENV`, по умолчанию
`~/.cftuv-native/venv`, вне репозитория и вне AppData: пути MSIX-Python туда не перенаправляются). `maturin develop`
не используется намеренно: он копирует `.pyd` в дерево исходников. Колёса лежат в `native/target/wheels`
(каталог в `.gitignore`).
"""

from __future__ import annotations

import argparse
import os
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
NATIVE = ROOT / "native"
WHEELS = NATIVE / "target" / "wheels"


def venv_python() -> Path:
    base = Path(os.environ.get("CFTUV_NATIVE_VENV", Path.home() / ".cftuv-native" / "venv"))
    candidate = base / ("Scripts/python.exe" if os.name == "nt" else "bin/python")
    if not candidate.exists():
        raise SystemExit(f"нет интерпретатора dev-venv: {candidate} (задайте CFTUV_NATIVE_VENV)")
    return candidate


def cargo_environment() -> dict:
    environment = dict(os.environ)
    cargo_bin = Path.home() / ".cargo" / "bin"
    environment["PATH"] = f"{cargo_bin}{os.pathsep}{environment.get('PATH', '')}"
    return environment


def run(command: list, *, cwd: Path, environment: dict) -> None:
    print("+", " ".join(str(part) for part in command), flush=True)
    subprocess.run([str(part) for part in command], cwd=cwd, env=environment, check=True)


def newest_wheel() -> Path:
    wheels = sorted(WHEELS.glob("cftuv_native-*.whl"), key=lambda path: path.stat().st_mtime)
    if not wheels:
        raise SystemExit(f"maturin не оставил колеса в {WHEELS}")
    return wheels[-1]


def main(argv: list | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--test", action="store_true", help="сначала `cargo test -p cftuv-core`")
    arguments = parser.parse_args(argv)
    python = venv_python()
    environment = cargo_environment()
    if arguments.test:
        run(["cargo", "test", "-p", "cftuv-core"], cwd=NATIVE, environment=environment)
    started = {path: path.stat().st_mtime for path in WHEELS.glob("cftuv_native-*.whl")}
    run([python, "-m", "maturin", "build", "--release", "-o", WHEELS], cwd=NATIVE / "cftuv-python", environment=environment)
    wheel = newest_wheel()
    if started.get(wheel) == wheel.stat().st_mtime:
        raise SystemExit(f"колесо {wheel.name} не обновилось: сборка ничего не произвела")
    run([python, "-m", "pip", "install", "-q", "--force-reinstall", "--no-deps", wheel], cwd=ROOT, environment=environment)
    print(f"установлено: {wheel.name}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
