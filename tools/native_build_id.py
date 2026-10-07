"""Личность сборки нативного ускорителя: что установлено и та ли это сборка, что соответствует дереву.

    python tools/native_build_id.py            # id и половины установленного расширения, id дерева; код возврата 1, если Rust-половины различаются
    python tools/native_build_id.py --json
    python tools/native_build_id.py --require  # код возврата 1, если расширения нет или ЛЮБАЯ из трёх величин (rust, shim, id) установленного не равна дереву (так CI доказывает, что стоит колесо ЭТОГО дерева)

`cftuv_native.native_build_id()` — sha256 содержимого (не байтов `_core.pyd`: MSVC пишет в каждую сборку метку времени и GUID PDB, и две сборки одного кода
различаются побайтно): половина `rust` — отпечаток исходников Rust, вшитый при сборке (`native/cftuv-python/build.rs`, правило в `digest.rs`), половина `shim` — отпечаток
`.py` самого шима (`cftuv_native/buildid.py`). Здесь — та же `rust` половина, посчитанная по дереву без сборки: `rust_tree_digest(native_root)`.

Правило (то же, что в `digest.rs`) — по рабочему каталогу `native/`: `Cargo.toml` и `Cargo.lock` корня; в каждом каталоге крейта (прямой подкаталог с `Cargo.toml`, кроме
`target` и скрытых) файлы прямо в нём с именем `Cargo.toml`, `pyproject.toml` или `*.rs` (`build.rs`, `digest.rs`) и каждый `*.rs` под его `src/`; sha256 по файлам в порядке
относительного пути (байты UTF-8, разделитель `/`), каждый как `путь 0x00 длина 0x00 содержимое 0x0A`, в содержимом CRLF превращён в LF.
"""

from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
NATIVE_ROOT = ROOT / "native"


def rust_source_files(native_root: Path) -> list:
    """`[(относительный путь, путь)]` хешируемых файлов дерева в порядке байтов относительного пути."""

    native_root = Path(native_root)
    found: list = []
    for name in ("Cargo.toml", "Cargo.lock"):
        if not (native_root / name).is_file():
            raise FileNotFoundError(f"{native_root} is not a native workspace root: no {name}")
        found.append((name, native_root / name))
    for crate in sorted(native_root.iterdir(), key=lambda path: path.name.encode("utf-8")):
        if crate.name == "target" or crate.name.startswith(".") or not crate.is_dir() or not (crate / "Cargo.toml").is_file():
            continue
        for item in crate.iterdir():
            if item.is_file() and (item.name in ("Cargo.toml", "pyproject.toml") or item.name.endswith(".rs")):
                found.append((f"{crate.name}/{item.name}", item))
        source = crate / "src"
        if source.is_dir():
            for path in source.rglob("*.rs"):
                if path.is_file():
                    found.append((f"{crate.name}/src/" + path.relative_to(source).as_posix(), path))
    return sorted(found, key=lambda item: item[0].encode("utf-8"))


def rust_tree_digest(native_root: Path = NATIVE_ROOT) -> str:
    """Отпечаток исходников Rust дерева `native_root` (то же число, что вшивает `build.rs`, если расширение собрано из этого дерева)."""

    hasher = hashlib.sha256()
    for relative, path in rust_source_files(native_root):
        content = path.read_bytes().replace(b"\r\n", b"\n")
        hasher.update(relative.encode("utf-8") + b"\0" + str(len(content)).encode("ascii") + b"\0" + content + b"\n")
    return hasher.hexdigest()


def tree_parts(native_root: Path = NATIVE_ROOT) -> dict:
    """Половины и id дерева: `rust` по исходникам, `shim` по `.py` шима в дереве."""

    shim_directory = Path(native_root) / "cftuv-python" / "python" / "cftuv_native"
    spec = importlib.util.spec_from_file_location("cftuv_native_buildid_file", shim_directory / "buildid.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    rust = rust_tree_digest(native_root)
    shim = module.shim_digest(shim_directory)
    return {"id": module.compose(rust, shim), "rust": rust, "shim": shim}


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--json", action="store_true", help="напечатать JSON")
    parser.add_argument("--require", action="store_true", help="код возврата 1, если расширения нет или rust, shim либо id установленного не равны дереву")
    arguments = parser.parse_args(argv)
    tree = tree_parts()
    report = {"tree": tree, "installed": None, "match": None}
    try:
        import cftuv_native
    except ModuleNotFoundError as error:
        if error.name != "cftuv_native":
            raise
    else:
        installed = cftuv_native.native_build_parts()
        report["installed"] = installed
        report["match"] = {"rust": installed["rust"] == tree["rust"], "shim": installed["shim"] == tree["shim"], "id": installed["id"] == tree["id"]}
    if arguments.json:
        print(json.dumps(report, indent=2))
    else:
        print(f"tree      id {tree['id']}  rust {tree['rust'][:16]}  shim {tree['shim'][:16]}")
        if report["installed"] is None:
            print("installed: нет расширения cftuv_native (`python tools/native_build.py`)")
        else:
            installed = report["installed"]
            print(f"installed id {installed['id']}  rust {installed['rust'][:16]}  shim {installed['shim'][:16]}  (version {installed['version']})")
            print("match: " + ", ".join(f"{name} {'yes' if value else 'NO'}" for name, value in report["match"].items()))
    if arguments.require:
        return 0 if report["match"] is not None and all(report["match"].values()) else 1
    return 0 if report["match"] is None or report["match"]["rust"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
