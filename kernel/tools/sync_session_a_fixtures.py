"""Синхронизировать принятый EC0 v5 corpus в standalone test fixtures."""

from __future__ import annotations

import argparse
import hashlib
import json
import shutil
from pathlib import Path


SOURCE_COMMIT = "1fb0394a634c089ed590f544e4d524be28f52123"
SOURCE_SCHEMA = "cftuv.envelope.ec0.corpus.v5"
MANIFEST_NAME = "fixture_hash_manifest.json"


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _source_files(root: Path) -> tuple[Path, ...]:
    return tuple(sorted(path for path in root.rglob("*") if path.is_file()))


def _manifest(root: Path) -> dict[str, object]:
    files = {
        path.relative_to(root).as_posix(): _sha256(path)
        for path in _source_files(root)
        if path.name != MANIFEST_NAME
    }
    return {
        "schema": "cftuv.envelope.ec1.fixture_hash_manifest.v1",
        "source_commit": SOURCE_COMMIT,
        "source_schema": SOURCE_SCHEMA,
        "files": files,
    }


def sync(source: Path, fixture_root: Path) -> None:
    if fixture_root.exists():
        shutil.rmtree(fixture_root)
    fixture_root.mkdir(parents=True)
    for path in _source_files(source):
        destination = fixture_root / path.relative_to(source)
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(path, destination)
    manifest = _manifest(fixture_root)
    (fixture_root / MANIFEST_NAME).write_text(
        json.dumps(manifest, ensure_ascii=False, sort_keys=True, indent=2) + "\n",
        encoding="utf-8",
        newline="\n",
    )


def check(fixture_root: Path, source: Path | None) -> tuple[str, ...]:
    manifest_path = fixture_root / MANIFEST_NAME
    if not manifest_path.is_file():
        return (f"missing {manifest_path}",)
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    errors: list[str] = []
    if manifest.get("source_commit") != SOURCE_COMMIT:
        errors.append("fixture source commit mismatch")
    expected_files = manifest.get("files")
    if not isinstance(expected_files, dict):
        return ("fixture manifest files must be an object",)
    actual = _manifest(fixture_root)["files"]
    if actual != expected_files:
        errors.append("standalone fixture hashes differ from manifest")
    if source is not None:
        source_hashes = _manifest(source)["files"]
        if source_hashes != expected_files:
            errors.append("accepted Session A corpus differs from fixture projection")
    return tuple(errors)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", type=Path)
    parser.add_argument("--fixture-root", type=Path, required=True)
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()
    if args.check:
        errors = check(args.fixture_root, args.source)
        if errors:
            for error in errors:
                print(error)
            return 1
        print("session-a-fixtures: OK")
        return 0
    if args.source is None:
        parser.error("--source is required unless --check verifies standalone fixtures")
    sync(args.source, args.fixture_root)
    print(f"synced {len(_source_files(args.fixture_root)) - 1} files")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

