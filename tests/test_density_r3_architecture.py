"""Числовая граница R3: V2-лист, целые решающие пути, память одной транзакции."""

from pathlib import Path
import re

ROOT = Path(__file__).resolve().parents[1]
CRATE = ROOT / "native" / "cftuv-density"


def test_density_depends_on_radical_v2_without_the_v1_runtime_stack():
    manifest = (CRATE / "Cargo.toml").read_text(encoding="utf-8")
    dependencies = re.search(r"^\[dependencies\]\s*$(.*?)(?=^\[|\Z)", manifest, re.MULTILINE | re.DOTALL).group(1)
    assert {line.strip() for line in dependencies.splitlines() if line.strip()} == {'cftuv-radical = { path = "../cftuv-radical" }'}
    workspace = (ROOT / "native" / "Cargo.toml").read_text(encoding="utf-8")
    assert '"cftuv-density"' in workspace


def test_density_decisions_have_no_float_program_bridge_or_process_cache():
    sources = list((CRATE / "src").rglob("*.rs"))
    assert sources
    for path in sources:
        source = re.sub(r"//[^\n]*|/\*.*?\*/", "", path.read_text(encoding="utf-8"), flags=re.DOTALL)
        assert not re.search(r"\bf(?:32|64)\b|\blibm\b|\bmpmath\b", source), path
        assert not re.search(r"cftuv_radical\s*::\s*program\b", source), path
        assert not re.search(r"(?m)^\s*(?:pub\s+)?static\s|\b(?:LazyLock|OnceLock)\b|thread_local!", source), path


def test_density_transaction_owns_its_exact_memory():
    source = (CRATE / "src" / "transaction.rs").read_text(encoding="utf-8")
    assert "pub struct DensityTransaction" in source
    assert "HashMap" in source
    assert "signum(None)" in source
