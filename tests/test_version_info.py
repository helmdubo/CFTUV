from pathlib import Path

from cftuv.version_info import resolve_addon_build_info


def test_resolves_branch_and_commit_from_regular_checkout(tmp_path: Path):
    git_dir = tmp_path / ".git"
    package_dir = tmp_path / "cftuv"
    (git_dir / "refs" / "heads" / "feature").mkdir(parents=True)
    package_dir.mkdir()
    (git_dir / "HEAD").write_text(
        "ref: refs/heads/feature/runtime\n", encoding="utf-8"
    )
    commit = "0123456789abcdef0123456789abcdef01234567"
    (git_dir / "refs" / "heads" / "feature" / "runtime").write_text(
        commit + "\n", encoding="utf-8"
    )

    info = resolve_addon_build_info(package_dir / "operators.py")

    assert info.branch == "feature/runtime"
    assert info.commit == commit
    assert info.short_commit == "01234567"


def test_resolves_linked_worktree_ref_from_common_git_dir(tmp_path: Path):
    common_dir = tmp_path / "repo" / ".git"
    worktree_git_dir = common_dir / "worktrees" / "preview"
    checkout = tmp_path / "preview"
    package_dir = checkout / "cftuv"
    worktree_git_dir.mkdir(parents=True)
    package_dir.mkdir(parents=True)
    (checkout / ".git").write_text(
        f"gitdir: {worktree_git_dir}\n", encoding="utf-8"
    )
    (worktree_git_dir / "commondir").write_text("../..\n", encoding="utf-8")
    (worktree_git_dir / "HEAD").write_text(
        "ref: refs/heads/preview\n", encoding="utf-8"
    )
    commit = "fedcba9876543210fedcba9876543210fedcba98"
    common_dir.mkdir(parents=True, exist_ok=True)
    (common_dir / "packed-refs").write_text(
        f"{commit} refs/heads/preview\n", encoding="utf-8"
    )

    info = resolve_addon_build_info(package_dir)

    assert info.branch == "preview"
    assert info.commit == commit


def test_reports_unavailable_outside_checkout(tmp_path: Path):
    info = resolve_addon_build_info(tmp_path)

    assert info.branch == "unavailable"
    assert info.commit == "unavailable"
    assert info.short_commit == "unavailable"
