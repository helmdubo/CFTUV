#!/usr/bin/env python3
"""Build a minimal bootstrap or offline packet for one CFTUV agent card."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
from typing import Any


def _load_json(path: Path) -> dict[str, Any]:
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except FileNotFoundError as exc:
        raise SystemExit(f"missing required file: {path}") from exc
    except json.JSONDecodeError as exc:
        raise SystemExit(f"invalid JSON in {path}: {exc}") from exc


def _find_task(manifest: dict[str, Any], card_id: str) -> dict[str, Any]:
    for task in manifest.get("tasks", []):
        if task.get("id") == card_id:
            status = str(task.get("status", ""))
            if status.startswith("SUPERSEDED_"):
                raise SystemExit(
                    f"card {card_id!r} is {status} and must not be executed; "
                    "read its satisfied_by receipt and select an active card"
                )
            return task
    known = ", ".join(task.get("id", "?") for task in manifest.get("tasks", []))
    raise SystemExit(f"unknown card {card_id!r}; known cards: {known}")


def _accepted_sha(repo_root: Path, manifest: dict[str, Any], card_id: str) -> str:
    if card_id == "BASE-00":
        return str(manifest["reviewed_baseline_sha"])

    status_path = repo_root / "docs" / "architecture_status.json"
    status = _load_json(status_path)
    for key in (
        "accepted_integration_sha",
        "integration_sha",
        "accepted_sha",
    ):
        value = status.get(key)
        if isinstance(value, str) and value:
            return value
    raise SystemExit(
        f"{status_path} does not contain accepted_integration_sha; "
        "do not start a post-BASE card without an immutable accepted base"
    )


def _read(path: Path) -> str:
    try:
        return path.read_text(encoding="utf-8").rstrip()
    except FileNotFoundError as exc:
        raise SystemExit(f"missing packet input: {path}") from exc


def _repo_prompt(
    *,
    repo_root: Path,
    pack_root: Path,
    manifest: dict[str, Any],
    task: dict[str, Any],
    base_sha: str,
    handoffs: list[Path],
    branch: str | None,
    control_ref: str | None,
) -> str:
    card_path = pack_root / task["file"]
    rel_card = card_path.relative_to(repo_root) if card_path.is_relative_to(repo_root) else card_path
    handoff_lines = "\n".join(f"- `{path}`" for path in handoffs) or "- none (BASE-00 only, or supply accepted dependency handoffs)"
    base_note = (
        f"Reviewed baseline SHA: `{base_sha}`\n"
        "`docs/architecture_status.json` does not exist yet; creating it is part of BASE-00."
        if task["id"] == "BASE-00"
        else f"Accepted integration SHA: `{base_sha}`"
    )
    control_note = (
        f"\nCanonical control status ref: `{control_ref}`\n"
        "The accepted SHA above was resolved from that ref. After creating a "
        "card branch from the immutable content SHA, do not replace it with an "
        "older self-referential status copy embedded in that content commit."
        if control_ref
        else ""
    )
    status_step = (
        f"current control status from "
        f"`git show {control_ref}:docs/architecture_status.json`"
        if control_ref
        else "`docs/architecture_status.json` (skip only for BASE-00)"
    )
    branch_line = f"Working branch/worktree: `{branch}`\n" if branch else ""
    deps = ", ".join(task.get("dependencies", [])) or "none"
    return f"""Work in repository `helmdubo/CFTUV`.

Active task: `{task['id']}` — {task['title']}
Card path: `{rel_card}`
Dependencies declared by the card: `{deps}`
{base_note}{control_note}
{branch_line}Accepted dependency handoffs:
{handoff_lines}

Read in this order:
1. `AGENTS.md`
2. {status_step}
3. `{pack_root.relative_to(repo_root) if pack_root.is_relative_to(repo_root) else pack_root}/01_GLOBAL_CANON.md`
4. `{pack_root.relative_to(repo_root) if pack_root.is_relative_to(repo_root) else pack_root}/02_AGENT_PROTOCOL.md`
5. the exact active card
6. accepted dependency handoffs
7. only code/tests in the card allowlist

Implement only this card. Do not read legacy decal geometry unless the card explicitly assigns that role. Do not start downstream cards. If a new product semantic decision is required, stop with a named issue instead of inventing behavior. Leave the mandatory repository handoff and exact test evidence.
"""


def _offline_packet(
    *,
    repo_root: Path,
    pack_root: Path,
    manifest: dict[str, Any],
    task: dict[str, Any],
    base_sha: str,
    handoffs: list[Path],
    branch: str | None,
    control_ref: str | None,
) -> str:
    sections: list[tuple[str, str]] = []
    sections.append(
        (
            "SESSION METADATA",
            json.dumps(
                {
                    "repository": manifest.get("repository"),
                    "card_id": task["id"],
                    "title": task["title"],
                    "dependencies": task.get("dependencies", []),
                    "base_sha": base_sha,
                    "control_status_ref": control_ref,
                    "branch_or_worktree": branch,
                    "handoffs": [str(path) for path in handoffs],
                    "warning": "This packet authorizes exactly one card.",
                },
                ensure_ascii=False,
                indent=2,
            ),
        )
    )

    agents = repo_root / "AGENTS.md"
    if agents.exists():
        sections.append(("REPOSITORY AGENTS.md", _read(agents)))

    status = repo_root / "docs" / "architecture_status.json"
    if task["id"] != "BASE-00":
        sections.append(("ARCHITECTURE STATUS", _read(status)))

    sections.extend(
        [
            ("GLOBAL CANON", _read(pack_root / "01_GLOBAL_CANON.md")),
            ("AGENT PROTOCOL", _read(pack_root / "02_AGENT_PROTOCOL.md")),
            ("DECISION LOG", _read(pack_root / "03_DECISION_LOG.md")),
            ("ACTIVE TASK CARD", _read(pack_root / task["file"])),
        ]
    )

    for handoff in handoffs:
        path = handoff if handoff.is_absolute() else repo_root / handoff
        sections.append((f"ACCEPTED HANDOFF: {handoff}", _read(path)))

    sections.append(("MANDATORY HANDOFF TEMPLATE", _read(pack_root / "templates" / "HANDOFF_TEMPLATE.md")))

    header = "# CFTUV MINIMAL OFFLINE AGENT PACKET\n\nThis packet authorizes exactly one task card. It is not permission to start downstream work.\n"
    body = "\n\n".join(f"# {title}\n\n{content}" for title, content in sections)
    return header + "\n\n" + body + "\n"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo-root", type=Path, default=Path.cwd())
    parser.add_argument("--pack-root", type=Path)
    parser.add_argument("--card", required=True)
    parser.add_argument("--mode", choices=("repo", "offline"), default="repo")
    parser.add_argument("--handoff", action="append", default=[], type=Path)
    parser.add_argument("--branch")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()

    repo_root = args.repo_root.resolve()
    pack_root = (
        args.pack_root.resolve()
        if args.pack_root
        else repo_root / "docs" / "agent_execution" / "envelope_v1"
    )
    manifest = _load_json(pack_root / "task_manifest.json")
    task = _find_task(manifest, args.card)
    base_sha = _accepted_sha(repo_root, manifest, args.card)
    control_ref: str | None = None
    if task["id"] != "BASE-00":
        status = _load_json(repo_root / "docs" / "architecture_status.json")
        value = status.get("accepted_integration_ref")
        if isinstance(value, str) and value:
            control_ref = value

    if task.get("dependencies") and not args.handoff:
        print(
            f"warning: {args.card} declares dependencies {task['dependencies']} but no --handoff was supplied",
            file=sys.stderr,
        )

    if args.mode == "repo":
        text = _repo_prompt(
            repo_root=repo_root,
            pack_root=pack_root,
            manifest=manifest,
            task=task,
            base_sha=base_sha,
            handoffs=args.handoff,
            branch=args.branch,
            control_ref=control_ref,
        )
    else:
        text = _offline_packet(
            repo_root=repo_root,
            pack_root=pack_root,
            manifest=manifest,
            task=task,
            base_sha=base_sha,
            handoffs=args.handoff,
            branch=args.branch,
            control_ref=control_ref,
        )

    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(text, encoding="utf-8")
        print(args.output)
    else:
        print(text, end="")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
