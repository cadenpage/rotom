#!/usr/bin/env python3
import argparse
import json
from pathlib import Path


def _resolve_episode_meta(path_arg: str) -> Path:
    path = Path(path_arg).expanduser()
    if path.is_dir():
        return path / "meta.json"
    return path


def _parse_success(value: str) -> bool:
    normalized = value.strip().lower()
    if normalized in {"true", "1", "yes", "y", "success"}:
        return True
    if normalized in {"false", "0", "no", "n", "failure"}:
        return False
    raise argparse.ArgumentTypeError(f"unsupported success value: {value}")


def main() -> None:
    parser = argparse.ArgumentParser(description="Annotate a recorded Rotom episode.")
    parser.add_argument("episode", help="Path to an episode directory or its meta.json.")
    parser.add_argument("--success", type=_parse_success, default=None, help="Set episode success label.")
    parser.add_argument("--task-id", default=None, help="Task identifier string.")
    parser.add_argument("--failure-code", default=None, help="Failure mode label.")
    parser.add_argument("--notes", default=None, help="Free-form notes.")
    args = parser.parse_args()

    meta_path = _resolve_episode_meta(args.episode)
    if not meta_path.exists():
        raise SystemExit(f"meta file not found: {meta_path}")

    meta = json.loads(meta_path.read_text(encoding="utf-8"))
    if args.success is not None:
        meta["success"] = bool(args.success)
        meta["label"] = "success" if args.success else "failure"
    if args.task_id is not None:
        meta["task_id"] = args.task_id
    if args.failure_code is not None:
        meta["failure_code"] = args.failure_code
    if args.notes is not None:
        meta["notes"] = args.notes

    meta_path.write_text(json.dumps(meta, indent=2) + "\n", encoding="utf-8")
    print(f"updated {meta_path}")


if __name__ == "__main__":
    main()
