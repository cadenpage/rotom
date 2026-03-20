#!/usr/bin/env python3
import argparse
import inspect
import json
import shutil
from pathlib import Path
from typing import Any

import numpy as np


def _load_json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def _load_steps(path: Path) -> list[dict[str, Any]]:
    steps: list[dict[str, Any]] = []
    with path.open("r", encoding="utf-8") as handle:
        for line in handle:
            stripped = line.strip()
            if stripped:
                steps.append(json.loads(stripped))
    return steps


def _episode_dirs(raw_root: Path) -> list[Path]:
    if not raw_root.exists():
        raise FileNotFoundError(f"raw episode root does not exist: {raw_root}")
    return sorted(path for path in raw_root.iterdir() if path.is_dir() and (path / "meta.json").exists())


def _filter_episode(meta: dict[str, Any], success_only: bool) -> bool:
    if not success_only:
        return True
    return meta.get("success") is True


def _joint_positions(step: dict[str, Any]) -> np.ndarray:
    joint_state = (step.get("observation") or {}).get("joint_state") or {}
    positions = joint_state.get("position") or []
    if len(positions) < 4:
        return np.zeros(4, dtype=np.float32)
    return np.asarray(positions[:4], dtype=np.float32)


def _action(step: dict[str, Any], mode: str) -> np.ndarray:
    delta = (step.get("action") or {}).get("task_delta_cmd") or {}
    xyz = delta.get("xyz") or [0.0, 0.0, 0.0]
    if mode == "xy":
        return np.asarray(xyz[:2], dtype=np.float32)
    if mode == "xyz":
        return np.asarray(xyz[:3], dtype=np.float32)
    raise ValueError(f"unsupported action mode: {mode}")


def _load_image_rgb(image_path: Path) -> np.ndarray:
    from PIL import Image

    with Image.open(image_path) as image:
        return np.asarray(image.convert("RGB"), dtype=np.uint8)


def _call_supported(func, **kwargs):
    signature = inspect.signature(func)
    params = signature.parameters
    accepts_kwargs = any(param.kind == inspect.Parameter.VAR_KEYWORD for param in params.values())
    if accepts_kwargs:
        return func(**kwargs)
    filtered = {key: value for key, value in kwargs.items() if key in params}
    return func(**filtered)


def _dataset_output_dir(root: Path, repo_id: str) -> Path:
    return root / Path(repo_id)


def _state_vector(step: dict[str, Any]) -> np.ndarray:
    return _joint_positions(step).astype(np.float32)


def main() -> None:
    parser = argparse.ArgumentParser(description="Convert raw Rotom demos into a local LeRobot dataset.")
    parser.add_argument("--raw-root", default="data/demos", help="Directory containing raw episode folders.")
    parser.add_argument("--repo-id", default="local/rotom_task_teleop", help="Local LeRobot dataset repo id.")
    parser.add_argument("--root", default="data/lerobot", help="Root directory for the local LeRobot dataset.")
    parser.add_argument("--fps", type=int, default=20, help="Dataset frame rate metadata.")
    parser.add_argument("--task", default="table_teleop", help="Fallback task name when episode metadata has no task_id.")
    parser.add_argument("--action-mode", choices=["xy", "xyz"], default="xy", help="Action dimensions to export.")
    parser.add_argument("--success-only", action="store_true", help="Keep only success-labeled episodes.")
    parser.add_argument("--force", action="store_true", help="Overwrite an existing local LeRobot dataset directory.")
    args = parser.parse_args()

    try:
        from lerobot.datasets.lerobot_dataset import LeRobotDataset
    except Exception as exc:
        raise SystemExit(
            "LeRobot is not installed in the active environment. Run `just lerobot-install` on the training machine first."
        ) from exc

    raw_root = Path(args.raw_root).expanduser().resolve()
    output_root = Path(args.root).expanduser().resolve()
    output_root.mkdir(parents=True, exist_ok=True)
    dataset_dir = _dataset_output_dir(output_root, args.repo_id)

    if dataset_dir.exists():
        if not args.force:
            raise SystemExit(
                f"target dataset already exists at {dataset_dir}. Re-run with --force to replace it."
            )
        shutil.rmtree(dataset_dir)

    state_dim = 4
    action_dim = 2 if args.action_mode == "xy" else 3
    features = {
        "observation.images.front": {"dtype": "image"},
        "observation.state": {"dtype": "float32", "shape": (state_dim,)},
        "action": {"dtype": "float32", "shape": (action_dim,)},
    }

    create_kwargs = {
        "repo_id": args.repo_id,
        "root": output_root,
        "fps": args.fps,
        "robot_type": "rotom",
        "features": features,
        "use_videos": False,
    }
    dataset = _call_supported(LeRobotDataset.create, **create_kwargs)

    episode_summaries: list[dict[str, Any]] = []
    frame_count = 0

    for episode_dir in _episode_dirs(raw_root):
        meta = _load_json(episode_dir / "meta.json")
        if not _filter_episode(meta, success_only=args.success_only):
            continue

        steps = _load_steps(episode_dir / "steps.jsonl")
        if not steps:
            continue

        task_name = meta.get("task_id") or args.task
        for step in steps:
            image_path = episode_dir / step["image_path"]
            state = _state_vector(step)
            action = _action(step, args.action_mode)
            frame = {
                "observation.images.front": _load_image_rgb(image_path),
                "observation.state": state,
                "action": action,
            }
            dataset.add_frame(frame)
            frame_count += 1

        save_episode = getattr(dataset, "save_episode")
        try:
            _call_supported(save_episode, task=task_name)
        except TypeError:
            save_episode()

        episode_summaries.append(
            {
                "episode_id": meta.get("episode_id", episode_dir.name),
                "task": task_name,
                "success": meta.get("success"),
                "label": meta.get("label", "unknown"),
                "task_id": meta.get("task_id"),
                "failure_code": meta.get("failure_code"),
                "notes": meta.get("notes", ""),
                "sample_count": len(steps),
                "state_mode": "joints",
                "action_mode": args.action_mode,
            }
        )

    if not episode_summaries:
        raise SystemExit("no episodes matched the requested filters")

    if hasattr(dataset, "finalize"):
        dataset.finalize()
    elif hasattr(dataset, "consolidate"):
        dataset.consolidate()

    dataset_dir.mkdir(parents=True, exist_ok=True)
    (dataset_dir / "episode_metadata.json").write_text(json.dumps(episode_summaries, indent=2) + "\n", encoding="utf-8")

    print(f"wrote LeRobot dataset to {dataset_dir}")
    print(f"episodes: {len(episode_summaries)}")
    print(f"frames: {frame_count}")


if __name__ == "__main__":
    main()
