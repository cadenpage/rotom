#!/usr/bin/env python3
import argparse
from pathlib import Path


def main() -> None:
    parser = argparse.ArgumentParser(description="Download a LeRobot dataset from the Hugging Face Hub into a local directory.")
    parser.add_argument('--repo-id', required=True, help='Hub dataset repo id, e.g. username/rotom_task_teleop')
    parser.add_argument('--root', default='data/lerobot_hub', help='Local root directory for the downloaded dataset.')
    parser.add_argument('--revision', default=None, help='Optional branch or tag to download.')
    parser.add_argument('--force', action='store_true', help='Delete an existing local copy before downloading.')
    args = parser.parse_args()

    from huggingface_hub import snapshot_download
    import shutil

    root = Path(args.root).expanduser().resolve()
    local_dir = root / Path(args.repo_id)
    if local_dir.exists() and args.force:
        shutil.rmtree(local_dir)
    local_dir.parent.mkdir(parents=True, exist_ok=True)

    snapshot_download(
        repo_id=args.repo_id,
        repo_type='dataset',
        revision=args.revision,
        local_dir=local_dir,
    )
    print(f'downloaded dataset to {local_dir}')


if __name__ == '__main__':
    main()
