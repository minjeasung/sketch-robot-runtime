#!/usr/bin/env python3
"""Fetch pinned public dependencies into this checkout, without editing other repos."""
import argparse
import json
from pathlib import Path
import subprocess


def fetch(root, name, spec):
    target = root / ".runtime/sources" / name
    revision = spec["revision"]
    if target.exists():
        head = subprocess.run(["git", "-C", str(target), "rev-parse", "HEAD"], text=True, capture_output=True)
        dirty = subprocess.check_output(["git", "-C", str(target), "status", "--porcelain"], text=True)
        origin = subprocess.check_output(["git", "-C", str(target), "remote", "get-url", "origin"], text=True).strip()
        if dirty or origin != spec["repository"] or (head.returncode == 0 and head.stdout.strip() != revision):
            raise RuntimeError(f"{target}: unexpected revision or local edits; refusing to overwrite")
        if head.returncode == 0:
            return
        # An interrupted first fetch has no HEAD yet; retry without deleting it.
    else:
        target.parent.mkdir(parents=True, exist_ok=True)
        subprocess.run(["git", "init", str(target)], check=True)
        subprocess.run(["git", "-C", str(target), "remote", "add", "origin", spec["repository"]], check=True)
    subprocess.run(["git", "-C", str(target), "fetch", "--depth", "1", "origin", revision], check=True)
    subprocess.run(["git", "-C", str(target), "checkout", "--detach", "FETCH_HEAD"], check=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--without-zed", action="store_true")
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    specs = json.loads((root / "config/runtime_sources.json").read_text())
    for name, spec in specs.items():
        if name == "zed-ros2-wrapper" and args.without_zed:
            continue
        fetch(root, name, spec)


if __name__ == "__main__":
    main()
