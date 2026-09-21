#!/usr/bin/env python3
"""Export current source changes, excluding credentials, builds and machine settings."""
import argparse
from datetime import datetime, timezone
import hashlib
import io
import json
from pathlib import Path
import tarfile

CALIBRATION_FILES = ("zed_d405_apriltag_calibration.json", "d405_eyeinhand_charuco_calibration.json",
                     "zed_eyetohand_charuco_calibration.json", "aft200_force_threshold.json")


def source_files(root):
    for folder in ("src", "web", "scripts", "config", "docs"):
        for path in sorted((root / folder).rglob("*")):
            relative = path.relative_to(root)
            if any(part.startswith(".") or part in ("__pycache__", "build", "install", "log", "logs") for part in relative.parts):
                continue
            if '.bak' in path.name or path.suffix in (".pyc", ".env") or '.local.' in path.name:
                continue
            if path.is_symlink():
                raise ValueError(f"Review symbolic link before export: {relative}")
            if path.is_file():
                yield path
    for name in ("README.md", "RUN.md", ".gitignore"):
        if (root / name).is_file():
            yield root / name


def export(root, output, include_calibration=False):
    files = list(source_files(root))
    if include_calibration:
        files = [root / name for name in CALIBRATION_FILES if (root / name).is_file()]
        if not files:
            raise ValueError("No calibration files found")
    manifest = {"created_at": datetime.now(timezone.utc).isoformat(),
                "kind": "machine-calibration" if include_calibration else "software-source",
                "files": {str(p.relative_to(root)): hashlib.sha256(p.read_bytes()).hexdigest() for p in files}}
    output.parent.mkdir(parents=True, exist_ok=True)
    # Exclusive creation: do not replace an existing release by accident.
    with output.open("xb") as stream:
        with tarfile.open(fileobj=stream, mode="w:gz", compresslevel=3) as archive:
            for path in files:
                archive.add(path, arcname=str(Path("sketch_robot_ws") / path.relative_to(root)), recursive=False)
            data = json.dumps(manifest, ensure_ascii=False, indent=2).encode()
            info = tarfile.TarInfo("sketch_robot_ws/" + ("CALIBRATION_MANIFEST.json" if include_calibration else "BUNDLE_MANIFEST.json"))
            info.size = len(data)
            archive.addfile(info, io.BytesIO(data))
    digest = hashlib.sha256(output.read_bytes()).hexdigest()
    output.with_name(output.name + ".sha256").write_text(f"{digest}  {output.name}\n")
    return len(files)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--calibration-only", action="store_true")
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    kind = "calibration" if args.calibration_only else "source"
    output = args.output or root / "dist" / f"sketch-robot-{kind}-{datetime.now():%Y%m%d-%H%M%S}.tar.gz"
    count = export(root, output, args.calibration_only)
    print(f"{output} ({count} files); SHA256: {output.name}.sha256")


if __name__ == "__main__":
    main()
